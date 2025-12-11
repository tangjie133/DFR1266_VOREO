#include "UserCode.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"
#include "UserConfig.h"
#include "esp_log.h"
#include "esp_mn_speech_commands.h"
#include "dsps_fft2r.h"
#include <string.h>

#include "AudioInitialization.h"

#include "esp_dsp.h"

#include <math.h>
#include <stdlib.h>


#define TAG "USER_CODE"

esp_codec_dev_handle_t record_dev = NULL;
srmodel_list_t *models = NULL;
int wakeup_flag = 0;
static const esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t *afe_data = NULL;

static QueueHandle_t audio_queue = NULL;

// 物理坐标（米），阵列布局：
//   M2 ---- M1   (y = +0.025)
//   |        |
//   M3 ---- M4   (y = -0.025)
//   x 轴向右，y 轴向上，间距 50mm
static const float mic_x[4] = { +0.025f, -0.025f, -0.025f, +0.025f }; // M1, M2, M3, M4
static const float mic_y[4] = { +0.025f, +0.025f, -0.025f, -0.025f };
static const float mic_z[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

/* ----------------- Buffers ----------------- */
/* raw I2S buffer: assuming 32-bit words and interleaved samples [ch0,ch1,ch2,ch3,...] */
static int32_t raw_i2s_buf[FRAME_SIZE * CHANNELS]; // <-- ensure size fits memory

/* per-channel float circular buffer (we'll just fill frame-by-frame in this example) */
static float ch_frame[CHANNELS][FRAME_SIZE];

/* FFT buffers (interleaved complex: re,im pairs) */
static float fft_A[FRAME_SIZE * 2];
static float fft_B[FRAME_SIZE * 2];
static float crossR[FRAME_SIZE * 2];

/* window */
static float hann_win[FRAME_SIZE];

// slot -> mic mapping (根据实际 TDM 槽位对应物理麦序)
// 若 ES7210 槽位与物理顺序一致，用 {0,1,2,3}
// 如左右或前后颠倒，可尝试 {1,0,2,3} 或 {0,2,1,3}
static int slot2mic[CHANNELS] = {0,2,1,3};

void initMIC(void)
{
    int ret = initAudio(&record_dev);
    if (ret != 0) {
        return;
    }

   

#ifdef ES7243E
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 2,
        .bits_per_sample = 32,
    };
#endif
#ifdef ES7210
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 4,
        .bits_per_sample = 32,
    };
#endif
    ret = esp_codec_dev_open(record_dev, &fs);

    if (ret != 0) {
        return;
    }

    // ret = esp_codec_dev_set_in_gain(record_dev, 30.0);
    // if (ret != 0) {
    //     return;
    // }
    // esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), 30.0);
    // esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1), 30.0);
    // esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2), 30.0);
    // esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(3), 30.0);

}


void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    ESP_LOGI(TAG, "Feed channel number: %d", audio_chunksize);
    ESP_LOGI(TAG, "Feed channel number: %d", nch);
#ifdef ES7243E
    int feed_channel = 2;
#endif
#ifdef ES7210
    int feed_channel = 4;
#endif
    assert(nch == feed_channel);
    // Buffer for reading 32-bit data from codec
    int32_t *i2s_buff_32 = malloc(audio_chunksize * sizeof(int32_t) * feed_channel);
    assert(i2s_buff_32);
    // Buffer for 16-bit data to feed AFE
    int16_t *i2s_buff_16 = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff_16);
    
    // Calculate actual data size needed
    size_t audio_data_size = audio_chunksize * sizeof(int32_t) * feed_channel;

    while (1) {
        // Read 32-bit audio data from codec
        esp_codec_dev_read(record_dev, i2s_buff_32, audio_data_size);
        // Allocate buffer for queue (receiver will free it)
        int32_t *audio_data = malloc(audio_data_size);
        if (!audio_data) {
            ESP_LOGE(TAG, "Failed to malloc audio data");
            continue;
        }

        memcpy(audio_data, i2s_buff_32, audio_data_size);

        // Send pointer to queue (receiver will free the memory); timeout 1 tick to avoid blocking
        if(xQueueSend(audio_queue, &audio_data, 1) != pdPASS) {
            ESP_LOGE(TAG, "Failed to send audio data to queue");
            free(audio_data);
            continue;
        }
        // Convert 32-bit to 16-bit: extract valid bits (31:8) and shift right by 14 bits
        // This extracts bits 31:14 which contains the valid audio data
        // Data format: bits 31:8 are valid, bits 7:0 are zero
        int total_samples = audio_chunksize * feed_channel;
        for (int i = 0; i < total_samples; i++) {
            // Extract valid data bits and convert to 16-bit
            // Right shift 14 bits to get 16-bit signed value (bits 31:14 -> 15:0)
            i2s_buff_16[i] = (int16_t)(i2s_buff_32[i] >> 14);
        }
        
        // Feed 16-bit data to AFE (AFE requires int16_t*)
        //afe_handle->feed(afe_data, i2s_buff_16);
        vTaskDelay(1);
    }
    if (i2s_buff_32) {
        free(i2s_buff_32);
        i2s_buff_32 = NULL;
    }
    if (i2s_buff_16) {
        free(i2s_buff_16);
        i2s_buff_16 = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    // #if ESP_MN_CH_MODEL
    // char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    // #endif
    // #if ESP_MN_EN_MODEL
    // char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    // #endif
    // esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    // model_iface_data_t *model_data = multinet->create(mn_name, 6000);
    // multinet->set_det_threshold(model_data, 0.1);
    // esp_mn_commands_alloc(multinet, model_data);
    // esp_mn_commands_clear();
    // // esp_mn_commands_add(1, "ni hao xiao zhi");
    // // esp_mn_commands_add(1, "xiao zhi xiao hao");
    // esp_mn_commands_add(1, "xiao ai tong xue");
    // esp_mn_commands_update();
    // esp_mn_active_commands_print();
    printf("------------detect start------------\n");
    uint8_t wakeup_flag = 0;
    while (1) {
        // 频繁调用 fetch() 以避免 AFE 环形缓冲区溢出
        afe_fetch_result_t* res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("wakeword detected\n");
	        printf("model index:%d, word index:%d\n", res->wakenet_model_index, res->wake_word_index);
            printf("-----------LISTENING-----------\n");
        }

        // if (wakeup_flag) {
        //     esp_mn_state_t mn_state = multinet->detect(model_data, res->data);
        //     if (mn_state == ESP_MN_STATE_DETECTING) {
        //         //vTaskDelay(1);  // 添加延迟，避免CPU占用过高导致看门狗超时
        //         //break;
        //     }
        //     if (mn_state == ESP_MN_STATE_DETECTED) {
        //         esp_mn_results_t *mn_result = multinet->get_results(model_data);
        //         for (int i = 0; i < mn_result->num; i++) {
        //             printf("TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f\n", 
        //             i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
        //         }
        //         printf("\n-----------listening-----------\n");
        //         //wakeup_flag = 0;
        //     }
        //     // if (mn_state == ESP_MN_STATE_TIMEOUT) {
        //     //     esp_mn_results_t *mn_result = multinet->get_results(model_data);
        //     //     printf("timeout, string:%s\n", mn_result->string);
        //     //     //afe_handle->enable_wakenet(afe_data);
        //     //     wakeup_flag = 0;
        //     //     printf("\n-----------awaits to be waken up-----------\n");
        //     //     continue;
        //     //}
        // }
        
        // 最小延迟，只用于让出 CPU，保持高频率的 fetch 调用
        vTaskDelay(1);
    }
    printf("detect exit\n");
    vTaskDelete(NULL);
}

void initSpeechRecog(void)
{
    models = esp_srmodel_init("model");
#ifdef ES7243E 
    afe_config_t *afe_config = afe_config_init("MM", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
#endif
#ifdef ES7210
    afe_config_t *afe_config = afe_config_init("MMNR", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
#endif 
    // if (afe_config) {
    //     // 禁用WakeNet，仅使用MultiNet
    //     afe_config->wakenet_init = false;
    //     afe_config->aec_init = false;  // 如果不需要回声消除
    // }
    afe_handle = esp_afe_handle_from_config(afe_config);
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);

    xTaskCreatePinnedToCore(detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
}


void autodetect_slot_mapping(void)
{
    ESP_LOGI(TAG, "Auto-detect slot->mic mapping. Please clap near each microphone in order (mic0 -> mic1 -> mic2 -> mic3).");
    vTaskDelay(pdMS_TO_TICKS(1000));
    for (int target_mic = 0; target_mic < CHANNELS; ++target_mic) {
        ESP_LOGI(TAG, "Prepare to clap near physical mic %d in 3s...", target_mic);
        vTaskDelay(pdMS_TO_TICKS(3000));
        // collect several frames and compute per-slot energy over 0.5s
        int num_frames = 8;
        float slot_energy[CHANNELS];
        for (int s=0;s<CHANNELS;s++) slot_energy[s]=0.0f;
        for (int f=0; f<num_frames; ++f) {
            size_t bytes_read = 0;
            size_t to_read = sizeof(int32_t) * FRAME_SIZE * CHANNELS;
            int32_t *raw_i2s_buf = malloc(to_read);
            assert(raw_i2s_buf);
            esp_codec_dev_read(record_dev, raw_i2s_buf, to_read);
            // if (r != ESP_OK) {
            //     ESP_LOGW(TAG, "i2s_read during autodetect failed %d", r);
            //     continue;
            // }
            // compute slot energy directly from raw slots (int32)
            for (int i=0;i<FRAME_SIZE;i++){
                int idx = i*CHANNELS;
                for (int s=0;s<CHANNELS;s++){
                    float v = (float)raw_i2s_buf[idx + s] * (1.0f/2147483648.0f);
                    slot_energy[s] += v*v;
                }
            }
        }
        // find slot with maximum energy -> assign to target_mic
        int best_slot = 0;
        float best_val = -1.0f;
        for (int s=0;s<CHANNELS;s++){
            if (slot_energy[s] > best_val) { best_val = slot_energy[s]; best_slot = s; }
        }
        // mark mapping: this slot corresponds to physical mic target_mic
        // We need slot2mic[slot] = micindex ; currently slot2mic maps slot->mic ; we want update such that mic index correct.
        // We'll create reverse mapping: new_slot2mic[slot] = assigned mic index
        // Build temporary new mapping only after all detected to avoid conflicts (here we set directly but ensure uniqueness)
        // For simplicity we'll overwrite slot2mic so that slot best_slot maps to target_mic
        // If conflicts occur, user can re-run
        slot2mic[best_slot] = target_mic;
        ESP_LOGI(TAG, "Detected slot %d -> mic %d (energy %.2f)", best_slot, target_mic, best_val);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "Auto-detect finished. slot2mic mapping:");
    for (int i=0;i<CHANNELS;i++) ESP_LOGI(TAG, " slot %d -> mic %d", i, slot2mic[i]);
}

/* ----------------- Helper funcs ----------------- */
static void init_hann_window(float *w, int n) {
    for (int i=0;i<n;i++) w[i] = 0.5f * (1.0f - cosf(2.0f*M_PI*i/(n-1)));
}

static float frame_energy(float *buf, int n) {
    double e = 0;
    for (int i=0;i<n;i++) e += buf[i]*buf[i];
    return (float)(e/n);
}

// clamp helper
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// forward declare
static void compute_taus_relative(float ch_frame_in[CHANNELS][FRAME_SIZE], int ref_idx, float taus[CHANNELS]);

/* time-domain GCC with limited lag window, returns tau (seconds) */
static float tdoa_td_gcc(const float *a, const float *b, int n, int maxLag, int fs) {
    float best = -1e30f;
    int bestLag = 0;
    float energy_a = 0, energy_b = 0;
    for (int i=0;i<n;i++) {
        energy_a += a[i]*a[i];
        energy_b += b[i]*b[i];
    }
    float norm = sqrtf(energy_a*energy_b) + 1e-9f;
    for (int lag = -maxLag; lag <= maxLag; lag++) {
        float acc = 0.0f;
        int start = (lag > 0) ? lag : 0;
        int end   = (lag > 0) ? n : n + lag;
        for (int i=start; i<end; i++) {
            acc += a[i] * b[i - lag];
        }
        float val = acc / norm;
        if (val > best) { best = val; bestLag = lag; }
    }
    return (float)bestLag / (float)fs;
}

/* Deinterleave int32 raw buffer -> float per-channel frame (assumes slot order) */
static void deinterleave_int32_to_float(int32_t *raw, int frames, float out[CHANNELS][FRAME_SIZE]) {
    const float scale = 1.0f / 2147483648.0f;
    for (int i=0;i<frames;i++) {
        int base = i * CHANNELS;
        for (int s=0;s<CHANNELS;s++) {
            int mic_idx = slot2mic[s]; // which mic this slot corresponds to
            out[mic_idx][i] = raw[base + s] * scale;
        }
    }
}

/* compute FFT of real time domain frame -> complex array (interleaved re,im) */
static void compute_fft_real_frame(const float *time, float *complex_out, int n) {
    for (int i=0;i<n;i++) {
        complex_out[2*i]   = time[i];
        complex_out[2*i+1] = 0.0f;
    }
    dsps_fft2r_fc32(complex_out, n);
}

/* compute GCC-PHAT between complex spectra X and Y, return tau (seconds)
   uses global buffer crossR for IFFT result.
   Added debug prints for peak index/value, and constrain search to physical lag window. */
static float gcc_phat_from_spectra(const float *X, const float *Y, int n, int fs) {
    for (int k=0;k<n;k++) {
        float xr = X[2*k], xi = X[2*k+1];
        float yr = Y[2*k], yi = Y[2*k+1];
        // R = X * conj(Y)
        float re = xr*yr + xi*yi;
        float im = xi*yr - xr*yi;
        float mag = sqrtf(re*re + im*im) + 1e-9f;
        crossR[2*k]   = re / mag;
        crossR[2*k+1] = im / mag;
    }
    // ifft: use forward FFT + bit reverse, then normalize by N
    dsps_fft2r_fc32(crossR, n);
    dsps_bit_rev_fc32(crossR, n);
    float inv_n = 1.0f / (float)n;
    for (int i = 0; i < 2 * n; i++) crossR[i] *= inv_n;

    // Physical max lag (samples) based on baseline 5cm
    float max_tau = 0.05f / SPEED_OF_SOUND;
    int maxLag = (int)ceilf(max_tau * (float)fs);
    if (maxLag < 1) maxLag = 1;

    // find peak only within [-maxLag, +maxLag]
    int best_idx = 0;
    float best_val = -1e30f;
    // search positive lags [0, maxLag]
    int upper = (maxLag < n) ? maxLag : n-1;
    for (int i=0; i<=upper; i++) {
        float val = crossR[2*i];
        if (val > best_val) { best_val = val; best_idx = i; }
    }
    // search negative lags: indices [n - maxLag, n-1]
    int lower = (n - maxLag >= 0) ? n - maxLag : 0;
    for (int i=lower; i<n; i++) {
        float val = crossR[2*i];
        if (val > best_val) { best_val = val; best_idx = i; }
    }
    int lag = best_idx;
    if (lag > n/2) lag -= n;
    // parabolic interpolation
    int im1 = (best_idx - 1 + n) % n;
    int ip1 = (best_idx + 1) % n;
    float ym = crossR[2*im1];
    float y0 = crossR[2*best_idx];
    float yp = crossR[2*ip1];
    float denom = (ym - 2*y0 + yp);
    float delta = 0.0f;
    if (fabsf(denom) > 1e-12f) delta = 0.5f * (ym - yp) / denom;
    float tau_samples = (float)lag + delta;
    float tau_sec = tau_samples / (float)fs;
    // clamp tau to physical window (extra guard)
    float maxLagSamplesF = (float)maxLag;
    if (tau_samples > maxLagSamplesF) tau_samples = maxLagSamplesF;
    if (tau_samples < -maxLagSamplesF) tau_samples = -maxLagSamplesF;
    tau_sec = tau_samples / (float)fs;

    ESP_LOGI(TAG, "GCC debug: peak_idx=%d best_val=%.4f lag=%.2f samples (maxLag=%d)", best_idx, best_val, tau_samples, maxLag);
    return tau_sec;
}

/* compute all pairwise taus relative to reference mic ref_idx using GCC-PHAT */
static void compute_taus_relative(float ch_frame_in[CHANNELS][FRAME_SIZE], int ref_idx, float taus[CHANNELS]) {
    // compute FFT for each channel into local arrays
    static float spec[CHANNELS][FRAME_SIZE*2];
    for (int ch=0; ch<CHANNELS; ch++) {
        compute_fft_real_frame(ch_frame_in[ch], spec[ch], FRAME_SIZE);
    }
    for (int i=0;i<CHANNELS;i++) {
        if (i == ref_idx) { taus[i] = 0.0f; continue; }
        taus[i] = gcc_phat_from_spectra(spec[ref_idx], spec[i], FRAME_SIZE, SAMPLE_RATE);
    }
}

/* Gauss-Newton to solve for 3D position (x,y,z) given taus (relative to ref 0).
   Uses mic_x/mic_y/mic_z arrays. di = taus[i] * c = r_i - r_ref.
   Solve nonlinear least squares on function f_i(x) = (||p - mic_i|| - ||p - mic_ref||) - di[i]
   Solve equations for i=1..3 (3 unknowns).
*/
static int estimate_position_gn(const float taus[CHANNELS], float *out_x, float *out_y, float *out_z) {
    // convert to di
    float di[CHANNELS];
    for (int i=0;i<CHANNELS;i++) di[i] = taus[i] * SPEED_OF_SOUND;
    // initial guess: use 2D solve to get x,y, z~0.15m
    // simple 2D linearized solve (as earlier) for initial x,y
    float Ainit[3][3], binit[3];
    int eq=0;
    for (int i=1;i<CHANNELS;i++) {
        Ainit[eq][0] = mic_x[i] - mic_x[0];
        Ainit[eq][1] = mic_y[i] - mic_y[0];
        Ainit[eq][2] = di[i];
        binit[eq] = 0.5f * ( (mic_x[i]*mic_x[i] + mic_y[i]*mic_y[i]) - (mic_x[0]*mic_x[0] + mic_y[0]*mic_y[0]) - di[i]*di[i] );
        eq++;
    }
    // solve 3x3 linear (gauss) -> initial x,y,r0
    float M[3][4];
    for (int i=0;i<3;i++){ for (int j=0;j<3;j++) M[i][j]=Ainit[i][j]; M[i][3]=binit[i]; }
    for (int i=0;i<3;i++){
        int piv=i; for (int r=i+1;r<3;r++) if (fabsf(M[r][i])>fabsf(M[piv][i])) piv=r;
        if (fabsf(M[piv][i])<1e-12f) return -1;
        if (piv!=i) for (int c=i;c<4;c++){ float t=M[i][c]; M[i][c]=M[piv][c]; M[piv][c]=t; }
        float diag = M[i][i]; for (int c=i;c<4;c++) M[i][c]/=diag;
        for (int r=i+1;r<3;r++){ float f=M[r][i]; for (int c=i;c<4;c++) M[r][c]-=f*M[i][c]; }
    }
    float sol[3];
    for (int i=2;i>=0;i--){ float v=M[i][3]; for (int c=i+1;c<3;c++) v -= M[i][c]*sol[c]; sol[i]=v/M[i][i]; }
    float x = sol[0], y = sol[1], r0 = sol[2];
    float z = 0.15f; // initial z guess 15 cm

    // GN iterations
    for (int iter=0; iter<12; iter++) {
        // build Jacobian (3x3) and residuals (3)
        float J[3][3]; memset(J,0,sizeof(J));
        float res[3]; memset(res,0,sizeof(res));
        int e=0;
        float r_ref = sqrtf((x-mic_x[0])*(x-mic_x[0]) + (y-mic_y[0])*(y-mic_y[0]) + (z-mic_z[0])*(z-mic_z[0]) ) + 1e-9f;
        for (int i=1;i<CHANNELS;i++){
            float ri = sqrtf((x-mic_x[i])*(x-mic_x[i]) + (y-mic_y[i])*(y-mic_y[i]) + (z-mic_z[i])*(z-mic_z[i]) ) + 1e-9f;
            float fi = (ri - r_ref) - di[i]; // residual
            // partials
            float dfdx = (x - mic_x[i]) / ri - (x - mic_x[0]) / r_ref;
            float dfdy = (y - mic_y[i]) / ri - (y - mic_y[0]) / r_ref;
            float dfdz = (z - mic_z[i]) / ri - (z - mic_z[0]) / r_ref;
            J[e][0] = dfdx; J[e][1] = dfdy; J[e][2] = dfdz;
            res[e] = -fi; // we want J * delta = -f
            e++;
        }
        // Normal equations JTJ * delta = JTr
        float JTJ[3][3] = {{0}}, JTr[3] = {0};
        for (int i=0;i<3;i++) for (int j=0;j<3;j++){
            float s=0; for (int k=0;k<3;k++) s += J[k][i]*J[k][j];
            JTJ[i][j] = s;
            float t=0; for (int k=0;k<3;k++) t += J[k][i]*res[k];
            JTr[i] = t;
        }
        // solve 3x3
        float A[3][4];
        for (int i=0;i<3;i++){ for (int j=0;j<3;j++) A[i][j]=JTJ[i][j]; A[i][3]=JTr[i]; }
        // gaussian
        int singular=0;
        for (int i=0;i<3;i++){
            int piv=i; for (int r=i+1;r<3;r++) if (fabsf(A[r][i])>fabsf(A[piv][i])) piv=r;
            if (fabsf(A[piv][i])<1e-12f) { singular=1; break; }
            if (piv!=i) for (int c=i;c<4;c++){ float t=A[i][c]; A[i][c]=A[piv][c]; A[piv][c]=t; }
            float diag=A[i][i]; for (int c=i;c<4;c++) A[i][c]/=diag;
            for (int r=i+1;r<3;r++){ float f=A[r][i]; for (int c=i;c<4;c++) A[r][c]-=f*A[i][c]; }
        }
        if (singular) return -1;
        float delta[3];
        for (int i=2;i>=0;i--){ float v=A[i][3]; for (int c=i+1;c<3;c++) v-=A[i][c]*delta[c]; delta[i]=v/A[i][i]; }
        x += delta[0]; y += delta[1]; z += delta[2];
        if (sqrtf(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]) < 1e-6f) break;
    }
    *out_x = x; *out_y = y; *out_z = z;
    return 0;
}

/* convert (x,y,z) to azimuth & elevation in degrees
   azimuth: 0..360 deg, 0 = +x axis, measured CCW (y positive is 90deg)
   elevation: -90..+90 deg (0 = horizontal plane positive upward)
*/
static void xyz_to_az_el(float x, float y, float z, float *az_deg, float *el_deg) {
    float az = atan2f(y, x) * 180.0f / M_PI;
    if (az < 0) az += 360.0f;
    float r = sqrtf(x*x + y*y + z*z) + 1e-9f;
    float el = asinf(z / r) * 180.0f / M_PI;
    *az_deg = az; *el_deg = el;
}

/* ----------------- Kalman filter for azimuth & elevation (independent) ----------------- */
typedef struct {
    float x[2]; // angle deg, rate deg/s
    float P[2][2];
    float Q[2][2];
    float R;
    float dt;
} kalman_ang_t;

static void kalman_init(kalman_ang_t *kf, float dt) {
    kf->dt = dt;
    kf->x[0]=0; kf->x[1]=0;
    kf->P[0][0]=1; kf->P[0][1]=0; kf->P[1][0]=0; kf->P[1][1]=1;
    kf->Q[0][0]=KALMAN_Q_ANGLE; kf->Q[0][1]=0; kf->Q[1][0]=0; kf->Q[1][1]=KALMAN_Q_RATE;
    kf->R = KALMAN_R_MEAS;
}

static float wrap_diff_deg(float a, float b) { // returns a-b wrapped to [-180,180]
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

static void kalman_update_angle(kalman_ang_t *kf, float meas_deg) {
    // predict
    float A00=1, A01=kf->dt, A10=0, A11=1;
    float x0 = A00*kf->x[0] + A01*kf->x[1];
    float x1 = A10*kf->x[0] + A11*kf->x[1];
    // P = A P A^T + Q (compute manually)
    float P00 = A00*kf->P[0][0]*A00 + A01*kf->P[1][0]*A00 + kf->Q[0][0];
    float P01 = A00*kf->P[0][1]*A01 + A01*kf->P[1][1]*A01 + kf->Q[0][1];
    float P10 = A10*kf->P[0][0]*A00 + A11*kf->P[1][0]*A00 + kf->Q[1][0];
    float P11 = A10*kf->P[0][1]*A01 + A11*kf->P[1][1]*A01 + kf->Q[1][1];
    kf->x[0]=x0; kf->x[1]=x1;
    kf->P[0][0]=P00; kf->P[0][1]=P01; kf->P[1][0]=P10; kf->P[1][1]=P11;
    // update using angle measurement (wrap handling)
    float pred = kf->x[0];
    float diff = wrap_diff_deg(meas_deg, pred);
    float S = kf->P[0][0] + kf->R;
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;
    kf->x[0] = kf->x[0] + K0 * diff;
    kf->x[1] = kf->x[1] + K1 * diff;
    // update P
    float I_KH00 = 1 - K0;
    float I_KH01 = 0;
    float I_KH10 = -K1;
    float I_KH11 = 1;
    float nP00 = I_KH00*kf->P[0][0] + I_KH01*kf->P[1][0];
    float nP01 = I_KH00*kf->P[0][1] + I_KH01*kf->P[1][1];
    float nP10 = I_KH10*kf->P[0][0] + I_KH11*kf->P[1][0];
    float nP11 = I_KH10*kf->P[0][1] + I_KH11*kf->P[1][1];
    kf->P[0][0]=nP00; kf->P[0][1]=nP01; kf->P[1][0]=nP10; kf->P[1][1]=nP11;
    // wrap angle to [0,360)
    while (kf->x[0] < 0) kf->x[0]+=360.0f;
    while (kf->x[0] >=360.0f) kf->x[0]-=360.0f;
}

/* ----------------- Processing pipeline ----------------- 
   Note: For simplicity we process frames without overlap here. For production use, maintain ring buffer and process with hop HOP_SIZE.
*/
static void process_one_frame() {
    // 1) deinterleave raw int32 -> ch_frame (float) according to slot2mic
    deinterleave_int32_to_float(raw_i2s_buf, FRAME_SIZE, ch_frame);

    // 2) compute per-channel energy and VAD
    float total_e = 0.0f;
    for (int ch=0; ch<CHANNELS; ch++) total_e += frame_energy(ch_frame[ch], FRAME_SIZE);
    if (total_e < VAD_ENERGY_TH) {
        // skip if silence
        return;
    }

    // 3) apply hann window in-place (or copy before FFT)
    for (int ch=0; ch<CHANNELS; ch++) {
        for (int i=0;i<FRAME_SIZE;i++) ch_frame[ch][i] *= hann_win[i];
    }

    // 4) 计算参考麦(0)相对的 TDOA（时间域 GCC，限最大延时 = 物理上限）
    const float baseline = 0.05f; // 5 cm
    const float max_tau = baseline / SPEED_OF_SOUND; // 约 0.00015s @16kHz ≈ 2.4 samples
    int maxLagSamples = (int)ceilf(max_tau * (float)SAMPLE_RATE);
    if (maxLagSamples < 1) maxLagSamples = 1;
    float tau_x = tdoa_td_gcc(ch_frame[0], ch_frame[1], FRAME_SIZE, maxLagSamples, SAMPLE_RATE); // M1 vs M2 (上排左右)
    float tau_y = tdoa_td_gcc(ch_frame[0], ch_frame[3], FRAME_SIZE, maxLagSamples, SAMPLE_RATE); // M1 vs M4 (右上 vs 右下)

    // 5) 方位角（平面波假设）+ 简单低通平滑减少抖动
    static int tau_init = 0;
    static float tau_x_f = 0.0f, tau_y_f = 0.0f;
    const float alpha = 0.2f; // 平滑系数
    if (!tau_init) { tau_x_f = tau_x; tau_y_f = tau_y; tau_init = 1; }
    // 物理范围截断
    if (tau_x >  max_tau) tau_x = max_tau;
    if (tau_x < -max_tau) tau_x = -max_tau;
    if (tau_y >  max_tau) tau_y = max_tau;
    if (tau_y < -max_tau) tau_y = -max_tau;
    // 一阶低通
    tau_x_f = tau_x_f * (1.0f - alpha) + alpha * tau_x;
    tau_y_f = tau_y_f * (1.0f - alpha) + alpha * tau_y;

    float norm_x = tau_x_f / max_tau;
    float norm_y = tau_y_f / max_tau;
    float az = atan2f(norm_y, norm_x) * 180.0f / M_PI; // 0度指向 +x
    if (az < 0) az += 360.0f;

    ESP_LOGI(TAG, "DOA az=%.1f deg  tau_x=%.2fus tau_y=%.2fus  energy=%.3e",
             az, tau_x*1e6f, tau_y*1e6f, total_e);
}

void audio_doa_task(void *arg)
{
    size_t bytes_to_read = sizeof(int32_t) * FRAME_SIZE * CHANNELS;
    while (1) {
        
        // int32_t *audio_data = NULL;
        // // Receive pointer from queue
        // if(xQueueReceive(audio_queue, &audio_data, portMAX_DELAY) != pdPASS) {
        //     ESP_LOGE(TAG, "Failed to receive audio data from queue");
        //     continue;
        // }
        
        // if (!audio_data) {
        //     ESP_LOGE(TAG, "Received NULL audio data pointer");
        //     continue;
        // }
        
        // // TODO: Process audio_data for DOA calculation
        // // float *doa_data = malloc(...);
        // // ... DOA processing ...
        
        // // Free the memory allocated by feed_Task
        // free(audio_data);
        // audio_data = NULL;


        esp_codec_dev_read(record_dev, raw_i2s_buf, bytes_to_read);
        process_one_frame();

        vTaskDelay(100);
    }
    vTaskDelete(NULL);
}


void initAudioDoa(void)
{
    init_hann_window(hann_win, FRAME_SIZE);
    dsps_fft2r_init_fc32(NULL, FRAME_SIZE);
    // Create queue for storing pointers (int32_t*), not data itself
    // This is more efficient for large data blocks
    // audio_queue = xQueueCreate(10, sizeof(int32_t*));
    // if (!audio_queue) {
    //     ESP_LOGE(TAG, "Failed to create audio queue");
    //     return;
    // }
    xTaskCreatePinnedToCore(audio_doa_task, "audio_doa", 8 * 1024, NULL, 5, NULL, 0);
}



