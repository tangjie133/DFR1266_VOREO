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

#include "AudioInitialization.h"

#define TAG "USER_CODE"

esp_codec_dev_handle_t record_dev = NULL;
srmodel_list_t *models = NULL;
int wakeup_flag = 0;
static const esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t *afe_data = NULL;


void initMIC(void)
{
    int ret = initAudio(&record_dev);
    if (ret != 0) {
        return;
    }

    ret = esp_codec_dev_set_in_gain(record_dev, 30.0);
    if (ret != 0) {
        return;
    }

#ifdef ES7243E
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 2,
        .bits_per_sample = 16,
    };
#elif ES7210
    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 4,
        .bits_per_sample = 16,
    };
#endif
    ret = esp_codec_dev_open(record_dev, &fs);

    if (ret != 0) {
        return;
    }

}


void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_feed_channel_num(afe_data);
    ESP_LOGI(TAG, "Feed channel number: %d", nch);
#ifdef ES7243E
    int feed_channel = 2;
#elif ES7210
    int feed_channel = 4;
#endif
    assert(nch == feed_channel);
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (1) {
        esp_codec_dev_read(record_dev, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        afe_handle->feed(afe_data, i2s_buff);
        vTaskDelay(1);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    //afe_handle->set_wakenet_threshold(afe_data, 1, 0.6); // set model1's threshold to 0.6
    //afe_handle->reset_wakenet_threshold(afe_data, 1);    // reset model1's threshold to default
    printf("------------detect start------------\n");
    while (1) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }
        // printf("vad state: %d\n", res->vad_state);

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("wakeword detected\n");
	        printf("model index:%d, word index:%d\n", res->wakenet_model_index, res->wake_word_index);
            printf("-----------LISTENING-----------\n");
        }
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
#elif ES7210
afe_config_t *afe_config = afe_config_init("MMMM", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
#endif 
    afe_handle = esp_afe_handle_from_config(afe_config);
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);

    xTaskCreatePinnedToCore(detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
}