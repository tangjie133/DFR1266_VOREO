#include "string.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#include "esp_private/rtc_clk.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es7243e_adc.h"

#include "esp_log.h"
#include "AudioInitialization.h"

#define TAG "AUDIO_INIT"

#include "UserConfig.h"


#define I2S_MAX_KEEP SOC_I2S_NUM


#ifdef ES7243E
#define ES7243E_ADDR     (0x22)
#define I2C_SDA_PIN      (47)
#define I2C_SCL_PIN      (48)

#define I2S_BCK_PIN      (0)
#define I2S_MCK_PIN      (3)
#define I2S_DATA_IN_PIN  (39)
#define I2S_DATA_OUT_PIN (-1)
#define I2S_DATA_WS_PIN  (38)

static i2s_comm_mode_t i2s_in_mode = I2S_COMM_MODE_STD;
static i2s_comm_mode_t i2s_out_mode = I2S_COMM_MODE_STD;
#elif ES7210

#define ES7210_ADDR     (0x22)
#define I2C_SDA_PIN      (10)
#define I2C_SCL_PIN      (11)

#define I2S_BCK_PIN      (6)
#define I2S_MCK_PIN      (5)
#define I2S_DATA_IN_PIN  (8)
#define I2S_DATA_OUT_PIN (-1)
#define I2S_DATA_WS_PIN  (7)
static i2s_comm_mode_t i2s_in_mode = I2S_COMM_MODE_TDM;
static i2s_comm_mode_t i2s_out_mode = I2S_COMM_MODE_STD;

#endif

typedef struct {
    int16_t  scl;
    int16_t  sda;
} codec_i2c_pin_t;

typedef struct {
    int16_t  mclk;
    int16_t  bclk;
    int16_t  ws;
    int16_t  dout;
    int16_t  din;
} codec_i2s_pin_t;

typedef struct {
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;
} i2s_keep_t;

static i2c_master_bus_handle_t i2c_bus_handle;
static i2s_keep_t *i2s_keep[I2S_MAX_KEEP];

static int i2cInit(uint8_t port, codec_i2c_pin_t *i2c_pin)
{
    i2c_master_bus_config_t i2c_bus_config = {0};
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port = port;
    i2c_bus_config.scl_io_num = i2c_pin ? i2c_pin->scl : I2C_SCL_PIN;
    i2c_bus_config.sda_io_num = i2c_pin ? i2c_pin->sda : I2C_SDA_PIN;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
}

static int i2cDeinit(uint8_t port)
{
    if (i2c_bus_handle) {
        i2c_del_master_bus(i2c_bus_handle);
    }
    return 0;
}

static int i2sInit(uint8_t port, codec_i2s_pin_t *i2s_pin, i2s_clock_src_t clk_src)
{
    if (port >= I2S_MAX_KEEP) {
        return -1;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(32, I2S_SLOT_MODE_STEREO),
        .gpio_cfg ={
            .mclk = i2s_pin ? i2s_pin->mclk : I2S_MCK_PIN,
            .bclk = i2s_pin ? i2s_pin->bclk : I2S_BCK_PIN,
            .ws = i2s_pin ? i2s_pin->ws : I2S_DATA_WS_PIN,
            .dout = i2s_pin ? i2s_pin->dout : I2S_DATA_OUT_PIN,
            .din = i2s_pin ? i2s_pin->din : I2S_DATA_IN_PIN,
        },
    };
    std_cfg.clk_cfg.clk_src = clk_src;
    if (i2s_keep[port] == NULL) {
        i2s_keep[port] = (i2s_keep_t *) calloc(1, sizeof(i2s_keep_t));
        if (i2s_keep[port] == NULL) {
            return -1;
        }
    }
#if SOC_I2S_SUPPORTS_TDM
    i2s_tdm_slot_mask_t slot_mask = I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3;
    i2s_tdm_config_t tdm_cfg = {
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(32, I2S_SLOT_MODE_STEREO, slot_mask),
        .clk_cfg  = I2S_TDM_CLK_DEFAULT_CONFIG(16000),
        .gpio_cfg = {
            .mclk = i2s_pin ? i2s_pin->mclk : I2S_MCK_PIN,
            .bclk = i2s_pin ? i2s_pin->bclk : I2S_BCK_PIN,
            .ws = i2s_pin ? i2s_pin->ws : I2S_DATA_WS_PIN,
            .dout = i2s_pin ? i2s_pin->dout : I2S_DATA_OUT_PIN,
            .din = i2s_pin ? i2s_pin->din : I2S_DATA_IN_PIN,
        },
    };
    tdm_cfg.slot_cfg.total_slot = 4;
    tdm_cfg.clk_cfg.clk_src = clk_src;
#endif

    int ret = i2s_new_channel(&chan_cfg,
        i2s_out_mode == I2S_COMM_MODE_NONE ? NULL :&i2s_keep[port]->tx_handle,
        i2s_in_mode == I2S_COMM_MODE_NONE ? NULL : &i2s_keep[port]->rx_handle);
        ESP_ERROR_CHECK(ret);
    
    if (i2s_out_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->tx_handle, &std_cfg);
    }
#if SOC_I2S_SUPPORTS_TDM
    else if (i2s_out_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->tx_handle, &tdm_cfg);
    }
#endif
    ESP_ERROR_CHECK(ret);
    if (i2s_in_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->rx_handle, &std_cfg);
    }
#if SOC_I2S_SUPPORTS_TDM
    else if (i2s_in_mode == I2S_COMM_MODE_TDM) {
        ret = i2s_channel_init_tdm_mode(i2s_keep[port]->rx_handle, &tdm_cfg);
    }
#endif
    ESP_ERROR_CHECK(ret);
    // For tx master using duplex mode
    i2s_channel_enable(i2s_keep[port]->tx_handle);

    return ret;
}

static int i2sDeinit(uint8_t port)
{
    if (port >= I2S_MAX_KEEP) {
        return -1;
    }
    // already installed
    if (i2s_keep[port] == NULL) {
        return 0;
    }
    i2s_channel_disable(i2s_keep[port]->tx_handle);
    i2s_channel_disable(i2s_keep[port]->rx_handle);
    i2s_del_channel(i2s_keep[port]->tx_handle);
    i2s_del_channel(i2s_keep[port]->rx_handle);
    free(i2s_keep[port]);
    i2s_keep[port] = NULL;
    return 0;
}

int initAudio(esp_codec_dev_handle_t *record_dev){
    int ret = i2cInit(0, NULL);
    ESP_ERROR_CHECK(ret);
    ret = i2sInit(0, NULL, I2S_CLK_SRC_DEFAULT);
    ESP_ERROR_CHECK(ret);
    audio_codec_i2s_cfg_t i2s_cfg = {
        .rx_handle = i2s_keep[0]->rx_handle,
        .tx_handle = i2s_keep[0]->tx_handle,
    };

    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (data_if == NULL){
        ESP_LOGE(TAG, "audio_codec_new_i2s_data ERROR");
        return ESP_FAIL;
    }

    audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES7243E_ADDR,
                                     .bus_handle = i2c_bus_handle};

    const audio_codec_ctrl_if_t *in_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    if (in_ctrl_if == NULL){
        ESP_LOGE(TAG, "audio_codec_new_i2c_ctrl ERROR");
        return ESP_FAIL;
    }
    #ifdef ES7243E
    // New input codec interface
    es7243e_codec_cfg_t es7243e_cfg = {
        .ctrl_if = in_ctrl_if
    };
    const audio_codec_if_t *in_codec_if = es7243e_codec_new(&es7243e_cfg);
    if (in_codec_if == NULL){
        ESP_LOGE(TAG, "es7243e_codec_new ERROR");
        return ESP_FAIL;
    }
    #elif ES7210
    // New input codec interface
    es7210_codec_cfg_t es7210_cfg = {
        .ctrl_if = in_ctrl_if,
        .mic_selected = ES7210_SEL_MIC1 | ES7210_SEL_MIC2 | ES7210_SEL_MIC3 | ES7210_SEL_MIC4,
    };
    const audio_codec_if_t *in_codec_if = es7210_codec_new(&es7210_cfg);
    if (in_codec_if == NULL){
        ESP_LOGE(TAG, "es7210_codec_new ERROR");
        return ESP_FAIL;
    }
    #endif
    // New input codec device
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = in_codec_if,
        .data_if = data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
    };

    *record_dev = esp_codec_dev_new(&dev_cfg);
    if (record_dev == NULL){
        ESP_LOGE(TAG, "esp_codec_dev_new ERROR");
        return ESP_FAIL;
    }
    
    ret = esp_codec_dev_set_in_gain(*record_dev, 30.0);
    ESP_ERROR_CHECK(ret);

    return ret;
}



