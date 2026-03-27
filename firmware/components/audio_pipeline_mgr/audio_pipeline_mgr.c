#include "audio_pipeline_mgr.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "filter_resample.h"
#include "raw_stream.h"
#include "esp_log.h"
#include "va_types.h"

static const char *TAG = "audio_mgr";

static audio_pipeline_handle_t s_pipeline   = NULL;
static audio_element_handle_t  s_i2s_reader = NULL;
static audio_element_handle_t  s_resample   = NULL;
static audio_element_handle_t  s_raw_read   = NULL;

esp_err_t audio_pipeline_mgr_init(void) {
    // Create pipeline
    audio_pipeline_cfg_t pipe_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipe_cfg);
    if (!s_pipeline) {
        ESP_LOGE(TAG, "pipeline init failed");
        return ESP_FAIL;
    }

    // I2S reader — 48kHz, 16-bit mono, from ES7210 mic array
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0, 48000, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
    i2s_cfg.std_cfg.slot_cfg.slot_mode        = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.gpio_cfg.mclk             = BOARD_I2S_MCLK_PIN;
    i2s_cfg.std_cfg.gpio_cfg.bclk             = BOARD_I2S_BCLK_PIN;
    i2s_cfg.std_cfg.gpio_cfg.ws               = BOARD_I2S_WS_PIN;
    i2s_cfg.std_cfg.gpio_cfg.din              = BOARD_I2S_DIN_PIN;
    i2s_cfg.std_cfg.gpio_cfg.dout             = I2S_GPIO_UNUSED;
    s_i2s_reader = i2s_stream_init(&i2s_cfg);
    if (!s_i2s_reader) {
        ESP_LOGE(TAG, "i2s_stream_init failed");
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    // Resample 48kHz mono → 16kHz mono (required by ESP-SR wake word engine)
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate  = 48000;
    rsp_cfg.dest_rate = 16000;
    rsp_cfg.src_ch    = 1;
    rsp_cfg.dest_ch   = 1;
    rsp_cfg.mode      = RESAMPLE_ENCODE_MODE;
    s_resample = rsp_filter_init(&rsp_cfg);
    if (!s_resample) {
        ESP_LOGE(TAG, "rsp_filter_init failed");
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    // Raw stream — read tap for wake word detector and WebSocket streamer
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    s_raw_read = raw_stream_init(&raw_cfg);
    if (!s_raw_read) {
        ESP_LOGE(TAG, "raw_stream_init failed");
        audio_pipeline_deinit(s_pipeline);
        s_pipeline = NULL;
        return ESP_FAIL;
    }

    // Register and link: i2s → resample → raw
    audio_pipeline_register(s_pipeline, s_i2s_reader, "i2s");
    audio_pipeline_register(s_pipeline, s_resample,   "resample");
    audio_pipeline_register(s_pipeline, s_raw_read,   "raw");

    const char *links[] = {"i2s", "resample", "raw"};
    audio_pipeline_link(s_pipeline, links, 3);

    ESP_LOGI(TAG, "audio capture pipeline ready (48kHz -> 16kHz mono)");
    return ESP_OK;
}

esp_err_t audio_pipeline_mgr_start(void) {
    if (!s_pipeline) return ESP_ERR_INVALID_STATE;
    return audio_pipeline_run(s_pipeline);
}

esp_err_t audio_pipeline_mgr_stop(void) {
    if (!s_pipeline) return ESP_ERR_INVALID_STATE;
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    return ESP_OK;
}

// Read `samples` 16-bit PCM samples into buf.
// Returns number of bytes read (= samples * 2), or <0 on error.
int audio_pipeline_mgr_read(int16_t *buf, int samples) {
    if (!s_raw_read) return -1;
    return raw_stream_read(s_raw_read, (char *)buf, samples * sizeof(int16_t));
}
