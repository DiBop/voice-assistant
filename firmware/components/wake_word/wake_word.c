#include "wake_word.h"
#include "audio_recorder.h"
#include "recorder_sr.h"
#include "audio_pipeline_mgr.h"
#include "esp_log.h"

static const char *TAG = "wake_word";

static audio_rec_handle_t s_recorder  = NULL;
static wake_word_cb_t     s_user_cb   = NULL;
static void              *s_user_ctx  = NULL;

/* Feed 16 kHz mono PCM from the pipeline into the AFE.
 * recorder_data_read_t signature: int (void *buf, int buf_sz, void *user_ctx, TickType_t ticks)
 * buf_sz is in bytes; audio_pipeline_mgr_read() expects samples (int16_t). */
static int feed_audio_to_afe(void *buf, int buf_sz, void *user_ctx, TickType_t ticks)
{
    int samples = buf_sz / sizeof(int16_t);
    int got = audio_pipeline_mgr_read((int16_t *)buf, samples);
    /* Return bytes read, or 0 on underrun so the AFE feed task just retries. */
    return (got > 0) ? got * (int)sizeof(int16_t) : 0;
}

static esp_err_t recorder_event_cb(audio_rec_evt_t *event, void *user_data)
{
    if (event->type == AUDIO_REC_WAKEUP_START) {
        ESP_LOGI(TAG, "wake word detected");
        if (s_user_cb) {
            s_user_cb(s_user_ctx);
        }
    }
    return ESP_OK;
}

esp_err_t wake_word_init(wake_word_cb_t cb, void *user_ctx)
{
    if (s_recorder != NULL) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_user_cb  = cb;
    s_user_ctx = user_ctx;

    /* "M" = single microphone channel, no reference channel. */
    recorder_sr_cfg_t sr_cfg = DEFAULT_RECORDER_SR_CFG(
        "M",
        "model",
        AFE_TYPE_SR,
        AFE_MODE_HIGH_PERF
    );

    /* Override defaults for a single-mic, wake-word-only scenario. */
    sr_cfg.afe_cfg->memory_alloc_mode       = AFE_MEMORY_ALLOC_MORE_PSRAM;
    sr_cfg.afe_cfg->wakenet_init            = true;
    sr_cfg.afe_cfg->vad_init                = true;
    sr_cfg.afe_cfg->vad_mode                = VAD_MODE_4;
    sr_cfg.afe_cfg->aec_init                = false;
    sr_cfg.afe_cfg->se_init                 = false;
    sr_cfg.afe_cfg->ns_init                 = false;
    sr_cfg.afe_cfg->agc_init                = false;
    sr_cfg.afe_cfg->wakenet_mode            = DET_MODE_90;
    /* pcm_config is already set by afe_config_init() from the "M" format string;
     * no manual override needed for single mic. */
    sr_cfg.multinet_init                    = false;

    audio_rec_cfg_t rec_cfg = AUDIO_RECORDER_DEFAULT_CFG();
    rec_cfg.read     = (recorder_data_read_t)feed_audio_to_afe;
    rec_cfg.event_cb = recorder_event_cb;
    rec_cfg.vad_off  = 1000; /* ms silence before VAD_END */

    rec_cfg.sr_handle = recorder_sr_create(&sr_cfg, &rec_cfg.sr_iface);
    if (rec_cfg.sr_handle == NULL) {
        ESP_LOGE(TAG, "recorder_sr_create failed");
        s_user_cb  = NULL;
        s_user_ctx = NULL;
        return ESP_FAIL;
    }

    s_recorder = audio_recorder_create(&rec_cfg);
    if (s_recorder == NULL) {
        ESP_LOGE(TAG, "audio_recorder_create failed");
        s_user_cb  = NULL;
        s_user_ctx = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "wake word detector ready (\"Hi ESP\")");
    return ESP_OK;
}

esp_err_t wake_word_start(void)
{
    if (!s_recorder) {
        return ESP_ERR_INVALID_STATE;
    }
    return audio_recorder_trigger_start(s_recorder);
}

esp_err_t wake_word_stop(void)
{
    if (!s_recorder) {
        return ESP_ERR_INVALID_STATE;
    }
    return audio_recorder_trigger_stop(s_recorder);
}
