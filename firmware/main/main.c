#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "va_types.h"
#include "led_indicator.h"
#include "audio_pipeline_mgr.h"
#include "wake_word.h"
#include "ws_transport.h"

static const char *TAG = "main";

// ── Event types ──────────────────────────────────────────────────────────

typedef enum {
    EVT_WAKE_WORD,
    EVT_SILENCE_TIMEOUT,
    EVT_SERVER_TIMEOUT,
    EVT_TTS_AUDIO,
    EVT_TTS_DONE,
} va_event_type_t;

typedef struct {
    va_event_type_t  type;
    const uint8_t   *data;
    int              data_len;
} va_event_t;

static QueueHandle_t s_evt_queue = NULL;

// ── Timeouts ─────────────────────────────────────────────────────────────

#define LISTEN_TIMEOUT_MS  8000
#define SERVER_TIMEOUT_MS  30000

// ── Streaming flag ───────────────────────────────────────────────────────

static volatile bool s_streaming = false;

// ── Callbacks ────────────────────────────────────────────────────────────

static void on_wake_word(void *ctx) {
    va_event_t ev = { .type = EVT_WAKE_WORD, .data = NULL, .data_len = 0 };
    xQueueSend(s_evt_queue, &ev, 0);
}

static void on_tts_audio(const uint8_t *data, int len, void *ctx) {
    va_event_t ev = { .type = EVT_TTS_AUDIO, .data = data, .data_len = len };
    xQueueSend(s_evt_queue, &ev, 0);
}

static void on_tts_done(void *ctx) {
    va_event_t ev = { .type = EVT_TTS_DONE, .data = NULL, .data_len = 0 };
    xQueueSend(s_evt_queue, &ev, 0);
}

// ── Stream task ──────────────────────────────────────────────────────────

static void stream_task(void *arg) {
    int16_t buf[320];  // 20ms of 16kHz mono
    while (1) {
        if (s_streaming && ws_transport_is_connected()) {
            int got = audio_pipeline_mgr_read(buf, 320);
            if (got > 0) {
                ws_transport_send_audio(buf, got / (int)sizeof(int16_t));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ── State machine task ───────────────────────────────────────────────────

static void state_machine_task(void *arg) {
    va_state_t state   = VA_STATE_IDLE;
    va_event_t ev;
    TickType_t deadline = portMAX_DELAY;

    va_led_set_state(VA_STATE_IDLE);

    while (1) {
        TickType_t now     = xTaskGetTickCount();
        TickType_t timeout = (deadline == portMAX_DELAY)
                             ? portMAX_DELAY
                             : (deadline > now ? deadline - now : 0);

        if (xQueueReceive(s_evt_queue, &ev, timeout) == pdFALSE) {
            // Timeout expired
            if (state == VA_STATE_LISTENING || state == VA_STATE_STREAMING) {
                ESP_LOGW(TAG, "recording timeout -> WAITING");
                s_streaming = false;
                ws_transport_send_control("end");
                state = VA_STATE_WAITING;
                va_led_set_state(state);
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SERVER_TIMEOUT_MS);
            } else if (state == VA_STATE_WAITING) {
                ESP_LOGE(TAG, "server timeout -> IDLE");
                state = VA_STATE_IDLE;
                va_led_set_state(state);
                deadline = portMAX_DELAY;
            }
            continue;
        }

        switch (state) {
            case VA_STATE_IDLE:
                if (ev.type == EVT_WAKE_WORD) {
                    ESP_LOGI(TAG, "IDLE -> LISTENING");
                    state = VA_STATE_LISTENING;
                    va_led_set_state(state);
                    ws_transport_send_control("start");
                    s_streaming = true;
                    state = VA_STATE_STREAMING;
                    va_led_set_state(state);
                    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LISTEN_TIMEOUT_MS);
                }
                break;

            case VA_STATE_LISTENING:
            case VA_STATE_STREAMING:
                if (ev.type == EVT_SILENCE_TIMEOUT) {
                    ESP_LOGI(TAG, "STREAMING -> WAITING (silence)");
                    s_streaming = false;
                    ws_transport_send_control("end");
                    state = VA_STATE_WAITING;
                    va_led_set_state(state);
                    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SERVER_TIMEOUT_MS);
                }
                break;

            case VA_STATE_WAITING:
                if (ev.type == EVT_TTS_AUDIO) {
                    ESP_LOGI(TAG, "WAITING -> SPEAKING");
                    state = VA_STATE_SPEAKING;
                    va_led_set_state(state);
                    deadline = portMAX_DELAY;
                    // TTS audio playback: future task — write ev.data to I2S output
                }
                break;

            case VA_STATE_SPEAKING:
                if (ev.type == EVT_TTS_AUDIO) {
                    // Additional TTS audio chunks — future: write to I2S output
                }
                if (ev.type == EVT_TTS_DONE) {
                    ESP_LOGI(TAG, "SPEAKING -> IDLE");
                    state = VA_STATE_IDLE;
                    va_led_set_state(state);
                    deadline = portMAX_DELAY;
                }
                break;

            default:
                break;
        }
    }
}

// ── App entry ────────────────────────────────────────────────────────────

void app_main(void) {
    ESP_LOGI(TAG, "Voice assistant v0.1 starting");

    s_evt_queue = xQueueCreate(10, sizeof(va_event_t));
    if (!s_evt_queue) {
        ESP_LOGE(TAG, "Failed to create event queue");
        return;
    }

    // Init LED first — show error state during boot
    ESP_ERROR_CHECK(va_led_init());
    va_led_set_state(VA_STATE_ERROR);

    // Audio pipeline
    ESP_ERROR_CHECK(audio_pipeline_mgr_init());
    ESP_ERROR_CHECK(audio_pipeline_mgr_start());

    // WebSocket transport (includes WiFi connect)
    ws_transport_set_tts_cb(on_tts_audio, on_tts_done, NULL);
    ESP_ERROR_CHECK(ws_transport_init(CONFIG_VA_SERVER_URI));
    ESP_ERROR_CHECK(ws_transport_connect());

    // Wake word detector
    ESP_ERROR_CHECK(wake_word_init(on_wake_word, NULL));
    ESP_ERROR_CHECK(wake_word_start());

    // Start tasks
    xTaskCreate(stream_task,        "stream",   4096, NULL, 10, NULL);
    xTaskCreate(state_machine_task, "state_m",  4096, NULL,  5, NULL);

    va_led_set_state(VA_STATE_IDLE);
    ESP_LOGI(TAG, "Ready — say 'Hi ESP'");
}
