#include "ws_transport.h"
#include "esp_websocket_client.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ws_transport";

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_CONNECT_TIMEOUT_MS  15000

#define WS_OPCODE_TEXT   0x1
#define WS_OPCODE_BINARY 0x2

static EventGroupHandle_t            s_wifi_events = NULL;
static esp_websocket_client_handle_t s_ws          = NULL;
static tts_audio_cb_t                s_audio_cb    = NULL;
static tts_done_cb_t                 s_done_cb     = NULL;
static void                         *s_cb_ctx      = NULL;

// ── WiFi ─────────────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi connected, ip=" IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_connect(void) {
    s_wifi_events = xEventGroupCreate();
    if (!s_wifi_events) return ESP_ERR_NO_MEM;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) return ret;

    esp_err_t netif_ret = esp_netif_init();
    if (netif_ret != ESP_OK && netif_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "netif init failed: %s", esp_err_to_name(netif_ret));
        return netif_ret;
    }
    esp_err_t loop_ret = esp_event_loop_create_default();
    if (loop_ret != ESP_OK && loop_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "event loop create failed: %s", esp_err_to_name(loop_ret));
        return loop_ret;
    }
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid,     CONFIG_VA_WIFI_SSID,     sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_VA_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS));
    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi connect timeout after %d ms", WIFI_CONNECT_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

// ── WebSocket ────────────────────────────────────────────────────────────

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data) {
    esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;
    switch (id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket connected");
            break;
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "WebSocket disconnected");
            break;
        case WEBSOCKET_EVENT_DATA:
            if (ev->op_code == WS_OPCODE_BINARY) {  // binary — TTS audio PCM
                if (s_audio_cb) {
                    s_audio_cb((const uint8_t *)ev->data_ptr, ev->data_len, s_cb_ctx);
                }
            } else if (ev->op_code == WS_OPCODE_TEXT) {  // text — control message
                if (ev->data_ptr && ev->data_len > 0 &&
                    memmem(ev->data_ptr, ev->data_len, "tts_done", 8) != NULL &&
                    s_done_cb) {
                    s_done_cb(s_cb_ctx);
                }
            }
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket error");
            break;
        default:
            break;
    }
}

// ── Public API ───────────────────────────────────────────────────────────

esp_err_t ws_transport_init(const char *uri) {
    if (s_ws != NULL) {
        ESP_LOGW(TAG, "already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = wifi_connect();
    if (ret != ESP_OK) return ret;

    esp_websocket_client_config_t ws_cfg = {0};
    ws_cfg.uri                   = uri;
    ws_cfg.reconnect_timeout_ms  = 2000;
    ws_cfg.network_timeout_ms    = 10000;

    s_ws = esp_websocket_client_init(&ws_cfg);
    if (!s_ws) {
        ESP_LOGE(TAG, "esp_websocket_client_init failed");
        return ESP_FAIL;
    }

    ret = esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY,
                                        ws_event_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register events failed: %s", esp_err_to_name(ret));
        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
        return ret;
    }

    ESP_LOGI(TAG, "ws_transport initialised, uri=%s", uri);
    return ESP_OK;
}

esp_err_t ws_transport_connect(void) {
    if (!s_ws) return ESP_ERR_INVALID_STATE;
    return esp_websocket_client_start(s_ws);
}

esp_err_t ws_transport_send_audio(const int16_t *pcm, int samples) {
    if (!s_ws || !esp_websocket_client_is_connected(s_ws)) {
        return ESP_ERR_INVALID_STATE;
    }
    int sent = esp_websocket_client_send_bin(s_ws, (const char *)pcm,
                                             samples * sizeof(int16_t),
                                             pdMS_TO_TICKS(1000));
    return (sent >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t ws_transport_send_control(const char *type) {
    if (!s_ws || !esp_websocket_client_is_connected(s_ws)) {
        return ESP_ERR_INVALID_STATE;
    }
    char msg[64];
    int len = snprintf(msg, sizeof(msg), "{\"type\":\"%s\"}", type);
    if (len < 0 || len >= (int)sizeof(msg)) return ESP_ERR_INVALID_ARG;
    int sent = esp_websocket_client_send_text(s_ws, msg, len, pdMS_TO_TICKS(1000));
    return (sent >= 0) ? ESP_OK : ESP_FAIL;
}

// Must be called before ws_transport_connect(). Callback pointers are
// read from the WebSocket event task — setting them after connect() is
// a data race on dual-core ESP32-S3.
void ws_transport_set_tts_cb(tts_audio_cb_t audio_cb, tts_done_cb_t done_cb, void *ctx) {
    if (s_ws && esp_websocket_client_is_connected(s_ws)) {
        ESP_LOGW(TAG, "set_tts_cb called after connect — potential race condition");
    }
    s_audio_cb = audio_cb;
    s_done_cb  = done_cb;
    s_cb_ctx   = ctx;
}

bool ws_transport_is_connected(void) {
    return s_ws && esp_websocket_client_is_connected(s_ws);
}
