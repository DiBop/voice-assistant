#pragma once
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

typedef void (*tts_audio_cb_t)(const uint8_t *data, int len, void *user_ctx);
typedef void (*tts_done_cb_t)(void *user_ctx);

esp_err_t ws_transport_init(const char *uri);
esp_err_t ws_transport_connect(void);
esp_err_t ws_transport_send_audio(const int16_t *pcm, int samples);
esp_err_t ws_transport_send_control(const char *type);
void ws_transport_set_tts_cb(tts_audio_cb_t audio_cb, tts_done_cb_t done_cb, void *ctx);
bool ws_transport_is_connected(void);
