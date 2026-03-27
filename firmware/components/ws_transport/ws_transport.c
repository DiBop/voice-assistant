#include "ws_transport.h"
esp_err_t ws_transport_init(const char *uri) { return ESP_OK; }
esp_err_t ws_transport_connect(void) { return ESP_OK; }
esp_err_t ws_transport_send_audio(const int16_t *pcm, int samples) { return ESP_OK; }
esp_err_t ws_transport_send_control(const char *type) { return ESP_OK; }
void ws_transport_set_tts_cb(tts_audio_cb_t a, tts_done_cb_t d, void *ctx) {}
bool ws_transport_is_connected(void) { return false; }
