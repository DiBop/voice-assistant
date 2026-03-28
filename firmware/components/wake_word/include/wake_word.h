#pragma once
#include "esp_err.h"

typedef void (*wake_word_cb_t)(void *user_ctx);

esp_err_t wake_word_init(wake_word_cb_t cb, void *user_ctx);
esp_err_t wake_word_start(void);
esp_err_t wake_word_stop(void);
