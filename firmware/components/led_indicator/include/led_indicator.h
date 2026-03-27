#pragma once
#include "esp_err.h"
#include "va_types.h"

esp_err_t led_indicator_init(void);
void led_set_state(va_state_t state);
