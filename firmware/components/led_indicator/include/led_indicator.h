#pragma once
#include "esp_err.h"
#include "va_types.h"

esp_err_t va_led_init(void);
void va_led_set_state(va_state_t state);
