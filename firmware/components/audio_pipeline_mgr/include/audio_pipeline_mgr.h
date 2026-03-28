#pragma once
#include "esp_err.h"
#include <stdint.h>

esp_err_t audio_pipeline_mgr_init(void);
esp_err_t audio_pipeline_mgr_start(void);
esp_err_t audio_pipeline_mgr_stop(void);
int audio_pipeline_mgr_read(int16_t *buf, int samples);
