#pragma once

// ── Board Pin Config (WaveShare ESP32-S3-Audio-Board) ─────────────────────
// Source: confirmed from community ESPHome implementations
// https://www.waveshare.com/wiki/ESP32-S3-AUDIO-Board
// Codec: ES8311 (DAC/speaker) + ES7210 (ADC/mic array)
// RGB: 7x WS2812 on GPIO38

// I2C control bus (ES8311 + ES7210 + TCA9555 IO expander)
#define BOARD_I2C_SDA_PIN     11     // ES8311/ES7210 control bus SDA
#define BOARD_I2C_SCL_PIN     10     // ES8311/ES7210 control bus SCL

// I2S audio bus
#define BOARD_I2S_MCLK_PIN    12     // I2S master clock
#define BOARD_I2S_BCLK_PIN    13     // I2S bit clock (SCLK)
#define BOARD_I2S_WS_PIN      14     // I2S word select / LRCLK
#define BOARD_I2S_DIN_PIN     15     // I2S data in  (mic → ESP, ES7210 SDOUT)
#define BOARD_I2S_DOUT_PIN    16     // I2S data out (ESP → speaker, ES8311 SDIN)

// RGB LED ring
#define BOARD_LED_GPIO        38     // 7x WS2812 RGB LEDs (GRB order)
#define BOARD_LED_COUNT       7

// Codec I2C addresses (7-bit)
#define BOARD_ES8311_ADDR     0x18   // ES8311 DAC  (AD pin low)
#define BOARD_ES7210_ADDR     0x40   // ES7210 ADC  (AD1=0, AD0=0)

// ── Voice Assistant State ──────────────────────────────────────────────────
typedef enum {
    VA_STATE_IDLE = 0,
    VA_STATE_LISTENING,
    VA_STATE_STREAMING,
    VA_STATE_WAITING,
    VA_STATE_SPEAKING,
    VA_STATE_ERROR,
} va_state_t;
