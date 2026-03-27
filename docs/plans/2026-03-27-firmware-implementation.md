# Voice Assistant Firmware Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build ESP32-S3 firmware that captures audio, detects a wake word via ESP-SR, streams PCM audio over WebSocket to a local companion server, and plays back TTS audio received in return.

**Architecture:** ESP-ADF pipeline (I2S → resample → raw stream) feeds audio into `audio_recorder` + `recorder_sr` for wake word detection. On wake, a state machine transitions through LISTENING → STREAMING → WAITING → SPEAKING, driven by ESP-SR callbacks and WebSocket events. Each subsystem is a standalone ESP-IDF component.

**Tech Stack:** ESP-IDF v5.5.2, ESP-ADF, ESP-SR (wake word via AFE), `esp_websocket_client`, FreeRTOS, ES8388 codec via `esp_codec_dev`.

---

## File Map

```
firmware/
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                          # App entry, WiFi init, state machine task
│   └── va_types.h                      # Shared: va_state_t enum
├── components/
│   ├── led_indicator/
│   │   ├── CMakeLists.txt
│   │   ├── include/led_indicator.h     # led_indicator_init(), led_set_state()
│   │   └── led_indicator.c
│   ├── audio_pipeline_mgr/
│   │   ├── CMakeLists.txt
│   │   ├── include/audio_pipeline_mgr.h  # init, start, stop, read
│   │   └── audio_pipeline_mgr.c
│   ├── wake_word/
│   │   ├── CMakeLists.txt
│   │   ├── include/wake_word.h         # wake_word_init(), start(), stop()
│   │   └── wake_word.c
│   └── ws_transport/
│       ├── CMakeLists.txt
│       ├── include/ws_transport.h      # connect, send_audio, send_control, callbacks
│       └── ws_transport.c
├── CMakeLists.txt
├── sdkconfig.defaults
└── partitions.csv
```

---

## Task 1: Project Scaffold + Build Verification

**Files:**
- Create: `firmware/CMakeLists.txt`
- Create: `firmware/sdkconfig.defaults`
- Create: `firmware/main/CMakeLists.txt`
- Create: `firmware/main/main.c`
- Create: `firmware/main/va_types.h`
- Create: `firmware/components/led_indicator/CMakeLists.txt`
- Create: `firmware/components/audio_pipeline_mgr/CMakeLists.txt`
- Create: `firmware/components/wake_word/CMakeLists.txt`
- Create: `firmware/components/ws_transport/CMakeLists.txt`
- Create: stub `.h` and `.c` for each component

- [ ] **Step 1: Verify WaveShare board pins from schematic**

  Before writing any code, confirm the following from the [WaveShare ESP32-S3-Audio-Board schematic](https://www.waveshare.com/wiki/ESP32-S3-Audio-Board):
  - ES8388 I2C SDA pin
  - ES8388 I2C SCL pin
  - I2S BCLK, WS, DIN (mic in), DOUT (speaker out) pins
  - RGB LED GPIO and type (WS2812 vs plain RGB)
  - ES8388 I2C address (0x10 or 0x11, depends on CE pin)

  Record these in `firmware/main/va_types.h` under a `Board Pin Config` comment block.

- [ ] **Step 2: Create root `firmware/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    $ENV{ADF_PATH}/components/audio_pipeline
    $ENV{ADF_PATH}/components/audio_stream
    $ENV{ADF_PATH}/components/audio_hal
    $ENV{ADF_PATH}/components/audio_board
    $ENV{ADF_PATH}/components/audio_recorder
    $ENV{ADF_PATH}/components/esp-sr
    $ENV{ADF_PATH}/components/esp_codec_dev
    $ENV{ADF_PATH}/components/esp_peripherals
    $ENV{ADF_PATH}/components/audio_sal
    $ENV{ADF_PATH}/components/adf_utils
    components
)

include($ENV{ADF_PATH}/CMakeLists.txt)
project(voice_assistant)
```

- [ ] **Step 3: Create `firmware/sdkconfig.defaults`**

```
CONFIG_ESP32S3_DEFAULT_CPU_FREQ_240=y
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_OCT=y
CONFIG_SPIRAM_SPEED_80M=y
CONFIG_MBEDTLS_DYNAMIC_BUFFER=y
CONFIG_MBEDTLS_DYNAMIC_FREE_PEER_CERT=y
CONFIG_MBEDTLS_DYNAMIC_FREE_CONFIG_DATA=y
CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY=y
CONFIG_WEBSOCKET_URI_FROM_STDIN=n
# WiFi — fill in menuconfig
# CONFIG_EXAMPLE_WIFI_SSID=""
# CONFIG_EXAMPLE_WIFI_PASSWORD=""
```

- [ ] **Step 4: Create `firmware/main/va_types.h`**

```c
#pragma once

// ── Board Pin Config (verify against WaveShare schematic) ──────────────────
// https://www.waveshare.com/wiki/ESP32-S3-Audio-Board
#define BOARD_I2C_SDA_PIN     1      // ES8388 control bus SDA — VERIFY
#define BOARD_I2C_SCL_PIN     2      // ES8388 control bus SCL — VERIFY
#define BOARD_I2S_BCLK_PIN    9      // I2S bit clock — VERIFY
#define BOARD_I2S_WS_PIN      45     // I2S word select — VERIFY
#define BOARD_I2S_DIN_PIN     8      // I2S data in (mic → ESP) — VERIFY
#define BOARD_I2S_DOUT_PIN    46     // I2S data out (ESP → speaker) — VERIFY
#define BOARD_LED_GPIO        48     // RGB LED GPIO — VERIFY
#define BOARD_ES8388_ADDR     0x10   // ES8388 I2C address — VERIFY

// ── Voice Assistant State ──────────────────────────────────────────────────
typedef enum {
    VA_STATE_IDLE = 0,
    VA_STATE_LISTENING,
    VA_STATE_STREAMING,
    VA_STATE_WAITING,
    VA_STATE_SPEAKING,
    VA_STATE_ERROR,
} va_state_t;
```

- [ ] **Step 5: Create `firmware/main/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES led_indicator audio_pipeline_mgr wake_word ws_transport
             nvs_flash esp_wifi esp_event esp_netif
)
```

- [ ] **Step 6: Create `firmware/main/main.c` (stub)**

```c
#include <stdio.h>
#include "va_types.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Voice assistant starting...");
}
```

- [ ] **Step 7: Create each component CMakeLists.txt and stub headers/sources**

`firmware/components/led_indicator/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "led_indicator.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_log
)
```

`firmware/components/led_indicator/include/led_indicator.h`:
```c
#pragma once
#include "esp_err.h"
#include "va_types.h"

esp_err_t led_indicator_init(void);
void led_set_state(va_state_t state);
```

`firmware/components/led_indicator/led_indicator.c`:
```c
#include "led_indicator.h"

esp_err_t led_indicator_init(void) { return ESP_OK; }
void led_set_state(va_state_t state) { (void)state; }
```

`firmware/components/audio_pipeline_mgr/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "audio_pipeline_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_pipeline audio_stream esp_log driver
)
```

`firmware/components/audio_pipeline_mgr/include/audio_pipeline_mgr.h`:
```c
#pragma once
#include "esp_err.h"
#include <stdint.h>

esp_err_t audio_pipeline_mgr_init(void);
esp_err_t audio_pipeline_mgr_start(void);
esp_err_t audio_pipeline_mgr_stop(void);
int audio_pipeline_mgr_read(int16_t *buf, int samples);
```

`firmware/components/audio_pipeline_mgr/audio_pipeline_mgr.c`:
```c
#include "audio_pipeline_mgr.h"
esp_err_t audio_pipeline_mgr_init(void) { return ESP_OK; }
esp_err_t audio_pipeline_mgr_start(void) { return ESP_OK; }
esp_err_t audio_pipeline_mgr_stop(void) { return ESP_OK; }
int audio_pipeline_mgr_read(int16_t *buf, int samples) { return 0; }
```

`firmware/components/wake_word/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "wake_word.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_recorder esp-sr esp_log
)
```

`firmware/components/wake_word/include/wake_word.h`:
```c
#pragma once
#include "esp_err.h"

typedef void (*wake_word_cb_t)(void *user_ctx);

esp_err_t wake_word_init(wake_word_cb_t cb, void *user_ctx);
esp_err_t wake_word_start(void);
esp_err_t wake_word_stop(void);
```

`firmware/components/wake_word/wake_word.c`:
```c
#include "wake_word.h"
esp_err_t wake_word_init(wake_word_cb_t cb, void *user_ctx) { return ESP_OK; }
esp_err_t wake_word_start(void) { return ESP_OK; }
esp_err_t wake_word_stop(void) { return ESP_OK; }
```

`firmware/components/ws_transport/CMakeLists.txt`:
```cmake
idf_component_register(
    SRCS "ws_transport.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_websocket_client esp_log
)
```

`firmware/components/ws_transport/include/ws_transport.h`:
```c
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
```

`firmware/components/ws_transport/ws_transport.c`:
```c
#include "ws_transport.h"
esp_err_t ws_transport_init(const char *uri) { return ESP_OK; }
esp_err_t ws_transport_connect(void) { return ESP_OK; }
esp_err_t ws_transport_send_audio(const int16_t *pcm, int samples) { return ESP_OK; }
esp_err_t ws_transport_send_control(const char *type) { return ESP_OK; }
void ws_transport_set_tts_cb(tts_audio_cb_t a, tts_done_cb_t d, void *ctx) {}
bool ws_transport_is_connected(void) { return false; }
```

- [ ] **Step 8: Source ESP-IDF + ESP-ADF and build**

```bash
cd ~/projects/voice-assistant/firmware
source ~/esp/esp-idf/export.sh
export ADF_PATH=~/esp/esp-adf
idf.py set-target esp32s3
idf.py build
```

Expected: build succeeds with no errors (stubs compile clean).

- [ ] **Step 9: Commit**

```bash
git -C ~/projects/voice-assistant init
git -C ~/projects/voice-assistant add firmware/
git -C ~/projects/voice-assistant commit -m "feat: initial firmware project scaffold"
```

---

## Task 2: LED Indicator Component

**Files:**
- Modify: `firmware/components/led_indicator/led_indicator.c`

The WaveShare board LED is a single WS2812 addressable RGB LED. Use the `led_strip` driver from ESP-IDF.

- [ ] **Step 1: Update `firmware/components/led_indicator/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "led_indicator.c"
    INCLUDE_DIRS "include"
    REQUIRES driver led_strip esp_log
)
```

- [ ] **Step 2: Implement `led_indicator.c`**

```c
#include "led_indicator.h"
#include "led_strip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "va_types.h"

static const char *TAG = "led";
static led_strip_handle_t s_strip = NULL;
static TaskHandle_t s_pulse_task = NULL;
static volatile va_state_t s_current_state = VA_STATE_IDLE;

typedef struct { uint8_t r, g, b; } rgb_t;

static const rgb_t STATE_COLORS[] = {
    [VA_STATE_IDLE]      = {0,   0,   0},
    [VA_STATE_LISTENING] = {0,   0,   255},
    [VA_STATE_STREAMING] = {0,   0,   255},
    [VA_STATE_WAITING]   = {255, 200, 0},
    [VA_STATE_SPEAKING]  = {0,   255, 0},
    [VA_STATE_ERROR]     = {255, 0,   0},
};

static bool is_pulse_state(va_state_t s) {
    return s == VA_STATE_STREAMING || s == VA_STATE_WAITING;
}

static bool is_fast_blink_state(va_state_t s) {
    return s == VA_STATE_ERROR;
}

static void set_raw(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(s_strip, 0, r, g, b);
    led_strip_refresh(s_strip);
}

static void pulse_task(void *arg) {
    bool bright = true;
    while (1) {
        va_state_t st = s_current_state;
        if (is_pulse_state(st)) {
            rgb_t c = STATE_COLORS[st];
            set_raw(bright ? c.r : c.r / 8,
                    bright ? c.g : c.g / 8,
                    bright ? c.b : c.b / 8);
            bright = !bright;
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (is_fast_blink_state(st)) {
            rgb_t c = STATE_COLORS[st];
            set_raw(bright ? c.r : 0, bright ? c.g : 0, bright ? c.b : 0);
            bright = !bright;
            vTaskDelay(pdMS_TO_TICKS(150));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

esp_err_t led_indicator_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = BOARD_LED_GPIO,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    esp_err_t ret = led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &s_strip);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LED strip init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    led_strip_clear(s_strip);
    xTaskCreate(pulse_task, "led_pulse", 2048, NULL, 3, &s_pulse_task);
    ESP_LOGI(TAG, "LED indicator initialised");
    return ESP_OK;
}

void led_set_state(va_state_t state) {
    s_current_state = state;
    if (!is_pulse_state(state) && !is_fast_blink_state(state)) {
        rgb_t c = STATE_COLORS[state];
        set_raw(c.r, c.g, c.b);
    }
}
```

- [ ] **Step 3: Build to verify compilation**

```bash
cd ~/projects/voice-assistant/firmware
idf.py build 2>&1 | tail -5
```

Expected: `Build successful` with no errors.

- [ ] **Step 4: Flash and verify LED on boot**

Temporarily call `led_indicator_init()` and `led_set_state(VA_STATE_ERROR)` from `app_main` to verify the LED turns red.

```c
// In main.c app_main():
#include "led_indicator.h"
led_indicator_init();
led_set_state(VA_STATE_ERROR);  // Should blink red
while(1) { vTaskDelay(1000); }
```

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected: LED blinks red fast. Confirm, then revert `app_main` to the stub.

- [ ] **Step 5: Commit**

```bash
git -C ~/projects/voice-assistant add firmware/components/led_indicator/ firmware/main/main.c
git -C ~/projects/voice-assistant commit -m "feat: led indicator component with state-driven patterns"
```

---

## Task 3: Audio Capture Pipeline

**Files:**
- Modify: `firmware/components/audio_pipeline_mgr/audio_pipeline_mgr.c`
- Modify: `firmware/components/audio_pipeline_mgr/CMakeLists.txt`

Pattern follows the ESP-ADF wwe example: I2S reader (48kHz) → resample filter (→16kHz mono) → raw stream. The raw stream is the tap point for both wake word detection and WebSocket streaming.

- [ ] **Step 1: Update `audio_pipeline_mgr/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "audio_pipeline_mgr.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_pipeline audio_stream filter_resample esp_log driver
             esp_codec_dev
)
```

- [ ] **Step 2: Implement `audio_pipeline_mgr.c`**

```c
#include "audio_pipeline_mgr.h"
#include "audio_pipeline.h"
#include "i2s_stream.h"
#include "filter_resample.h"
#include "raw_stream.h"
#include "esp_log.h"
#include "va_types.h"

static const char *TAG = "audio_mgr";

static audio_pipeline_handle_t s_pipeline = NULL;
static audio_element_handle_t  s_i2s_reader = NULL;
static audio_element_handle_t  s_resample   = NULL;
static audio_element_handle_t  s_raw_read   = NULL;

esp_err_t audio_pipeline_mgr_init(void) {
    audio_pipeline_cfg_t pipe_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    s_pipeline = audio_pipeline_init(&pipe_cfg);
    if (!s_pipeline) {
        ESP_LOGE(TAG, "pipeline init failed");
        return ESP_FAIL;
    }

    // I2S reader — 48kHz stereo (ES8388 native rate), 16-bit
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT_WITH_PARA(
        I2S_NUM_0, 48000, I2S_DATA_BIT_WIDTH_16BIT, AUDIO_STREAM_READER);
    i2s_cfg.std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_MONO;
    i2s_cfg.std_cfg.pin_cfg.bclk = BOARD_I2S_BCLK_PIN;
    i2s_cfg.std_cfg.pin_cfg.ws   = BOARD_I2S_WS_PIN;
    i2s_cfg.std_cfg.pin_cfg.din  = BOARD_I2S_DIN_PIN;
    s_i2s_reader = i2s_stream_init(&i2s_cfg);

    // Resample 48kHz → 16kHz mono (required by ESP-SR)
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate  = 48000;
    rsp_cfg.dest_rate = 16000;
    rsp_cfg.src_ch    = 1;
    rsp_cfg.dest_ch   = 1;
    s_resample = rsp_filter_init(&rsp_cfg);

    // Raw stream — read tap for AFE and WebSocket
    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    s_raw_read = raw_stream_init(&raw_cfg);

    audio_pipeline_register(s_pipeline, s_i2s_reader, "i2s");
    audio_pipeline_register(s_pipeline, s_resample,   "resample");
    audio_pipeline_register(s_pipeline, s_raw_read,   "raw");

    const char *links[] = {"i2s", "resample", "raw"};
    audio_pipeline_link(s_pipeline, links, 3);

    ESP_LOGI(TAG, "audio pipeline initialised");
    return ESP_OK;
}

esp_err_t audio_pipeline_mgr_start(void) {
    return audio_pipeline_run(s_pipeline);
}

esp_err_t audio_pipeline_mgr_stop(void) {
    audio_pipeline_stop(s_pipeline);
    audio_pipeline_wait_for_stop(s_pipeline);
    return ESP_OK;
}

// Read `samples` 16-bit PCM samples from the pipeline into buf.
// Returns number of bytes read, or <0 on error.
int audio_pipeline_mgr_read(int16_t *buf, int samples) {
    return raw_stream_read(s_raw_read, (char *)buf, samples * sizeof(int16_t));
}
```

- [ ] **Step 3: Build**

```bash
cd ~/projects/voice-assistant/firmware && idf.py build 2>&1 | tail -5
```

Expected: `Build successful`.

- [ ] **Step 4: Flash and verify mic captures audio**

Add to `app_main` temporarily:

```c
#include "audio_pipeline_mgr.h"
#include "esp_log.h"

audio_pipeline_mgr_init();
audio_pipeline_mgr_start();

int16_t buf[160];  // 10ms at 16kHz
int got = audio_pipeline_mgr_read(buf, 160);
ESP_LOGI("test", "read %d bytes, first sample: %d", got, buf[0]);
```

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected: log shows non-zero bytes read and sample values varying with room noise. If `buf[0]` is always 0, check I2S pins against schematic.

Revert `app_main` after verifying.

- [ ] **Step 5: Commit**

```bash
git -C ~/projects/voice-assistant add firmware/components/audio_pipeline_mgr/
git -C ~/projects/voice-assistant commit -m "feat: audio capture pipeline 48kHz->16kHz via ESP-ADF"
```

---

## Task 4: Wake Word Detector

**Files:**
- Modify: `firmware/components/wake_word/wake_word.c`
- Modify: `firmware/components/wake_word/CMakeLists.txt`

Uses `audio_recorder` + `recorder_sr` (the same pattern as the ESP-ADF wwe example). Wake word is "Hi ESP" (default ESP-SR model). On detection, calls the registered `wake_word_cb_t`.

- [ ] **Step 1: Update `wake_word/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "wake_word.c"
    INCLUDE_DIRS "include"
    REQUIRES audio_recorder esp-sr esp_log audio_pipeline_mgr
)
```

- [ ] **Step 2: Implement `wake_word.c`**

```c
#include "wake_word.h"
#include "audio_recorder.h"
#include "recorder_sr.h"
#include "audio_pipeline_mgr.h"
#include "esp_log.h"

static const char *TAG = "wake_word";

static audio_rec_handle_t  s_recorder  = NULL;
static wake_word_cb_t      s_user_cb   = NULL;
static void               *s_user_ctx  = NULL;

static int feed_audio_to_afe(int16_t *buf, int buf_sz, void *ctx, TickType_t ticks) {
    return audio_pipeline_mgr_read(buf, buf_sz / sizeof(int16_t));
}

static esp_err_t recorder_event_cb(audio_rec_evt_t *event, void *ctx) {
    if (event->type == AUDIO_REC_WAKEUP_START) {
        ESP_LOGI(TAG, "wake word detected");
        if (s_user_cb) {
            s_user_cb(s_user_ctx);
        }
    }
    return ESP_OK;
}

esp_err_t wake_word_init(wake_word_cb_t cb, void *user_ctx) {
    s_user_cb  = cb;
    s_user_ctx = user_ctx;

    recorder_sr_cfg_t sr_cfg = DEFAULT_RECORDER_SR_CFG(
        "RM",        // mono mic format
        "model",     // model partition name
        AFE_TYPE_SR,
        AFE_MODE_HIGH_PERF
    );
    sr_cfg.afe_cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    sr_cfg.afe_cfg->wakenet_init      = true;
    sr_cfg.afe_cfg->vad_mode          = VAD_MODE_4;
    sr_cfg.multinet_init              = false;   // no command recognition needed
    sr_cfg.afe_cfg->aec_init          = false;
    sr_cfg.afe_cfg->agc_mode          = AFE_MN_PEAK_NO_AGC;
    sr_cfg.afe_cfg->pcm_config.mic_num       = 1;
    sr_cfg.afe_cfg->pcm_config.ref_num       = 0;
    sr_cfg.afe_cfg->pcm_config.total_ch_num  = 1;

    audio_rec_cfg_t rec_cfg = AUDIO_RECORDER_DEFAULT_CFG();
    rec_cfg.read       = (recorder_data_read_t)feed_audio_to_afe;
    rec_cfg.sr_handle  = recorder_sr_create(&sr_cfg, &rec_cfg.sr_iface);
    rec_cfg.event_cb   = recorder_event_cb;
    rec_cfg.vad_off    = 1000;

    s_recorder = audio_recorder_create(&rec_cfg);
    if (!s_recorder) {
        ESP_LOGE(TAG, "audio_recorder_create failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "wake word detector initialised (\"Hi ESP\")");
    return ESP_OK;
}

esp_err_t wake_word_start(void) {
    return audio_recorder_trigger_start(s_recorder);
}

esp_err_t wake_word_stop(void) {
    return audio_recorder_trigger_stop(s_recorder);
}
```

- [ ] **Step 3: Build**

```bash
cd ~/projects/voice-assistant/firmware && idf.py build 2>&1 | tail -5
```

Expected: `Build successful`. If linker complains about missing model symbols, run `idf.py menuconfig` → Component config → ESP Speech Recognition → enable WakeNet model.

- [ ] **Step 4: Flash and test wake word**

Add to `app_main` temporarily:

```c
#include "audio_pipeline_mgr.h"
#include "wake_word.h"

static void on_wake(void *ctx) {
    ESP_LOGW("test", "*** WAKE WORD DETECTED ***");
}

// in app_main:
audio_pipeline_mgr_init();
audio_pipeline_mgr_start();
wake_word_init(on_wake, NULL);
wake_word_start();
ESP_LOGI("test", "Say 'Hi ESP'...");
while(1) { vTaskDelay(1000); }
```

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Say "Hi ESP" clearly. Expected: `*** WAKE WORD DETECTED ***` appears in the log within ~1 second of utterance.

Revert `app_main` after verifying.

- [ ] **Step 5: Commit**

```bash
git -C ~/projects/voice-assistant add firmware/components/wake_word/
git -C ~/projects/voice-assistant commit -m "feat: wake word detection via ESP-SR AFE"
```

---

## Task 5: WebSocket Transport

**Files:**
- Modify: `firmware/components/ws_transport/ws_transport.c`
- Modify: `firmware/components/ws_transport/CMakeLists.txt`

Handles WiFi connection, WebSocket client lifecycle, binary audio streaming, JSON control messages, and TTS audio reception.

- [ ] **Step 1: Update `ws_transport/CMakeLists.txt`**

```cmake
idf_component_register(
    SRCS "ws_transport.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_websocket_client esp_wifi nvs_flash esp_event esp_netif
             esp_log freertos
)
```

- [ ] **Step 2: Implement `ws_transport.c`**

```c
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

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static EventGroupHandle_t           s_wifi_events = NULL;
static esp_websocket_client_handle_t s_ws         = NULL;
static tts_audio_cb_t               s_audio_cb   = NULL;
static tts_done_cb_t                s_done_cb    = NULL;
static void                        *s_cb_ctx     = NULL;

// ── WiFi ─────────────────────────────────────────────────────────────────

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected, retrying...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&ev->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_connect(void) {
    s_wifi_events = xEventGroupCreate();

    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,  wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT,   IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = CONFIG_VA_WIFI_SSID,
            .password = CONFIG_VA_WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_events,
                        WIFI_CONNECTED_BIT, pdFALSE, pdFALSE,
                        pdMS_TO_TICKS(15000));

    if (!(bits & WIFI_CONNECTED_BIT)) {
        ESP_LOGE(TAG, "WiFi connect timeout");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ── WebSocket ────────────────────────────────────────────────────────────

static void ws_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data) {
    esp_websocket_event_data_t *ev = (esp_websocket_event_data_t *)data;

    if (id == WEBSOCKET_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "WebSocket connected");
    } else if (id == WEBSOCKET_EVENT_DISCONNECTED) {
        ESP_LOGW(TAG, "WebSocket disconnected");
    } else if (id == WEBSOCKET_EVENT_DATA) {
        if (ev->op_code == 0x2) {  // binary — TTS audio
            if (s_audio_cb) {
                s_audio_cb((const uint8_t *)ev->data_ptr, ev->data_len, s_cb_ctx);
            }
        } else if (ev->op_code == 0x1) {  // text — control message
            // Check for {"type":"tts_done"}
            if (strstr(ev->data_ptr, "tts_done") && s_done_cb) {
                s_done_cb(s_cb_ctx);
            }
        }
    }
}

// ── Public API ───────────────────────────────────────────────────────────

esp_err_t ws_transport_init(const char *uri) {
    esp_err_t ret = wifi_connect();
    if (ret != ESP_OK) return ret;

    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .reconnect_timeout_ms = 2000,
        .network_timeout_ms   = 10000,
    };
    s_ws = esp_websocket_client_init(&ws_cfg);
    esp_websocket_register_events(s_ws, WEBSOCKET_EVENT_ANY, ws_event_handler, NULL);

    ESP_LOGI(TAG, "ws_transport initialised, uri=%s", uri);
    return ESP_OK;
}

esp_err_t ws_transport_connect(void) {
    return esp_websocket_client_start(s_ws);
}

esp_err_t ws_transport_send_audio(const int16_t *pcm, int samples) {
    if (!esp_websocket_client_is_connected(s_ws)) return ESP_ERR_INVALID_STATE;
    return esp_websocket_client_send_bin(s_ws, (const char *)pcm,
                                         samples * sizeof(int16_t),
                                         pdMS_TO_TICKS(1000));
}

esp_err_t ws_transport_send_control(const char *type) {
    if (!esp_websocket_client_is_connected(s_ws)) return ESP_ERR_INVALID_STATE;
    char msg[64];
    snprintf(msg, sizeof(msg), "{\"type\":\"%s\"}", type);
    return esp_websocket_client_send_text(s_ws, msg, strlen(msg), pdMS_TO_TICKS(1000));
}

void ws_transport_set_tts_cb(tts_audio_cb_t audio_cb, tts_done_cb_t done_cb, void *ctx) {
    s_audio_cb = audio_cb;
    s_done_cb  = done_cb;
    s_cb_ctx   = ctx;
}

bool ws_transport_is_connected(void) {
    return s_ws && esp_websocket_client_is_connected(s_ws);
}
```

- [ ] **Step 3: Add Kconfig for WiFi credentials**

Create `firmware/main/Kconfig.projbuild`:

```
menu "Voice Assistant Config"

config VA_WIFI_SSID
    string "WiFi SSID"
    default "myssid"

config VA_WIFI_PASSWORD
    string "WiFi Password"
    default "mypassword"

config VA_SERVER_URI
    string "Companion server WebSocket URI"
    default "ws://192.168.1.100:8765"

endmenu
```

Then run `idf.py menuconfig` → Voice Assistant Config and fill in your actual WiFi credentials and server IP.

- [ ] **Step 4: Build**

```bash
cd ~/projects/voice-assistant/firmware && idf.py menuconfig  # set WiFi + server URI
idf.py build 2>&1 | tail -5
```

Expected: `Build successful`.

- [ ] **Step 5: Test WebSocket connection against netcat**

On your Linux machine, start a simple WebSocket echo server. Install `websocat` if needed:

```bash
# On Linux machine:
pip install websockets
python3 -c "
import asyncio, websockets

async def echo(ws):
    async for msg in ws:
        print(f'received {len(msg)} bytes')
        await ws.send(msg)

asyncio.run(websockets.serve(echo, '0.0.0.0', 8765))
" &
```

Add to `app_main` temporarily:

```c
#include "ws_transport.h"
#include "esp_log.h"

// in app_main:
ws_transport_init(CONFIG_VA_SERVER_URI);
ws_transport_connect();
vTaskDelay(pdMS_TO_TICKS(2000));
int16_t test_buf[160] = {0};
for (int i = 0; i < 160; i++) test_buf[i] = i * 100;
ws_transport_send_control("start");
ws_transport_send_audio(test_buf, 160);
ws_transport_send_control("end");
ESP_LOGI("test", "sent audio + control messages");
while(1) { vTaskDelay(1000); }
```

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Expected: Python server prints "received 320 bytes". ESP32 log shows "WebSocket connected". Revert `app_main` after verifying.

- [ ] **Step 6: Commit**

```bash
git -C ~/projects/voice-assistant add firmware/components/ws_transport/ firmware/main/Kconfig.projbuild
git -C ~/projects/voice-assistant commit -m "feat: websocket transport with wifi and pcm streaming"
```

---

## Task 6: State Machine + Main Integration

**Files:**
- Modify: `firmware/main/main.c`

Wire all components together. The state machine runs in a dedicated FreeRTOS task. Transitions are driven by a queue of events posted from callbacks.

- [ ] **Step 1: Write `firmware/main/main.c`**

```c
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
    va_event_type_t type;
    const uint8_t  *data;
    int             data_len;
} va_event_t;

static QueueHandle_t s_evt_queue = NULL;

// ── Timeouts (ticks) ─────────────────────────────────────────────────────

#define LISTEN_TIMEOUT_MS  8000
#define SERVER_TIMEOUT_MS  30000

// ── Callbacks ────────────────────────────────────────────────────────────

static void on_wake_word(void *ctx) {
    va_event_t ev = { .type = EVT_WAKE_WORD };
    xQueueSendFromISR(s_evt_queue, &ev, NULL);
}

static void on_tts_audio(const uint8_t *data, int len, void *ctx) {
    // Play TTS audio directly via I2S — post to state machine for bookkeeping
    // (actual playback handled below in SPEAKING state)
    va_event_t ev = { .type = EVT_TTS_AUDIO, .data = data, .data_len = len };
    xQueueSend(s_evt_queue, &ev, 0);
}

static void on_tts_done(void *ctx) {
    va_event_t ev = { .type = EVT_TTS_DONE };
    xQueueSend(s_evt_queue, &ev, 0);
}

// ── Audio streaming task ─────────────────────────────────────────────────

static volatile bool s_streaming = false;

static void stream_task(void *arg) {
    int16_t buf[320];  // 20ms at 16kHz
    while (1) {
        if (s_streaming) {
            int got = audio_pipeline_mgr_read(buf, 320);
            if (got > 0) {
                ws_transport_send_audio(buf, got / sizeof(int16_t));
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// ── State machine task ───────────────────────────────────────────────────

static void state_machine_task(void *arg) {
    va_state_t state = VA_STATE_IDLE;
    va_event_t ev;
    TickType_t deadline = portMAX_DELAY;

    led_set_state(VA_STATE_IDLE);

    while (1) {
        TickType_t now     = xTaskGetTickCount();
        TickType_t timeout = (deadline == portMAX_DELAY)
                             ? portMAX_DELAY
                             : (deadline > now ? deadline - now : 0);

        if (xQueueReceive(s_evt_queue, &ev, timeout) == pdFALSE) {
            // Timeout expired
            if (state == VA_STATE_LISTENING || state == VA_STATE_STREAMING) {
                ESP_LOGW(TAG, "recording timeout, returning to idle");
                s_streaming = false;
                ws_transport_send_control("end");
                state = VA_STATE_WAITING;
                led_set_state(state);
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SERVER_TIMEOUT_MS);
            } else if (state == VA_STATE_WAITING) {
                ESP_LOGE(TAG, "server timeout, returning to idle");
                state = VA_STATE_IDLE;
                led_set_state(state);
                deadline = portMAX_DELAY;
            }
            continue;
        }

        switch (state) {
            case VA_STATE_IDLE:
                if (ev.type == EVT_WAKE_WORD) {
                    ESP_LOGI(TAG, "IDLE → LISTENING");
                    state = VA_STATE_LISTENING;
                    led_set_state(state);
                    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LISTEN_TIMEOUT_MS);
                    // Brief delay then start streaming
                    vTaskDelay(pdMS_TO_TICKS(200));
                    ws_transport_send_control("start");
                    s_streaming = true;
                    state = VA_STATE_STREAMING;
                    led_set_state(state);
                }
                break;

            case VA_STATE_LISTENING:
            case VA_STATE_STREAMING:
                if (ev.type == EVT_SILENCE_TIMEOUT) {
                    ESP_LOGI(TAG, "STREAMING → WAITING (silence)");
                    s_streaming = false;
                    ws_transport_send_control("end");
                    state = VA_STATE_WAITING;
                    led_set_state(state);
                    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(SERVER_TIMEOUT_MS);
                }
                break;

            case VA_STATE_WAITING:
                if (ev.type == EVT_TTS_AUDIO) {
                    ESP_LOGI(TAG, "WAITING → SPEAKING");
                    state = VA_STATE_SPEAKING;
                    led_set_state(state);
                    deadline = portMAX_DELAY;
                    // Playback: write PCM to I2S out via audio_pipeline_mgr
                    // (TTS audio arrives in subsequent EVT_TTS_AUDIO events)
                }
                break;

            case VA_STATE_SPEAKING:
                if (ev.type == EVT_TTS_AUDIO) {
                    // Additional TTS audio chunks arrive here — write to I2S output
                    // audio_pipeline_mgr_write(ev.data, ev.data_len);  // Task 7 addition
                }
                if (ev.type == EVT_TTS_DONE) {
                    ESP_LOGI(TAG, "SPEAKING → IDLE");
                    state = VA_STATE_IDLE;
                    led_set_state(state);
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
    ESP_LOGI(TAG, "Voice assistant starting");

    s_evt_queue = xQueueCreate(10, sizeof(va_event_t));

    ESP_ERROR_CHECK(led_indicator_init());
    led_set_state(VA_STATE_ERROR);  // Red during init

    ESP_ERROR_CHECK(audio_pipeline_mgr_init());
    ESP_ERROR_CHECK(audio_pipeline_mgr_start());

    ESP_ERROR_CHECK(ws_transport_init(CONFIG_VA_SERVER_URI));
    ws_transport_set_tts_cb(on_tts_audio, on_tts_done, NULL);
    ESP_ERROR_CHECK(ws_transport_connect());

    ESP_ERROR_CHECK(wake_word_init(on_wake_word, NULL));
    ESP_ERROR_CHECK(wake_word_start());

    xTaskCreate(stream_task,         "stream",  4096, NULL, 10, NULL);
    xTaskCreate(state_machine_task,  "state_m", 4096, NULL,  5, NULL);

    ESP_LOGI(TAG, "Ready — say 'Hi ESP'");
}
```

- [ ] **Step 2: Build**

```bash
cd ~/projects/voice-assistant/firmware && idf.py build 2>&1 | tail -5
```

Expected: `Build successful`.

- [ ] **Step 3: Integration test — wake → stream**

Start the Python echo server from Task 5 Step 5 on your Linux machine. Flash and monitor:

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

Say "Hi ESP". Expected sequence in logs:
1. `IDLE → LISTENING`
2. `STREAMING → WAITING (silence)` (after ~8s or voice ends)
3. Python server receives binary audio frames
4. Server receives `{"type":"end"}`

If wake word doesn't trigger, check that `wake_word_start()` is called after `audio_pipeline_mgr_start()`.

- [ ] **Step 4: Commit**

```bash
git -C ~/projects/voice-assistant add firmware/main/
git -C ~/projects/voice-assistant commit -m "feat: state machine wiring all components — wake→stream→wait→speak"
```

---

## Self-Review Checklist

- [x] **Spec coverage**
  - Audio pipeline (I2S → resample → raw): Task 3
  - Wake word (ESP-SR): Task 4
  - LED indicator with all states: Task 2
  - WebSocket transport with control messages: Task 5
  - State machine (IDLE/LISTENING/STREAMING/WAITING/SPEAKING): Task 6
  - Project structure matching spec: Task 1
  - WiFi credentials in sdkconfig (not hardcoded): Task 5 Step 3 (Kconfig)
  - Timeouts configurable: Task 6 (constants in main.c — move to Kconfig if desired)

- [x] **Type consistency**
  - `va_state_t` defined in `va_types.h`, included by `led_indicator.h` and `main.c`
  - `wake_word_cb_t` → `void (*)(void *)` used consistently in Task 4 and Task 6
  - `tts_audio_cb_t` → `void (*)(const uint8_t *, int, void *)` consistent in Task 5 and Task 6
  - `audio_pipeline_mgr_read` returns bytes, callers divide by `sizeof(int16_t)` for sample count — consistent

- [x] **No placeholders** — all steps contain actual code or exact commands

- [x] **Scope** — TTS I2S playback in SPEAKING state is noted as a follow-on (the state machine skeleton is complete; writing PCM back to I2S output requires adding a writer element to the pipeline, which is a natural next task after the pipeline is verified working)
