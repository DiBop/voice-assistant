#include "led_indicator.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "led";

// ── RMT / WS2812 internals ────────────────────────────────────────────────────

#define WS2812_RMT_RESOLUTION_HZ  10000000  // 10 MHz → 0.1 µs per tick

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int            state;
    rmt_symbol_word_t reset_code;
} ws2812_encoder_t;

// GRB pixel buffer sent over RMT
typedef struct { uint8_t g, r, b; } grb_t;

static RMT_ENCODER_FUNC_ATTR size_t ws2812_encode(
        rmt_encoder_t *encoder, rmt_channel_handle_t channel,
        const void *data, size_t data_size, rmt_encode_state_t *ret_state)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encode_state_t sess = RMT_ENCODING_RESET, st = RMT_ENCODING_RESET;
    size_t n = 0;
    switch (enc->state) {
    case 0:
        n += enc->bytes_encoder->encode(enc->bytes_encoder, channel, data, data_size, &sess);
        if (sess & RMT_ENCODING_COMPLETE) { enc->state = 1; }
        if (sess & RMT_ENCODING_MEM_FULL) { st |= RMT_ENCODING_MEM_FULL; goto out; }
        // fall-through
    case 1:
        n += enc->copy_encoder->encode(enc->copy_encoder, channel,
                &enc->reset_code, sizeof(enc->reset_code), &sess);
        if (sess & RMT_ENCODING_COMPLETE) {
            enc->state = RMT_ENCODING_RESET;
            st |= RMT_ENCODING_COMPLETE;
        }
        if (sess & RMT_ENCODING_MEM_FULL) { st |= RMT_ENCODING_MEM_FULL; goto out; }
    }
out:
    *ret_state = st;
    return n;
}

static esp_err_t ws2812_encoder_reset(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_encoder_reset(enc->bytes_encoder);
    rmt_encoder_reset(enc->copy_encoder);
    enc->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t ws2812_encoder_del(rmt_encoder_t *encoder)
{
    ws2812_encoder_t *enc = __containerof(encoder, ws2812_encoder_t, base);
    rmt_del_encoder(enc->bytes_encoder);
    rmt_del_encoder(enc->copy_encoder);
    free(enc);
    return ESP_OK;
}

static esp_err_t create_ws2812_encoder(rmt_encoder_handle_t *ret)
{
    ws2812_encoder_t *enc = rmt_alloc_encoder_mem(sizeof(ws2812_encoder_t));
    if (!enc) return ESP_ERR_NO_MEM;

    enc->base.encode = ws2812_encode;
    enc->base.reset  = ws2812_encoder_reset;
    enc->base.del    = ws2812_encoder_del;
    enc->state       = 0;

    // WS2812 timing @ 10 MHz (1 tick = 100 ns)
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 4,  // T0H = 0.4 µs
                  .level1 = 0, .duration1 = 8 }, // T0L = 0.8 µs  (total 1.2 µs)
        .bit1 = { .level0 = 1, .duration0 = 8,  // T1H = 0.8 µs
                  .level1 = 0, .duration1 = 4 }, // T1L = 0.4 µs  (total 1.2 µs)
        .flags.msb_first = 1,                    // WS2812: GRB, MSB first
    };
    esp_err_t err = rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes_encoder);
    if (err != ESP_OK) { free(enc); return err; }

    rmt_copy_encoder_config_t copy_cfg = {};
    err = rmt_new_copy_encoder(&copy_cfg, &enc->copy_encoder);
    if (err != ESP_OK) { rmt_del_encoder(enc->bytes_encoder); free(enc); return err; }

    // Reset pulse: LOW for 50 µs = 500 ticks at 10 MHz, split into two durations
    uint32_t reset_ticks = WS2812_RMT_RESOLUTION_HZ / 1000000 * 50 / 2; // 250 ticks
    enc->reset_code = (rmt_symbol_word_t){
        .level0 = 0, .duration0 = reset_ticks,
        .level1 = 0, .duration1 = reset_ticks,
    };

    *ret = &enc->base;
    return ESP_OK;
}

// ── Module state ──────────────────────────────────────────────────────────────

static rmt_channel_handle_t s_rmt_chan  = NULL;
static rmt_encoder_handle_t s_encoder   = NULL;
static TaskHandle_t         s_pulse_task = NULL;
static volatile va_state_t  s_current_state = VA_STATE_IDLE;
static grb_t                s_pixels[BOARD_LED_COUNT];

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

static bool is_blink_state(va_state_t s) {
    return s == VA_STATE_ERROR;
}

static void flush_pixels(void)
{
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_transmit(s_rmt_chan, s_encoder, s_pixels, sizeof(s_pixels), &tx_cfg);
    rmt_tx_wait_all_done(s_rmt_chan, pdMS_TO_TICKS(100));
}

static void set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < BOARD_LED_COUNT; i++) {
        s_pixels[i] = (grb_t){ .g = g, .r = r, .b = b };
    }
    flush_pixels();
}

static void pulse_task(void *arg)
{
    bool bright = true;
    while (1) {
        va_state_t st = s_current_state;
        if (is_pulse_state(st)) {
            rgb_t c = STATE_COLORS[st];
            uint8_t div = bright ? 1 : 8;
            set_all(c.r / div, c.g / div, c.b / div);
            bright = !bright;
            vTaskDelay(pdMS_TO_TICKS(500));
        } else if (is_blink_state(st)) {
            rgb_t c = STATE_COLORS[st];
            set_all(bright ? c.r : 0, bright ? c.g : 0, bright ? c.b : 0);
            bright = !bright;
            vTaskDelay(pdMS_TO_TICKS(150));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}

// ── Public API ────────────────────────────────────────────────────────────────

esp_err_t va_led_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num         = BOARD_LED_GPIO,
        .clk_src          = RMT_CLK_SRC_DEFAULT,
        .resolution_hz    = WS2812_RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    esp_err_t ret = rmt_new_tx_channel(&chan_cfg, &s_rmt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT channel init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = create_ws2812_encoder(&s_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WS2812 encoder init failed: %s", esp_err_to_name(ret));
        rmt_del_channel(s_rmt_chan);
        return ret;
    }

    ret = rmt_enable(s_rmt_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RMT enable failed: %s", esp_err_to_name(ret));
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_rmt_chan);
        return ret;
    }

    // Clear all LEDs
    set_all(0, 0, 0);

    xTaskCreate(pulse_task, "led_pulse", 2048, NULL, 3, &s_pulse_task);
    ESP_LOGI(TAG, "LED indicator ready (%d LEDs on GPIO %d)", BOARD_LED_COUNT, BOARD_LED_GPIO);
    return ESP_OK;
}

void va_led_set_state(va_state_t state)
{
    s_current_state = state;
    if (!is_pulse_state(state) && !is_blink_state(state)) {
        rgb_t c = STATE_COLORS[state];
        set_all(c.r, c.g, c.b);
    }
}
