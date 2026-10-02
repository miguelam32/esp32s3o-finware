/*
 * ws2812.c — WS2812 sobre RMT, IDF 6.0.1
 *
 * Resolucion de 10 MHz -> tick = 100 ns. Los tiempos del WS2812 caen en
 * numeros enteros limpios:
 *   T0H = 400 ns -> 4 ticks     T0L = 850 ns -> 9 ticks
 *   T1H = 800 ns -> 8 ticks     T1L = 450 ns -> 5 ticks
 *   RESET > 50 us -> simbolo extra de 120 us en bajo
 *
 * Orden de color del WS2812C: GRB (no RGB).
 */
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ws2812.h"

static const char *TAG = "ws2812";

#define WS2812_RES_HZ   (10u * 1000u * 1000u)   /* 10 MHz */

#define T0H   4u
#define T0L   9u
#define T1H   8u
#define T1L   5u
#define T_RES 600u      /* 60 us por mitad = 120 us de reset */

static rmt_channel_handle_t  s_chan;
static rmt_encoder_handle_t  s_enc;
static int                   s_ready;

/* ------------------------------------------------------------------ */
esp_err_t ws2812_init(int gpio)
{
    if (s_ready) return ESP_OK;

    rmt_tx_channel_config_t cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = gpio,
        .mem_block_symbols = 64,            /* 25 simbolos: sobra */
        .resolution_hz     = WS2812_RES_HZ,
        .trans_queue_depth = 4,
    };

    esp_err_t e = rmt_new_tx_channel(&cfg, &s_chan);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel(GPIO%d): %s", gpio, esp_err_to_name(e));
        return e;
    }

    rmt_copy_encoder_config_t ecfg = { };
    e = rmt_new_copy_encoder(&ecfg, &s_enc);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder: %s", esp_err_to_name(e));
        return e;
    }

    e = rmt_enable(s_chan);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable: %s", esp_err_to_name(e));
        return e;
    }

    s_ready = 1;
    ESP_LOGI(TAG, "WS2812 listo en GPIO%d", gpio);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
void ws2812_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_ready) return;

    /* 24 bits de color (GRB) + 1 simbolo de reset = 25 */
    rmt_symbol_word_t sym[25];

    const uint8_t grb[3] = { g, r, b };
    int i = 0;

    for (int byte = 0; byte < 3; byte++) {
        uint8_t v = grb[byte];
        for (int bit = 7; bit >= 0; bit--) {
            if (v & (1u << bit)) {
                sym[i].level0 = 1; sym[i].duration0 = T1H;
                sym[i].level1 = 0; sym[i].duration1 = T1L;
            } else {
                sym[i].level0 = 1; sym[i].duration0 = T0H;
                sym[i].level1 = 0; sym[i].duration1 = T0L;
            }
            i++;
        }
    }

    /* reset: linea en bajo > 50 us para que el LED latch-e */
    sym[24].level0    = 0; sym[24].duration0 = T_RES;
    sym[24].level1    = 0; sym[24].duration1 = T_RES;

    rmt_transmit_config_t tcfg = { .loop_count = 0 };

    if (rmt_transmit(s_chan, s_enc, sym, sizeof(sym), &tcfg) != ESP_OK) {
        return;   /* transmision anterior aun en curso: descartamos */
    }
    /* bloqueante: al volver, el color ya esta en el LED */
    rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(50));
}

void ws2812_off(void)
{
    ws2812_set(0, 0, 0);
}

/* ------------------------------------------------------------------ */
void ws2812_wheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b)
{
    if (pos < 85) {
        *r = (uint8_t)(pos * 3);
        *g = (uint8_t)(255 - pos * 3);
        *b = 0;
    } else if (pos < 170) {
        pos = (uint8_t)(pos - 85);
        *r = (uint8_t)(255 - pos * 3);
        *g = 0;
        *b = (uint8_t)(pos * 3);
    } else {
        pos = (uint8_t)(pos - 170);
        *r = 0;
        *g = (uint8_t)(pos * 3);
        *b = (uint8_t)(255 - pos * 3);
    }
}
