/*
 * nrf24_dual.c — orquestacion de dos nRF24L01+
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nrf24.h"
#include "nrf24_dual.h"

static const char *TAG = "nrf24_dual";

static nrf24_dev_t *s_a;
static nrf24_dev_t *s_b;

static uint8_t           s_rpd[NRF_DUAL_CHANNELS];
static volatile uint32_t s_done;
static volatile uint32_t s_sweep_us;

static TaskHandle_t s_ta;
static TaskHandle_t s_tb;

#define SPLIT 63u      /* A: 0..62    B: 63..125 */

/* ------------------------------------------------------------------ */
static void sweep_worker(void *arg)
{
    nrf24_dev_t *d = (nrf24_dev_t *)arg;
    bool first = (d == s_a);
    uint8_t from = first ? 0u : SPLIT;
    uint8_t to   = first ? (SPLIT - 1u) : (NRF_DUAL_CHANNELS - 1u);

    nrf24_rx_cont(d);                       /* CE alto, y ahi se queda */

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   /* espera orden */

        int64_t t0 = esp_timer_get_time();
        for (uint8_t ch = from; ch <= to; ch++) {
            s_rpd[ch] = nrf24_rpd_fast(d, ch);
        }
        if (first) {
            s_sweep_us = (uint32_t)(esp_timer_get_time() - t0);
        }
        s_done++;
    }
}

/* ------------------------------------------------------------------ */
esp_err_t nrf24_dual_init(const nrf24_cfg_t *a, const nrf24_cfg_t *b)
{
    if (a == NULL || b == NULL) return ESP_ERR_INVALID_ARG;

    s_a = nrf24_new(a);
    s_b = nrf24_new(b);
    if (s_a == NULL || s_b == NULL) {
        ESP_LOGE(TAG, "fallo al crear algun modulo");
        return ESP_FAIL;
    }

    if (xTaskCreatePinnedToCore(sweep_worker, "swp_a", 4096, s_a, 6,
                                &s_ta, 0) != pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(sweep_worker, "swp_b", 4096, s_b, 6,
                                &s_tb, 1) != pdPASS) return ESP_ERR_NO_MEM;

    memset(s_rpd, 0, sizeof(s_rpd));
    s_done = 0;

    ESP_LOGI(TAG, "dual listo: A=SPI%d B=SPI%d", (int)a->host, (int)b->host);
    return ESP_OK;
}

void nrf24_dual_sweep_async(void)
{
    s_done = 0;
    if (s_ta) xTaskNotifyGive(s_ta);
    if (s_tb) xTaskNotifyGive(s_tb);
}

bool nrf24_dual_sweep_done(void)
{
    return s_done >= 2u;
}

void nrf24_dual_result(nrf_dual_result_t *out)
{
    if (out == NULL) return;
    memcpy(out->rpd, s_rpd, sizeof(s_rpd));
    out->sweep_us = s_sweep_us;

    uint32_t hits = 0;
    for (uint32_t i = 0; i < NRF_DUAL_CHANNELS; i++) {
        if (s_rpd[i]) hits++;
    }
    out->hits = hits;
}

nrf24_dev_t *nrf24_dual_a(void) { return s_a; }
nrf24_dev_t *nrf24_dual_b(void) { return s_b; }
