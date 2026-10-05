/*
 * nrf24_dual.c - Motor del jammer: listas de canales + tarea por radio.
 *
 * Dwell deterministico: cada canal recibe un tiempo fijo. La version
 * Arduino del repo de Bruce saltaba con dwell 0, o sea molestia
 * puramente estadistica.
 */
#include "nrf24_dual.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "jam";

/* canal nRF = MHz sobre 2400 */
static const uint8_t ch_test[] = {
    50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80,
     2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
    34, 36, 38, 40, 42, 44, 46, 48
};
static const uint8_t ch_wifi[] = { 2, 7, 12, 17, 22, 27, 32, 37, 42, 47, 52, 57, 62, 67, 72, 77 };
static const uint8_t ch_ble_data[] = {
     2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37
};
static const uint8_t ch_ble_adv[] = { 2, 26, 80 };      /* BLE 37/38/39 */
static const uint8_t ch_spray[] = {
     2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61,
    62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80
};
static const uint8_t ch_usb3[] = { 32, 34, 36, 38, 40, 42, 44, 46, 48, 50,
                                   52, 54, 56, 58, 60, 62, 64, 66, 68, 70 };
static const uint8_t ch_video[] = { 60, 62, 64, 66,  68,  70,  72,  74,  76,  78,
                                    80, 82, 84, 86,  88,  90,  92,  94,  96,  98,
                                   100, 102, 104, 106, 108, 110, 112, 114, 116, 118,
                                   120, 122, 124 };
static const uint8_t ch_rc[] = { 1, 3, 5, 7, 9, 11, 13, 15, 17, 19,
                                21, 23, 25, 27, 29, 31, 33, 35, 37, 39 };
static const uint8_t ch_zigbee[] = {
     4,  5,  6,   9, 10, 11,  14, 15, 16,  19, 20, 21,
    24, 25, 26,  29, 30, 31,  34, 35, 36,  39, 40, 41,
    44, 45, 46,  49, 50, 51,  54, 55, 56,  59, 60, 61,
    64, 65, 66,  69, 70, 71,  74, 75, 76,  79, 80, 81
};
static uint8_t ch_full[124];
static bool    s_lists_ready;

static void build_lists(void)
{
    for (uint8_t i = 0; i < 124; i++) ch_full[i] = (uint8_t)(i + 1);
    s_lists_ready = true;
}

const jam_channel_list_t *jam_mode_list(jam_mode_t mode)
{
    static const jam_channel_list_t lists[JAM_MODE_COUNT] = {
        [JAM_MODE_TEST]       = { "Test",     ch_test,     sizeof(ch_test)     },
        [JAM_MODE_WIFI]       = { "WiFi",     ch_wifi,     sizeof(ch_wifi)     },
        [JAM_MODE_BLE_DATA]   = { "BLE data", ch_ble_data, sizeof(ch_ble_data) },
        [JAM_MODE_BLE_ADV]    = { "BLE adv",  ch_ble_adv,  sizeof(ch_ble_adv)  },
        [JAM_MODE_BAND_SPRAY] = { "Spray",    ch_spray,    sizeof(ch_spray)    },
        [JAM_MODE_USB3]       = { "USB3",     ch_usb3,     sizeof(ch_usb3)     },
        [JAM_MODE_VIDEO]      = { "Video",    ch_video,    sizeof(ch_video)    },
        [JAM_MODE_RC]         = { "RC",       ch_rc,       sizeof(ch_rc)       },
        [JAM_MODE_ZIGBEE]     = { "Zigbee",   ch_zigbee,   sizeof(ch_zigbee)   },
        [JAM_MODE_FULL]       = { "Full",     ch_full,     sizeof(ch_full)     },
    };
    if (!s_lists_ready) build_lists();
    if (mode < 0 || mode >= JAM_MODE_COUNT) mode = JAM_MODE_FULL;
    return &lists[mode];
}

/* ---- PRNG propio (LCG): sin dependencias ---- */
static uint32_t s_seed = 0x1234567u;

static uint32_t jam_rand(void)
{
    s_seed = s_seed * 1664525u + 1013904223u;
    return s_seed;
}

static void jam_shuffle(uint8_t *a, size_t n)
{
    for (size_t i = n - 1; i > 0; i--) {
        size_t j = (size_t)(jam_rand() % (uint32_t)(i + 1));
        uint8_t t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

/* ---- estado por radio ---- */
typedef struct {
    nrf24_dev_t   *r;
    TaskHandle_t   task;
    jam_cfg_t      cfg;
    volatile bool  active, running;
    uint8_t        order[124];
} jam_slot_t;

static jam_slot_t s_jam[NRF_MAX_DEV];

static jam_slot_t *jam_slot_for(nrf24_dev_t *r)
{
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (s_jam[i].r == r) return &s_jam[i];
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (!s_jam[i].r) { s_jam[i].r = r; return &s_jam[i]; }
    return NULL;
}

static void jam_delay(uint32_t us)
{
    if (us >= 20000) {
        vTaskDelay(pdMS_TO_TICKS(us / 1000));
        nrf24_busy_delay_us(us % 1000);
    } else {
        nrf24_busy_delay_us(us);
    }
}

static void jam_task(void *arg)
{
    jam_slot_t *s = (jam_slot_t *)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        s->running = true;

        while (s->active) {
            const jam_channel_list_t *l = jam_mode_list(s->cfg.mode);
            size_t n = l->count;
            if (!n) break;

            if (s->cfg.fhss) {
                for (size_t i = 0; i < n; i++) s->order[i] = (uint8_t)i;
                jam_shuffle(s->order, n);
            }
            for (size_t i = 0; i < n; i++) {
                if (!s->active) break;
                if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;   /* cambio de config */
                size_t idx = s->cfg.fhss ? s->order[i] : i;
                nrf24_cw_hop(s->r, l->channels[idx]);
                jam_delay(s->cfg.dwell_us);
            }
        }

        nrf24_cw_stop(s->r);
        s->running = false;
        ESP_LOGI(TAG, "[%s] jammer detenido", nrf24_name(s->r));
    }
}

/* ---- API ---- */
esp_err_t jam_start(nrf24_dev_t *r, const jam_cfg_t *cfg)
{
    if (!nrf24_is_up(r)) return ESP_ERR_INVALID_STATE;

    jam_slot_t *s = jam_slot_for(r);
    if (!s) return ESP_ERR_NO_MEM;
    if (cfg) s->cfg = *cfg;
    if (s_seed == 0x1234567u) s_seed ^= (uint32_t)esp_timer_get_time();

    if (!s->task) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            jam_task, "jam", 4096, s,
            s->cfg.priority ? s->cfg.priority : 5,
            &s->task,
            (s->cfg.core >= 0) ? (BaseType_t)s->cfg.core : tskNO_AFFINITY);
        if (ok != pdPASS) { s->task = NULL; return ESP_ERR_NO_MEM; }
    }

    /* arranca el carrier de inmediato, sin esperar al scheduler */
    const jam_channel_list_t *l = jam_mode_list(s->cfg.mode);
    nrf24_cw_start(r, l->channels[0], s->cfg.pa);

    s->active = true;
    xTaskNotifyGive(s->task);

    ESP_LOGI(TAG, "[%s] JAM ON modo=%s canales=%d dwell=%luus pa=%ddBm %s",
             nrf24_name(r), l->name, (int)l->count, (unsigned long)s->cfg.dwell_us,
             -18 + 6 * (int)s->cfg.pa, s->cfg.fhss ? "FHSS" : "seq");
    return ESP_OK;
}

esp_err_t jam_stop(nrf24_dev_t *r)
{
    jam_slot_t *s = jam_slot_for(r);
    if (!s || !s->active) return ESP_OK;
    s->active = false;
    if (s->task) xTaskNotifyGive(s->task);
    for (int i = 0; i < 200 && s->running; i++) vTaskDelay(pdMS_TO_TICKS(2));
    nrf24_cw_stop(r);
    return ESP_OK;
}

bool jam_active(const nrf24_dev_t *r)
{
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (s_jam[i].r == r) return s_jam[i].active && s_jam[i].running;
    return false;
}

jam_mode_t jam_mode(const nrf24_dev_t *r)
{
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (s_jam[i].r == r) return s_jam[i].cfg.mode;
    return JAM_MODE_FULL;
}

uint32_t jam_dwell_us(const nrf24_dev_t *r)
{
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (s_jam[i].r == r) return s_jam[i].cfg.dwell_us;
    return 0;
}

int jam_active_count(void)
{
    int c = 0;
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (s_jam[i].r && s_jam[i].active && s_jam[i].running) c++;
    return c;
}

void jam_set_mode(nrf24_dev_t *r, jam_mode_t mode)
{
    jam_slot_t *s = jam_slot_for(r);
    if (!s || mode < 0 || mode >= JAM_MODE_COUNT || mode == s->cfg.mode) return;
    s->cfg.mode = mode;
    if (s->active && s->task) xTaskNotifyGive(s->task);
}

void jam_set_fhss(nrf24_dev_t *r, bool on)
{
    jam_slot_t *s = jam_slot_for(r);
    if (!s) return;
    s->cfg.fhss = on;
    if (s->active && s->task) xTaskNotifyGive(s->task);
}

void jam_set_dwell_us(nrf24_dev_t *r, uint32_t us)
{
    jam_slot_t *s = jam_slot_for(r);
    if (!s) return;
    s->cfg.dwell_us = (us < 50) ? 50 : us;
}

void jam_set_pa(nrf24_dev_t *r, nrf24_pa_t pa)
{
    jam_slot_t *s = jam_slot_for(r);
    if (!s) return;
    s->cfg.pa = pa;
    if (nrf24_cw_active(r)) nrf24_cw_start(r, nrf24_channel(r), pa);
}
