/*
 * rf_sniffer.c — parseo 802.11 de bajo coste + tabla de APs
 *
 * Sin malloc, sin String, sin Arduino. Todo son offsets y structs fijas.
 * El hash de la tabla es abierto con linear probing: O(1) amortizado y
 * sin listas enlazadas que fragmenten la RAM interna.
 */
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"

#include "rf_core.h"

static const char *TAG = "rf_sniff";

static rf_ap_entry_t *s_ap;
static uint32_t       s_ap_count;

/* ------------------------------------------------------------------ */
static inline uint16_t rd_le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* Offset donde empiezan los tagged parameters segun subtipo. */
static uint16_t tags_start(uint8_t type, uint8_t subtype)
{
    if (type != RF_MGMT) return 0;
    switch (subtype) {
        case RF_SUB_BEACON:
        case RF_SUB_PROBE_RESP:   return 24u + 12u;  /* fixed params  */
        case RF_SUB_ASSOC_RESP:
        case RF_SUB_REASSOC_RESP: return 24u + 6u;
        case RF_SUB_ASSOC_REQ:
        case RF_SUB_REASSOC_REQ:
        case RF_SUB_PROBE_REQ:    return 24u;
        default:                  return 0;          /* sin tags utiles */
    }
}

/* ------------------------------------------------------------------ */
bool rf_parse_mgmt(const rf_frame_t *f, rf_mgmt_info_t *out)
{
    if (f == NULL || out == NULL) return false;
    if (f->len < 24u) return false;

    const uint8_t *p = f->payload;
    uint16_t fc = rd_le16(p);

    out->type            = (uint8_t)((fc >> 2) & 0x3u);
    out->subtype         = (uint8_t)((fc >> 4) & 0xFu);
    out->protected_frame = (uint8_t)((fc & 0x4000u) ? 1u : 0u);

    if (out->type != RF_MGMT) return false;

    memcpy(out->receiver,    p + 4,  6);
    memcpy(out->transmitter, p + 10, 6);
    memcpy(out->bssid,       p + 16, 6);

    out->chan     = f->chan;
    out->rssi     = f->rssi;
    out->seq      = (uint16_t)((rd_le16(p + 22) >> 4) & 0x0FFFu);
    out->ssid[0]  = '\0';
    out->ssid_len = 0;

    uint16_t off = tags_start(out->type, out->subtype);
    if (off == 0) return true;

    while ((uint32_t)off + 2u <= f->len) {
        uint8_t id = p[off];
        uint8_t ln = p[off + 1u];
        if ((uint32_t)off + 2u + ln > f->len) break;   /* tag corrupto */

        if (id == 0u && ln <= 32u) {                   /* SSID */
            out->ssid_len = ln;
            memcpy(out->ssid, p + off + 2u, ln);
            out->ssid[ln] = '\0';
            break;                                     /* el 1o gana */
        }
        if (id == 48u) {                               /* RSN -> WPA2 */
            out->protected_frame = 1u;
        }
        off = (uint16_t)(off + 2u + ln);
    }
    return true;
}

/* ------------------------------------------------------------------ */
static inline uint32_t ap_hash(const uint8_t *bssid)
{
    uint32_t h = 2166136261u;                           /* FNV-1a */
    for (int i = 0; i < 6; i++) {
        h ^= bssid[i];
        h *= 16777619u;
    }
    return h & (RF_AP_TABLE_MAX - 1u);
}

esp_err_t rf_ap_table_init(void)
{
    if (s_ap != NULL) return ESP_OK;
    s_ap = heap_caps_calloc(1, (size_t)RF_AP_TABLE_MAX * sizeof(rf_ap_entry_t),
                            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (s_ap == NULL) return ESP_ERR_NO_MEM;
    s_ap_count = 0;
    ESP_LOGI(TAG, "tabla AP: %u entradas en RAM interna", (unsigned)RF_AP_TABLE_MAX);
    return ESP_OK;
}

rf_ap_entry_t *rf_ap_upsert(const rf_mgmt_info_t *m)
{
    if (s_ap == NULL || m == NULL) return NULL;

    uint32_t i = ap_hash(m->bssid);
    int64_t  now = esp_timer_get_time();

    for (uint32_t n = 0; n < RF_AP_TABLE_MAX; n++) {
        rf_ap_entry_t *e = &s_ap[(i + n) & (RF_AP_TABLE_MAX - 1u)];

        if (!e->used) {                                  /* hueco libre */
            memcpy(e->bssid, m->bssid, 6);
            e->ssid_len  = m->ssid_len;
            memcpy(e->ssid, m->ssid, (size_t)m->ssid_len + 1u);
            e->chan      = m->chan;
            e->rssi      = m->rssi;
            e->wpa2      = m->protected_frame;
            e->beacons   = 1;
            e->clients   = 0;
            e->used      = true;
            e->first_seen_us = now;
            e->last_seen_us  = now;
            s_ap_count++;
            return e;
        }

        if (memcmp(e->bssid, m->bssid, 6) == 0) {        /* ya existe */
            e->beacons++;
            e->chan = m->chan;
            if (m->rssi > e->rssi) e->rssi = m->rssi;
            if (m->protected_frame) e->wpa2 = 1;
            if (e->ssid_len == 0 && m->ssid_len > 0) {
                e->ssid_len = m->ssid_len;
                memcpy(e->ssid, m->ssid, (size_t)m->ssid_len + 1u);
            }
            e->last_seen_us = now;
            return e;
        }
    }
    return NULL;                                         /* tabla llena */
}

void rf_ap_iterate(bool (*cb)(const rf_ap_entry_t *e, void *user), void *user)
{
    if (s_ap == NULL || cb == NULL) return;
    for (uint32_t i = 0; i < RF_AP_TABLE_MAX; i++) {
        if (!s_ap[i].used) continue;
        if (!cb(&s_ap[i], user)) break;
    }
}

uint32_t rf_ap_count(void)
{
    return s_ap_count;
}

void rf_ap_clear(void)
{
    if (s_ap == NULL) return;
    memset(s_ap, 0, (size_t)RF_AP_TABLE_MAX * sizeof(rf_ap_entry_t));
    s_ap_count = 0;
}
