/*
 * rf_core.h — nucleo de radio de esp32s3o-finware
 *
 *   1. La callback promiscua NO parsea nada. Solo copia al siguiente slot
 *      libre de un ring en PSRAM y publica un indice.
 *   2. Un consumidor dedicado (core 1) parsea a su ritmo. Lock-free.
 *   3. La inyeccion tiene su propia tarea en core 0, junto al stack Wi-Fi.
 *   4. El salto de canal es otra tarea: escanear y atacar a la vez.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "freertos/FreeRTOS.h"      /* TickType_t */

#ifdef __cplusplus
extern "C" {
#endif

#define RF_FRAME_MAX     2344u
#define RF_RING_SLOTS    256u
#define RF_TXQ_DEPTH     16u
#define RF_AP_TABLE_MAX  256u
#define RF_SSID_MAX      33u

#if !defined(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) || (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ == 0)
#define RF_CPU_MHZ 240u
#else
#define RF_CPU_MHZ ((uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ)
#endif

typedef struct {
    uint64_t ts_us;
    uint32_t ccount;
    uint32_t len;
    uint32_t rate;
    uint8_t  chan;
    int8_t   rssi;
    uint8_t  ptype;
    uint8_t  _pad0;
    uint16_t _pad1;
    uint16_t seq;
    uint8_t  payload[RF_FRAME_MAX];
} __attribute__((aligned(32))) rf_frame_t;

#define RF_MGMT 0u
#define RF_CTRL 1u
#define RF_DATA 2u
#define RF_EXT  3u

#define RF_SUB_ASSOC_REQ     0x00u
#define RF_SUB_ASSOC_RESP    0x01u
#define RF_SUB_REASSOC_REQ   0x02u
#define RF_SUB_REASSOC_RESP  0x03u
#define RF_SUB_PROBE_REQ     0x04u
#define RF_SUB_PROBE_RESP    0x05u
#define RF_SUB_BEACON        0x08u
#define RF_SUB_ATIM          0x09u
#define RF_SUB_DISASSOC      0x0Au
#define RF_SUB_AUTH          0x0Bu
#define RF_SUB_DEAUTH        0x0Cu

typedef struct {
    uint8_t  bssid[6];
    uint8_t  transmitter[6];
    uint8_t  receiver[6];
    uint8_t  type;
    uint8_t  subtype;
    uint8_t  chan;
    int8_t   rssi;
    uint16_t seq;
    uint8_t  protected_frame;
    uint8_t  ssid_len;
    char     ssid[RF_SSID_MAX];
} rf_mgmt_info_t;

/* Tabla de APs: hash abierto en PSRAM, sin allocations en caliente. */
typedef struct {
    uint8_t  bssid[6];
    uint8_t  chan;
    int8_t   rssi;
    uint8_t  ssid_len;
    char     ssid[RF_SSID_MAX];      /* <-- faltaba: lo usa rf_sniffer.c */
    uint8_t  wpa2;
    uint32_t beacons;
    uint32_t clients;
    int64_t  first_seen_us;
    int64_t  last_seen_us;
    bool     used;
} rf_ap_entry_t;

esp_err_t rf_core_init(void);
esp_err_t rf_core_start(void);
esp_err_t rf_core_stop(void);

esp_err_t rf_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t rf_promisc_enable(bool en);
esp_err_t rf_promisc_filter(uint32_t mask);
esp_err_t rf_hop_start(uint32_t dwell_ms, uint8_t from, uint8_t to);
esp_err_t rf_hop_stop(void);
uint8_t  rf_hop_channel(void);

typedef void (*rf_frame_cb_t)(const rf_frame_t *f, void *user);
void     rf_sniffer_attach(rf_frame_cb_t cb, void *user);

esp_err_t rf_tx_now(const uint8_t *frame, size_t len);
esp_err_t rf_tx_queue(const uint8_t *frame, size_t len, TickType_t wait);

esp_err_t rf_deauth(const uint8_t *bssid, const uint8_t *sta, uint8_t reason);
esp_err_t rf_deauth_pair(const uint8_t *bssid, const uint8_t *sta, uint8_t reason);
esp_err_t rf_deauth_bcast(const uint8_t *bssid, uint8_t reason);
esp_err_t rf_disassoc(const uint8_t *bssid, const uint8_t *sta, uint8_t reason);
esp_err_t rf_auth_open(const uint8_t *bssid, const uint8_t *sta);

size_t   rf_beacon_build(uint8_t *out, size_t out_max, const char *ssid,
                         const uint8_t *bssid, uint8_t chan, bool wpa2);
esp_err_t rf_beacon_tx(const char *ssid, const uint8_t *bssid, uint8_t chan, bool wpa2);

bool     rf_parse_mgmt(const rf_frame_t *f, rf_mgmt_info_t *out);
uint16_t rf_frame_seq(const uint8_t *frame);
uint64_t rf_ccount_to_us(uint32_t a, uint32_t b);

esp_err_t     rf_ap_table_init(void);
rf_ap_entry_t *rf_ap_upsert(const rf_mgmt_info_t *m);
void          rf_ap_iterate(bool (*cb)(const rf_ap_entry_t *e, void *user), void *user);
uint32_t      rf_ap_count(void);
void          rf_ap_clear(void);

typedef struct {
    uint32_t rx_total;
    uint32_t rx_dropped;
    uint32_t rx_parsed;
    uint32_t tx_ok;
    uint32_t tx_fail;
    uint32_t hopper_hops;
} rf_stats_t;

rf_stats_t rf_stats(void);

#ifdef __cplusplus
}
#endif
