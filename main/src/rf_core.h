/*
 * rf_core.h — nucleo de radio de esp32s3o-finware
 *
 *   1. La callback promiscua NO parsea nada. Solo copia al siguiente slot
 *      libre de un ring en PSRAM y publica un indice.
 *   2. Un consumidor dedicado (core 1, prioridad alta) parsea a su ritmo.
 *      Productor/consumidor lock-free: ni un mutex en el camino caliente.
 *   3. La inyeccion tiene su propia tarea en core 0 (mismo core que el
 *      stack Wi-Fi) para no cruzar cores al llamar esp_wifi_80211_tx().
 *   4. El salto de canal es una tarea independiente: puedes escanear y
 *      atacar a la vez, que es justo lo que Bruce no hace bien.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_wifi_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Tunables                                                            */
/* ------------------------------------------------------------------ */
#define RF_FRAME_MAX     2344u          /* maximo que guardamos por trama */
#define RF_RING_SLOTS    256u           /* slots del ring (PSRAM)         */
#define RF_TXQ_DEPTH     16u            /* frames encolados para inyeccion*/
#define RF_AP_TABLE_MAX  256u           /* potencia de 2 (hash abierto)   */
#define RF_SSID_MAX      33u

#if !defined(CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ) || (CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ == 0)
#define RF_CPU_MHZ 240u
#else
#define RF_CPU_MHZ ((uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ)
#endif

/* ------------------------------------------------------------------ */
/* Registro de trama capturada (1 slot del ring, alineado a cache line) */
/* ------------------------------------------------------------------ */
typedef struct {
    uint64_t ts_us;        /* timestamp de parseo (consumer)            */
    uint32_t ccount;       /* CCOUNT crudo en el momento de captura     */
    uint32_t len;          /* bytes validos en payload[]               */
    uint32_t rate;         /* rate reportado por el controlador         */
    uint8_t  chan;
    int8_t   rssi;
    uint8_t  ptype;        /* wifi_promiscuous_pkt_type_t               */
    uint8_t  _pad0;
    uint16_t _pad1;
    uint16_t seq;          /* indice monotono de captura                */
    uint8_t  payload[RF_FRAME_MAX];
} __attribute__((aligned(32))) rf_frame_t;

/* ------------------------------------------------------------------ */
/* Tipos/subtipos 802.11                                              */
/* ------------------------------------------------------------------ */
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

/* ------------------------------------------------------------------ */
/* Frame de management ya parseado (sin allocations)                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  bssid[6];
    uint8_t  transmitter[6];
    uint8_t  receiver[6];
    uint8_t  type;
    uint8_t  subtype;
    uint8_t  chan;
    int8_t   rssi;
    uint16_t seq;
    uint8_t  protected_frame;   /* bit Protected / RSN IE presente */
    uint8_t  ssid_len;
    char     ssid[RF_SSID_MAX];
} rf_mgmt_info_t;

/* ------------------------------------------------------------------ */
/* Tabla de APs (hash abierto, linear probing)                        */
/* ------------------------------------------------------------------ */
typedef struct {
    uint8_t  bssid[6];
    uint8_t  chan;
    int8_t   rssi;
    uint8_t  ssid_len;
    uint8_t  wpa2;              /* RSN IE visto en algun beacon */
    uint32_t beacons;
    uint32_t clients;           /* data frames vistos con este BSSID */
    int64_t  first_seen_us;
    int64_t  last_seen_us;
    bool     used;
} rf_ap_entry_t;

/* ------------------------------------------------------------------ */
/* API                                                                 */
/* ------------------------------------------------------------------ */
esp_err_t rf_core_init(void);      /* reserva ring + colas en PSRAM  */
esp_err_t rf_core_start(void);     /* levanta Wi-Fi, promiscuo, tareas */
esp_err_t rf_core_stop(void);

esp_err_t rf_set_channel(uint8_t primary, wifi_second_chan_t second);
esp_err_t rf_promisc_enable(bool en);
esp_err_t rf_promisc_filter(uint32_t mask);   /* WIFI_PROMIS_FILTER_MASK_* */
esp_err_t rf_hop_start(uint32_t dwell_ms, uint8_t from, uint8_t to);
esp_err_t rf_hop_stop(void);
uint8_t  rf_hop_channel(void);

typedef void (*rf_frame_cb_t)(const rf_frame_t *f, void *user);
void     rf_sniffer_attach(rf_frame_cb_t cb, void *user);

esp_err_t rf_tx_now(const uint8_t *frame, size_t len);                    /* bloqueante */
esp_err_t rf_tx_queue(const uint8_t *frame, size_t len, TickType_t wait); /* async     */

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
uint64_t rf_ccount_to_us(uint32_t a, uint32_t b);   /* delta wrap-safe */

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
