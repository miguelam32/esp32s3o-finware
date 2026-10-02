/*
 * rf_core.c — nucleo de radio, ESP-IDF 6.0.1, ESP32-S3
 *
 * Reglas inquebrantables:
 *   1. La callback NUNCA reserva memoria, NUNCA loguea, NUNCA llama al
 *      usuario y NUNCA llama a la API de FreeRTOS. Cero riesgo de crash
 *      por flash-cache-off y cero coste de cambio de contexto.
 *   2. Si el ring esta lleno se descarta la trama: nunca se bloquea la radio.
 *   3. Sincronizacion SPSC con indices `volatile` + barrera `memw`.
 *      NO usamos <stdatomic.h> a proposito: en Xtensa los RMW de 32 bits
 *      pueden acabar en llamadas a libgcc que viven en flash, y eso es
 *      exactamente lo que no puede haber en una funcion IRAM.
 *      Como cada indice lo escribe UN SOLO contexto (head: la callback;
 *      tail: el consumidor), `volatile` + barrera es correcto y seguro.
 *   4. Ningun contador de estadisticas se escribe desde dos contextos.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "xtensa_api.h"        /* xthal_get_ccount() */

#include "rf_core.h"

static const char *TAG = "rf_core";

/* Barrera de escritura: garantiza que las stores previas son visibles
 * antes de que el indice se publique. `memw` drena el write buffer. */
#define RF_WMB() __asm__ __volatile__("memw" ::: "memory")

/* ------------------------------------------------------------------ */
static rf_frame_t   *s_ring;              /* ring en PSRAM             */
static volatile uint32_t s_head;          /* productor  (callback)     */
static volatile uint32_t s_tail;          /* consumidor (tarea)        */
static volatile int   s_running;
static volatile int   s_hop_run;

static QueueHandle_t s_txq;
static TaskHandle_t  s_sniff_task;
static TaskHandle_t  s_tx_task;
static TaskHandle_t  s_hop_task;

static rf_frame_cb_t s_cb;
static void         *s_cb_user;

static volatile uint32_t s_rx_total;
static volatile uint32_t s_rx_dropped;
static volatile uint32_t s_rx_parsed;
static volatile uint32_t s_tx_ok;
static volatile uint32_t s_tx_fail;
static volatile uint32_t s_hops;
static volatile int      s_hop_cur;

static uint32_t s_hop_dwell_ms = 120;
static uint8_t  s_hop_from     = 1;
static uint8_t  s_hop_to       = 14;

/* ------------------------------------------------------------------ */
/* Copia segura para IRAM: dst siempre alineado (rf_frame_t aligned32) */
/* ------------------------------------------------------------------ */
IRAM_ATTR static inline void rf_copy(uint8_t *dst, const uint8_t *src, uint32_t n)
{
    while (n && ((uintptr_t)src & 3u)) { *dst++ = *src++; n--; }

    uint32_t       *d = (uint32_t *)(void *)dst;
    const uint32_t *s = (const uint32_t *)(const void *)src;
    while (n >= 4u) { *d++ = *s++; n -= 4u; }

    uint8_t       *db = (uint8_t *)(void *)d;
    const uint8_t *sb = (const uint8_t *)(const void *)s;
    while (n--) { *db++ = *sb++; }
}

/* ------------------------------------------------------------------ */
/* CAMINO CALIENTE: recepcion promiscua. Nada de FreeRTOS aqui dentro. */
/* ------------------------------------------------------------------ */
IRAM_ATTR static void rf_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    if (buf == NULL) return;
    if (s_running == 0) return;

    const wifi_promiscuous_pkt_t *pkt = (const wifi_promiscuous_pkt_t *)buf;
    const wifi_pkt_rx_ctrl_t     *c   = &pkt->rx_ctrl;

    uint32_t len = c->sig_len;
    if (len == 0u || len > RF_FRAME_MAX) {
        s_rx_dropped++;
        return;
    }

    uint32_t head = s_head;
    uint32_t tail = s_tail;
    uint32_t next = (head + 1u) % RF_RING_SLOTS;

    if (next == tail) {                        /* ring lleno -> fuera */
        s_rx_dropped++;
        return;
    }

    rf_frame_t *f = &s_ring[head];
    f->ccount = xthal_get_ccount();
    f->len    = len;
    f->chan   = (uint8_t)c->channel;
    f->rssi   = (int8_t)c->rssi;
    f->ptype  = (uint8_t)type;
    f->rate   = c->rate;
    rf_copy(f->payload, pkt->payload, len);

    s_head = next;                 /* publica SOLO despues de la copia */
    RF_WMB();

    s_rx_total++;
}

/* ------------------------------------------------------------------ */
/* Consumidor: core 1, prioridad alta                                 */
/* ------------------------------------------------------------------ */
static void rf_sniffer_task(void *arg)
{
    (void)arg;
    uint16_t seq  = 0;
    uint32_t idle = 0;

    for (;;) {
        uint32_t tail = s_tail;
        uint32_t head = s_head;

        if (tail == head) {
            /* Spin corto (latencia minima con trafico), y cada 64 vueltas
             * dormimos 1 ms para que el idle task corra y el Task Watchdog
             * no salte. Sin notificaciones: la callback no toca FreeRTOS. */
            if (++idle >= 64u) {
                vTaskDelay(pdMS_TO_TICKS(1));
                idle = 0;
            }
            continue;
        }
        idle = 0;

        rf_frame_t *f = &s_ring[tail];
        f->seq   = seq++;
        f->ts_us = esp_timer_get_time();

        if (s_cb != NULL) s_cb(f, s_cb_user);

        s_rx_parsed++;
        s_tail = (tail + 1u) % RF_RING_SLOTS;
        RF_WMB();
    }
}

/* ------------------------------------------------------------------ */
/* Inyeccion: core 0 (mismo core que el stack Wi-Fi)                  */
/* ------------------------------------------------------------------ */
typedef struct {
    uint16_t len;
    uint8_t  data[RF_FRAME_MAX];
} rf_tx_item_t;

static void rf_tx_task(void *arg)
{
    (void)arg;
    rf_tx_item_t item;

    for (;;) {
        if (xQueueReceive(s_txq, &item, portMAX_DELAY) != pdTRUE) continue;

        esp_err_t e = rf_tx_now(item.data, item.len);
        if (e == ESP_OK) {
            s_tx_ok++;
        } else {
            s_tx_fail++;
            ESP_LOGD(TAG, "tx fail: %s", esp_err_to_name(e));
        }
    }
}

/* ------------------------------------------------------------------ */
/* Salto de canal: tarea propia, independiente del resto              */
/* ------------------------------------------------------------------ */
static void rf_hop_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_hop_run == 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        for (uint8_t ch = s_hop_from; ch <= s_hop_to; ch++) {
            if (s_hop_run == 0) break;
            if (ch < 1u || ch > 14u) continue;

            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            s_hop_cur = (int)ch;
            s_hops++;

            vTaskDelay(pdMS_TO_TICKS(s_hop_dwell_ms));
        }
    }
}

/* ------------------------------------------------------------------ */
esp_err_t rf_core_init(void)
{
    if (s_ring != NULL) return ESP_OK;

    size_t psram = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t need  = (size_t)RF_RING_SLOTS * sizeof(rf_frame_t);

    ESP_LOGI(TAG, "PSRAM: %u KB / ring necesita %u KB",
             (unsigned)(psram / 1024), (unsigned)(need / 1024));

    if (psram < need) {
        ESP_LOGE(TAG, "PSRAM insuficiente: baja RF_RING_SLOTS en rf_core.h");
        return ESP_ERR_NO_MEM;
    }

    s_ring = heap_caps_calloc(1, need, MALLOC_CAP_SPIRAM);
    if (s_ring == NULL) {
        ESP_LOGE(TAG, "heap_caps_calloc(SPIRAM) fallo");
        return ESP_ERR_NO_MEM;
    }

    s_head = 0;
    s_tail = 0;
    s_running = 0;
    s_hop_run = 0;
    s_hop_cur = 1;
    s_rx_total = s_rx_dropped = s_rx_parsed = 0;
    s_tx_ok = s_tx_fail = s_hops = 0;

    s_txq = xQueueCreate(RF_TXQ_DEPTH, sizeof(rf_tx_item_t));
    if (s_txq == NULL) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "ring listo: %u slots x %u B", (unsigned)RF_RING_SLOTS,
             (unsigned)sizeof(rf_frame_t));
    return ESP_OK;
}

esp_err_t rf_core_start(void)
{
    if (s_ring == NULL) return ESP_ERR_INVALID_STATE;
    if (s_running) return ESP_OK;

    /* AP fantasma: solo necesitamos que la interfaz este "up" para poder
     * inyectar tramas crudas con esp_wifi_80211_tx().                    */
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t e = esp_wifi_init(&init);
    if (e != ESP_OK) return e;

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    wifi_config_t ap;
    memset(&ap, 0, sizeof(ap));
    memcpy(ap.ap.ssid, "rf-core", 8);
    ap.ap.ssid_len       = 7;
    ap.ap.channel        = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode       = WIFI_AUTH_OPEN;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));

    ESP_ERROR_CHECK(esp_wifi_start());
    (void)esp_wifi_set_max_tx_power(78);   /* 19.5 dBm (limitado por HW) */

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(rf_rx_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    s_running = 1;

    /* core 1 = parseo (nunca compite con el stack Wi-Fi) */
    if (xTaskCreatePinnedToCore(rf_sniffer_task, "rf_sniff", 4096, NULL, 10,
                                &s_sniff_task, 1) != pdPASS) return ESP_ERR_NO_MEM;
    /* core 0 = inyeccion (localidad de cache con esp_wifi_80211_tx) */
    if (xTaskCreatePinnedToCore(rf_tx_task, "rf_tx", 4096, NULL, 9,
                                &s_tx_task, 0) != pdPASS) return ESP_ERR_NO_MEM;
    if (xTaskCreatePinnedToCore(rf_hop_task, "rf_hop", 3072, NULL, 4,
                                &s_hop_task, 0) != pdPASS) return ESP_ERR_NO_MEM;

    ESP_LOGI(TAG, "arrancado: sniff=core1 tx=core0 hop=core0");
    return ESP_OK;
}

esp_err_t rf_core_stop(void)
{
    s_hop_run = 0;
    s_running = 0;
    esp_wifi_set_promiscuous(false);
    esp_wifi_stop();
    esp_wifi_deinit();
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t rf_set_channel(uint8_t primary, wifi_second_chan_t second)
{
    if (primary < 1u || primary > 14u) return ESP_ERR_INVALID_ARG;
    return esp_wifi_set_channel(primary, second);
}

esp_err_t rf_promisc_enable(bool en)
{
    return esp_wifi_set_promiscuous(en);
}

esp_err_t rf_promisc_filter(uint32_t mask)
{
    wifi_promiscuous_filter_t f = { .filter_mask = mask };
    return esp_wifi_set_promiscuous_filter(&f);
}

esp_err_t rf_hop_start(uint32_t dwell_ms, uint8_t from, uint8_t to)
{
    if (from < 1u)  from = 1u;
    if (to   > 14u) to   = 14u;
    if (from > to)  return ESP_ERR_INVALID_ARG;

    s_hop_from     = from;
    s_hop_to       = to;
    s_hop_dwell_ms = dwell_ms ? dwell_ms : 1u;
    s_hop_run      = 1;
    return ESP_OK;
}

esp_err_t rf_hop_stop(void)
{
    s_hop_run = 0;
    return ESP_OK;
}

uint8_t rf_hop_channel(void)
{
    return (uint8_t)s_hop_cur;
}

void rf_sniffer_attach(rf_frame_cb_t cb, void *user)
{
    s_cb      = cb;
    s_cb_user = user;
}

/* ------------------------------------------------------------------ */
esp_err_t rf_tx_now(const uint8_t *frame, size_t len)
{
    if (frame == NULL || len < 4u || len > RF_FRAME_MAX) return ESP_ERR_INVALID_ARG;

    esp_err_t e = esp_wifi_80211_tx(WIFI_IF_AP, frame, (int)len, false);
    if (e != ESP_OK) {
        /* Algunos firmwares solo aceptan STA como interfaz de inyeccion. */
        e = esp_wifi_80211_tx(WIFI_IF_STA, frame, (int)len, false);
    }
    return e;
}

esp_err_t rf_tx_queue(const uint8_t *frame, size_t len, TickType_t wait)
{
    if (frame == NULL || len < 4u || len > RF_FRAME_MAX) return ESP_ERR_INVALID_ARG;
    if (s_txq == NULL) return ESP_ERR_INVALID_STATE;

    rf_tx_item_t item;
    item.len = (uint16_t)len;
    memcpy(item.data, frame, len);

    if (xQueueSend(s_txq, &item, wait) != pdTRUE) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
uint64_t rf_ccount_to_us(uint32_t a, uint32_t b)
{
    uint32_t d = b - a;                     /* resta sin signo: wrap-safe */
    return (uint64_t)d / (uint64_t)RF_CPU_MHZ;
}

uint16_t rf_frame_seq(const uint8_t *frame)
{
    if (frame == NULL) return 0;
    uint16_t s = (uint16_t)((uint16_t)frame[22] | ((uint16_t)frame[23] << 8));
    return (uint16_t)((s >> 4) & 0x0FFFu);
}

rf_stats_t rf_stats(void)
{
    rf_stats_t s;
    s.rx_total    = s_rx_total;
    s.rx_dropped  = s_rx_dropped;
    s.rx_parsed   = s_rx_parsed;
    s.tx_ok       = s_tx_ok;
    s.tx_fail     = s_tx_fail;
    s.hopper_hops = s_hops;
    return s;
}
