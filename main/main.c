/*
 * main.c - Consola unica para todo el firmware.
 *
 *   WiFi side  : rf_core (sniffer/injector) + karma (respondedor de probes)
 *   nRF side   : nrf24 dual + nrf24_dual (jammer CW)
 *   Indicador  : ws2812 (arcoiris / latido / pulso)
 *   Consola    : REPL por USB Serial/JTAG
 *
 * Reparto de cores:
 *   - Wi-Fi stack, rf_tx_task, rf_hop_task, karma_task  -> core 0
 *   - rf_sniffer_task                                    -> core 1
 *   - jam_task (por radio)                               -> core libre
 *   - ws2812_task                                        -> sin pinning
 *
 * Prerequisites que este archivo cubre ANTES de tocar nada:
 *   nvs_flash_init() + esp_netif_init() + esp_event_loop_create_default()
 *   (rf_core_start() hace esp_wifi_init/start por su cuenta)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "sdkconfig.h"

#include "src/rf_core.h"
#include "src/karma.h"
#include "src/nrf24.h"
#include "src/nrf24_dual.h"
#include "src/ws2812.h"

static const char *TAG = "app";

#define NUM_RADIOS 2
static nrf24_dev_t *s_r[NUM_RADIOS];

/* ==================================================================
 * Glue: puente entre rf_core (frames crudos) y karma / tabla de APs
 * ================================================================== */
static void sniffer_cb(const rf_frame_t *f, void *user)
{
    (void)user;
    rf_mgmt_info_t m;
    if (!rf_parse_mgmt(f, &m)) return;
    rf_ap_upsert(&m);      /* alimenta la tabla de APs */
    karma_feed(&m);        /* alimenta karma (si esta ON) */
}

/* ==================================================================
 * Consola
 * ================================================================== */
static int read_line(char *buf, size_t max)
{
    size_t n = 0;
    for (;;) {
        char c;
        if (read(STDIN_FILENO, &c, 1) <= 0) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }
        if (c == '\r' || c == '\n') { buf[n] = 0; printf("\r\n"); return (int)n; }
        if (c == 0x08 || c == 0x7F) { if (n) { n--; printf("\b \b"); } continue; }
        if (c < 0x20) continue;
        if (n + 1 < max) { buf[n++] = c; putchar(c); }
    }
}

static int split(char *s, char *argv[], int max)
{
    int argc = 0;
    char *save = NULL;
    for (char *t = strtok_r(s, " \t", &save); t && argc < max;
         t = strtok_r(NULL, " \t", &save))
        argv[argc++] = t;
    return argc;
}

static nrf24_dev_t *radio_at(int i)
{
    return (i >= 0 && i < NUM_RADIOS) ? s_r[i] : NULL;
}

/* ==================================================================
 * Comandos: nRF / jammer
 * ================================================================== */
static void cmd_radio(int argc, char **argv)
{
    if (argc > 1 && !strcmp(argv[1], "selftest")) {
        for (int i = 0; i < NUM_RADIOS; i++) {
            nrf24_dev_t *r = radio_at(i);
            printf("  r%d [%s]: %s\r\n", i, r ? nrf24_name(r) : "-",
                   r ? esp_err_to_name(nrf24_self_test(r)) : "no detectada");
        }
        return;
    }
    for (int i = 0; i < NUM_RADIOS; i++) {
        nrf24_dev_t *r = radio_at(i);
        if (!r) { printf("  r%d: NO DETECTADA\r\n", i); continue; }
        printf("  r%d [%s] ch=%u (%lu Hz) carrier=%d rpd=%d\r\n",
               i, nrf24_name(r), nrf24_channel(r),
               (unsigned long)nrf24_channel_to_freq(nrf24_channel(r)) * 1000UL,
               nrf24_cw_active(r) ? 1 : 0, nrf24_carrier_detect(r) ? 1 : 0);
        nrf24_dump(r);
    }
}

static void cmd_jam(int argc, char **argv)
{
    if (argc < 3) {
        printf("uso: jam <r> <on|off|mode|hop|dwell|pa|status>\r\n");
        return;
    }
    nrf24_dev_t *r = radio_at(atoi(argv[1]));
    if (!r) { printf("radio invalida (0 o 1)\r\n"); return; }

    if (!strcmp(argv[2], "on")) {
        jam_cfg_t c = JAM_CFG_DEFAULT();
        c.mode = jam_mode(r);
        c.core = CONFIG_JAM_CORE;
        esp_err_t e = jam_start(r, &c);
        if (e == ESP_OK) ws2812_notify(1);
        printf("[%s] jam: %s\r\n", nrf24_name(r), esp_err_to_name(e));
    } else if (!strcmp(argv[2], "off")) {
        jam_stop(r);
        printf("[%s] jam off\r\n", nrf24_name(r));
    } else if (!strcmp(argv[2], "mode")) {
        if (argc < 4) {
            for (int i = 0; i < JAM_MODE_COUNT; i++)
                printf("  %d = %-10s (%d canales)\r\n", i,
                       jam_mode_list((jam_mode_t)i)->name,
                       (int)jam_mode_list((jam_mode_t)i)->count);
            return;
        }
        int m = atoi(argv[3]);
        if (m < 0 || m >= JAM_MODE_COUNT) { printf("modo invalido\r\n"); return; }
        jam_set_mode(r, (jam_mode_t)m);
        printf("[%s] modo = %s\r\n", nrf24_name(r),
               jam_mode_list((jam_mode_t)m)->name);
    } else if (!strcmp(argv[2], "hop")) {
        bool on = (argc > 3) && atoi(argv[3]);
        jam_set_fhss(r, on);
        printf("[%s] hop = %s\r\n", nrf24_name(r), on ? "FHSS" : "secuencial");
    } else if (!strcmp(argv[2], "dwell")) {
        if (argc < 4) { printf("uso: jam <r> dwell <us>\r\n"); return; }
        jam_set_dwell_us(r, (uint32_t)atoi(argv[3]));
        printf("[%s] dwell = %lu us\r\n", nrf24_name(r),
               (unsigned long)jam_dwell_us(r));
    } else if (!strcmp(argv[2], "pa")) {
        if (argc < 4) { printf("uso: jam <r> pa <0..3>\r\n"); return; }
        int p = atoi(argv[3]);
        if (p < 0 || p > 3) { printf("pa invalido\r\n"); return; }
        jam_set_pa(r, (nrf24_pa_t)p);
        printf("[%s] pa = %d dBm\r\n", nrf24_name(r), -18 + 6 * p);
    } else if (!strcmp(argv[2], "status")) {
        printf("[%s] activo=%d modo=%s dwell=%luus ch=%u\r\n",
               nrf24_name(r), jam_active(r) ? 1 : 0,
               jam_mode_list(jam_mode(r))->name,
               (unsigned long)jam_dwell_us(r), nrf24_channel(r));
    }
}

static void cmd_jamall(int argc, char **argv)
{
    if (argc < 2) return;
    if (!strcmp(argv[1], "on")) {
        for (int i = 0; i < NUM_RADIOS; i++) {
            nrf24_dev_t *r = radio_at(i);
            if (!r) continue;
            jam_cfg_t c = JAM_CFG_DEFAULT();
            c.mode = jam_mode(r);
            c.core = (i == 0) ? CONFIG_JAM_CORE : -1;
            jam_start(r, &c);
        }
        int n = jam_active_count();
        if (n) ws2812_notify(n);
        printf("jamall ON (%d activas)\r\n", n);
    } else if (!strcmp(argv[1], "off")) {
        for (int i = 0; i < NUM_RADIOS; i++) {
            nrf24_dev_t *r = radio_at(i);
            if (r) jam_stop(r);
        }
        printf("jamall OFF\r\n");
    }
}

/* ==================================================================
 * Comandos: LED
 * ================================================================== */
static void cmd_led(int argc, char **argv)
{
    if (argc < 2) {
        printf("uso: led <rainbow|breathe|pulse|solid|off> [brillo]\r\n");
        return;
    }
    if (!ws2812_ready()) { printf("LED no disponible\r\n"); return; }

    if      (!strcmp(argv[1], "rainbow")) ws2812_set_mode(WS_MODE_RAINBOW);
    else if (!strcmp(argv[1], "breathe")) ws2812_set_mode(WS_MODE_BREATHE);
    else if (!strcmp(argv[1], "pulse"))   ws2812_set_mode(WS_MODE_PULSE);
    else if (!strcmp(argv[1], "off"))     ws2812_set_mode(WS_MODE_OFF);
    else if (!strcmp(argv[1], "solid")) {
        if (argc < 5) { printf("uso: led solid <r> <g> <b>\r\n"); return; }
        ws2812_set_rgb((uint8_t)atoi(argv[2]),
                       (uint8_t)atoi(argv[3]),
                       (uint8_t)atoi(argv[4]));
    } else {
        printf("modo desconocido\r\n");
        return;
    }
    if (argc >= 3 && atoi(argv[2]) > 0)
        ws2812_set_brightness((uint8_t)atoi(argv[2]));
    printf("LED ok\r\n");
}

/* ==================================================================
 * Comandos: WiFi (rf_core) y Karma
 * ================================================================== */
static void cmd_wifi(int argc, char **argv)
{
    if (argc < 2) { printf("uso: wifi start|stop|status\r\n"); return; }

    if (!strcmp(argv[1], "start")) {
        esp_err_t e = rf_core_start();
        if (e == ESP_OK) rf_sniffer_attach(sniffer_cb, NULL);
        printf("wifi start: %s\r\n", esp_err_to_name(e));
    } else if (!strcmp(argv[1], "stop")) {
        rf_core_stop();
        printf("wifi stop\r\n");
    } else if (!strcmp(argv[1], "status")) {
        rf_stats_t s = rf_stats();
        printf("  rx=%lu drop=%lu parsed=%lu tx_ok=%lu tx_fail=%lu hops=%lu ch=%u\r\n",
               (unsigned long)s.rx_total, (unsigned long)s.rx_dropped,
               (unsigned long)s.rx_parsed, (unsigned long)s.tx_ok,
               (unsigned long)s.tx_fail, (unsigned long)s.hopper_hops,
               rf_hop_channel());
    }
}

static void cmd_hop(int argc, char **argv)
{
    if (argc < 2) {
        printf("uso: hop <dwell_ms> [from] [to] | hop stop | hop ch <n>\r\n");
        return;
    }
    if (!strcmp(argv[1], "stop")) { rf_hop_stop(); printf("hop stop\r\n"); return; }

    if (!strcmp(argv[1], "ch")) {
        if (argc < 3) { printf("uso: hop ch <1..14>\r\n"); return; }
        int ch = atoi(argv[2]);
        if (ch < 1 || ch > 14) { printf("canal fuera de rango\r\n"); return; }
        rf_hop_stop();
        esp_err_t e = rf_set_channel((uint8_t)ch, WIFI_SECOND_CHAN_NONE);
        printf("ch=%d: %s\r\n", ch, esp_err_to_name(e));
        return;
    }

    uint32_t dwell = (uint32_t)atoi(argv[1]);
    uint8_t from = (argc > 2) ? (uint8_t)atoi(argv[2]) : 1;
    uint8_t to   = (argc > 3) ? (uint8_t)atoi(argv[3]) : 14;
    esp_err_t e = rf_hop_start(dwell, from, to);
    printf("hop %u..%u @%lums: %s\r\n", from, to, (unsigned long)dwell,
           esp_err_to_name(e));
}

static void cmd_karma(int argc, char **argv)
{
    if (argc < 2) { printf("uso: karma start|stop|status\r\n"); return; }

    if (!strcmp(argv[1], "start"))      { karma_start(); printf("karma ON\r\n"); }
    else if (!strcmp(argv[1], "stop"))  { karma_stop();  printf("karma OFF\r\n"); }
    else if (!strcmp(argv[1], "status")) {
        printf("  targets activos = %lu\r\n", (unsigned long)karma_target_count());
        karma_target_t *t = karma_targets();
        for (int i = 0; i < KARMA_MAX_TARGETS; i++) {
            if (!t[i].active) continue;
            printf("  [%d] '%s' ch=%u probes=%lu wpa2=%d\r\n",
                   i, t[i].ssid, t[i].chan,
                   (unsigned long)t[i].probes, t[i].wpa2);
        }
    }
}

/* ==================================================================
 * Comandos: tabla de APs y estadisticas
 * ================================================================== */
static bool ap_print_cb(const rf_ap_entry_t *e, void *user)
{
    int *n = (int *)user;
    printf("  %02X:%02X:%02X:%02X:%02X:%02X ch=%2u rssi=%4d %-5s '%s'\r\n",
           e->bssid[0], e->bssid[1], e->bssid[2],
           e->bssid[3], e->bssid[4], e->bssid[5],
           e->chan, e->rssi, e->wpa2 ? "WPA2" : "OPEN", e->ssid);
    (*n)++;
    return true;
}

static void cmd_scan(int argc, char **argv)
{
    (void)argc; (void)argv;
    int n = 0;
    printf("APs en tabla: %lu\r\n", (unsigned long)rf_ap_count());
    rf_ap_iterate(ap_print_cb, &n);
    printf("listados: %d\r\n", n);
}

static void cmd_clear(int argc, char **argv)
{
    (void)argc; (void)argv;
    rf_ap_clear();
    printf("tabla AP limpiada\r\n");
}

static void cmd_stats(int argc, char **argv)
{
    (void)argc; (void)argv;
    rf_stats_t s = rf_stats();
    printf("  rx=%lu drop=%lu parsed=%lu\r\n",
           (unsigned long)s.rx_total, (unsigned long)s.rx_dropped,
           (unsigned long)s.rx_parsed);
    printf("  tx_ok=%lu tx_fail=%lu hops=%lu ch=%u\r\n",
           (unsigned long)s.tx_ok, (unsigned long)s.tx_fail,
           (unsigned long)s.hopper_hops, rf_hop_channel());
    printf("  jam activas=%d  APs=%lu\r\n",
           jam_active_count(), (unsigned long)rf_ap_count());
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!s_r[i]) continue;
        printf("  r%d [%s] ch=%u carrier=%d\r\n",
               i, nrf24_name(s_r[i]), nrf24_channel(s_r[i]),
               nrf24_cw_active(s_r[i]) ? 1 : 0);
    }
}

static void help(void)
{
    printf("\r\n--- nRF24 / jammer ---\r\n");
    printf("  radio [selftest]              estado + dump de ambas radios\r\n");
    printf("  jam <r> on|off                arranca/para el carrier\r\n");
    printf("  jam <r> mode [n]              modo (sin n: lista los 10)\r\n");
    printf("  jam <r> hop [0|1]             secuencial / aleatorio\r\n");
    printf("  jam <r> dwell [us]            permanencia por canal\r\n");
    printf("  jam <r> pa [0..3]             -18 -12 -6 0 dBm\r\n");
    printf("  jam <r> status\r\n");
    printf("  jamall on|off                 ambas a la vez\r\n");
    printf("\r\n--- WiFi / Karma ---\r\n");
    printf("  wifi start|stop|status        sniffer + inyector\r\n");
    printf("  hop <dwell_ms> [from] [to]    hopper de canales\r\n");
    printf("  hop stop | hop ch <1..14>     parar / canal fijo\r\n");
    printf("  karma start|stop|status       respondedor de probes\r\n");
    printf("  scan                          lista APs en tabla\r\n");
    printf("  clear                         limpia tabla APs\r\n");
    printf("  stats                         contadores rf_core + radios\r\n");
    printf("\r\n--- LED ---\r\n");
    printf("  led <rainbow|breathe|pulse|solid|off> [brillo 0-255]\r\n");
    printf("  led solid <r> <g> <b>\r\n");
    printf("\r\n  help | ?\r\n");
}

/* ==================================================================
 * app_main
 * ================================================================== */
void app_main(void)
{
    /* ---- 1. NVS ---- */
    esp_err_t rv = nvs_flash_init();
    if (rv == ESP_ERR_NVS_NO_FREE_PAGES || rv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* ---- 2. Prerequisites que rf_core_start() asume hechos ---- */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* ---- 3. LED primero: prueba de humo del build ---- */
    if (ws2812_init() != ESP_OK)
        printf("WS2812 no disponible (sigue sin LED)\r\n");

    /* ---- 4. Radios nRF ---- */
    nrf24_cfg_t c[NUM_RADIOS];

    c[0] = (nrf24_cfg_t)NRF24_CFG_DEFAULT();
    c[0].name = "r1";  c[0].host = SPI2_HOST;
    c[0].ce   = (gpio_num_t)CONFIG_NRF1_CE_GPIO;
    c[0].cs   = (gpio_num_t)CONFIG_NRF1_CS_GPIO;
    c[0].sck  = (gpio_num_t)CONFIG_NRF1_SCK_GPIO;
    c[0].mosi = (gpio_num_t)CONFIG_NRF1_MOSI_GPIO;
    c[0].miso = (gpio_num_t)CONFIG_NRF1_MISO_GPIO;
    c[0].irq  = (gpio_num_t)CONFIG_NRF1_IRQ_GPIO;

    c[1] = (nrf24_cfg_t)NRF24_CFG_DEFAULT();
    c[1].name = "r2";  c[1].host = SPI3_HOST;
    c[1].ce   = (gpio_num_t)CONFIG_NRF2_CE_GPIO;
    c[1].cs   = (gpio_num_t)CONFIG_NRF2_CS_GPIO;
    c[1].sck  = (gpio_num_t)CONFIG_NRF2_SCK_GPIO;
    c[1].mosi = (gpio_num_t)CONFIG_NRF2_MOSI_GPIO;
    c[1].miso = (gpio_num_t)CONFIG_NRF2_MISO_GPIO;
    c[1].irq  = (gpio_num_t)CONFIG_NRF2_IRQ_GPIO;

    printf("\r\n=== esp32s3o-finware / ESP-IDF %s ===\r\n", IDF_VER);
    for (int i = 0; i < NUM_RADIOS; i++) {
        s_r[i] = nrf24_create(&c[i]);
        if (!s_r[i]) {
            printf("r%d NO DETECTADA (CE=%d CS=%d SCK=%d MOSI=%d MISO=%d IRQ=%d)\r\n",
                   i, (int)c[i].ce, (int)c[i].cs, (int)c[i].sck,
                   (int)c[i].mosi, (int)c[i].miso, (int)c[i].irq);
        }
    }

    /* ---- 5. WiFi side: rf_core + karma ---- */
    ESP_ERROR_CHECK(rf_core_init());
    ESP_ERROR_CHECK(rf_ap_table_init());

    esp_err_t we = rf_core_start();
    if (we == ESP_OK) {
        rf_sniffer_attach(sniffer_cb, NULL);
        printf("rf_core: OK (sniffer + injector arriba)\r\n");
    } else {
        printf("rf_core_start: %s\r\n", esp_err_to_name(we));
    }

    if (karma_init() == ESP_OK)
        printf("karma: listo (comando 'karma start' para activar)\r\n");

    /* ---- 6. REPL ---- */
    help();
    printf("\r\nnrf> ");
    fflush(stdout);

    static char line[300];
    static char *argv[24];

    for (;;) {
        if (read_line(line, sizeof(line)) <= 0) continue;
        int argc = split(line, argv, 24);
        if (!argc) { printf("nrf> "); fflush(stdout); continue; }

        if      (!strcmp(argv[0], "help")  || !strcmp(argv[0], "?")) help();
        else if (!strcmp(argv[0], "radio"))  cmd_radio(argc, argv);
        else if (!strcmp(argv[0], "jam"))    cmd_jam(argc, argv);
        else if (!strcmp(argv[0], "jamall")) cmd_jamall(argc, argv);
        else if (!strcmp(argv[0], "led"))    cmd_led(argc, argv);
        else if (!strcmp(argv[0], "wifi"))   cmd_wifi(argc, argv);
        else if (!strcmp(argv[0], "hop"))    cmd_hop(argc, argv);
        else if (!strcmp(argv[0], "karma"))  cmd_karma(argc, argv);
        else if (!strcmp(argv[0], "scan"))   cmd_scan(argc, argv);
        else if (!strcmp(argv[0], "clear"))  cmd_clear(argc, argv);
        else if (!strcmp(argv[0], "stats"))  cmd_stats(argc, argv);
        else printf("desconocido: %s\r\n", argv[0]);

        printf("nrf> ");
        fflush(stdout);
    }
}
