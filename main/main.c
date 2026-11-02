/*
 * Test de 6x nRF24L01+ en ESP32-S3 N16R8 (ESP-IDF 5.x / 6.x)
 *
 * Fase 1 (SPI): por cada radio y por cada velocidad (16, 10, 8, 4, 1 MHz)
 *               escribe/lee de vuelta registros con valores aleatorios.
 * Fase 2 (RF) : todos contra todos (30 enlaces), -18 dBm.
 * LED GPIO48  : amarillo = probando
 *               arcoiris = los 6 radios pasan SPI y TX/RX
 *               parpadeos verdes = radios OK, rojos = radios con falla
 *
 * main/CMakeLists.txt:
 *   idf_component_register(SRCS "main.c"
 *                          INCLUDE_DIRS "."
 *                          REQUIRES esp_driver_spi esp_driver_gpio esp_driver_rmt)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

static const char *TAG = "NRF_TEST";

/* ====================== CONFIG ====================== */
#define TEST_SPI_HZ      16000000
#define REG_ITERS        300
#define RF_PKTS          10
#define RF_CHANNEL       76
#define RF_SETUP_VAL     0x00       /* 1 Mbps, -18 dBm */
#define LED_GPIO         48
#define RESULT_SHOW_MS   5000

#define NUM_NRF          6

typedef struct {
    const char *name;
    spi_host_device_t host;
    int ce;
    int cs;
    spi_device_handle_t dev;
    int cur_hz;
    int best_hz;
    int errs[8];
    bool reg_ok;
    int tx_ok, rx_ok;
    bool alive;
} nrf_t;

static nrf_t nrf[NUM_NRF] = {
    { .name = "NRF1", .host = SPI2_HOST, .ce = 4,  .cs = 10 },
    { .name = "NRF2", .host = SPI2_HOST, .ce = 5,  .cs = 9  },
    { .name = "NRF3", .host = SPI2_HOST, .ce = 6,  .cs = 14 },
    { .name = "NRF4", .host = SPI3_HOST, .ce = 1,  .cs = 21 },
    { .name = "NRF5", .host = SPI3_HOST, .ce = 15, .cs = 16 },
    { .name = "NRF6", .host = SPI3_HOST, .ce = 17, .cs = 38 },
};

/* bus: SCK, MOSI, MISO */
static const struct { spi_host_device_t host; int sck, mosi, miso; } buses[] = {
    { SPI2_HOST, 12, 11, 13 },
    { SPI3_HOST, 18, 8,  7  },
};

static const int speeds[] = { 16000000, 10000000, 8000000, 4000000, 1000000 };
#define NUM_SPEEDS (sizeof(speeds) / sizeof(speeds[0]))

/* ====================== nRF24 ====================== */
#define R_REGISTER   0x00
#define W_REGISTER   0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX     0xE1
#define FLUSH_RX     0xE2
#define NOP          0xFF

#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RX_ADDR_P0  0x0A
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_FIFO_STATUS 0x17

#define CFG_PWR_UP   0x02
#define CFG_PRIM_RX  0x01
#define CFG_EN_CRC   0x08

#define ST_RX_DR     0x40
#define ST_TX_DS     0x20
#define ST_MAX_RT    0x10

/* CS en alto y CE en bajo para que ningun radio ensucie el MISO compartido */
static void park_pins(nrf_t *r)
{
    gpio_reset_pin(r->cs);
    gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
    gpio_set_level(r->cs, 1);
    gpio_reset_pin(r->ce);
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_level(r->ce, 0);
}

static esp_err_t nrf_open(nrf_t *r, int hz)
{
    spi_device_interface_config_t dc = {
        .clock_speed_hz = hz,
        .mode = 0,
        .spics_io_num = r->cs,
        .queue_size = 1,
    };
    esp_err_t e = spi_bus_add_device(r->host, &dc, &r->dev);
    if (e == ESP_OK) r->cur_hz = hz;
    return e;
}

static void nrf_close(nrf_t *r)
{
    if (r->dev) {
        spi_bus_remove_device(r->dev);
        r->dev = NULL;
    }
    gpio_set_level(r->ce, 0);
    park_pins(r);
}

static void nrf_xfer(nrf_t *r, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    spi_device_polling_transmit(r->dev, &t);
}

static uint8_t nrf_cmd(nrf_t *r, uint8_t cmd)
{
    uint8_t rx = 0;
    nrf_xfer(r, &cmd, &rx, 1);
    return rx;
}

static void nrf_write_reg(nrf_t *r, uint8_t reg, const uint8_t *data, size_t n)
{
    uint8_t tx[33], rx[33];
    tx[0] = W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], data, n);
    nrf_xfer(r, tx, rx, n + 1);
}

static void nrf_write_reg8(nrf_t *r, uint8_t reg, uint8_t v)
{
    nrf_write_reg(r, reg, &v, 1);
}

static void nrf_read_reg(nrf_t *r, uint8_t reg, uint8_t *data, size_t n)
{
    uint8_t tx[33] = { 0 }, rx[33];
    tx[0] = R_REGISTER | (reg & 0x1F);
    nrf_xfer(r, tx, rx, n + 1);
    memcpy(data, &rx[1], n);
}

static uint8_t nrf_read_reg8(nrf_t *r, uint8_t reg)
{
    uint8_t v;
    nrf_read_reg(r, reg, &v, 1);
    return v;
}

/* ====================== FASE 1: SPI ====================== */
/* Devuelve errores en REG_ITERS iteraciones. -1 si el bus esta muerto (todo 0x00 o 0xFF). */
static int reg_stress(nrf_t *r)
{
    nrf_cmd(r, NOP);
    nrf_write_reg8(r, REG_CONFIG, CFG_EN_CRC);

    int errs = 0;
    int dead = 0;
    for (int i = 0; i < REG_ITERS; i++) {
        uint8_t addr[5], back[5];
        esp_fill_random(addr, 5);
        nrf_write_reg(r, REG_TX_ADDR, addr, 5);
        nrf_read_reg(r, REG_TX_ADDR, back, 5);
        if (memcmp(addr, back, 5) != 0) {
            errs++;
            bool all0 = true, allF = true;
            for (int k = 0; k < 5; k++) {
                if (back[k] != 0x00) all0 = false;
                if (back[k] != 0xFF) allF = false;
            }
            if (all0 || allF) dead++;
        }

        uint8_t ch = esp_random() % 126;
        nrf_write_reg8(r, REG_RF_CH, ch);
        if (nrf_read_reg8(r, REG_RF_CH) != ch) errs++;
    }
    if (dead >= REG_ITERS - 2) return -1;
    return errs;
}

static void phase1_spi(void)
{
    ESP_LOGI(TAG, "---- FASE 1: SPI (%d iteraciones) ----", REG_ITERS);
    for (int i = 0; i < NUM_NRF; i++) {
        nrf_t *r = &nrf[i];
        r->best_hz = 0;
        r->reg_ok = false;
        for (int s = 0; s < (int)NUM_SPEEDS; s++) {
            r->errs[s] = -2;
            if (nrf_open(r, speeds[s]) != ESP_OK) continue;
            r->errs[s] = reg_stress(r);
            nrf_close(r);
            if (r->errs[s] == 0 && r->best_hz == 0) r->best_hz = speeds[s];
        }
        r->reg_ok = (r->best_hz != 0);

        char line[160];
        int n = snprintf(line, sizeof(line), "%s  ", r->name);
        for (int s = 0; s < (int)NUM_SPEEDS; s++) {
            const char *res;
            char buf[16];
            if (r->errs[s] == -1)      res = "MUERTO";
            else if (r->errs[s] == -2) res = "n/a";
            else if (r->errs[s] == 0)  res = "OK";
            else { snprintf(buf, sizeof(buf), "%d err", r->errs[s]); res = buf; }
            n += snprintf(line + n, sizeof(line) - n, "%dMHz:%s  ", speeds[s] / 1000000, res);
        }
        if (r->best_hz == TEST_SPI_HZ)  ESP_LOGI(TAG, "%s -> pasa a %d MHz", line, TEST_SPI_HZ / 1000000);
        else if (r->best_hz)            ESP_LOGW(TAG, "%s -> NO pasa a %d MHz, maximo estable %d MHz",
                                                  line, TEST_SPI_HZ / 1000000, r->best_hz / 1000000);
        else                            ESP_LOGE(TAG, "%s -> FALLA en todas las velocidades", line);
    }
}

/* ====================== FASE 2: RF ====================== */
static const uint8_t RF_ADDR[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

static void nrf_power_down(nrf_t *r)
{
    gpio_set_level(r->ce, 0);
    nrf_write_reg8(r, REG_CONFIG, CFG_EN_CRC);
}

static void nrf_common_setup(nrf_t *r)
{
    nrf_write_reg8(r, REG_EN_AA, 0x00);
    nrf_write_reg8(r, REG_EN_RXADDR, 0x01);
    nrf_write_reg8(r, REG_SETUP_AW, 0x03);
    nrf_write_reg8(r, REG_SETUP_RETR, 0x00);
    nrf_write_reg8(r, REG_RF_CH, RF_CHANNEL);
    nrf_write_reg8(r, REG_RF_SETUP, RF_SETUP_VAL);
    nrf_write_reg(r, REG_RX_ADDR_P0, RF_ADDR, 5);
    nrf_write_reg(r, REG_TX_ADDR, RF_ADDR, 5);
    nrf_write_reg8(r, REG_RX_PW_P0, 4);
    nrf_cmd(r, FLUSH_TX);
    nrf_cmd(r, FLUSH_RX);
    nrf_write_reg8(r, REG_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
}

/* Cuantos paquetes (de RF_PKTS) llegaron bien de tx a rx */
static int rf_link(nrf_t *tx, nrf_t *rx)
{
    int got = 0;

    nrf_common_setup(rx);
    nrf_write_reg8(rx, REG_CONFIG, CFG_EN_CRC | CFG_PWR_UP | CFG_PRIM_RX);
    nrf_common_setup(tx);
    nrf_write_reg8(tx, REG_CONFIG, CFG_EN_CRC | CFG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(rx->ce, 1);
    esp_rom_delay_us(150);

    for (int seq = 0; seq < RF_PKTS; seq++) {
        uint8_t pl[5] = { W_TX_PAYLOAD, (uint8_t)(tx - nrf), (uint8_t)(rx - nrf), (uint8_t)seq, (uint8_t)~seq };
        uint8_t dummy[5];
        nrf_xfer(tx, pl, dummy, 5);

        gpio_set_level(tx->ce, 1);
        esp_rom_delay_us(20);
        gpio_set_level(tx->ce, 0);

        int t_us = 0;
        while (t_us < 3000) {
            uint8_t st = nrf_cmd(tx, NOP);
            if (st & (ST_TX_DS | ST_MAX_RT)) break;
            esp_rom_delay_us(50);
            t_us += 50;
        }
        nrf_write_reg8(tx, REG_STATUS, ST_TX_DS | ST_MAX_RT);
        nrf_cmd(tx, FLUSH_TX);

        esp_rom_delay_us(300);

        uint8_t fifo = nrf_read_reg8(rx, REG_FIFO_STATUS);
        if (!(fifo & 0x01)) {
            uint8_t tb[5] = { R_RX_PAYLOAD, 0, 0, 0, 0 }, rb[5];
            nrf_xfer(rx, tb, rb, 5);
            if (rb[1] == (uint8_t)(tx - nrf) && rb[2] == (uint8_t)(rx - nrf) &&
                rb[3] == (uint8_t)seq && rb[4] == (uint8_t)~seq) got++;
            nrf_write_reg8(rx, REG_STATUS, ST_RX_DR);
            nrf_cmd(rx, FLUSH_RX);
        }
    }

    nrf_power_down(rx);
    nrf_power_down(tx);
    return got;
}

static void phase2_rf(void)
{
    ESP_LOGI(TAG, "---- FASE 2: enlace RF todos contra todos (%d paquetes) ----", RF_PKTS);

    for (int i = 0; i < NUM_NRF; i++) {
        nrf[i].tx_ok = nrf[i].rx_ok = 0;
        if (nrf[i].reg_ok) {
            if (nrf_open(&nrf[i], nrf[i].best_hz) == ESP_OK) nrf_power_down(&nrf[i]);
            else nrf[i].reg_ok = false;
        }
    }

    int m[NUM_NRF][NUM_NRF];
    memset(m, -1, sizeof(m));
    for (int t = 0; t < NUM_NRF; t++) {
        for (int rcv = 0; rcv < NUM_NRF; rcv++) {
            if (t == rcv || !nrf[t].reg_ok || !nrf[rcv].reg_ok) continue;
            m[t][rcv] = rf_link(&nrf[t], &nrf[rcv]);
            if (m[t][rcv] > 0) { nrf[t].tx_ok++; nrf[rcv].rx_ok++; }
        }
    }

    for (int i = 0; i < NUM_NRF; i++) if (nrf[i].dev) nrf_close(&nrf[i]);

    printf("\n       RX: ");
    for (int c = 0; c < NUM_NRF; c++) printf("  %s", nrf[c].name);
    printf("\n");
    for (int t = 0; t < NUM_NRF; t++) {
        printf("  TX %s: ", nrf[t].name);
        for (int c = 0; c < NUM_NRF; c++) {
            if (m[t][c] < 0) printf("    -- ");
            else printf("  %2d/%d ", m[t][c], RF_PKTS);
        }
        printf("\n");
    }
    printf("\n");
}

/* ====================== LED WS2812 (RMT) ====================== */
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} led_enc_t;

static size_t led_enc_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                             const void *data, size_t size, rmt_encode_state_t *ret_state)
{
    led_enc_t *e = (led_enc_t *)encoder;
    rmt_encode_state_t session = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded = 0;

    switch (e->state) {
    case 0:
        encoded += e->bytes_encoder->encode(e->bytes_encoder, channel, data, size, &session);
        if (session & RMT_ENCODING_COMPLETE) e->state = 1;
        if (session & RMT_ENCODING_MEM_FULL) { state |= RMT_ENCODING_MEM_FULL; goto out; }
        /* fallthrough */
    case 1:
        encoded += e->copy_encoder->encode(e->copy_encoder, channel, &e->reset_code,
                                           sizeof(e->reset_code), &session);
        if (session & RMT_ENCODING_COMPLETE) { e->state = 0; state |= RMT_ENCODING_COMPLETE; }
        if (session & RMT_ENCODING_MEM_FULL) { state |= RMT_ENCODING_MEM_FULL; goto out; }
    }
out:
    *ret_state = state;
    return encoded;
}

static esp_err_t led_enc_reset(rmt_encoder_t *encoder)
{
    led_enc_t *e = (led_enc_t *)encoder;
    rmt_encoder_reset(e->bytes_encoder);
    rmt_encoder_reset(e->copy_encoder);
    e->state = 0;
    return ESP_OK;
}

static esp_err_t led_enc_del(rmt_encoder_t *encoder)
{
    led_enc_t *e = (led_enc_t *)encoder;
    rmt_del_encoder(e->bytes_encoder);
    rmt_del_encoder(e->copy_encoder);
    free(e);
    return ESP_OK;
}

static rmt_channel_handle_t led_chan;
static rmt_encoder_handle_t led_encoder;

static void led_init(void)
{
    rmt_tx_channel_config_t cc = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 10000000,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&cc, &led_chan));

    led_enc_t *e = calloc(1, sizeof(led_enc_t));
    e->base.encode = led_enc_encode;
    e->base.reset = led_enc_reset;
    e->base.del = led_enc_del;

    rmt_bytes_encoder_config_t bc = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bc, &e->bytes_encoder));
    rmt_copy_encoder_config_t cp = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&cp, &e->copy_encoder));
    e->reset_code = (rmt_symbol_word_t){ .level0 = 0, .duration0 = 250, .level1 = 0, .duration1 = 250 };

    led_encoder = &e->base;
    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tc = { .loop_count = 0 };
    rmt_transmit(led_chan, led_encoder, grb, sizeof(grb), &tc);
    rmt_tx_wait_all_done(led_chan, 100);
}

static void hsv2rgb(uint16_t h, uint8_t s, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b)
{
    h %= 360;
    uint16_t region = h / 60, rem = (h % 60) * 255 / 60;
    uint8_t p = (v * (255 - s)) / 255;
    uint8_t q = (v * (255 - (s * rem) / 255)) / 255;
    uint8_t t = (v * (255 - (s * (255 - rem)) / 255)) / 255;
    switch (region) {
    case 0: *r = v; *g = t; *b = p; break;
    case 1: *r = q; *g = v; *b = p; break;
    case 2: *r = p; *g = v; *b = t; break;
    case 3: *r = p; *g = q; *b = v; break;
    case 4: *r = t; *g = p; *b = v; break;
    default: *r = v; *g = p; *b = q; break;
    }
}

static void led_rainbow_for(int ms)
{
    uint16_t hue = 0;
    for (int t = 0; t < ms; t += 20) {
        uint8_t r, g, b;
        hsv2rgb(hue, 255, 40, &r, &g, &b);
        led_set(r, g, b);
        hue += 6;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void led_blink(uint8_t r, uint8_t g, uint8_t b, int n)
{
    for (int i = 0; i < n; i++) {
        led_set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(250));
        led_set(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* verdes = radios OK, pausa, rojos = radios con falla, pausa; repite */
static void led_show_counts(int ok, int bad, int ms)
{
    for (int t = 0; t < ms; ) {
        led_blink(0, 40, 0, ok);
        vTaskDelay(pdMS_TO_TICKS(400));
        led_blink(40, 0, 0, bad);
        vTaskDelay(pdMS_TO_TICKS(900));
        t += ok * 500 + 400 + bad * 500 + 900;
    }
}

/* ====================== MAIN ====================== */
static void init_spi_buses(void)
{
    for (int i = 0; i < NUM_NRF; i++) park_pins(&nrf[i]);

    for (size_t b = 0; b < sizeof(buses) / sizeof(buses[0]); b++) {
        spi_bus_config_t bc = {
            .sclk_io_num = buses[b].sck,
            .mosi_io_num = buses[b].mosi,
            .miso_io_num = buses[b].miso,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 64,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(buses[b].host, &bc, SPI_DMA_DISABLED));
    }
}

void app_main(void)
{
    led_init();
    led_set(40, 30, 0);
    init_spi_buses();

    ESP_LOGI(TAG, "Prueba de 6 nRF24L01 | objetivo SPI: %d MHz", TEST_SPI_HZ / 1000000);

    for (int round = 1; ; round++) {
        ESP_LOGI(TAG, "================ RONDA %d ================", round);
        led_set(40, 30, 0);

        phase1_spi();
        phase2_rf();

        int ok = 0;
        for (int i = 0; i < NUM_NRF; i++) {
            nrf[i].alive = nrf[i].reg_ok && nrf[i].tx_ok > 0 && nrf[i].rx_ok > 0;
            if (nrf[i].alive) ok++;
            if (nrf[i].alive)
                ESP_LOGI(TAG, "%s: VIVO (SPI max %d MHz, TX a %d radios, RX de %d radios)",
                         nrf[i].name, nrf[i].best_hz / 1000000, nrf[i].tx_ok, nrf[i].rx_ok);
            else if (nrf[i].reg_ok)
                ESP_LOGW(TAG, "%s: SPI OK pero RF falla (TX a %d, RX de %d) -> revisar antena/PA/LNA/cristal/alimentacion",
                         nrf[i].name, nrf[i].tx_ok, nrf[i].rx_ok);
            else
                ESP_LOGE(TAG, "%s: SPI NO responde -> revisar CS/CE/MISO/MOSI/SCK, soldadura y 3.3V", nrf[i].name);
        }

        if (ok == NUM_NRF) {
            ESP_LOGI(TAG, "*** LOS 6 NRF VIVOS ***");
            led_rainbow_for(RESULT_SHOW_MS);
        } else {
            ESP_LOGE(TAG, "Solo %d de %d radios vivos", ok, NUM_NRF);
            led_show_counts(ok, NUM_NRF - ok, RESULT_SHOW_MS);
        }
    }
}
