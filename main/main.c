/*
 * nrf_txrx_test.c - prueba real de enlace (Enhanced ShockBurst) entre
 * las dos nRF24L01+ del mismo ESP32-S3 N16R8.
 *
 * Radio #1 (SPI2: CE=9 CS=10 SCK=12 MOSI=11 MISO=13) transmite texto.
 * Radio #2 (SPI3: CE=4 CS=5 SCK=7 MOSI=6 MISO=8) escucha y lo imprime
 * por el monitor serie.
 *
 * No es onda portadora continua (eso seria jamming): es trafico real
 * de paquetes nRF24. AutoACK desactivado para simplificar el
 * diagnostico (asi se aisla si el problema es TX o RX). GFSK es la
 * modulacion que usa el chip siempre, no hay que configurarla aparte
 * de canal y data rate.
 *
 * Para correr esto: reemplaza temporalmente tu main.c por este
 * archivo (o comenta el app_main real), compila y mira el monitor.
 * Cuando confirmes que el radio 2 esta vivo, volves a tu firmware.
 */
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *PTAG = "probe";
static const char *TTAG = "tx";
static const char *RTAG = "rx";

/* ---------- pinout ---------- */
#define R1_HOST SPI2_HOST
#define R1_CE   GPIO_NUM_9
#define R1_CS   GPIO_NUM_10
#define R1_SCK  GPIO_NUM_12
#define R1_MOSI GPIO_NUM_11
#define R1_MISO GPIO_NUM_13

#define R2_HOST SPI3_HOST
#define R2_CE   GPIO_NUM_4
#define R2_CS   GPIO_NUM_5
#define R2_SCK  GPIO_NUM_7
#define R2_MOSI GPIO_NUM_6
#define R2_MISO GPIO_NUM_8

#define RF_CHANNEL 100          /* 2500 MHz, lejos de WiFi 2.4G (canales 1-13) */
#define PAYLOAD_SZ 32

static const uint8_t LINK_ADDR[5] = { 0xC2, 0xC2, 0xC2, 0xC2, 0xC2 };

/* ---------- registros nRF24L01+ ---------- */
#define R_CONFIG      0x00
#define R_EN_AA       0x01
#define R_EN_RXADDR   0x02
#define R_SETUP_AW    0x03
#define R_SETUP_RETR  0x04
#define R_RF_CH       0x05
#define R_RF_SETUP    0x06
#define R_STATUS      0x07
#define R_RX_ADDR_P0  0x0A
#define R_TX_ADDR     0x10
#define R_RX_PW_P0    0x11
#define R_FIFO_STATUS 0x17

#define CMD_W_REG        0x20
#define CMD_R_RX_PAYLOAD 0x61
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX     0xE1
#define CMD_FLUSH_RX     0xE2
#define CMD_NOP          0xFF

#define CFG_PRIM_RX (1u << 0)
#define CFG_PWR_UP  (1u << 1)
#define CFG_EN_CRC  (1u << 3)

#define ST_RX_DR  (1u << 6)
#define ST_TX_DS  (1u << 5)
#define ST_MAX_RT (1u << 4)

typedef struct {
    gpio_num_t ce;
    spi_device_handle_t spi;
} nrf_t;

/* ---------- SPI de bajo nivel (buffers tx/rx separados, sin lios de DMA) ---------- */
static esp_err_t xfer(nrf_t *r, const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = { .length = len * 8, .tx_buffer = tx, .rx_buffer = rx };
    return spi_device_polling_transmit(r->spi, &t);
}

static uint8_t rd(nrf_t *r, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x1F), CMD_NOP };
    uint8_t rx[2] = { 0 };
    xfer(r, tx, rx, 2);
    return rx[1];
}

static void wr(nrf_t *r, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(CMD_W_REG | (reg & 0x1F)), val };
    xfer(r, tx, NULL, 2);
}

static void wr_buf(nrf_t *r, uint8_t cmd, const uint8_t *data, size_t len)
{
    uint8_t tx[1 + PAYLOAD_SZ];
    tx[0] = cmd;
    memcpy(&tx[1], data, len);
    xfer(r, tx, NULL, len + 1);
}

static void rd_buf(nrf_t *r, uint8_t cmd, uint8_t *data, size_t len)
{
    uint8_t tx[1 + PAYLOAD_SZ];
    uint8_t rx[1 + PAYLOAD_SZ];
    memset(tx, CMD_NOP, len + 1);
    tx[0] = cmd;
    xfer(r, tx, rx, len + 1);
    memcpy(data, &rx[1], len);
}

static void cmd_only(nrf_t *r, uint8_t cmd)
{
    uint8_t tx[1] = { cmd };
    xfer(r, tx, NULL, 1);
}

/* self-test: escribe RF_CH y lo relee. Si no coincide, el chip no
 * esta contestando por SPI (cableado, alimentacion o modulo muerto). */
static bool alive(nrf_t *r, const char *name)
{
    wr(r, R_RF_CH, 0x5A);
    uint8_t back = rd(r, R_RF_CH);
    bool ok = (back == 0x5A);
    ESP_LOGW(PTAG, "%s -> RF_CH leido=0x%02X %s", name, back, ok ? "OK" : "FALLA");
    return ok;
}

static nrf_t radio_open(spi_host_device_t host, gpio_num_t ce, gpio_num_t cs,
                         gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso)
{
    nrf_t r = { .ce = ce };

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << ce),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(ce, 0);

    spi_bus_config_t bus = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = PAYLOAD_SZ + 1,
    };
    esp_err_t e = spi_bus_initialize(host, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(PTAG, "spi_bus_initialize SPI%d: %s", (int)host, esp_err_to_name(e));
    }

    spi_device_interface_config_t dev = {
        .mode = 0, .clock_speed_hz = 10000000, .spics_io_num = cs, .queue_size = 1,
        .cs_ena_pretrans = 2, .cs_ena_posttrans = 2,
    };
    esp_err_t e2 = spi_bus_add_device(host, &dev, &r.spi);
    if (e2 != ESP_OK) {
        ESP_LOGE(PTAG, "spi_bus_add_device SPI%d: %s", (int)host, esp_err_to_name(e2));
    }
    return r;
}

static void setup_tx(nrf_t *r)
{
    gpio_set_level(r->ce, 0);
    wr(r, R_EN_AA, 0x00);
    wr(r, R_EN_RXADDR, 0x00);
    wr(r, R_SETUP_AW, 0x03);
    wr(r, R_SETUP_RETR, 0x00);
    wr(r, R_RF_CH, RF_CHANNEL);
    wr(r, R_RF_SETUP, 0x06);                         /* 1 Mbps, 0 dBm */
    wr_buf(r, CMD_W_REG | R_TX_ADDR, LINK_ADDR, 5);
    wr(r, R_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
    cmd_only(r, CMD_FLUSH_TX);
    wr(r, R_CONFIG, CFG_EN_CRC | CFG_PWR_UP);        /* PTX */
}

static void setup_rx(nrf_t *r)
{
    gpio_set_level(r->ce, 0);
    wr(r, R_EN_AA, 0x00);
    wr(r, R_EN_RXADDR, 0x01);
    wr(r, R_SETUP_AW, 0x03);
    wr(r, R_RF_CH, RF_CHANNEL);
    wr(r, R_RF_SETUP, 0x06);
    wr_buf(r, CMD_W_REG | R_RX_ADDR_P0, LINK_ADDR, 5);
    wr(r, R_RX_PW_P0, PAYLOAD_SZ);
    wr(r, R_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
    cmd_only(r, CMD_FLUSH_RX);
    wr(r, R_CONFIG, CFG_EN_CRC | CFG_PWR_UP | CFG_PRIM_RX); /* PRX */
    gpio_set_level(r->ce, 1);                         /* a escuchar */
}

static bool send_line(nrf_t *r, const char *text)
{
    uint8_t payload[PAYLOAD_SZ];
    memset(payload, 0, PAYLOAD_SZ);
    strncpy((char *)payload, text, PAYLOAD_SZ - 1);

    cmd_only(r, CMD_FLUSH_TX);
    wr_buf(r, CMD_W_TX_PAYLOAD, payload, PAYLOAD_SZ);

    gpio_set_level(r->ce, 1);
    esp_rom_delay_us(15);
    gpio_set_level(r->ce, 0);

    uint8_t st = 0;
    for (int i = 0; i < 2000; i++) {                  /* ~2 ms de margen */
        st = rd(r, R_STATUS);
        if (st & (ST_TX_DS | ST_MAX_RT)) break;
        esp_rom_delay_us(1);
    }
    wr(r, R_STATUS, ST_TX_DS | ST_MAX_RT);
    return (st & ST_TX_DS) != 0;
}

/* Texto que manda radio 1 y que radio 2 debe recibir e imprimir igual,
 * linea por linea. Cada linea entra en un paquete (max 31 caracteres
 * + el nulo final, se corta si es mas larga). Reemplaza estas lineas
 * por las que quieras mandarte vos mismo: no puedo dejar aqui letra
 * real de ninguna cancion con derechos de autor, pero el array es
 * tuyo, pegale lo que quieras. */
static const char *LINES[] = {
    "LINEA 1 (pegue aqui su texto)",
    "LINEA 2 (pegue aqui su texto)",
    "LINEA 3 (pegue aqui su texto)",
    "LINEA 4 (pegue aqui su texto)",
};
#define N_LINES (sizeof(LINES) / sizeof(LINES[0]))

static nrf_t g_tx, g_rx;

static void tx_task(void *arg)
{
    (void)arg;
    setup_tx(&g_tx);
    for (;;) {
        for (size_t i = 0; i < N_LINES; i++) {
            bool ok = send_line(&g_tx, LINES[i]);
            ESP_LOGW(TTAG, "enviado [%d]: \"%s\" %s",
                     (int)i, LINES[i], ok ? "(TX_DS)" : "(TIMEOUT!)");
            vTaskDelay(pdMS_TO_TICKS(400));
        }
    }
}

static void rx_task(void *arg)
{
    (void)arg;
    setup_rx(&g_rx);
    for (;;) {
        uint8_t fifo = rd(&g_rx, R_FIFO_STATUS);
        if (!(fifo & 0x01)) {                         /* bit0=RX_EMPTY -> hay datos */
            uint8_t payload[PAYLOAD_SZ + 1];
            rd_buf(&g_rx, CMD_R_RX_PAYLOAD, payload, PAYLOAD_SZ);
            payload[PAYLOAD_SZ] = '\0';
            wr(&g_rx, R_STATUS, ST_RX_DR);
            ESP_LOGW(RTAG, "recibido: \"%s\"", (char *)payload);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    g_tx = radio_open(R1_HOST, R1_CE, R1_CS, R1_SCK, R1_MOSI, R1_MISO);
    g_rx = radio_open(R2_HOST, R2_CE, R2_CS, R2_SCK, R2_MOSI, R2_MISO);

    bool ok1 = alive(&g_tx, "radio1 (SPI2)");
    bool ok2 = alive(&g_rx, "radio2 (SPI3)");
    if (!ok1 || !ok2) {
        ESP_LOGW(PTAG, "al menos un radio no respondio por SPI - revisa "
                 "cableado/alimentacion antes de confiar en el resto del test");
    }

    xTaskCreate(tx_task, "tx", 4096, NULL, 5, NULL);
    xTaskCreate(rx_task, "rx", 4096, NULL, 5, NULL);
}

