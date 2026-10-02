/*
 * nrf24.c — driver nRF24L01+ (ESP-IDF 6.0.1, SPI nativo, multi-dispositivo)
 */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "nrf24.h"

static const char *TAG = "nrf24";

struct nrf24_dev {
    spi_device_handle_t spi;
    int ce;
};

/* ------------------------------------------------------------------ */
static esp_err_t xfer(nrf24_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t n)
{
    spi_transaction_t t = {
        .length    = n * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(d->spi, &t);
}

/* ------------------------------------------------------------------ */
nrf24_dev_t *nrf24_new(const nrf24_cfg_t *cfg)
{
    if (cfg == NULL) return NULL;

    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi,
        .miso_io_num     = cfg->miso,
        .sclk_io_num     = cfg->sclk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 32,
    };

    /* ESP_ERR_INVALID_STATE = bus ya inicializado (lo toleramos) */
    esp_err_t e = spi_bus_initialize(cfg->host, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize(%d): %s",
                 (int)cfg->host, esp_err_to_name(e));
        return NULL;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = 10000000,     /* 10 MHz: maximo del nRF24L01+ */
        .mode           = 0,            /* CPOL=0 CPHA=0 */
        .spics_io_num   = cfg->cs,
        .queue_size     = 1,            /* 1: usamos polling */
        .flags          = 0,
    };

    spi_device_handle_t spi = NULL;
    e = spi_bus_add_device(cfg->host, &dev, &spi);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(e));
        return NULL;
    }

    nrf24_dev_t *d = calloc(1, sizeof(nrf24_dev_t));
    if (d == NULL) return NULL;
    d->spi = spi;
    d->ce  = cfg->ce;

    if (cfg->ce >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->ce),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&io) != ESP_OK) {
            free(d);
            return NULL;
        }
        gpio_set_level(cfg->ce, 0);
    }

    nrf24_write_reg(d, NRF24_CONFIG,
                    NRF24_CFG_PWR_UP | NRF24_CFG_EN_CRC |
                    NRF24_CFG_CRCO_2B | NRF24_CFG_PRIM_RX);
    nrf24_write_reg(d, NRF24_EN_AA, 0x00);        /* auto-ack off */
    nrf24_write_reg(d, NRF24_SETUP_AW, 0x03);     /* direcciones 5B */
    nrf24_write_reg(d, NRF24_RX_PW_P0, 32);
    nrf24_write_reg(d, NRF24_RF_SETUP,            /* 2 Mbps, 0 dBm */
                    NRF24_RF_DR_HIGH | NRF24_RF_PWR_0);

    ESP_LOGI(TAG, "nRF24 listo (host=%d CE=%d)", (int)cfg->host, d->ce);
    return d;
}

/* ------------------------------------------------------------------ */
esp_err_t nrf24_fast(nrf24_dev_t *d, uint8_t cmd,
                     const uint8_t *tx, uint8_t *rx, size_t n)
{
    if (d == NULL || n > 5) return ESP_ERR_INVALID_ARG;

    uint8_t tbuf[6] = { 0 };
    uint8_t rbuf[6] = { 0 };

    tbuf[0] = cmd;
    if (tx != NULL) memcpy(&tbuf[1], tx, n);

    spi_transaction_t t = {
        .length    = (n + 1) * 8,
        .tx_buffer = tbuf,
        .rx_buffer = rbuf,
    };

    esp_err_t e = spi_device_polling_transmit(d->spi, &t);
    if (e != ESP_OK) return e;
    if (rx != NULL) memcpy(rx, &rbuf[1], n);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t nrf24_write_reg(nrf24_dev_t *d, uint8_t reg, uint8_t val)
{
    if (d == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx[2] = { (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu)), val };
    uint8_t rx[2] = { 0 };
    return xfer(d, tx, rx, 2);
}

esp_err_t nrf24_read_reg(nrf24_dev_t *d, uint8_t reg, uint8_t *val)
{
    if (d == NULL || val == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx[2] = { (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu)), 0xFF };
    uint8_t rx[2] = { 0 };
    esp_err_t e = xfer(d, tx, rx, 2);
    if (e != ESP_OK) return e;
    *val = rx[1];
    return ESP_OK;
}

esp_err_t nrf24_write_buf(nrf24_dev_t *d, uint8_t reg,
                          const uint8_t *buf, size_t len)
{
    if (d == NULL || buf == NULL || len == 0 || len > 5) return ESP_ERR_INVALID_ARG;
    uint8_t tx[6] = { 0 }, rx[6] = { 0 };
    tx[0] = (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu));
    memcpy(&tx[1], buf, len);
    return xfer(d, tx, rx, len + 1);
}

esp_err_t nrf24_read_buf(nrf24_dev_t *d, uint8_t reg, uint8_t *buf, size_t len)
{
    if (d == NULL || buf == NULL || len == 0 || len > 32) return ESP_ERR_INVALID_ARG;
    uint8_t tx[33] = { 0 }, rx[33] = { 0 };
    tx[0] = (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu));
    memset(&tx[1], 0xFF, len);
    esp_err_t e = xfer(d, tx, rx, len + 1);
    if (e != ESP_OK) return e;
    memcpy(buf, &rx[1], len);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
esp_err_t nrf24_set_channel(nrf24_dev_t *d, uint8_t ch)
{
    if (d == NULL || ch > 125u) return ESP_ERR_INVALID_ARG;
    return nrf24_fast(d, NRF24_CMD_W_REGISTER | NRF24_RF_CH, &ch, NULL, 1);
}

void nrf24_ce(nrf24_dev_t *d, int level)
{
    if (d == NULL || d->ce < 0) return;
    gpio_set_level(d->ce, level ? 1 : 0);
}

void nrf24_rx_mode(nrf24_dev_t *d)
{
    if (d == NULL) return;
    nrf24_ce(d, 0);
    uint8_t c = 0;
    if (nrf24_read_reg(d, NRF24_CONFIG, &c) == ESP_OK) {
        nrf24_write_reg(d, NRF24_CONFIG,
                        (uint8_t)(c | NRF24_CFG_PWR_UP | NRF24_CFG_PRIM_RX));
    }
}

void nrf24_power_down(nrf24_dev_t *d)
{
    if (d == NULL) return;
    nrf24_ce(d, 0);
    uint8_t c = 0;
    if (nrf24_read_reg(d, NRF24_CONFIG, &c) == ESP_OK) {
        nrf24_write_reg(d, NRF24_CONFIG, (uint8_t)(c & ~NRF24_CFG_PWR_UP));
    }
}

void nrf24_rx_cont(nrf24_dev_t *d)
{
    if (d == NULL) return;
    nrf24_ce(d, 0);
    uint8_t c = 0;
    if (nrf24_read_reg(d, NRF24_CONFIG, &c) == ESP_OK) {
        nrf24_write_reg(d, NRF24_CONFIG,
                        (uint8_t)(c | NRF24_CFG_PWR_UP | NRF24_CFG_PRIM_RX));
    }
    nrf24_ce(d, 1);
}

uint8_t nrf24_rpd(nrf24_dev_t *d, uint8_t ch)
{
    if (d == NULL || ch > 125u) return 0;
    nrf24_rx_mode(d);
    return nrf24_rpd_fast(d, ch);
}

uint8_t nrf24_rpd_fast(nrf24_dev_t *d, uint8_t ch)
{
    if (d == NULL || ch > 125u) return 0;

    uint8_t v = 0;
    nrf24_fast(d, NRF24_CMD_W_REGISTER | NRF24_RF_CH, &ch, NULL, 1);
    usleep(200);                       /* asentamiento del PLL */
    nrf24_fast(d, NRF24_CMD_R_REGISTER | NRF24_RPD, NULL, &v, 1);
    return (uint8_t)(v & 0x01u);
}

/* ------------------------------------------------------------------ */
/* Jammer: carrier continuo. El radio en TX con REUSE_TX_PL y CE alto
 * emite portadora continua en el canal dado.                          */
esp_err_t nrf24_carrier(nrf24_dev_t *d, uint8_t ch)
{
    if (d == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t c = 0;
    if (nrf24_read_reg(d, NRF24_CONFIG, &c) != ESP_OK) return ESP_FAIL;

    /* modo TX (PRIM_RX = 0), power up */
    nrf24_write_reg(d, NRF24_CONFIG,
                    (uint8_t)((c | NRF24_CFG_PWR_UP) & ~NRF24_CFG_PRIM_RX));
    nrf24_write_reg(d, NRF24_RF_SETUP, NRF24_RF_DR_HIGH | NRF24_RF_PWR_0);

    uint8_t zero[32];
    memset(zero, 0, sizeof(zero));
    nrf24_write_buf(d, NRF24_TX_ADDR, zero, 5);
    nrf24_write_buf(d, NRF24_CMD_W_TX_PAYLOAD, zero, 32);

    nrf24_set_channel(d, ch);
    nrf24_fast(d, NRF24_CMD_REUSE_TX_PL, NULL, NULL, 0);
    nrf24_ce(d, 1);
    return ESP_OK;
}

esp_err_t nrf24_carrier_stop(nrf24_dev_t *d)
{
    if (d == NULL) return ESP_ERR_INVALID_ARG;
    nrf24_ce(d, 0);
    nrf24_fast(d, NRF24_CMD_FLUSH_TX, NULL, NULL, 0);
    nrf24_rx_cont(d);
    return ESP_OK;
}
