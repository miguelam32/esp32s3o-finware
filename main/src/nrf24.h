/*
 * nrf24.h — driver nRF24L01+ multi-dispositivo, SPI nativo (ESP-IDF 6.0.1)
 *
 * Cada modulo es un handle independiente. Varios modulos en distintos
 * hosts SPI, usados a la vez desde cores distintos, sin colas compartidas.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nrf24_dev nrf24_dev_t;

typedef struct {
    spi_host_device_t host;   /* SPI2_HOST o SPI3_HOST. NUNCA SPI0/1. */
    int mosi;
    int miso;
    int sclk;
    int cs;
    int ce;                    /* -1 si no conectas CE */
} nrf24_cfg_t;

/* ---- registros ---- */
#define NRF24_CONFIG       0x00u
#define NRF24_EN_AA        0x01u
#define NRF24_SETUP_AW     0x03u
#define NRF24_RF_CH        0x05u
#define NRF24_RF_SETUP     0x06u
#define NRF24_STATUS       0x07u
#define NRF24_RPD          0x09u
#define NRF24_RX_ADDR_P0   0x0Au
#define NRF24_TX_ADDR      0x10u
#define NRF24_RX_PW_P0     0x11u
#define NRF24_FIFO_STATUS  0x17u
#define NRF24_DYNPD        0x1Cu
#define NRF24_FEATURE      0x1Du

/* ---- comandos SPI ---- */
#define NRF24_CMD_R_REGISTER    0x00u
#define NRF24_CMD_W_REGISTER    0x20u
#define NRF24_CMD_R_RX_PAYLOAD  0x61u
#define NRF24_CMD_W_TX_PAYLOAD  0xA0u
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8u
#define NRF24_CMD_FLUSH_TX      0xE1u
#define NRF24_CMD_FLUSH_RX      0xE2u
#define NRF24_CMD_REUSE_TX_PL   0xE3u
#define NRF24_CMD_NOP           0xFFu

/* ---- CONFIG ---- */
#define NRF24_CFG_PRIM_RX   0x01u
#define NRF24_CFG_PWR_UP    0x02u
#define NRF24_CFG_CRCO_2B   0x04u
#define NRF24_CFG_EN_CRC    0x08u

/* ---- RF_SETUP ---- */
#define NRF24_RF_DR_HIGH    0x08u
#define NRF24_RF_PWR_0      0x06u    /* 0 dBm: maximo del chip */

/* ---- API ---- */
nrf24_dev_t *nrf24_new(const nrf24_cfg_t *cfg);   /* NULL si falla */

esp_err_t nrf24_write_reg(nrf24_dev_t *d, uint8_t reg, uint8_t val);
esp_err_t nrf24_read_reg (nrf24_dev_t *d, uint8_t reg, uint8_t *val);
esp_err_t nrf24_write_buf(nrf24_dev_t *d, uint8_t reg, const uint8_t *buf, size_t len);
esp_err_t nrf24_read_buf (nrf24_dev_t *d, uint8_t reg, uint8_t *buf, size_t len);

/* CAMINO RAPIDO: polling, sin ISR ni semaforo por transferencia. */
esp_err_t nrf24_fast(nrf24_dev_t *d, uint8_t cmd,
                     const uint8_t *tx, uint8_t *rx, size_t n);

esp_err_t nrf24_set_channel(nrf24_dev_t *d, uint8_t ch);
void     nrf24_rx_mode(nrf24_dev_t *d);
void     nrf24_power_down(nrf24_dev_t *d);
void     nrf24_ce(nrf24_dev_t *d, int level);

/* Deja el modulo en RX con CE alto de forma permanente: durante un
 * barrido ya no hay que tocar CONFIG ni el CE, solo el canal. */
void     nrf24_rx_cont(nrf24_dev_t *d);
uint8_t  nrf24_rpd(nrf24_dev_t *d, uint8_t ch);       /* auto-contenida */
uint8_t  nrf24_rpd_fast(nrf24_dev_t *d, uint8_t ch);  /* usa rx_cont previo */

esp_err_t nrf24_carrier(nrf24_dev_t *d, uint8_t ch);
esp_err_t nrf24_carrier_stop(nrf24_dev_t *d);

#ifdef __cplusplus
}
#endif
