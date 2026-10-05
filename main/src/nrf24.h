/*
 * nrf24.h - Driver nativo multi-instancia para nRF24L01+ / PA+LNA.
 * ESP-IDF v6.0.x, sin Arduino.
 *
 * Reglas:
 *   1. Toda secuencia SPI compuesta es atomica (mutex por instancia).
 *   2. CE se baja SIEMPRE antes de tocar RF_CH (fix del carrier
 *      congelado en modulos PA+LNA: nRF24/RF24#714).
 *
 * Mapa de registros: nRF24L01+ Product Specification v2.0 (Nordic).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_MAX_DEV   4
#define NRF_MAX_XFER  33          /* 1 byte comando + 32 payload */

/* ---- registros ---- */
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_OBSERVE_TX  0x08
#define NRF_RPD         0x09
#define NRF_RX_ADDR_P0  0x0A
#define NRF_TX_ADDR     0x10
#define NRF_RX_PW_P0    0x11
#define NRF_FIFO_STATUS 0x17
#define NRF_DYNPD       0x1C
#define NRF_FEATURE     0x1D

/* ---- comandos SPI ---- */
#define NRF_CMD_R_REGISTER         0x00
#define NRF_CMD_W_REGISTER         0x20
#define NRF_CMD_R_RX_PAYLOAD       0x61
#define NRF_CMD_W_TX_PAYLOAD       0xA0
#define NRF_CMD_W_TX_PAYLOAD_NOACK 0xB0
#define NRF_CMD_FLUSH_TX           0xE1
#define NRF_CMD_FLUSH_RX           0xE2
#define NRF_CMD_NOP                0xFF

/* ---- CONFIG ---- */
#define NRF_CFG_MASK_RX_DR   (1u << 6)
#define NRF_CFG_MASK_TX_DS   (1u << 5)
#define NRF_CFG_MASK_MAX_RT  (1u << 4)
#define NRF_CFG_EN_CRC       (1u << 3)
#define NRF_CFG_CRCO         (1u << 2)
#define NRF_CFG_PWR_UP       (1u << 1)
#define NRF_CFG_PRIM_RX      (1u << 0)

/* ---- RF_SETUP ---- */
#define NRF_RF_CONT_WAVE     (1u << 7)
#define NRF_RF_PLL_LOCK      (1u << 4)
#define NRF_RF_RF_DR_LOW     (1u << 5)
#define NRF_RF_RF_DR_HIGH    (1u << 3)

/* ---- STATUS ---- */
#define NRF_ST_RX_DR         (1u << 6)
#define NRF_ST_TX_DS         (1u << 5)
#define NRF_ST_MAX_RT        (1u << 4)
#define NRF_ST_TX_FULL       (1u << 0)

/* ---- FIFO_STATUS ---- */
#define NRF_FIFO_RX_EMPTY    (1u << 1)
#define NRF_FIFO_TX_FULL     (1u << 6)

/* ---- enums ---- */
typedef enum { NRF24_DR_250K = 0, NRF24_DR_1M, NRF24_DR_2M } nrf24_dr_t;

/* El PA del modulo amplifica, pero el nRF solo emite hasta 0 dBm */
typedef enum {
    NRF24_PA_M18DBM = 0, NRF24_PA_M12DBM, NRF24_PA_M6DBM, NRF24_PA_0DBM
} nrf24_pa_t;

typedef enum { NRF24_CRC_OFF = 0, NRF24_CRC_8BIT, NRF24_CRC_16BIT } nrf24_crc_t;

/* ---- config por radio ---- */
typedef struct {
    const char       *name;
    spi_host_device_t host;
    gpio_num_t        sck, miso, mosi, ce, cs, irq;
    uint32_t          clock_hz;     /* 8 MHz: el nRF aguanta 10 MHz maximo */
} nrf24_cfg_t;

#define NRF24_CFG_DEFAULT()                       \
    {                                             \
        .name = "nrf", .host = SPI2_HOST,         \
        .sck = GPIO_NUM_12, .miso = GPIO_NUM_13,  \
        .mosi = GPIO_NUM_11, .ce = GPIO_NUM_9,    \
        .cs = GPIO_NUM_10, .irq = GPIO_NUM_NC,    \
        .clock_hz = 8000000                       \
    }

/* ---- API ---- */
typedef struct nrf24_dev nrf24_dev_t;

nrf24_dev_t *nrf24_create(const nrf24_cfg_t *cfg);
void         nrf24_destroy(nrf24_dev_t *d);
bool         nrf24_is_up(const nrf24_dev_t *d);
esp_err_t    nrf24_self_test(nrf24_dev_t *d);
const char  *nrf24_name(const nrf24_dev_t *d);
uint8_t      nrf24_index(const nrf24_dev_t *d);

uint8_t nrf24_status(nrf24_dev_t *d);
uint8_t nrf24_read_reg(nrf24_dev_t *d, uint8_t reg);
void    nrf24_write_reg(nrf24_dev_t *d, uint8_t reg, uint8_t val);
void    nrf24_read_reg_mb(nrf24_dev_t *d, uint8_t reg, uint8_t *val, size_t len);
void    nrf24_write_reg_mb(nrf24_dev_t *d, uint8_t reg, const uint8_t *val, size_t len);
uint8_t nrf24_cmd(nrf24_dev_t *d, uint8_t cmd);

void    nrf24_flush_rx(nrf24_dev_t *d);
void    nrf24_flush_tx(nrf24_dev_t *d);
void    nrf24_clear_irq(nrf24_dev_t *d, uint8_t mask);
void    nrf24_lock(nrf24_dev_t *d);
void    nrf24_unlock(nrf24_dev_t *d);

void     nrf24_set_ce(nrf24_dev_t *d, bool level);
void     nrf24_power_up(nrf24_dev_t *d, bool up);
void     nrf24_set_channel(nrf24_dev_t *d, uint8_t ch);
uint8_t  nrf24_channel(const nrf24_dev_t *d);
uint32_t nrf24_channel_to_freq(uint8_t ch);
void     nrf24_set_data_rate(nrf24_dev_t *d, nrf24_dr_t dr);
void     nrf24_set_pa(nrf24_dev_t *d, nrf24_pa_t pa);
void     nrf24_set_address_width(nrf24_dev_t *d, uint8_t bytes);
void     nrf24_set_payload_size(nrf24_dev_t *d, uint8_t size);
void     nrf24_set_auto_ack(nrf24_dev_t *d, bool en);
void     nrf24_set_crc(nrf24_dev_t *d, nrf24_crc_t crc);
void     nrf24_set_retr(nrf24_dev_t *d, uint16_t delay_us, uint8_t count);
void     nrf24_enable_dynamic_payload(nrf24_dev_t *d, bool en);

void nrf24_open_writing_pipe(nrf24_dev_t *d, const uint8_t *addr);
void nrf24_open_reading_pipe(nrf24_dev_t *d, uint8_t pipe, const uint8_t *addr);
void nrf24_start_listening(nrf24_dev_t *d);
void nrf24_stop_listening(nrf24_dev_t *d);
bool nrf24_available(nrf24_dev_t *d);
bool nrf24_carrier_detect(nrf24_dev_t *d);
bool nrf24_write(nrf24_dev_t *d, const uint8_t *buf, size_t len, bool no_ack);
bool nrf24_wait_rx(nrf24_dev_t *d, uint32_t timeout_ms);

/* portadora continua */
void nrf24_cw_start(nrf24_dev_t *d, uint8_t ch, nrf24_pa_t pa);
void nrf24_cw_hop(nrf24_dev_t *d, uint8_t ch);
void nrf24_cw_stop(nrf24_dev_t *d);
bool nrf24_cw_active(const nrf24_dev_t *d);

void nrf24_busy_delay_us(uint32_t us);
void nrf24_dump(const nrf24_dev_t *d);

#ifdef __cplusplus
}
#endif
