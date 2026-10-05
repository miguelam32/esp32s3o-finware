#ifndef NRF24_HAL_H
#define NRF24_HAL_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <string.h>

// ═══════════ REGISTROS ═══════════
#define NRF24_REG_CONFIG       0x00
#define NRF24_REG_EN_AA        0x01
#define NRF24_REG_EN_RXADDR    0x02
#define NRF24_REG_SETUP_AW     0x03
#define NRF24_REG_SETUP_RETR   0x04
#define NRF24_REG_RF_CH        0x05
#define NRF24_REG_RF_SETUP     0x06
#define NRF24_REG_STATUS       0x07
#define NRF24_REG_RX_PW_P0     0x11

// ═══════════ COMANDOS SPI ═══════════
#define NRF24_CMD_R_REG         0x00
#define NRF24_CMD_W_REG         0x20
#define NRF24_CMD_NOP           0xFF
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_REUSE_TX_PL   0xE3

// ═══════════ BITS CONFIG ═══════════
#define NRF24_CFG_PWR_UP        0x02
#define NRF24_CFG_CRC_2B        0x04
#define NRF24_CFG_EN_CRC        0x08
#define NRF24_CFG_PRIM_RX       0x01

// ═══════════ BITS RF_SETUP ═══════════
#define NRF24_RF_CONT_WAVE      0x80
#define NRF24_RF_PLL_LOCK       0x10
#define NRF24_RF_DR_HIGH        0x08
#define NRF24_RF_PWR_MASK       0x06

// ═══════════ POTENCIA ═══════════
#define NRF24_PA_MIN   (0 << 1)   // -18 dBm
#define NRF24_PA_LOW   (1 << 1)   // -12 dBm
#define NRF24_PA_HIGH  (2 << 1)   //  -6 dBm
#define NRF24_PA_MAX   (3 << 1)   //   0 dBm -> con tu PA+LNA ~+20 dBm

// ═══════════ DATA RATE ═══════════
#define NRF24_DR_1MBPS   0x00
#define NRF24_DR_2MBPS   NRF24_RF_DR_HIGH

// ═══════════ ESTRUCTURAS ═══════════
typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t pin_ce;
    gpio_num_t pin_csn;
    gpio_num_t pin_sck;
    gpio_num_t pin_mosi;
    gpio_num_t pin_miso;
    gpio_num_t pin_irq;
} nrf24_pins_t;

typedef struct {
    nrf24_pins_t pins;
    spi_device_handle_t spi;
    bool initialized;
    bool present;           // ← NUEVO: respondió por SPI (STATUS != 0x00/0xFF)
    bool spam;              // true = ráfaga de paquetes (clones Si24R1)
    uint8_t current_channel;
    uint8_t current_pa;
    uint8_t current_dr;
} nrf24_dev_t;

// ═══════════ API ═══════════
esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_pins_t *pins);
uint8_t nrf24_read_reg(nrf24_dev_t *dev, uint8_t reg);
esp_err_t nrf24_write_reg(nrf24_dev_t *dev, uint8_t reg, uint8_t value);
esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel);
esp_err_t nrf24_start_carrier(nrf24_dev_t *dev, uint8_t channel);
esp_err_t nrf24_stop_carrier(nrf24_dev_t *dev);
esp_err_t nrf24_carrier_hop(nrf24_dev_t *dev, uint8_t new_channel);
bool nrf24_is_present(nrf24_dev_t *dev);
uint8_t nrf24_get_status(nrf24_dev_t *dev);   // ← NUEVO

// ═══════════ JAMMER ═══════════
void     nrf24_set_spam(nrf24_dev_t *dev, bool enable);
esp_err_t nrf24_pulse_tx(nrf24_dev_t *dev, uint8_t channel);
void     nrf24_hop_random(nrf24_dev_t *dev, uint8_t ch_min, uint8_t ch_max);

#endif // NRF24_HAL_H
