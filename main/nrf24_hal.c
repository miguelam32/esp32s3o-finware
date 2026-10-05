#include "nrf24_hal.h"
#include "esp_random.h"
#include "rom/ets_sys.h"

static const char *TAG = "NRF24";

static esp_err_t nrf24_spi_xfer(nrf24_dev_t *dev,
                                 const uint8_t *tx, uint8_t *rx, size_t len) {
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev->spi, &t);
}

static void nrf24_apply_rfsetup(nrf24_dev_t *dev) {
    uint8_t rf;
    if (dev->spam) {
        rf = dev->current_pa | dev->current_dr;                              // 0x0E
    } else {
        rf = NRF24_RF_CONT_WAVE | NRF24_RF_PLL_LOCK |
             dev->current_pa | dev->current_dr;                              // 0x9E
    }
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, rf);
}

// NOP devuelve STATUS en el byte de MISO
uint8_t nrf24_get_status(nrf24_dev_t *dev) {
    uint8_t tx = NRF24_CMD_NOP;
    uint8_t rx = 0;
    nrf24_spi_xfer(dev, &tx, &rx, 1);
    return rx;
}

esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_pins_t *pins) {
    esp_err_t err;

    if (!dev || !pins) return ESP_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(nrf24_dev_t));
    memcpy(&dev->pins, pins, sizeof(nrf24_pins_t));

    spi_bus_config_t buscfg = {
        .mosi_io_num = pins->pin_mosi,
        .miso_io_num = pins->pin_miso,
        .sclk_io_num = pins->pin_sck,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 33,
    };

    err = spi_bus_initialize(pins->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,   // 10 MHz — máximo del nRF24
        .mode = 0,
        .spics_io_num = pins->pin_csn,
        .queue_size = 7,
    };

    err = spi_bus_add_device(pins->spi_host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        return err;
    }

    gpio_config_t ce_conf = {
        .pin_bit_mask = (1ULL << pins->pin_ce),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ce_conf);
    gpio_set_level(pins->pin_ce, 0);

    dev->initialized = true;
    dev->spam = false;
    dev->current_pa = NRF24_PA_MAX;
    dev->current_dr = NRF24_DR_2MBPS;

    // Config TX puro
    nrf24_write_reg(dev, NRF24_REG_CONFIG,
                    NRF24_CFG_PWR_UP | NRF24_CFG_CRC_2B | NRF24_CFG_EN_CRC);
    nrf24_write_reg(dev, NRF24_REG_EN_AA, 0x00);      // sin auto-ACK
    nrf24_write_reg(dev, NRF24_REG_EN_RXADDR, 0x00);  // sin pipes RX
    nrf24_write_reg(dev, NRF24_REG_SETUP_RETR, 0x00); // sin retransmisión
    nrf24_write_reg(dev, NRF24_REG_SETUP_AW, 0x03);   // addr 5 bytes
    nrf24_write_reg(dev, NRF24_REG_RF_CH, 50);
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP,
                    dev->current_pa | dev->current_dr);
    dev->current_channel = 50;

    uint8_t flush = NRF24_CMD_FLUSH_TX;
    nrf24_spi_xfer(dev, &flush, NULL, 1);

    // ═══════════ TEST DE COMUNICACIÓN ═══════════
    dev->present = nrf24_is_present(dev);
    uint8_t st = nrf24_get_status(dev);

    if (!dev->present) {
        ESP_LOGE(TAG, "NRF24 host=%d NO RESPONDE (STATUS=0x%02X)", pins->spi_host, st);
        if (st == 0x00) {
            ESP_LOGE(TAG, "  -> 0x00 = sin energia / CSN o CE mal / módulo muerto");
        } else if (st == 0xFF) {
            ESP_LOGE(TAG, "  -> 0xFF = MISO colgado (cable roto, MISO/MOSI invertidos)");
        } else {
            ESP_LOGE(TAG, "  -> STATUS inesperado: cables largos? bajá el clock a 1MHz");
        }
    } else {
        ESP_LOGI(TAG, "NRF24 host=%d OK, PA_MAX armed (STATUS=0x%02X)",
                 pins->spi_host, st);
    }
    return ESP_OK;
}

uint8_t nrf24_read_reg(nrf24_dev_t *dev, uint8_t reg) {
    uint8_t tx[2] = { NRF24_CMD_R_REG | (reg & 0x1F), NRF24_CMD_NOP };
    uint8_t rx[2] = { 0 };
    nrf24_spi_xfer(dev, tx, rx, 2);
    return rx[1];
}

esp_err_t nrf24_write_reg(nrf24_dev_t *dev, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { NRF24_CMD_W_REG | (reg & 0x1F), value };
    return nrf24_spi_xfer(dev, tx, NULL, 2);
}

esp_err_t nrf24_set_channel(nrf24_dev_t *dev, uint8_t channel) {
    if (channel > 125) return ESP_ERR_INVALID_ARG;
    dev->current_channel = channel;
    return nrf24_write_reg(dev, NRF24_REG_RF_CH, channel);
}

esp_err_t nrf24_start_carrier(nrf24_dev_t *dev, uint8_t channel) {
    nrf24_set_channel(dev, channel);
    nrf24_apply_rfsetup(dev);
    gpio_set_level(dev->pins.pin_ce, 1);
    ets_delay_us(200);
    return ESP_OK;
}

esp_err_t nrf24_stop_carrier(nrf24_dev_t *dev) {
    gpio_set_level(dev->pins.pin_ce, 0);
    ets_delay_us(20);
    uint8_t rf = nrf24_read_reg(dev, NRF24_REG_RF_SETUP);
    rf &= ~(NRF24_RF_CONT_WAVE | NRF24_RF_PLL_LOCK);
    return nrf24_write_reg(dev, NRF24_REG_RF_SETUP, rf);
}

// Hop rápido: SIN power-cycle
esp_err_t nrf24_carrier_hop(nrf24_dev_t *dev, uint8_t new_channel) {
    if (new_channel > 125) new_channel = 125;
    dev->current_channel = new_channel;
    nrf24_write_reg(dev, NRF24_REG_RF_CH, new_channel);
    ets_delay_us(150);   // el PLL lockea en ~130us
    return ESP_OK;
}

// ═════════ SALTO ALEATORIO ═════════
void nrf24_hop_random(nrf24_dev_t *dev, uint8_t ch_min, uint8_t ch_max) {
    if (ch_max > 125) ch_max = 125;
    uint32_t span = ch_max - ch_min + 1;
    uint8_t ch = ch_min + (uint8_t)(esp_random() % span);
    dev->current_channel = ch;

    if (dev->spam) {
        nrf24_pulse_tx(dev, ch);
    } else {
        nrf24_write_reg(dev, NRF24_REG_RF_CH, ch);
        ets_delay_us(150);
    }
}

// ═════════ MODO SPAM (clones Si24R1) ═════════
void nrf24_set_spam(nrf24_dev_t *dev, bool enable) {
    gpio_set_level(dev->pins.pin_ce, 0);
    ets_delay_us(50);
    dev->spam = enable;

    if (enable) {
        uint8_t buf[33];
        buf[0] = NRF24_CMD_W_TX_PAYLOAD;
        for (int i = 1; i <= 32; i++) buf[i] = (uint8_t)esp_random();
        nrf24_spi_xfer(dev, buf, NULL, 33);
        uint8_t reuse = NRF24_CMD_REUSE_TX_PL;
        nrf24_spi_xfer(dev, &reuse, NULL, 1);
    } else {
        uint8_t flush = NRF24_CMD_FLUSH_TX;
        nrf24_spi_xfer(dev, &flush, NULL, 1);
    }

    nrf24_apply_rfsetup(dev);
    gpio_set_level(dev->pins.pin_ce, 1);
    ets_delay_us(200);
}

esp_err_t nrf24_pulse_tx(nrf24_dev_t *dev, uint8_t channel) {
    nrf24_write_reg(dev, NRF24_REG_RF_CH, channel);
    dev->current_channel = channel;
    gpio_set_level(dev->pins.pin_ce, 0);
    ets_delay_us(15);
    gpio_set_level(dev->pins.pin_ce, 1);   // dispara paquete
    ets_delay_us(260);                     // ~32B a 2Mbps
    return ESP_OK;
}

bool nrf24_is_present(nrf24_dev_t *dev) {
    uint8_t test = 0x03;
    nrf24_write_reg(dev, NRF24_REG_SETUP_AW, test);
    uint8_t read = nrf24_read_reg(dev, NRF24_REG_SETUP_AW);
    return (read == test);
}
