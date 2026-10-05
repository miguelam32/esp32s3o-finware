#include "nrf24_hal.h"

static const char *TAG = "NRF24";

// Transacción SPI
static esp_err_t nrf24_spi_xfer(nrf24_dev_t *dev, 
                                 const uint8_t *tx, uint8_t *rx, size_t len) {
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    return spi_device_transmit(dev->spi, &t);
}

esp_err_t nrf24_init(nrf24_dev_t *dev, const nrf24_pins_t *pins) {
    esp_err_t err;
    
    if (!dev || !pins) return ESP_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(nrf24_dev_t));
    memcpy(&dev->pins, pins, sizeof(nrf24_pins_t));

    // Inicializar bus SPI
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

    // Añadir dispositivo NRF24
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = pins->pin_csn,
        .queue_size = 7,
    };

    err = spi_bus_add_device(pins->spi_host, &devcfg, &dev->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(err));
        return err;
    }

    // Configurar CE
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

    // Configuración inicial
    nrf24_write_reg(dev, NRF24_REG_CONFIG, 
                    NRF24_CFG_PWR_UP | 0x70);
    nrf24_write_reg(dev, NRF24_REG_RF_CH, 50);
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, 
                    NRF24_DR_2MBPS | NRF24_PA_MAX);

    dev->current_channel = 50;
    dev->current_pa = NRF24_PA_MAX;
    dev->current_dr = NRF24_DR_2MBPS;

    if (!nrf24_is_present(dev)) {
        ESP_LOGW(TAG, "NRF24 host=%d NOT DETECTED (revisa cables)", 
                 pins->spi_host);
    } else {
        ESP_LOGI(TAG, "NRF24 host=%d OK ✓", pins->spi_host);
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
    
    uint8_t rf_setup = NRF24_RF_CONT_WAVE | NRF24_RF_PLL_LOCK | 
                       dev->current_pa | dev->current_dr;
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, rf_setup);
    
    gpio_set_level(dev->pins.pin_ce, 1);
    return ESP_OK;
}

esp_err_t nrf24_stop_carrier(nrf24_dev_t *dev) {
    gpio_set_level(dev->pins.pin_ce, 0);
    uint8_t rf_setup = nrf24_read_reg(dev, NRF24_REG_RF_SETUP);
    rf_setup &= ~(NRF24_RF_CONT_WAVE | NRF24_RF_PLL_LOCK);
    return nrf24_write_reg(dev, NRF24_REG_RF_SETUP, rf_setup);
}

esp_err_t nrf24_carrier_hop(nrf24_dev_t *dev, uint8_t new_channel) {
    nrf24_stop_carrier(dev);
    
    // Power cycle (fix para módulos PA+LNA)
    uint8_t cfg = nrf24_read_reg(dev, NRF24_REG_CONFIG);
    nrf24_write_reg(dev, NRF24_REG_CONFIG, cfg & ~NRF24_CFG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(2));
    nrf24_write_reg(dev, NRF24_REG_CONFIG, cfg | NRF24_CFG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    return nrf24_start_carrier(dev, new_channel);
}

bool nrf24_is_present(nrf24_dev_t *dev) {
    uint8_t test = 0x03;
    nrf24_write_reg(dev, NRF24_REG_SETUP_AW, test);
    uint8_t read = nrf24_read_reg(dev, NRF24_REG_SETUP_AW);
    return (read == test);
}
