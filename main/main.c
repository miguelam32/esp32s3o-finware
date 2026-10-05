/*
 * radio2_only_probe.c - diagnostico EN VIVO, solo para el radio que NO
 * responde (SPI3: CE=4 CS=5 SCK=7 MOSI=6 MISO=8).
 *
 * El radio 1 queda totalmente fuera de la ecuacion: ni se inicializa
 * su GPIO ni su SPI. Este programa escribe y relee dos registros
 * distintos en loop rapido (~150 ms) e imprime el resultado, para que
 * muevas cables/pines en caliente y veas el cambio en el monitor al
 * toque.
 *
 * Tambien baja el clock SPI a 1 MHz (en vez de los 10 MHz de antes)
 * por si el problema es de integridad de señal con cableado largo o
 * flojo, no del chip en si.
 *
 * Reemplaza tu main.c por este archivo para correr la prueba; el
 * mismo CMakeLists.txt que ya armaste sirve tal cual.
 */
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "r2";

/* ---------- pinout radio 2 (SPI3) ---------- */
#define R2_HOST SPI3_HOST
#define R2_CE   GPIO_NUM_4
#define R2_CS   GPIO_NUM_5
#define R2_SCK  GPIO_NUM_7
#define R2_MOSI GPIO_NUM_6
#define R2_MISO GPIO_NUM_8

/* ---------- registros nRF24L01+ usados ---------- */
#define R_SETUP_AW 0x03   /* valor de fabrica: 0x03 = direcciones de 5 bytes */
#define R_RF_CH    0x05
#define R_STATUS   0x07
#define CMD_W_REG  0x20
#define CMD_NOP    0xFF

static spi_device_handle_t spi;

static esp_err_t xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    spi_transaction_t t = { .length = len * 8, .tx_buffer = tx, .rx_buffer = rx };
    return spi_device_polling_transmit(spi, &t);
}

static uint8_t rd(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x1F), CMD_NOP };
    uint8_t rx[2] = { 0 };
    xfer(tx, rx, 2);
    return rx[1];
}

static void wr(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(CMD_W_REG | (reg & 0x1F)), val };
    xfer(tx, NULL, 2);
}

void app_main(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << R2_CE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(R2_CE, 0);

    spi_bus_config_t bus = {
        .mosi_io_num = R2_MOSI, .miso_io_num = R2_MISO, .sclk_io_num = R2_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 8,
    };
    esp_err_t e = spi_bus_initialize(R2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(e));
    }

    spi_device_interface_config_t dev = {
        .mode = 0,
        .clock_speed_hz = 1000000,          /* 1 MHz: SPI "normal", nada agresivo */
        .spics_io_num = R2_CS,
        .queue_size = 1,
        .cs_ena_pretrans = 2,
        .cs_ena_posttrans = 2,
    };
    esp_err_t e2 = spi_bus_add_device(R2_HOST, &dev, &spi);
    if (e2 != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(e2));
    }

    ESP_LOGW(TAG, "arrancando prueba en vivo (CE=%d CS=%d SCK=%d MOSI=%d MISO=%d) a 1 MHz",
             (int)R2_CE, (int)R2_CS, (int)R2_SCK, (int)R2_MOSI, (int)R2_MISO);
    ESP_LOGW(TAG, "mueve cables ahora y mira si algun OK aparece");

    bool toggle = false;
    for (;;) {
        uint8_t expect = toggle ? 0x5A : 0xA5;   /* alterna para no confundir con un valor pegado */
        wr(R_RF_CH, expect);
        uint8_t back_ch = rd(R_RF_CH);

        wr(R_SETUP_AW, 0x03);
        uint8_t back_aw = rd(R_SETUP_AW);

        uint8_t raw_status = rd(R_STATUS);       /* si varia, hay vida en el bus aunque las otras fallen */

        ESP_LOGW(TAG,
                 "RF_CH: escribi 0x%02X leido 0x%02X %s | SETUP_AW leido 0x%02X %s | STATUS 0x%02X",
                 expect, back_ch, (back_ch == expect) ? "OK" : "falla",
                 back_aw, (back_aw == 0x03) ? "OK" : "falla",
                 raw_status);

        toggle = !toggle;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

