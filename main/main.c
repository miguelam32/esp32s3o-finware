/*
 * main.c
 * ESP32-S3 N16R8 (ESP-IDF v6.0.3) — 3x nRF24L01 hopping TX + LED RGB (GPIO48)
 *
 * Mismo código de hopping de nrf24_tri_tx_hopping.c (pinout SPI2/SPI3,
 * chequeo de presencia, monitor de estado). Lo único nuevo es el LED:
 *
 *   - Mientras los 3 NRF estén presentes Y transmitiendo (activos==3),
 *     el WS2812 de GPIO48 hace un ciclo arcoíris a 5 Hz: un color nuevo
 *     cada 200 ms, recorriendo el círculo de matiz (hue) completo.
 *   - Si falta alguno (activos<3) o ninguno respondió, el LED queda
 *     apagado — así el LED mismo te dice de un vistazo si están los 3
 *     saltando o no, sin mirar el monitor serial.
 *
 * Requiere el componente led_strip (RMT). Agregalo con:
 *   idf.py add-dependency "espressif/led_strip^2.5.5"
 * o a mano en main/idf_component.yml:
 *   dependencies:
 *     espressif/led_strip: "^2.5.5"
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_random.h"
#include "rom/ets_sys.h"
#include "led_strip.h"

static const char *TAG = "NRF24_TRI_HIBRIDO";

// ---------- LED RGB (WS2812) ----------
#define LED_GPIO        GPIO_NUM_48
#define LED_HZ          5                 // ciclos de color por segundo
#define LED_PERIOD_MS   (1000 / LED_HZ)
static led_strip_handle_t led;

// ---------- Bus SPI2 (NRF1 + NRF2, compartido) ----------
#define PIN_SCK2   GPIO_NUM_12
#define PIN_MOSI2  GPIO_NUM_11
#define PIN_MISO2  GPIO_NUM_13

// ---------- Bus SPI3 (exclusivo de NRF3) ----------
#define PIN_SCK3   GPIO_NUM_18
#define PIN_MOSI3  GPIO_NUM_8
#define PIN_MISO3  GPIO_NUM_7

// ---------- Registros / comandos nRF24L01 ----------
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_TX_ADDR     0x10
#define NRF_FEATURE     0x1D
#define NRF_CD_RPD      0x09

#define CMD_R_REGISTER          0x00
#define CMD_W_REGISTER          0x20
#define CMD_W_TX_PAYLOAD_NOACK  0xB0
#define CMD_FLUSH_TX            0xE1

#define CONFIG_MODO_TX  0x0E
#define CONFIG_MODO_RX  0x0F
#define RF_SETUP_VALOR  0x0E

// ---------- Reparto del espectro: 126 canales / 3 radios = 42 c/u ----------
#define NUM_RADIOS            3
#define CHANNELS_PER_BLOQUE   42
#define DWELL_US              350
#define STATUS_MS             2000

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t ce_pin;
    gpio_num_t irq_pin;
    uint8_t base_channel;
    uint8_t hop_table[CHANNELS_PER_BLOQUE];
    uint8_t hop_count;
    uint8_t blacklist[CHANNELS_PER_BLOQUE];
    uint32_t hop_index;
    uint32_t packet_counter;
    bool presente;
    volatile uint8_t canal_actual;
    uint32_t vueltas;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .ce_pin = GPIO_NUM_4, .irq_pin = GPIO_NUM_15, .base_channel = 0 * CHANNELS_PER_BLOQUE },
    { .ce_pin = GPIO_NUM_5, .irq_pin = GPIO_NUM_16, .base_channel = 1 * CHANNELS_PER_BLOQUE },
    { .ce_pin = GPIO_NUM_1, .irq_pin = GPIO_NUM_2,  .base_channel = 2 * CHANNELS_PER_BLOQUE },
};

static const gpio_num_t CSN_PINS[NUM_RADIOS] = { GPIO_NUM_10, GPIO_NUM_9, GPIO_NUM_21 };
static const spi_host_device_t HOST_FOR_RADIO[NUM_RADIOS] = { SPI2_HOST, SPI2_HOST, SPI3_HOST };
static const char *BUS_NAME[NUM_RADIOS] = { "SPI2", "SPI2", "SPI3" };

static esp_timer_handle_t hop_timer;
static TaskHandle_t radio3_task_handle = NULL;
static volatile int g_activos = 0; // cuántos NRF están presentes; lo lee la tarea del LED

#define RADIO3_TASK_PRIO   10
#define RADIO3_TASK_STACK  2048

// ---------- SPI de bajo nivel ----------
static inline void spi_cmd(spi_device_handle_t dev, const uint8_t *tx, uint8_t *rx, size_t len) {
    spi_transaction_t t = { .length = len * 8, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev, &t);
}

static void nrf_write_reg(spi_device_handle_t dev, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = { (uint8_t)(CMD_W_REGISTER | reg), value };
    spi_cmd(dev, tx, NULL, 2);
}

static void nrf_write_reg_multi(spi_device_handle_t dev, uint8_t reg, const uint8_t *buf, uint8_t len) {
    uint8_t tx[1 + 5];
    tx[0] = CMD_W_REGISTER | reg;
    memcpy(&tx[1], buf, len);
    spi_cmd(dev, tx, NULL, 1 + len);
}

static void nrf_send_cmd(spi_device_handle_t dev, uint8_t cmd) {
    uint8_t tx = cmd;
    spi_cmd(dev, &tx, NULL, 1);
}

static uint8_t nrf_read_reg(spi_device_handle_t dev, uint8_t reg) {
    uint8_t tx[2] = { (uint8_t)(CMD_R_REGISTER | reg), 0xFF };
    uint8_t rx[2] = { 0, 0 };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(dev, &t);
    return rx[1];
}

static void nrf_write_payload_noack(spi_device_handle_t dev, const uint8_t *payload, uint8_t len) {
    uint8_t tx[1 + 8];
    tx[0] = CMD_W_TX_PAYLOAD_NOACK;
    memcpy(&tx[1], payload, len);
    spi_cmd(dev, tx, NULL, 1 + len);
}

// ---------- Tabla de saltos ----------
static void reshuffle_hop_table(radio_t *r) {
    for (int i = r->hop_count - 1; i > 0; i--) {
        int j = esp_random() % (i + 1);
        uint8_t tmp = r->hop_table[i];
        r->hop_table[i] = r->hop_table[j];
        r->hop_table[j] = tmp;
    }
}

static void generar_hop_table(radio_t *r) {
    uint8_t libres[CHANNELS_PER_BLOQUE];
    int n = 0;
    for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) {
        if (!r->blacklist[i]) libres[n++] = r->base_channel + i;
    }
    if (n == 0) {
        for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) libres[i] = r->base_channel + i;
        n = CHANNELS_PER_BLOQUE;
    }
    memcpy(r->hop_table, libres, n);
    r->hop_count = (uint8_t)n;
    r->hop_index = 0;
    reshuffle_hop_table(r);
}

static bool canal_ocupado(radio_t *r, uint8_t canal) {
    nrf_write_reg(r->spi, NRF_RF_CH, canal);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_RX);
    gpio_set_level(r->ce_pin, 1);
    ets_delay_us(200);
    uint8_t cd = nrf_read_reg(r->spi, NRF_CD_RPD) & 0x01;
    gpio_set_level(r->ce_pin, 0);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    return cd != 0;
}

static void escanear_bloque(radio_t *r) {
    int sucios = 0;
    for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) {
        bool ocupado = canal_ocupado(r, r->base_channel + i);
        r->blacklist[i] = ocupado ? 1 : 0;
        if (ocupado) sucios++;
    }
    ESP_LOGI(TAG, "scan radio base_ch=%d: %d/%d canales sucios",
             r->base_channel, sucios, CHANNELS_PER_BLOQUE);
}

// ---------- Init de buses + de cada radio ----------
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 32,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
}

static void radio_init(radio_t *r, int idx) {
    gpio_set_direction(r->ce_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(r->ce_pin, 0);
    gpio_set_direction(r->irq_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(r->irq_pin, GPIO_PULLUP_ONLY);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = CSN_PINS[idx],
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(HOST_FOR_RADIO[idx], &devcfg, &r->spi));

    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r->spi, NRF_SETUP_AW, 0x03);
    nrf_write_reg(r->spi, NRF_RF_SETUP, RF_SETUP_VALOR);
    nrf_write_reg(r->spi, NRF_FEATURE, 0x01);

    uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, (uint8_t)(0xE1 + idx) };
    nrf_write_reg_multi(r->spi, NRF_TX_ADDR, addr, 5);
}

static bool nrf_presente(radio_t *r, int idx) {
    uint8_t cfg = nrf_read_reg(r->spi, NRF_CONFIG);
    uint8_t rf  = nrf_read_reg(r->spi, NRF_RF_SETUP);
    r->presente = (cfg == CONFIG_MODO_TX) && (rf == RF_SETUP_VALOR);

    if (r->presente) {
        ESP_LOGI(TAG, "NRF%d OK  [%s CSN%d CE%d] CONFIG=0x%02X RF_SETUP=0x%02X",
                 idx + 1, BUS_NAME[idx], (int)CSN_PINS[idx], (int)r->ce_pin, cfg, rf);
    } else {
        ESP_LOGE(TAG, "NRF%d NO RESPONDE  [%s CSN%d CE%d] CONFIG=0x%02X RF_SETUP=0x%02X",
                 idx + 1, BUS_NAME[idx], (int)CSN_PINS[idx], (int)r->ce_pin, cfg, rf);
        ESP_LOGE(TAG, "NRF%d -> revisa MISO, CSN, 3.3V/GND y el capacitor del modulo. Queda DESACTIVADO.",
                 idx + 1);
    }
    return r->presente;
}

static inline void preparar_y_transmitir_payload(radio_t *r, uint8_t radio_id) {
    r->hop_index++;
    if (r->hop_index >= r->hop_count) {
        r->hop_index = 0;
        r->vueltas++;
        reshuffle_hop_table(r);
    }
    uint8_t canal = r->hop_table[r->hop_index];
    r->canal_actual = canal;
    nrf_write_reg(r->spi, NRF_RF_CH, canal);

    uint8_t payload[8];
    payload[0] = radio_id;
    payload[1] = (uint8_t)r->hop_index;
    memcpy(&payload[2], &r->packet_counter, 4);
    payload[6] = 0xAA;
    payload[7] = 0xAA;
    nrf_write_payload_noack(r->spi, payload, 8);
    r->packet_counter++;
}

static void IRAM_ATTR on_hop_timer(void *arg) {
    BaseType_t hp_woken = pdFALSE;
    if (radios[2].presente) {
        vTaskNotifyGiveFromISR(radio3_task_handle, &hp_woken);
    }

    bool alguno = false;
    for (int i = 0; i < 2; i++) {
        if (!radios[i].presente) continue;
        preparar_y_transmitir_payload(&radios[i], (uint8_t)i);
        alguno = true;
    }

    if (alguno) {
        for (int i = 0; i < 2; i++) if (radios[i].presente) gpio_set_level(radios[i].ce_pin, 1);
        ets_delay_us(15);
        for (int i = 0; i < 2; i++) if (radios[i].presente) gpio_set_level(radios[i].ce_pin, 0);
        for (int i = 0; i < 2; i++) {
            if (radios[i].presente) nrf_send_cmd(radios[i].spi, CMD_FLUSH_TX);
        }
    }

    if (hp_woken) portYIELD_FROM_ISR();
}

static void radio3_task(void *arg) {
    radio_t *r = &radios[2];
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        preparar_y_transmitir_payload(r, 2);
        gpio_set_level(r->ce_pin, 1);
        ets_delay_us(15);
        gpio_set_level(r->ce_pin, 0);
        nrf_send_cmd(r->spi, CMD_FLUSH_TX);
    }
}

#define REESCANEO_MS  5000

static void tarea_reescaneo(void *arg) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(REESCANEO_MS));
        esp_timer_stop(hop_timer);
        for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) escanear_bloque(&radios[i]);
        for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) generar_hop_table(&radios[i]);
        esp_timer_start_periodic(hop_timer, DWELL_US);
    }
}

// ---------- Monitor de estado por serial ----------
#define C_VERDE  "\033[32m"
#define C_ROJO   "\033[31m"
#define C_AMAR   "\033[33m"
#define C_NEG    "\033[1m"
#define C_RESET  "\033[0m"

static void mostrar_estado(uint32_t t_seg, uint32_t prev[NUM_RADIOS]) {
    printf("\n" C_NEG "──── NRF24 TX hopping · t=%lus ────" C_RESET "\n", (unsigned long)t_seg);
    for (int i = 0; i < NUM_RADIOS; i++) {
        radio_t *r = &radios[i];
        uint32_t ahora = r->packet_counter;
        uint32_t pps = (uint32_t)(((uint64_t)(ahora - prev[i]) * 1000) / STATUS_MS);
        prev[i] = ahora;

        if (!r->presente) {
            printf("NRF%d [%s CSN%-2d CE%-2d] " C_ROJO "✖ NO RESPONDE" C_RESET "\n",
                   i + 1, BUS_NAME[i], (int)CSN_PINS[i], (int)r->ce_pin);
        } else if (pps == 0) {
            printf("NRF%d [%s CSN%-2d CE%-2d] " C_AMAR "▲ SIN PAQUETES" C_RESET "  pkts=%lu\n",
                   i + 1, BUS_NAME[i], (int)CSN_PINS[i], (int)r->ce_pin, (unsigned long)ahora);
        } else {
            printf("NRF%d [%s CSN%-2d CE%-2d] " C_VERDE "● TX" C_RESET
                   "  pkts=%-9lu %5lu/s  canal=%3u  libres=%2u/%d  vueltas=%lu\n",
                   i + 1, BUS_NAME[i], (int)CSN_PINS[i], (int)r->ce_pin,
                   (unsigned long)ahora, (unsigned long)pps,
                   (unsigned)r->canal_actual, (unsigned)r->hop_count,
                   CHANNELS_PER_BLOQUE, (unsigned long)r->vueltas);
        }
    }
}

// ---------- LED RGB arcoíris (GPIO48) ----------
// HSV->RGB simple, hue 0-359. Satura y brillo fijos (color vivo, no muy
// fuerte para no encandilar). Se llama una vez por paso (5 Hz).
static void hsv_to_rgb(int hue, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t region = hue / 60;
    uint8_t rem = (hue % 60) * 255 / 60;
    uint8_t v = 60, p = 0, q = 60 - (60 * rem / 255), t = (60 * rem / 255);
    switch (region) {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default: *r = v; *g = p; *b = q; break;
    }
}

static void led_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led));
    led_strip_clear(led);
}

// Tarea del LED: corre siempre, independiente del hopping. Mientras
// g_activos == NUM_RADIOS (los 3 NRF presentes y por lo tanto saltando),
// hace un paso de arcoíris cada 200 ms (5 Hz). Si falta alguno, apaga
// el LED y espera a que vuelvan a estar los 3.
static void led_task(void *arg) {
    int hue = 0;
    while (1) {
        if (g_activos == NUM_RADIOS) {
            uint8_t r, g, b;
            hsv_to_rgb(hue, &r, &g, &b);
            led_strip_set_pixel(led, 0, r, g, b);
            led_strip_refresh(led);
            hue = (hue + 20) % 360; // 360/20 = 18 pasos -> vuelta completa cada ~3.6s a 5Hz
        } else {
            led_strip_clear(led);
        }
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

void app_main(void) {
    led_init();

    bus_init(SPI2_HOST, PIN_SCK2, PIN_MOSI2, PIN_MISO2);
    bus_init(SPI3_HOST, PIN_SCK3, PIN_MOSI3, PIN_MISO3);

    for (int i = 0; i < NUM_RADIOS; i++) radio_init(&radios[i], i);
    vTaskDelay(pdMS_TO_TICKS(5));

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (nrf_presente(&radios[i], i)) activos++;
    g_activos = activos;

    if (activos == NUM_RADIOS) {
        ESP_LOGI(TAG, "Los %d NRF responden correctamente", NUM_RADIOS);
    } else if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde: no se arranca el hopping.");
    } else {
        ESP_LOGW(TAG, "Solo %d de %d NRF responden: sigo con los que si", activos, NUM_RADIOS);
    }

    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) escanear_bloque(&radios[i]);
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) generar_hop_table(&radios[i]);

    xTaskCreatePinnedToCore(radio3_task, "radio3_task", RADIO3_TASK_STACK,
                             NULL, RADIO3_TASK_PRIO, &radio3_task_handle, 1);
    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);

    const esp_timer_create_args_t timer_args = { .callback = &on_hop_timer, .name = "tri_hop_timer" };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &hop_timer));

    if (activos > 0) {
        ESP_ERROR_CHECK(esp_timer_start_periodic(hop_timer, DWELL_US));
        xTaskCreatePinnedToCore(tarea_reescaneo, "reescaneo", 4096, NULL,
                                 tskIDLE_PRIORITY + 1, NULL, 0);
        ESP_LOGI(TAG, "%d radio(s) activos — dwell=%dus — reescaneo cada %dms — LED arcoiris=%dHz cuando activos==%d",
                 activos, DWELL_US, REESCANEO_MS, LED_HZ, NUM_RADIOS);
    }

    uint32_t prev[NUM_RADIOS] = { 0 };
    uint32_t t_seg = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_MS));
        t_seg += STATUS_MS / 1000;
        mostrar_estado(t_seg, prev);
    }
}

