/*
 * main.c
 * ESP32-S3 N16R8 (ESP-IDF v6.0.3) — Jammer BT/BLE con 2x nRF24L01+ PA/LNA
 *
 *   NRF2 -> SPI2 (bus propio)   canales  2..40   tarea en núcleo 0
 *   NRF3 -> SPI3 (bus propio)   canales 41..80   tarea en núcleo 1
 *
 * Sin consola, sin comandos. Se enchufa y transmite.
 *
 * Modo: portadora continua (CONT_WAVE + PLL_LOCK) barriendo el rango que de
 * verdad usa Bluetooth (2402-2480 MHz = canales nRF 2..80), partido entre los
 * dos radios para cubrirlo entero.
 *
 * Dwell automático: cada 10 s sube un peldaño, de 140 us a 450 us. Cuando
 * llega al techo vuelve al piso. Así se prueban las dos estrategias a la vez:
 *   - dwell bajo  = mucha frecuencia de visita, blips cortos
 *   - dwell alto  = pocas visitas pero cada una corrompe paquetes de verdad
 *
 * LED WS2812 (GPIO48):
 *   VERDE    : jammer transmitiendo
 *   BLANCO   : destello corto en cada cambio de dwell
 *   ROJO     : falta algún NRF
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "led_strip.h"

static const char *TAG = "JAM";

// ---------- LED ----------
#define LED_GPIO        GPIO_NUM_48
#define LED_BRILLO      60
static led_strip_handle_t led;

// ---------- Buses ----------
#define PIN_SCK2   GPIO_NUM_12
#define PIN_MOSI2  GPIO_NUM_11
#define PIN_MISO2  GPIO_NUM_13
#define PIN_SCK3   GPIO_NUM_18
#define PIN_MOSI3  GPIO_NUM_8
#define PIN_MISO3  GPIO_NUM_7
#define SPI_HZ     (10 * 1000 * 1000)

// ---------- nRF24L01+ ----------
#define NRF_CONFIG     0x00
#define NRF_EN_AA      0x01
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_AW   0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH      0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS     0x07

#define CMD_R_REGISTER  0x00
#define CMD_W_REGISTER  0x20
#define CMD_FLUSH_TX    0xE1

#define CFG_PWR_UP    (1u << 1)
#define CFG_PRIM_RX   (1u << 0)
#define RF_CONT_WAVE  (1u << 7)
#define RF_PLL_LOCK   (1u << 4)

/* 0x0E = 2 Mbps + RF_PWR=11 (0 dBm en el pin; con PA+LNA salen ~+20 dBm) */
#define RF_SETUP_VALOR  0x0E
#define CONFIG_MODO_TX  0x0E      /* PWR_UP + PTX + CRC 2 bytes */

#define MAX_XFER  8

// ---------- Jammer ----------
#define NUM_RADIOS  2

/* Rango útil de BT/BLE: 2402-2480 MHz. Fuera de aca es espectro desperdiciado. */
#define JAM_CH_LO   2
#define JAM_CH_HI   80
#define JAM_CH_N    (JAM_CH_HI - JAM_CH_LO + 1)

/* El PLL del nRF tarda 130 us en enganchar: 140 us es el piso real.
 * Pasados ~450 us la vuelta completa supera los 17 ms y el BT se recupera
 * entre visitas, asi que ahi deja de servir subir. */
static const uint32_t dwell_ladder[] = { 180, 220, 260, 300, 360, 420, 480 };
#define DWELL_STEPS      (sizeof(dwell_ladder) / sizeof(dwell_ladder[0]))
#define DWELL_PERIOD_MS  10000

typedef struct {
    const char       *nombre;
    spi_host_device_t host;
    gpio_num_t        ce, cs, sck, mosi, miso;
    uint32_t          seed;
    int               core;

    spi_device_handle_t spi;
    TaskHandle_t        task;

    uint8_t  order[JAM_CH_N];      /* permutación de canales de este radio */
    uint8_t  order_len;
    uint8_t  order_idx;
    uint32_t vueltas;

    volatile uint32_t hops;
    volatile uint8_t  canal;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre = "NRF2", .host = SPI2_HOST,
      .ce = GPIO_NUM_5,  .cs = GPIO_NUM_9,  .sck = GPIO_NUM_12,
      .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13,
      .seed = 0xA17E56u, .core = 0 },
    { .nombre = "NRF3", .host = SPI3_HOST,
      .ce = GPIO_NUM_1,  .cs = GPIO_NUM_21, .sck = GPIO_NUM_18,
      .mosi = GPIO_NUM_8, .miso = GPIO_NUM_7,
      .seed = 0xA17E57u, .core = 1 },
};

static gptimer_handle_t dwell_timer;
static volatile uint32_t g_dwell = dwell_ladder[0];
static volatile bool     g_flash = false;      /* destello de LED */

/* ================= CE por registro directo =================
 * ESP32-S3: base GPIO = 0x60004000.
 *   OUT_W1TS  +0x08   OUT_W1TC  +0x0C   (pines 0-31)
 *   OUT1_W1TS +0x14   OUT1_W1TC +0x18   (pines 32-45)
 * gpio_set_level() cuesta 2-5 us; esto es 1 ciclo de bus.
 */
#define OUT_W1TS    (*(volatile uint32_t *)0x60004008u)
#define OUT_W1TC    (*(volatile uint32_t *)0x6000400Cu)
#define OUT1_W1TS   (*(volatile uint32_t *)0x60004014u)
#define OUT1_W1TC   (*(volatile uint32_t *)0x60004018u)

static inline void ce_hi(radio_t *r) {
    if (r->ce < 32) OUT_W1TS  = (1u << (uint32_t)r->ce);
    else            OUT1_W1TS = (1u << ((uint32_t)r->ce - 32));
}
static inline void ce_lo(radio_t *r) {
    if (r->ce < 32) OUT_W1TC  = (1u << (uint32_t)r->ce);
    else            OUT1_W1TC = (1u << ((uint32_t)r->ce - 32));
}

/* ================= SPI (sin DMA: <=8 bytes va por TXDATA/RXDATA) ================= */
static void nrf_write_reg(spi_device_handle_t dev, uint8_t reg, uint8_t val) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 16 };
    t.tx_data[0] = CMD_W_REGISTER | reg;
    t.tx_data[1] = val;
    spi_device_polling_transmit(dev, &t);
}

static uint8_t nrf_read_reg(spi_device_handle_t dev, uint8_t reg) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 16 };
    t.tx_data[0] = CMD_R_REGISTER | reg;
    t.tx_data[1] = 0xFF;
    spi_device_polling_transmit(dev, &t);
    return t.rx_data[1];
}

static void nrf_cmd(spi_device_handle_t dev, uint8_t cmd) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 8 };
    t.tx_data[0] = cmd;
    spi_device_polling_transmit(dev, &t);
}

/* ================= permutación de canales ================= */
static uint32_t xorshift32(uint32_t *s) {
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    return *s;
}

/* Baraja y recorre completa: cada vuelta toca TODOS los canales del radio.
 * Random (no secuencial) para que el AFH de Bluetooth no aprenda el patrón
 * y mueva su whitelist justo para esquivarnos. */
static void barajar(radio_t *r) {
    uint32_t s = r->seed ^ (r->vueltas * 0x9E3779B1u);
    if (s == 0) s = 1;
    for (int i = r->order_len - 1; i > 0; i--) {
        int j = (int)(xorshift32(&s) % (uint32_t)(i + 1));
        uint8_t tmp = r->order[i]; r->order[i] = r->order[j]; r->order[j] = tmp;
    }
}

/* Reparto 2..80 entre los dos radios:
 *   NRF2 -> 2..40 (39)      NRF3 -> 41..80 (40)                          */
static void generar_tabla(radio_t *r) {
    int idx  = (int)(r - radios);
    int por  = JAM_CH_N / NUM_RADIOS;            /* 39 */
    int lo   = JAM_CH_LO + idx * por;
    int hi   = lo + por - 1;
    if (idx == NUM_RADIOS - 1) hi = JAM_CH_HI;   /* el último se lleva el resto */

    int n = 0;
    for (int c = lo; c <= hi; c++) r->order[n++] = (uint8_t)c;
    r->order_len = (uint8_t)n;
    r->order_idx = 0;
    r->vueltas   = 0;
    barajar(r);
}

/* ================= portadora continua ================= */
/* Spin de ciclos puro. NO uses ets_delay_us() aca: en FreeRTOS hace
 * busy-wait contra el timer del sistema y dispara el task watchdog cuando
 * el dwell baja de ~200us. */
static inline void pll_settle(uint32_t us) {
    if (!us) return;
    volatile uint32_t n = (us * 240u) / 4u;   /* 240 MHz: ~4 ciclos por vuelta */
    while (n--) { }
}

static void cw_up(radio_t *r, uint8_t ch) {
    ce_lo(r);                                       /* CE abajo primero */
    nrf_write_reg(r->spi, NRF_RF_CH, (uint8_t)(ch & 0x7F));
    pll_settle(140);                                /* asentamiento del PLL */
    ce_hi(r);                                       /* carrier en el canal */
}

static void jam_enter(radio_t *r) {
    ce_lo(r);
    nrf_cmd(r->spi, CMD_FLUSH_TX);
    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);          /* PTX */
    nrf_write_reg(r->spi, NRF_SETUP_RETR, 0x00);
    nrf_write_reg(r->spi, NRF_RF_SETUP,
                  (uint8_t)(RF_SETUP_VALOR | RF_CONT_WAVE | RF_PLL_LOCK));
    generar_tabla(r);
    cw_up(r, r->order[0]);
}

/* Un salto: mismo recorrido de canales, carrier sostenido.
 * CE abajo -> RF_CH -> espera PLL -> CE arriba. Si se cambia el canal con el
 * carrier activo y CE alto, el PLL se desengancha y el carrier se congela. */
static inline void jam_hop(radio_t *r) {
    r->order_idx++;
    if (r->order_idx >= r->order_len) {
        r->order_idx = 0;
        r->vueltas++;
        barajar(r);
    }
    uint8_t ch = r->order[r->order_idx];
    r->canal = ch;
    cw_up(r, ch);
    r->hops++;
}

/* ================= init ================= */
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = MAX_XFER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED));
}

static bool radio_init(radio_t *r) {
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_level(r->ce, 0);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_HZ,
        .mode = 0,
        .spics_io_num = r->cs,
        .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &devcfg, &r->spi) != ESP_OK) return false;

    /* self-test: escribir y releer RF_CH */
    nrf_write_reg(r->spi, NRF_RF_CH, 0x5A);
    uint8_t back = nrf_read_reg(r->spi, NRF_RF_CH);
    if (back != 0x5A) {
        ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d SCK=%d MOSI=%d MISO=%d)",
                 r->nombre, (int)r->cs, (int)r->sck, (int)r->mosi, (int)r->miso);
        return false;
    }

    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r->spi, NRF_SETUP_AW, 0x03);
    nrf_write_reg(r->spi, NRF_SETUP_RETR, 0x00);
    nrf_write_reg(r->spi, NRF_STATUS, 0x70);
    nrf_cmd(r->spi, CMD_FLUSH_TX);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    nrf_write_reg(r->spi, NRF_RF_SETUP, RF_SETUP_VALOR);

    ESP_LOGI(TAG, "%s OK  [SPI%d CS=%d CE=%d]", r->nombre,
             (int)r->host, (int)r->cs, (int)r->ce);
    return true;
}

/* ================= tareas ================= */
static bool IRAM_ATTR on_alarm(gptimer_handle_t t,
                               const gptimer_alarm_event_data_t *e, void *ctx) {
    BaseType_t woken = pdFALSE;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (radios[i].task) vTaskNotifyGiveFromISR(radios[i].task, &woken);
    }
    return woken == pdTRUE;
}

static void radio_task(void *arg) {
    radio_t *r = (radio_t *)arg;
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        jam_hop(r);
        /* Ceder la CPU de verdad. Sin esto el task watchdog de IDLE dispara
         * cuando el dwell baja de ~200us: la tarea queda al 100% de CPU y
         * el scheduler nunca llega a IDLE. */
        vTaskDelay(0);
    }
}

/* Cada 10 s sube un peldaño de dwell. Al llegar al techo vuelve al piso. */
static void dwell_task(void *arg) {
    size_t idx = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DWELL_PERIOD_MS));
        idx = (idx + 1) % DWELL_STEPS;

        gptimer_stop(dwell_timer);
        g_dwell = dwell_ladder[idx];
        gptimer_alarm_config_t alarm = {
            .alarm_count = g_dwell,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        gptimer_set_alarm_action(dwell_timer, &alarm);
        gptimer_start(dwell_timer);

        int canales = 0, n = 0;
        for (int i = 0; i < NUM_RADIOS; i++) {
            if (!radios[i].spi) continue;
            canales += radios[i].order_len; n++;
        }
        float vuelta = n ? ((float)(canales / n) * g_dwell) / 1000.0f : 0.0f;
        ESP_LOGI(TAG, "dwell -> %3lu us   vuelta completa ~%.1f ms",
                 (unsigned long)g_dwell, vuelta);
        g_flash = true;
    }
}

/* ---------- LED ---------- */
static void led_set(uint8_t rr, uint8_t gg, uint8_t bb) {
    led_strip_set_pixel(led, 0, rr, gg, bb);
    led_strip_refresh(led);
}

static void led_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO, .max_leds = 1,
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

    led_set(60, 0, 0);   vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 60, 0);   vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 0, 60);   vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 0, 0);
}

static void led_task(void *arg) {
    while (1) {
        if (g_flash) {
            g_flash = false;
            led_set(60, 60, 60);
            vTaskDelay(pdMS_TO_TICKS(120));
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        led_set(0, 60, 0);              /* verde: transmitiendo */
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ---------- arranque ---------- */
static void gptimer_init(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &dwell_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(dwell_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(dwell_timer));
    gptimer_alarm_config_t alarm = {
        .alarm_count = g_dwell,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(dwell_timer, &alarm));
}

void app_main(void) {
    led_init();

    bus_init(SPI2_HOST, PIN_SCK2, PIN_MOSI2, PIN_MISO2);
    bus_init(SPI3_HOST, PIN_SCK3, PIN_MOSI3, PIN_MISO3);

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radio_init(&radios[i])) activos++;

    if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde. Revisa cableado y 3.3V.");
        for (;;) { led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }
    if (activos < NUM_RADIOS) {
        ESP_LOGW(TAG, "Solo %d de %d NRF: sigo con los que si", activos, NUM_RADIOS);
    }

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        jam_enter(&radios[i]);                       /* carrier arriba ya */
        xTaskCreatePinnedToCore(radio_task, radios[i].nombre, 3072, &radios[i],
                                configMAX_PRIORITIES - 1, &radios[i].task,
                                (BaseType_t)radios[i].core);
    }

    gptimer_init();
    gptimer_start(dwell_timer);

    xTaskCreate(led_task,  "led",   2048, NULL, 3, NULL);
    xTaskCreate(dwell_task, "dwell", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "JAM ON  ·  %d radio(s)  ·  canales %d..%d  ·  dwell %lu us",
             activos, JAM_CH_LO, JAM_CH_HI, (unsigned long)g_dwell);
    ESP_LOGI(TAG, "escalera de dwell: %lu..%lu us cada %d ms",
             (unsigned long)dwell_ladder[0],
             (unsigned long)dwell_ladder[DWELL_STEPS - 1], DWELL_PERIOD_MS);

    /* log de estado: no acepta comandos, solo informa */
    uint32_t prev[NUM_RADIOS] = { 0 };
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        for (int i = 0; i < NUM_RADIOS; i++) {
            radio_t *r = &radios[i];
            if (!r->spi) continue;
            uint32_t pps = (r->hops - prev[i]) / 2;
            prev[i] = r->hops;
            ESP_LOGI(TAG, "%s  %4lu saltos/s  canal=%2u  libres=%2u  vueltas=%lu  dwell=%lu",
                     r->nombre, (unsigned long)pps, (unsigned)r->canal,
                     (unsigned)r->order_len, (unsigned long)r->vueltas,
                     (unsigned long)g_dwell);
        }
    }
}
