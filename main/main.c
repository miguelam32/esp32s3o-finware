/*
 * main.c — MARTILLO — ESP32-S3 N16R8 (ESP-IDF v6.0.3) — 3x nRF24L01+ PA/LNA
 *
 *   NRF2 -> SPI2 (CS  9)  canales  2..27   | core 1
 *   NRF1 -> SPI2 (CS 10)  canales 28..53   | core 1
 *   NRF3 -> SPI3 (CS 21)  canales 54..80   | core 0
 *
 * CONT_WAVE + PLL_LOCK, CE strobe (CE bajo -> RF_CH -> CE alto).
 *
 * ============================================================================
 *  QUE CAMBIA RESPECTO AL JAMMER VIEJO (y por que)
 * ============================================================================
 *  1) ORDEN SECUENCIAL en vez de random. Un salto de 1 MHz deja que el PLL
 *     enganche en una fraccion del dwell, asi que TODO el dwell es un strike
 *     real sobre el canal objetivo. Con orden random el salto es de hasta
 *     50 MHz y el PLL se come el dwell entero re-lockeando al pedo: de 150 us
 *     de dwell solo ~20 us pegaban de verdad. Eso alcanza para un SBC de
 *     auricular barato, no para un LDAC con AFH agresiva y buffer grande.
 *
 *  2) La escalera de dwell ahora SI incluye el punto medio y el rapido:
 *     150 us (strike limpio, revisita 3.9 ms) -> 60 -> 30 (revisita 780 us).
 *     Duty por canal es 1/26 = 3.8% y NO cambia con el dwell: el dwell solo
 *     mueve la revisita. Menos dwell = mas seguido pega = mas PER sobre
 *     paquetes cortos. Mas dwell = strike mas limpio = mas alcance.
 *
 *  3) NRF1 y NRF2 van JUNTOS en core 1 (comparten SPI2, asi no se pelean el
 *     lock del bus entre nucleos). NRF3 solo en core 0.
 *
 *  4) Refresh de RF_SETUP cada vuelta: los clones Si24R1 se caen solos del
 *     modo CONT_WAVE y dejan de radiar.
 *
 * ============================================================================
 *  POR QUE LOS CAROS SOBREVIVEN (para que sepas contra que peleas)
 * ============================================================================
 *  AFH: mientras TODOS los canales esten malos parejo, AFH no tiene a donde
 *  escapar y mantiene el hop set lleno. Eso el split ya lo logra.
 *  Lo que NO logra un strike sucio es subir el PER lo suficiente: el caro
 *  tiene buffer grande + error concealment + mejor front-end y absorbe el
 *  residual. Contra eso se pelea con strike LIMPIO (secuencial + dwell >=
 *  lock) y revisita corta, no con mas potencia bruta.
 *
 *  Si con esto sigue sin caer, el techo de 3 radios sobre 79 canales ya lo
 *  diste: ahi se sube con mas modulos, mejores antenas, o metiendo el WiFi
 *  del propio ESP32 como fuente ancha.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "led_strip.h"

static const char *TAG = "MART";

// ---------- LED ----------
#define LED_GPIO        GPIO_NUM_48
#define LED_BRILLO      51
static led_strip_handle_t led;
static volatile bool g_jam_ok = false;

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

#define RF_CONT_WAVE  (1u << 7)
#define RF_PLL_LOCK   (1u << 4)

#define RF_SETUP_VALOR  0x0E       /* RF_PWR=11 (0 dBm) */
#define CONFIG_MODO_TX  0x0E

// ---------- Jammer ----------
#define NUM_RADIOS  3

#define JAM_CH_LO   2
#define JAM_CH_HI   80
#define JAM_CH_N    (JAM_CH_HI - JAM_CH_LO + 1)   /* 79 */

/* Piso del silicio: ~130 us de asentamiento del PLL. 150 = 130 + margen.
 * El techo practico del barrido: por encima de 170 us la vuelta pasa de 6.6 ms
 * y el duty por canal cae sin ganar nada.
 * El escalon de 30 us: con orden SECUENCIAL el salto es de 1 MHz, el PLL
 * engancha rapido y la revisita baja a 780 us -> pega mucho mas seguido.
 * Ese es el escalon que apunta a los dispositivos con AFH buena y buffer. */
static const uint32_t dwell_ladder[] = { 150, 60, 30 };
#define DWELL_STEPS      (sizeof(dwell_ladder) / sizeof(dwell_ladder[0]))
#define DWELL_PERIOD_MS  5000

typedef struct {
    const char       *nombre;
    spi_host_device_t host;
    gpio_num_t        ce, cs, sck, mosi, miso;
    int               core;

    spi_device_handle_t spi;
    TaskHandle_t        task;

    uint8_t  ch_lo, ch_hi;      /* rebanada de este radio */
    uint8_t  canal;
    uint32_t vueltas;
    uint32_t ch_idx;

    volatile uint32_t hops;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    /* Regla: NUNCA dos radios del MISMO bus en nucleos distintos (se pelean
     * el lock del bus entre cores). NRF1 y NRF2 comparten SPI2 -> van juntos
     * en core 1. NRF3 en SPI3 solo, en core 0. */
    { .nombre = "NRF2", .host = SPI2_HOST,
      .ce = GPIO_NUM_5,  .cs = GPIO_NUM_9,  .sck = GPIO_NUM_12,
      .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13, .core = 1 },
    { .nombre = "NRF1", .host = SPI2_HOST,
      .ce = GPIO_NUM_4,  .cs = GPIO_NUM_10, .sck = GPIO_NUM_12,
      .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13, .core = 1 },
    { .nombre = "NRF3", .host = SPI3_HOST,
      .ce = GPIO_NUM_1,  .cs = GPIO_NUM_21, .sck = GPIO_NUM_18,
      .mosi = GPIO_NUM_8,  .miso = GPIO_NUM_7,  .core = 0 },
};

static gptimer_handle_t  dwell_timer;
static volatile uint32_t g_dwell = dwell_ladder[0];
static volatile bool     g_flash = false;

static inline void ce_hi(radio_t *r) { gpio_set_level(r->ce, 1); }
static inline void ce_lo(radio_t *r) { gpio_set_level(r->ce, 0); }

/* ================= SPI ================= */
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

/* ================= tabla de canales (SECUENCIAL) ================= */
/* Reparto 2..80 entre los tres radios (79/3 = 26):
 *   NRF2 ->  2..27   NRF1 -> 28..53   NRF3 -> 54..80
 * El orden es secuencial: cada salto es de 1 MHz y el PLL engancha rapido.
 * Al cerrar la vuelta se rota el arranque random, para que el patron no sea
 * siempre el mismo ciclo alineado. */
static void generar_tabla(radio_t *r) {
    int idx = (int)(r - radios);
    int por = JAM_CH_N / NUM_RADIOS;
    int lo  = JAM_CH_LO + idx * por;
    int hi  = lo + por - 1;
    if (idx == NUM_RADIOS - 1) hi = JAM_CH_HI;

    r->ch_lo   = (uint8_t)lo;
    r->ch_hi   = (uint8_t)hi;
    r->ch_idx  = (uint32_t)(esp_timer_get_time() % (hi - lo + 1));
    r->canal   = (uint8_t)(lo + r->ch_idx);
    r->vueltas = 0;
}

/* ================= portadora continua ================= */
/* CE abajo -> RF_CH -> CE alto. El PLL se asienta DURANTE el dwell, con el
 * carrier ya radiando. No hay que esperarlo con CE abajo. */
static void cw_up(radio_t *r, uint8_t ch) {
    ce_lo(r);
    nrf_write_reg(r->spi, NRF_RF_CH, (uint8_t)(ch & 0x7F));
    ce_hi(r);
}

static void jam_enter(radio_t *r) {
    ce_lo(r);
    nrf_cmd(r->spi, CMD_FLUSH_TX);
    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r->spi, NRF_SETUP_AW, 0x03);
    nrf_write_reg(r->spi, NRF_SETUP_RETR, 0x00);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    nrf_write_reg(r->spi, NRF_RF_SETUP,
                  (uint8_t)(RF_SETUP_VALOR | RF_CONT_WAVE | RF_PLL_LOCK));
    generar_tabla(r);
    cw_up(r, r->canal);
}

static inline void jam_hop(radio_t *r) {
    r->ch_idx++;
    if (r->ch_idx > (uint32_t)(r->ch_hi - r->ch_lo)) {
        r->ch_idx = 0;
        r->vueltas++;
        /* refresh del CONT_WAVE: los Si24R1 clones se caen solos */
        nrf_write_reg(r->spi, NRF_RF_SETUP,
                      (uint8_t)(RF_SETUP_VALOR | RF_CONT_WAVE | RF_PLL_LOCK));
    }
    uint8_t ch = (uint8_t)(r->ch_lo + r->ch_idx);
    r->canal = ch;
    cw_up(r, ch);
    r->hops++;
}

/* ================= init ================= */
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
}

static bool radio_init(radio_t *r) {
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_level(r->ce, 0);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_HZ, .mode = 0, .spics_io_num = r->cs, .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &devcfg, &r->spi) != ESP_OK) return false;

    nrf_write_reg(r->spi, NRF_RF_CH, 0x5A);
    if (nrf_read_reg(r->spi, NRF_RF_CH) != 0x5A) {
        ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d SCK=%d MOSI=%d MISO=%d)",
                 r->nombre, (int)r->cs, (int)r->sck, (int)r->mosi, (int)r->miso);
        return false;
    }
    nrf_write_reg(r->spi, NRF_STATUS, 0x70);
    nrf_cmd(r->spi, CMD_FLUSH_TX);
    ESP_LOGI(TAG, "%s OK  [SPI%d CS=%d CE=%d]", r->nombre, (int)r->host, (int)r->cs, (int)r->ce);
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
    }
}

static void dwell_task(void *arg) {
    size_t idx = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DWELL_PERIOD_MS));
        idx = (idx + 1) % DWELL_STEPS;

        gptimer_stop(dwell_timer);
        g_dwell = dwell_ladder[idx];
        gptimer_alarm_config_t alarm = {
            .alarm_count = g_dwell, .reload_count = 0, .flags.auto_reload_on_alarm = true,
        };
        gptimer_set_alarm_action(dwell_timer, &alarm);
        gptimer_start(dwell_timer);

        int canales = 0, n = 0;
        for (int i = 0; i < NUM_RADIOS; i++) {
            if (!radios[i].spi) continue;
            canales += (radios[i].ch_hi - radios[i].ch_lo + 1);
            n++;
        }
        int por_radio = n ? canales / n : 0;
        uint32_t revisita = (uint32_t)por_radio * g_dwell;
        ESP_LOGI(TAG, "dwell -> %3lu us   revisita ~%lu us   %s",
                 (unsigned long)g_dwell, (unsigned long)revisita,
                 revisita < 625 ? ">> REVISITA < SLOT BT" : ">> strike limpio");
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
        .strip_gpio_num = LED_GPIO, .max_leds = 1, .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, .flags.invert_out = false,
    };
    led_strip_rmt_config_t rmt_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000, .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &led));
    led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 60, 0); vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 0, 60); vTaskDelay(pdMS_TO_TICKS(150));
    led_set(0, 0, 0);
}
static void led_task(void *arg) {
    int64_t ciclo = 9000000;
    while (1) {
        if (!g_jam_ok) { led_set(0, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        if (g_flash) {
            g_flash = false;
            led_set(LED_BRILLO, LED_BRILLO, LED_BRILLO);
            vTaskDelay(pdMS_TO_TICKS(120));
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        int64_t t = esp_timer_get_time();
        int hue = (int)((t % ciclo) * 360 / ciclo);
        int region = hue / 60, f = hue % 60;
        uint8_t v = LED_BRILLO, r, g, b;
        int p = (v * (255 - 255 * f / 60)) / 255;
        int q = (v * (255 - 255 * (60 - f) / 60)) / 255;
        int u = (v * 255 * f / 60) / 255;
        switch (region) {
            case 0: r = v; g = u; b = 0; break;
            case 1: r = q; g = v; b = 0; break;
            case 2: r = 0; g = v; b = u; break;
            case 3: r = 0; g = q; b = v; break;
            case 4: r = u; g = 0; b = v; break;
            default: r = v; g = 0; b = p; break;
        }
        led_set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------- timer de dwell ---------- */
static void gptimer_init(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, .direction = GPTIMER_COUNT_UP, .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &dwell_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(dwell_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(dwell_timer));
    gptimer_alarm_config_t alarm = {
        .alarm_count = g_dwell, .reload_count = 0, .flags.auto_reload_on_alarm = true,
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
    if (activos < NUM_RADIOS) ESP_LOGW(TAG, "Solo %d de %d NRF: sigo con los que si", activos, NUM_RADIOS);

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        jam_enter(&radios[i]);
        xTaskCreatePinnedToCore(radio_task, radios[i].nombre, 3072, &radios[i],
                                configMAX_PRIORITIES - 1, &radios[i].task,
                                (BaseType_t)radios[i].core);
    }

    gptimer_init();
    gptimer_start(dwell_timer);

    g_jam_ok = (activos == NUM_RADIOS);
    xTaskCreate(led_task,   "led",   2048, NULL, 3, NULL);
    xTaskCreate(dwell_task, "dwell", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "MARTILLO ON · %d radio(s) · canales %d..%d · orden SECUENCIAL · dwell %lu us",
             activos, JAM_CH_LO, JAM_CH_HI, (unsigned long)g_dwell);

    uint32_t prev[NUM_RADIOS] = {0};
    int64_t  tprev[NUM_RADIOS] = {0};
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int64_t now = esp_timer_get_time();
        for (int i = 0; i < NUM_RADIOS; i++) {
            radio_t *r = &radios[i];
            if (!r->spi) continue;
            uint32_t dh = r->hops - prev[i];
            uint32_t us_hop = dh ? (uint32_t)((now - tprev[i]) / dh) : 0;
            prev[i] = r->hops; tprev[i] = now;
            int nch = r->ch_hi - r->ch_lo + 1;
            uint32_t rev = us_hop * (uint32_t)nch;
            ESP_LOGI(TAG, "%s %4lu saltos/s (%lu us/hop) ch=%2u rango=%u..%u revisita=%luus vueltas=%lu dwell=%lu",
                     r->nombre, (unsigned long)(dh / 2), (unsigned long)us_hop,
                     (unsigned)r->canal, (unsigned)r->ch_lo, (unsigned)r->ch_hi,
                     (unsigned long)rev, (unsigned long)r->vueltas, (unsigned long)g_dwell);
        }
    }
}
