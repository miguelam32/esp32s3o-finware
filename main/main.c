/*
 * main.c — MARTILLO v2 — ESP32-S3 N16R8 (ESP-IDF v6) — 3x nRF24L01+ PA/LNA
 *
 *   NRF2 -> SPI2 (CS  9) CE 5   2..27   | core 1
 *   NRF1 -> SPI2 (CS 10) CE 4  28..53   | core 1
 *   NRF3 -> SPI3 (CS 21) CE 1  54..80   | core 0
 *
 * ============================================================================
 *  POR QUE NO HAY TIMER
 * ============================================================================
 *  Con gptimer + notificacion, el ida y vuelta ISR->task cuesta ~65 us. Con
 *  dwell 60 el task no alcanza a bloquearse, se queda girando, y como esta en
 *  prioridad 24 el led_task (3), dwell_task (2) y el main (1) NUNCA corren:
 *  el LED se congela y se cortan los logs. Con dwell <70 us el esquema de
 *  timer no sirve.
 *
 *  Ahora cada task gira contra el contador de ciclos del CPU (rsr.ccount),
 *  que es exacto y sin latencia de scheduler, y el housekeeping se hace
 *  ADENTRO del task del core 0. Se cede un tick cada 2 s para que el idle
 *  alimente el task watchdog y el main respire: eso es 10 ms cada 2000 ms
 *  = 0.5% de hueco, nada.
 *
 * ============================================================================
 *  ORDEN SECUENCIAL
 * ============================================================================
 *  Cada radio avanza de a 1 canal (1 MHz). El PLL solo tiene que moverse
 *  1 MHz y engancha en una fraccion del dwell, asi que casi todo el dwell es
 *  carrier limpio sobre el canal objetivo. Con orden random el salto es de
 *  hasta 50 MHz y el PLL se come el dwell entero re-lockeando al pedo.
 *
 *  Escalera de dwell: 150 us (strike limpio, mas alcance) -> 60 -> 25
 *  (revisita ~650 us, pega mas seguido). El duty por canal es 1/26 = 3.8% y
 *  NO cambia con el dwell; el dwell solo mueve la revisita.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "soc/spi_struct.h"
#include "led_strip.h"

static const char *TAG = "MART";

/* ======================= CONFIG ======================= */
#define CH_LO        2
#define CH_HI        80
#define NUM_CANALES  (CH_HI - CH_LO + 1)     /* 79 */

#define NUM_RADIOS   3
#define CPU_MHZ      240u
#define CEDE_MS      2000                    /* cada cuanto se cede un tick */

static const uint32_t dwell_ladder[] = { 150, 60, 25 };
#define DWELL_STEPS      (sizeof(dwell_ladder) / sizeof(dwell_ladder[0]))
#define DWELL_PERIOD_MS  5000

/* ======================= tiempo ======================= */
static inline __attribute__((always_inline)) uint32_t cyc(void) {
    uint32_t c; __asm__ __volatile__("rsr.ccount %0" : "=a"(c)); return c;
}

/* ======================= LED ======================= */
#define LED_GPIO    GPIO_NUM_48
#define LED_BRILLO  51
static led_strip_handle_t led;
static volatile bool g_jam_ok = false;
static volatile bool g_flash  = false;

/* ======================= nRF24 ======================= */
#define NRF_CONFIG     0x00
#define NRF_EN_AA      0x01
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_AW   0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH      0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS     0x07
#define NRF_TX_ADDR    0x10

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_FLUSH_TX   0xE1

#define RF_CONT_WAVE (1u << 7)
#define RF_PLL_LOCK  (1u << 4)
#define RF_SETUP_MAX 0x0E
#define RF_SETUP_CW  (RF_SETUP_MAX | RF_CONT_WAVE | RF_PLL_LOCK)
#define CONFIG_TX    0x0E

/* ======================= radios ======================= */
typedef struct {
    const char *nombre;
    spi_host_device_t host;
    gpio_num_t ce, cs, sck, mosi, miso;
    spi_device_handle_t spi;
    spi_dev_t *hw;
    uint8_t  ch_lo, ch_hi, canal;
    uint32_t ch_idx;
    uint32_t vueltas;
    volatile uint32_t hops;
    bool bang_ok;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre="NRF2", .host=SPI2_HOST,
      .ce=GPIO_NUM_5, .cs=GPIO_NUM_9,  .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13 },
    { .nombre="NRF1", .host=SPI2_HOST,
      .ce=GPIO_NUM_4, .cs=GPIO_NUM_10, .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13 },
    { .nombre="NRF3", .host=SPI3_HOST,
      .ce=GPIO_NUM_1, .cs=GPIO_NUM_21, .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7  },
};

static volatile uint32_t g_dwell = dwell_ladder[0];
static uint8_t           g_bang_align = 0;

#define GPIO_OUT_W1TS (*(volatile uint32_t *)0x60004008u)
#define GPIO_OUT_W1TC (*(volatile uint32_t *)0x6000400Cu)

static inline void ce_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->ce; }
static inline void ce_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->ce; }
static inline void cs_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->cs; }
static inline void cs_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->cs; }

/* ======================= SPI directo (bang) ======================= */
static inline spi_dev_t *spi_hw(spi_host_device_t h) {
    return (h == SPI2_HOST) ? &GPSPI2 : &GPSPI3;
}

#define BANG_W(c,v) (g_bang_align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                    g_bang_align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))         : \
                                        (((uint32_t)(c))|((uint32_t)(v)<<8)))

static void bang_arm(radio_t *r) {
    spi_dev_t *hw = r->hw;
    hw->user.usr_mosi = 1;  hw->user.usr_miso = 0;
    hw->user.usr_addr = 0;  hw->user.usr_command = 0;  hw->user.usr_dummy = 0;
    hw->dma_conf.dma_tx_ena = 0;  hw->dma_conf.dma_rx_ena = 0;
    hw->ms_dlen.ms_data_bitlen = 15;
}

static inline IRAM_ATTR void bang_write(radio_t *r, uint8_t cmd, uint8_t val) {
    spi_dev_t *hw = r->hw;
    cs_lo(r);
    hw->data_buf[0] = BANG_W(cmd, val);
    hw->cmd.usr = 1;
    uint32_t g = 100000;
    while (hw->cmd.usr && --g) { }
    cs_hi(r);
}

/* ======================= SPI por driver (init / fallback) ======================= */
static void nrf_write_reg(radio_t *r, uint8_t reg, uint8_t val) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 16 };
    t.tx_data[0] = CMD_W_REGISTER | reg;  t.tx_data[1] = val;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
}
static uint8_t nrf_read_reg(radio_t *r, uint8_t reg) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 16 };
    t.tx_data[0] = CMD_R_REGISTER | reg;  t.tx_data[1] = 0xFF;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    return t.rx_data[1];
}
static void nrf_write_multi(radio_t *r, uint8_t reg, const uint8_t *b, int n) {
    uint8_t tx[8]; tx[0] = CMD_W_REGISTER | reg;
    for (int i = 0; i < n; i++) tx[1 + i] = b[i];
    spi_transaction_t t = { .length = (size_t)((n + 1) * 8), .tx_buffer = tx };
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
}
static void nrf_cmd(radio_t *r, uint8_t c) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 8 };
    t.tx_data[0] = c;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
}

/* ======================= reparto de canales ======================= */
static void generar_tabla(radio_t *r) {
    int idx = (int)(r - radios);
    int por = NUM_CANALES / NUM_RADIOS;
    int lo  = CH_LO + idx * por;
    int hi  = lo + por - 1;
    if (idx == NUM_RADIOS - 1) hi = CH_HI;
    r->ch_lo  = (uint8_t)lo;
    r->ch_hi  = (uint8_t)hi;
    r->ch_idx = 0;
    r->canal  = (uint8_t)lo;
    r->vueltas = 0;
}

static void jam_enter(radio_t *r) {
    static const uint8_t dummy[5] = {0,0,0,0,0};
    ce_lo(r);
    nrf_cmd(r, CMD_FLUSH_TX);
    nrf_write_reg(r, NRF_EN_AA, 0x00);
    nrf_write_reg(r, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r, NRF_SETUP_AW, 0x03);
    nrf_write_reg(r, NRF_SETUP_RETR, 0x00);
    nrf_write_reg(r, NRF_CONFIG, CONFIG_TX);
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);
    nrf_write_multi(r, NRF_TX_ADDR, dummy, 5);
    nrf_cmd(r, CMD_FLUSH_TX);
    vTaskDelay(pdMS_TO_TICKS(3));
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    nrf_write_reg(r, NRF_RF_CH, r->ch_lo);
    generar_tabla(r);
    ce_hi(r);
    bang_arm(r);
}

/* ======================= init ======================= */
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED));
}

static bool radio_init(radio_t *r) {
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
    GPIO_OUT_W1TC = 1u << (uint32_t)r->ce;
    GPIO_OUT_W1TS = 1u << (uint32_t)r->cs;
    r->hw = spi_hw(r->host);

    spi_device_interface_config_t dc = {
        .clock_speed_hz = 10 * 1000 * 1000, .mode = 0, .spics_io_num = -1, .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) return false;

    nrf_write_reg(r, NRF_RF_CH, 0x5A);
    if (nrf_read_reg(r, NRF_RF_CH) != 0x5A) {
        ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d)", r->nombre, (int)r->cs);
        return false;
    }
    ESP_LOGI(TAG, "%s OK [SPI%d CS=%d CE=%d]", r->nombre, (int)r->host, (int)r->cs, (int)r->ce);
    return true;
}

static void bang_detectar(radio_t *r) {
    spi_dev_t *prim = spi_hw(r->host);
    spi_dev_t *alt  = (prim == &GPSPI2) ? &GPSPI3 : &GPSPI2;
    spi_dev_t *cand[2] = { prim, alt };
    for (int i = 0; i < 2; i++) {
        for (int a = 0; a < 3; a++) {
            g_bang_align = (uint8_t)a;
            r->hw = cand[i];
            bang_write(r, CMD_W_REGISTER | NRF_RF_CH, 0x2A);
            if (nrf_read_reg(r, NRF_RF_CH) == 0x2A) {
                r->bang_ok = true;
                ESP_LOGI(TAG, "%s BANG OK: GPSPI%d align=%d", r->nombre, i + 2, a);
                return;
            }
        }
    }
    r->bang_ok = false;
    r->hw = prim;
    ESP_LOGW(TAG, "%s BANG NO: uso driver (mas lento)", r->nombre);
}

/* ======================= hop ======================= */
static inline IRAM_ATTR void hop(radio_t *r) {
    r->ch_idx++;
    if (r->ch_idx > (uint32_t)(r->ch_hi - r->ch_lo)) {
        r->ch_idx = 0;
        r->vueltas++;
        /* refresh del CONT_WAVE: los clones Si24R1 se caen solos del modo CW */
        if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_SETUP, RF_SETUP_CW);
        else            nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    }
    uint8_t ch = (uint8_t)(r->ch_lo + r->ch_idx);
    if (ch == r->canal) return;
    r->canal = ch;
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    r->hops++;
}

/* ======================= LED + telemetria (core 0) ======================= */
static void led_set(uint8_t rr, uint8_t gg, uint8_t bb) {
    led_strip_set_pixel(led, 0, rr, gg, bb);
    led_strip_refresh(led);
}
static void led_init(void) {
    led_strip_config_t sc = {
        .strip_gpio_num = LED_GPIO, .max_leds = 1, .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led));
    led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 60, 0); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 0, 60); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 0, 0);
}
static void led_update(void) {
    if (!g_jam_ok) { led_set(0, 0, 0); return; }
    if (g_flash)   { g_flash = false; led_set(LED_BRILLO, LED_BRILLO, LED_BRILLO); return; }
    int64_t t = esp_timer_get_time();
    int hue = (int)((t % 9000000) * 360 / 9000000);
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
}

static void telemetria(void) {
    static uint32_t prev[NUM_RADIOS];
    static int64_t  tprev[NUM_RADIOS];
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < NUM_RADIOS; i++) {
        radio_t *r = &radios[i];
        if (!r->spi) continue;
        uint32_t dh = r->hops - prev[i];
        int64_t  dt = now - tprev[i];
        prev[i] = r->hops; tprev[i] = now;
        if (!dh || dt <= 0) continue;
        uint32_t us_hop = (uint32_t)(dt / dh);
        int nch = r->ch_hi - r->ch_lo + 1;
        ESP_LOGI(TAG, "%s %4lu saltos/s (%lu us/hop) ch=%2u rango=%u..%u revisita=%luus vueltas=%lu",
                 r->nombre, (unsigned long)((uint64_t)dh * 1000000 / (uint64_t)dt),
                 (unsigned long)us_hop, (unsigned)r->canal, (unsigned)r->ch_lo,
                 (unsigned)r->ch_hi, (unsigned long)(us_hop * (uint32_t)nch),
                 (unsigned long)r->vueltas);
    }
}

static void siguiente_dwell(void) {
    static size_t idx = 0;
    idx = (idx + 1) % DWELL_STEPS;
    g_dwell = dwell_ladder[idx];
    int nch = 0, n = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        nch += radios[i].ch_hi - radios[i].ch_lo + 1; n++;
    }
    uint32_t rev = (n ? (uint32_t)(nch / n) : 0) * g_dwell;
    ESP_LOGI(TAG, "dwell -> %3lu us   revisita ~%lu us   %s",
             (unsigned long)g_dwell, (unsigned long)rev,
             rev && rev < 625 ? ">> revisita < slot BT" : ">> strike limpio");
    g_flash = true;
}

/* ======================= tasks ======================= */
/* Core 1: NRF2 + NRF1 (mismo bus SPI2, mismo core -> sin lock cruzado). */
static void IRAM_ATTR jam_core1(void *arg) {
    int64_t t_cede = esp_timer_get_time();
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[0]);
        hop(&radios[1]);
        uint32_t d = g_dwell * CPU_MHZ;
        while (cyc() - c0 < d) { }

        int64_t now = esp_timer_get_time();
        if (now - t_cede >= (int64_t)CEDE_MS * 1000) { t_cede = now; vTaskDelay(1); }
    }
}

/* Core 0: NRF3 + LED + telemetria + escalera de dwell. */
static void IRAM_ATTR jam_core0(void *arg) {
    int64_t t_led = esp_timer_get_time();
    int64_t t_tel = t_led, t_dw = t_led, t_cede = t_led;
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[2]);
        uint32_t d = g_dwell * CPU_MHZ;
        while (cyc() - c0 < d) { }

        int64_t now = esp_timer_get_time();
        if (now - t_led  >= 16000)                     { t_led  = now; led_update(); }
        if (now - t_tel  >= 2000000)                   { t_tel  = now; telemetria(); }
        if (now - t_dw   >= (int64_t)DWELL_PERIOD_MS * 1000) { t_dw = now; siguiente_dwell(); }
        if (now - t_cede >= (int64_t)CEDE_MS * 1000)   { t_cede = now; vTaskDelay(1); }
    }
}

/* ======================= arranque ======================= */
void app_main(void) {
    led_init();
    bus_init(SPI2_HOST, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_13);
    bus_init(SPI3_HOST, GPIO_NUM_18, GPIO_NUM_8,  GPIO_NUM_7);

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radio_init(&radios[i])) activos++;

    if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde. Revisa cableado y 3.3V.");
        for (;;) { led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }
    if (activos < NUM_RADIOS) ESP_LOGW(TAG, "Solo %d de %d NRF", activos, NUM_RADIOS);

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        jam_enter(&radios[i]);
        bang_detectar(&radios[i]);
        nrf_write_reg(&radios[i], NRF_RF_CH, radios[i].ch_lo);
        radios[i].canal = radios[i].ch_lo;
        bang_arm(&radios[i]);
    }

    g_jam_ok = (activos == NUM_RADIOS);

    xTaskCreatePinnedToCore(jam_core1, "jam1", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    xTaskCreatePinnedToCore(jam_core0, "jam0", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    ESP_LOGI(TAG, "MARTILLO v2 · %d radio(s) · %d canales · SECUENCIAL · spin en ciclos · dwell %lu us",
             activos, NUM_CANALES, (unsigned long)g_dwell);

    vTaskDelay(portMAX_DELAY);
}
