/*
 * main.c — JAMMER 2.4 GHz — ESP32-S3 N16R8 — ESP-IDF v6
 * 3x nRF24L01+ PA/LNA. Barrido de los 79 canales BT Classic (2..80).
 *
 *   NRF2 -> SPI2 (CS 9)  CE 5  IRQ 16  | core 1
 *   NRF1 -> SPI2 (CS 10) CE 4  IRQ 15  | core 1
 *   NRF3 -> SPI3 (CS 21) CE 1  IRQ 2   | core 0
 *
 * CE FIJO (PA siempre on).
 *
 * ====== 3 MODOS, MEDIBLES EN EL LOG ======
 * MODO_ANCHO    : 3 radios en tercios, dwell 10us. Revisita ~390us.
 *                 Golpea mas veces, cada golpe es chirp (potencia dispersa).
 *                 El BT salta cada 625us -> 390 < 625 => ACIERTA.
 * MODO_PLENO    : 3 radios en tercios, dwell 130us (PLL clavado).
 *                 Revisita ~3.5ms. Golpea menos, pero con potencia PLENA.
 *                 Es lo que usa AntonBronnfjell ("130us PLL dwell, CW strikes").
 * MODO_TRIDENTE : los 3 radios JUNTOS en el mismo canal, dwell 130us.
 *                 +4.8 dB en un canal (3 fuentes incoherentes suman potencia).
 *                 Revisita 79 x 135 = 10.7ms: cobertura lenta a cambio de golpe
 *                 fuerte. Sirve si el objetivo esta lejos.
 *
 * Elegi por medicion, no por fe: el log imprime REVISITA y us/hop reales.
 *
 * ====== POR QUE SPI_DMA_DISABLED ======
 * Medido con DMA activo: 20700 hops/s EXACTOS en los 3 radios => 48.3us por
 * hop, de los cuales solo 20 eran dwell. Los otros 28us eran overhead del
 * motor DMA (setup de descriptores) en transferencias de 2 bytes. Sin DMA
 * el hop baja a ~5us. Revisita = 26 x (dwell + overhead): el overhead
 * account la mitad del barrido.
 *
 * ====== TECHO FISICO (no es codigo) ======
 * Link budget medido: telefono->audifono -36 dBm, jammer->audifono -31.5 dBm
 * => S/J = -4.5 dB. Para matar un paquete BT hace falta S/J <= -10 dB, o sea
 * 14 dB mas. Se consiguen ACERCANDOSE (3m -> 0.6m = +14 dB), no con software.
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
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

static const char *TAG = "JAM";

/* ================= config ================= */
#define SPI_HZ_RAPIDO (16 * 1000 * 1000)
#define SPI_HZ_SEGURO (10 * 1000 * 1000)
#define WIFI_JAM      0

/* ---- elegi UNO ---- */
#define MODO_ANCHO     1   /* revisita ~390us : maxima cobertura   */
#define MODO_PLENO     2   /* revisita ~3.5ms : potencia por golpe */
#define MODO_TRIDENTE  3   /* +4.8 dB, revisita ~10.7ms           */
#ifndef MODO_JAM
#define MODO_JAM MODO_ANCHO
#endif

/* ================= GPIO directo ================= */
#define GPIO_OUT_W1TS  (*(volatile uint32_t *)0x60004008u)
#define GPIO_OUT_W1TC  (*(volatile uint32_t *)0x6000400Cu)

/* ================= tiempo ================= */
#define CPU_MHZ 240u
#define MS_CYC  (CPU_MHZ * 1000u)
#define YIELD_MS 500u

static inline __attribute__((always_inline)) uint32_t cyc(void) {
    uint32_t c; __asm__ __volatile__("rsr.ccount %0" : "=a"(c)); return c;
}

/* ================= LED ================= */
#define LED_GPIO        GPIO_NUM_48
#define LED_BRILLO      51
#define LED_CICLO_US    18000000.0
#define LED_BREATH_US   6000000.0
#define LED_BREATH_MIN  0.55
#define LED_REFRESCO_MS 16

static led_strip_handle_t led;
static volatile bool    g_jam_ok = false;
static volatile int64_t g_flash_until = 0;

/* ================= nRF24L01+ ================= */
#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_TX_ADDR 0x10

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_FLUSH_TX   0xE1

#define RF_CONT_WAVE (1u << 7)
#define RF_PLL_LOCK  (1u << 4)
#define RF_SETUP_MAX 0x0E
#define RF_SETUP_CW  (RF_SETUP_MAX | RF_CONT_WAVE | RF_PLL_LOCK)
#define CONFIG_TX    0x0E

/* ================= jammer ================= */
#define NUM_RADIOS 3
#define JAM_CH_LO  2
#define JAM_CH_HI  80
#define JAM_CH_N   (JAM_CH_HI - JAM_CH_LO + 1)   /* 79: BT Classic COMPLETO */

typedef struct {
    const char *nombre;
    spi_host_device_t host;
    gpio_num_t ce, cs, sck, mosi, miso, irq;
    uint8_t    fase;
    spi_device_handle_t spi;
    spi_dev_t  *hw;
    uint8_t    canal;
    volatile uint32_t hops;
    bool       bang_ok;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre="NRF2", .host=SPI2_HOST, .fase=0,
      .ce=GPIO_NUM_5, .cs=GPIO_NUM_9,  .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_16 },
    { .nombre="NRF1", .host=SPI2_HOST, .fase=26,
      .ce=GPIO_NUM_4, .cs=GPIO_NUM_10, .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_15 },
    { .nombre="NRF3", .host=SPI3_HOST, .fase=53,
      .ce=GPIO_NUM_1, .cs=GPIO_NUM_21, .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7,  .irq=GPIO_NUM_2  },
};

/* dwell y ladder por modo */
#if   MODO_JAM == MODO_ANCHO
  static const uint16_t ladder[] = { 10, 10, 10, 130 };
  #define MODO_TXT "ANCHO (cobertura)"
#elif MODO_JAM == MODO_PLENO
  static const uint16_t ladder[] = { 130, 130, 130, 130 };
  #define MODO_TXT "PLENO (potencia/golpe)"
#else
  static const uint16_t ladder[] = { 130, 130, 130, 130 };
  #define MODO_TXT "TRIDENTE (+4.8 dB)"
#endif
#define DWELL_STEPS (sizeof(ladder)/sizeof(ladder[0]))
#define DWELL_PERIODO_MS 8000

static volatile uint16_t g_dwell = ladder[0];
static volatile uint32_t g_k = 0;
static volatile uint32_t g_irq_cnt[NUM_RADIOS];
static uint8_t           g_bang_align = 0;
static int               g_spi_hz = SPI_HZ_RAPIDO;

static inline void ce_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->ce; }
static inline void ce_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->ce; }
static inline void cs_lo(radio_t *r) { GPIO_OUT_W1TC = 1u << (uint32_t)r->cs; }
static inline void cs_hi(radio_t *r) { GPIO_OUT_W1TS = 1u << (uint32_t)r->cs; }

static void IRAM_ATTR irq_isr(void *arg) { g_irq_cnt[(int)(intptr_t)arg]++; }

/* ================= SPI por registro (IRAM) ================= */
static inline spi_dev_t *spi_hw(spi_host_device_t h) {
    return (h == SPI2_HOST) ? &GPSPI2 : &GPSPI3;
}
#define BANG_W(c,v) (g_bang_align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                    g_bang_align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))        : \
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

/* ================= driver (init), CS manual ================= */
static void nrf_write_reg(radio_t *r, uint8_t reg, uint8_t val) {
    spi_transaction_t t = { .flags=SPI_TRANS_USE_TXDATA, .length=16 };
    t.tx_data[0] = CMD_W_REGISTER | reg;  t.tx_data[1] = val;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
}
static uint8_t nrf_read_reg(radio_t *r, uint8_t reg) {
    spi_transaction_t t = { .flags=SPI_TRANS_USE_TXDATA|SPI_TRANS_USE_RXDATA, .length=16 };
    t.tx_data[0] = CMD_R_REGISTER | reg;  t.tx_data[1] = 0xFF;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    return t.rx_data[1];
}
static void nrf_write_multi(radio_t *r, uint8_t reg, const uint8_t *b, int n) {
    uint8_t tx[8]; tx[0] = CMD_W_REGISTER | reg;
    for (int i = 0; i < n; i++) tx[1+i] = b[i];
    spi_transaction_t t = { .length = (size_t)((n+1)*8), .tx_buffer = tx };
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
}
static void nrf_cmd(radio_t *r, uint8_t c) {
    spi_transaction_t t = { .flags=SPI_TRANS_USE_TXDATA, .length=8 };
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
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    nrf_write_reg(r, NRF_RF_CH, JAM_CH_LO);
    r->canal = JAM_CH_LO;
    ce_hi(r);
    bang_arm(r);
}

/* ================= LED ================= */
static void led_set(uint8_t rr, uint8_t gg, uint8_t bb) {
    led_strip_set_pixel(led, 0, rr, gg, bb);
    led_strip_refresh(led);
}
static void arcoiris(int64_t t, uint8_t *r, uint8_t *g, uint8_t *b) {
    double d = (double)t;
    double fc = 2.0*M_PI*d/LED_CICLO_US;
    double rr = (sin(fc)+1.0)*0.5, gg = (sin(fc+2.0*M_PI/3.0)+1.0)*0.5, bb = (sin(fc+4.0*M_PI/3.0)+1.0)*0.5;
    double fr = 2.0*M_PI*d/LED_BREATH_US;
    double br = (double)LED_BRILLO*(LED_BREATH_MIN + (1.0-LED_BREATH_MIN)*((sin(fr)+1.0)*0.5));
    *r=(uint8_t)(rr*br+0.5); *g=(uint8_t)(gg*br+0.5); *b=(uint8_t)(bb*br+0.5);
}
static void led_update(void) {
    if (!g_jam_ok) { led_set(0,0,0); return; }
    int64_t t = esp_timer_get_time();
    if (t < g_flash_until) { led_set(LED_BRILLO,LED_BRILLO,LED_BRILLO); return; }
    uint8_t r,g,b; arcoiris(t,&r,&g,&b); led_set(r,g,b);
}
static void led_init(void) {
    led_strip_config_t sc = { .strip_gpio_num=LED_GPIO, .max_leds=1,
        .led_model=LED_MODEL_WS2812, .color_component_format=LED_STRIP_COLOR_COMPONENT_FMT_GRB };
    led_strip_rmt_config_t rc = { .clk_src=RMT_CLK_SRC_DEFAULT, .resolution_hz=10*1000*1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc,&rc,&led));
    led_set(60,0,0); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0,60,0); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0,0,60); vTaskDelay(pdMS_TO_TICKS(100));
    led_set(0,0,0);
}

/* ================= init ================= */
static void bus_init(spi_host_device_t h, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t b = { .mosi_io_num=mosi, .miso_io_num=miso, .sclk_io_num=sck,
        .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=64 };
    /* DMA OFF: con 2 bytes por transferencia el setup del DMA cuesta ~28us,
     * mas que la transferencia. Medido: 20700 hops/s con DMA. */
    ESP_ERROR_CHECK(spi_bus_initialize(h, &b, SPI_DMA_DISABLED));
}

static bool radio_add(radio_t *r, int hz) {
    spi_device_interface_config_t dc = { .clock_speed_hz=hz, .mode=0,
        .spics_io_num=-1, .queue_size=1 };
    if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) return false;
    nrf_write_reg(r, NRF_RF_CH, 0x5A);
    return nrf_read_reg(r, NRF_RF_CH) == 0x5A;
}

static bool radio_init(radio_t *r) {
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
    GPIO_OUT_W1TC = 1u << (uint32_t)r->ce;
    GPIO_OUT_W1TS = 1u << (uint32_t)r->cs;

    gpio_config_t io = { .pin_bit_mask = 1ULL << (uint32_t)r->irq,
        .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_NEGEDGE };
    gpio_config(&io);
    r->hw = spi_hw(r->host);

    if (!radio_add(r, SPI_HZ_RAPIDO)) {
        if (r->spi) spi_bus_remove_device(r->spi);
        if (!radio_add(r, SPI_HZ_SEGURO)) {
            ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d)", r->nombre, (int)r->cs);
            return false;
        }
        g_spi_hz = SPI_HZ_SEGURO;
        ESP_LOGW(TAG, "%s cayo a SPI 10 MHz", r->nombre);
    }
    ESP_LOGI(TAG, "%s OK [SPI%d CS=%d CE=%d IRQ=%d @%dMHz]",
             r->nombre, (int)r->host, (int)r->cs, (int)r->ce, (int)r->irq, g_spi_hz/1000000);
    return true;
}

static void irq_selftest(radio_t *r) {
    nrf_write_reg(r, NRF_STATUS, 0x70);
    esp_rom_delay_us(100);
    if (gpio_get_level(r->irq) == 1) ESP_LOGI(TAG, "%s IRQ OK", r->nombre);
    else ESP_LOGW(TAG, "%s IRQ trabado en bajo", r->nombre);
}

static void bang_detectar(radio_t *r) {
    spi_dev_t *cand[2];
    cand[0] = &GPSPI2;
    cand[1] = &GPSPI3;

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
    r->hw = spi_hw(r->host);
    ESP_LOGW(TAG, "%s BANG NO: uso driver", r->nombre);
}

/* ================= lazo caliente ================= */
static inline IRAM_ATTR void hop(radio_t *r, uint32_t k) {
    uint8_t ch;
#if MODO_JAM == MODO_TRIDENTE
    /* los 3 radios al MISMO canal: 3 fuentes incoherentes suman potencia
     * => +4.8 dB en ese canal. Revisita = 79 x dwell. */
    ch = (uint8_t)(JAM_CH_LO + k % JAM_CH_N);
#else
    /* cada radio su tercio: cobertura total, revisita = 26 x dwell */
    ch = (uint8_t)(JAM_CH_LO + (r->fase + k) % JAM_CH_N);
#endif
    r->canal = ch;
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    r->hops++;
}

/* core 1: SPI2 (NRF2 + NRF1). Tick global + LED. */
static void IRAM_ATTR jam_loop_a(void *arg) {
    uint32_t t_led = 0, t_peld = 0, t_yield = 0;
    int peld = 0;
    for (;;) {
        uint32_t c0 = cyc();
        g_k++;
        if (radios[0].spi) hop(&radios[0], g_k);
        if (radios[1].spi) hop(&radios[1], g_k);

        while (cyc() - c0 < (uint32_t)g_dwell * CPU_MHZ) { }

        uint32_t now = cyc();
        if (now - t_led >= (uint32_t)LED_REFRESCO_MS * MS_CYC) { t_led = now; led_update(); }
        if (now - t_peld >= (uint32_t)DWELL_PERIODO_MS * MS_CYC) {
            t_peld = now;
            peld = (peld + 1) % DWELL_STEPS;
            g_dwell = ladder[peld];
            g_flash_until = esp_timer_get_time() + 100000;
        }
        if (now - t_yield >= YIELD_MS * MS_CYC) { t_yield = now; vTaskDelay(1); }
    }
}

/* core 0: SPI3 (NRF3). Resincroniza tras el yield. */
static void IRAM_ATTR jam_loop_b(void *arg) {
    uint32_t last = 0, t_yield = cyc();
    for (;;) {
        uint32_t k = g_k;
        if (k != last && radios[2].spi) { last = k; hop(&radios[2], k); }

        uint32_t now = cyc();
        if (now - t_yield >= YIELD_MS * MS_CYC) {
            t_yield = now;
            vTaskDelay(1);
            last = g_k;
        }
    }
}

/* Telemetria: us/hop REAL + revisita. Es el numero que decide. */
static void tel_task(void *arg) {
    uint32_t p0 = 0, p1 = 0, p2 = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t h0 = radios[0].hops, h1 = radios[1].hops, h2 = radios[2].hops;
        uint32_t us0 = (h0 - p0) ? (2000000UL / (h0 - p0)) : 0;
#if MODO_JAM == MODO_TRIDENTE
        uint32_t rev = 79UL * us0;
#else
        uint32_t rev = 26UL * us0;
#endif
        ESP_LOGI(TAG, "hops/s %lu/%lu/%lu | us/hop=%lu | REVISITA=%luus | dwell=%u | %s",
                 (unsigned long)((h0-p0)/2), (unsigned long)((h1-p1)/2), (unsigned long)((h2-p2)/2),
                 (unsigned long)us0, (unsigned long)rev,
                 (unsigned)g_dwell,
                 rev < 625 ? ">> ACIERTA AL BT" : ">> EL BT SE VA");
        p0 = h0; p1 = h1; p2 = h2;
    }
}

/* ================= arranque ================= */
void app_main(void) {
    led_init();
    gpio_install_isr_service(0);

    bus_init(SPI2_HOST, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_13);
    bus_init(SPI3_HOST, GPIO_NUM_18, GPIO_NUM_8,  GPIO_NUM_7);

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radio_init(&radios[i])) activos++;
    if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde. Revisa cableado y 3.3V.");
        for (;;) { led_set(60,0,0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }
    if (activos < NUM_RADIOS)
        ESP_LOGW(TAG, "Solo %d de %d NRF: sigo con los que si", activos, NUM_RADIOS);

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        irq_selftest(&radios[i]);
        gpio_isr_handler_add(radios[i].irq, irq_isr, (void *)(intptr_t)i);
        jam_enter(&radios[i]);
    }

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].spi) continue;
        bang_detectar(&radios[i]);
        nrf_write_reg(&radios[i], NRF_RF_CH, JAM_CH_LO);
        radios[i].canal = JAM_CH_LO;
        bang_arm(&radios[i]);
    }

    g_jam_ok = (activos == NUM_RADIOS);

    xTaskCreatePinnedToCore(jam_loop_a, "jamA", 4096, NULL, configMAX_PRIORITIES-1, NULL, 1);
    xTaskCreatePinnedToCore(jam_loop_b, "jamB", 4096, NULL, configMAX_PRIORITIES-1, NULL, 0);
    xTaskCreatePinnedToCore(tel_task,   "tel",  2048, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "MODO=%s | %d NRF @%dMHz | 79 canales | DMA OFF | dwell=%u",
             MODO_TXT, activos, g_spi_hz/1000000, (unsigned)g_dwell);
    ESP_LOGI(TAG, "meta: REVISITA < 625us (salto del BT). Si dice 'EL BT SE VA',");
    ESP_LOGI(TAG, "baja el dwell o cambia de modo. Y ACERCATE: +14 dB por ir a 0.6 m.");
    vTaskDelay(portMAX_DELAY);
}
