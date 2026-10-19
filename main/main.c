/*
 * main.c — JAMMER 2.4 GHz TOTAL — ESP32-S3 N16R8 — ESP-IDF v6
 * 3x nRF24L01+ PA/LNA + WiFi del ESP32 como transmisor wideband.
 * Barre los 79 canales BT Classic + BLE. Flash y listo.
 *
 *   NRF2 -> SPI2 (CS 9)  CE 5  IRQ 16   |  core 1
 *   NRF1 -> SPI2 (CS 10) CE 4  IRQ 15   |  core 1
 *   NRF3 -> SPI3 (CS 21) CE 1  IRQ 2    |  core 0
 *
 * El PA del RFX2401C se apaga si bajas CE -> CHIRP (CE fijo) siempre.
 * Por debajo de 130us el PLL no engancha y la portadora BARRE: eso es
 * interferencia broadband deliberada, no un defecto.
 *
 * CS MANUAL (spics_io_num = -1): el periferico guarda el cs_pin_id del ultimo
 * device y solo lo cambia cuando el driver cambia de device. Al saltear el
 * driver con bang_write, el CS quedaba apuntando al radio equivocado.
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
#define SPI_HZ_RAPIDO (16 * 1000 * 1000)   /* baja a 10M si el modulo no lo banca */
#define SPI_HZ_SEGURO (10 * 1000 * 1000)
#define WIFI_JAM      1                    /* 1 = WiFi como 4o transmisor wideband */

/* ================= GPIO directo ================= */
#define GPIO_OUT_RD    (*(volatile uint32_t *)0x60004004u)
#define GPIO_OUT_W1TS  (*(volatile uint32_t *)0x60004008u)
#define GPIO_OUT_W1TC  (*(volatile uint32_t *)0x6000400Cu)

/* ================= tiempo ================= */
#define CPU_MHZ 240u
#define MS_CYC  (CPU_MHZ * 1000u)
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
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre="NRF2", .host=SPI2_HOST, .fase=0,
      .ce=GPIO_NUM_5, .cs=GPIO_NUM_9,  .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_16 },
    { .nombre="NRF1", .host=SPI2_HOST, .fase=26,
      .ce=GPIO_NUM_4, .cs=GPIO_NUM_10, .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13, .irq=GPIO_NUM_15 },
    { .nombre="NRF3", .host=SPI3_HOST, .fase=52,
      .ce=GPIO_NUM_1, .cs=GPIO_NUM_21, .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7,  .irq=GPIO_NUM_2  },
};

/* dwell: rango donde P(pisar paquete) ~ 1 para paquetes BT y el chirp sigue
 * siendo broadband. Sin intervencion, rota solo. */
static const uint16_t ladder[] = { 8, 12, 18, 28 };
#define DWELL_STEPS (sizeof(ladder)/sizeof(ladder[0]))
#define DWELL_PERIODO_MS 10000

static volatile uint16_t g_dwell = 12;
static volatile uint32_t g_k = 0;          /* tick compartido entre cores */
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
    hw->user.wr_bit_order = 0;  hw->user.rd_bit_order = 0;
    hw->dma_conf.dma_tx_ena = 0;  hw->dma_conf.dma_rx_ena = 0;
    hw->ms_dlen.ms_data_bitlen = 15;
}

static inline IRAM_ATTR void bang_write(radio_t *r, uint8_t cmd, uint8_t val) {
    spi_dev_t *hw = r->hw;
    cs_lo(r);
    hw->data_buf[0] = BANG_W(cmd, val);
    hw->cmd.usr = 1;
    uint32_t g = 200000;
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

static inline IRAM_ATTR void hop(radio_t *r, uint32_t k) {
    uint8_t ch = (uint8_t)(JAM_CH_LO + (r->fase + k) % JAM_CH_N);
    r->canal = ch;
    bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);   /* CE fijo: PA siempre on */
    r->hops++;
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

/* ================= WiFi: 4o transmisor wideband ================= */
#if WIFI_JAM
static const uint8_t wch[3] = { 1, 6, 11 };   /* 20 MHz cada uno: ~20 canales BT */
static int      wch_idx = 0;
static uint8_t  wframe[24 + 64];
static bool     wifi_ok = false;

/* Contador de frames realmente transmitidos: permite medir el duty real.
 * 802.11b/g a 1-6 Mbps -> un frame de 88 B dura ~250-700us en el aire.
 *   ~4000 frames/s  -> duty alto, el CSMA no lo frena
 *   ~20 frames/s    -> el CSMA/CA lo esta matando (backoff por los nRF) */
static volatile uint32_t g_wifi_frames = 0;

#define WIFI_INTERVALO_US 0    /* 0 = loop agresivo. Subilo si el WDT barkea. */
#define WIFI_ROTA_MS      100  /* rotacion de canal WiFi */

static void wifi_jam_init(void) {
    if (nvs_flash_init() != ESP_OK) { ESP_LOGW(TAG, "NVS fail"); return; }
    if (esp_netif_init() != ESP_OK) return;
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) return;
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return;
    if (esp_wifi_start() != ESP_OK) return;
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(84);
    esp_wifi_set_channel(wch[0], WIFI_SECOND_CHAN_NONE);

    memset(wframe, 0, sizeof(wframe));
    wframe[0] = 0x08; wframe[1] = 0x00;          /* data frame */
    memset(&wframe[4], 0xFF, 6);                 /* addr1 broadcast */
    wframe[10] = 0x02; wframe[15] = 0x01;        /* addr2 */
    memset(&wframe[16], 0xFF, 6);                /* addr3 */
    memset(&wframe[24], 0xAA, 64);
    wifi_ok = true;
    ESP_LOGI(TAG, "WiFi TX wideband ON (ch 1/6/11)");
}

/* Tarea del WiFi: loop agresivo de inyeccion. En core 0, prioridad ALTA
 * (no maxima) para no matar a jam_loop_b, que tambien esta en core 0.
 * OJO: esp_wifi_80211_tx es bloqueante (~250-700us). Durante ese tiempo
 * NRF3 no salta. Si ves que NRF3 pierde muchos hop/s, subi WIFI_INTERVALO_US. */
static void wifi_task(void *arg) {
    uint32_t t_ch = 0;
    while (1) {
        if (wifi_ok) {
            esp_wifi_80211_tx(WIFI_IF_STA, wframe, sizeof(wframe), false);
            g_wifi_frames++;
        }
#if WIFI_INTERVALO_US > 0
        esp_rom_delay_us(WIFI_INTERVALO_US);
#endif
        uint32_t now = cyc();
        if (now - t_ch >= (uint32_t)WIFI_ROTA_MS * MS_CYC) {
            t_ch = now;
            wch_idx = (wch_idx + 1) % 3;
            esp_wifi_set_channel(wch[wch_idx], WIFI_SECOND_CHAN_NONE);
        }
        vTaskDelay(1);   /* cede: si no, el WDT de core 0 barkea */
    }
}

/* Log del duty del WiFi. En core 0, prioridad baja: nunca puede colgar el
 * jammer. Mide si el CSMA/CA esta frenando la inyeccion por culpa de los nRF.
 *   4000 frames/s -> duty alto, el CSMA no lo frena
 *     20 frames/s -> el CSMA/CA lo mato (backoff), el WiFi no aporta */
static void wifi_log_task(void *arg) {
    uint32_t prev = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t dwf = g_wifi_frames - prev;
        prev = g_wifi_frames;
        ESP_LOGI(TAG, "WiFi: %lu frames/s (duty ~%lu%%) ch_wifi=%u",
                 (unsigned long)(dwf / 2), (unsigned long)(dwf / 2 * 100 / 4000),
                 (unsigned)wch[wch_idx]);
    }
}
#endif /* WIFI_JAM */

/* ================= init ================= */
static void bus_init(spi_host_device_t h, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t b = { .mosi_io_num=mosi, .miso_io_num=miso, .sclk_io_num=sck,
        .quadwp_io_num=-1, .quadhd_io_num=-1, .max_transfer_sz=64 };
    ESP_ERROR_CHECK(spi_bus_initialize(h, &b, SPI_DMA_CH_AUTO));
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

    /* intenta el SPI rapido; si el modulo no lo banca, cae a 10 MHz */
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
    int antes = gpio_get_level(r->irq);       /* STATUS=0x0E por POR -> IRQ BAJO */
    nrf_write_reg(r, NRF_STATUS, 0x70);
    esp_rom_delay_us(100);
    int desp = gpio_get_level(r->irq);
    if (antes == 0 && desp == 1) ESP_LOGI(TAG, "%s IRQ OK", r->nombre);
    else ESP_LOGW(TAG, "%s IRQ raro (%d->%d)", r->nombre, antes, desp);
}

static bool bang_probar(radio_t *r, uint8_t a) {
    g_bang_align = a;
    bang_write(r, CMD_W_REGISTER | NRF_RF_CH, 0x2A);
    return nrf_read_reg(r, NRF_RF_CH) == 0x2A;
}

/* ================= loops ================= */
/* core 1: SPI2 (NRF2 + NRF1). Lleva el tick global y el LED. */
static void IRAM_ATTR jam_loop_a(void *arg) {
    uint32_t t_led = 0, t_peld = 0, t_safe = 0;
    int peld = 0;
    for (;;) {
        uint32_t c0 = cyc();
        g_k++;
        if (radios[0].spi) hop(&radios[0], g_k);
        if (radios[1].spi) hop(&radios[1], g_k);

        uint32_t obj = (uint32_t)g_dwell * CPU_MHZ;
        while (cyc() - c0 < obj) { }

        uint32_t now = cyc();
        if (now - t_led >= (uint32_t)LED_REFRESCO_MS * MS_CYC) { t_led = now; led_update(); }
        /* El duty del WiFi se loguea desde wifi_log_task (core 0): un ESP_LOGI
         * desde esta tarea, de prioridad maxima, puede colgarse con el lock
         * de stdout tomado. */
        if (now - t_safe >= 5000u * MS_CYC) { t_safe = now; vTaskDelay(1); }
        if (now - t_peld >= (uint32_t)DWELL_PERIODO_MS * MS_CYC) {
            t_peld = now;
            peld = (peld + 1) % DWELL_STEPS;
            g_dwell = ladder[peld];
            g_flash_until = esp_timer_get_time() + 100000;
        }
    }
}

/* core 0: SPI3 (NRF3). Sigue el tick de core 1. */
static void IRAM_ATTR jam_loop_b(void *arg) {
    uint32_t last = 0, t_yield = 0;
    for (;;) {
        uint32_t k = g_k;
        if (k != last && radios[2].spi) { last = k; hop(&radios[2], k); }

        uint32_t now = cyc();
        if (now - t_yield >= 2000u * MS_CYC) { t_yield = now; vTaskDelay(1); }
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
        int al = -1;
        for (int a = 0; a < 3 && al < 0; a++) if (bang_probar(&radios[i], (uint8_t)a)) al = a;
        if (al < 0) {
            ESP_LOGE(TAG, "%s BANG ROTO: las 3 alineaciones fallan -> ABORTO", radios[i].nombre);
            for (;;) { led_set(60,0,0); vTaskDelay(pdMS_TO_TICKS(200)); }
        }
        ESP_LOGI(TAG, "%s bang OK (align=%d)", radios[i].nombre, al);
        nrf_write_reg(&radios[i], NRF_RF_CH, JAM_CH_LO);
        radios[i].canal = JAM_CH_LO;
        bang_arm(&radios[i]);
    }

#if WIFI_JAM
    wifi_jam_init();
#endif

    /* Arcoiris solo con los 3 vivos. Si falta uno: LED apagado, pero sigue
     * jameando con los que si. */
    g_jam_ok = (activos == NUM_RADIOS);

    xTaskCreatePinnedToCore(jam_loop_a, "jamA", 4096, NULL, configMAX_PRIORITIES-1, NULL, 1);
    xTaskCreatePinnedToCore(jam_loop_b, "jamB", 4096, NULL, configMAX_PRIORITIES-1, NULL, 0);
#if WIFI_JAM
    if (wifi_ok) {
        xTaskCreatePinnedToCore(wifi_task, "wifi", 4096, NULL,
                                configMAX_PRIORITIES-2, NULL, 0);
        xTaskCreatePinnedToCore(wifi_log_task, "wlog", 2048, NULL, 1, NULL, 0);
    }
#endif

    ESP_LOGI(TAG, "JAM TOTAL · %d NRF @%dMHz · 79 canales · %s · dwell %u us",
             activos, g_spi_hz/1000000, wifi_ok ? "WiFi wideband ON" : "solo NRF",
             (unsigned)g_dwell);
    ESP_LOGI(TAG, "WiFi como 4o TX independiente: +6 dBm/MHz al 100%% duty");
    ESP_LOGI(TAG, "  nRF: +17 dBm/MHz al 3.8%% duty  -> impacto instantaneo");
    ESP_LOGI(TAG, "  WiFi: 20 MHz continuos        -> piso de ruido permanente");
    vTaskDelay(portMAX_DELAY);
}
