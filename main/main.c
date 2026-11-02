/*
 * main.c — CAZADOR v9.1 — 6x nRF24L01+ PA/LNA — ESP32-S3 N16R8 — ESP-IDF v6.0.3
 *
 * SPI2 (SCK 12, MOSI 11, MISO 13): NRF1(CE 4 CS 10) NRF2(CE 5 CS 9)  NRF3(CE 6 CS 14)
 * SPI3 (SCK 18, MOSI 8,  MISO 7):  NRF4(CE 1 CS 21) NRF5(CE 15 CS 16) NRF6(CE 17 CS 38)
 * WS2812 en GPIO 48.
 *
 * ====== v9.1 SOBRE v9: BARRIDO BARAJADO + JITTER ======
 * 1) ESCALON_MS 4000 -> 6000.
 * 2) tablas_barrido ya NO reparte bloques contiguos. Hace una permutacion
 *    Fisher-Yates de los 79 canales y la reparte round-robin entre los radios
 *    vivos. Rompe el "peine rigido" de 6 canales contiguos que se desplazaba
 *    en lockstep: cada radio ve canales esparcidos por TODA la banda, en orden
 *    impredecible, y dos radios nunca coinciden en el mismo indice.
 * 3) Jitter de dwell en los dos buses (+0..50%). Rompe la periodicidad y
 *    decorrelaciona los dos buses que antes caminaban en fase.
 *
 * ====== POR QUE EL BARRIDO COMPLETO YA ES ANTI-AFH ======
 * AFH re-clasifica cada ~1 s (afh_scan_interval, Silabs). Un ciclo completo
 * de 79 canales a 320us son 4.5 ms: 200x mas rapido que la ventana de AFH.
 * La piconet ve TODOS los canales como malos, no puede hacer nada. Eso es
 * lo que la caza (v10) perdia: fijaba 20 canales, AFH los excluia y se iba
 * a los otros 59. Con 6 radios no se gana por potencia: se ataca todo.
 *
 * Lo que SI queda expuesto con barrido puro es la PREDICTIBILIDAD: con 6
 * bloques contiguos en lockstep se puede caracterizar el patron en un ciclo.
 * De ahi el barajado y el jitter.
 *
 * ====== v9 QUE SIGUE VALIENDO ======
 * - park_all_pins() ANTES de cualquier transaccion SPI (v8.1): con CS
 *   flotando en bajo, su nRF drivea MISO y colisiona el bus -> falso
 *   "NO RESPONDE".
 * - radio_init() con 5 reintentos y esperas de 100ms: el power-on reset del
 *   nRF24L01+ no es confiable en los primeros milisegundos.
 * - radio_init() LIBERA el device en cada fallo: el IDF v6.0.3 reserva un
 *   slot de CS por dispositivo y hay 3 por bus. Sin liberar, 2 fallos matan
 *   el bus.
 * - VENTANA DE SILENCIO (Ventana_SCAN=1): el escaner esta a centimetros de
 *   5 TX de +18 dBm. Sin silencio el mapa sale todo saturado.
 * - Score en punto fijo 1/256: con division entera y DEC=24, cualquier
 *   score entre 1 y 23 no decae nunca. Guardado en 1/256 el decremento
 *   existe para cualquier valor.
 *
 * ====== GEOMETRIA ======
 * gap = (L-1)*d. Un paquete de duracion T cae en una ventana garantizada
 * si T > gap. En barrido L=14 (79/6): gap 2340us a 180us (cubre DH5),
 * 4160us a 320us (no cubre DH5 pero el ciclo completo sigue siendo 4.5ms).
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
#include "esp_heap_caps.h"

static const char *TAG = "CAZA";

/* ===================== CONFIG ===================== */
#define CH_LO        2
#define CH_HI        80
#define NUM_CANALES  (CH_HI - CH_LO + 1)      /* 79 */
#define NUM_RADIOS   6

/* ---- ESCALERA DE DWELL ----
 * 6 s por escalon. 130us es el umbral del PLL: por debajo la portadora
 * BARRE (chirp), por encima queda CW plena. */
#define ESCALON_MS    6000u      /* 6 s por escalon */
#define DWELL_LOOP    0          /* 1 = al llegar arriba vuelve a empezar */
static const uint16_t k_dwell[] = { 180u, 220u, 260u, 300u, 320u };
#define N_ESCALONES   ((int)(sizeof(k_dwell) / sizeof(k_dwell[0])))

#define DWELL_MIN_US  k_dwell[0]
#define DWELL_US      DWELL_MIN_US

/* ---- MODO ---- */
#define FORZAR_BARRIDO  1        /* 1 = nunca caza: los 6 barren los 79 siempre */
#define VENTANA_SCAN    1        /* 1 = ventana de silencio + MAPA (cuesta 5%) */

#define PAQUETE_DH3  1622u
#define PAQUETE_DH5  2870u

#define SCAN_MUESTRAS  3
#define REESCANEO_MS   1200
#define SCAN_HARD_MS   900

/* Score en 1/256 de unidad (punto fijo). */
#define SCORE_SHIFT   8
#define SCORE_ONE     (1u << SCORE_SHIFT)

/* BT~3.6 · ruido~0.72 · WiFi~61 */
#define SCORE_DEC      24
#define SCORE_MIN      2
#define SAT_MIN       12

#define WARMUP_SCANS   12

#define SPI_HZ_RAPIDO (16 * 1000 * 1000)
#define SPI_HZ_SEGURO (10 * 1000 * 1000)

#define CPU_MHZ      240u
#define CEDE_MS      2000

#define GAP_MAX_US   1200u
#define L_MAX  ((GAP_MAX_US / DWELL_MIN_US) + 1u)
#define N_MAX  ((NUM_RADIOS * L_MAX) < NUM_CANALES ? (NUM_RADIOS * L_MAX) : NUM_CANALES)
#define MAX_TAB (N_MAX + 2)

/* ===================== tiempo ===================== */
static inline __attribute__((always_inline)) uint32_t cyc(void) {
    uint32_t c; __asm__ __volatile__("rsr.ccount %0" : "=a"(c)); return c;
}

/* ===================== LED ===================== */
#define LED_GPIO    GPIO_NUM_48
#define LED_BRILLO  51
static led_strip_handle_t led;
static volatile bool g_jam_ok = false;
static volatile int  g_vivos = 0;
static volatile bool g_flash  = false;
static volatile bool g_cazando = false;
static volatile bool g_quiet  = false;
static volatile int64_t g_pulso_hasta = 0;
static volatile uint8_t g_escalon = 0;

/* ===================== nRF24 ===================== */
#define NRF_CONFIG     0x00
#define NRF_EN_AA      0x01
#define NRF_EN_RXADDR  0x02
#define NRF_SETUP_AW   0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH      0x05
#define NRF_RF_SETUP   0x06
#define NRF_STATUS     0x07
#define NRF_RPD        0x09
#define NRF_TX_ADDR    0x10

#define CMD_R_REGISTER 0x00
#define CMD_W_REGISTER 0x20
#define CMD_FLUSH_TX   0xE1

#define RF_CONT_WAVE (1u << 7)
#define RF_PLL_LOCK  (1u << 4)

/* RF_SETUP 0x0E = 0b00001110:
 *   bit3 RF_DR_HIGH=1, bit5 RF_DR_LOW=0 -> 2 Mbps
 *   bits2:1 RF_PWR=11                   -> 0 dBm (maximo)
 * En CW no viaja dato, pero la desviacion del modulador y el nivel del PA
 * quedan fijados por estos bits. 0x0E es el techo de la parte. */
#define RF_SETUP_MAX 0x0E
#define RF_SETUP_CW  (RF_SETUP_MAX | RF_CONT_WAVE | RF_PLL_LOCK)
#define CONFIG_TX    0x0E
#define CONFIG_RX    0x0F

/* ===================== tablas (doble buffer) ===================== */
typedef struct {
    uint8_t ch[NUM_RADIOS][MAX_TAB];
    uint8_t len[NUM_RADIOS];
    uint8_t n_total;
    uint8_t n_rafaga;
} tablas_t;

static tablas_t g_buf[2];
static tablas_t *g_tab = &g_buf[0];

/* ===================== estado del scan ===================== */
static uint8_t  g_cuenta[NUM_CANALES];
static uint16_t g_score[NUM_CANALES];
#define HIST_SCANS   256
#define SAT_FRAC_MAX 30
#define PRES_FRAC_MIN 8
static uint8_t  *s_hist = NULL;
static uint16_t  s_hist_idx = 0;
static uint16_t  s_hist_fill = 0;

static int      scan_ch = 0;
static uint8_t  scan_pasada = 0;
static uint8_t  g_scans = 0;
static bool     scan_activo = false;
static int64_t  scan_t0 = 0;
static int      scan_radio_idx = 5;
static volatile int g_scan_idx = -1;
static volatile uint32_t g_ventana_us = 0;
static volatile uint8_t  g_sel = 0;

/* ===================== radios ===================== */
typedef struct {
    const char *nombre;
    spi_host_device_t host;
    gpio_num_t ce, cs, sck, mosi, miso;
    bool cs_alto;
    spi_device_handle_t spi;
    spi_dev_t *hw;
    uint8_t  idx;
    uint8_t  canal;
    uint32_t vueltas;
    volatile uint32_t hops;
    bool bang_ok;
    bool bang_sucio;
    bool vivo;
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

static volatile uint32_t g_dwell = DWELL_US;
static uint8_t           g_bang_align = 0;
static int               g_spi_hz = SPI_HZ_RAPIDO;

/* ---- RNG: xorshift32 en IRAM. Semilla fija, periodo 2^32-1, sin div. ---- */
static volatile uint32_t g_rng = 0x9E3779B9u;
static inline IRAM_ATTR uint32_t rnd(void) {
    uint32_t r = g_rng;
    r ^= r << 13; r ^= r >> 17; r ^= r << 5;
    g_rng = r;
    return r;
}

/* GPIO 0..31 -> OUT_W1TS/W1TC ; GPIO 32..46 -> OUT1_* */
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
    else                       GPIO_OUT_W1TC = 1u << (uint32_t)r->ce;
}
static inline void cs_lo(radio_t *r) {
    if (r->cs_alto) GPIO_OUT1_W1TC = 1u << ((uint32_t)r->cs - 32);
    else            GPIO_OUT_W1TC = 1u << (uint32_t)r->cs;
}
static inline void cs_hi(radio_t *r) {
    if (r->cs_alto) GPIO_OUT1_W1TS = 1u << ((uint32_t)r->cs - 32);
    else            GPIO_OUT_W1TS = 1u << (uint32_t)r->cs;
}

/* ===================== SPI directo (bang) ===================== */
static inline spi_dev_t *spi_hw(spi_host_device_t h) {
    return (h == SPI2_HOST) ? &GPSPI2 : &GPSPI3;
}
#define BANG_W(c,v) (g_bang_align == 0 ? (((uint32_t)(c)<<24)|((uint32_t)(v)<<16)) : \
                    g_bang_align == 1 ? (((uint32_t)(c)<<8)|(uint32_t)(v))         : \
                                        (((uint32_t)(c))|((uint32_t)(v)<<8)))

static void IRAM_ATTR bang_arm(radio_t *r) {
    spi_dev_t *hw = r->hw;
    hw->user.usr_mosi = 1;  hw->user.usr_miso = 0;
    hw->user.usr_addr = 0;  hw->user.usr_command = 0;  hw->user.usr_dummy = 0;
    hw->dma_conf.dma_tx_ena = 0;  hw->dma_conf.dma_rx_ena = 0;
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

/* ===================== SPI por driver ===================== */
static void nrf_write_reg(radio_t *r, uint8_t reg, uint8_t val) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 16 };
    t.tx_data[0] = CMD_W_REGISTER | reg;  t.tx_data[1] = val;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}
static uint8_t nrf_read_reg(radio_t *r, uint8_t reg) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 16 };
    t.tx_data[0] = CMD_R_REGISTER | reg;  t.tx_data[1] = 0xFF;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
    return t.rx_data[1];
}
static void nrf_write_multi(radio_t *r, uint8_t reg, const uint8_t *b, int n) {
    uint8_t tx[8]; tx[0] = CMD_W_REGISTER | reg;
    for (int i = 0; i < n; i++) tx[1 + i] = b[i];
    spi_transaction_t t = { .length = (size_t)((n + 1) * 8), .tx_buffer = tx };
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}
static void nrf_cmd(radio_t *r, uint8_t c) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 8 };
    t.tx_data[0] = c;
    cs_lo(r); spi_device_polling_transmit(r->spi, &t); cs_hi(r);
    marcar_sucio(r->host);
}

/* ===================== init ===================== */
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_DISABLED));
}

static void park_all_pins(void) {
    for (int i = 0; i < NUM_RADIOS; i++) {
        radio_t *r = &radios[i];
        r->cs_alto = ((uint32_t)r->cs >= 32);

        gpio_reset_pin(r->ce);
        gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
        gpio_set_level(r->ce, 0);

        gpio_reset_pin(r->cs);
        gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
        gpio_set_level(r->cs, 1);

        r->hw = spi_hw(r->host);
    }
    ESP_LOGI(TAG, "Pines estacionados: 6x CS alto, 6x CE bajo");
}

static bool radio_add(radio_t *r, int hz) {
    spi_device_interface_config_t dc = {
        .clock_speed_hz = hz, .mode = 0, .spics_io_num = -1, .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) return false;
    nrf_write_reg(r, NRF_RF_CH, 0x5A);
    return nrf_read_reg(r, NRF_RF_CH) == 0x5A;
}

static bool radio_init(radio_t *r) {
    ce_lo(r);
    cs_hi(r);
    r->hw = spi_hw(r->host);

    bool ok = false;
    for (int intento = 0; intento < 5 && !ok; intento++) {
        if (intento) vTaskDelay(pdMS_TO_TICKS(100));
        if (radio_add(r, SPI_HZ_RAPIDO)) { ok = true; break; }
        if (r->spi) { spi_bus_remove_device(r->spi); r->spi = NULL; }
        vTaskDelay(pdMS_TO_TICKS(100));
        if (radio_add(r, SPI_HZ_SEGURO)) { ok = true; g_spi_hz = SPI_HZ_SEGURO; break; }
        if (r->spi) { spi_bus_remove_device(r->spi); r->spi = NULL; }
    }
    if (!ok) {
        r->spi = NULL;
        ESP_LOGE(TAG, "%s NO RESPONDE tras 5 intentos (CS=%d CE=%d) - REVISAR HW",
                 r->nombre, (int)r->cs, (int)r->ce);
        return false;
    }
    nrf_write_reg(r, NRF_STATUS, 0x70);
    nrf_cmd(r, CMD_FLUSH_TX);
    ESP_LOGI(TAG, "%s OK [SPI%d CE=%d CS=%d @%dMHz]",
             r->nombre, (int)r->host, (int)r->ce, (int)r->cs, g_spi_hz / 1000000);
    return true;
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
    bang_arm(r);
    ce_hi(r);
}

static void bang_detectar(radio_t *r) {
    for (int a = 0; a < 3; a++) {
        g_bang_align = (uint8_t)a;
        r->hw = spi_hw(r->host);
        bang_write(r, CMD_W_REGISTER | NRF_RF_CH, 0x2A);
        if (nrf_read_reg(r, NRF_RF_CH) == 0x2A) {
            r->bang_ok = true;
            ESP_LOGI(TAG, "%s BANG OK align=%d", r->nombre, a);
            return;
        }
    }
    r->bang_ok = false;
    ESP_LOGW(TAG, "%s BANG NO, uso driver", r->nombre);
}

/* ===================== tablas ===================== */
static void hist_init(void) {
    s_hist = heap_caps_malloc((size_t)NUM_CANALES * HIST_SCANS, MALLOC_CAP_SPIRAM);
    if (!s_hist) {
        ESP_LOGW(TAG, "PSRAM: sin memoria para historia, uso el score simple");
        return;
    }
    memset(s_hist, 0, (size_t)NUM_CANALES * HIST_SCANS);
    ESP_LOGI(TAG, "PSRAM: %d scans x %d canales = %d KB",
             HIST_SCANS, NUM_CANALES,
             (int)(((size_t)NUM_CANALES * HIST_SCANS) / 1024));
}

static inline void hist_push(const uint8_t *cuenta) {
    if (!s_hist) return;
    for (int c = 0; c < NUM_CANALES; c++)
        s_hist[(size_t)c * HIST_SCANS + s_hist_idx] = cuenta[c];
    s_hist_idx = (uint16_t)((s_hist_idx + 1) % HIST_SCANS);
    if (s_hist_fill < HIST_SCANS) s_hist_fill++;
}

static inline void hist_stats(int c, int *sat, int *pres, int *total) {
    *sat = 0; *pres = 0;
    *total = s_hist_fill ? s_hist_fill : 1;
    if (!s_hist) return;
    for (int k = 0; k < s_hist_fill; k++) {
        int slot = (int)((s_hist_idx + k) % HIST_SCANS);
        uint8_t h = s_hist[(size_t)c * HIST_SCANS + slot];
        if (h >= SCAN_MUESTRAS) (*sat)++;
        else if (h > 0)         (*pres)++;
    }
}

/* v9.1: permutacion Fisher-Yates + reparto round-robin. Rompe el peine
 * rigido de bloques contiguos. Dos radios NUNCA coinciden en el mismo indice
 * (el reparto es k % nv), asi que siempre estan en canales distintos. */
static void IRAM_ATTR tablas_barrido(tablas_t *t) {
    static uint8_t perm[NUM_CANALES];
    for (int i = 0; i < NUM_CANALES; i++) perm[i] = (uint8_t)(CH_LO + i);
    for (int i = NUM_CANALES - 1; i > 0; i--) {
        int j = (int)(rnd() % (uint32_t)(i + 1));
        uint8_t tmp = perm[i]; perm[i] = perm[j]; perm[j] = tmp;
    }

    int vivos[NUM_RADIOS], nv = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        t->len[i] = 0;
        if (radios[i].vivo) vivos[nv++] = i;
    }
    if (nv == 0) return;

    for (int k = 0; k < NUM_CANALES; k++) {
        int r = vivos[k % nv];
        if (t->len[r] < MAX_TAB) t->ch[r][t->len[r]++] = perm[k];
    }

    t->n_total = NUM_CANALES;
    t->n_rafaga = 0;
    g_cazando = false;
    g_sel = 0;
}

static inline void insertar(uint8_t *elig, uint16_t *rank, int *ne, int n,
                            uint8_t ch, uint16_t rk) {
    int pos = *ne;
    if (pos >= n) {
        int min = 0;
        for (int k = 1; k < *ne; k++) if (rank[k] < rank[min]) min = k;
        if (rk <= rank[min]) return;
        pos = min;
        (*ne)--;
    }
    while (pos > 0 && rank[pos - 1] < rk) {
        elig[pos] = elig[pos - 1];
        rank[pos] = rank[pos - 1];
        pos--;
    }
    elig[pos] = ch;
    rank[pos] = rk;
    if (*ne < n) (*ne)++;
}

static void IRAM_ATTR tablas_caza(tablas_t *t, int n_max) {
    uint8_t  elig[MAX_TAB];
    uint16_t rank[MAX_TAB];
    int ne = 0, nraf = 0;

    if (FORZAR_BARRIDO) { tablas_barrido(t); return; }

    if (s_hist && s_hist_fill >= WARMUP_SCANS) {
        for (int c = 0; c < NUM_CANALES; c++) {
            if (g_cuenta[c] >= SCAN_MUESTRAS) continue;
            int sat, pres, total;
            hist_stats(c, &sat, &pres, &total);
            if (sat * 100 > total * SAT_FRAC_MAX)   continue;
            if (pres * 100 < total * PRES_FRAC_MIN) continue;
            insertar(elig, rank, &ne, n_max, (uint8_t)(CH_LO + c),
                     (uint16_t)(pres * 4 + g_cuenta[c]));
        }
        if (ne > 0) g_sel = 1;
    }

    if (ne == 0) {
        for (int c = 0; c < NUM_CANALES; c++) {
            if (g_cuenta[c] >= SCAN_MUESTRAS) continue;
            uint32_t sc = g_score[c] >> SCORE_SHIFT;
            if (sc < SCORE_MIN) continue;
            if (sc >= SAT_MIN) continue;
            insertar(elig, rank, &ne, n_max, (uint8_t)(CH_LO + c),
                     (uint16_t)((g_score[c] >> 4) + g_cuenta[c]));
        }
        if (ne > 0) g_sel = 2;
    }
    nraf = ne;

    if (ne == 0 || g_scans < WARMUP_SCANS) { tablas_barrido(t); return; }

    int vivos[NUM_RADIOS], nv = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        t->len[i] = 0;
        if (radios[i].vivo) vivos[nv++] = i;
    }
    if (nv == 0) { tablas_barrido(t); return; }

    for (int k = 0; k < ne; k++) {
        int r = vivos[k % nv];
        if (t->len[r] < MAX_TAB) t->ch[r][t->len[r]++] = elig[k];
    }
    for (int k = ne; k < nv; k++) {
        int r = vivos[k];
        if (t->len[r] == 0) t->ch[r][t->len[r]++] = elig[k % nraf];
    }

    t->n_total = (uint8_t)ne;
    t->n_rafaga = (uint8_t)nraf;
    g_cazando = true;
}

static void IRAM_ATTR recalcular(void) {
    tablas_t *nueva = (g_tab == &g_buf[0]) ? &g_buf[1] : &g_buf[0];
    int n_max = N_MAX;
    if (n_max < 1) n_max = 1;
    if ((uint32_t)n_max > (uint32_t)MAX_TAB) n_max = MAX_TAB;

    tablas_caza(nueva, n_max);
    __atomic_store_n(&g_tab, nueva, __ATOMIC_RELEASE);
}

/* ===================== hop ===================== */
static inline IRAM_ATTR void hop(radio_t *r, int ridx) {
    if (!r->vivo) return;
    if (ridx == g_scan_idx) return;
    const tablas_t *t = __atomic_load_n(&g_tab, __ATOMIC_ACQUIRE);
    if (t->len[ridx] == 0) return;

    r->idx++;
    if (r->idx >= t->len[ridx]) { r->idx = 0; r->vueltas++; }
    uint8_t ch = t->ch[ridx][r->idx];
    r->canal = ch;
    if (r->bang_ok) bang_write(r, CMD_W_REGISTER | NRF_RF_CH, ch);
    else            nrf_write_reg(r, NRF_RF_CH, ch);
    r->hops++;
}

/* ===================== escalera de dwell ===================== */
static inline IRAM_ATTR void escalon_subir(void) {
    if (g_escalon + 1 < N_ESCALONES) {
        g_escalon++;
        g_dwell = k_dwell[g_escalon];
    } else if (DWELL_LOOP) {
        g_escalon = 0;
        g_dwell = k_dwell[0];
    } else {
        return;
    }
    g_pulso_hasta = esp_timer_get_time() + 600000;
}

/* ===================== scan con VENTANA DE SILENCIO ===================== */
static radio_t *scan_radio(void) {
    for (int k = 1; k <= 3; k++) {
        int i = 3 + ((scan_radio_idx - 3 + k + 3) % 3);
        if (radios[i].vivo) { scan_radio_idx = i; return &radios[i]; }
    }
    return NULL;
}

static inline void todos_ce_lo(void) {
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) ce_lo(&radios[i]);
}
static inline void todos_ce_hi(void) {
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) ce_hi(&radios[i]);
}

static void radio_devolver_tx(radio_t *r) {
    if (!r || !r->vivo) return;
    ce_lo(r);
    nrf_write_reg(r, NRF_CONFIG, CONFIG_TX);
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_CW);
    ce_hi(r);
}

static inline void scan_iniciar(void) {
    radio_t *r = scan_radio();
    if (!r) { scan_activo = false; g_scan_idx = -1; return; }

    todos_ce_lo();
    g_quiet = true;

    g_scan_idx = (int)(r - radios);
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);
    nrf_write_reg(r, NRF_CONFIG, CONFIG_RX);
    ce_hi(r);

    memset(g_cuenta, 0, sizeof(g_cuenta));
    scan_pasada = 0;
    scan_ch = 0;
    scan_activo = true;
    scan_t0 = esp_timer_get_time();
}

static inline void scan_paso(void) {
    if (g_scan_idx < 0 || g_scan_idx >= NUM_RADIOS) { scan_activo = false; return; }
    radio_t *r = &radios[g_scan_idx];
    if (!r->vivo) { scan_activo = false; g_scan_idx = -1; return; }
    nrf_write_reg(r, NRF_RF_CH, (uint8_t)(CH_LO + scan_ch));
    esp_rom_delay_us(200);
    if (nrf_read_reg(r, NRF_RPD) & 0x01) g_cuenta[scan_ch]++;

    scan_ch++;
    if (scan_ch >= NUM_CANALES) { scan_ch = 0; scan_pasada++; }
}

static inline void score_actualizar(void) {
    for (int c = 0; c < NUM_CANALES; c++) {
        if (g_cuenta[c] >= SCAN_MUESTRAS) { g_score[c] = 0; continue; }
        uint32_t s = (uint32_t)g_score[c] + (uint32_t)g_cuenta[c] * SCORE_ONE;
        s -= s / SCORE_DEC;
        if (s > 0xFFFFu) s = 0xFFFFu;
        g_score[c] = (uint16_t)s;
    }
}

static inline void scan_fin(void) {
    if (g_scan_idx >= 0 && g_scan_idx < NUM_RADIOS)
        radio_devolver_tx(&radios[g_scan_idx]);

    score_actualizar();
    hist_push(g_cuenta);
    if (g_scans < 255) g_scans++;
    g_scan_idx = -1;

    todos_ce_hi();
    g_quiet = false;
    g_ventana_us = (uint32_t)(esp_timer_get_time() - scan_t0);

    scan_activo = false;
    recalcular();
    g_flash = true;
}

/* ===================== LED ===================== */
static void led_set(uint8_t rr, uint8_t gg, uint8_t bb) {
    led_strip_set_pixel(led, 0, rr, gg, bb);
    led_strip_refresh(led);
}
static void led_init(void) {
    led_strip_config_t sc = { .strip_gpio_num = LED_GPIO, .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB };
    led_strip_rmt_config_t rc = { .clk_src = RMT_CLK_SRC_DEFAULT, .resolution_hz = 10 * 1000 * 1000 };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&sc, &rc, &led));
    led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 60, 0); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 0, 60); vTaskDelay(pdMS_TO_TICKS(120));
    led_set(0, 0, 0);
}

static void led_hsv(uint8_t h, uint8_t s, uint8_t v) {
    uint8_t reg = (uint8_t)(h / 43);
    uint8_t rem = (uint8_t)((h - reg * 43) * 6);
    uint8_t p = (uint8_t)((v * (255 - s)) >> 8);
    uint8_t q = (uint8_t)((v * (255 - ((s * rem) >> 8))) >> 8);
    uint8_t t = (uint8_t)((v * (255 - ((s * (255 - rem)) >> 8))) >> 8);
    switch (reg) {
        case 0:  led_set(v, t, p); break;
        case 1:  led_set(q, v, p); break;
        case 2:  led_set(p, v, t); break;
        case 3:  led_set(p, q, v); break;
        case 4:  led_set(t, p, v); break;
        default: led_set(v, p, q); break;
    }
}

static void led_update(void) {
    static uint8_t hue = 0;

    if (g_vivos == 0) { led_set(LED_BRILLO, 0, 0); return; }
    if (g_flash) { g_flash = false; led_set(LED_BRILLO, LED_BRILLO, LED_BRILLO); return; }

    if (esp_timer_get_time() < g_pulso_hasta) {
        hue = (uint8_t)(hue + 48);
        led_hsv(hue, 255, LED_BRILLO);
        return;
    }

    uint8_t v = (g_vivos < NUM_RADIOS) ? (uint8_t)(LED_BRILLO / 3)
                                       : (uint8_t)(LED_BRILLO / 2);
    hue = (uint8_t)(hue + 2);
    led_hsv(hue, 255, v);
}

/* ===================== tasks ===================== */
static void IRAM_ATTR jam_bus2(void *arg) {
    int64_t t_cede = esp_timer_get_time();
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[0], 0);
        hop(&radios[1], 1);
        hop(&radios[2], 2);
        uint32_t d = g_dwell * CPU_MHZ;
        d += rnd() % (d / 2);              /* +0..50% : rompe la periodicidad */
        while (cyc() - c0 < d) { }
        int64_t now = esp_timer_get_time();
        if (now - t_cede >= (int64_t)CEDE_MS * 1000) { t_cede = now; vTaskDelay(1); }
    }
}

static void IRAM_ATTR jam_bus3(void *arg) {
    int64_t t_cede = esp_timer_get_time(), t_scan = esp_timer_get_time(),
            t_led = t_cede, t_esc = t_cede;
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[3], 3);
        hop(&radios[4], 4);
        hop(&radios[5], 5);

        int64_t now = esp_timer_get_time();

#if VENTANA_SCAN
        if (!scan_activo && !g_quiet && now - t_scan >= (int64_t)REESCANEO_MS * 1000) {
            t_scan = now;
            scan_iniciar();
        }
        if (scan_activo) {
            scan_paso();
            if (scan_pasada >= SCAN_MUESTRAS ||
                (scan_ch == 0 && (now - scan_t0) >= (int64_t)SCAN_HARD_MS * 1000)) {
                scan_fin();
            }
        }
#endif

        uint32_t d = g_dwell * CPU_MHZ;
        d += rnd() % (d / 2);              /* +0..50% : rompe la periodicidad */
        while (cyc() - c0 < d) { }

        now = esp_timer_get_time();
        if (now - t_esc >= (int64_t)ESCALON_MS * 1000) { t_esc = now; escalon_subir(); }
        if (now - t_led >= 64000) { t_led = now; led_update(); }
        if (now - t_cede >= (int64_t)CEDE_MS * 1000) { t_cede = now; vTaskDelay(1); }
    }
}

/* ===================== monitor ===================== */
static void mapa_linea(char *out, int n) {
    int i = 0;
    for (int c = 0; c < NUM_CANALES && i < n - 1; c++) {
        uint32_t s = g_score[c] >> SCORE_SHIFT;
        out[i++] = (s == 0) ? '.' : (s < SCORE_MIN) ? 'o'
                 : (s < SAT_MIN)   ? 'O' : (s < 64) ? '#' : '@';
    }
    out[i] = 0;
}

static void monitor_task(void *arg) {
    uint32_t prev[NUM_RADIOS];
    int64_t  tprev[NUM_RADIOS];
    for (int i = 0; i < NUM_RADIOS; i++) { prev[i] = 0; tprev[i] = esp_timer_get_time(); }
    char mapa[NUM_CANALES + 1];

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int64_t now = esp_timer_get_time();
        const tablas_t *t = __atomic_load_n(&g_tab, __ATOMIC_ACQUIRE);

        int vivos = 0, len_max = 0;
        uint32_t us = 0xFFFFFFFFu;
        for (int i = 0; i < NUM_RADIOS; i++) {
            radio_t *r = &radios[i];
            if (!r->vivo) continue;
            vivos++;
            if (t->len[i] > len_max) len_max = t->len[i];
            uint32_t dh = r->hops - prev[i];
            int64_t  dt = now - tprev[i];
            prev[i] = r->hops; tprev[i] = now;
            if (dh && dt > 0) {
                uint32_t u = (uint32_t)(dt / dh);
                if (u < us) us = u;
            }
        }
        if (vivos == 0 || len_max == 0) continue;
        if (us == 0xFFFFFFFFu) us = 0;

        uint32_t gap       = (uint32_t)(len_max - 1) * g_dwell;
        uint32_t ocup      = 100u / (uint32_t)len_max;
        uint32_t est       = (g_dwell > 130u) ? ((g_dwell - 130u) * 100u / g_dwell) : 0;
        uint32_t estable   = ocup * est / 100u;
        uint32_t cobertura = (t->n_total > 0) ? ((uint32_t)vivos * 100u / t->n_total) : 0;
        uint32_t perdida   = (g_ventana_us > 0) ? (g_ventana_us * 100u / ((uint32_t)REESCANEO_MS * 1000u)) : 0;

        ESP_LOGI(TAG, "%s | obj=%d | L=%d | GAP=%luus | esc=%u/%d dwell=%lu",
                 g_cazando ? "CAZA" : "BARRIDO", t->n_total, len_max,
                 (unsigned long)gap, (unsigned)g_escalon, N_ESCALONES - 1,
                 (unsigned long)g_dwell);
        ESP_LOGI(TAG, "  ocup/canal=%lu%%  PLL_est=%lu%%  =>  ESTABLE=%lu%%  cobertura=%lu%%  ventana=%luus (-%lu%%)",
                 (unsigned long)ocup, (unsigned long)est, (unsigned long)estable,
                 (unsigned long)cobertura, (unsigned long)g_ventana_us,
                 (unsigned long)(perdida > 100 ? 100 : perdida));
        ESP_LOGI(TAG, "  DH3(%uus):%s  DH5(%uus):%s  SPI=%dMHz vivos=%d scan=%s",
                 (unsigned)PAQUETE_DH3, (gap < PAQUETE_DH3) ? "OK" : "NO",
                 (unsigned)PAQUETE_DH5, (gap < PAQUETE_DH5) ? "OK" : "NO",
                 g_spi_hz / 1000000, vivos,
                 (g_scan_idx >= 0) ? radios[g_scan_idx].nombre : "-");

        mapa_linea(mapa, sizeof(mapa));
        ESP_LOGI(TAG, "  MAPA[2..80] .=0 o<%d O<SAT #<64 @=denso:", SCORE_MIN);
        ESP_LOGI(TAG, "  %s", mapa);
    }
}

/* ===================== piso de ruido propio ===================== */
static void test_piso_ruido(void) {
    radio_t *r = NULL;
    for (int i = 5; i >= 3; i--) if (radios[i].vivo) { r = &radios[i]; break; }
    if (!r) { ESP_LOGW(TAG, "PISO: sin radio disponible"); return; }

    const uint8_t CH = 40;
    const uint8_t probes[3] = { CH, CH + 2, CH + 20 };

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].vivo || &radios[i] == r) continue;
        nrf_write_reg(&radios[i], NRF_RF_CH, CH);
    }

    int cocal[3] = {0,0,0}, lejos[3] = {0,0,0};

    for (int p = 0; p < 3; p++) {
        ce_lo(r);
        nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);
        nrf_write_reg(r, NRF_CONFIG, CONFIG_RX);
        ce_hi(r);
        nrf_write_reg(r, NRF_RF_CH, probes[p]);
        esp_rom_delay_us(200);
        if (nrf_read_reg(r, NRF_RPD) & 0x01) cocal[p] = 1;
    }

    todos_ce_lo();
    for (int p = 0; p < 3; p++) {
        ce_lo(r);
        nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);
        nrf_write_reg(r, NRF_CONFIG, CONFIG_RX);
        ce_hi(r);
        nrf_write_reg(r, NRF_RF_CH, probes[p]);
        esp_rom_delay_us(200);
        if (nrf_read_reg(r, NRF_RPD) & 0x01) lejos[p] = 1;
    }

    radio_devolver_tx(r);
    todos_ce_hi();

    ESP_LOGI(TAG, "PISO (ch%d): co-canal TX=%d sil=%d | +2 TX=%d sil=%d | +20 TX=%d sil=%d",
             CH, cocal[0], lejos[0], cocal[1], lejos[1], cocal[2], lejos[2]);
    if (cocal[0] && !lejos[0])
        ESP_LOGW(TAG, "  Te oyes a ti mismo: la ventana de silencio es OBLIGATORIA.");
    else if (cocal[0])
        ESP_LOGW(TAG, "  Hay otra fuente de RF en el canal %d.", CH);
    else
        ESP_LOGI(TAG, "  Buen aislamiento en el mismo canal.");
}

/* ===================== arranque ===================== */
void app_main(void) {
    led_init();

    park_all_pins();

    bus_init(SPI2_HOST, GPIO_NUM_12, GPIO_NUM_11, GPIO_NUM_13);
    bus_init(SPI3_HOST, GPIO_NUM_18, GPIO_NUM_8,  GPIO_NUM_7);

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) radios[i].vivo = radio_init(&radios[i]);
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) activos++;

    if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF. Revisa alimentacion.");
        for (;;) { led_set(60, 0, 0); vTaskDelay(pdMS_TO_TICKS(200)); }
    }

    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].vivo) continue;
        jam_enter(&radios[i]);
        bang_detectar(&radios[i]);
        radios[i].bang_sucio = true;
    }

    tablas_barrido(&g_buf[0]);
    tablas_barrido(&g_buf[1]);
    __atomic_store_n(&g_tab, &g_buf[0], __ATOMIC_RELEASE);

    g_jam_ok = (activos > 0);
    g_vivos = activos;
    g_dwell = k_dwell[0];
    g_escalon = 0;

    hist_init();
    test_piso_ruido();

    xTaskCreatePinnedToCore(jam_bus2, "bus2", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    xTaskCreatePinnedToCore(jam_bus3, "bus3", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(monitor_task, "mon", 3072, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "CAZADOR v9.1 · %d/6 radios · BARRIDO BARAJADO 79 canales · 2 Mbps 0 dBm",
             activos);
    ESP_LOGI(TAG, "Escalera dwell: %u -> %u us en %d pasos de %u ms%s",
             (unsigned)k_dwell[0], (unsigned)k_dwell[N_ESCALONES - 1],
             N_ESCALONES - 1, (unsigned)ESCALON_MS, DWELL_LOOP ? " (en loop)" : "");
    ESP_LOGI(TAG, "Barajado Fisher-Yates + jitter dwell +0..50%% · VENTANA_SCAN=%d · FORZAR_BARRIDO=%d",
             VENTANA_SCAN, FORZAR_BARRIDO);
    ESP_LOGI(TAG, "Arcoiris lento = barriendo · pulso = cambio de dwell · rojo = sin radios");
    vTaskDelay(portMAX_DELAY);
}
