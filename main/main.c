/*
 * main.c — CAZADOR v8 — 6x nRF24L01+ PA/LNA — ESP32-S3 N16R8 — ESP-IDF v6.0.3
 *
 * SPI2 (SCK 12, MOSI 11, MISO 13): NRF1(CE 4 CS 10) NRF2(CE 5 CS 9)  NRF3(CE 6 CS 14)
 * SPI3 (SCK 18, MOSI 8,  MISO 7):  NRF4(CE 1 CS 21) NRF5(CE 15 CS 16) NRF6(CE 17 CS 38)
 * WS2812 en GPIO 48.
 *
 * ====== QUE HACE ======
 * 1. Arranca en BARRIDO: los 6 radios cubren los 79 canales de BT Classic.
 * 2. Cada ~1.2 s hace una VENTANA DE SILENCIO: baja el CE de los 6 radios (el
 *    PA/LNA se apaga), y un radio prestado escanea los 79 canales con el RPD.
 * 3. Acumula confianza por canal (score con decaimiento real).
 * 4. Cuando el mapa converge, pasa a CAZA: los 6 radios martillan SOLO los
 *    canales que el detector dice que son del piconet. Revisita ~4x mejor.
 * 5. Si el mapa se queda sin candidatos, vuelve a BARRIDO. Nunca para.
 *
 * ====== POR QUE LA VENTANA DE SILENCIO (no es opcional) ======
 * El escaner esta a centimetros de 5 transmisores de +18 dBm. Con 30-40 dB de
 * aislamiento entre modulos en la misma PCB llegan -17 dBm a su antena: 47 dB
 * POR ENCIMA del umbral del RPD (-64 dBm). Y el rechazo de canal adyacente del
 * GFSK es pobre (C/I1ST ~8 dB, >12 MHz -40 dB, tabla de selectivity del
 * datasheet Nordic), asi que la fuga alcanza ~30 canales.
 * Sin ventana, el mapa sale TODO saturado -> ne=0 -> BARRIDO para siempre.
 * Bajar CE es legal: standby-I no emite, y escribir RF_CH en standby tambien.
 * El hopping sigue corriendo, la fase no se pierde.
 *
 * ====== POR QUE EL SCORE ES EN PUNTO FIJO ======
 * s = score + cuenta; s -= s/DEC. Con division ENTERA y DEC=24, cualquier
 * score entre 1 y 23 NUNCA decae: el mapa no olvidaria nada y los falsos
 * positivos se acumulan para siempre. Guardado en 1/256 el decremento existe
 * para cualquier valor.
 *
 * ====== CALIBRACION (formula, no ajuste a ojo) ======
 * El equilibrio de s = s + c - s/DEC es s = DEC*c, y c = MUESTRAS * duty.
 *     score_eq = SCORE_DEC * SCAN_MUESTRAS * duty
 *     BT (5%)   = 24*3*0.05  = 3.6
 *     ruido(1%) = 24*3*0.01  = 0.72
 *     WiFi(85%) = 24*3*0.85  = 61
 * SCORE_MIN entre ruido y BT (2). SAT_MIN entre BT y WiFi (12).
 * Convergencia: el deficit decae por (1-1/DEC) por scan, asi que llegar a
 * SCORE_MIN=2 desde 0 sobre un equilibrio de 3.6 tarda
 *     n = ln(1 - 2/3.6) / ln(23/24) = 19 scans  (~23 s a 1.2 s/scan)
 * El fallback ne==0 -> BARRIDO cubre ese periodo sin necesidad de warmup largo.
 *
 * ====== GEOMETRIA ======
 * gap = (L-1)*d, NO la revisita. Un paquete de duracion T cae en una ventana
 * de golpeo garantizada si T > gap. Con d=320 y GAP_MAX=1200 -> L=4, 24 canales,
 * gap 960 us: cubre DH3 (1622) con 662 us de margen.
 *
 * ====== DOS METRICAS DE DUTY ======
 * Ocupacion = 1/L (fraccion del radio en ESE canal).
 * Estable = (d-130)/d (fraccion con PLL enganchado; debajo de 130 us la
 * portadora BARRE y la potencia se dispersa). d=320 -> estable 59%.
 *
 * ====== COSTE DE LA VENTANA ======
 * 3 pasadas x 79 canales x ~330 us = ~75 ms cada 1200 ms = 6.3% del duty de
 * jamming. A cambio el mapa es VERDAD. El valor real se imprime en el log.
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

static const char *TAG = "CAZA";

/* ===================== CONFIG ===================== */
#define CH_LO        2
#define CH_HI        80
#define NUM_CANALES  (CH_HI - CH_LO + 1)      /* 79 */
#define NUM_RADIOS   6

#define DWELL_US     320u        /* > 130us: PLL engancha, potencia plena */

#define PAQUETE_DH3  1622u       /* duracion real de paquete DH3 */
#define PAQUETE_DH5  2870u       /* duracion real de paquete DH5 (BR) */

#define SCAN_MUESTRAS  3         /* pasadas completas por ventana */
#define REESCANEO_MS   1200      /* cada cuanto se escanea */
#define SCAN_HARD_MS   900       /* techo duro; solo corta en frontera de pasada */

/* Score en 1/256 de unidad (punto fijo). Ver cabecera. */
#define SCORE_SHIFT   8
#define SCORE_ONE     (1u << SCORE_SHIFT)

/* BT~3.6 · ruido~0.72 · WiFi~61 */
#define SCORE_DEC      24
#define SCORE_MIN      2         /* entre ruido y BT */
#define SAT_MIN        12        /* entre BT y WiFi */

#define WARMUP_SCANS   12        /* el fallback ne==0 cubre el resto */

#define SPI_HZ_RAPIDO (16 * 1000 * 1000)
#define SPI_HZ_SEGURO (10 * 1000 * 1000)

#define CPU_MHZ      240u
#define CEDE_MS      2000

/* gap = (L-1)*d < GAP_MAX. 1200 deja 422us de margen sobre el DH3. */
#define GAP_MAX_US   1200u
#define L_MAX  ((GAP_MAX_US / DWELL_US) + 1u)
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
static volatile bool g_flash  = false;
static volatile bool g_cazando = false;
static volatile bool g_quiet  = false;

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
static uint16_t g_score[NUM_CANALES];     /* 1/256 de unidad */
static int      scan_ch = 0;
static uint8_t  scan_pasada = 0;
static uint8_t  g_scans = 0;
static bool     scan_activo = false;
static int64_t  scan_t0 = 0;
static int      scan_radio_idx = 5;
static volatile int g_scan_idx = -1;
static volatile uint32_t g_ventana_us = 0;

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

/* El driver reescribe user/ms_dlen/dma_conf en cada transaccion. Cualquier
 * transaccion por driver marca a TODOS los radios de ese bus, y cada uno se
 * re-arma en su propio proximo bang_write. */
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

static bool radio_add(radio_t *r, int hz) {
    spi_device_interface_config_t dc = {
        .clock_speed_hz = hz, .mode = 0, .spics_io_num = -1, .queue_size = 1,
    };
    if (spi_bus_add_device(r->host, &dc, &r->spi) != ESP_OK) return false;
    nrf_write_reg(r, NRF_RF_CH, 0x5A);
    return nrf_read_reg(r, NRF_RF_CH) == 0x5A;
}

static bool radio_init(radio_t *r) {
    r->cs_alto = (r->cs >= 32);
    gpio_set_direction(r->ce, GPIO_MODE_OUTPUT);
    gpio_set_direction(r->cs, GPIO_MODE_OUTPUT);
    ce_lo(r);
    cs_hi(r);
    r->hw = spi_hw(r->host);

    if (!radio_add(r, SPI_HZ_RAPIDO)) {
        if (r->spi) spi_bus_remove_device(r->spi);
        r->spi = NULL;
        if (!radio_add(r, SPI_HZ_SEGURO)) {
            r->spi = NULL;
            ESP_LOGE(TAG, "%s NO RESPONDE (CS=%d CE=%d)", r->nombre, (int)r->cs, (int)r->ce);
            return false;
        }
        g_spi_hz = SPI_HZ_SEGURO;
        ESP_LOGW(TAG, "%s cayo a SPI 10 MHz", r->nombre);
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
static void IRAM_ATTR tablas_barrido(tablas_t *t) {
    int vivos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].vivo) vivos++;
    if (vivos == 0) vivos = 1;

    int por = NUM_CANALES / vivos;
    int usados = 0;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].vivo) { t->len[i] = 0; continue; }
        int lo = CH_LO + usados * por;
        int hi = lo + por - 1;
        if (usados == vivos - 1) hi = CH_HI;
        uint8_t n = 0;
        for (int c = lo; c <= hi && n < MAX_TAB; c++) t->ch[i][n++] = (uint8_t)c;
        t->len[i] = n;
        usados++;
    }
    t->n_total = NUM_CANALES;
    t->n_rafaga = 0;
    g_cazando = false;
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

/* Caza. Solo rafagas con confianza suficiente. Los densos (WiFi) se excluyen:
 * no son el piconet y cada radio gastado en ellos es un radio que no machaca
 * BT. Si sobran radios, se duplican las mejores rafagas. */
static void IRAM_ATTR tablas_caza(tablas_t *t, int n_max) {
    uint8_t  elig[MAX_TAB];
    uint16_t rank[MAX_TAB];
    int ne = 0, nraf = 0;

    for (int c = 0; c < NUM_CANALES; c++) {
        if (g_cuenta[c] >= SCAN_MUESTRAS) continue;      /* saturado este scan */
        uint32_t sc = g_score[c] >> SCORE_SHIFT;
        if (sc < SCORE_MIN) continue;                     /* sin confianza */
        if (sc >= SAT_MIN) continue;                      /* denso: no es BT */
        uint16_t rk = (uint16_t)((g_score[c] >> 4) + g_cuenta[c]);
        insertar(elig, rank, &ne, n_max, (uint8_t)(CH_LO + c), rk);
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
    if (ridx == g_scan_idx) return;          /* radio prestado al scan */
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

/* NO puede fallar: si no levanta el CE, la placa queda muda para siempre. */
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

    todos_ce_lo();          /* toda la placa a standby: nadie emite */
    g_quiet = true;

    g_scan_idx = (int)(r - radios);
    nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);   /* sin CONT_WAVE */
    nrf_write_reg(r, NRF_CONFIG, CONFIG_RX);
    ce_hi(r);                                        /* solo el escaner en RX */

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
    esp_rom_delay_us(200);              /* 130us sintetizador + 40us AGC */
    if (nrf_read_reg(r, NRF_RPD) & 0x01) g_cuenta[scan_ch]++;

    scan_ch++;
    if (scan_ch >= NUM_CANALES) { scan_ch = 0; scan_pasada++; }
}

/* PUNTO FIJO x256: el decremento existe para cualquier score. Con s/DEC entero
 * un score < DEC NUNCA decaeria y el mapa no olvidaria nada. */
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
    /* TODOS los caminos de salida levantan los CE */
    if (g_scan_idx >= 0 && g_scan_idx < NUM_RADIOS)
        radio_devolver_tx(&radios[g_scan_idx]);

    score_actualizar();
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
static void led_update(void) {
    if (!g_jam_ok) { led_set(0, 0, 0); return; }
    if (g_flash)   { g_flash = false; led_set(LED_BRILLO, LED_BRILLO, LED_BRILLO); return; }
    int64_t t = esp_timer_get_time();
    int fase = (int)((t % 1500000) * 2 / 1500000);
    if (g_cazando) led_set(0, LED_BRILLO, (uint8_t)(LED_BRILLO / 3 * fase));
    else           led_set(0, 0, LED_BRILLO);
}

/* ===================== tasks ===================== */
static void IRAM_ATTR jam_bus2(void *arg) {
    int64_t t_cede = esp_timer_get_time();
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[0], 0);
        hop(&radios[1], 1);
        hop(&radios[2], 2);
        /* wait anclado a c0 (antes de los hops): periodo exacto, sin drift */
        uint32_t d = g_dwell * CPU_MHZ;
        while (cyc() - c0 < d) { }
        int64_t now = esp_timer_get_time();
        if (now - t_cede >= (int64_t)CEDE_MS * 1000) { t_cede = now; vTaskDelay(1); }
    }
}

static void IRAM_ATTR jam_bus3(void *arg) {
    int64_t t_cede = esp_timer_get_time(), t_scan = esp_timer_get_time(), t_led = t_cede;
    for (;;) {
        uint32_t c0 = cyc();
        hop(&radios[3], 3);
        hop(&radios[4], 4);
        hop(&radios[5], 5);

        int64_t now = esp_timer_get_time();

        if (!scan_activo && !g_quiet && now - t_scan >= (int64_t)REESCANEO_MS * 1000) {
            t_scan = now;
            scan_iniciar();
        }

        if (scan_activo) {
            scan_paso();
            /* solo corta en frontera de pasada: cortar a media pasada daba
             * mas muestras a los canales bajos */
            if (scan_pasada >= SCAN_MUESTRAS ||
                (scan_ch == 0 && (now - scan_t0) >= (int64_t)SCAN_HARD_MS * 1000)) {
                scan_fin();
            }
        }

        uint32_t d = g_dwell * CPU_MHZ;
        while (cyc() - c0 < d) { }

        now = esp_timer_get_time();
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

        ESP_LOGI(TAG, "%s | obj=%d (rafaga=%d) | L=%d | GAP=%luus | scans=%d | ventana=%luus (-%lu%%)",
                 g_cazando ? "CAZA" : "BARRIDO", t->n_total, t->n_rafaga,
                 len_max, (unsigned long)gap, g_scans,
                 (unsigned long)g_ventana_us, (unsigned long)(perdida > 100 ? 100 : perdida));
        ESP_LOGI(TAG, "  ocup/canal=%lu%%  PLL_est=%lu%%  =>  ESTABLE=%lu%%  cobertura=%lu%%",
                 (unsigned long)ocup, (unsigned long)est, (unsigned long)estable,
                 (unsigned long)cobertura);
        ESP_LOGI(TAG, "  DH3(%uus):%s  DH5(%uus):%s  dwell=%lu SPI=%dMHz vivos=%d",
                 (unsigned)PAQUETE_DH3, (gap < PAQUETE_DH3) ? "OK" : "NO",
                 (unsigned)PAQUETE_DH5, (gap < PAQUETE_DH5) ? "OK" : "NO",
                 (unsigned long)g_dwell, g_spi_hz / 1000000, vivos);

        mapa_linea(mapa, sizeof(mapa));
        ESP_LOGI(TAG, "  MAPA[2..80] .=0 o<%d O<SAT #<64 @=denso:", SCORE_MIN);
        ESP_LOGI(TAG, "  %s", mapa);
    }
}

/* ===================== piso de ruido propio =====================
 * Mide el PEOR CASO: co-canal con un transmisor. radio_add deja RF_CH=0x5A
 * (canal 90, fuera de la banda BT) en los 6 radios, asi que hay que ponerlos
 * en un canal conocido DENTRO de la banda antes de medir. */
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

    for (int p = 0; p < 3; p++) {                    /* con TX */
        ce_lo(r);
        nrf_write_reg(r, NRF_RF_SETUP, RF_SETUP_MAX);
        nrf_write_reg(r, NRF_CONFIG, CONFIG_RX);
        ce_hi(r);
        nrf_write_reg(r, NRF_RF_CH, probes[p]);
        esp_rom_delay_us(200);
        if (nrf_read_reg(r, NRF_RPD) & 0x01) cocal[p] = 1;
    }

    todos_ce_lo();
    for (int p = 0; p < 3; p++) {                    /* en silencio */
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

    g_jam_ok = (activos == NUM_RADIOS);

    test_piso_ruido();

    xTaskCreatePinnedToCore(jam_bus2, "bus2", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);
    xTaskCreatePinnedToCore(jam_bus3, "bus3", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 0);
    xTaskCreatePinnedToCore(monitor_task, "mon", 3072, NULL, 1, NULL, 0);

    ESP_LOGI(TAG, "CAZADOR v8 · %d/6 radios · dwell=%uus · N_max=%d · SPI=%dMHz",
             activos, (unsigned)DWELL_US, N_MAX, g_spi_hz / 1000000);
    ESP_LOGI(TAG, "Arranca en BARRIDO. La caza tarda ~20-25 s en converger.");
    ESP_LOGI(TAG, "BT~%.1f · ruido~%.1f · WiFi~%.1f | SCORE_MIN=%d SAT_MIN=%d",
             (double)SCORE_DEC * SCAN_MUESTRAS * 0.05,
             (double)SCORE_DEC * SCAN_MUESTRAS * 0.01,
             (double)SCORE_DEC * SCAN_MUESTRAS * 0.85,
             SCORE_MIN, SAT_MIN);
    ESP_LOGI(TAG, "Verde = cazando · Azul = barrido · Blanco = recalculo");
    vTaskDelay(portMAX_DELAY);
}
