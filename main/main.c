/*
 * main.c
 * ESP32-S3 N16R8 (ESP-IDF v6.0.3) — Jammer BT/BLE con 3x nRF24L01+ PA/LNA
 *
 *   NRF2 -> SPI2 (CS 9)   canales  2..27   tarea en núcleo 0
 *   NRF1 -> SPI2 (CS 10)  canales 28..53   tarea en núcleo 1
 *   NRF3 -> SPI3 (CS 21)  canales 54..80   tarea en núcleo 1
 *
 * Sin consola, sin comandos. Se enchufa y transmite.
 *
 * Modo: portadora continua (CONT_WAVE + PLL_LOCK) barriendo el rango que de
 * verdad usa Bluetooth (2402-2480 MHz = canales nRF 2..80), partido entre los
 * tres radios para cubrirlo entero.
 *
 * Dwell automático: cada 5 s sube un peldaño, de 150 us a 170 us. Cuando
 * llega al techo vuelve al piso. Así se prueban las dos estrategias a la vez:
 *   - dwell bajo  = mucha frecuencia de visita, blips cortos
 *   - dwell alto  = pocas visitas pero cada una corrompe paquetes de verdad
 *
 * LED WS2812 (GPIO48), brillo base 20%:
 *   ARCOIRIS lento y suave (senos desfasados + respiracion de brillo)
 *     : los 3 NRF vivos y jamming
 *   APAGADO        : algun NRF no responde (no se prende nada)
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

static const char *TAG = "JAM";

// ---------- LED ----------
#define LED_GPIO        GPIO_NUM_48
#define LED_BRILLO      51      /* 20% de 255: brillo base, tenue a proposito */

/* Arcoiris estilo "gamer": senos continuos en vez de HSV entero (sin
 * bandas), mas lento, y con respiracion de brillo superpuesta. */
#define LED_CICLO_US     18000000.0   /* 18 s por vuelta completa de color   */
#define LED_BREATH_US     6000000.0   /* 6 s por ciclo de respiracion        */
#define LED_BREATH_MIN     0.55       /* piso de brillo durante la respiracion */
#define LED_REFRESCO_MS     16        /* ~60 fps: transicion imperceptible   */

static led_strip_handle_t led;
static volatile bool g_jam_ok = false;   /* true solo si los NUM_RADIOS viven */

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
#define NUM_RADIOS  3

/* Rango útil de BT/BLE: 2402-2480 MHz. Fuera de aca es espectro desperdiciado. */
#define JAM_CH_LO   2
#define JAM_CH_HI   80
#define JAM_CH_N    (JAM_CH_HI - JAM_CH_LO + 1)

/* ---- Estrategia de cobertura ----
 * El duty por canal es 1/(canales que cubre cada radio), y NO depende del
 * dwell: un burst mas largo sobre un barrido mas lento da el mismo duty.
 *
 *   JAM_SPAN = 79 -> los 3 radios se reparten el espectro (26/26/27).
 *                   Cobertura total, 3.8% de duty. AFH-proof.
 *   JAM_SPAN = 26 -> los 3 radios cubren EL MISMO subconjunto. 11.5% de
 *                   duty (+4.8 dB), pero deja 53 canales libres y ahi el
 *                   AFH de BT puede mover el enlace y esquivarnos.
 *
 * Semillas distintas por radio => en cada instante estan en canales
 * diferentes dentro de la ventana, asi que se solapan poco. */
#define JAM_SPAN   26
#define JAM_BASE    JAM_CH_LO

/* Piso del silicio: el PLL del nRF tarda 130 us tipicos en enganchar al
 * cambiar de canal. 150 us = 130 de settle + 20 de margen. Por debajo de eso
 * el carrier no se estabiliza. El overhead de esta app es ~0 (el us/hop
 * medido coincide con el dwell), asi que este es el unico limite.
 * Techo 170 us: por encima de eso la vuelta completa pasa de 6.6 ms y el duty
 * por canal cae sin ganar nada. 150-170 dieron silencio confirmado. */
static const uint32_t dwell_ladder[] = { 150, 160, 170 };
#define DWELL_STEPS      (sizeof(dwell_ladder) / sizeof(dwell_ladder[0]))
#define DWELL_PERIOD_MS  5000

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
    /* Regla: nunca dos radios del MISMO bus en el MISMO nucleo.
     *   core 0 -> NRF2 (SPI2), solo
     *   core 1 -> NRF1 (SPI2) + NRF3 (SPI3): buses distintos, corren en
     *            paralelo real sin contenerse por el lock del bus.
     * NRF2 y NRF1 comparten SCK=12 MOSI=11 MISO=13 con CS distintos. */
    { .nombre = "NRF2", .host = SPI2_HOST,
      .ce = GPIO_NUM_5,  .cs = GPIO_NUM_9,  .sck = GPIO_NUM_12,
      .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13,
      .seed = 0xA17E56u, .core = 0 },
    { .nombre = "NRF1", .host = SPI2_HOST,
      .ce = GPIO_NUM_4,  .cs = GPIO_NUM_10, .sck = GPIO_NUM_12,
      .mosi = GPIO_NUM_11, .miso = GPIO_NUM_13,
      .seed = 0xA17E55u, .core = 1 },
    { .nombre = "NRF3", .host = SPI3_HOST,
      .ce = GPIO_NUM_1,  .cs = GPIO_NUM_21, .sck = GPIO_NUM_18,
      .mosi = GPIO_NUM_8, .miso = GPIO_NUM_7,
      .seed = 0xA17E57u, .core = 1 },
};

static gptimer_handle_t dwell_timer;
static volatile uint32_t g_dwell = dwell_ladder[0];
static volatile bool     g_flash = false;      /* destello de LED */

/* CE por la API del driver. Las escrituras directas a 0x60004008/0x0C no
 * las pudimos verificar en este chip: si apuntan a otro registro el CE nunca
 * se mueve y el carrier queda clavado en un solo canal (no jamea nada).
 * gpio_set_level() cuesta ~2-3us; a 180-480us de dwell eso es 1-2%. */
#define GPIO_OUT_REG_A  (*(volatile uint32_t *)0x60004004u)   /* pines 0-31  */
#define GPIO_OUT1_REG_A (*(volatile uint32_t *)0x60004010u)   /* pines 32-45 */

static inline void ce_hi(radio_t *r) { gpio_set_level(r->ce, 1); }
static inline void ce_lo(radio_t *r) { gpio_set_level(r->ce, 0); }

/* Lee el pin de vuelta: si esto da siempre lo mismo, el CE no togglea. */
static inline int ce_state(radio_t *r) {
    uint32_t v = (r->ce < 32) ? GPIO_OUT_REG_A : GPIO_OUT1_REG_A;
    int bit = (r->ce < 32) ? (int)r->ce : (int)((uint32_t)r->ce - 32);
    return (int)((v >> bit) & 1u);
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

static void generar_tabla(radio_t *r) {
    int span = JAM_SPAN;
    if (span > JAM_CH_N) span = JAM_CH_N;
    int lo = JAM_BASE;
    int hi = lo + span - 1;
    if (hi > JAM_CH_HI) hi = JAM_CH_HI;

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
    ce_lo(r);                                       /* corta el carrier */
    nrf_write_reg(r->spi, NRF_RF_CH, (uint8_t)(ch & 0x7F));
    ce_hi(r);                                       /* carrier arriba YA */
    /* Los ~130us de asentamiento del PLL ocurren con el carrier ya radiando,
     * durante el dwell. NO hay que esperarlos con CE abajo: eso le corta el
     * tiempo de aire a ~25us por salto y el jammer deja de hacer nada. */
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
        .max_transfer_sz = 64,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
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

/* ---------- LED: arcoiris suave por senos + respiracion de brillo ---------- */
/* Tres senos desfasados 120 grados (2*pi/3) dan un barrido de color
 * continuo en punto flotante: sin los "escalones" que mete el HSV entero.
 * Encima, el brillo "respira" con su propio seno mas lento (independiente
 * del color), que es justo el efecto que usan los perifericos gamer en vez
 * de solo girar el matiz a brillo fijo. */
static void arcoiris_suave(int64_t ahora_us, uint8_t *r, uint8_t *g, uint8_t *b) {
    double t = (double)ahora_us;

    double fase_color = 2.0 * M_PI * t / LED_CICLO_US;
    double rr = (sin(fase_color) + 1.0) * 0.5;
    double gg = (sin(fase_color + 2.0 * M_PI / 3.0) + 1.0) * 0.5;
    double bb = (sin(fase_color + 4.0 * M_PI / 3.0) + 1.0) * 0.5;

    double fase_resp = 2.0 * M_PI * t / LED_BREATH_US;
    double resp = LED_BREATH_MIN
                + (1.0 - LED_BREATH_MIN) * ((sin(fase_resp) + 1.0) * 0.5);
    double brillo = (double)LED_BRILLO * resp;

    *r = (uint8_t)(rr * brillo + 0.5);
    *g = (uint8_t)(gg * brillo + 0.5);
    *b = (uint8_t)(bb * brillo + 0.5);
}

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
        if (!g_jam_ok) {
            /* Falta algun NRF: no se prende nada. */
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        if (g_flash) {
            g_flash = false;
            led_set(LED_BRILLO, LED_BRILLO, LED_BRILLO);
            vTaskDelay(pdMS_TO_TICKS(120));
            led_set(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }
        /* Los NUM_RADIOS vivos y transmitiendo: arcoiris lento, suave y
         * respirando, estilo periferico gamer. */
        int64_t ahora = esp_timer_get_time();
        uint8_t r, g, b;
        arcoiris_suave(ahora, &r, &g, &b);
        led_set(r, g, b);
        vTaskDelay(pdMS_TO_TICKS(LED_REFRESCO_MS));  /* ~60 fps */
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

    g_jam_ok = (activos == NUM_RADIOS);   /* arcoiris solo con los 3 vivos */

    xTaskCreate(led_task,  "led",   2048, NULL, 3, NULL);
    xTaskCreate(dwell_task, "dwell", 3072, NULL, 2, NULL);

    ESP_LOGI(TAG, "JAM ON  ·  %d radio(s)  ·  canales %d..%d  ·  dwell %lu us",
             activos, JAM_CH_LO, JAM_CH_HI, (unsigned long)g_dwell);
    ESP_LOGI(TAG, "escalera de dwell: %lu..%lu us cada %d ms",
             (unsigned long)dwell_ladder[0],
             (unsigned long)dwell_ladder[DWELL_STEPS - 1], DWELL_PERIOD_MS);

    /* log de estado: no acepta comandos, solo informa */
    uint32_t prev[NUM_RADIOS] = { 0 };
    int64_t  tprev[NUM_RADIOS] = { 0 };
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int64_t now = esp_timer_get_time();
        for (int i = 0; i < NUM_RADIOS; i++) {
            radio_t *r = &radios[i];
            if (!r->spi) continue;
            uint32_t dh = r->hops - prev[i];
            uint32_t us_hop = dh ? (uint32_t)((now - tprev[i]) / dh) : 0;
            prev[i]  = r->hops;
            tprev[i] = now;
            ESP_LOGI(TAG, "%s %4lu saltos/s (%lu us/hop) canal=%2u libres=%2u vueltas=%lu dwell=%lu CE=%d",
                     r->nombre, (unsigned long)(dh / 2), (unsigned long)us_hop,
                     (unsigned)r->canal, (unsigned)r->order_len,
                     (unsigned long)r->vueltas, (unsigned long)g_dwell,
                     ce_state(r));
        }
    }
}
