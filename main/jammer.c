/*
 * jammer.c - nRF24L01+ PA/LNA jammer, ESP-IDF v6.0.x.
 *
 * REGLAS QUE EVITAN LOS BUGS DE ANTES:
 *   1. _rd()/_wr() NUNCA toman el lock. Las publicas si. Sin deadlock.
 *   2. CE ABAJO antes de RF_CH, CE ARRIBA despues del settle. Si no,
 *      el PLL se desengancha y el carrier se congela (bug RF24#714).
 *   3. La tarea bloquea con vTaskDelay. Sin task watchdog.
 *
 * OPTIMIZACIONES DE ATAQUE:
 *   - BLE usa solo canales PARES (2,4..78) + 80. Jammear los impares
 *     es perder el 50% del tiempo. JAM_BLE tiene 40 canales, no 79.
 *   - Orden aleatorio: el AFH del BT clasico no puede predecir el barrido.
 *   - Settle de 150us (datasheet: 130us). Vuelta completa mas rapida.
 */
#include "jammer.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "jam";

/* ================= registros nRF24L01+ ================= */
#define R_CONFIG     0x00
#define R_EN_AA      0x01
#define R_EN_RXADDR  0x02
#define R_SETUP_AW   0x03
#define R_SETUP_RETR 0x04
#define R_RF_CH      0x05
#define R_RF_SETUP   0x06
#define R_STATUS     0x07
#define R_FIFO_STATUS 0x17

#define C_FLUSH_TX   0xE1
#define C_FLUSH_RX   0xE2
#define C_NOP        0xFF

#define CFG_PWR_UP   (1u << 1)
#define CFG_PRIM_RX  (1u << 0)
#define RF_CONT_WAVE (1u << 7)
#define RF_PLL_LOCK  (1u << 4)

#define SETTLE_US 150
#define MAX_CH    80
#define MAX_XFER  33

/* ================= tablas de canales ================= */
static uint8_t ch_adv[3];    /* 2, 26, 80                       */
static uint8_t ch_ble[40];   /* 2,4,6,...,78, 80                */
static uint8_t ch_bt[79];    /* 2,3,4,...,80                    */
static bool    tables_ready;

static void build_tables(void)
{
    ch_adv[0] = 2; ch_adv[1] = 26; ch_adv[2] = 80;

    int n = 0;
    for (uint8_t c = 2; c <= 78; c += 2) ch_ble[n++] = c;
    ch_ble[n++] = 80;                       /* total 40 */

    for (uint8_t c = 2; c <= 80; c++) ch_bt[c - 2] = c;

    tables_ready = true;
}

/* Fisher-Yates: el AFH del BT no puede predecir el barrido */
static void shuffle(uint8_t *a, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = (int)(esp_random() % (uint32_t)(i + 1));
        uint8_t t = a[i]; a[i] = a[j]; a[j] = t;
    }
}

const char *jam_mode_name(jam_mode_t m)
{
    switch (m) {
        case JAM_ADV: return "BLE adv";
        case JAM_BLE: return "BLE full";
        case JAM_BT:  return "BT classic";
        default:      return "?";
    }
}

int jam_mode_count(jam_mode_t m)
{
    switch (m) {
        case JAM_ADV: return 3;
        case JAM_BLE: return 40;
        case JAM_BT:  return 79;
        default:      return 0;
    }
}

/* ================= instancia ================= */
struct jam_dev {
    char                name[8];
    bool                init;
    spi_host_device_t   host;
    spi_device_handle_t spi;
    gpio_num_t          ce, cs;
    SemaphoreHandle_t   lock;
    uint8_t            *tx, *rx;
    TaskHandle_t        task;
    volatile bool       active, running;
    volatile jam_mode_t mode;
    volatile uint32_t   dwell_us;
    int8_t              core;
    uint8_t             order[MAX_CH];   /* orden de barrido */
    int                 order_len;
};

/* ================= bajo nivel (SIN lock) ================= */
static void busy_us(uint32_t us)
{
    if (!us) return;
    int64_t end = esp_timer_get_time() + (int64_t)us;
    while (esp_timer_get_time() < end) { }
}

static esp_err_t xfer_nolock(jam_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    memcpy(d->tx, tx, len);
    memset(d->rx, 0, len);
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = d->tx,
        .rx_buffer = d->rx,
    };
    esp_err_t e = spi_device_polling_transmit(d->spi, &t);
    if (e == ESP_OK && rx) memcpy(rx, d->rx, len);
    return e;
}

static uint8_t rd_nolock(jam_dev_t *d, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x1F), C_NOP };
    uint8_t rx[2] = { 0 };
    xfer_nolock(d, tx, rx, 2);
    return rx[1];
}

static void wr_nolock(jam_dev_t *d, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(0x20 | (reg & 0x1F)), val };
    xfer_nolock(d, tx, NULL, 2);
}

/* ================= portadora continua ================= */
static void cw_start(jam_dev_t *d, uint8_t ch)
{
    if (!d->lock) return;

    gpio_set_level(d->ce, 0);                    /* CE abajo primero */

    xSemaphoreTake(d->lock, portMAX_DELAY);

    uint8_t c = rd_nolock(d, R_CONFIG);
    c |= CFG_PWR_UP;
    c &= (uint8_t)~CFG_PRIM_RX;                  /* PTX, no PRX */
    wr_nolock(d, R_CONFIG, c);

    uint8_t s = rd_nolock(d, R_RF_SETUP);
    s &= (uint8_t)~0x06u;
    s |= (uint8_t)(0x03u << 1);                  /* 0 dBm = max PA */
    wr_nolock(d, R_RF_SETUP, s);

    wr_nolock(d, R_RF_CH, (uint8_t)(ch & 0x7F));

    s = rd_nolock(d, R_RF_SETUP);
    s |= RF_CONT_WAVE | RF_PLL_LOCK;
    wr_nolock(d, R_RF_SETUP, s);

    xSemaphoreGive(d->lock);

    busy_us(SETTLE_US);
    gpio_set_level(d->ce, 1);                    /* recien ahora CE */
}

static void cw_hop(jam_dev_t *d, uint8_t ch)
{
    if (!d->lock) return;

    gpio_set_level(d->ce, 0);                    /* 1. CE abajo SIEMPRE */

    xSemaphoreTake(d->lock, portMAX_DELAY);
    wr_nolock(d, R_RF_CH, (uint8_t)(ch & 0x7F)); /* 2. canal */
    xSemaphoreGive(d->lock);

    busy_us(SETTLE_US);                          /* 3. PLL relock */
    gpio_set_level(d->ce, 1);                    /* 4. CE */
}

static void cw_stop(jam_dev_t *d)
{
    if (!d->lock) return;
    gpio_set_level(d->ce, 0);
    xSemaphoreTake(d->lock, portMAX_DELAY);
    uint8_t s = rd_nolock(d, R_RF_SETUP);
    s &= (uint8_t)~(RF_CONT_WAVE | RF_PLL_LOCK);
    wr_nolock(d, R_RF_SETUP, s);
    xSemaphoreGive(d->lock);
    busy_us(50);
}

/* ================= tarea ================= */
static void jam_task(void *arg)
{
    jam_dev_t *d = (jam_dev_t *)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        d->running = true;

        while (d->active && d->order_len > 0) {
            for (int i = 0; i < d->order_len; i++) {
                if (!d->active) break;
                if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;  /* cambio de config */

                cw_hop(d, d->order[i]);

                uint32_t dw = d->dwell_us;
                if (dw >= 1000) {
                    vTaskDelay(pdMS_TO_TICKS(dw / 1000));    /* bloquea */
                    busy_us(dw % 1000);
                } else {
                    busy_us(dw);
                }

                if ((i & 15) == 15) vTaskDelay(1);           /* cede CPU */
            }
        }

        cw_stop(d);
        d->running = false;
        ESP_LOGI(TAG, "[%s] detenido", d->name);
    }
}

/* ================= API ================= */
jam_dev_t *jam_init(const jam_cfg_t *cfg)
{
    if (!cfg) return NULL;
    if (!tables_ready) build_tables();

    jam_dev_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    strncpy(d->name, cfg->name ? cfg->name : "jam", sizeof(d->name) - 1);
    d->host = cfg->host;
    d->ce = cfg->ce;
    d->cs = cfg->cs;
    d->core = cfg->core;
    d->mode = JAM_BLE;
    d->dwell_us = 1000;
    uint32_t clk = cfg->clock_hz ? cfg->clock_hz : 10000000u;

    d->lock = xSemaphoreCreateMutex();
    d->tx = heap_caps_malloc(MAX_XFER, MALLOC_CAP_DMA);
    d->rx = heap_caps_malloc(MAX_XFER, MALLOC_CAP_DMA);
    if (!d->lock || !d->tx || !d->rx) goto fail;

    gpio_config_t ce_io = {
        .pin_bit_mask = (1ULL << (uint32_t)d->ce),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ce_io));
    gpio_set_level(d->ce, 0);

    spi_bus_config_t bus = {
        .mosi_io_num = cfg->mosi, .miso_io_num = cfg->miso, .sclk_io_num = cfg->sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = MAX_XFER,
    };
    esp_err_t err = spi_bus_initialize(d->host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "[%s] spi_bus: %s", d->name, esp_err_to_name(err));
        goto fail;
    }

    spi_device_interface_config_t dev = {
        .mode = 0, .clock_speed_hz = (int)clk, .spics_io_num = d->cs,
        .queue_size = 1, .cs_ena_pretrans = 2, .cs_ena_posttrans = 2,
    };
    err = spi_bus_add_device(d->host, &dev, &d->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] add_device: %s", d->name, esp_err_to_name(err));
        goto fail;
    }

    /* self-test */
    xSemaphoreTake(d->lock, portMAX_DELAY);
    wr_nolock(d, R_RF_CH, 0x5A);
    uint8_t back = rd_nolock(d, R_RF_CH);
    xSemaphoreGive(d->lock);

    if (back != 0x5A) {
        ESP_LOGE(TAG, "[%s] no responde (CE=%d CS=%d SCK=%d MOSI=%d MISO=%d)",
                 d->name, (int)cfg->ce, (int)cfg->cs, (int)cfg->sck,
                 (int)cfg->mosi, (int)cfg->miso);
        goto fail;
    }

    /* estado limpio */
    xSemaphoreTake(d->lock, portMAX_DELAY);
    wr_nolock(d, R_EN_AA, 0x00);
    wr_nolock(d, R_EN_RXADDR, 0x03);
    wr_nolock(d, R_SETUP_AW, 0x03);
    wr_nolock(d, R_SETUP_RETR, 0x00);
    wr_nolock(d, R_STATUS, 0x70);
    xfer_nolock(d, (uint8_t[]){ C_FLUSH_TX }, NULL, 1);
    xfer_nolock(d, (uint8_t[]){ C_FLUSH_RX }, NULL, 1);
    wr_nolock(d, R_CONFIG, CFG_PWR_UP);
    xSemaphoreGive(d->lock);

    d->init = true;

    if (xTaskCreatePinnedToCore(jam_task, "jam", 4096, d, 5, &d->task,
                                (d->core >= 0) ? (BaseType_t)d->core : tskNO_AFFINITY)
        != pdPASS) {
        ESP_LOGE(TAG, "[%s] tarea", d->name);
        goto fail;
    }

    ESP_LOGI(TAG, "[%s] listo SPI%d CE=%d CS=%d SCK=%d MOSI=%d MISO=%d",
             d->name, (int)d->host, (int)cfg->ce, (int)cfg->cs, (int)cfg->sck,
             (int)cfg->mosi, (int)cfg->miso);
    return d;

fail:
    if (d->spi) spi_bus_remove_device(d->spi);
    if (d->lock) vSemaphoreDelete(d->lock);
    free(d->tx); free(d->rx); free(d);
    return NULL;
}

esp_err_t jam_start(jam_dev_t *d, jam_mode_t mode, uint32_t dwell_us)
{
    if (!d || !d->init) return ESP_ERR_INVALID_STATE;
    if (mode < 0 || mode >= JAM_MODE_COUNT) return ESP_ERR_INVALID_ARG;

    d->mode = mode;
    d->dwell_us = (dwell_us < 200) ? 200 : dwell_us;

    /* copiar tabla + mezclar */
    int n = jam_mode_count(mode);
    const uint8_t *src = (mode == JAM_ADV) ? ch_adv
                      : (mode == JAM_BLE) ? ch_ble : ch_bt;
    memcpy(d->order, src, (size_t)n);
    if (mode != JAM_ADV) shuffle(d->order, n);   /* adv no necesita */
    d->order_len = n;

    cw_start(d, d->order[0]);
    d->active = true;
    xTaskNotifyGive(d->task);

    ESP_LOGI(TAG, "[%s] ON %s (%d canales) dwell=%luus",
             d->name, jam_mode_name(mode), n, (unsigned long)d->dwell_us);
    return ESP_OK;
}

esp_err_t jam_stop(jam_dev_t *d)
{
    if (!d || !d->init) return ESP_ERR_INVALID_STATE;
    if (!d->active) return ESP_OK;
    d->active = false;
    xTaskNotifyGive(d->task);
    for (int i = 0; i < 250 && d->running; i++) vTaskDelay(pdMS_TO_TICKS(2));
    cw_stop(d);
    return ESP_OK;
}

bool       jam_running(const jam_dev_t *d) { return d && d->active && d->running; }
jam_mode_t jam_get_mode(const jam_dev_t *d) { return d ? d->mode : JAM_BLE; }

void jam_set_dwell(jam_dev_t *d, uint32_t dwell_us)
{
    if (!d) return;
    d->dwell_us = (dwell_us < 200) ? 200 : dwell_us;
    if (d->active && d->task) xTaskNotifyGive(d->task);
}

uint8_t jam_channel(jam_dev_t *d)
{
    if (!d || !d->lock) return 0xFF;
    xSemaphoreTake(d->lock, portMAX_DELAY);
    uint8_t v = rd_nolock(d, R_RF_CH);
    xSemaphoreGive(d->lock);
    return (uint8_t)(v & 0x7F);
}
