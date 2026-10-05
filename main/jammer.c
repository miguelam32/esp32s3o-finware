/*
 * jammer.c - Driver nRF24L01+ + jammer de portadora continua.
 *
 * DOS REGLAS que hacen que esto funcione (y que antes fallaba):
 *
 * 1. LOCK DISCIPLINADO. Las funciones internas _rd()/_wr() NUNCA toman
 *    el lock; las publicas si. Asi no hay deadlock por mutex no-recursivo.
 *
 * 2. CE SIEMPRE ABAJO ANTES DE RF_CH. El bug clasico de la libreria RF24
 *    de Arduino: cambia el canal con el carrier activo y sin bajar CE.
 *    En modulos PA+LNA el PLL se desengancha y el carrier se congela
 *    tras el primer salto (nRF24/RF24#714). Aca: CE abajo -> RF_CH ->
 *    200 us -> CE arriba.
 *
 * 3. LA TAREA CEDE LA CPU. El dwell entre canales bloquea con vTaskDelay
 *    y cada 16 canales cede 1 tick. Sin eso el task watchdog de IDLE
 *    dispara y la consola se vuelve ilegible.
 */
#include "jammer.h"

#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "jam";

/* ================= registros nRF24L01+ ================= */
#define R_CONFIG      0x00
#define R_EN_AA       0x01
#define R_EN_RXADDR   0x02
#define R_SETUP_AW    0x03
#define R_SETUP_RETR  0x04
#define R_RF_CH       0x05
#define R_RF_SETUP    0x06
#define R_STATUS      0x07
#define R_OBSERVE_TX  0x08
#define R_RPD         0x09
#define R_RX_PW_P0    0x11
#define R_FIFO_STATUS 0x17
#define R_DYNPD       0x1C
#define R_FEATURE     0x1D

#define C_R_REGISTER   0x00
#define C_W_REGISTER   0x20
#define C_FLUSH_TX     0xE1
#define C_FLUSH_RX     0xE2
#define C_NOP          0xFF

#define CFG_PWR_UP     (1u << 1)
#define CFG_PRIM_RX    (1u << 0)
#define RF_CONT_WAVE   (1u << 7)
#define RF_PLL_LOCK    (1u << 4)
#define RF_DR_HIGH     (1u << 3)
#define ST_RX_DR       (1u << 6)
#define ST_TX_DS       (1u << 5)
#define ST_MAX_RT      (1u << 4)

#define MAX_XFER 33

/* ================= listas de canales ================= */
static const uint8_t ch_adv[]  = { 2, 26, 80 };

static const uint8_t ch_all[]  = {
     2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 80
};

static uint8_t ch_spray[79];       /* 2..80 */
static bool    s_spray_ready;

static void build_spray(void)
{
    for (uint8_t i = 0; i < 79; i++) ch_spray[i] = (uint8_t)(i + 2);
    s_spray_ready = true;
}

const char *jam_mode_name(jam_mode_t m)
{
    switch (m) {
        case JAM_BLE_ADV: return "BLE adv";
        case JAM_BLE_ALL: return "BLE all";
        case JAM_SPRAY:   return "Spray";
        default:          return "?";
    }
}

int jam_mode_count(jam_mode_t m)
{
    switch (m) {
        case JAM_BLE_ADV: return (int)sizeof(ch_adv);
        case JAM_BLE_ALL: return (int)sizeof(ch_all);
        case JAM_SPRAY:   return s_spray_ready ? (int)sizeof(ch_spray) : 79;
        default:          return 0;
    }
}

static const uint8_t *jam_channels(jam_mode_t m)
{
    switch (m) {
        case JAM_BLE_ADV: return ch_adv;
        case JAM_BLE_ALL: return ch_all;
        case JAM_SPRAY:   return ch_spray;
        default:          return ch_adv;
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
    uint8_t            *tx, *rx;          /* DMA-capable */
    TaskHandle_t        task;
    volatile bool       active, running;
    volatile jam_mode_t mode;
    volatile uint32_t   dwell_us;
    int8_t              core;
};

/* ================= utilidades ================= */
static void jam_delay_us(uint32_t us)
{
    if (!us) return;
    int64_t end = esp_timer_get_time() + (int64_t)us;
    while (esp_timer_get_time() < end) { /* spin */ }
}

/* ---- SPI SIN lock: el llamador debe tener el lock ---- */
static esp_err_t xfer_nolock(jam_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    memcpy(d->tx, tx, len);
    memset(d->rx, 0, len);

    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = d->tx,
        .rx_buffer = d->rx,
    };
    esp_err_t err = spi_device_polling_transmit(d->spi, &t);
    if (err == ESP_OK && rx) memcpy(rx, d->rx, len);
    return err;
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
/*
 * Receta del datasheet Nordic: PWR_UP=1, PRIM_RX=0, CONT_WAVE=1, CE=1.
 */
static void cw_start(jam_dev_t *d, uint8_t ch)
{
    if (!d->lock) return;
    xSemaphoreTake(d->lock, portMAX_DELAY);

    gpio_set_level(d->ce, 0);                       /* CE abajo primero */

    uint8_t c = rd_nolock(d, R_CONFIG);
    c |= CFG_PWR_UP;
    c &= (uint8_t)~CFG_PRIM_RX;
    wr_nolock(d, R_CONFIG, c);

    /* potencia: 0 dBm (RF_PWR = 11) */
    uint8_t s = rd_nolock(d, R_RF_SETUP);
    s &= (uint8_t)~0x06u;
    s |= (uint8_t)(0x03u << 1);
    wr_nolock(d, R_RF_SETUP, s);

    wr_nolock(d, R_RF_CH, (uint8_t)(ch & 0x7F));    /* canal nuevo */

    s = rd_nolock(d, R_RF_SETUP);
    s |= RF_CONT_WAVE | RF_PLL_LOCK;
    wr_nolock(d, R_RF_SETUP, s);

    xSemaphoreGive(d->lock);

    jam_delay_us(200);                              /* asentamiento del PLL */
    gpio_set_level(d->ce, 1);                       /* recien ahora CE */
}

static void cw_hop(jam_dev_t *d, uint8_t ch)
{
    if (!d->lock) return;

    gpio_set_level(d->ce, 0);                       /* 1. CE abajo SIEMPRE */

    xSemaphoreTake(d->lock, portMAX_DELAY);
    wr_nolock(d, R_RF_CH, (uint8_t)(ch & 0x7F));    /* 2. canal nuevo */
    xSemaphoreGive(d->lock);

    jam_delay_us(200);                              /* 3. PLL relock */
    gpio_set_level(d->ce, 1);                       /* 4. recien ahora CE */
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
    jam_delay_us(50);
}

/* ================= tarea del jammer ================= */
static void jam_task(void *arg)
{
    jam_dev_t *d = (jam_dev_t *)arg;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        d->running = true;

        const uint8_t *chs = jam_channels(d->mode);
        int n = jam_mode_count(d->mode);
        uint32_t yield_ctr = 0;

        while (d->active && n > 0) {
            for (int i = 0; i < n; i++) {
                if (!d->active) break;

                /* un cambio de config (modo/dwell) rompe la vuelta */
                if (ulTaskNotifyTake(pdTRUE, 0) > 0) break;

                cw_hop(d, chs[i]);

                uint32_t dw = d->dwell_us;
                if (dw >= 1000) {
                    vTaskDelay(pdMS_TO_TICKS(dw / 1000));   /* bloquea: watchdog OK */
                    jam_delay_us(dw % 1000);
                } else {
                    jam_delay_us(dw);
                }

                /* cede la CPU cada 16 canales: mantiene el duty cycle
                 * alto sin matar al scheduler */
                if (++yield_ctr >= 16) { vTaskDelay(1); yield_ctr = 0; }
            }
        }

        cw_stop(d);
        d->running = false;
        ESP_LOGI(TAG, "[%s] jammer detenido", d->name);
    }
}

/* ================= API publica ================= */
jam_dev_t *jam_init(const jam_cfg_t *cfg)
{
    if (!cfg) return NULL;

    if (!s_spray_ready) build_spray();

    jam_dev_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    strncpy(d->name, cfg->name ? cfg->name : "jam", sizeof(d->name) - 1);
    d->host = cfg->host;
    d->ce = cfg->ce;
    d->cs = cfg->cs;
    d->core = cfg->core;
    d->mode = JAM_BLE_ADV;
    d->dwell_us = 1500;
    uint32_t clk = cfg->clock_hz ? cfg->clock_hz : 8000000u;

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
    /* INVALID_STATE = el bus ya lo agarró otra radio: es correcto */
    esp_err_t err = spi_bus_initialize(d->host, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "[%s] spi_bus_initialize: %s", d->name, esp_err_to_name(err));
        goto fail;
    }

    spi_device_interface_config_t dev = {
        .mode = 0,                      /* modo 0 del nRF */
        .clock_speed_hz = (int)clk,
        .spics_io_num = d->cs,
        .queue_size = 1,
        .cs_ena_pretrans = 2, .cs_ena_posttrans = 2,
    };
    err = spi_bus_add_device(d->host, &dev, &d->spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[%s] spi_bus_add_device: %s", d->name, esp_err_to_name(err));
        goto fail;
    }

    /* self-test: escribir 0x5A en RF_CH y verificarlo */
    xSemaphoreTake(d->lock, portMAX_DELAY);
    wr_nolock(d, R_RF_CH, 0x5A);
    uint8_t back = rd_nolock(d, R_RF_CH);
    xSemaphoreGive(d->lock);

    if (back != 0x5A) {
        ESP_LOGE(TAG, "[%s] nRF24 no responde (CE=%d CS=%d SCK=%d MOSI=%d MISO=%d)",
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
    wr_nolock(d, R_STATUS, ST_RX_DR | ST_TX_DS | ST_MAX_RT);
    wr_nolock(d, R_FIFO_STATUS, 0x00);
    xfer_nolock(d, (uint8_t[]){ C_FLUSH_TX }, NULL, 1);
    xfer_nolock(d, (uint8_t[]){ C_FLUSH_RX }, NULL, 1);
    wr_nolock(d, R_CONFIG, CFG_PWR_UP);
    xSemaphoreGive(d->lock);

    d->init = true;

    if (xTaskCreatePinnedToCore(jam_task, "jam", 4096, d, 5, &d->task,
                                (d->core >= 0) ? (BaseType_t)d->core : tskNO_AFFINITY)
        != pdPASS) {
        ESP_LOGE(TAG, "[%s] no se pudo crear la tarea", d->name);
        goto fail;
    }

    ESP_LOGI(TAG, "[%s] listo host=SPI%d (CE=%d CS=%d SCK=%d MOSI=%d MISO=%d)",
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

    /* sube el carrier de inmediato, sin esperar al scheduler */
    cw_start(d, jam_channels(mode)[0]);

    d->active = true;
    xTaskNotifyGive(d->task);

    ESP_LOGI(TAG, "[%s] JAM ON modo=%s (%d canales) dwell=%luus",
             d->name, jam_mode_name(mode), jam_mode_count(mode),
             (unsigned long)d->dwell_us);
    return ESP_OK;
}

esp_err_t jam_stop(jam_dev_t *d)
{
    if (!d || !d->init) return ESP_ERR_INVALID_STATE;
    if (!d->active) return ESP_OK;

    d->active = false;
    xTaskNotifyGive(d->task);              /* saca la tarea de su espera */
    for (int i = 0; i < 250 && d->running; i++) vTaskDelay(pdMS_TO_TICKS(2));
    cw_stop(d);
    return ESP_OK;
}

bool       jam_running(const jam_dev_t *d) { return d && d->active && d->running; }
jam_mode_t jam_get_mode(const jam_dev_t *d) { return d ? d->mode : JAM_BLE_ADV; }

void jam_set_dwell(jam_dev_t *d, uint32_t dwell_us)
{
    if (!d) return;
    d->dwell_us = (dwell_us < 200) ? 200 : dwell_us;
    if (d->active && d->task) xTaskNotifyGive(d->task);
}

uint8_t jam_read_reg(jam_dev_t *d, uint8_t reg)
{
    if (!d || !d->lock) return 0xFF;
    xSemaphoreTake(d->lock, portMAX_DELAY);
    uint8_t v = rd_nolock(d, reg);
    xSemaphoreGive(d->lock);
    return v;
}

uint8_t jam_channel(jam_dev_t *d)
{
    return (uint8_t)(jam_read_reg(d, R_RF_CH) & 0x7F);
}

uint32_t jam_freq_hz(jam_dev_t *d)
{
    return (2400u + jam_channel(d)) * 1000000u;
}
