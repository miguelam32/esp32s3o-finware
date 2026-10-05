/*
 * nrf24.c - Driver nativo multi-instancia, ESP-IDF v6.0.x.
 */
#include "nrf24.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "nrf24";

struct nrf24_dev {
    char                name[8];
    bool                init;
    uint8_t             slot;
    spi_host_device_t   host;
    spi_device_handle_t spi;
    gpio_num_t          ce, cs, irq;
    bool                irq_enabled;
    SemaphoreHandle_t   lock;
    EventGroupHandle_t  evt;
    uint8_t            *tx, *rx;        /* DMA-capable */
    uint8_t             addr_width;
    uint8_t             payload_size;
    bool                cw;
};

static nrf24_dev_t *s_slots[NRF_MAX_DEV];
static bool         s_isr_service_up;

void nrf24_busy_delay_us(uint32_t us)
{
    if (!us) return;
    int64_t end = esp_timer_get_time() + (int64_t)us;
    while (esp_timer_get_time() < end) { /* spin */ }
}

static void IRAM_ATTR nrf24_isr(void *arg)
{
    nrf24_dev_t *d = (nrf24_dev_t *)arg;
    BaseType_t woken = pdFALSE;
    xEventGroupSetBitsFromISR(d->evt, NRF_EVT_RX | NRF_EVT_TX | NRF_EVT_MAX_RT, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* ---------- SPI sin lock (el llamador mantiene el lock) ---------- */
static esp_err_t xfer_nolock(nrf24_dev_t *d, const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (!d->init || !len || len > NRF_MAX_XFER) return ESP_ERR_INVALID_STATE;

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

static uint8_t rd(nrf24_dev_t *d, uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x1F), NRF_CMD_NOP };
    uint8_t rx[2] = { 0 };
    xfer_nolock(d, tx, rx, 2);
    return rx[1];
}

static void wr(nrf24_dev_t *d, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (uint8_t)(0x20 | (reg & 0x1F)), val };
    xfer_nolock(d, tx, NULL, 2);
}

static void rd_mb(nrf24_dev_t *d, uint8_t reg, uint8_t *val, size_t len)
{
    uint8_t tx[NRF_MAX_XFER] = { 0 }, rx[NRF_MAX_XFER] = { 0 };
    tx[0] = (uint8_t)(reg & 0x1F);
    for (size_t i = 1; i <= len; i++) tx[i] = NRF_CMD_NOP;
    xfer_nolock(d, tx, rx, len + 1);
    memcpy(val, rx + 1, len);
}

static void wr_mb(nrf24_dev_t *d, uint8_t reg, const uint8_t *val, size_t len)
{
    uint8_t tx[NRF_MAX_XFER] = { 0 };
    tx[0] = (uint8_t)(0x20 | (reg & 0x1F));
    memcpy(tx + 1, val, len);
    xfer_nolock(d, tx, NULL, len + 1);
}

void nrf24_lock(nrf24_dev_t *d)   { if (d->lock) xSemaphoreTake(d->lock, portMAX_DELAY); }
void nrf24_unlock(nrf24_dev_t *d) { if (d->lock) xSemaphoreGive(d->lock); }

/* ---------- ciclo de vida ---------- */
static esp_err_t setup_irq(nrf24_dev_t *d, gpio_num_t irq)
{
    if (irq == GPIO_NUM_NC) return ESP_OK;
    if (!s_isr_service_up) {
        esp_err_t e = gpio_install_isr_service(0);
        if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
        s_isr_service_up = true;
    }
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << (uint32_t)irq),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,     /* IRQ open-drain, activo bajo */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    esp_err_t e = gpio_isr_handler_add(irq, nrf24_isr, d);
    if (e != ESP_OK) return e;
    d->irq_enabled = true;
    return ESP_OK;
}

nrf24_dev_t *nrf24_create(const nrf24_cfg_t *cfg)
{
    if (!cfg) return NULL;

    int slot = -1;
    for (int i = 0; i < NRF_MAX_DEV; i++)
        if (!s_slots[i]) { slot = i; break; }
    if (slot < 0) { ESP_LOGE(TAG, "sin slots libres"); return NULL; }

    nrf24_dev_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;

    d->slot = (uint8_t)slot;
    d->host = cfg->host;
    d->ce = cfg->ce; d->cs = cfg->cs; d->irq = cfg->irq;
    uint32_t clk = cfg->clock_hz ? cfg->clock_hz : 8000000u;
    strncpy(d->name, cfg->name ? cfg->name : "nrf", sizeof(d->name) - 1);

    d->lock = xSemaphoreCreateMutex();
    d->evt  = xEventGroupCreate();
    d->tx   = heap_caps_malloc(NRF_MAX_XFER, MALLOC_CAP_DMA);
    d->rx   = heap_caps_malloc(NRF_MAX_XFER, MALLOC_CAP_DMA);
    if (!d->lock || !d->evt || !d->tx || !d->rx) goto fail;

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
        .max_transfer_sz = NRF_MAX_XFER,
    };
    /* Si el bus ya existe (lo comparte otra radio), INVALID_STATE es correcto */
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

    s_slots[slot] = d;
    d->init = true;

    /* validamos que el chip contesta antes de prometer nada */
    if (nrf24_self_test(d) != ESP_OK) {
        ESP_LOGE(TAG, "[%s] nRF24 no responde (revisa cableado y 3.3V)", d->name);
        nrf24_destroy(d);
        return NULL;
    }

    nrf24_power_up(d, false);
    nrf24_write_reg(d, NRF_EN_AA, 0x00);
    nrf24_write_reg(d, NRF_EN_RXADDR, 0x03);
    nrf24_write_reg(d, NRF_SETUP_AW, 0x03);
    nrf24_write_reg(d, NRF_SETUP_RETR, 0x00);
    nrf24_set_channel(d, 76);
    nrf24_set_data_rate(d, NRF24_DR_2M);
    nrf24_set_pa(d, NRF24_PA_0DBM);
    nrf24_set_crc(d, NRF24_CRC_16BIT);
    nrf24_set_payload_size(d, 32);
    nrf24_set_address_width(d, 5);
    nrf24_flush_rx(d);
    nrf24_flush_tx(d);
    nrf24_clear_irq(d, NRF_ST_RX_DR | NRF_ST_TX_DS | NRF_ST_MAX_RT);

    esp_err_t irq_err = setup_irq(d, d->irq);
    if (irq_err != ESP_OK) {
        ESP_LOGW(TAG, "[%s] IRQ no configurado (%s), modo polling",
                 d->name, esp_err_to_name(irq_err));
        d->irq_enabled = false;
    }

    ESP_LOGI(TAG, "[%s] listo host=SPI%d ch=%u (%lu Hz) IRQ=%s",
             d->name, (int)d->host, nrf24_channel(d),
             (unsigned long)nrf24_channel_to_freq(nrf24_channel(d)) * 1000UL,
             d->irq_enabled ? "si" : "no");
    return d;

fail:
    nrf24_destroy(d);
    return NULL;
}

void nrf24_destroy(nrf24_dev_t *d)
{
    if (!d) return;
    if (d->init) {
        nrf24_cw_stop(d);
        nrf24_set_ce(d, false);
        nrf24_power_up(d, false);
    }
    if (d->irq_enabled) { gpio_isr_handler_remove(d->irq); d->irq_enabled = false; }
    if (d->spi) { spi_bus_remove_device(d->spi); d->spi = NULL; }
    if (s_slots[d->slot] == d) s_slots[d->slot] = NULL;
    d->init = false;
    if (d->lock) { vSemaphoreDelete(d->lock); d->lock = NULL; }
    if (d->evt)  { vEventGroupDelete(d->evt); d->evt = NULL; }
    free(d->tx); free(d->rx); free(d);
}

bool        nrf24_is_up(const nrf24_dev_t *d) { return d && d->init; }
const char *nrf24_name(const nrf24_dev_t *d)   { return d ? d->name : "?"; }
uint8_t     nrf24_index(const nrf24_dev_t *d)  { return d ? d->slot : 0xFF; }

esp_err_t nrf24_self_test(nrf24_dev_t *d)
{
    if (!d || !d->init) return ESP_ERR_INVALID_STATE;
    const uint8_t magic = 0x5A;
    nrf24_lock(d);
    uint8_t old  = rd(d, NRF_RF_CH);
    wr(d, NRF_RF_CH, magic);
    uint8_t back = rd(d, NRF_RF_CH);
    wr(d, NRF_RF_CH, old);
    nrf24_unlock(d);
    return (back == magic) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

/* ---------- bajo nivel ---------- */
uint8_t nrf24_status(nrf24_dev_t *d)
{
    uint8_t tx[1] = { NRF_CMD_NOP }, rx[1] = { 0 };
    nrf24_lock(d);
    xfer_nolock(d, tx, rx, 1);
    nrf24_unlock(d);
    return rx[0];
}

uint8_t nrf24_read_reg(nrf24_dev_t *d, uint8_t reg)
{
    nrf24_lock(d);
    uint8_t v = rd(d, reg);
    nrf24_unlock(d);
    return v;
}

void nrf24_write_reg(nrf24_dev_t *d, uint8_t reg, uint8_t val)
{
    nrf24_lock(d);
    wr(d, reg, val);
    nrf24_unlock(d);
}

void nrf24_read_reg_mb(nrf24_dev_t *d, uint8_t reg, uint8_t *val, size_t len)
{
    nrf24_lock(d);
    rd_mb(d, reg, val, len);
    nrf24_unlock(d);
}

void nrf24_write_reg_mb(nrf24_dev_t *d, uint8_t reg, const uint8_t *val, size_t len)
{
    nrf24_lock(d);
    wr_mb(d, reg, val, len);
    nrf24_unlock(d);
}

uint8_t nrf24_cmd(nrf24_dev_t *d, uint8_t cmd)
{
    uint8_t tx[1] = { cmd }, rx[1] = { 0 };
    nrf24_lock(d);
    xfer_nolock(d, tx, rx, 1);
    nrf24_unlock(d);
    return rx[0];
}

void nrf24_flush_rx(nrf24_dev_t *d) { nrf24_cmd(d, NRF_CMD_FLUSH_RX); }
void nrf24_flush_tx(nrf24_dev_t *d) { nrf24_cmd(d, NRF_CMD_FLUSH_TX); }

void nrf24_clear_irq(nrf24_dev_t *d, uint8_t mask)
{
    nrf24_lock(d);
    wr(d, NRF_STATUS, mask);          /* escribir 1 borra el flag */
    nrf24_unlock(d);
}

/* ---------- configuracion ---------- */
void nrf24_set_ce(nrf24_dev_t *d, bool level) { gpio_set_level(d->ce, level ? 1 : 0); }

void nrf24_power_up(nrf24_dev_t *d, bool up)
{
    nrf24_lock(d);
    uint8_t c = rd(d, NRF_CONFIG);
    if (up) c |= NRF_CFG_PWR_UP; else c &= (uint8_t)~NRF_CFG_PWR_UP;
    wr(d, NRF_CONFIG, c);
    nrf24_unlock(d);
}

void nrf24_set_channel(nrf24_dev_t *d, uint8_t ch)
{
    nrf24_lock(d);
    wr(d, NRF_RF_CH, (uint8_t)(ch & 0x7F));
    nrf24_unlock(d);
}

uint8_t nrf24_channel(const nrf24_dev_t *d)
{
    return (uint8_t)(nrf24_read_reg((nrf24_dev_t *)d, NRF_RF_CH) & 0x7F);
}

uint32_t nrf24_channel_to_freq(uint8_t ch) { return 2400u + (uint32_t)(ch & 0x7F); }

void nrf24_set_data_rate(nrf24_dev_t *d, nrf24_dr_t dr)
{
    nrf24_lock(d);
    uint8_t s = rd(d, NRF_RF_SETUP);
    s &= (uint8_t)~(NRF_RF_RF_DR_LOW | NRF_RF_RF_DR_HIGH);
    if (dr == NRF24_DR_250K)    s |= NRF_RF_RF_DR_LOW;
    else if (dr == NRF24_DR_2M) s |= NRF_RF_RF_DR_HIGH;
    wr(d, NRF_RF_SETUP, s);
    nrf24_unlock(d);
}

void nrf24_set_pa(nrf24_dev_t *d, nrf24_pa_t pa)
{
    nrf24_lock(d);
    uint8_t s = rd(d, NRF_RF_SETUP);
    s &= (uint8_t)~0x06u;                              /* limpia RF_PWR 2:1 */
    s |= (uint8_t)(((uint8_t)pa & 0x03u) << 1);
    wr(d, NRF_RF_SETUP, s);
    nrf24_unlock(d);
}

void nrf24_set_address_width(nrf24_dev_t *d, uint8_t bytes)
{
    uint8_t aw = (bytes == 3) ? 0x01 : (bytes == 4) ? 0x02 : 0x03;
    nrf24_write_reg(d, NRF_SETUP_AW, aw);
}

void nrf24_set_payload_size(nrf24_dev_t *d, uint8_t size)
{
    d->payload_size = size;
    nrf24_lock(d);
    for (uint8_t p = 0; p < 6; p++) wr(d, (uint8_t)(NRF_RX_PW_P0 + p), size);
    nrf24_unlock(d);
}

void nrf24_set_auto_ack(nrf24_dev_t *d, bool en)
{
    nrf24_write_reg(d, NRF_EN_AA, en ? 0x3F : 0x00);
}

void nrf24_set_crc(nrf24_dev_t *d, nrf24_crc_t crc)
{
    nrf24_lock(d);
    uint8_t c = rd(d, NRF_CONFIG);
    c &= (uint8_t)~(NRF_CFG_EN_CRC | NRF_CFG_CRCO);
    if (crc == NRF24_CRC_8BIT)       c |= NRF_CFG_EN_CRC;
    else if (crc == NRF24_CRC_16BIT) c |= NRF_CFG_EN_CRC | NRF_CFG_CRCO;
    wr(d, NRF_CONFIG, c);
    nrf24_unlock(d);
}

void nrf24_set_retr(nrf24_dev_t *d, uint16_t delay_us, uint8_t count)
{
    uint8_t ard = (uint8_t)((delay_us / 250) - 1);
    if (ard > 15) ard = 15;
    if (count > 15) count = 15;
    nrf24_write_reg(d, NRF_SETUP_RETR, (uint8_t)((ard << 4) | count));
}

void nrf24_enable_dynamic_payload(nrf24_dev_t *d, bool en)
{
    nrf24_lock(d);
    if (en) {
        wr(d, NRF_FEATURE, rd(d, NRF_FEATURE) | 0x07u);
        wr(d, NRF_DYNPD, 0x3F);
    } else {
        wr(d, NRF_FEATURE, rd(d, NRF_FEATURE) & (uint8_t)~0x04u);
        wr(d, NRF_DYNPD, 0x00);
    }
    nrf24_unlock(d);
}

void nrf24_open_writing_pipe(nrf24_dev_t *d, const uint8_t *addr)
{
    nrf24_write_reg_mb(d, NRF_TX_ADDR, addr, d->addr_width);
}

void nrf24_open_reading_pipe(nrf24_dev_t *d, uint8_t pipe, const uint8_t *addr)
{
    if (pipe > 5) return;
    nrf24_lock(d);
    if (pipe < 2) wr_mb(d, (uint8_t)(NRF_RX_ADDR_P0 + pipe), addr, d->addr_width);
    else          wr(d, (uint8_t)(NRF_RX_ADDR_P0 + pipe), addr[0]);
    uint8_t er = rd(d, NRF_EN_RXADDR);
    er |= (uint8_t)(1u << pipe);
    wr(d, NRF_EN_RXADDR, er);
    nrf24_unlock(d);
}

/* ---------- RX / TX ---------- */
void nrf24_start_listening(nrf24_dev_t *d)
{
    nrf24_lock(d);
    uint8_t c = rd(d, NRF_CONFIG);
    c |= NRF_CFG_PWR_UP | NRF_CFG_PRIM_RX;
    wr(d, NRF_CONFIG, c);
    nrf24_unlock(d);
    nrf24_set_ce(d, true);
}

void nrf24_stop_listening(nrf24_dev_t *d)
{
    nrf24_set_ce(d, false);
    nrf24_busy_delay_us(200);
    nrf24_lock(d);
    uint8_t c = rd(d, NRF_CONFIG);
    c &= (uint8_t)~NRF_CFG_PRIM_RX;
    wr(d, NRF_CONFIG, c);
    nrf24_unlock(d);
}

bool nrf24_available(nrf24_dev_t *d)
{
    return (nrf24_read_reg(d, NRF_FIFO_STATUS) & NRF_FIFO_RX_EMPTY) == 0;
}

bool nrf24_carrier_detect(nrf24_dev_t *d)
{
    return (nrf24_read_reg(d, NRF_RPD) & 0x01u) != 0;
}

bool nrf24_write(nrf24_dev_t *d, const uint8_t *buf, size_t len, bool no_ack)
{
    nrf24_lock(d);
    nrf24_clear_irq(d, NRF_ST_TX_DS | NRF_ST_MAX_RT);

    uint8_t c = rd(d, NRF_CONFIG);
    c |= NRF_CFG_PWR_UP;
    c &= (uint8_t)~NRF_CFG_PRIM_RX;
    wr(d, NRF_CONFIG, c);

    uint8_t tx[NRF_MAX_XFER] = { 0 };
    tx[0] = no_ack ? NRF_CMD_W_TX_PAYLOAD_NOACK : NRF_CMD_W_TX_PAYLOAD;
    memcpy(tx + 1, buf, len);
    xfer_nolock(d, tx, NULL, len + 1);

    nrf24_set_ce(d, true);
    nrf24_busy_delay_us(20);              /* datasheet: CE alto >= 10 us */
    nrf24_set_ce(d, false);

    int64_t deadline = esp_timer_get_time() + 3000;
    bool ok = false;
    for (;;) {
        uint8_t st = rd(d, NRF_STATUS);
        if (st & (NRF_ST_TX_DS | NRF_ST_MAX_RT)) {
            wr(d, NRF_STATUS, st);
            ok = (st & NRF_ST_TX_DS) != 0;
            break;
        }
        if (esp_timer_get_time() > deadline) {
            uint8_t fcmd = NRF_CMD_FLUSH_TX;
            xfer_nolock(d, &fcmd, NULL, 1);
            break;
        }
        nrf24_busy_delay_us(10);
    }
    nrf24_unlock(d);
    return ok;
}

bool nrf24_wait_rx(nrf24_dev_t *d, uint32_t timeout_ms)
{
    if (!d || !d->evt) return false;
    EventBits_t b = xEventGroupWaitBits(d->evt, NRF_EVT_RX, pdTRUE, pdFALSE,
                                        pdMS_TO_TICKS(timeout_ms));
    return (b & NRF_EVT_RX) != 0;
}

/* ---------- portadora continua ----------
 * Receta del datasheet Nordic: PWR_UP=1, PRIM_RX=0, CONT_WAVE=1, CE=1.
 *
 * El bug clasico de la libreria RF24 de Arduino: cambia RF_CH con el
 * carrier activo y sin bajar CE. En modulos PA+LNA el PLL se desengancha
 * y el carrier se congela o muere tras el primer salto (nRF24/RF24#714).
 * Aca: CE abajo -> RF_CH -> 200 us -> CE arriba.
 */
void nrf24_cw_start(nrf24_dev_t *d, uint8_t ch, nrf24_pa_t pa)
{
    nrf24_lock(d);
    nrf24_set_ce(d, false);

    uint8_t c = rd(d, NRF_CONFIG);
    c |= NRF_CFG_PWR_UP;
    c &= (uint8_t)~NRF_CFG_PRIM_RX;
    wr(d, NRF_CONFIG, c);

    {
        uint8_t s = rd(d, NRF_RF_SETUP);
        s &= (uint8_t)~0x06u;
        s |= (uint8_t)(((uint8_t)pa & 0x03u) << 1);
        wr(d, NRF_RF_SETUP, s);
    }
    wr(d, NRF_RF_CH, (uint8_t)(ch & 0x7F));

    uint8_t s = rd(d, NRF_RF_SETUP);
    s |= NRF_RF_CONT_WAVE | NRF_RF_PLL_LOCK;
    wr(d, NRF_RF_SETUP, s);
    d->cw = true;
    nrf24_unlock(d);

    nrf24_busy_delay_us(200);          /* asentamiento del PLL */
    nrf24_set_ce(d, true);
}

void nrf24_cw_hop(nrf24_dev_t *d, uint8_t ch)
{
    if (!d->cw) { nrf24_cw_start(d, ch, NRF24_PA_0DBM); return; }
    nrf24_lock(d);
    nrf24_set_ce(d, false);                        /* 1. CE abajo siempre */
    wr(d, NRF_RF_CH, (uint8_t)(ch & 0x7F));        /* 2. canal nuevo      */
    nrf24_unlock(d);
    nrf24_busy_delay_us(200);                      /* 3. PLL relock       */
    nrf24_set_ce(d, true);                         /* 4. recien ahora CE  */
}

void nrf24_cw_stop(nrf24_dev_t *d)
{
    if (!d || !d->cw) return;
    nrf24_set_ce(d, false);
    nrf24_lock(d);
    uint8_t s = rd(d, NRF_RF_SETUP);
    s &= (uint8_t)~(NRF_RF_CONT_WAVE | NRF_RF_PLL_LOCK);
    wr(d, NRF_RF_SETUP, s);
    d->cw = false;
    nrf24_unlock(d);
    nrf24_busy_delay_us(50);
}

bool nrf24_cw_active(const nrf24_dev_t *d) { return d && d->cw; }

/* ---------- debug ---------- */
void nrf24_dump(const nrf24_dev_t *d)
{
    nrf24_dev_t *w = (nrf24_dev_t *)d;
    ESP_LOGI(TAG, "[%s] CONFIG=0x%02X EN_AA=0x%02X EN_RXADDR=0x%02X SETUP_AW=0x%02X",
             d->name, nrf24_read_reg(w, NRF_CONFIG), nrf24_read_reg(w, NRF_EN_AA),
             nrf24_read_reg(w, NRF_EN_RXADDR), nrf24_read_reg(w, NRF_SETUP_AW));
    ESP_LOGI(TAG, "[%s] SETUP_RETR=0x%02X RF_CH=%u (%lu Hz) RF_SETUP=0x%02X",
             d->name, nrf24_read_reg(w, NRF_SETUP_RETR), nrf24_channel(d),
             (unsigned long)nrf24_channel_to_freq(nrf24_channel(d)) * 1000UL,
             nrf24_read_reg(w, NRF_RF_SETUP));
    ESP_LOGI(TAG, "[%s] STATUS=0x%02X OBSERVE_TX=0x%02X RPD=%u FIFO=0x%02X CW=%d",
             d->name, nrf24_status(w), nrf24_read_reg(w, NRF_OBSERVE_TX),
             nrf24_carrier_detect(w) ? 1 : 0, nrf24_read_reg(w, NRF_FIFO_STATUS),
             nrf24_cw_active(d) ? 1 : 0);
}
