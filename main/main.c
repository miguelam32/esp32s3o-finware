/*
 * main.c
 * ESP32-S3 N16R8 (ESP-IDF v6.0.3) — Hopping TX con 2x nRF24L01
 *                       + JAMMER de portadora continua
 *
 *   NRF2 -> SPI2 (bus propio)   canales   0..62   tarea en núcleo 0
 *   NRF3 -> SPI3 (bus propio)   canales  63..125  tarea en núcleo 1
 *   (NRF1 está muerto: no se usa)
 *
 * Dos modos de operación, conmutables por consola:
 *   HOP  (default al flashear, "jam off") : RF_CH -> payload -> pulso de CE
 *   JAM  ("jam on", arranca activo)       : RF_CH -> CE alto sostenido (carrier)
 *
 * El jammer NO toca la lógica de hop: reutiliza la misma hop_table,
 * el mismo gptimer y las mismas tareas. Solo cambia qué se ejecuta
 * dentro del tick.
 *
 * Comandos por el monitor serie (115200):
 *   jam [on|off]   portadora continua en ambos NRF
 *   full           barre TODOS los canales del bloque (ignora blacklist)
 *   help | ?       ayuda
 *
 * LED WS2812 (GPIO48):
 *   ARCOÍRIS : modo HOP, los 2 NRF transmitiendo
 *   VERDE    : modo JAM activo
 *   ROJO     : falta algún NRF
 *   ÁMBAR    : presentes pero sin tráfico
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "hal/gpio_ll.h"
#include "led_strip.h"

static const char *TAG = "NRF24_2TX";

// ---------- LED ----------
#define LED_GPIO        GPIO_NUM_48     // algunas DevKitC v1.1 lo traen en GPIO38
#define LED_PERIOD_MS   200             // 5 Hz
#define LED_BRILLO      50              // 0-255, tenue para no encandilar
#define LED_HUE_STEP    20              // 360/20 = 18 pasos por vuelta de arcoíris
static led_strip_handle_t led;

// ---------- Buses ----------
#define PIN_SCK2   GPIO_NUM_12
#define PIN_MOSI2  GPIO_NUM_11
#define PIN_MISO2  GPIO_NUM_13
#define PIN_SCK3   GPIO_NUM_18
#define PIN_MOSI3  GPIO_NUM_8
#define PIN_MISO3  GPIO_NUM_7
#define SPI_HZ     (10 * 1000 * 1000)   // máximo del nRF24L01(+). Si el chequeo de presencia falla, prueba 8 MHz

// ---------- nRF24L01 ----------
#define NRF_CONFIG      0x00
#define NRF_EN_AA       0x01
#define NRF_EN_RXADDR   0x02
#define NRF_SETUP_AW    0x03
#define NRF_RF_CH       0x05
#define NRF_RF_SETUP    0x06
#define NRF_STATUS      0x07
#define NRF_CD_RPD      0x09
#define NRF_TX_ADDR     0x10
#define NRF_FEATURE     0x1D

#define CMD_R_REGISTER          0x00
#define CMD_W_REGISTER          0x20
#define CMD_W_TX_PAYLOAD_NOACK  0xB0
#define CMD_FLUSH_TX            0xE1

#define CONFIG_MODO_TX  0x0E   // PWR_UP, TX, CRC 2 bytes
#define CONFIG_MODO_RX  0x0F   // PWR_UP, RX, CRC 2 bytes (solo para escanear)
#define RF_SETUP_VALOR  0x0E   // 2 Mbps, potencia máxima

// ---------- Hopping ----------
#define NUM_RADIOS            2
#define CHANNELS_PER_BLOQUE   63      // 126 canales / 2 radios
#define MIN_CANALES_LIBRES    16      // si el escaneo deja menos, se usa el bloque completo
#define DWELL_US              170     // ciclo del chip ~300us (130 settling + ~165 aire) => ~150us de margen
#define PAYLOAD_LEN           32      // máximo del nRF24
#define REESCANEO_MS          5000
#define STATUS_MS             2000

// ---------- Jammer (portadora continua) ----------
#define RF_CONT_WAVE_BIT  0x80        // bit 7 de RF_SETUP
#define RF_PLL_LOCK_BIT   0x10        // bit 4 de RF_SETUP

// Rango que de verdad usa BT/BLE: 2402-2480 MHz = canales nRF 2..80.
// Fuera de ahi es desperdicio. El radio 3 barria 63..125 (2463..2525 MHz):
// 45 de esos 63 canales no existen para BT ni BLE.
#define JAM_CH_LO   2
#define JAM_CH_HI   80
#define JAM_CH_N    (JAM_CH_HI - JAM_CH_LO + 1)

typedef struct {
    const char *nombre;
    uint8_t numero;                 // 2 o 3: define la dirección (0xE0 + numero)
    spi_host_device_t host;
    gpio_num_t csn_pin, ce_pin, irq_pin;
    uint8_t base_channel;
    uint32_t seed;
    int core;

    spi_device_handle_t spi;
    TaskHandle_t task;
    bool presente;

    uint8_t libres[CHANNELS_PER_BLOQUE];     // canales sin ocupar, en orden
    uint8_t hop_table[CHANNELS_PER_BLOQUE];  // permutación de esta vuelta
    uint8_t hop_count;
    uint8_t blacklist[CHANNELS_PER_BLOQUE];  // 1 = ocupado (WiFi, etc.)
    uint32_t hop_index;
    uint32_t vueltas;

    volatile uint32_t packet_counter;
    volatile uint8_t canal_actual;
    volatile uint32_t perdidos;              // dwells que la tarea no alcanzó a atender
    volatile uint32_t flushes;               // veces que el FIFO TX quedó trabado

    uint8_t payload[PAYLOAD_LEN];
    uint8_t txb[36] __attribute__((aligned(4)));   // 1 cmd + 32 payload, alineado para SPI
    uint8_t rxb[36] __attribute__((aligned(4)));
} radio_t;

static radio_t radios[NUM_RADIOS] = {
    { .nombre = "NRF2", .numero = 2, .host = SPI2_HOST,
      .csn_pin = GPIO_NUM_9,  .ce_pin = GPIO_NUM_5, .irq_pin = GPIO_NUM_16,
      .base_channel = 0 * CHANNELS_PER_BLOQUE, .seed = 0xA17E56u, .core = 0 },
    { .nombre = "NRF3", .numero = 3, .host = SPI3_HOST,
      .csn_pin = GPIO_NUM_21, .ce_pin = GPIO_NUM_1, .irq_pin = GPIO_NUM_2,
      .base_channel = 1 * CHANNELS_PER_BLOQUE, .seed = 0xA17E57u, .core = 1 },
};

#define EV_PAUSED(i)   (1u << (i))
#define EV_PAUSED_ALL  ((1u << NUM_RADIOS) - 1)
#define EV_RESUME      (1u << 4)

static EventGroupHandle_t eg;
static volatile bool g_pausa = false;
static gptimer_handle_t dwell_timer;

// Estado del jammer
static volatile bool g_jammer   = true;    // arranca en modo jammer
static volatile bool g_jam_full = true;   // true = ignora blacklist, barre todo el bloque

// ---------- SPI de bajo nivel ----------
// Transacciones de <=4 bytes usan TXDATA/RXDATA: sin buffers ni DMA, lo más rápido.
static void nrf_write_reg(spi_device_handle_t dev, uint8_t reg, uint8_t value) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 16 };
    t.tx_data[0] = CMD_W_REGISTER | reg;
    t.tx_data[1] = value;
    spi_device_polling_transmit(dev, &t);
}

static uint8_t nrf_read_reg(spi_device_handle_t dev, uint8_t reg) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA, .length = 16 };
    t.tx_data[0] = CMD_R_REGISTER | reg;
    t.tx_data[1] = 0xFF;
    spi_device_polling_transmit(dev, &t);
    return t.rx_data[1];
}

static void nrf_send_cmd(spi_device_handle_t dev, uint8_t cmd) {
    spi_transaction_t t = { .flags = SPI_TRANS_USE_TXDATA, .length = 8 };
    t.tx_data[0] = cmd;
    spi_device_polling_transmit(dev, &t);
}

static void nrf_write_addr(spi_device_handle_t dev, const uint8_t addr[5]) {
    uint8_t tx[8] __attribute__((aligned(4))) = { CMD_W_REGISTER | NRF_TX_ADDR };
    memcpy(&tx[1], addr, 5);
    spi_transaction_t t = { .length = 6 * 8, .tx_buffer = tx };
    spi_device_polling_transmit(dev, &t);
}

// Devuelve el byte STATUS (el nRF lo saca por MISO mientras recibe el comando).
static uint8_t nrf_write_payload_noack(radio_t *r) {
    r->txb[0] = CMD_W_TX_PAYLOAD_NOACK;
    memcpy(&r->txb[1], r->payload, PAYLOAD_LEN);
    spi_transaction_t t = {
        .length = (1 + PAYLOAD_LEN) * 8,
        .tx_buffer = r->txb,
        .rx_buffer = r->rxb,
    };
    spi_device_polling_transmit(r->spi, &t);
    return r->rxb[0];
}

// ---------- Tabla de saltos ----------
static uint32_t xorshift32(uint32_t *s) {
    *s ^= *s << 13;
    *s ^= *s >> 17;
    *s ^= *s << 5;
    return *s;
}

// Baraja los canales libres de forma DETERMINISTA: depende solo de
// (seed, número de vuelta, conjunto de libres). Así un receptor con el mismo
// seed y la misma lista puede reproducir el orden exacto. El número de vuelta
// va dentro del payload para que el RX se resincronice.
static void barajar(radio_t *r) {
    memcpy(r->hop_table, r->libres, r->hop_count);
    uint32_t s = r->seed ^ (r->vueltas * 0x9E3779B1u);
    if (s == 0) s = 1;
    for (int i = r->hop_count - 1; i > 0; i--) {
        int j = xorshift32(&s) % (i + 1);
        uint8_t tmp = r->hop_table[i];
        r->hop_table[i] = r->hop_table[j];
        r->hop_table[j] = tmp;
    }
}

static void generar_hop_table(radio_t *r) {
    int n = 0;
    for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) {
        if (!r->blacklist[i]) r->libres[n++] = r->base_channel + i;
    }
    if (n < MIN_CANALES_LIBRES) {   // casi todo "sucio": mejor el bloque completo que saltar entre pocos
        for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) r->libres[i] = r->base_channel + i;
        n = CHANNELS_PER_BLOQUE;
    }
    r->hop_count = (uint8_t)n;
    r->hop_index = 0;
    r->vueltas = 0;
    barajar(r);
}

// ---------- Escaneo de canales (detector de portadora del chip) ----------
static bool canal_ocupado(radio_t *r, uint8_t canal) {
    nrf_write_reg(r->spi, NRF_RF_CH, canal);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_RX);
    gpio_set_level(r->ce_pin, 1);
    ets_delay_us(200);
    uint8_t cd = nrf_read_reg(r->spi, NRF_CD_RPD) & 0x01;
    gpio_set_level(r->ce_pin, 0);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    return cd != 0;
}

static void escanear_bloque(radio_t *r) {
    int sucios = 0;
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x01);   // pipe 0 activo solo mientras escanea
    for (int i = 0; i < CHANNELS_PER_BLOQUE; i++) {
        bool ocupado = canal_ocupado(r, r->base_channel + i);
        r->blacklist[i] = ocupado ? 1 : 0;
        if (ocupado) sucios++;
    }
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x00);
    ESP_LOGI(TAG, "%s scan: %d/%d canales sucios", r->nombre, sucios, CHANNELS_PER_BLOQUE);
}

// ---------- Init ----------
static void bus_init(spi_host_device_t host, gpio_num_t sck, gpio_num_t mosi, gpio_num_t miso) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = mosi, .miso_io_num = miso, .sclk_io_num = sck,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = 64,      // el comando de payload mide 33 bytes: con 32 fallaría
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
}

static void radio_init(radio_t *r) {
    gpio_set_direction(r->ce_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(r->ce_pin, 0);
    gpio_set_direction(r->irq_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(r->irq_pin, GPIO_PULLUP_ONLY);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_HZ,
        .mode = 0,
        .spics_io_num = r->csn_pin,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(r->host, &devcfg, &r->spi));

    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);        // sin auto-ACK
    nrf_write_reg(r->spi, NRF_EN_RXADDR, 0x00);
    nrf_write_reg(r->spi, NRF_SETUP_AW, 0x03);     // direcciones de 5 bytes
    nrf_write_reg(r->spi, NRF_RF_SETUP, RF_SETUP_VALOR);
    nrf_write_reg(r->spi, NRF_FEATURE, 0x01);      // EN_DYN_ACK: habilita W_TX_PAYLOAD_NOACK
    nrf_write_reg(r->spi, NRF_STATUS, 0x70);       // limpia flags
    nrf_send_cmd(r->spi, CMD_FLUSH_TX);

    uint8_t addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, (uint8_t)(0xE0 + r->numero) };
    nrf_write_addr(r->spi, addr);

    // Relleno fijo del payload: lo único que cambia por salto son los primeros 9 bytes.
    for (int i = 0; i < PAYLOAD_LEN; i++) r->payload[i] = (uint8_t)(0xA5 ^ i);
}

static bool nrf_presente(radio_t *r) {
    uint8_t cfg = nrf_read_reg(r->spi, NRF_CONFIG);
    uint8_t rf  = nrf_read_reg(r->spi, NRF_RF_SETUP);
    r->presente = (cfg == CONFIG_MODO_TX) && (rf == RF_SETUP_VALOR);
    if (r->presente) {
        ESP_LOGI(TAG, "%s OK  [SPI%d CSN%d CE%d] CONFIG=0x%02X RF_SETUP=0x%02X",
                 r->nombre, r->host == SPI2_HOST ? 2 : 3, (int)r->csn_pin, (int)r->ce_pin, cfg, rf);
    } else {
        ESP_LOGE(TAG, "%s NO RESPONDE [SPI%d CSN%d CE%d] CONFIG=0x%02X RF_SETUP=0x%02X",
                 r->nombre, r->host == SPI2_HOST ? 2 : 3, (int)r->csn_pin, (int)r->ce_pin, cfg, rf);
        ESP_LOGE(TAG, "%s -> revisa MISO, CSN, 3.3V/GND y capacitor. Si todo está bien, baja SPI_HZ a 8 MHz.", r->nombre);
    }
    return r->presente;
}

// ============================================================================
//  JAMMER — portadora continua
// ============================================================================
// Reutiliza hop_table, gptimer y las tareas: solo cambia qué se hace en el
// tick. Nada de la lógica de hop se toca.
//
// Regla del PLL (el bug clásico): hay que bajar CE ANTES de escribir RF_CH y
// subirlo DESPUÉS. Si cambiás el canal con el carrier activo y CE alto, el
// PLL se desengancha y el carrier se congela tras el primer salto.
// ----------------------------------------------------------------------------

// ¿Está sucio este canal según el escaneo de CUALQUIERA de los dos radios?
// Cada radio solo escanea su mitad, así que hay que consultar los dos.
static bool jam_ch_sucio(int canal) {
    for (int i = 0; i < NUM_RADIOS; i++) {
        int idx = canal - radios[i].base_channel;
        if (idx >= 0 && idx < CHANNELS_PER_BLOQUE && radios[i].blacklist[idx]) return true;
    }
    return false;
}

// Tabla del jammer: SOLO el rango útil 2..80, partido entre los dos radios.
//   radio 0 (NRF2) -> 2..40       radio 1 (NRF3) -> 41..80
// respetar_blacklist = true  -> salta los canales WiFi
static void generar_jam_table(radio_t *r, bool respetar_blacklist) {
    int idx = (int)(r - radios);
    int por_radio = JAM_CH_N / NUM_RADIOS;              // 39
    int lo = JAM_CH_LO + idx * por_radio;
    int hi = lo + por_radio - 1;
    if (idx == NUM_RADIOS - 1) hi = JAM_CH_HI;          // el último se lleva el resto

    int n = 0;
    for (int c = lo; c <= hi; c++) {
        if (respetar_blacklist && jam_ch_sucio(c)) continue;
        r->libres[n++] = (uint8_t)c;
    }
    if (n < 8) {          // casi todo sucio: mejor el rango completo que 4 canales
        n = 0;
        for (int c = lo; c <= hi; c++) r->libres[n++] = (uint8_t)c;
    }
    r->hop_count = (uint8_t)n;
    r->hop_index = 0;
    r->vueltas = 0;
    barajar(r);
}

// CE por registro directo: gpio_set_level() cuesta 2-5us de overhead, y a
// 170us de dwell eso es ~2% del tiempo de aire tirado a la basura.
#define OUT_W1TS   (*(volatile uint32_t *)0x3F4040008u)
#define OUT_W1TC   (*(volatile uint32_t *)0x3F404000Cu)

static inline void ce_hi(radio_t *r) {
    OUT_W1TS = (1u << (uint32_t)r->ce_pin);
}
static inline void ce_lo(radio_t *r) {
    OUT_W1TC = (1u << (uint32_t)r->ce_pin);
}

// Un salto en modo jammer: cambia de canal manteniendo el carrier arriba.
static inline void jam_hop(radio_t *r) {
    r->hop_index++;
    if (r->hop_index >= r->hop_count) {
        r->hop_index = 0;
        r->vueltas++;
        barajar(r);                     // nuevo orden cada vuelta
    }
    uint8_t canal = r->hop_table[r->hop_index];
    r->canal_actual = canal;

    ce_lo(r);                                           // 1. CE abajo
    nrf_write_reg(r->spi, NRF_RF_CH, canal);            // 2. canal nuevo
    ce_hi(r);                                           // 3. carrier en el canal nuevo

    r->packet_counter++;
}

// Entra en modo carrier: CONT_WAVE + PLL_LOCK, CE alto sostenido.
static void jam_enter(radio_t *r) {
    if (!r->presente) return;

    gpio_set_level(r->ce_pin, 0);

    nrf_send_cmd(r->spi, CMD_FLUSH_TX);
    nrf_write_reg(r->spi, NRF_EN_AA, 0x00);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);            // PTX
    nrf_write_reg(r->spi, NRF_RF_SETUP,
                  (uint8_t)(RF_SETUP_VALOR | RF_CONT_WAVE_BIT | RF_PLL_LOCK_BIT));

    // Tabla del jammer: rango útil 2..80 partido entre los dos radios.
    // g_jam_full = true  -> ignora blacklist, barre los 79 canales útiles
    // g_jam_full = false -> salta los canales WiFi (donde el BT con AFH no vive)
    generar_jam_table(r, !g_jam_full);

    r->canal_actual = r->hop_table[0];
    nrf_write_reg(r->spi, NRF_RF_CH, r->canal_actual);
    gpio_set_level(r->ce_pin, 1);                                 // carrier arriba
}

// Vuelve a modo hopping TX.
static void jam_exit(radio_t *r) {
    if (!r->presente) return;

    gpio_set_level(r->ce_pin, 0);
    nrf_write_reg(r->spi, NRF_RF_SETUP, RF_SETUP_VALOR);          // fuera CONT_WAVE
    nrf_send_cmd(r->spi, CMD_FLUSH_TX);
    nrf_write_reg(r->spi, NRF_CONFIG, CONFIG_MODO_TX);
    generar_hop_table(r);
}

// Cambia el modo en ambos radios.
static void jam_set(bool on) {
    if (on == g_jammer) return;
    g_jammer = on;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (!radios[i].presente) continue;
        if (on) jam_enter(&radios[i]);
        else    jam_exit(&radios[i]);
    }
    ESP_LOGW(TAG, ">>> JAMMER %s%s", on ? "ON" : "OFF",
             (on && g_jam_full) ? " (barrido completo)" : "");
}

// Fuerza la reconstrucción de las tablas sin cambiar de modo (para "full").
static void jam_refresh(void) {
    if (!g_jammer) return;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (radios[i].presente) jam_enter(&radios[i]);
    }
}

// ============================================================================
//  HOPPING TX (intacto)
// ============================================================================

// Un salto: RF_CH -> payload -> pulso de CE
static inline void hacer_hop(radio_t *r) {
    r->hop_index++;
    if (r->hop_index >= r->hop_count) {
        r->hop_index = 0;
        r->vueltas++;
        barajar(r);
    }
    uint8_t canal = r->hop_table[r->hop_index];
    r->canal_actual = canal;
    nrf_write_reg(r->spi, NRF_RF_CH, canal);

    uint32_t c = r->packet_counter;
    r->payload[0] = r->numero;
    r->payload[1] = (uint8_t)(r->vueltas & 0xFF);
    r->payload[2] = (uint8_t)((r->vueltas >> 8) & 0xFF);
    r->payload[3] = (uint8_t)r->hop_index;
    r->payload[4] = canal;
    memcpy(&r->payload[5], &c, 4);

    uint8_t st = nrf_write_payload_noack(r);
    if (st & 0x01) {                       // TX_FULL: el FIFO quedó trabado, lo limpiamos y reintentamos
        nrf_send_cmd(r->spi, CMD_FLUSH_TX);
        nrf_write_payload_noack(r);
        r->flushes++;
    }

    // Pulso de CE: dispara el ShockBurst (mínimo ~10us). NO se hace FLUSH_TX
    // después: el paquete sale ~130us más tarde y un flush lo cancelaría.
    gpio_set_level(r->ce_pin, 1);
    ets_delay_us(15);
    gpio_set_level(r->ce_pin, 0);

    r->packet_counter = c + 1;
}

// ISR del gptimer: solo despierta a las tareas (no toca SPI).
static bool IRAM_ATTR on_dwell_alarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *e, void *ctx) {
    BaseType_t woken = pdFALSE;
    for (int i = 0; i < NUM_RADIOS; i++) {
        if (radios[i].task) vTaskNotifyGiveFromISR(radios[i].task, &woken);
    }
    return woken == pdTRUE;
}

static void radio_task(void *arg) {
    radio_t *r = (radio_t *)arg;
    int idx = (int)(r - radios);
    while (1) {
        uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (g_pausa) {
            xEventGroupSetBits(eg, EV_PAUSED(idx));
            xEventGroupWaitBits(eg, EV_RESUME, pdFALSE, pdFALSE, portMAX_DELAY);
            ulTaskNotifyTake(pdTRUE, 0);   // descarta los ticks acumulados durante la pausa
            continue;
        }
        if (n > 1) r->perdidos += (n - 1);

        // ---- único cambio respecto al hopping puro ----
        if (g_jammer) jam_hop(r);
        else          hacer_hop(r);
    }
}

// Reescaneo: pausa LOS DOS radios a la vez (así uno no ensucia la medición del
// otro con su propia transmisión), escanea, reconstruye tablas y reanuda.
static void tarea_reescaneo(void *arg) {
    uint32_t mask = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) mask |= EV_PAUSED(i);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(REESCANEO_MS));
        xEventGroupClearBits(eg, EV_RESUME | EV_PAUSED_ALL);
        g_pausa = true;
        EventBits_t b = xEventGroupWaitBits(eg, mask, pdFALSE, pdTRUE, pdMS_TO_TICKS(100));
        if ((b & mask) == mask) {
            ets_delay_us(500);   // deja salir el último paquete en vuelo
            for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) escanear_bloque(&radios[i]);
            for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) generar_hop_table(&radios[i]);
            // si seguimos en jammer, hay que volver a levantar el carrier
            if (g_jammer) jam_refresh();
        } else {
            ESP_LOGW(TAG, "reescaneo cancelado: un radio no pauso a tiempo");
        }
        g_pausa = false;
        xEventGroupSetBits(eg, EV_RESUME);
    }
}

static void iniciar_gptimer(void) {
    gptimer_config_t cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,   // 1 tick = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &dwell_timer));
    gptimer_event_callbacks_t cbs = { .on_alarm = on_dwell_alarm };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(dwell_timer, &cbs, NULL));
    ESP_ERROR_CHECK(gptimer_enable(dwell_timer));
    gptimer_alarm_config_t alarm = {
        .alarm_count = DWELL_US,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(dwell_timer, &alarm));
    ESP_ERROR_CHECK(gptimer_start(dwell_timer));
}

// ---------- Consola ----------
static void procesar_comando(const char *line) {
    char cmd[32];
    if (sscanf(line, "%31s", cmd) != 1) return;

    if (!strcmp(cmd, "jam")) {
        if (strstr(line, "on"))       jam_set(true);
        else if (strstr(line, "off")) jam_set(false);
        else                          jam_set(!g_jammer);
    } else if (!strcmp(cmd, "full")) {
        g_jam_full = !g_jam_full;
        printf("barrido completo: %s\n", g_jam_full ? "SI (ignora blacklist)" : "NO (solo canales libres)");
        jam_refresh();
    } else if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
        printf("\n--- comandos ---\n"
               "  jam [on|off]   portadora continua en ambos NRF\n"
               "  full           barre TODOS los canales del bloque\n"
               "  status         estado\n\n");
    } else {
        printf("desconocido: %s  (escribi 'help')\n", cmd);
    }
    printf("nrf> ");
    fflush(stdout);
}

static void uart_consola_init(void) {
    uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0);
}

static void consola_task(void *arg) {
    char line[64];
    int n = 0;
    uint8_t ch;
    while (1) {
        int len = uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &ch, 1, pdMS_TO_TICKS(30));
        if (len <= 0) continue;
        if (ch == '\r' || ch == '\n') {
            if (n > 0) {
                line[n] = 0;
                procesar_comando(line);
                n = 0;
            } else {
                printf("nrf> ");
                fflush(stdout);
            }
        } else if (n < (int)sizeof(line) - 1) {
            line[n++] = (char)ch;
        }
    }
}

// ---------- Monitor serial ----------
#define C_VERDE  "\033[32m"
#define C_ROJO   "\033[31m"
#define C_AMAR   "\033[33m"
#define C_CYAN   "\033[36m"
#define C_NEG    "\033[1m"
#define C_RESET  "\033[0m"

static void mostrar_estado(uint32_t t_seg, uint32_t prev[NUM_RADIOS]) {
    printf("\n" C_NEG "──── NRF24 %s · t=%lus ────" C_RESET "\n",
           g_jammer ? "JAMMER" : "TX hop", (unsigned long)t_seg);
    for (int i = 0; i < NUM_RADIOS; i++) {
        radio_t *r = &radios[i];
        uint32_t ahora = r->packet_counter;
        uint32_t pps = (uint32_t)(((uint64_t)(ahora - prev[i]) * 1000) / STATUS_MS);
        prev[i] = ahora;
        int bus = r->host == SPI2_HOST ? 2 : 3;

        if (!r->presente) {
            printf("%s [SPI%d CSN%-2d CE%-2d] " C_ROJO "✖ NO RESPONDE" C_RESET "\n",
                   r->nombre, bus, (int)r->csn_pin, (int)r->ce_pin);
        } else if (pps == 0) {
            printf("%s [SPI%d CSN%-2d CE%-2d] " C_AMAR "▲ SIN PAQUETES" C_RESET "  pkts=%lu\n",
                   r->nombre, bus, (int)r->csn_pin, (int)r->ce_pin, (unsigned long)ahora);
        } else {
            printf("%s [SPI%d CSN%-2d CE%-2d] " C_VERDE "● %s" C_RESET
                   "  pkts=%-9lu %5lu/s  canal=%3u  libres=%2u/%d  vueltas=%lu  perdidos=%lu  flush=%lu\n",
                   r->nombre, bus, (int)r->csn_pin, (int)r->ce_pin,
                   g_jammer ? "CARRIER" : "TX",
                   (unsigned long)ahora, (unsigned long)pps,
                   (unsigned)r->canal_actual, (unsigned)r->hop_count, CHANNELS_PER_BLOQUE,
                   (unsigned long)r->vueltas, (unsigned long)r->perdidos, (unsigned long)r->flushes);
        }
    }
}

// ---------- LED ----------
static void hsv_to_rgb(int hue, uint8_t v, uint8_t *r, uint8_t *g, uint8_t *b) {
    int region = hue / 60;
    int rem = (hue % 60) * 255 / 60;
    uint8_t t = (uint8_t)(v * rem / 255);
    uint8_t q = (uint8_t)(v - t);
    switch (region) {
        case 0:  *r = v; *g = t; *b = 0; break;
        case 1:  *r = q; *g = v; *b = 0; break;
        case 2:  *r = 0; *g = v; *b = t; break;
        case 3:  *r = 0; *g = q; *b = v; break;
        case 4:  *r = t; *g = 0; *b = v; break;
        default: *r = v; *g = 0; *b = q; break;
    }
}

static void led_set(uint8_t r, uint8_t g, uint8_t b) {
    led_strip_set_pixel(led, 0, r, g, b);
    led_strip_refresh(led);
}

static void led_init(void) {
    led_strip_config_t strip_cfg = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = 1,
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

    // Autotest: rojo -> verde -> azul. Si esto NO se ve, el problema es el
    // pin/LED (prueba LED_GPIO = GPIO_NUM_38), no la lógica de abajo.
    led_set(LED_BRILLO, 0, 0);  vTaskDelay(pdMS_TO_TICKS(250));
    led_set(0, LED_BRILLO, 0);  vTaskDelay(pdMS_TO_TICKS(250));
    led_set(0, 0, LED_BRILLO);  vTaskDelay(pdMS_TO_TICKS(250));
    led_strip_clear(led);
}

static void led_task(void *arg) {
    int hue = 0;
    uint32_t prev[NUM_RADIOS] = { 0 };
    while (1) {
        bool todos_presentes = true, todos_tx = true;
        for (int i = 0; i < NUM_RADIOS; i++) {
            if (!radios[i].presente) { todos_presentes = false; todos_tx = false; continue; }
            uint32_t ahora = radios[i].packet_counter;
            if (ahora == prev[i]) todos_tx = false;
            prev[i] = ahora;
        }

        if (g_jammer) {
            led_set(0, LED_BRILLO, 0);                        // VERDE: jammer activo
        } else if (todos_presentes && todos_tx) {
            uint8_t r, g, b;
            hsv_to_rgb(hue, LED_BRILLO, &r, &g, &b);
            led_set(r, g, b);
            hue = (hue + LED_HUE_STEP) % 360;
        } else if (!todos_presentes) {
            led_set(LED_BRILLO, 0, 0);                        // rojo: falta un NRF
        } else {
            led_set(LED_BRILLO, LED_BRILLO / 2, 0);           // ámbar: presentes pero sin tráfico
        }
        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

void app_main(void) {
    led_init();
    eg = xEventGroupCreate();

    bus_init(SPI2_HOST, PIN_SCK2, PIN_MOSI2, PIN_MISO2);
    bus_init(SPI3_HOST, PIN_SCK3, PIN_MOSI3, PIN_MISO3);

    for (int i = 0; i < NUM_RADIOS; i++) radio_init(&radios[i]);
    vTaskDelay(pdMS_TO_TICKS(5));

    int activos = 0;
    for (int i = 0; i < NUM_RADIOS; i++) if (nrf_presente(&radios[i])) activos++;

    if (activos == NUM_RADIOS) {
        ESP_LOGI(TAG, "Los %d NRF responden correctamente", NUM_RADIOS);
    } else if (activos == 0) {
        ESP_LOGE(TAG, "NINGUN NRF responde: no se arranca el hopping.");
    } else {
        ESP_LOGW(TAG, "Solo %d de %d NRF responden: sigo con los que si", activos, NUM_RADIOS);
    }

    // Primer escaneo real antes de transmitir
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) escanear_bloque(&radios[i]);
    for (int i = 0; i < NUM_RADIOS; i++) if (radios[i].presente) generar_hop_table(&radios[i]);

    xTaskCreate(led_task, "led_task", 2048, NULL, 3, NULL);
    uart_consola_init();
    xTaskCreate(consola_task, "consola", 3072, NULL, 2, NULL);

    if (activos > 0) {
        for (int i = 0; i < NUM_RADIOS; i++) {
            if (!radios[i].presente) continue;
            xTaskCreatePinnedToCore(radio_task, radios[i].nombre, 3072, &radios[i],
                                    configMAX_PRIORITIES - 1, &radios[i].task, radios[i].core);
        }
        iniciar_gptimer();
        xTaskCreate(tarea_reescaneo, "reescaneo", 4096, NULL, 5, NULL);

        // ---- arranca en modo jammer si así está configurado ----
        if (g_jammer) jam_refresh();

        ESP_LOGI(TAG, "%d radio(s) activos · dwell=%dus · SPI=%dMHz · modo=%s",
                 activos, DWELL_US, SPI_HZ / 1000000, g_jammer ? "JAMMER" : "HOPPING TX");
        ESP_LOGI(TAG, "barrido de los %d canales cada %.1f ms",
                 NUM_RADIOS * CHANNELS_PER_BLOQUE,
                 (CHANNELS_PER_BLOQUE * DWELL_US) / 1000.0f);
    }

    printf("\n" C_CYAN "  jam [on|off]  portadora continua   |   full  barrido completo   |   help\n" C_RESET);
    printf("nrf> ");
    fflush(stdout);

    uint32_t prev[NUM_RADIOS] = { 0 };
    uint32_t t_seg = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_MS));
        t_seg += STATUS_MS / 1000;
        mostrar_estado(t_seg, prev);
    }
}
