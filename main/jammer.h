/*
 * jammer.h - Jammer BLE/nRF24L01+ todo-en-uno. ESP-IDF v6.0.x.
 *
 * Un solo archivo: driver SPI + portadora continua + tarea de barrido.
 * Arranca solo al boot, no necesita consola.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JAM_BLE_ADV = 0,   /* 2, 26, 80   -> canales de publicidad BLE 37/38/39 */
    JAM_BLE_ALL,       /* 2..37 + 80  -> datos + publicidad (audio incluido) */
    JAM_SPRAY,         /* 2..80       -> toda la banda util                 */
    JAM_MODE_COUNT
} jam_mode_t;

typedef struct {
    const char       *name;
    spi_host_device_t host;
    gpio_num_t        sck, miso, mosi, ce, cs;
    uint32_t          clock_hz;   /* 8 MHz: techo comodo del nRF24 */
    int8_t            core;       /* -1 = sin pinning              */
} jam_cfg_t;

#define JAM_CFG(NAME, HOST, CE, CS, SCK, MOSI, MISO)  \
    { .name = NAME, .host = HOST, .ce = CE, .cs = CS, \
      .sck = SCK, .mosi = MOSI, .miso = MISO,         \
      .clock_hz = 8000000, .core = -1 }

typedef struct jam_dev jam_dev_t;

jam_dev_t *jam_init(const jam_cfg_t *cfg);
esp_err_t  jam_start(jam_dev_t *d, jam_mode_t mode, uint32_t dwell_us);
esp_err_t  jam_stop(jam_dev_t *d);
bool       jam_running(const jam_dev_t *d);
jam_mode_t jam_get_mode(const jam_dev_t *d);
void       jam_set_dwell(jam_dev_t *d, uint32_t dwell_us);

/* diagnostico */
uint8_t    jam_read_reg(jam_dev_t *d, uint8_t reg);
uint8_t    jam_channel(jam_dev_t *d);
uint32_t   jam_freq_hz(jam_dev_t *d);
const char *jam_mode_name(jam_mode_t m);
int        jam_mode_count(jam_mode_t m);

#ifdef __cplusplus
}
#endif
