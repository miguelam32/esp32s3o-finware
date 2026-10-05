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
    JAM_ADV = 0,   /* 2,26,80        -> solo publicidad BLE, 3 canales  */
    JAM_BLE,       /* pares 2..78+80 -> BLE completo, 40 canales        */
    JAM_BT,        /* 2..80          -> BT clasico A2DP, 79 canales     */
    JAM_MODE_COUNT
} jam_mode_t;

typedef struct {
    const char       *name;
    spi_host_device_t host;
    gpio_num_t        sck, miso, mosi, ce, cs;
    uint32_t          clock_hz;
    int8_t            core;
} jam_cfg_t;

#define JAM_CFG(NAME, HOST, CE, CS, SCK, MOSI, MISO)  \
    { .name = NAME, .host = HOST, .ce = CE, .cs = CS, \
      .sck = SCK, .mosi = MOSI, .miso = MISO,         \
      .clock_hz = 10000000, .core = -1 }

typedef struct jam_dev jam_dev_t;

jam_dev_t *jam_init(const jam_cfg_t *cfg);
esp_err_t  jam_start(jam_dev_t *d, jam_mode_t mode, uint32_t dwell_us);
esp_err_t  jam_stop(jam_dev_t *d);
bool       jam_running(const jam_dev_t *d);
jam_mode_t jam_get_mode(const jam_dev_t *d);
void       jam_set_dwell(jam_dev_t *d, uint32_t dwell_us);

const char *jam_mode_name(jam_mode_t m);
int         jam_mode_count(jam_mode_t m);
uint8_t     jam_channel(jam_dev_t *d);

#ifdef __cplusplus
}
#endif
