/*
 * nrf24_dual.h - Motor del jammer CW sobre cualquier instancia de nrf24.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "nrf24.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    JAM_MODE_TEST = 0,     /* canales de prueba del repo original  */
    JAM_MODE_WIFI,         /* centros de canal WiFi 2.4 GHz        */
    JAM_MODE_BLE_DATA,     /* BLE datos: indice 0..35 -> nRF 2..37 */
    JAM_MODE_BLE_ADV,      /* BLE adv 37/38/39 -> nRF 2/26/80      */
    JAM_MODE_BAND_SPRAY,   /* barrido de toda la banda util        */
    JAM_MODE_USB3,         /* armonicos del USB 3.0                */
    JAM_MODE_VIDEO,        /* TX de video analogico                */
    JAM_MODE_RC,           /* juguetes RC (canales impares)        */
    JAM_MODE_ZIGBEE,       /* IEEE 802.15.4 canales 11..26         */
    JAM_MODE_FULL,         /* 1..124 MHz sobre 2400                */
    JAM_MODE_COUNT
} jam_mode_t;

typedef struct {
    const char  *name;
    const uint8_t *channels;
    size_t       count;
} jam_channel_list_t;

const jam_channel_list_t *jam_mode_list(jam_mode_t mode);

typedef struct {
    jam_mode_t  mode;
    bool        fhss;        /* false = secuencial, true = aleatorio */
    uint32_t    dwell_us;    /* permanencia por canal                */
    nrf24_pa_t  pa;
    int8_t      core;        /* -1 = sin pinning                     */
    uint8_t     priority;
} jam_cfg_t;

#define JAM_CFG_DEFAULT()                 \
    {                                     \
        .mode = JAM_MODE_BLE_ADV,         \
        .fhss = false,                    \
        .dwell_us = 1500,                 \
        .pa = NRF24_PA_0DBM,              \
        .core = -1, .priority = 5,        \
    }

esp_err_t  jam_start(nrf24_dev_t *r, const jam_cfg_t *cfg);
esp_err_t  jam_stop(nrf24_dev_t *r);
bool       jam_active(const nrf24_dev_t *r);
jam_mode_t jam_mode(const nrf24_dev_t *r);
uint32_t   jam_dwell_us(const nrf24_dev_t *r);
int        jam_active_count(void);

void jam_set_mode(nrf24_dev_t *r, jam_mode_t mode);
void jam_set_fhss(nrf24_dev_t *r, bool on);
void jam_set_dwell_us(nrf24_dev_t *r, uint32_t us);
void jam_set_pa(nrf24_dev_t *r, nrf24_pa_t pa);

#ifdef __cplusplus
}
#endif
