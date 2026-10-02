/*
 * ws2812.h — driver WS2812/WS2812B minimo sobre RMT nativo (ESP-IDF 6.0.1)
 *
 * Usa la API nueva (rmt_new_tx_channel + rmt_new_copy_encoder), que es la
 * unica que existe en IDF 6.0: la legacy (driver/rmt.h) fue eliminada.
 *
 * Cero dependencias externas: todo vive en IDF.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* GPIO48 = LED RGB del ESP32-S3 DevKitC-1 (N16R8).
 * Si tu placa es otra y no enciende, prueba 38, 47 o 2. */
#ifndef WS2812_GPIO
#define WS2812_GPIO 48
#endif

esp_err_t ws2812_init(int gpio);
void      ws2812_set(uint8_t r, uint8_t g, uint8_t b);   /* bloqueante */
void      ws2812_off(void);

/* Rueda clasica de color: pos 0..255 cicla de forma continua. */
void      ws2812_wheel(uint8_t pos, uint8_t *r, uint8_t *g, uint8_t *b);

#ifdef __cplusplus
}
#endif
