/*
 * ws2812.h - LED WS2812 con arcoiris lento + latido del jammer.
 * Usa el componente oficial led_strip (backend RMT).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    WS_MODE_OFF = 0,
    WS_MODE_RAINBOW,     /* arcoiris lento, una vuelta cada ~24 s */
    WS_MODE_BREATHE,     /* respiracion blanquiazul ~4 s           */
    WS_MODE_PULSE,       /* destello corto: 1 por radio emitiendo  */
    WS_MODE_SOLID,       /* color fijo                            */
} ws_mode_t;

esp_err_t  ws2812_init(void);
void       ws2812_deinit(void);
bool       ws2812_ready(void);

void       ws2812_set_mode(ws_mode_t mode);
ws_mode_t  ws2812_get_mode(void);
void       ws2812_set_rgb(uint8_t r, uint8_t g, uint8_t b);
void       ws2812_set_brightness(uint8_t br);   /* 0..255 */

/* Llamar cada vez que una radio arranca a emitir */
void       ws2812_notify(int count);

#ifdef __cplusplus
}
#endif
