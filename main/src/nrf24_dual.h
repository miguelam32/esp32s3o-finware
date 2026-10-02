/*
 * nrf24_dual.h — dos modulos nRF24 en paralelo, uno por host SPI y uno
 * por core. El barrido de 126 canales tarda la mitad.
 *
 * Modo recomendado ("split"):
 *   A = siempre en RX (escucha / espectro)
 *   B = siempre en TX (jammer / inyeccion)
 * Asi nunca dejas de escuchar mientras atacas.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "nrf24.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NRF_DUAL_CHANNELS 126u

typedef struct {
    uint8_t  rpd[NRF_DUAL_CHANNELS];   /* mapa de energia 0/1 por canal */
    uint32_t sweep_us;                 /* duracion del ultimo barrido    */
    uint32_t hits;                     /* canales con energia            */
} nrf_dual_result_t;

esp_err_t nrf24_dual_init(const nrf24_cfg_t *a, const nrf24_cfg_t *b);

/* Barrido no bloqueante: A barre 0..62, B barre 63..125, a la vez. */
void      nrf24_dual_sweep_async(void);
bool      nrf24_dual_sweep_done(void);
void      nrf24_dual_result(nrf_dual_result_t *out);

nrf24_dev_t *nrf24_dual_a(void);
nrf24_dev_t *nrf24_dual_b(void);

#ifdef __cplusplus
}
#endif
