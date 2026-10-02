/*
 * karma.h — respuesta automatica a probe requests
 *
 * No es un puerto de karma_attack.cpp: es la version minima que funciona
 * sobre rf_core. Escucha los PROBE_REQ y contesta con un beacon que dice
 * "soy esa red". El cliente se asocia el solo. Cero disrupcion.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "rf_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KARMA_MAX_TARGETS 8

typedef struct {
    bool     active;
    bool     wpa2;
    uint8_t  chan;
    uint8_t  ssid_len;
    uint8_t  bssid[6];
    uint32_t probes;
    int64_t  first_us;
    int64_t  last_probe_us;
    int64_t  last_beacon_us;
    char     ssid[33];
} karma_target_t;

/* Se dispara cuando un cliente intenta autenticarse contra nuestro BSSID:
 * es el momento de reconvertir el softAP para que la asociacion complete. */
typedef void (*karma_auth_cb_t)(const char *ssid, uint8_t chan);

esp_err_t karma_init(void);
void      karma_start(void);
void      karma_stop(void);

/* Llamar desde tu callback de frames, para cada management frame. */
void      karma_feed(const rf_mgmt_info_t *m);

void      karma_set_auth_cb(karma_auth_cb_t cb);
uint32_t  karma_target_count(void);
karma_target_t *karma_targets(void);

#ifdef __cplusplus
}
#endif
