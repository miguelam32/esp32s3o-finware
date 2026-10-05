/*
 * main.c - Arranca el jammer solo al boot.
 *
 * Si tu main.c tenia init de nvs/wifi para karma o rf_core, deja esas
 * lineas donde estan y solo engancha la seccion JAMMER.
 */
#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "jammer.h"
#include "ws2812.h"

static const char *TAG = "app";

/* Elegi el modo aca. JAM_BLE_ALL es el que mata el audio de un headset. */
#ifndef CONFIG_JAM_MODE
#define CONFIG_JAM_MODE JAM_BLE_ALL
#endif

void app_main(void)
{
    esp_err_t rv = nvs_flash_init();
    if (rv == ESP_ERR_NVS_NO_FREE_PAGES || rv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* --- tu init existente (rf_core, karma, wifi) va aca --- */

    if (ws2812_init() == ESP_OK) ws2812_set_mode(WS_MODE_PULSE);

    /* --- las dos radios --- */
    jam_cfg_t c1 = JAM_CFG("r1", SPI2_HOST,
                           (gpio_num_t)CONFIG_NRF1_CE_GPIO,
                           (gpio_num_t)CONFIG_NRF1_CS_GPIO,
                           (gpio_num_t)CONFIG_NRF1_SCK_GPIO,
                           (gpio_num_t)CONFIG_NRF1_MOSI_GPIO,
                           (gpio_num_t)CONFIG_NRF1_MISO_GPIO);

    jam_cfg_t c2 = JAM_CFG("r2", SPI3_HOST,
                           (gpio_num_t)CONFIG_NRF2_CE_GPIO,
                           (gpio_num_t)CONFIG_NRF2_CS_GPIO,
                           (gpio_num_t)CONFIG_NRF2_SCK_GPIO,
                           (gpio_num_t)CONFIG_NRF2_MOSI_GPIO,
                           (gpio_num_t)CONFIG_NRF2_MISO_GPIO);

    jam_dev_t *r1 = jam_init(&c1);
    jam_dev_t *r2 = jam_init(&c2);      /* NULL si no esta conectada: no pasa nada */

    if (!r1) ESP_LOGE(TAG, "radio 1 no detectada");

    if (r1) {
        esp_err_t e = jam_start(r1, CONFIG_JAM_MODE, 2000);
        ESP_LOGI(TAG, "jammer r1: %s", esp_err_to_name(e));
    }
    if (r2) {
        jam_start(r2, CONFIG_JAM_MODE, 2000);
    }

    ESP_LOGI(TAG, "listo. modo=%s  r1=%s r2=%s",
             jam_mode_name(CONFIG_JAM_MODE),
             r1 ? "ON" : "-", r2 ? "ON" : "-");
}
