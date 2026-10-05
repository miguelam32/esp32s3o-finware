#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "jammer.h"
#include "ws2812.h"

static const char *TAG = "app";

#ifndef CONFIG_JAM_MODE
#define CONFIG_JAM_MODE JAM_BLE
#endif

#ifndef CONFIG_JAM_DWELL
#define CONFIG_JAM_DWELL 1000
#endif

void app_main(void)
{
    esp_err_t rv = nvs_flash_init();
    if (rv == ESP_ERR_NVS_NO_FREE_PAGES || rv == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (ws2812_init() == ESP_OK) ws2812_set_mode(WS_MODE_PULSE);

    jam_cfg_t c1 = JAM_CFG("r1", SPI2_HOST,
                           (gpio_num_t)CONFIG_NRF1_CE_GPIO,
                           (gpio_num_t)CONFIG_NRF1_CS_GPIO,
                           (gpio_num_t)CONFIG_NRF1_SCK_GPIO,
                           (gpio_num_t)CONFIG_NRF1_MOSI_GPIO,
                           (gpio_num_t)CONFIG_NRF1_MISO_GPIO);

    jam_dev_t *r1 = jam_init(&c1);
    if (!r1) {
        ESP_LOGE(TAG, "radio 1 no detectada");
        return;
    }

    jam_start(r1, CONFIG_JAM_MODE, CONFIG_JAM_DWELL);
    ESP_LOGI(TAG, "ON modo=%s dwell=%dus",
             jam_mode_name(CONFIG_JAM_MODE), CONFIG_JAM_DWELL);
}
