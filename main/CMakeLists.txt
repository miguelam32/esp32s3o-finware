idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_driver_rmt
        esp_timer
        esp_hw_support
        esp_rom
        freertos
        log
        heap
        nvs_flash
        esp_netif
        esp_event
        esp_wifi
        esp_driver_gptimer
        led_strip
)

