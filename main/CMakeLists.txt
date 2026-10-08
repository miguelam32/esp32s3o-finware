idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_spi
        esp_driver_gpio
        esp_driver_gptimer
        esp_driver_rmt
        esp_timer
        esp_hw_support
        heap
        freertos
        log
        led_strip
)
