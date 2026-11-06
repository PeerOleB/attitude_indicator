idf_component_register(SRCS "main.c" "sensors.c" "touch.c" "settings.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_lcd driver esp_timer esp_driver_gpio esp_driver_spi esp_driver_i2c esp_driver_uart nvs_flash)
