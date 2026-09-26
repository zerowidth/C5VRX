idf_component_register(SRCS "main.c" "console.c" "census.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer)
