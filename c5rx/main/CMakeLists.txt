idf_component_register(SRCS "main.c" "lanes.c" "link.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_parlio)
