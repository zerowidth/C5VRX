idf_component_register(SRCS "main.c" "lanes.c" "link.c" "radio.c"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_parlio
                                    esp_wifi esp_phy esp_netif esp_event nvs_flash esp_rom)
