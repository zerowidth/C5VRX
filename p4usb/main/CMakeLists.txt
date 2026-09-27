idf_component_register(SRCS "main.c" "console.c" "census.c" "c5.c" "bridge.c" "wires.c" "link.c"
                            "overlay.c" "video.c" "uvc.c"
                       PRIV_INCLUDE_DIRS "../../main"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer esp_driver_parlio
                                     esp_driver_jpeg usb_device_uvc espressif__tinyusb)
