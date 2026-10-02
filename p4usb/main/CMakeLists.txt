idf_component_register(SRCS "main.c" "console.c" "census.c" "c5.c" "bridge.c" "wires.c" "link.c" "iq.c" "decode.c" "deint.S" "chroma.S" "demod.S" "jumps.S" "luma.S" "tnr.S"
                            "overlay.c" "video.c" "uvc.c" "usb.c"
                       PRIV_INCLUDE_DIRS "../../main"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer esp_driver_parlio esp_driver_bitscrambler esp_hal_dma
                                     esp_driver_jpeg esp_driver_dma esp_hw_support esp_mm espressif__tinyusb)

target_bitscrambler_add_src("bs_pass.bsasm")
target_bitscrambler_add_src("bs_half.bsasm")
target_bitscrambler_add_src("bs_phase.bsasm")
target_bitscrambler_add_src("bs_phase_check.bsasm")

# TinyUSB reads its configuration from tusb_config.h on its own include path.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/tusb")

# usb.c keeps TinyUSB's video class from taking a new TX FIFO each time it reopens its endpoint.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=dcd_edpt_open" "-Wl,--wrap=dcd_edpt_close_all")
