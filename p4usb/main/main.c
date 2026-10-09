#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bridge.h"
#include "c5.h"
#include "console.h"
#include "decode.h"
#include "pins.h"
#include "usb.h"
#include "uvc.h"
#include "video.h"

void app_main(void)
{
    /* Nothing is known about the C5's firmware, so leave its RX undriven. */
    gpio_reset_pin(PIN_C5_TX);
    gpio_set_direction(PIN_C5_TX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_FLOATING);

    /* Above the decoder, so the console and the flashing bridge answer however busy it gets. */
    vTaskPrioritySet(NULL, 6);
    console_init();
    c5_init();
    video_init();
    uvc_init();
    usb_init();
    decode_start(NULL);
    say("\np4usb ready, type help\n> ");

    uint8_t buf[64];
    for (;;) {
        size_t n = host_read(buf, sizeof buf, 20);
        bridge_poll();
        if (bridge_host_bytes(buf, n)) continue;
        for (size_t i = 0; i < n; ++i) console_feed(buf[i]);
    }
}
