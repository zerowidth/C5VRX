#include "driver/gpio.h"

#include "c5.h"
#include "console.h"
#include "pins.h"

void app_main(void)
{
    /* Nothing is known about the C5's firmware, so leave its RX undriven. */
    gpio_reset_pin(PIN_C5_TX);
    gpio_set_direction(PIN_C5_TX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_FLOATING);

    console_init();
    c5_init();
    say("\np4usb ready, C5 held in reset, type help\n> ");

    uint8_t buf[64];
    for (;;) {
        size_t n = host_read(buf, sizeof buf, 20);
        for (size_t i = 0; i < n; ++i) console_feed(buf[i]);
    }
}
