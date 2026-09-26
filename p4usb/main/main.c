#include "driver/gpio.h"

#include "console.h"
#include "pins.h"

void app_main(void)
{
    /* Nothing is known about the C5's firmware, so drive nothing it could fight. */
    const int control[] = {PIN_C5_EN, PIN_C5_BOOT, PIN_C5_RX, PIN_C5_TX};
    for (size_t i = 0; i < sizeof control / sizeof control[0]; ++i) {
        gpio_reset_pin(control[i]);
        gpio_set_direction(control[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(control[i], GPIO_FLOATING);
    }

    console_init();
    say("\np4usb ready, type help\n> ");

    uint8_t buf[64];
    for (;;) {
        size_t n = host_read(buf, sizeof buf, 20);
        for (size_t i = 0; i < n; ++i) console_feed(buf[i]);
    }
}
