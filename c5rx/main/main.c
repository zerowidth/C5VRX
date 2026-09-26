#include <stdio.h>

#include "driver/gpio.h"

/* Waveshare C5-Zero antenna switch: low selects the on-board antenna. */
#define ANTENNA_SEL_GPIO GPIO_NUM_26

void app_main(void)
{
    gpio_set_direction(ANTENNA_SEL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ANTENNA_SEL_GPIO, 0);

    printf("c5rx ready\n");
}
