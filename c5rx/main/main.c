#include <stdio.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Waveshare C5-Zero antenna switch: low selects the on-board antenna. */
#define ANTENNA_SEL_GPIO GPIO_NUM_26
/* Long enough for the P4 to hear the step's line and sample the bus. */
#define WALK_STEP_MS 20

/* Same order and names as the P4's wire table in p4usb/main/census.c. */
static const struct {
    const char *name;
    gpio_num_t gpio;
} LANES[] = {
    {"CLK", 0},
    {"Q7", 1}, {"Q6", 2}, {"Q5", 3}, {"Q4", 4}, {"Q3", 23}, {"Q2", 24}, {"Q1", 25},
    {"I7", 6}, {"I6", 7}, {"I5", 8}, {"I4", 9}, {"I3", 10}, {"I2", 13}, {"I1", 14},
};
#define LANE_COUNT (sizeof LANES / sizeof LANES[0])

static void walk_step(const char *name)
{
    printf("walk %s\n", name);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(WALK_STEP_MS));
}

/* One lane high at a time, announced on the UART, so the P4 can name miswired lanes. */
static void walk_ones(void)
{
    for (size_t i = 0; i < LANE_COUNT; ++i) {
        gpio_reset_pin(LANES[i].gpio);
        gpio_set_direction(LANES[i].gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(LANES[i].gpio, 0);
    }
    walk_step("none");
    for (size_t i = 0; i < LANE_COUNT; ++i) {
        gpio_set_level(LANES[i].gpio, 1);
        walk_step(LANES[i].name);
        gpio_set_level(LANES[i].gpio, 0);
    }
    printf("walk done\n");
}

void app_main(void)
{
    gpio_set_direction(ANTENNA_SEL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ANTENNA_SEL_GPIO, 0);

    printf("c5rx ready\n");
    walk_ones();
}
