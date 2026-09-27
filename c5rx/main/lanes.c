#include "lanes.h"

const lane_t LANES[LANE_COUNT] = {
    {"CLK", 0},
    {"Q7", 1}, {"Q6", 2}, {"Q5", 3}, {"Q4", 4}, {"Q3", 23}, {"Q2", 24}, {"Q1", 25},
    {"I7", 6}, {"I6", 7}, {"I5", 8}, {"I4", 9}, {"I3", 10}, {"I2", 13}, {"I1", 14},
};

void lanes_drive_low(void)
{
    for (int i = 0; i < LANE_COUNT; ++i) {
        gpio_reset_pin(LANES[i].gpio);
        gpio_set_direction(LANES[i].gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(LANES[i].gpio, 0);
    }
}
