#include "census.h"

#include <stdbool.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "console.h"
#include "pins.h"

#define SAMPLES 2000

const wire_t BUS_WIRES[] = {
    {"CLK", 0, 24},
    {"Q7", 1, 25}, {"Q6", 2, 50}, {"Q5", 3, 51}, {"Q4", 4, 52},
    {"Q3", 23, 3}, {"Q2", 24, 2}, {"Q1", 25, 49},
    {"I7", 6, 30}, {"I6", 7, 29}, {"I5", 8, 48}, {"I4", 9, 28},
    {"I3", 10, 47}, {"I2", 13, 33}, {"I1", 14, 46},
};
const int BUS_WIRE_COUNT = sizeof BUS_WIRES / sizeof BUS_WIRES[0];

static const wire_t s_control_wires[] = {
    {"EN", -1, PIN_C5_EN},
    {"BOOT", 28, PIN_C5_BOOT},
    {"C5 TX", 11, PIN_C5_RX},
    {"C5 RX", 12, PIN_C5_TX},
};

static int count_highs(int pin, gpio_pull_mode_t pull)
{
    gpio_set_pull_mode(pin, pull);
    esp_rom_delay_us(100);
    int highs = 0;
    for (int i = 0; i < SAMPLES; ++i) highs += gpio_get_level(pin);
    return highs;
}

static void report(const wire_t *w, bool as_input)
{
    /* Control pins may be driven by this firmware, so only reconfigure bus pins. */
    if (as_input) {
        gpio_reset_pin(w->p4);
        gpio_set_direction(w->p4, GPIO_MODE_INPUT);
    }
    int down = count_highs(w->p4, GPIO_PULLDOWN_ONLY);
    int up = count_highs(w->p4, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(w->p4, GPIO_FLOATING);

    const char *state;
    if (down == 0 && up == SAMPLES) state = "floating";
    else if (down == 0 && up == 0) state = "low";
    else if (down == SAMPLES && up == SAMPLES) state = "high";
    else state = "toggling";

    char c5[12] = "pad";
    if (w->c5 >= 0) snprintf(c5, sizeof c5, "%d", w->c5);
    say("  %-6s C5 %-3s P4 %-2d  %-8s  pd %3d%% pu %3d%%\n", w->name, c5, w->p4, state,
        down * 100 / SAMPLES, up * 100 / SAMPLES);
}

void census_run(int argc, char **argv)
{
    say("bus (P4 inputs):\n");
    for (int i = 0; i < BUS_WIRE_COUNT; ++i) report(&BUS_WIRES[i], true);
    say("control:\n");
    for (size_t i = 0; i < sizeof s_control_wires / sizeof s_control_wires[0]; ++i) {
        report(&s_control_wires[i], false);
    }
}
