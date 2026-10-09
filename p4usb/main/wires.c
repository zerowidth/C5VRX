#include "wires.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "c5.h"
#include "console.h"
#include "pins.h"

#define TIMEOUT_MS 3000
#define NO_STEP UINT32_MAX

static SemaphoreHandle_t s_done;
static volatile bool s_armed;
/* Bitmask of P4 bus pins seen high, per walk step; index 0 is the all-low step. */
static uint32_t s_seen[1 + 32];

static uint32_t sample_bus(void)
{
    uint32_t mask = 0;
    for (int i = 0; i < BUS_WIRE_COUNT; ++i) {
        if (gpio_get_level(BUS_WIRES[i].p4)) mask |= 1u << i;
    }
    return mask;
}

void wires_line(const char *line)
{
    if (!s_armed || strncmp(line, "walk ", 5) != 0) return;
    const char *name = line + 5;
    if (strcmp(name, "done") == 0) {
        s_armed = false;
        xSemaphoreGive(s_done);
    } else if (strcmp(name, "none") == 0) {
        s_seen[0] = sample_bus();
    } else {
        for (int i = 0; i < BUS_WIRE_COUNT; ++i) {
            if (strcmp(name, BUS_WIRES[i].name) == 0) s_seen[1 + i] = sample_bus();
        }
    }
}

static int report(void)
{
    int problems = 0;
    if (s_seen[0] == NO_STEP) {
        say("  no all-low step received\n");
        ++problems;
    } else {
        for (int j = 0; j < BUS_WIRE_COUNT; ++j) {
            if (s_seen[0] & (1u << j)) {
                say("  P4 %d (%s) high with every C5 lane low\n", BUS_WIRES[j].p4, BUS_WIRES[j].name);
                ++problems;
            }
        }
    }
    for (int i = 0; i < BUS_WIRE_COUNT; ++i) {
        const wire_t *w = &BUS_WIRES[i];
        uint32_t seen = s_seen[1 + i];
        if (seen == NO_STEP) {
            say("  %s: no walk step received\n", w->name);
            ++problems;
            continue;
        }
        seen &= ~s_seen[0];
        if (seen == 1u << i) continue;
        ++problems;
        if (seen == 0) say("  %s: C5 %d not seen on any P4 pin, expected %d\n", w->name, w->c5, w->p4);
        for (int j = 0; j < BUS_WIRE_COUNT; ++j) {
            if (j != i && (seen & (1u << j))) {
                say("  %s: C5 %d seen on P4 %d, expected %d\n", w->name, w->c5, BUS_WIRES[j].p4, w->p4);
            }
        }
    }
    return problems;
}

void wires_run(int argc, char **argv)
{
    if (!s_done) s_done = xSemaphoreCreateBinary();
    /* Pull-downs make an open wire read low rather than float. */
    for (int i = 0; i < BUS_WIRE_COUNT; ++i) {
        gpio_reset_pin(BUS_WIRES[i].p4);
        gpio_set_direction(BUS_WIRES[i].p4, GPIO_MODE_INPUT);
        gpio_set_pull_mode(BUS_WIRES[i].p4, GPIO_PULLDOWN_ONLY);
    }
    for (size_t i = 0; i < sizeof s_seen / sizeof s_seen[0]; ++i) s_seen[i] = NO_STEP;
    xSemaphoreTake(s_done, 0);
    s_armed = true;
    c5_run();

    if (xSemaphoreTake(s_done, pdMS_TO_TICKS(TIMEOUT_MS)) != pdTRUE) {
        s_armed = false;
        say("wires: no 'walk done' from the C5; is c5rx flashed?\n");
    }
    int problems = report();
    if (problems == 0) say("wires: all %d ok\n", BUS_WIRE_COUNT);
    else say("wires: %d problem(s)\n", problems);

    for (int i = 0; i < BUS_WIRE_COUNT; ++i) gpio_set_pull_mode(BUS_WIRES[i].p4, GPIO_FLOATING);
}
