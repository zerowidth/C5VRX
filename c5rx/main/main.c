#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lanes.h"
#include "link.h"
#include "radio.h"

/* Waveshare C5-Zero antenna switch: low selects the on-board antenna. */
#define ANTENNA_SEL_GPIO GPIO_NUM_26
/* Long enough for the P4 to hear the step's line and sample the bus. */
#define WALK_STEP_MS 20
#define CONSOLE_UART UART_NUM_0

static void walk_step(const char *name)
{
    printf("walk %s\n", name);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(WALK_STEP_MS));
}

/* One lane high at a time, announced on the UART, so the P4 can name miswired lanes. */
static void walk_ones(void)
{
    lanes_drive_low();
    walk_step("none");
    for (int i = 0; i < LANE_COUNT; ++i) {
        gpio_set_level(LANES[i].gpio, 1);
        walk_step(LANES[i].name);
        gpio_set_level(LANES[i].gpio, 0);
    }
    printf("walk done\n");
}

static void reply(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);
}

static void run_command(char *line)
{
    char *argv[4];
    int argc = 0;
    for (char *tok = strtok(line, " "); tok && argc < 4; tok = strtok(NULL, " ")) argv[argc++] = tok;
    if (argc == 0) return;

    if (strcmp(argv[0], "ping") == 0) {
        reply("ok c5rx");
    } else if (strcmp(argv[0], "link") == 0 && argc == 2 && strcmp(argv[1], "off") == 0) {
        link_stop();
        reply("ok link off");
    } else if (strcmp(argv[0], "link") == 0 && argc == 3) {
        radio_iq_stop();
        esp_err_t err = link_start(argv[1][0], (uint32_t)atoi(argv[2]) * 1000000u);
        if (err == ESP_OK) reply("ok link %s", argv[1]);
        else reply("err %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "tune") == 0 && argc == 2) {
        esp_err_t err = radio_tune(argv[1]);
        if (err == ESP_OK) reply("ok tune %s %u", radio_channel(), radio_mhz());
        else reply("err %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "gain") == 0 && argc == 2) {
        radio_set_gain((uint8_t)atoi(argv[1]));
        reply("ok gain %u", radio_gain());
    } else if (strcmp(argv[0], "iq") == 0 && (argc == 2 || argc == 3 || argc == 5) && strcmp(argv[1], "on") == 0) {
        link_stop();
        int every = argc >= 3 ? atoi(argv[2]) : 2;
        int q_top = argc == 5 ? atoi(argv[3]) : 9, i_top = argc == 5 ? atoi(argv[4]) : 19;
        esp_err_t err = radio_iq_start(every, q_top, i_top);
        if (err == ESP_OK) reply("ok iq on %d %d %d", every, q_top, i_top);
        else reply("err %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "iq") == 0 && argc == 2 && strcmp(argv[1], "off") == 0) {
        radio_iq_stop();
        reply("ok iq off");
    } else if (strcmp(argv[0], "clk") == 0 && argc == 3 && strcmp(argv[1], "slip") == 0) {
        radio_clock_slip(atoi(argv[2]));
        reply("ok clk slip %d", atoi(argv[2]));
    } else if (strcmp(argv[0], "status") == 0) {
        int rssi = 0, noise = 0;
        bool have_rssi = radio_rssi(&rssi), have_noise = radio_noise_floor(&noise);
        reply("ok status ch=%s freq=%u gain=%u rssi=%d%s noise=%d%s", radio_channel(), radio_mhz(), radio_gain(),
              rssi, have_rssi ? "" : "?", noise, have_noise ? "" : "?");
    } else {
        reply("err unknown command %s", argv[0]);
    }
}

void app_main(void)
{
    gpio_set_direction(ANTENNA_SEL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ANTENNA_SEL_GPIO, 0);

    ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL, 0));
    printf("c5rx ready\n");
    walk_ones();

    char line[64];
    size_t len = 0;
    for (;;) {
        uint8_t c;
        if (uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY) != 1) continue;
        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            len = 0;
            run_command(line);
        } else if (len < sizeof line - 1) {
            line[len++] = (char)c;
        }
    }
}
