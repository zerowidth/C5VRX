#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "lanes.h"
#include "link.h"
#include "radio.h"

/* PHY internals for the lab commands. */
extern unsigned char phy_param[];
extern uint8_t phy_i2c_readReg(uint8_t block, uint8_t host, uint8_t reg);
extern void phy_i2c_writeReg(uint8_t block, uint8_t host, uint8_t reg, uint8_t value);

/* Waveshare C5-Zero antenna switch: low selects the on-board antenna, high the U.FL. */
#define ANTENNA_SEL_GPIO GPIO_NUM_26
/* Long enough for the P4 to hear the step's line and sample the bus. */
#define WALK_STEP_MS 20
#define CONSOLE_UART UART_NUM_0

static bool s_external;

static void antenna_set(bool external)
{
    s_external = external;
    gpio_set_level(ANTENNA_SEL_GPIO, external);
}

/* Saved here because the P4 resets the C5 on every decoder start. */
static void antenna_load(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    nvs_handle_t nvs;
    uint8_t external = 0;
    if (err == ESP_OK && nvs_open("c5rx", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, "antenna", &external);
        nvs_close(nvs);
    }
    gpio_set_direction(ANTENNA_SEL_GPIO, GPIO_MODE_OUTPUT);
    antenna_set(external);
}

static esp_err_t antenna_save(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("c5rx", NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_u8(nvs, "antenna", s_external);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

static const char *antenna_name(void)
{
    return s_external ? "ext" : "int";
}

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
    char *argv[6];
    int argc = 0;
    for (char *tok = strtok(line, " "); tok && argc < 6; tok = strtok(NULL, " ")) argv[argc++] = tok;
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
    } else if (strcmp(argv[0], "11p") == 0 && argc == 3) {
        radio_11p(atoi(argv[1]), atoi(argv[2]));
        reply("ok 11p %d %d", atoi(argv[1]), atoi(argv[2]));
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
    } else if (strcmp(argv[0], "antenna") == 0 && argc == 1) {
        reply("ok antenna %s", antenna_name());
    } else if (strcmp(argv[0], "antenna") == 0 && argc == 2 &&
               (strcmp(argv[1], "int") == 0 || strcmp(argv[1], "ext") == 0)) {
        bool external = argv[1][0] == 'e';
        esp_err_t err = ESP_OK;
        if (external != s_external) {
            antenna_set(external);
            err = antenna_save();
        }
        if (err == ESP_OK) reply("ok antenna %s", antenna_name());
        else reply("err antenna %s not saved: %s", antenna_name(), esp_err_to_name(err));
    } else if (strcmp(argv[0], "scan") == 0 && argc == 1) {
        esp_err_t err = radio_scan();
        if (err == ESP_OK) reply("ok scan started");
        else reply("err %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "scan") == 0 && argc == 2) {
        unsigned channel, bssid;
        int rssi;
        if (radio_scan_result((unsigned)atoi(argv[1]), &channel, &rssi, &bssid)) reply("ok ap ch=%u rssi=%d bssid=%04x", channel, rssi, bssid);
        else reply("err no such result");
    } else if (strcmp(argv[0], "sniff") == 0 && argc == 3 && strcmp(argv[1], "on") == 0) {
        esp_err_t err = radio_sniff((uint8_t)atoi(argv[2]));
        if (err == ESP_OK) reply("ok sniff %d", atoi(argv[2]));
        else reply("err %s", esp_err_to_name(err));
    } else if (strcmp(argv[0], "sniff") == 0 && argc == 2) {
        unsigned key, count, beacons;
        int rssi, noise, len;
        if (radio_sniff_result((unsigned)atoi(argv[1]), &key, &count, &beacons, &rssi, &noise, &len)) {
            reply("ok tx=%04x n=%u beacons=%u rssi=%d noise=%d len=%d", key, count, beacons, rssi, noise, len);
        } else {
            reply("err no such result");
        }
    } else if (strcmp(argv[0], "peek") == 0 && argc == 2) {
        reply("ok peek %08lx", (unsigned long)*(volatile uint32_t *)strtoul(argv[1], NULL, 16));
    } else if (strcmp(argv[0], "poke") == 0 && argc == 3) {
        *(volatile uint32_t *)strtoul(argv[1], NULL, 16) = strtoul(argv[2], NULL, 16);
        reply("ok poke");
    } else if (strcmp(argv[0], "i2c") == 0 && (argc == 4 || argc == 5)) {
        uint8_t block = strtoul(argv[1], NULL, 16), host = atoi(argv[2]), reg = atoi(argv[3]);
        if (argc == 5) phy_i2c_writeReg(block, host, reg, strtoul(argv[4], NULL, 16));
        reply("ok i2c %02x", phy_i2c_readReg(block, host, reg));
    } else if (strcmp(argv[0], "param") == 0 && (argc == 2 || argc == 3)) {
        unsigned offset = strtoul(argv[1], NULL, 16);
        if (argc == 3) phy_param[offset] = strtoul(argv[2], NULL, 16);
        reply("ok param %02x", phy_param[offset]);
    } else if (strcmp(argv[0], "call") == 0 && argc >= 2) {
        /* Lab only: a PHY function by name with up to three integer arguments. */
        uint32_t a[3] = {0};
        for (int k = 2; k < argc && k < 5; ++k) a[k - 2] = strtoul(argv[k], NULL, 0);
        reply("ok call %ld", (long)radio_call(argv[1], a[0], a[1], a[2]));
    } else if (strcmp(argv[0], "status") == 0) {
        int rssi = 0, noise = 0;
        bool have_rssi = radio_rssi(&rssi), have_noise = radio_noise_floor(&noise);
        reply("ok status ch=%s freq=%u gain=%u rssi=%d%s noise=%d%s ant=%s", radio_channel(), radio_mhz(), radio_gain(),
              rssi, have_rssi ? "" : "?", noise, have_noise ? "" : "?", antenna_name());
    } else {
        reply("err unknown command %s", argv[0]);
    }
}

void app_main(void)
{
    antenna_load();

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
