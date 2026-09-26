#include "c5.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bridge.h"
#include "console.h"
#include "pins.h"
#include "wires.h"

#define C5_BAUD 115200
/* The C5's EN has 10k and 1 uF, so it takes about 10 ms to rise after release. */
#define EN_LOW_MS 50
#define BOOT_AFTER_EN_MS 100

/* Off by default: unknown C5 firmware may drive GPIO 11 with anything. */
static volatile bool s_log;

static void open_drain_high(int pin)
{
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
}

static void split_lines(const uint8_t *buf, size_t n)
{
    static char line[96];
    static size_t len;
    for (size_t i = 0; i < n; ++i) {
        if (buf[i] == '\n' || buf[i] == '\r') {
            line[len] = '\0';
            if (len > 0) wires_line(line);
            len = 0;
        } else if (buf[i] >= 0x20 && buf[i] < 0x7f && len < sizeof line - 1) {
            line[len++] = (char)buf[i];
        }
    }
}

static void relay_task(void *arg)
{
    uint8_t buf[128];
    for (;;) {
        size_t n = uart_read_some(C5_UART, buf, sizeof buf, 20);
        if (n == 0) continue;
        bridge_c5_bytes(buf, n);
        if (s_log || bridge_active()) host_write(buf, n);
        if (!bridge_active()) split_lines(buf, n);
    }
}

void c5_init(void)
{
    open_drain_high(PIN_C5_BOOT);
    open_drain_high(PIN_C5_EN);
    c5_hold();

    /* RX only: the C5's GPIO 12 may be an output under unknown firmware. */
    const uart_config_t cfg = {
        .baud_rate = C5_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(C5_UART, 4096, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(C5_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(C5_UART, UART_PIN_NO_CHANGE, PIN_C5_RX, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    /* Keeps the line idle rather than floating while the C5 is in reset. */
    gpio_pullup_en(PIN_C5_RX);
    xTaskCreate(relay_task, "c5_relay", 3072, NULL, 5, NULL);
}

void c5_hold(void)
{
    gpio_set_level(PIN_C5_EN, 0);
}

void c5_run(void)
{
    gpio_set_level(PIN_C5_BOOT, 1);
    gpio_set_level(PIN_C5_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(EN_LOW_MS));
    gpio_set_level(PIN_C5_EN, 1);
}

void c5_download(void)
{
    gpio_set_level(PIN_C5_BOOT, 0);
    gpio_set_level(PIN_C5_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(EN_LOW_MS));
    gpio_set_level(PIN_C5_EN, 1);
    vTaskDelay(pdMS_TO_TICKS(BOOT_AFTER_EN_MS));
    gpio_set_level(PIN_C5_BOOT, 1);
}

void c5_command(int argc, char **argv)
{
    const char *sub = argc > 1 ? argv[1] : "";
    if (strcmp(sub, "hold") == 0) {
        c5_hold();
    } else if (strcmp(sub, "run") == 0) {
        c5_run();
    } else if (strcmp(sub, "dl") == 0) {
        c5_download();
    } else if (strcmp(sub, "log") == 0 && argc > 2) {
        s_log = strcmp(argv[2], "on") == 0;
    } else {
        say("c5 hold | run | dl | log on|off\n");
        return;
    }
    say("ok\n");
}
