#include "c5.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
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
/* Set once c5rx announces itself after a reset, so GPIO 12 is known to be its UART RX. */
static volatile bool s_c5rx;
static SemaphoreHandle_t s_ready;
static SemaphoreHandle_t s_reply;
static char s_reply_line[96];

static void open_drain_high(int pin)
{
    gpio_reset_pin(pin);
    gpio_set_pull_mode(pin, GPIO_FLOATING);
    gpio_set_level(pin, 1);
    gpio_set_direction(pin, GPIO_MODE_INPUT_OUTPUT_OD);
}

static void on_line(const char *line)
{
    wires_line(line);
    if (strcmp(line, "c5rx ready") == 0) {
        c5_tx_attach();
        s_c5rx = true;
        xSemaphoreGive(s_ready);
    } else if (strncmp(line, "ok", 2) == 0 || strncmp(line, "err", 3) == 0) {
        strlcpy(s_reply_line, line, sizeof s_reply_line);
        xSemaphoreGive(s_reply);
    }
}

static void split_lines(const uint8_t *buf, size_t n)
{
    static char line[96];
    static size_t len;
    for (size_t i = 0; i < n; ++i) {
        if (buf[i] == '\n' || buf[i] == '\r') {
            line[len] = '\0';
            if (len > 0) on_line(line);
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
        if (bridge_active()) {
            bridge_c5_bytes(buf, n);
            continue;
        }
        if (s_log) host_write(buf, n);
        split_lines(buf, n);
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
    s_ready = xSemaphoreCreateBinary();
    s_reply = xSemaphoreCreateBinary();
    xTaskCreate(relay_task, "c5_relay", 3072, NULL, 5, NULL);
}

void c5_tx_attach(void)
{
    ESP_ERROR_CHECK(uart_set_pin(C5_UART, PIN_C5_TX, PIN_C5_RX, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    /* Open-drain, so a C5 that is already running its app can't fight the P4.
     * gpio_set_direction would unroute the UART signal, so only flip the driver. */
    gpio_od_enable(PIN_C5_TX);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_PULLUP_ONLY);
}

void c5_tx_detach(void)
{
    s_c5rx = false;
    gpio_set_direction(PIN_C5_TX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_FLOATING);
}

bool c5_run_c5rx(uint32_t timeout_ms)
{
    xSemaphoreTake(s_ready, 0);
    c5_run();
    return xSemaphoreTake(s_ready, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

bool c5_request(const char *cmd, char *reply, size_t reply_len, uint32_t timeout_ms)
{
    if (!s_c5rx) {
        strlcpy(reply, "err c5rx not running", reply_len);
        return false;
    }
    xSemaphoreTake(s_reply, 0);
    uart_write_bytes(C5_UART, cmd, strlen(cmd));
    uart_write_bytes(C5_UART, "\n", 1);
    if (xSemaphoreTake(s_reply, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        strlcpy(reply, "err no reply", reply_len);
        return false;
    }
    strlcpy(reply, s_reply_line, reply_len);
    return strncmp(reply, "ok", 2) == 0;
}

void c5_hold(void)
{
    c5_tx_detach();
    gpio_set_level(PIN_C5_EN, 0);
}

void c5_run(void)
{
    c5_tx_detach();
    gpio_set_level(PIN_C5_BOOT, 1);
    gpio_set_level(PIN_C5_EN, 0);
    vTaskDelay(pdMS_TO_TICKS(EN_LOW_MS));
    gpio_set_level(PIN_C5_EN, 1);
}

void c5_download(void)
{
    c5_tx_detach();
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
    } else if (strcmp(sub, "send") == 0 && argc > 2) {
        char cmd[96] = "";
        for (int i = 2; i < argc; ++i) {
            if (i > 2) strlcat(cmd, " ", sizeof cmd);
            strlcat(cmd, argv[i], sizeof cmd);
        }
        char reply[96];
        /* Starting the radio takes about a second. */
        c5_request(cmd, reply, sizeof reply, 5000);
        say("%s\n", reply);
        return;
    } else {
        say("c5 hold | run | dl | log on|off | send <command>\n");
        return;
    }
    say("ok\n");
}
