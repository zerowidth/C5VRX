#include "console.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "bridge.h"
#include "c5.h"
#include "census.h"
#include "link.h"
#include "wires.h"

#define MAX_ARGS 8

typedef struct {
    const char *name;
    void (*run)(int argc, char **argv);
    const char *help;
} command_t;

static void cmd_help(int argc, char **argv);

static void cmd_info(int argc, char **argv)
{
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    say("p4usb rev v%d.%d uptime %lld ms\n", chip.revision / 100, chip.revision % 100,
        esp_timer_get_time() / 1000);
    bridge_info();
}

static const command_t s_commands[] = {
    {"help", cmd_help, "list commands"},
    {"info", cmd_info, "chip revision, uptime and bridge counters"},
    {"census", census_run, "read every C5-facing pin with pull-down, then pull-up"},
    {"wires", wires_run, "reset the C5 and check its walking-ones boot test lane by lane"},
    {"link", link_run, "clocked counter test on each side's lanes, optionally at one MHz"},
    {"c5", c5_command, "hold, run or download-reset the C5; log on|off relays its UART; send talks to c5rx"},
};

static void cmd_help(int argc, char **argv)
{
    for (size_t i = 0; i < sizeof s_commands / sizeof s_commands[0]; ++i) {
        say("  %-8s %s\n", s_commands[i].name, s_commands[i].help);
    }
}

void console_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(HOST_UART, 4096, 4096, 0, NULL, 0));
}

void host_write(const void *data, size_t len)
{
    uart_write_bytes(HOST_UART, data, len);
}

void say(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof buf - 1) n = sizeof buf - 1;
    if (n > 0) host_write(buf, n);
}

size_t uart_read_some(uart_port_t port, uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    /* uart_read_bytes waits for all len bytes, so block for one and take what else is buffered. */
    if (len == 0 || uart_read_bytes(port, buf, 1, pdMS_TO_TICKS(timeout_ms)) != 1) return 0;
    size_t more = 0;
    uart_get_buffered_data_len(port, &more);
    if (more > len - 1) more = len - 1;
    int n = more ? uart_read_bytes(port, buf + 1, more, 0) : 0;
    return 1 + (n > 0 ? (size_t)n : 0);
}

size_t host_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    return uart_read_some(HOST_UART, buf, len, timeout_ms);
}

static void run_line(char *line)
{
    char *argv[MAX_ARGS];
    int argc = 0;
    for (char *tok = strtok(line, " \t"); tok && argc < MAX_ARGS; tok = strtok(NULL, " \t")) {
        argv[argc++] = tok;
    }
    if (argc == 0) return;
    for (size_t i = 0; i < sizeof s_commands / sizeof s_commands[0]; ++i) {
        if (strcmp(argv[0], s_commands[i].name) == 0) {
            s_commands[i].run(argc, argv);
            return;
        }
    }
    say("unknown command '%s', try help\n", argv[0]);
}

void console_feed(uint8_t c)
{
    static char line[128];
    static size_t len;

    if (c == '\r' || c == '\n') {
        say("\n");
        line[len] = '\0';
        len = 0;
        run_line(line);
        say("> ");
    } else if (c == 0x08 || c == 0x7f) {
        if (len > 0) {
            --len;
            say("\b \b");
        }
    } else if (c >= 0x20 && c < 0x7f && len < sizeof line - 1) {
        line[len++] = (char)c;
        host_write(&c, 1);
    }
}
