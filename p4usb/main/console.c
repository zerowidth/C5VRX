#include "console.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "soc/lp_system_reg.h"
#include "soc/uart_pins.h"

#include "bridge.h"
#include "c5.h"
#include "census.h"
#include "decode.h"
#include "iq.h"
#include "link.h"
#include "usb.h"
#include "video.h"
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
    static const char *const reasons[] = {
        [ESP_RST_POWERON] = "power-on", [ESP_RST_SW] = "restart",      [ESP_RST_PANIC] = "panic",
        [ESP_RST_INT_WDT] = "interrupt watchdog", [ESP_RST_TASK_WDT] = "task watchdog",
        [ESP_RST_WDT] = "watchdog", [ESP_RST_BROWNOUT] = "brownout",
    };
    esp_reset_reason_t r = esp_reset_reason();
    const char *reason = r < sizeof reasons / sizeof reasons[0] && reasons[r] ? reasons[r] : "other";
    /* The panic dump itself only reaches the USB-C, so name the cause here. */
    say("p4usb rev v%d.%d uptime %lld ms, last reset: %s (%d)\n", chip.revision / 100, chip.revision % 100,
        esp_timer_get_time() / 1000, reason, (int)r);
    say("internal RAM free %u (largest block %u), PSRAM free %u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    bridge_info();
    usb_console_info();
}

void console_reboot(bool download)
{
    /* The ROM loader then listens on UART0 and the high-speed USB port. */
    if (download) REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);
    esp_restart();
}

static void cmd_reboot(int argc, char **argv)
{
    say("rebooting\n");
    vTaskDelay(pdMS_TO_TICKS(50));
    console_reboot(argc > 1 && strcmp(argv[1], "download") == 0);
}

static void cmd_channel(int argc, char **argv)
{
    unsigned mhz;
    if (argc > 1 && !decode_start(argv[1])) {
        say("err %s\n", argv[1]);
        return;
    }
    const char *chan = decode_channel(&mhz);
    say("channel %s %u\n", chan, mhz);
}

static void cmd_antenna(int argc, char **argv)
{
    char cmd[24] = "antenna", reply[96];
    if (argc > 1) snprintf(cmd, sizeof cmd, "antenna %s", argv[1]);
    /* The C5 saves the choice, and its `ok ` is dropped so the reply reads like channel's. */
    say("%s\n", c5_request(cmd, reply, sizeof reply, 1000) ? reply + 3 : reply);
}

static const command_t s_commands[] = {
    {"help", cmd_help, "list commands"},
    {"info", cmd_info, "chip revision, uptime and bridge counters"},
    {"census", census_run, "read every C5-facing pin with pull-down, then pull-up"},
    {"wires", wires_run, "reset the C5 and check its walking-ones boot test lane by lane"},
    {"link", link_run, "clocked counter test on each side's lanes, optionally at one MHz"},
    {"iq", iq_command, "capture two fields of I/Q; start [channel] tunes the C5 and picks a clock edge; dump sends the capture"},
    {"decode", decode_command, "on [channel] tunes the C5 and decodes its video into the camera; off stops; alone shows lock and load; luma plain|soft|peak sets the luma filter, clicks hold|gray|off whether a line with an FM click takes the color above, none or its own, and tnr auto|LEVELS the noise reduction over fields; bench times the demodulator"},
    {"channel", cmd_channel, "channel [BAND+N | MHZ] retunes (bands R A B E F L, 5180-5945 MHz); alone prints channel and MHz"},
    {"antenna", cmd_antenna, "antenna [int | ext] picks the C5's on-board antenna or its U.FL socket, and is kept across restarts; alone prints the one in use"},
    {"video", video_command, "frame and JPEG stats; grab prints the newest JPEG as base64; quality N sets the JPEG quality's ceiling"},
    {"reboot", cmd_reboot, "restart the P4; reboot download enters its ROM loader"},
    {"c5", c5_command, "hold, run or download-reset the C5; log on|off relays its UART; send talks to c5rx"},
};

static void cmd_help(int argc, char **argv)
{
    for (size_t i = 0; i < sizeof s_commands / sizeof s_commands[0]; ++i) {
        say("  %-8s %s\n", s_commands[i].name, s_commands[i].help);
    }
}

typedef enum { FROM_ANY, FROM_UART, FROM_USB } source_t;

/* Replies go where the last input came from; until then, to both. */
static volatile source_t s_source;

void console_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(HOST_UART, 4096, 4096, 0, NULL, 0));
    /* With the USB-C unplugged the RX line floats, and its noise would steal replies from USB. */
    gpio_set_pull_mode(U0RXD_GPIO_NUM, GPIO_PULLUP_ONLY);
}

bool host_is_uart(void)
{
    return s_source == FROM_UART;
}

void host_write(const void *data, size_t len)
{
    if (s_source != FROM_UART) usb_console_write(data, len);
    if (s_source != FROM_USB) uart_write_bytes(HOST_UART, data, len);
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
    size_t n = uart_read_some(HOST_UART, buf, len, 0);
    if (n) {
        s_source = FROM_UART;
        return n;
    }
    n = usb_console_read(buf, len, timeout_ms);
    if (n) s_source = FROM_USB;
    return n;
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
