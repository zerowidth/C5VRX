#include "bridge.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "c5.h"
#include "console.h"
#include "pins.h"

/* esptool's first SYNC frame starts with these bytes: SLIP end, then command 0x08. */
static const uint8_t SYNC_PREFIX[] = {0xc0, 0x00, 0x08, 0x24, 0x00};
static const char ROM_BANNER[] = "ESP-ROM:";
/* Long enough for a whole-chip erase, which is silent in both directions. */
#define IDLE_US (30 * 1000 * 1000)

static volatile bool s_active;
static volatile bool s_answered;
static volatile bool s_rebooted;
static volatile int64_t s_last_us;
static size_t s_sync_match;
static uint32_t s_sessions;
static volatile uint32_t s_to_c5;
static volatile uint32_t s_from_c5;
static size_t s_banner_match;

bool bridge_active(void)
{
    return s_active;
}

static void attach_tx(void)
{
    ESP_ERROR_CHECK(uart_set_pin(C5_UART, PIN_C5_TX, PIN_C5_RX, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    /* Open-drain, so a C5 that is already running its app can't fight the P4.
     * gpio_set_direction would unroute the UART signal, so only flip the driver. */
    gpio_od_enable(PIN_C5_TX);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_PULLUP_ONLY);
}

static void detach_tx(void)
{
    gpio_set_direction(PIN_C5_TX, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_C5_TX, GPIO_FLOATING);
}

static void start(void)
{
    c5_download();
    attach_tx();
    s_answered = false;
    s_rebooted = false;
    s_banner_match = 0;
    s_last_us = esp_timer_get_time();
    s_to_c5 = 0;
    s_from_c5 = 0;
    ++s_sessions;
    s_active = true;
}

static void stop(const char *why)
{
    s_active = false;
    detach_tx();
    say("\nbridge: %s\n> ", why);
}

bool bridge_host_bytes(const uint8_t *buf, size_t n)
{
    if (n == 0) return s_active;
    if (s_active) {
        s_last_us = esp_timer_get_time();
        uart_write_bytes(C5_UART, buf, n);
        s_to_c5 += n;
        return true;
    }
    for (size_t i = 0; i < n; ++i) {
        s_sync_match = buf[i] == SYNC_PREFIX[s_sync_match] ? s_sync_match + 1
                       : buf[i] == SYNC_PREFIX[0]          ? 1
                                                           : 0;
        if (s_sync_match == sizeof SYNC_PREFIX) {
            /* esptool resends SYNC until the ROM answers, so this frame can be dropped. */
            s_sync_match = 0;
            start();
            return true;
        }
    }
    return false;
}

void bridge_c5_bytes(const uint8_t *buf, size_t n)
{
    if (!s_active) return;
    s_last_us = esp_timer_get_time();
    s_from_c5 += n;
    for (size_t i = 0; i < n; ++i) {
        if (buf[i] == 0xc0) s_answered = true;
        /* The download-mode banner arrives before the first answer; a later one is a reboot. */
        if (!s_answered) continue;
        s_banner_match = buf[i] == ROM_BANNER[s_banner_match] ? s_banner_match + 1
                         : buf[i] == ROM_BANNER[0]            ? 1
                                                              : 0;
        if (s_banner_match == sizeof ROM_BANNER - 1) s_rebooted = true;
    }
}

void bridge_poll(void)
{
    if (!s_active) return;
    if (s_rebooted) {
        stop("C5 rebooted");
    } else if (esp_timer_get_time() - s_last_us > IDLE_US) {
        c5_run();
        stop("idle, C5 reset to run");
    }
}

void bridge_info(void)
{
    say("bridge %s, %lu session(s); last: %lu bytes to C5, %lu from C5%s\n",
        s_active ? "active" : "idle", (unsigned long)s_sessions, (unsigned long)s_to_c5,
        (unsigned long)s_from_c5, s_answered ? ", C5 answered" : "");
}
