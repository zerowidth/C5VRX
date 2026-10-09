#include "bridge.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

#include "c5.h"
#include "console.h"
#include "decode.h"

/* esptool's first SYNC frame starts with these bytes: SLIP end, then command 0x08. */
static const uint8_t SYNC_PREFIX[] = {0xc0, 0x00, 0x08, 0x24, 0x00};
#define CMD_CHANGE_BAUDRATE 0x0f
#define BRIDGE_BAUD 115200
/* A few bytes outside any SLIP frame mean the ROM is printing its boot banner. */
#define REBOOT_JUNK 4
/* esptool also discards what arrives during this window after a baud change. */
#define SETTLE_US (100 * 1000)
/* Long enough for a whole-chip erase, which is silent in both directions. */
#define IDLE_US (30 * 1000 * 1000)
/* esptool gives up after a few seconds, so an unanswered session should not hold the console. */
#define ANSWER_US (2 * 1000 * 1000)

static volatile bool s_active;
static volatile bool s_answered;
static volatile bool s_rebooted;
static volatile int64_t s_last_us;
static int64_t s_start_us;
static size_t s_sync_match;
static uint32_t s_sessions;
static volatile uint32_t s_to_c5;
static volatile uint32_t s_from_c5;
static uint32_t s_junk;
static int64_t s_settle_until_us;
static uint32_t s_baud = BRIDGE_BAUD;
static volatile uint32_t s_pending_baud;

typedef enum { SLIP_NONE, SLIP_END, SLIP_JUNK } slip_event_t;

/* Tracks frame boundaries and keeps the first bytes of each frame: direction, op, length,
 * checksum or value, then data. */
typedef struct {
    bool in;
    bool esc;
    size_t len;
    uint8_t head[16];
} slip_t;

static slip_t s_from_host;
static slip_t s_from_chip;

static slip_event_t slip_feed(slip_t *f, uint8_t b)
{
    if (b == 0xc0) {
        if (f->in && f->len > 0) {
            f->in = false;
            return SLIP_END;
        }
        *f = (slip_t){.in = true};
        return SLIP_NONE;
    }
    if (!f->in) return SLIP_JUNK;
    if (f->esc) {
        b = b == 0xdc ? 0xc0 : b == 0xdd ? 0xdb : b;
        f->esc = false;
    } else if (b == 0xdb) {
        f->esc = true;
        return SLIP_NONE;
    }
    if (f->len < sizeof f->head) f->head[f->len] = b;
    ++f->len;
    return SLIP_NONE;
}

static uint32_t le32(const uint8_t *p)
{
    return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static void set_baud(uint32_t baud)
{
    if (host_is_uart()) {
        uart_wait_tx_done(HOST_UART, pdMS_TO_TICKS(50));
        uart_set_baudrate(HOST_UART, baud);
    }
    uart_set_baudrate(C5_UART, baud);
    s_baud = baud;
    s_settle_until_us = esp_timer_get_time() + SETTLE_US;
}

bool bridge_active(void)
{
    return s_active;
}

static bool s_resume_decode;

static void start(void)
{
    /* The C5 stops exporting I/Q while it sits in its ROM loader. */
    s_resume_decode = decode_running();
    decode_stop();
    c5_download();
    c5_tx_attach();
    s_answered = false;
    s_rebooted = false;
    s_junk = 0;
    s_pending_baud = 0;
    s_from_host = (slip_t){0};
    s_from_chip = (slip_t){0};
    s_start_us = s_last_us = esp_timer_get_time();
    s_to_c5 = 0;
    s_from_c5 = 0;
    ++s_sessions;
    s_active = true;
}

static void stop(const char *why)
{
    s_active = false;
    c5_tx_detach();
    if (s_baud != BRIDGE_BAUD) set_baud(BRIDGE_BAUD);
    say("\nbridge: %s\n> ", why);
    /* An esptool that gave up leaves a queue of SYNCs behind, each of which would start a session, wait
     * out its timeout and start the decoder again, keeping the console busy for minutes. */
    uint8_t stale[64];
    while (host_read(stale, sizeof stale, 50)) {
    }
    if (s_resume_decode) decode_start(NULL);
}

bool bridge_host_bytes(const uint8_t *buf, size_t n)
{
    if (n == 0) return s_active;
    if (s_active) {
        s_last_us = esp_timer_get_time();
        for (size_t i = 0; i < n; ++i) {
            const slip_t *f = &s_from_host;
            if (slip_feed(&s_from_host, buf[i]) == SLIP_END && f->len >= 12 &&
                f->head[1] == CMD_CHANGE_BAUDRATE) {
                s_pending_baud = le32(&f->head[8]);
            }
        }
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
    s_last_us = esp_timer_get_time();
    s_from_c5 += n;
    host_write(buf, n);
    bool change = false;
    for (size_t i = 0; i < n; ++i) {
        const slip_t *f = &s_from_chip;
        slip_event_t ev = slip_feed(&s_from_chip, buf[i]);
        if (ev == SLIP_END) {
            s_answered = true;
            s_junk = 0;
            if (f->head[0] == 0x01 && f->head[1] == CMD_CHANGE_BAUDRATE && s_pending_baud) change = true;
        } else if (ev == SLIP_JUNK && s_answered && s_last_us > s_settle_until_us &&
                   ++s_junk >= REBOOT_JUNK) {
            /* Before the first answer this is the download-mode banner; after it, a reboot. */
            s_rebooted = true;
        }
    }
    /* The chip switches after sending its answer, and esptool after reading it. */
    if (change) {
        set_baud(s_pending_baud);
        s_pending_baud = 0;
    }
}

void bridge_poll(void)
{
    if (!s_active) return;
    if (s_rebooted) {
        stop("C5 rebooted");
    } else if (!s_answered && esp_timer_get_time() - s_start_us > ANSWER_US) {
        stop("C5 did not answer SYNC");
    } else if (esp_timer_get_time() - s_last_us > IDLE_US) {
        c5_run();
        stop("idle, C5 reset to run");
    }
}

void bridge_info(void)
{
    say("bridge %s at %lu baud, %lu session(s); last: %lu bytes to C5, %lu from C5%s\n",
        s_active ? "active" : "idle", (unsigned long)s_baud, (unsigned long)s_sessions,
        (unsigned long)s_to_c5, (unsigned long)s_from_c5, s_answered ? ", C5 answered" : "");
}
