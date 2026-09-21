/**
 * rhnode.c - RotorHazard timing node protocol over USB serial.
 *
 * Mirrors the upstream node firmware closely enough for a server to run a
 * race against it: a 255-sample median at 1 kHz, enter/exit crossing
 * detection, a lap whose time is the middle of the peak, and the queue of
 * peaks and nadirs the server draws its RSSI graph from.
 */

#include "rhnode.h"

#include <string.h>

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "rf.h"
#include "rx5808.h"

#define NODE_API_LEVEL 36
#define API_VERIFY     0x25

#define READ_ADDRESS         0x00
#define READ_FREQUENCY       0x03
#define READ_LAP_STATS       0x05
#define READ_LAP_PASS_STATS  0x0D
#define READ_LAP_EXTREMUMS   0x0E
#define READ_RHFEAT_FLAGS    0x11
#define READ_REVISION_CODE   0x22
#define READ_NODE_RSSI_PEAK  0x23
#define READ_NODE_RSSI_NADIR 0x24
#define READ_ENTER_AT_LEVEL  0x31
#define READ_EXIT_AT_LEVEL   0x32
#define READ_TIME_MILLIS     0x33
#define READ_MULTINODE_COUNT 0x39
#define READ_CURNODE_INDEX   0x3A
#define READ_NODE_SLOTIDX    0x3C
#define READ_FW_VERSION      0x3D
#define READ_FW_BUILDDATE    0x3E
#define READ_FW_BUILDTIME    0x3F
#define READ_FW_PROCTYPE     0x40
#define WRITE_FREQUENCY      0x51
#define WRITE_ENTER_AT_LEVEL 0x71
#define WRITE_EXIT_AT_LEVEL  0x72
#define SEND_STATUS_MESSAGE  0x75
#define FORCE_END_CROSSING   0x78
#define RESET_PAIRED_NODE    0x79
#define WRITE_CURNODE_INDEX  0x7A
#define JUMP_TO_BOOTLOADER   0x7E

#define LAPSTATS_FLAG_CROSSING 0x01
#define LAPSTATS_FLAG_PEAK     0x02

#define MEDIAN_SAMPLES 255
#define FW_TEXT_BLOCK  16
#define HISTORY_DEPTH  10
#define MAX_DURATION   0xffffu

/* A turning point of the filtered signal: where it stopped rising (a peak) or
 * stopped falling (a nadir), and how long it held there. */
typedef struct {
    uint8_t rssi;
    uint32_t first_ms;
    uint16_t duration;
} extremum_t;

/* Oldest entry is dropped when full, as upstream's CircularBuffer does. */
typedef struct {
    extremum_t e[HISTORY_DEPTH];
    uint8_t head, count;
} history_queue_t;

typedef struct {
    uint8_t ring[MEDIAN_SAMPLES];
    uint16_t hist[256];
    uint16_t ring_pos;
    uint16_t filled;

    uint8_t rssi;
    uint8_t enter_at, exit_at;
    uint8_t node_peak, node_nadir;
    uint8_t pass_nadir;
    bool crossing;

    uint8_t pass_peak;
    uint32_t peak_first_ms, peak_last_ms;

    uint8_t lap;
    uint32_t lap_ms;
    uint8_t lap_peak, lap_nadir;

    uint16_t freq_mhz;

    uint8_t last_rssi;
    int8_t direction; /* sign of the last nonzero change; 0 before any */
    extremum_t peak, nadir;
    history_queue_t peaks, nadirs;
} node_t;

static node_t s_nodes[RHNODE_COUNT];
static uint8_t s_cur_node;
static bool s_active;
static uint32_t s_now_ms;
static uint16_t s_loop_us = 1000;
/* The command task reads the history queues while the measurement task,
 * which it preempts, is appending to them. */
static portMUX_TYPE s_history_lock = portMUX_INITIALIZER_UNLOCKED;

/* Command in progress, while its payload and checksum arrive. */
static uint8_t s_cmd;
static uint8_t s_payload[8];
static uint8_t s_payload_len, s_payload_want;

/* The server reads each node's frequency before it writes one, and rejects
 * zero, so start from what the receivers are already tuned to. */
static void node_init(node_t *n, uint8_t index)
{
    n->freq_mhz = index == RHNODE_C5 ? rf_get_freq() : rx5808_get_freq();
    n->enter_at = 96;
    n->exit_at = 80;
    n->node_nadir = 255;
    n->pass_nadir = 255;
    n->lap_nadir = 255;
}

static void queue_push(history_queue_t *q, extremum_t e)
{
    q->e[(q->head + q->count) % HISTORY_DEPTH] = e;
    if (q->count < HISTORY_DEPTH) q->count++;
    else q->head = (q->head + 1) % HISTORY_DEPTH;
}

static extremum_t queue_pop(history_queue_t *q)
{
    extremum_t e = q->e[q->head];
    q->head = (q->head + 1) % HISTORY_DEPTH;
    q->count--;
    return e;
}

static void push_extremum(history_queue_t *q, const extremum_t *e)
{
    portENTER_CRITICAL(&s_history_lock);
    queue_push(q, *e);
    portEXIT_CRITICAL(&s_history_lock);
}

static void start_extremum(extremum_t *e, uint8_t rssi, uint32_t t)
{
    e->rssi = rssi;
    e->first_ms = t;
    e->duration = 0;
}

static void extend_extremum(extremum_t *e, uint32_t t)
{
    uint32_t d = t - e->first_ms;
    e->duration = d > MAX_DURATION ? MAX_DURATION : (uint16_t)d;
}

/* A turning point is only final once the signal moves the other way. Flat
 * steps extend the current one, so a staircase ramp yields one extremum
 * rather than one per step. */
static void update_history(node_t *n, uint8_t rssi, uint32_t t)
{
    int change = (int)rssi - (int)n->last_rssi;
    n->last_rssi = rssi;

    if (change > 0) {
        if (n->direction < 0) push_extremum(&n->nadirs, &n->nadir);
        start_extremum(&n->peak, rssi, t);
        n->direction = 1;
    } else if (change < 0) {
        if (n->direction > 0) push_extremum(&n->peaks, &n->peak);
        start_extremum(&n->nadir, rssi, t);
        n->direction = -1;
    } else if (n->direction > 0) {
        extend_extremum(&n->peak, t);
    } else if (n->direction < 0) {
        extend_extremum(&n->nadir, t);
    }
}

static uint8_t median_add(node_t *n, uint8_t value)
{
    if (n->filled == MEDIAN_SAMPLES) n->hist[n->ring[n->ring_pos]]--;
    else n->filled++;
    n->ring[n->ring_pos] = value;
    n->hist[value]++;
    n->ring_pos = (n->ring_pos + 1) % MEDIAN_SAMPLES;

    uint16_t half = n->filled / 2, cum = 0;
    for (int b = 0; b < 256; ++b) {
        cum += n->hist[b];
        if (cum > half) return (uint8_t)b;
    }
    return 0;
}

void rhnode_feed(uint8_t node, uint8_t rssi, uint32_t now_ms)
{
    if (node >= RHNODE_COUNT) return;
    node_t *n = &s_nodes[node];
    s_now_ms = now_ms;

    uint8_t filtered = median_add(n, rssi);
    if (n->filled < MEDIAN_SAMPLES) return;
    if (n->filled == MEDIAN_SAMPLES && n->rssi == 0 && n->direction == 0) n->last_rssi = filtered;
    n->rssi = filtered;

    /* The filtered value describes the middle of its window, so lap times
     * are not pushed late by the filter's length. */
    uint32_t t = now_ms - MEDIAN_SAMPLES / 2;
    update_history(n, filtered, t);

    if (filtered > n->node_peak) n->node_peak = filtered;
    if (filtered < n->node_nadir) n->node_nadir = filtered;

    if (!n->crossing && filtered >= n->enter_at) {
        n->crossing = true;
        n->pass_peak = 0;
        n->pass_nadir = 255;
    } else if (n->crossing && filtered < n->exit_at) {
        n->crossing = false;
        n->lap++;
        n->lap_ms = (n->peak_first_ms + n->peak_last_ms) / 2;
        n->lap_peak = n->pass_peak;
        n->lap_nadir = n->pass_nadir;
    }

    if (n->crossing) {
        if (filtered > n->pass_peak) {
            n->pass_peak = filtered;
            n->peak_first_ms = t;
            n->peak_last_ms = t;
        } else if (filtered == n->pass_peak) {
            n->peak_last_ms = t;
        }
    } else if (filtered < n->pass_nadir) {
        n->pass_nadir = filtered;
    }
}

bool rhnode_active(void)
{
    return s_active;
}

void rhnode_set_loop_us(uint32_t us)
{
    s_loop_us = us > 0xffff ? 0xffff : (uint16_t)us;
}

bool rhnode_is_command_byte(uint8_t b)
{
    switch (b) {
    case READ_ADDRESS: case READ_FREQUENCY: case READ_LAP_STATS:
    case READ_LAP_PASS_STATS: case READ_LAP_EXTREMUMS: case READ_RHFEAT_FLAGS:
    case READ_REVISION_CODE: case READ_NODE_RSSI_PEAK: case READ_NODE_RSSI_NADIR:
    case READ_ENTER_AT_LEVEL: case READ_EXIT_AT_LEVEL: case READ_TIME_MILLIS:
    case READ_MULTINODE_COUNT: case READ_CURNODE_INDEX: case READ_NODE_SLOTIDX:
    case READ_FW_VERSION: case READ_FW_BUILDDATE: case READ_FW_BUILDTIME:
    case READ_FW_PROCTYPE: case WRITE_FREQUENCY: case WRITE_ENTER_AT_LEVEL:
    case WRITE_EXIT_AT_LEVEL: case SEND_STATUS_MESSAGE: case FORCE_END_CROSSING:
    case RESET_PAIRED_NODE: case WRITE_CURNODE_INDEX: case JUMP_TO_BOOTLOADER:
        return true;
    default:
        return false;
    }
}

static uint8_t write_payload_size(uint8_t cmd)
{
    switch (cmd) {
    case WRITE_FREQUENCY: case SEND_STATUS_MESSAGE:
        return 2;
    case WRITE_ENTER_AT_LEVEL: case WRITE_EXIT_AT_LEVEL: case WRITE_CURNODE_INDEX:
    case FORCE_END_CROSSING: case RESET_PAIRED_NODE: case JUMP_TO_BOOTLOADER:
        return 1;
    default:
        return 0;
    }
}

/* Replies carry a checksum over the payload only, as the server expects. */
static void reply(const uint8_t *data, size_t len)
{
    uint8_t out[FW_TEXT_BLOCK + 1];
    if (len > FW_TEXT_BLOCK) return;
    memcpy(out, data, len);
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    out[len] = sum;
    usb_serial_jtag_write_bytes(out, len + 1, pdMS_TO_TICKS(50));
}

static void reply8(uint8_t v) { reply(&v, 1); }

static void reply16(uint16_t v)
{
    const uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    reply(b, 2);
}

static void reply32(uint32_t v)
{
    const uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v };
    reply(b, 4);
}

static void reply_text(const char *s)
{
    uint8_t block[FW_TEXT_BLOCK] = { 0 };
    strncpy((char *)block, s, FW_TEXT_BLOCK);
    reply(block, FW_TEXT_BLOCK);
}

static void fill_pass_stats(node_t *n, uint8_t *b)
{
    uint16_t since = (uint16_t)(s_now_ms - n->lap_ms);
    b[0] = n->lap;
    b[1] = (uint8_t)(since >> 8);
    b[2] = (uint8_t)since;
    b[3] = n->rssi;
    b[4] = n->node_peak;
    b[5] = n->lap_peak;
    b[6] = (uint8_t)(s_loop_us >> 8);
    b[7] = (uint8_t)s_loop_us;
}

/* Sends the oldest queued turning point, peak or nadir, and removes it. The
 * server stitches these back into the RSSI graph. */
static void fill_extremums(node_t *n, uint8_t *b)
{
    memset(b, 0, 8);
    b[0] = n->crossing ? LAPSTATS_FLAG_CROSSING : 0;
    b[1] = n->lap_nadir;
    b[2] = n->node_nadir;

    portENTER_CRITICAL(&s_history_lock);
    bool has_peak = n->peaks.count > 0, has_nadir = n->nadirs.count > 0;
    bool send_peak = has_peak &&
        (!has_nadir || n->peaks.e[n->peaks.head].first_ms < n->nadirs.e[n->nadirs.head].first_ms);
    extremum_t e = { 0 };
    if (send_peak) {
        e = queue_pop(&n->peaks);
        b[0] |= LAPSTATS_FLAG_PEAK;
    } else if (has_nadir) {
        e = queue_pop(&n->nadirs);
    }
    portEXIT_CRITICAL(&s_history_lock);

    if (send_peak || has_nadir) {
        uint16_t since = (uint16_t)(s_now_ms - e.first_ms);
        b[3] = e.rssi;
        b[4] = (uint8_t)(since >> 8);
        b[5] = (uint8_t)since;
        b[6] = (uint8_t)(e.duration >> 8);
        b[7] = (uint8_t)e.duration;
    }
}

static void handle_read(uint8_t cmd)
{
    node_t *n = &s_nodes[s_cur_node];
    uint8_t buf[16];

    switch (cmd) {
    case READ_ADDRESS: reply8(0); break;
    case READ_FREQUENCY: reply16(n->freq_mhz); break;
    case READ_LAP_STATS:
        fill_pass_stats(n, buf);
        fill_extremums(n, buf + 8);
        reply(buf, 16);
        break;
    case READ_LAP_PASS_STATS:
        fill_pass_stats(n, buf);
        reply(buf, 8);
        break;
    case READ_LAP_EXTREMUMS:
        fill_extremums(n, buf);
        reply(buf, 8);
        break;
    case READ_ENTER_AT_LEVEL: reply8(n->enter_at); break;
    case READ_EXIT_AT_LEVEL: reply8(n->exit_at); break;
    case READ_REVISION_CODE: reply16((API_VERIFY << 8) + NODE_API_LEVEL); break;
    case READ_NODE_RSSI_PEAK: reply8(n->node_peak); break;
    case READ_NODE_RSSI_NADIR: reply8(n->node_nadir); break;
    case READ_TIME_MILLIS: reply32(s_now_ms); break;
    case READ_RHFEAT_FLAGS: reply16(0); break;
    case READ_MULTINODE_COUNT: reply8(RHNODE_COUNT); break;
    case READ_CURNODE_INDEX: reply8(s_cur_node); break;
    case READ_NODE_SLOTIDX: reply8(s_cur_node); break;
    case READ_FW_VERSION: reply_text("C5RSSI 1.0"); break;
    case READ_FW_BUILDDATE: reply_text(__DATE__); break;
    case READ_FW_BUILDTIME: reply_text(__TIME__); break;
    case READ_FW_PROCTYPE: reply_text("ESP32C5"); break;
    default: break;
    }
}

static void handle_write(uint8_t cmd, const uint8_t *p)
{
    node_t *n = &s_nodes[s_cur_node];

    switch (cmd) {
    case WRITE_FREQUENCY: {
        uint16_t mhz = (uint16_t)((p[0] << 8) | p[1]);
        n->freq_mhz = mhz;
        if (mhz >= 5000u) {
            if (s_cur_node == RHNODE_C5) rf_set_freq(mhz);
            else rx5808_set_freq(mhz);
        }
        break;
    }
    case WRITE_ENTER_AT_LEVEL: n->enter_at = p[0]; break;
    case WRITE_EXIT_AT_LEVEL: n->exit_at = p[0]; break;
    case WRITE_CURNODE_INDEX:
        if (p[0] < RHNODE_COUNT) s_cur_node = p[0];
        break;
    case FORCE_END_CROSSING: n->crossing = false; break;
    default: break;
    }
}

bool rhnode_rx_byte(uint8_t b)
{
    if (!s_active) {
        for (int i = 0; i < RHNODE_COUNT; ++i) node_init(&s_nodes[i], (uint8_t)i);
        /* Log lines would land in the middle of binary replies. */
        esp_log_level_set("*", ESP_LOG_NONE);
        s_active = true;
    }

    if (s_payload_want) {
        s_payload[s_payload_len++] = b;
        if (s_payload_len < s_payload_want + 1u) return true;

        uint8_t sum = 0;
        for (int i = 0; i < s_payload_want; ++i) sum += s_payload[i];
        if (sum == s_payload[s_payload_want]) handle_write(s_cmd, s_payload);
        s_payload_want = 0;
        s_payload_len = 0;
        return true;
    }

    s_cmd = b;
    s_payload_want = write_payload_size(b);
    s_payload_len = 0;
    if (!s_payload_want) handle_read(b);
    return true;
}
