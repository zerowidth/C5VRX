#include <stdio.h>
#include <string.h>
#include "rhnode.h"

unsigned char tx_buf[512];
int tx_len;
static unsigned tuned_c5, tuned_rx;
void rf_set_freq(uint16_t mhz) { tuned_c5 = mhz; }
void rx5808_set_freq(uint16_t mhz) { tuned_rx = mhz; }

static int fails;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static int read_cmd(unsigned char cmd, unsigned char *out, int expect)
{
    tx_len = 0;
    rhnode_rx_byte(cmd);
    CHECK(tx_len == expect + 1, "cmd 0x%02x returned %d bytes, want %d", cmd, tx_len, expect + 1);
    unsigned char sum = 0;
    for (int i = 0; i < tx_len - 1; i++) sum += tx_buf[i];
    CHECK(sum == tx_buf[tx_len - 1], "cmd 0x%02x bad checksum", cmd);
    memcpy(out, tx_buf, tx_len);
    return tx_len;
}

static void write_cmd(unsigned char cmd, const unsigned char *payload, int len)
{
    rhnode_rx_byte(cmd);
    unsigned char sum = 0;
    for (int i = 0; i < len; i++) { sum += payload[i]; rhnode_rx_byte(payload[i]); }
    rhnode_rx_byte(sum);
}

int main(void)
{
    unsigned char r[32];

    read_cmd(0x22, r, 2);
    CHECK(r[0] == 0x25 && r[1] == 36, "revision code %02x %02x", r[0], r[1]);
    read_cmd(0x39, r, 1);
    CHECK(r[0] == 2, "multinode count %d", r[0]);

    /* Tune each node through the server's own command. */
    unsigned char freq[2] = { 5732 >> 8, 5732 & 0xff };
    write_cmd(0x51, freq, 2);
    unsigned char idx = 1;
    write_cmd(0x7a, &idx, 1);
    read_cmd(0x3a, r, 1);
    CHECK(r[0] == 1, "current node %d", r[0]);
    unsigned char freq2[2] = { 5917 >> 8, 5917 & 0xff };
    write_cmd(0x51, freq2, 2);
    CHECK(tuned_c5 == 5732, "C5 tuned to %u", tuned_c5);
    CHECK(tuned_rx == 5917, "RX5808 tuned to %u", tuned_rx);
    idx = 0;
    write_cmd(0x7a, &idx, 1);

    unsigned char enter = 120, exit_at = 100;
    write_cmd(0x71, &enter, 1);
    write_cmd(0x72, &exit_at, 1);
    read_cmd(0x31, r, 1);
    CHECK(r[0] == 120, "enter level %d", r[0]);

    /* A pass: 2 s of noise, a 1 s rise to 200 peaking at t=3500 ms, back down. */
    uint32_t ms = 0;
    for (; ms < 2000; ms++) rhnode_feed(0, 60 + (ms % 3), ms);
    for (; ms < 3500; ms++) rhnode_feed(0, 60 + (ms - 2000) * 140 / 1500, ms);
    for (; ms < 5000; ms++) rhnode_feed(0, 200 - (ms - 3500) * 140 / 1500, ms);
    for (; ms < 7000; ms++) rhnode_feed(0, 60 + (ms % 3), ms);

    read_cmd(0x0d, r, 8);
    int lap = r[0], since = (r[1] << 8) | r[2], peak = r[5];
    printf("lap=%d since=%d ms rssi=%d node_peak=%d pass_peak=%d\n", lap, since, r[3], r[4], peak);
    CHECK(lap == 1, "expected exactly 1 lap, got %d", lap);
    CHECK(peak > 180, "pass peak %d should be near 200", peak);
    int lap_time = (int)ms - since;
    CHECK(lap_time > 3300 && lap_time < 3700, "lap time %d ms should be near the 3500 ms peak", lap_time);

    read_cmd(0x0e, r, 8);
    CHECK((r[0] & 1) == 0, "should not still be crossing");

    printf(fails ? "%d checks FAILED\n" : "all checks passed\n", fails);
    return fails != 0;
}
