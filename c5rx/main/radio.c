#include "radio.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/parlio_tx.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "hal/misc.h"
#include "esp_netif.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"
#include "soc/pcr_struct.h"

#include "lanes.h"

/* The receiver setup follows main/rf.c, which documents each step. */
#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF 0x600a4d6cu
#define MAC_TXQ_STRIDE 0x10u
#define MAC_TXQ_ENABLE 0x80000000u
#define MAC_TXQ_COUNT 5u
#define DUMP_CTRL 0x600a9004u
#define DUMP_PTR_MODE 0x600a9008u
#define DUMP_FORMAT 0x600a9018u
#define FE_PATH 0x600a20b4u
#define FE_ENABLE 0x600a0800u
#define SOURCE_CTRL 0x600a08ccu
#define SOURCE_MUX 0x600a70b8u
#define MODEM_CLOCK 0x600a9c04u
#define HP_SRAM_USAGE 0x60095004u
#define CTRL_ENABLE 0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK 0x01fe0000u

#define DEFAULT_GAIN 52
#define BUS_HZ 80000000
#define CLOCK_PATTERN_BYTES 64

extern int lmac_stop_hw_txq(void);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_set_freq(uint16_t freq_mhz, int offset);
extern int phy_get_noise_floor(void);
extern int phy_get_rssi(void);
extern void phy_11p_set(int enable, int mode);

static const char BANDS[] = "RABEFL";
static const uint16_t CHANNEL_MHZ[6][8] = {
    {5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917},
    {5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725},
    {5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866},
    {5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945},
    {5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880},
    {5362, 5399, 5436, 5473, 5510, 5547, 5584, 5621},
};

/* Public 5 GHz centers; other frequencies are reached with phy_set_freq from the nearest one. */
static const struct {
    uint8_t channel;
    uint16_t mhz;
} WIFI_CENTERS[] = {
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720}, {149, 5745}, {153, 5765},
    {157, 5785}, {161, 5805}, {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};
#define MIN_MHZ 5180
#define MAX_MHZ 5945

static bool s_started;
static char s_channel[4];
static uint16_t s_mhz;
static uint8_t s_gain = DEFAULT_GAIN;
/* The PHY's 802.11p setting narrows the receive filter to about 12 MHz and leaves the P4 two clock positions
 * in three that read the lanes steady, against one in three without it. */
static bool s_11p = true;
static parlio_tx_unit_handle_t s_clock;
static uint8_t *s_clock_pattern;

static esp_err_t lock_rx_only(void)
{
    lmac_stop_hw_txq();
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q) REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    for (unsigned q = 0; q < MAC_TXQ_COUNT; ++q) {
        if (REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE) return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

/* Keeps the ADC streaming onto MODEM_DIAG with no packets present. */
static void continuous_modem(void)
{
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;
    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;

    REG32(DUMP_FORMAT) = (REG32(DUMP_FORMAT) & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = ((REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u) | 0x01000000u;
    REG32(DUMP_PTR_MODE) = (REG32(DUMP_PTR_MODE) & ~SELECTOR_MASK) | TX_START_SELECT;

    uint32_t ctrl = REG32(DUMP_CTRL);
    ctrl &= ~(CTRL_ENABLE | 0x00080000u | 0x00040000u);
    ctrl |= CTRL_DUMP_FIRST;
    ctrl = (ctrl & ~0x0001ffffu) | 16384u;
    REG32(DUMP_CTRL) = ctrl;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    REG32(DUMP_CTRL) = ctrl | CTRL_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
}

/* The vendor packet AGC hunts on analog FM, so gain stays fixed. */
static void receive_state(void)
{
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(1);
    phy_force_rx_gain(true, s_gain);
    phy_11p_set(s_11p, 0);
}

static bool s_wifi_up;

static esp_err_t wifi_up(void)
{
    if (s_wifi_up) return ESP_OK;
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;
    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    /* Otherwise the driver powers the PHY down between scans and MODEM_DIAG stops. */
    cfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK) return err;
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;
    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_11AX,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK) return err;
    /* BW40 gives the full video bandwidth; there is no BW20 fallback. */
    wifi_bandwidths_t bw = {.ghz_2g = WIFI_BW20, .ghz_5g = WIFI_BW40};
    if ((err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bw)) != ESP_OK) return err;
    s_wifi_up = true;
    return ESP_OK;
}

static esp_err_t start(void)
{
    esp_err_t err = wifi_up();
    if (err != ESP_OK) return err;
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = {.filter_mask = 0};
    esp_wifi_set_promiscuous_filter(&filter);
    if ((err = lock_rx_only()) != ESP_OK) return err;
    s_started = true;
    return ESP_OK;
}

/* A scan with the vendor AGC still on, before the first tune turns it off for good: each access point's
 * RSSI is an absolute level to set the forced-gain I/Q against. */
esp_err_t radio_scan(void)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
    esp_err_t err = wifi_up();
    if (err != ESP_OK) return err;
    return esp_wifi_scan_start(NULL, false);
}

bool radio_scan_result(unsigned index, unsigned *channel, int *rssi, unsigned *bssid)
{
    static wifi_ap_record_t aps[24];
    static uint16_t count;
    if (index == 0) {
        count = sizeof aps / sizeof aps[0];
        if (esp_wifi_scan_get_ap_records(&count, aps) != ESP_OK) count = 0;
    }
    if (index >= count) return false;
    *channel = aps[index].primary;
    *rssi = aps[index].rssi;
    *bssid = aps[index].bssid[4] << 8 | aps[index].bssid[5];
    return true;
}

static bool parse_channel(const char *name, char *label, uint16_t *mhz)
{
    const char *band = strchr(BANDS, toupper((unsigned char)name[0]));
    if (name[0] && band && name[1] >= '1' && name[1] <= '8' && !name[2]) {
        *mhz = CHANNEL_MHZ[band - BANDS][name[1] - '1'];
        label[0] = *band;
        label[1] = name[1];
        label[2] = '\0';
        return true;
    }
    char *end;
    long v = strtol(name, &end, 10);
    if (*end || v < MIN_MHZ || v > MAX_MHZ) return false;
    *mhz = (uint16_t)v;
    label[0] = '\0';
    return true;
}

esp_err_t radio_tune(const char *name)
{
    char label[4];
    uint16_t mhz;
    if (!parse_channel(name, label, &mhz) || mhz < MIN_MHZ || mhz > MAX_MHZ) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    if (!s_started && (err = start()) != ESP_OK) return err;

    size_t best = 0;
    for (size_t i = 1; i < sizeof WIFI_CENTERS / sizeof WIFI_CENTERS[0]; ++i) {
        if (abs(mhz - WIFI_CENTERS[i].mhz) < abs(mhz - WIFI_CENTERS[best].mhz)) best = i;
    }
    if ((err = esp_wifi_set_channel(WIFI_CENTERS[best].channel, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;
    if (mhz != WIFI_CENTERS[best].mhz) phy_set_freq(mhz, 0);
    continuous_modem();
    receive_state();
    strlcpy(s_channel, label, sizeof s_channel);
    s_mhz = mhz;
    return ESP_OK;
}

const char *radio_channel(void)
{
    return s_channel;
}

uint16_t radio_mhz(void)
{
    return s_mhz;
}

void radio_set_gain(uint8_t index)
{
    s_gain = index;
    if (s_started) phy_force_rx_gain(true, index);
}

void radio_11p(int enable, int mode)
{
    s_11p = enable;
    if (s_started) phy_11p_set(enable, mode);
}

uint8_t radio_gain(void)
{
    return s_gain;
}

bool radio_rssi(int *dbm)
{
    if (!s_started) return false;
    *dbm = phy_get_rssi();
    return *dbm >= -140 && *dbm <= 10;
}

bool radio_noise_floor(int *dbm)
{
    if (!s_started) return false;
    *dbm = phy_get_noise_floor();
    return *dbm >= -140 && *dbm <= -20;
}

/* PARLIO TX with no data pins, looping, is a continuous clock from the same PLL as the modem. */
static esp_err_t start_clock(uint32_t hz)
{
    if (!s_clock_pattern) {
        s_clock_pattern = heap_caps_calloc(1, CLOCK_PATTERN_BYTES, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!s_clock_pattern) return ESP_ERR_NO_MEM;
    }
    parlio_tx_unit_config_t cfg = {
        .clk_src = PARLIO_CLK_SRC_DEFAULT,
        .clk_in_gpio_num = -1,
        .output_clk_freq_hz = hz,
        .data_width = 8,
        .clk_out_gpio_num = LANES[LANE_CLK].gpio,
        .valid_gpio_num = -1,
        .trans_queue_depth = 1,
        .max_transfer_size = CLOCK_PATTERN_BYTES,
        .shift_edge = PARLIO_SHIFT_EDGE_POS,
        .bit_pack_order = PARLIO_BIT_PACK_ORDER_LSB,
    };
    for (int k = 0; k < 8; ++k) cfg.data_gpio_nums[k] = -1;
    esp_err_t err = parlio_new_tx_unit(&cfg, &s_clock);
    if (err != ESP_OK) return err;
    if ((err = parlio_tx_unit_enable(s_clock)) != ESP_OK) return err;
    const parlio_transmit_config_t tx = {.flags.loop_transmission = 1};
    return parlio_tx_unit_transmit(s_clock, s_clock_pattern, CLOCK_PATTERN_BYTES * 8, &tx);
}

esp_err_t radio_iq_start(int every, int q_top, int i_top)
{
    if (every < 2 || every > 16) return ESP_ERR_INVALID_ARG;
    if (!s_started) return ESP_ERR_INVALID_STATE;
    if (q_top < 6 || q_top > 31 || i_top < 6 || i_top > 31) return ESP_ERR_INVALID_ARG;
    radio_iq_stop();
    for (int k = 0; k < 7; ++k) {
        const gpio_num_t q = LANES[1 + k].gpio, i = LANES[8 + k].gpio;
        gpio_reset_pin(q);
        gpio_reset_pin(i);
        gpio_set_direction(q, GPIO_MODE_OUTPUT);
        gpio_set_direction(i, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(q, MODEM_DIAG0_IDX + q_top - k, false, false);
        esp_rom_gpio_connect_out_signal(i, MODEM_DIAG0_IDX + i_top - k, false, false);
    }
    esp_err_t err = start_clock(BUS_HZ / every);
    if (err != ESP_OK) radio_iq_stop();
    return err;
}

/* Runs the clock one tick of its 240 MHz slow for a while, which moves its edge a tick later against the
 * bus for each period that passes. The bus changes every 3 ticks, and where the edge falls against it is
 * settled at reset; the P4 slips it until it reads the lanes steady. */
void radio_clock_slip(uint32_t us)
{
    uint32_t div = PCR.parl_clk_tx_conf.parl_clk_tx_div_num;
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_tx_conf, parl_clk_tx_div_num, div + 1);
    if (us) esp_rom_delay_us(us);
    HAL_FORCE_MODIFY_U32_REG_FIELD(PCR.parl_clk_tx_conf, parl_clk_tx_div_num, div);
}

void radio_iq_stop(void)
{
    if (s_clock) {
        parlio_tx_unit_disable(s_clock);
        parlio_del_tx_unit(s_clock);
        s_clock = NULL;
    }
    lanes_drive_low();
}

/* Lab only: frames heard with the vendor AGC on, by transmitter, to set the forced-gain I/Q against. */
#define SNIFF_SLOTS 16
static struct {
    uint16_t key, count, beacons;
    int32_t rssi, noise, len;
} s_sniff[SNIFF_SLOTS];

static void sniffed(void *buf, wifi_promiscuous_pkt_type_t type)
{
    const wifi_promiscuous_pkt_t *pkt = buf;
    unsigned len = pkt->rx_ctrl.sig_len;
    /* Frames too short for a second address are acknowledgements and the like. */
    uint16_t key = len >= 20 ? pkt->payload[14] << 8 | pkt->payload[15] : 0xffff;
    for (int i = 0; i < SNIFF_SLOTS; ++i) {
        if (s_sniff[i].count && s_sniff[i].key != key) continue;
        s_sniff[i].key = key;
        ++s_sniff[i].count;
        s_sniff[i].beacons += len >= 20 && pkt->payload[0] == 0x80;
        s_sniff[i].rssi += pkt->rx_ctrl.rssi;
        s_sniff[i].noise += pkt->rx_ctrl.noise_floor;
        s_sniff[i].len += len;
        return;
    }
}

esp_err_t radio_sniff(uint8_t channel)
{
    if (s_started) return ESP_ERR_INVALID_STATE;
    esp_err_t err = wifi_up();
    if (err != ESP_OK) return err;
    memset(s_sniff, 0, sizeof s_sniff);
    if ((err = esp_wifi_set_promiscuous_rx_cb(sniffed)) != ESP_OK) return err;
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    return esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

bool radio_sniff_result(unsigned i, unsigned *key, unsigned *count, unsigned *beacons, int *rssi, int *noise, int *len)
{
    if (i >= SNIFF_SLOTS || !s_sniff[i].count) return false;
    *key = s_sniff[i].key;
    *count = s_sniff[i].count;
    *beacons = s_sniff[i].beacons;
    *rssi = s_sniff[i].rssi / s_sniff[i].count;
    *noise = s_sniff[i].noise / s_sniff[i].count;
    *len = s_sniff[i].len / s_sniff[i].count;
    return true;
}

typedef long (*phy_fn_t)(uint32_t, uint32_t, uint32_t);
extern void phy_pbus_set_rxgain(void);
extern void phy_pbus_rd(void);
extern void phy_pbus_xpd_rx_on(void);
extern void phy_read_hw_noisefloor(void);
extern void phy_check_sigrssi_en(void);
extern void phy_get_sigrssi(void);
extern void phy_pbus_debugmode(void);
extern void phy_pbus_workmode(void);
extern void phy_pbus_force_test(void);
extern void phy_enable_agc(void);

long radio_call(const char *name, uint32_t a, uint32_t b, uint32_t c)
{
    static const struct {
        const char *name;
        void (*fn)(void);
    } FNS[] = {
        {"pbus_set_rxgain", phy_pbus_set_rxgain}, {"pbus_rd", phy_pbus_rd}, {"pbus_xpd_rx_on", phy_pbus_xpd_rx_on},
        {"read_hw_noisefloor", phy_read_hw_noisefloor}, {"check_sigrssi_en", phy_check_sigrssi_en},
        {"get_sigrssi", phy_get_sigrssi}, {"pbus_debugmode", phy_pbus_debugmode}, {"pbus_workmode", phy_pbus_workmode},
        {"pbus_force_test", phy_pbus_force_test}, {"force_rx_gain", (void (*)(void))phy_force_rx_gain},
        {"enable_agc", phy_enable_agc}, {"disable_agc", (void (*)(void))phy_disable_agc},
        {"rfagc_disable", (void (*)(void))phy_rfagc_disable}, {"fbw_sel", (void (*)(void))phy_wifi_fbw_sel},
    };
    for (size_t i = 0; i < sizeof FNS / sizeof FNS[0]; ++i) {
        if (strcmp(name, FNS[i].name) == 0) return ((phy_fn_t)FNS[i].fn)(a, b, c);
    }
    return -1;
}
