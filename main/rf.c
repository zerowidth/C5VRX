/**
 * rf.c - ESP32-C5 Wi-Fi PHY as a receive-only tuner.
 *
 * Starts Wi-Fi in promiscuous mode with TX queues disabled and the vendor AGC
 * off, forces continuous modem sampling, and routes the top 4 bits of I and Q
 * from MODEM_DIAG to GPIO so PARLIO RX can capture them.
 */

#include "rf.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rom_gpio.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "soc/gpio_sig_map.h"

#define RF_MIN_MHZ 5180u
#define RF_MAX_MHZ 5950u

#define REG32(a)         (*(volatile uint32_t *)(uintptr_t)(a))
#define MAC_TXQ0_CONF    0x600a4d6cu
#define MAC_TXQ_STRIDE   0x10u
#define MAC_TXQ_ENABLE   0x80000000u
#define MAC_TXQ_COUNT    5u

/* Continuous modem front-end registers. Without these the ADC only clocks
 * I/Q onto MODEM_DIAG while an 802.11 packet is being received. */
#define DUMP_CTRL       0x600a9004u
#define DUMP_PTR_MODE   0x600a9008u
#define DUMP_FORMAT     0x600a9018u
#define FE_PATH         0x600a20b4u
#define FE_ENABLE       0x600a0800u
#define SOURCE_CTRL     0x600a08ccu
#define SOURCE_MUX      0x600a70b8u
#define MODEM_CLOCK     0x600a9c04u
#define CTRL_ENABLE     0x80000000u
#define CTRL_DUMP_FIRST 0x00020000u
#define TX_START_SELECT 0x00060000u
#define SELECTOR_MASK   0x01fe0000u
#define HP_SRAM_USAGE   0x60095004u

/* Waveshare ESP32-C5-Zero: low selects the on-board antenna, high the U.FL. */
#define ANTENNA_SEL_GPIO GPIO_NUM_26

/* Nothing is wired to these: each pad loops its own output back to its input.
 * They sit on the back pads and the left edge so the whole lower right edge
 * stays free for an RX5808 module, whose analog RSSI output needs a pin away
 * from lines switching at tens of MHz. */
const gpio_num_t rf_iq_pins[RF_IQ_LANES] = {
    GPIO_NUM_1, GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_23,
    GPIO_NUM_24, GPIO_NUM_5, GPIO_NUM_3, GPIO_NUM_4,
};

/* Only DIAG 6-9 = Q[9:6] and 16-19 = I[9:6] are hardware-verified. The 8-bit
 * layouts assume the pattern continues: DIAG n = Q[n], DIAG 10+n = I[n]. */
static const uint8_t s_layout_diag[RF_LAYOUT_COUNT][RF_IQ_LANES] = {
    [RF_LAYOUT_IQ4] = { 6, 7, 8, 9, 16, 17, 18, 19 },
    [RF_LAYOUT_Q8]  = { 2, 3, 4, 5, 6, 7, 8, 9 },
    [RF_LAYOUT_I8]  = { 12, 13, 14, 15, 16, 17, 18, 19 },
};
static rf_layout_t s_layout = RF_LAYOUT_IQ4;

extern int lmac_stop_hw_txq(void);
extern void phy_disable_agc(void);
extern void phy_rfagc_disable(void);
extern void phy_wifi_fbw_sel(uint32_t val);
extern void phy_force_rx_gain(bool enable, uint8_t gain_idx);
extern void phy_set_freq(uint16_t freq_mhz, int offset);

static const char *TAG = "rf";

static uint16_t s_freq_mhz = 5865u;
static uint8_t s_gain = 40u;
static bool s_bw40 = true;
static bool s_external_antenna = false;

typedef struct {
    uint8_t channel;
    uint16_t mhz;
} wifi5_center_t;

static const wifi5_center_t s_wifi5_centers[] = {
    {36, 5180}, {40, 5200}, {44, 5220}, {48, 5240},
    {52, 5260}, {56, 5280}, {60, 5300}, {64, 5320},
    {100, 5500}, {104, 5520}, {108, 5540}, {112, 5560},
    {116, 5580}, {120, 5600}, {124, 5620}, {128, 5640},
    {132, 5660}, {136, 5680}, {140, 5700}, {144, 5720},
    {149, 5745}, {153, 5765}, {157, 5785}, {161, 5805},
    {165, 5825}, {169, 5845}, {173, 5865}, {177, 5885},
};

static const wifi5_center_t *nearest_center(uint16_t mhz)
{
    const wifi5_center_t *best = &s_wifi5_centers[0];
    int best_delta = 0x7fffffff;
    for (size_t i = 0; i < sizeof(s_wifi5_centers) / sizeof(s_wifi5_centers[0]); ++i) {
        int d = abs((int)mhz - (int)s_wifi5_centers[i].mhz);
        if (d < best_delta) {
            best_delta = d;
            best = &s_wifi5_centers[i];
        }
    }
    return best;
}

static esp_err_t lock_rx_only(void)
{
    (void)lmac_stop_hw_txq();
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q)
        REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) &= ~MAC_TXQ_ENABLE;
    __asm__ __volatile__("fence iorw, iorw" ::: "memory");
    for (unsigned q = 0u; q < MAC_TXQ_COUNT; ++q) {
        if (REG32(MAC_TXQ0_CONF - q * MAC_TXQ_STRIDE) & MAC_TXQ_ENABLE)
            return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t route_modem_iq(void)
{
    uint64_t mask = 0u;
    for (unsigned lane = 0u; lane < RF_IQ_LANES; ++lane)
        mask |= 1ULL << rf_iq_pins[lane];
    const gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    rf_set_layout(s_layout);
    return ESP_OK;
}

static void enable_continuous_modem(void)
{
    REG32(HP_SRAM_USAGE) = (REG32(HP_SRAM_USAGE) & 0xfffef0ffu) | 0x00010000u;

    REG32(SOURCE_CTRL) &= 0xff87ffffu;
    REG32(SOURCE_MUX) = (REG32(SOURCE_MUX) & 0xfffffff8u) | 1u;
    REG32(MODEM_CLOCK) = UINT32_MAX;
    REG32(FE_ENABLE) |= 4u;
    REG32(FE_PATH) &= ~1u;

    uint32_t v = REG32(DUMP_FORMAT);
    v = (v & 0xff03ffffu) | 0x006c0000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffc0fffu) | 0x0001a000u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xfffff03fu) | 0x00000640u;
    REG32(DUMP_FORMAT) = v;
    v = (REG32(DUMP_FORMAT) & 0xffffffc0u) | 0x18u;
    REG32(DUMP_FORMAT) = v | 0x01000000u;

    /* TX_START never fires with the TX queues off, so the dump engine streams
     * pre-trigger samples onto MODEM_DIAG forever. */
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

/* Retuning can reset PHY receive state, so reassert everything after it. */
static void apply_rx_settings(void)
{
    enable_continuous_modem();
    phy_disable_agc();
    phy_rfagc_disable();
    phy_wifi_fbw_sel(s_bw40 ? 1u : 0u);
    phy_force_rx_gain(true, s_gain);
}

esp_err_t rf_start(void)
{
    gpio_reset_pin(ANTENNA_SEL_GPIO);
    gpio_set_direction(ANTENNA_SEL_GPIO, GPIO_MODE_OUTPUT);
    rf_set_external_antenna(s_external_antenna);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if ((err = nvs_flash_erase()) == ESP_OK) err = nvs_flash_init();
    }
    if (err != ESP_OK) return err;

    err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    /* Disconnected-station power saving periodically powers down the PHY,
     * which stops MODEM_DIAG. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.sta_disconnected_pm = false;
    if ((err = esp_wifi_init(&cfg)) != ESP_OK) return err;
    if ((err = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK) return err;
    if ((err = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    if ((err = esp_wifi_set_band_mode(WIFI_BAND_MODE_5G_ONLY)) != ESP_OK) return err;
    if ((err = esp_wifi_set_ps(WIFI_PS_NONE)) != ESP_OK) return err;

    wifi_protocols_t protocols = {
        .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
        .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N,
    };
    if ((err = esp_wifi_set_protocols(WIFI_IF_STA, &protocols)) != ESP_OK) return err;
    wifi_bandwidths_t bandwidths = { .ghz_2g = WIFI_BW20, .ghz_5g = WIFI_BW40 };
    if ((err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bandwidths)) != ESP_OK) return err;
    if ((err = esp_wifi_set_channel(173, WIFI_SECOND_CHAN_NONE)) != ESP_OK) return err;

    /* Promiscuous mode keeps the RX path running; the empty filter keeps the
     * MAC from delivering packets to software. */
    if ((err = esp_wifi_set_promiscuous(true)) != ESP_OK) return err;
    wifi_promiscuous_filter_t filter = { .filter_mask = 0 };
    (void)esp_wifi_set_promiscuous_filter(&filter);

    if ((err = lock_rx_only()) != ESP_OK) return err;
    if ((err = route_modem_iq()) != ESP_OK) return err;

    apply_rx_settings();
    ESP_LOGW(TAG, "RF ready at %u MHz, gain %u", s_freq_mhz, s_gain);
    return ESP_OK;
}

esp_err_t rf_set_freq(uint16_t mhz)
{
    if (mhz < RF_MIN_MHZ || mhz > RF_MAX_MHZ) return ESP_ERR_INVALID_ARG;

    /* Land on the nearest public channel first so the undocumented jump to
     * the exact frequency is as small as possible. */
    const wifi5_center_t *c = nearest_center(mhz);
    esp_err_t err = esp_wifi_set_channel(c->channel, WIFI_SECOND_CHAN_NONE);
    if (err != ESP_OK) return err;
    if (mhz != c->mhz) phy_set_freq(mhz, 0);

    apply_rx_settings();
    s_freq_mhz = mhz;
    return ESP_OK;
}

uint16_t rf_get_freq(void)
{
    return s_freq_mhz;
}

void rf_set_gain(uint8_t gain)
{
    if (gain > 62u) gain = 62u;
    s_gain = gain;
    phy_force_rx_gain(true, gain);
}

uint8_t rf_get_gain(void)
{
    return s_gain;
}

void rf_set_bw40(bool bw40)
{
    s_bw40 = bw40;
    phy_wifi_fbw_sel(bw40 ? 1u : 0u);
}

bool rf_get_bw40(void)
{
    return s_bw40;
}

void rf_set_external_antenna(bool external)
{
    s_external_antenna = external;
    gpio_set_level(ANTENNA_SEL_GPIO, external ? 1 : 0);
}

bool rf_get_external_antenna(void)
{
    return s_external_antenna;
}

void rf_set_layout(rf_layout_t layout)
{
    s_layout = layout;
    for (unsigned lane = 0u; lane < RF_IQ_LANES; ++lane) {
        esp_rom_gpio_connect_out_signal(rf_iq_pins[lane],
                                        MODEM_DIAG0_IDX + s_layout_diag[layout][lane],
                                        false, false);
    }
}

rf_layout_t rf_get_layout(void)
{
    return s_layout;
}
