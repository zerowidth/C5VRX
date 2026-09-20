/**
 * rx5808.c - RX5808 module as a reference receiver.
 *
 * The module tunes over a 3-wire SPI-like bus (25-bit frames, LSB first) and
 * reports signal strength as a voltage on an ADC pin. Only the synthesizer
 * register is written; everything else keeps its power-on default.
 */

#include "rx5808.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define PIN_DATA GPIO_NUM_8
#define PIN_LE   GPIO_NUM_9
#define PIN_CLK  GPIO_NUM_10
#define RSSI_ADC_CHANNEL ADC_CHANNEL_5 /* GP6 */

#define REG_SYNTH_B 0x1u
#define IF_MHZ      479u   /* Module's intermediate frequency. */
#define BIT_US      1u

static const char *TAG = "rx5808";
static adc_oneshot_unit_handle_t s_adc;

static void clock_bit(bool value)
{
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_DATA, value);
    esp_rom_delay_us(BIT_US);
    gpio_set_level(PIN_CLK, 1);
    esp_rom_delay_us(BIT_US);
    gpio_set_level(PIN_CLK, 0);
    esp_rom_delay_us(BIT_US);
}

static void write_register(uint8_t addr, uint32_t data20)
{
    gpio_set_level(PIN_LE, 0);
    esp_rom_delay_us(BIT_US);
    for (int b = 0; b < 4; ++b) clock_bit((addr >> b) & 1u);
    clock_bit(true); /* write */
    for (int b = 0; b < 20; ++b) clock_bit((data20 >> b) & 1u);
    gpio_set_level(PIN_LE, 1);
    esp_rom_delay_us(BIT_US);
}

esp_err_t rx5808_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_DATA) | (1ULL << PIN_LE) | (1ULL << PIN_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) return err;
    gpio_set_level(PIN_LE, 1);

    const adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    if ((err = adc_oneshot_new_unit(&unit, &s_adc)) != ESP_OK) return err;

    /* 12 dB of attenuation covers the module's full RSSI swing. */
    const adc_oneshot_chan_cfg_t chan = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    return adc_oneshot_config_channel(s_adc, RSSI_ADC_CHANNEL, &chan);
}

void rx5808_set_freq(uint16_t mhz)
{
    /* The datasheet's tuning equation: f = 2 * (N * 32 + A) + IF. */
    uint32_t steps = (mhz - IF_MHZ) / 2u;
    uint32_t n = steps / 32u, a = steps % 32u;
    write_register(REG_SYNTH_B, (n << 7) | a);
    ESP_LOGW(TAG, "tuned %u MHz (N=%lu A=%lu)", mhz, (unsigned long)n, (unsigned long)a);
}

int rx5808_read_mv(void)
{
    if (!s_adc) return 0;
    int raw = 0;
    if (adc_oneshot_read(s_adc, RSSI_ADC_CHANNEL, &raw) != ESP_OK) return 0;
    /* Raw counts scaled by the nominal full-scale of this attenuation; good
     * enough for comparing against our own readings, which are also relative. */
    return raw * 3100 / 4095;
}
