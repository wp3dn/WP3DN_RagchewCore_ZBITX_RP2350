#include "wm8731_codec.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define WM8731_ADDR 0x1a
#define WM_LEFT_LINE_IN  0x00
#define WM_RIGHT_LINE_IN 0x01
#define WM_LEFT_HP_OUT   0x02
#define WM_RIGHT_HP_OUT  0x03
#define WM_ANALOG_PATH   0x04
#define WM_DIGITAL_PATH  0x05
#define WM_POWER_DOWN    0x06
#define WM_DIGITAL_IF    0x07
#define WM_SAMPLE_RATE   0x08
#define WM_ACTIVE        0x09
#define WM_RESET         0x0f

static uint8_t rx_input_percent_ = 100;
static uint8_t mic_input_percent_ = 20;
static uint8_t ear_output_percent_ = 100;
static uint8_t mod_output_percent_ = 100;

static bool wm_write(uint8_t reg, uint16_t value) {
    uint8_t b[2] = {(uint8_t)((reg << 1) | ((value >> 8) & 1u)), (uint8_t)value};
    return i2c_write_blocking(i2c1, WM8731_ADDR, b, 2, false) == 2;
}

static uint16_t line_value(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint8_t step = (uint8_t)(((uint16_t)percent * 31u + 50u) / 100u);
    // Bit 8 (LRINBOTH) deliberately remains clear so LEFT and RIGHT are independent.
    return step;
}

static uint16_t hp_value(uint8_t percent) {
    if (percent > 100) percent = 100;
    // Bit 8 (LRHPBOTH) deliberately remains clear so LEFT and RIGHT are independent.
    return percent == 0 ? 0 : (uint16_t)(0x30u + ((uint32_t)(percent - 1u) * 0x4fu) / 99u);
}

bool wm8731_set_rx_input_percent(uint8_t percent) {
    if (percent > 100) return false;
    rx_input_percent_ = percent;
    return wm_write(WM_LEFT_LINE_IN, line_value(percent));
}

bool wm8731_set_mic_input_percent(uint8_t percent) {
    if (percent > 100) return false;
    mic_input_percent_ = percent;
    return wm_write(WM_RIGHT_LINE_IN, line_value(percent));
}

bool wm8731_set_ear_output_percent(uint8_t percent) {
    if (percent > 100) return false;
    ear_output_percent_ = percent;
    return wm_write(WM_LEFT_HP_OUT, hp_value(percent));
}

bool wm8731_set_mod_output_percent(uint8_t percent) {
    if (percent > 100) return false;
    mod_output_percent_ = percent;
    return wm_write(WM_RIGHT_HP_OUT, hp_value(percent));
}

uint8_t wm8731_get_rx_input_percent(void) { return rx_input_percent_; }
uint8_t wm8731_get_mic_input_percent(void) { return mic_input_percent_; }
uint8_t wm8731_get_ear_output_percent(void) { return ear_output_percent_; }
uint8_t wm8731_get_mod_output_percent(void) { return mod_output_percent_; }

bool wm8731_set_input_percent(uint8_t percent) {
    bool ok = wm8731_set_rx_input_percent(percent);
    ok &= wm8731_set_mic_input_percent(percent);
    return ok;
}
bool wm8731_set_tx_output_percent(uint8_t percent) { return wm8731_set_mod_output_percent(percent); }
uint8_t wm8731_get_input_percent(void) { return wm8731_get_rx_input_percent(); }
uint8_t wm8731_get_tx_output_percent(void) { return wm8731_get_mod_output_percent(); }

void wm8731_mute(bool mute) { wm_write(WM_DIGITAL_PATH, mute ? 0x008 : 0x000); }

bool wm8731_init_master_96k_16bit(uint8_t rx_input_percent,
                                  uint8_t mic_input_percent,
                                  uint8_t ear_output_percent,
                                  uint8_t mod_output_percent) {
    bool ok = wm_write(WM_RESET, 0x000); sleep_ms(10);
    ok &= wm8731_set_rx_input_percent(rx_input_percent);
    ok &= wm8731_set_mic_input_percent(mic_input_percent);
    ok &= wm8731_set_mod_output_percent(mod_output_percent);
    ok &= wm8731_set_ear_output_percent(ear_output_percent);
    ok &= wm_write(WM_ANALOG_PATH, 0x012);
    ok &= wm_write(WM_DIGITAL_PATH, 0x000);
    ok &= wm_write(WM_POWER_DOWN, 0x000);
    ok &= wm_write(WM_DIGITAL_IF, 0x042); // master, Philips I2S, 16-bit
    ok &= wm_write(WM_SAMPLE_RATE, 0x01d);
    ok &= wm_write(WM_ACTIVE, 0x001);
    sleep_ms(20);
    return ok;
}
