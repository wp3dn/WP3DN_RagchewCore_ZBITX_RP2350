#include "rx_agc.h"

#define Q16_ONE 65536u
#define Q16_MIN_GAIN 4096u   // 1/16x: permits attenuation of very strong signals.

static bool enabled_ = true;
static rx_agc_speed_t speed_ = RX_AGC_SLOW;
static uint8_t target_percent_ = 35;
static uint8_t max_gain_ = 16;
static uint32_t envelope_ = 0;
static uint32_t gain_q16_ = Q16_ONE;

static inline uint32_t abs32(int32_t sample) {
    return sample < 0 ? (uint32_t)(-(int64_t)sample) : (uint32_t)sample;
}

void rx_agc_init(void) {
    enabled_ = true;
    speed_ = RX_AGC_SLOW;
    target_percent_ = 35;
    max_gain_ = 16;
    rx_agc_reset();
}

void rx_agc_reset(void) {
    envelope_ = 0;
    gain_q16_ = Q16_ONE;
}

int32_t rx_agc_process(int32_t sample) {
    if (!enabled_) return sample;

    uint32_t magnitude = abs32(sample);
    // Fast envelope attack and slower decay. This envelope is only used to
    // estimate the required gain; audio itself is not rectified or filtered.
    uint8_t env_shift = magnitude > envelope_ ? 3u : (speed_ == RX_AGC_FAST ? 10u : 13u);
    int32_t env_error = (int32_t)magnitude - (int32_t)envelope_;
    envelope_ = (uint32_t)((int32_t)envelope_ + (env_error >> env_shift));

    uint32_t target = ((uint32_t)target_percent_ * 32767u) / 100u;
    uint32_t desired_q16;
    if (envelope_ < 1u) {
        desired_q16 = (uint32_t)max_gain_ * Q16_ONE;
    } else {
        desired_q16 = (uint32_t)(((uint64_t)target << 16) / envelope_);
    }

    uint32_t max_q16 = (uint32_t)max_gain_ * Q16_ONE;
    if (desired_q16 > max_q16) desired_q16 = max_q16;
    if (desired_q16 < Q16_MIN_GAIN) desired_q16 = Q16_MIN_GAIN;

    // Gain falls quickly on strong signals and rises gently after they leave.
    uint8_t gain_shift = desired_q16 < gain_q16_ ? 3u : (speed_ == RX_AGC_FAST ? 11u : 14u);
    int64_t gain_error = (int64_t)desired_q16 - (int64_t)gain_q16_;
    gain_q16_ = (uint32_t)((int64_t)gain_q16_ + (gain_error >> gain_shift));

    return (int32_t)(((int64_t)sample * gain_q16_) >> 16);
}

void rx_agc_set_enabled(bool enabled) { enabled_ = enabled; if (enabled) rx_agc_reset(); }
bool rx_agc_get_enabled(void) { return enabled_; }
void rx_agc_set_speed(rx_agc_speed_t speed) { speed_ = speed; }
rx_agc_speed_t rx_agc_get_speed(void) { return speed_; }

bool rx_agc_set_target_percent(uint8_t percent) {
    if (percent < 5u || percent > 90u) return false;
    target_percent_ = percent;
    return true;
}
uint8_t rx_agc_get_target_percent(void) { return target_percent_; }

bool rx_agc_set_max_gain(uint8_t gain) {
    if (gain < 1u || gain > 64u) return false;
    max_gain_ = gain;
    uint32_t max_q16 = (uint32_t)gain * Q16_ONE;
    if (gain_q16_ > max_q16) gain_q16_ = max_q16;
    return true;
}
uint8_t rx_agc_get_max_gain(void) { return max_gain_; }
uint32_t rx_agc_get_gain_q16(void) { return gain_q16_; }
uint32_t rx_agc_get_envelope(void) { return envelope_; }
