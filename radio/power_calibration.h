#ifndef RADIOCORE_POWER_CALIBRATION_H
#define RADIOCORE_POWER_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t low_hz;
    uint32_t high_hz;
    uint8_t original_power;
    float scale;
    const char *name;
} power_setting_t;

const power_setting_t *power_calibration_for_frequency(uint32_t frequency_hz);
float power_calibration_get_scale(uint32_t frequency_hz);
uint32_t power_calibration_get_multiplier_q16(uint32_t frequency_hz);
bool power_calibration_tx_allowed(uint32_t frequency_hz);
const char *power_calibration_get_band_name(uint32_t frequency_hz);

#endif
