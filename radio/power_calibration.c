#include "radio/power_calibration.h"
#include <stddef.h>

// Original ZBITX values supplied by WP3DN. 0.0015 (40 m) is the reference
// for this 16-bit DSP implementation, preserving the already-proven 40 m level.
#define SCALE_REFERENCE 0.0015f

static const power_setting_t band_power[] = {
    { 3500000u,  4000000u, 37, 0.0020f, "80m" },
    { 5200000u,  5800009u, 40, 0.0015f, "60m" },
    { 7000000u,  7300009u, 40, 0.0015f, "40m" },
    {10000000u, 10200000u, 35, 0.0019f, "30m" },
    {14000000u, 14300000u, 35, 0.0025f, "20m" },
    {18000000u, 18200000u, 20, 0.0023f, "17m" },
    {21000000u, 21450000u, 20, 0.0030f, "15m" },
    {24800000u, 25000000u, 20, 0.0034f, "12m" },
    {28000000u, 29700000u, 20, 0.0037f, "10m" }
};

const power_setting_t *power_calibration_for_frequency(uint32_t frequency_hz) {
    for (size_t i = 0; i < sizeof(band_power) / sizeof(band_power[0]); ++i) {
        if (frequency_hz >= band_power[i].low_hz && frequency_hz <= band_power[i].high_hz)
            return &band_power[i];
    }
    return NULL;
}

float power_calibration_get_scale(uint32_t frequency_hz) {
    const power_setting_t *p = power_calibration_for_frequency(frequency_hz);
    return p ? p->scale : 0.0f;
}

uint32_t power_calibration_get_multiplier_q16(uint32_t frequency_hz) {
    const power_setting_t *p = power_calibration_for_frequency(frequency_hz);
    if (!p) return 0u; // Explicit out-of-band TX inhibit.
    float ratio = p->scale / SCALE_REFERENCE;
    if (ratio <= 0.0f) return 0u;
    return (uint32_t)(ratio * 65536.0f + 0.5f);
}

bool power_calibration_tx_allowed(uint32_t frequency_hz) {
    return power_calibration_for_frequency(frequency_hz) != NULL;
}

const char *power_calibration_get_band_name(uint32_t frequency_hz) {
    const power_setting_t *p = power_calibration_for_frequency(frequency_hz);
    return p ? p->name : "OUT-OF-BAND";
}
