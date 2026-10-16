#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RADIOCORE_BAND_COUNT 9u

typedef struct {
    uint32_t frequency_hz;
    uint8_t sideband;
    uint16_t rx_low_hz, rx_high_hz, tx_low_hz, tx_high_hz;
    uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out;
    uint8_t if_gain;
    uint16_t volume, mic_gain;
    uint8_t tx_drive;
    uint8_t step_index;
    uint8_t agc_enabled, agc_speed, agc_target, agc_max;
    uint32_t tx_if_hz;
    uint8_t operating_mode;
    uint16_t cw_offset_hz;
    uint8_t sidetone_level;
    uint16_t cw_delay_ms;
    uint8_t cw_wpm;
    uint32_t bfo_hz, global_if_hz;
    uint8_t clk1_drive_ma, clk2_drive_ma;

    /* Per-band operating state. */
    uint32_t band_frequency_hz[RADIOCORE_BAND_COUNT];
    int16_t band_spectrum_offset_db[RADIOCORE_BAND_COUNT];

    /* v7 extension: retain v6 binary prefix for migration. */
    uint32_t vfo_frequency_hz[2];
    uint8_t active_vfo;
    int16_t rit_offset_hz;
    uint8_t dsp_nr_enabled;
    uint16_t am_rx_bw_hz;
    uint8_t am_carrier_percent;
    uint8_t cw_keyer_mode;
    uint16_t cw_decode_pitch_hz;
    int16_t bfo_delta_hz;
    uint8_t split_enabled;
    uint8_t band_mode[RADIOCORE_BAND_COUNT];
    uint16_t mode_rx_low_hz[7], mode_rx_high_hz[7];
    uint8_t ft8_tx_first, ft8_auto, ft8_repeat;
    uint16_t ft8_tx_pitch_hz;
    char station_callsign[16], station_grid[12];
    uint8_t usb_mode;
    uint8_t reserved[16];

} radiocore_settings_t;

bool settings_load(radiocore_settings_t *out);
bool settings_save(const radiocore_settings_t *in);

#ifdef RAGCHEW_SD_DIAG
void settings_sd_diag_print(void);
#endif

/* Append an already formatted ADIF QSO record; true only after sync/close. */
bool log_adif_append(const char *record);

/* Read-only bounded ADIF snapshot for stock panel viewer. */
int log_adif_read_snapshot(char *out, unsigned capacity);
