#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "audio/spectrum.h"

typedef enum {
 UI_ITEM_BAND,
 UI_ITEM_STEP,
 UI_ITEM_RIT,
 UI_ITEM_MODE,
 UI_ITEM_VOLUME,
 UI_ITEM_IF_GAIN,
 UI_ITEM_AGC,
 UI_ITEM_AGC_TARGET,
 UI_ITEM_AGC_MAX,
 UI_ITEM_RX_CODEC_IN,
 UI_ITEM_RX_LOW,
 UI_ITEM_RX_HIGH,
 UI_ITEM_AM_RX_BW,
 UI_ITEM_SPECTRUM_OFFSET,
 UI_ITEM_TX_DRIVE,
 UI_ITEM_MIC_GAIN,
 UI_ITEM_MIC_CODEC_IN,
 UI_ITEM_MOD_CODEC_OUT,
 UI_ITEM_TX_LOW,
 UI_ITEM_TX_HIGH,
 UI_ITEM_AM_CARRIER,
 UI_ITEM_TX_IF,
 UI_ITEM_EAR_CODEC_OUT,
 UI_ITEM_CW_OFFSET,
 UI_ITEM_SIDETONE,
 UI_ITEM_CW_DELAY,
 UI_ITEM_CW_SPEED,
 UI_ITEM_KEYER_MODE,
 UI_ITEM_CW_DECODE_PITCH,
 UI_ITEM_BFO_FREQ,
 UI_ITEM_BFO_DELTA,
 UI_ITEM_DSP_NR,
 UI_ITEM_GLOBAL_IF,
 UI_ITEM_CLK1_DRIVE,
 UI_ITEM_CLK2_DRIVE,
 UI_ITEM_ENGINE_MODE,
 UI_ITEM_PROGRAM_MODE,
 UI_ITEM_SAVE,
 UI_ITEM_DEFAULTS,
 UI_ITEM_ABOUT,
 UI_ITEM_COUNT
} ui_item_t;

typedef enum {
 UI_EVENT_NONE,
 UI_EVENT_TUNE,
 UI_EVENT_ADJUST,
 UI_EVENT_CYCLE_STEP,
 UI_EVENT_FORCE_SAVE,
 UI_EVENT_FACTORY_DEFAULTS,
 UI_EVENT_PROGRAM_MODE,
 UI_EVENT_APPLY_DISPLAY_SETTINGS,
 UI_EVENT_REMOTE_PTT,
 UI_EVENT_SET_BW,
 UI_EVENT_SET_CW_PITCH,
 UI_EVENT_SET_MODE,
 UI_EVENT_SET_VFO,
 UI_EVENT_FT8_COMMAND
} ui_event_type_t;

#pragma pack(push,1)
typedef struct {
 uint32_t frequency_hz;
 uint8_t operating_mode;
 uint8_t step_index;
 uint16_t volume;
 uint8_t if_gain;
 uint8_t agc_mode;
 uint8_t agc_target;
 uint8_t agc_max;
 uint8_t rx_codec_in;
 uint16_t rx_low;
 uint16_t rx_high;
 uint16_t am_rx_bw_hz;
 uint8_t am_carrier_percent;
 int16_t spectrum_offset_db;
 uint8_t tx_drive;
 uint8_t mic_codec_in;
 uint8_t mod_codec_out;
 uint16_t tx_low;
 uint16_t tx_high;
 uint32_t tx_if_hz;
 uint8_t ear_codec_out;
 uint16_t cw_offset_hz;
 uint8_t sidetone_level;
 uint16_t cw_delay_ms;
 uint8_t cw_wpm;
 uint8_t keyer_mode;
 uint16_t cw_decode_pitch_hz;
 uint32_t bfo_hz;
 uint32_t global_if_hz;
 uint8_t clk1_drive_ma;
 uint8_t clk2_drive_ma;
 uint32_t band_frequency_hz[9];
 int16_t band_spectrum_offset_db[9];
} panel_settings_t;
#pragma pack(pop)

typedef struct {
 ui_event_type_t type;
 int32_t delta;
 ui_item_t item;
 panel_settings_t settings;
 char ft8_command[100];
} ui_event_t;

typedef struct {
 uint32_t frequency_hz;
 uint8_t operating_mode;
 bool tx;
 uint16_t volume, mic_gain;
 uint8_t if_gain, tx_drive;
 uint16_t rx_low, rx_high, tx_low, tx_high;
 uint16_t am_rx_bw_hz;
 uint8_t am_carrier_percent;
 uint32_t tx_if_hz;
 uint32_t signal_envelope;
 uint8_t band_index, agc_mode, agc_target, agc_max;
 uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out;
 uint8_t step_index;
 int16_t rit_offset_hz;
 uint16_t cw_offset_hz, cw_delay_ms;
 uint8_t sidetone_level, cw_wpm, keyer_mode;
 uint16_t cw_decode_pitch_hz;
 uint32_t bfo_hz, global_if_hz;
 int16_t bfo_delta_hz;
 uint8_t active_vfo;
 uint32_t vfo_a_hz, vfo_b_hz;
 bool dsp_nr_enabled;
 uint8_t clk1_drive_ma, clk2_drive_ma;
 int16_t spectrum_offset_db;
 uint8_t spectrum[SPECTRUM_BINS];
 uint8_t spectrum_wide[SPECTRUM_WIDE_BINS];
 bool cw_decode_active;
 char cw_decode_text[43];
 uint8_t cw_decode_wpm;
 int16_t cw_decode_tone_error_hz;
 bool protocol1_mode;
 bool p1_net_initialized;
 bool p1_link_up;
 bool p1_discovered;
 bool p1_control_seen;
 bool p1_rx_streaming;
 bool p1_mox_requested;
 uint32_t p1_rx_frequency_hz;
} ui_snapshot_t;

bool front_panel_init(void);
ui_event_t front_panel_poll(const ui_snapshot_t *s);
void front_panel_render(const ui_snapshot_t *s, bool force);

/* Phase 2M narrow audio stream.
   Capture is ISR-safe and non-blocking. Service runs from the main loop. */
void front_panel_audio_capture_6k(int16_t sample);
void front_panel_audio_service(void);

/* Phase 2BZ reverse 6 kHz audio path: Display USB playback -> RadioCore TX. */
bool front_panel_tx_audio_pop_6k(int16_t *sample);
void front_panel_tx_audio_flush(void);
uint16_t front_panel_tx_audio_queued(void);

/* N6 FT8 decoded-list transport to stock ZBITX panel. */
void front_panel_ft8_message(const char *line);

/* Update an existing stock-panel field (does not save a QSO). */
void front_panel_set_field(const char *label, const char *value);

void front_panel_logbook_qso(const char *record);
