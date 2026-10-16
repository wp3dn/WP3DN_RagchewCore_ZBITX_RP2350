#ifndef SSB_TX_H
#define SSB_TX_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SSB_TX_USB = 0,
    SSB_TX_LSB = 1
} ssb_tx_sideband_t;

void ssb_tx_init(uint32_t sample_rate, uint32_t if_hz);
void ssb_tx_reset(void);
int16_t ssb_tx_process(int16_t mic_sample, ssb_tx_sideband_t sideband);

void ssb_tx_set_enabled(bool enabled);
bool ssb_tx_get_enabled(void);
bool ssb_tx_set_mic_gain(uint16_t percent);
uint16_t ssb_tx_get_mic_gain(void);
bool ssb_tx_set_drive(uint8_t percent);
uint8_t ssb_tx_get_drive(void);
void ssb_tx_set_band_multiplier_q16(uint32_t multiplier_q16);
uint32_t ssb_tx_get_band_multiplier_q16(void);
bool ssb_tx_set_if_hz(uint32_t hz);
uint32_t ssb_tx_get_if_hz(void);
uint32_t ssb_tx_get_clip_count(void);
int32_t ssb_tx_get_mic_peak(void);
int32_t ssb_tx_get_dac_peak(void);

#endif

bool ssb_tx_set_filter(uint16_t low_hz, uint16_t high_hz);
uint16_t ssb_tx_get_low_hz(void);
uint16_t ssb_tx_get_high_hz(void);
