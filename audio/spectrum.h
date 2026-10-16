#ifndef RADIOCORE_SPECTRUM_H
#define RADIOCORE_SPECTRUM_H

#include <stdbool.h>
#include <stdint.h>

#define SPECTRUM_BINS 64u
#define SPECTRUM_WIDE_BINS 128u

void spectrum_init(void);
void spectrum_reset(void);
/* Feed one complex baseband sample at 24 kHz. Safe from the audio IRQ. */
void spectrum_feed(int32_t i_sample, int32_t q_sample);
/* Invasion #4: publish one 64-bin Fourier RX power frame (linear power). */
void spectrum_feed_fourier_power(const float power[SPECTRUM_BINS]);
/* Native +/-12 kHz RX FFT bins for high-resolution front panels. */
void spectrum_feed_fourier_wide_power(const float power[SPECTRUM_WIDE_BINS]);
/* Invasion #10A: TX snapshot uses an independent 96 dB display span. */
void spectrum_feed_fourier_tx_power(const float power[SPECTRUM_BINS]);
/* Calculate a new display frame when enough samples are available. */
bool spectrum_process(void);
/* Copy 0..18 pixel heights, low frequency at the left and high at the right. */
void spectrum_get_bins(uint8_t bins[SPECTRUM_BINS]);
/* Copy 128 native 187.5-Hz bins across +/-12 kHz, without legacy OLED hold/blur. */
void spectrum_get_wide_bins(uint8_t bins[SPECTRUM_WIDE_BINS]);
/* Operator-set amount of noise shown above the bottom edge, 0..24 dB. */
void spectrum_set_offset_db(int16_t offset_db);
int16_t spectrum_get_offset_db(void);

#endif
