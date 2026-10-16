#ifndef RADIOCORE_FFT_SSB_RX_H
#define RADIOCORE_FFT_SSB_RX_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FFT_RX_USB = 0,
    FFT_RX_LSB,
    FFT_RX_AM
} fft_rx_sideband_t;

void fft_ssb_rx_init(uint32_t sample_rate, uint32_t center_if_hz);
void fft_ssb_rx_reset(void);
void fft_ssb_rx_set_if_hz(uint32_t center_if_hz);
void fft_ssb_rx_set_filter(uint16_t low_hz, uint16_t high_hz);
void fft_ssb_rx_set_noise_reduction(bool enabled);
bool fft_ssb_rx_get_noise_reduction(void);
int16_t fft_ssb_rx_process(int32_t sample, fft_rx_sideband_t sideband);

#endif
