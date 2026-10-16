#ifndef FFT_SSB_TX_H
#define FFT_SSB_TX_H
#include <stdint.h>
#include <stdbool.h>
typedef enum { FFT_SSB_USB=0, FFT_SSB_LSB=1, FFT_SSB_AM=2 } fft_ssb_sideband_t;
void fft_ssb_tx_init(uint32_t sample_rate, uint32_t if_hz);
void fft_ssb_tx_reset(void);
void fft_ssb_tx_set_filter(uint16_t low_hz, uint16_t high_hz);
void fft_ssb_tx_set_am_carrier(uint8_t percent);
uint8_t fft_ssb_tx_get_am_carrier(void);
int16_t fft_ssb_tx_process(int16_t sample, fft_ssb_sideband_t sideband);
#endif
