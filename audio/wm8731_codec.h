#ifndef RADIOCORE_WM8731_CODEC_H
#define RADIOCORE_WM8731_CODEC_H
#include <stdbool.h>
#include <stdint.h>

bool wm8731_init_master_96k_16bit(uint8_t rx_input_percent,
                                  uint8_t mic_input_percent,
                                  uint8_t ear_output_percent,
                                  uint8_t mod_output_percent);

bool wm8731_set_rx_input_percent(uint8_t percent);   // ADC LEFT / AUDIO-DEMOD
bool wm8731_set_mic_input_percent(uint8_t percent);  // ADC RIGHT / microphone
bool wm8731_set_ear_output_percent(uint8_t percent); // physical EAR output (verified LEFT register)
bool wm8731_set_mod_output_percent(uint8_t percent); // physical AUDIO-MOD output (verified RIGHT register)

uint8_t wm8731_get_rx_input_percent(void);
uint8_t wm8731_get_mic_input_percent(void);
uint8_t wm8731_get_ear_output_percent(void);
uint8_t wm8731_get_mod_output_percent(void);

// Compatibility aliases retained for older console/code paths.
bool wm8731_set_input_percent(uint8_t percent);
bool wm8731_set_tx_output_percent(uint8_t percent);
uint8_t wm8731_get_input_percent(void);
uint8_t wm8731_get_tx_output_percent(void);

void wm8731_mute(bool mute);
#endif
