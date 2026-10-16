#pragma once
#include <stdint.h>
#include <stdbool.h>
/* Prepares an FT8 waveform at 6 kHz, for transmission at a UTC slot boundary. */
bool ft8_tx6_prepare(const char *message, unsigned base_hz);
void ft8_tx6_start(void);
void ft8_tx6_abort(void);
bool ft8_tx6_active(void);
/* Called exactly once per 96 kHz audio sample, interpolates 6 kHz waveform. */
int16_t ft8_tx6_next_96k(void);
