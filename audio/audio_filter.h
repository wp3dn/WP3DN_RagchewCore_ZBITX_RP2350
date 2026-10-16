#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Original lightweight 1-pole HP + 1-pole LP filter.  Kept unchanged for TX. */
typedef struct {
    int32_t hp_alpha_q15;
    int32_t lp_alpha_q15;
    int32_t hp_x1;
    int32_t hp_y1;
    int32_t lp_y;
} audio_bandpass_t;

void audio_bandpass_config(audio_bandpass_t *f, uint32_t sample_rate, uint16_t low_hz, uint16_t high_hz);
void audio_bandpass_reset(audio_bandpass_t *f);
int32_t audio_bandpass_process(audio_bandpass_t *f, int32_t x);

/* Steeper RX-only biquad section. Two sections are cascaded in main.c. */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} audio_biquad_t;

typedef struct {
    audio_biquad_t hp;
    audio_biquad_t lp;
} audio_rx_biquad_bandpass_t;

void audio_rx_biquad_config(audio_rx_biquad_bandpass_t *f, uint32_t sample_rate,
                            uint16_t low_hz, uint16_t high_hz,
                            uint8_t butterworth_section);
void audio_rx_biquad_reset(audio_rx_biquad_bandpass_t *f);
int32_t audio_rx_biquad_process(audio_rx_biquad_bandpass_t *f, int32_t x);
