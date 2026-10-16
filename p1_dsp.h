#pragma once
#include <stdint.h>
#include <stdbool.h>

#define P1_RX_DECIM_TAPS 15u

typedef struct {
    uint32_t rx_phase;
    uint32_t tx_phase;
    uint32_t phase_inc;
    int16_t prev_i;
    int16_t prev_q;
    bool tx_have_prev;

    /* RX anti-alias FIR state for 96 -> 48 kHz complex decimation. */
    int32_t rx_i_hist[P1_RX_DECIM_TAPS];
    int32_t rx_q_hist[P1_RX_DECIM_TAPS];
    uint8_t rx_fir_pos;
    uint8_t rx_decim_phase;
} p1_dsp_t;

void p1_dsp_init(p1_dsp_t *s, uint32_t if_hz);
void p1_dsp_set_if(p1_dsp_t *s, uint32_t if_hz);
void p1_dsp_reset_rx(p1_dsp_t *s);
void p1_dsp_reset_tx(p1_dsp_t *s);
/* 96 kHz real IF -> filtered 48 kHz complex IQ. Returns IQ frames. */
int p1_rx_real96_to_iq48(p1_dsp_t *s, const int16_t *real96, int n96,
                         int32_t *i24, int32_t *q24, int max48);
/* 48 kHz complex IQ -> 96 kHz real IF. Returns real samples. */
int p1_tx_iq48_to_real96(p1_dsp_t *s, const int16_t *iq_interleaved,
                         int n48, int16_t *real96, int max96);
