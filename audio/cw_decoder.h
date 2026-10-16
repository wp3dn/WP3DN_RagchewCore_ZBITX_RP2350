#pragma once
#include <stdbool.h>
#include <stdint.h>

/*
 * Lightweight RP2350 CW decoder.
 * Architecture inspired by Mike/KB2ML's sBITX modem_cw decoder:
 * multi-bin tone detection + adaptive mark/space timing + Morse lookup.
 * This implementation is rewritten for RadioCore: the DMA path only
 * enqueues already-demodulated 24 kHz CW audio; all Goertzel/timing work
 * runs later in the foreground.
 */
void cw_decoder_init(uint32_t sample_rate_hz, uint16_t tone_hz, uint8_t initial_wpm);
void cw_decoder_reset(void);
void cw_decoder_set_tone(uint16_t tone_hz);
void cw_decoder_set_wpm(uint8_t wpm);
void cw_decoder_feed(int32_t sample);   /* IRQ-safe single-producer enqueue */
void cw_decoder_process(void);          /* foreground only */
const char *cw_decoder_text(void);
uint8_t cw_decoder_wpm_estimate(void);
int16_t cw_decoder_tone_error_hz(void);
uint32_t cw_decoder_drops(void);
