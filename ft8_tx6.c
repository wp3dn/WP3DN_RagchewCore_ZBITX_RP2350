#include "ft8_tx6.h"
#include "ft8_lib/ft8/pack.h"
#include "ft8_lib/ft8/encode.h"
#include <math.h>
#include <string.h>

/* Stream 6 kHz GFSK into the 96 kHz TX path: no 151 kB PCM array.
   The Gaussian pulse is prepared once, never in the audio callback. */
#define SPS 960u
#define NSYM 79u
#define N (SPS * NSYM)
#define PI 3.14159265358979323846f
static uint8_t tx_tones[NSYM];
#include "ft8_tx6_pulse_flash.h" /* const table in flash, not .bss */
static float phase, base_frequency;
static uint32_t sample_index, play_index;
static int16_t previous_sample, current_sample;
static volatile bool playing;
static bool ready;

bool ft8_tx6_prepare(const char *message, unsigned base_hz) {
    uint8_t payload[10];
    if (playing || !message || base_hz < 200u || base_hz > 2500u) return false;
    ready = false;
    if (pack77(message, payload) < 0) return false;
    ft8_encode(payload, tx_tones);
    base_frequency = (float)base_hz;
    phase = 0.f;
    sample_index = play_index = 0;
    previous_sample = current_sample = 0;
    ready = true;
    return true;
}
void ft8_tx6_start(void) {
    if (ready) {
        phase = 0.f;
        sample_index = play_index = 0;
        previous_sample = current_sample = 0;
        playing = true;
    }
}
void ft8_tx6_abort(void) { playing = false; ready = false; play_index = sample_index = 0; }
bool ft8_tx6_active(void) { return playing; }

static int16_t next_sample_6k(void) {
    uint32_t i = sample_index++;
    if (i >= N) return 0;
    unsigned k = i + SPS;
    float shift = 0.f;
    for (int s = (int)(k/SPS)-2; s <= (int)(k/SPS); ++s) {
        if (s < 0 || s >= (int)NSYM) continue;
        int j = (int)k - s*(int)SPS;
        if (j >= 0 && j < (int)(3u*SPS)) shift += tx_tones[s]*pulse[j];
    }
    if (k < 2u*SPS) shift += tx_tones[0]*pulse[k+SPS];
    if (k >= N && k < N+2u*SPS) shift += tx_tones[NSYM-1]*pulse[k-N];
    float amplitude = .30f;
    if (i < SPS/8u) amplitude *= .5f - .5f*cosf(PI*(float)i/(SPS/8u));
    if (i >= N-SPS/8u) amplitude *= .5f - .5f*cosf(PI*(float)(N-1u-i)/(SPS/8u));
    int16_t output = (int16_t)(sinf(phase)*amplitude*32767.f);
    phase += (2.f*PI/6000.f)*(base_frequency + 6.25f*shift);
    if (phase > 2.f*PI) phase -= 2.f*PI;
    return output;
}
int16_t ft8_tx6_next_96k(void) {
    if (!playing) return 0;
    uint32_t i = play_index++;
    if (i >= N*16u) { playing = false; return 0; }
    if ((i & 15u) == 0u) {
        previous_sample = current_sample;
        current_sample = next_sample_6k();
    }
    int32_t a = previous_sample, b = current_sample;
    return (int16_t)(a + (((b-a)*(int32_t)(i&15u)) >> 4));
}
