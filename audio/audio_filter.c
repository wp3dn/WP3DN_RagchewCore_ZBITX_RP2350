#include "audio/audio_filter.h"
#include <math.h>
#include <limits.h>

/* ---- Original lightweight filter: preserved for the TX path. ---- */
void audio_bandpass_config(audio_bandpass_t *f, uint32_t fs, uint16_t low_hz, uint16_t high_hz) {
    float hp_a = expf(-2.0f * (float)M_PI * (float)low_hz / (float)fs);
    float lp_a = 1.0f - expf(-2.0f * (float)M_PI * (float)high_hz / (float)fs);
    f->hp_alpha_q15 = (int32_t)lroundf(hp_a * 32768.0f);
    f->lp_alpha_q15 = (int32_t)lroundf(lp_a * 32768.0f);
}

void audio_bandpass_reset(audio_bandpass_t *f) {
    f->hp_x1 = f->hp_y1 = f->lp_y = 0;
}

int32_t audio_bandpass_process(audio_bandpass_t *f, int32_t x) {
    int32_t hp = (int32_t)(((int64_t)f->hp_alpha_q15 * (f->hp_y1 + x - f->hp_x1)) >> 15);
    f->hp_x1 = x;
    f->hp_y1 = hp;
    f->lp_y += (int32_t)(((int64_t)(hp - f->lp_y) * f->lp_alpha_q15) >> 15);
    return f->lp_y;
}

/* ---- RX-only 4th-order Butterworth HP + LP cascade support. ---- */
static const float butter_q[2] = {
    0.5411961001f,
    1.3065629649f
};

static void biquad_lowpass(audio_biquad_t *b, float fs, float fc, float q) {
    const float k = tanf((float)M_PI * fc / fs);
    const float k2 = k * k;
    const float norm = 1.0f / (1.0f + k / q + k2);
    b->b0 = k2 * norm;
    b->b1 = 2.0f * b->b0;
    b->b2 = b->b0;
    b->a1 = 2.0f * (k2 - 1.0f) * norm;
    b->a2 = (1.0f - k / q + k2) * norm;
}

static void biquad_highpass(audio_biquad_t *b, float fs, float fc, float q) {
    const float k = tanf((float)M_PI * fc / fs);
    const float k2 = k * k;
    const float norm = 1.0f / (1.0f + k / q + k2);
    b->b0 = norm;
    b->b1 = -2.0f * norm;
    b->b2 = norm;
    b->a1 = 2.0f * (k2 - 1.0f) * norm;
    b->a2 = (1.0f - k / q + k2) * norm;
}

static inline float biquad_process(audio_biquad_t *b, float x) {
    const float y = b->b0 * x + b->z1;
    b->z1 = b->b1 * x - b->a1 * y + b->z2;
    b->z2 = b->b2 * x - b->a2 * y;
    return y;
}

void audio_rx_biquad_config(audio_rx_biquad_bandpass_t *f, uint32_t fs,
                            uint16_t low_hz, uint16_t high_hz,
                            uint8_t section) {
    if (section > 1u) section = 1u;
    float low = (float)low_hz;
    float high = (float)high_hz;
    const float nyquist_guard = (float)fs * 0.49f;
    if (low < 1.0f) low = 1.0f;
    if (high > nyquist_guard) high = nyquist_guard;
    if (high <= low) high = low + 1.0f;

    const float q = butter_q[section];
    biquad_highpass(&f->hp, (float)fs, low, q);
    biquad_lowpass(&f->lp, (float)fs, high, q);
    audio_rx_biquad_reset(f);
}

void audio_rx_biquad_reset(audio_rx_biquad_bandpass_t *f) {
    f->hp.z1 = f->hp.z2 = 0.0f;
    f->lp.z1 = f->lp.z2 = 0.0f;
}

int32_t audio_rx_biquad_process(audio_rx_biquad_bandpass_t *f, int32_t x) {
    float y = biquad_process(&f->hp, (float)x);
    y = biquad_process(&f->lp, y);
    if (y > (float)INT32_MAX) return INT32_MAX;
    if (y < (float)INT32_MIN) return INT32_MIN;
    return (int32_t)lrintf(y);
}
