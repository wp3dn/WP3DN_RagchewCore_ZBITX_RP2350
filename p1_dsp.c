#include "p1_dsp.h"
#include <string.h>

#define FS96 96000u

/*
 * 15-tap half-band FIR for 96 -> 48 kHz decimation.
 * Every other coefficient is zero, so only 5 multiplies/channel/output
 * are required when symmetry is used. Q15 coefficients, unity DC gain.
 */
static const int16_t rx_decim_fir_q15[P1_RX_DECIM_TAPS] = {
    -1309, 0, 1682, 0, -3312, 0, 10776, 17094,
    10776, 0, -3312, 0, 1682, 0, -1309
};

static int16_t sin_q15(uint32_t phase) {
    /* At the normal 24 kHz IF / 96 kHz Fs this lands exactly on quadrants. */
    uint16_t x = (uint16_t)(phase >> 16);
    int32_t y;
    if (x < 16384) y = (int32_t)x * 2;
    else if (x < 49152) y = 32767 - ((int32_t)(x - 16384) * 2);
    else y = -32767 + ((int32_t)(x - 49152) * 2);
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    return (int16_t)y;
}

static int16_t cos_q15(uint32_t phase) {
    return sin_q15(phase + 0x40000000u);
}

void p1_dsp_init(p1_dsp_t *s, uint32_t if_hz) {
    memset(s, 0, sizeof(*s));
    p1_dsp_set_if(s, if_hz);
}

void p1_dsp_set_if(p1_dsp_t *s, uint32_t if_hz) {
    s->phase_inc = (uint32_t)(((uint64_t)if_hz << 32) / FS96);
}

void p1_dsp_reset_rx(p1_dsp_t *s) {
    s->rx_phase = 0;
    memset(s->rx_i_hist, 0, sizeof(s->rx_i_hist));
    memset(s->rx_q_hist, 0, sizeof(s->rx_q_hist));
    s->rx_fir_pos = 0;
    s->rx_decim_phase = 0;
}

void p1_dsp_reset_tx(p1_dsp_t *s) {
    s->tx_phase = 0;
    s->prev_i = 0;
    s->prev_q = 0;
    s->tx_have_prev = false;
}

/* Return history sample lag samples back from newest sample. */
static inline int32_t hist_get(const int32_t *hist, uint8_t newest, unsigned lag) {
    int idx = (int)newest - (int)lag;
    while (idx < 0) idx += (int)P1_RX_DECIM_TAPS;
    return hist[idx];
}

static inline int32_t fir_one(const int32_t *hist, uint8_t newest) {
    /*
     * Same 15-tap half-band response as Phase 3f, but fully unrolled.
     * The mixer math remains unchanged from the known-good 3f build.
     */
    int32_t x0  = hist_get(hist, newest, 0u);
    int32_t x2  = hist_get(hist, newest, 2u);
    int32_t x4  = hist_get(hist, newest, 4u);
    int32_t x6  = hist_get(hist, newest, 6u);
    int32_t x7  = hist_get(hist, newest, 7u);
    int32_t x8  = hist_get(hist, newest, 8u);
    int32_t x10 = hist_get(hist, newest, 10u);
    int32_t x12 = hist_get(hist, newest, 12u);
    int32_t x14 = hist_get(hist, newest, 14u);

    int64_t acc = 0;
    acc += ((int64_t)x0  + x14) * rx_decim_fir_q15[0];
    acc += ((int64_t)x2  + x12) * rx_decim_fir_q15[2];
    acc += ((int64_t)x4  + x10) * rx_decim_fir_q15[4];
    acc += ((int64_t)x6  + x8)  * rx_decim_fir_q15[6];
    acc += (int64_t)x7 * rx_decim_fir_q15[7];

    return (int32_t)(acc >> 15);
}

int p1_rx_real96_to_iq48(p1_dsp_t *s, const int16_t *x, int n96,
                         int32_t *io, int32_t *qo, int max48) {
    int out = 0;

    for (int n = 0; n < n96; ++n) {
        int32_t sample = x[n];
        int32_t c = cos_q15(s->rx_phase);
        int32_t qlo = -sin_q15(s->rx_phase);
        s->rx_phase += s->phase_inc;

        /* Keep the same approximate 24-bit scale used by the Phase 3 path. */
        int32_t mix_i = (int32_t)(((int64_t)sample * c) >> 6);
        int32_t mix_q = (int32_t)(((int64_t)sample * qlo) >> 6);

        uint8_t p = s->rx_fir_pos;
        s->rx_i_hist[p] = mix_i;
        s->rx_q_hist[p] = mix_q;
        uint8_t newest = p;
        p++;
        if (p >= P1_RX_DECIM_TAPS) p = 0;
        s->rx_fir_pos = p;

        /* Produce exactly one 48 kHz IQ frame for every two 96 kHz inputs. */
        s->rx_decim_phase ^= 1u;
        if (s->rx_decim_phase != 0u) continue;
        if (out >= max48) break;

        io[out] = fir_one(s->rx_i_hist, newest);
        qo[out] = fir_one(s->rx_q_hist, newest);
        ++out;
    }

    return out;
}

int p1_tx_iq48_to_real96(p1_dsp_t *s, const int16_t *iq, int n48,
                         int16_t *y, int max96) {
    int out = 0;
    for (int n = 0; n < n48 && out + 1 < max96; ++n) {
        int16_t ci = iq[2 * n + 0];
        int16_t cq = iq[2 * n + 1];

        if (!s->tx_have_prev) {
            s->prev_i = ci;
            s->prev_q = cq;
            s->tx_have_prev = true;
        }

        int16_t iv[2] = {
            s->prev_i,
            (int16_t)(((int32_t)s->prev_i + ci) / 2)
        };
        int16_t qv[2] = {
            s->prev_q,
            (int16_t)(((int32_t)s->prev_q + cq) / 2)
        };

        for (int k = 0; k < 2; ++k) {
            int32_t c = cos_q15(s->tx_phase);
            int32_t sn = sin_q15(s->tx_phase);
            /* Match the proven hpsdrd TX convention. */
            s->tx_phase -= s->phase_inc;
            int32_t v = (int32_t)(((int64_t)iv[k] * c -
                                   (int64_t)qv[k] * sn) >> 15);
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            y[out++] = (int16_t)v;
        }

        s->prev_i = ci;
        s->prev_q = cq;
    }
    return out;
}
