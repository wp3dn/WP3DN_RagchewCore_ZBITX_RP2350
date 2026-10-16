#include "audio/fft_ssb_rx.h"
#include "audio/spectrum.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

/* RadioCore Fourier Invasion #6 - Velvet Spartan RX filter
 *
 * Branches from the frozen Invasion #5 baseline.  The demodulation, LSB
 * conjugation, shared spectrum, overlap/add, and external interfaces remain
 * unchanged.  Only the RX spectral filter edges are softened by fractional
 * FFT-bin overlap weights so 50 Hz UI moves no longer snap whole 187.5 Hz
 * bins abruptly on/off.
 *
 * RadioCore Fourier Invasion #5 - Local SSB/CW RX
 *
 * Real 96 kHz AUDIO-DEMOD input -> 512 point STFT -> select only the wanted
 * sideband around the radio IF -> translate those bins directly to baseband
 * -> IFFT/overlap-add -> real audio.
 *
 * At the normal 96 kHz / 24 kHz IF operating point, the IF is exactly FFT
 * bin 128.  With a 256-sample hop, that translation is phase-continuous from
 * frame to frame without a time-domain NCO or Hilbert phasing network.
 */
#define N 512u
#define HOP 256u
#define PI_F 3.14159265358979323846f

typedef struct { float r, i; } cpx;

static cpx bins[N];
static float window[N];
static float inbuf[N];
static float hopbuf[HOP];
static float ola[N];
static float ola_i[N];
static float am_env_dc;
static int16_t outq[HOP];
static uint16_t in_count;
static uint16_t out_pos;
static uint16_t out_count;
static uint32_t fs = 96000u;
static uint32_t if_hz = 24000u;
static uint16_t filter_low_hz = 300u;
static uint16_t filter_high_hz = 2500u;
static bool primed;
static bool nr_enabled;
static float nr_floor[N/2u + 1u];

static void fft(cpx *x, bool inverse) {
    for (unsigned i = 1, j = 0; i < N; ++i) {
        unsigned bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            cpx t = x[i];
            x[i] = x[j];
            x[j] = t;
        }
    }

    for (unsigned len = 2; len <= N; len <<= 1) {
        const float ang = (inverse ? 2.0f : -2.0f) * PI_F / (float)len;
        const float wr0 = cosf(ang);
        const float wi0 = sinf(ang);
        for (unsigned i = 0; i < N; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (unsigned j = 0; j < len / 2; ++j) {
                const cpx u = x[i + j];
                const cpx v = x[i + j + len / 2];
                const float vr = v.r * wr - v.i * wi;
                const float vi = v.r * wi + v.i * wr;
                x[i + j].r = u.r + vr;
                x[i + j].i = u.i + vi;
                x[i + j + len / 2].r = u.r - vr;
                x[i + j + len / 2].i = u.i - vi;
                const float nwr = wr * wr0 - wi * wi0;
                wi = wr * wi0 + wi * wr0;
                wr = nwr;
            }
        }
    }

    if (inverse) {
        for (unsigned i = 0; i < N; ++i) {
            x[i].r /= (float)N;
            x[i].i /= (float)N;
        }
    }
}

static int16_t sat16(float v) {
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

/* Velvet Spartan: fractional boundary-bin weighting.
 *
 * Treat each FFT bin as covering one bin-width centered on k*fs/N and return
 * the fraction of that interval which lies inside the requested audio
 * passband.  Interior bins remain exactly 1.0, exterior bins remain 0.0, and
 * only the edge bins become fractional.  This preserves the very sharp
 * Fourier selectivity while making 50 Hz filter-setting changes continuous
 * instead of snapping whole 187.5 Hz bins on/off.
 */
static float passband_bin_weight(unsigned k) {
    const float bin_hz = (float)fs / (float)N;
    const float center = (float)k * bin_hz;
    const float bin_lo = center - 0.5f * bin_hz;
    const float bin_hi = center + 0.5f * bin_hz;
    const float pass_lo = (float)filter_low_hz;
    const float pass_hi = (float)filter_high_hz;

    float overlap_lo = (bin_lo > pass_lo) ? bin_lo : pass_lo;
    float overlap_hi = (bin_hi < pass_hi) ? bin_hi : pass_hi;
    float overlap = overlap_hi - overlap_lo;
    if (overlap <= 0.0f) return 0.0f;
    if (overlap >= bin_hz) return 1.0f;
    return overlap / bin_hz;
}

/* AM upper-edge weight.  Unlike SSB/CW, AM must keep the carrier and audio
 * all the way down to DC.  Only the requested +/- AM bandwidth edge is
 * fractionally weighted by the Velvet Spartan rule. */
static float am_bin_weight(unsigned k) {
    const float bin_hz = (float)fs / (float)N;
    const float center = (float)k * bin_hz;
    const float bin_lo = center - 0.5f * bin_hz;
    const float bin_hi = center + 0.5f * bin_hz;
    const float pass_hi = (float)filter_high_hz;

    if (k == 0u) return 1.0f;
    float overlap_lo = bin_lo > 0.0f ? bin_lo : 0.0f;
    float overlap_hi = (bin_hi < pass_hi) ? bin_hi : pass_hi;
    float overlap = overlap_hi - overlap_lo;
    if (overlap <= 0.0f) return 0.0f;
    if (overlap >= bin_hz) return 1.0f;
    return overlap / bin_hz;
}

static void make_block(fft_rx_sideband_t sb) {
    for (unsigned i = 0; i < N; ++i) {
        bins[i].r = inbuf[i] * window[i];
        bins[i].i = 0.0f;
    }
    fft(bins, false);

    /* Keep a copy of the positive-frequency real-IF spectrum before bins[]
       becomes the output spectrum. */
    cpx src[N / 2u + 1u];
    memcpy(src, bins, sizeof(src));

    /* Invasion #4: the main spectrum now comes from THIS SAME RX FFT.
       Display width is +/-12 kHz around IF.  At 187.5 Hz/FFT bin that is
       128 source bins, so combine adjacent pairs into the existing 64-pixel
       spectrum packet (375 Hz/pixel), matching the old display scale. */
    float wide_power[SPECTRUM_WIDE_BINS];
    const int ifbin_wide = (int)(((uint64_t)if_hz * N + fs / 2u) / fs);
    const int wide_first = ifbin_wide - 64;
    for (unsigned x=0; x<SPECTRUM_WIDE_BINS; ++x) {
        int b=wide_first+(int)x;
        wide_power[x]=(b>0 && b<(int)(N/2u)) ?
            (src[b].r*src[b].r + src[b].i*src[b].i) : 0.0f;
    }
    spectrum_feed_fourier_wide_power(wide_power);

    float display_power[SPECTRUM_BINS];
    const int ifbin_display = (int)(((uint64_t)if_hz * N + fs / 2u) / fs);
    const int first = ifbin_display - 64;
    for (unsigned x = 0; x < SPECTRUM_BINS; ++x) {
        int b0 = first + (int)(2u * x);
        int b1 = b0 + 1;
        float pwr = 0.0f;
        if (b0 > 0 && b0 < (int)(N/2u)) pwr += src[b0].r*src[b0].r + src[b0].i*src[b0].i;
        if (b1 > 0 && b1 < (int)(N/2u)) pwr += src[b1].r*src[b1].r + src[b1].i*src[b1].i;
        display_power[x] = pwr;
    }
    spectrum_feed_fourier_power(display_power);

    memset(bins, 0, sizeof(bins));

    const unsigned ifbin = (unsigned)(((uint64_t)if_hz * N + fs / 2u) / fs);

    if (sb == FFT_RX_AM) {
        /* Invasion #7A: proper AM detector.
         *
         * Keep the carrier plus BOTH RF sidebands, but translate the filtered
         * positive-IF channel to a COMPLEX baseband spectrum.  Do not impose
         * conjugate symmetry here: the IFFT must produce analytic I/Q so the
         * following magnitude detector is phase-insensitive like a normal AM
         * envelope detector. */
        if (ifbin > 0u && ifbin < N/2u) bins[0] = src[ifbin];

        unsigned hi = (unsigned)(((uint64_t)filter_high_hz * N + fs - 1u) / fs);
        if (hi < N/2u - 2u) ++hi;
        else hi = N/2u - 1u;

        for (unsigned k = 1u; k <= hi; ++k) {
            const float w = am_bin_weight(k);
            if (w <= 0.0f) continue;
            const int up = (int)ifbin + (int)k;
            const int dn = (int)ifbin - (int)k;
            if (up > 0 && up < (int)(N/2u)) {
                bins[k].r = src[up].r * w;
                bins[k].i = src[up].i * w;
            }
            if (dn > 0 && dn < (int)(N/2u)) {
                /* Negative complex-baseband frequency lives at N-k. */
                bins[N-k].r = src[dn].r * w;
                bins[N-k].i = src[dn].i * w;
            }
        }
    } else {
        /* Include one bin beyond each nominal edge because that bin can overlap
           the requested passband fractionally. */
        unsigned lo = (unsigned)(((uint64_t)filter_low_hz * N) / fs);
        unsigned hi = (unsigned)(((uint64_t)filter_high_hz * N + fs - 1u) / fs);
        if (lo > 1u) --lo;
        else lo = 1u;
        if (hi < N / 2u - 2u) ++hi;
        else hi = N / 2u - 1u;

        for (unsigned k = lo; k <= hi; ++k) {
            const float w = passband_bin_weight(k);
            if (w <= 0.0f) continue;

            int src_bin = (sb == FFT_RX_USB)
                ? (int)ifbin + (int)k
                : (int)ifbin - (int)k;
            if (src_bin <= 0 || src_bin >= (int)(N / 2u)) continue;

            /* Translate the selected RF/IF sideband directly to positive audio
               bin k, then restore conjugate symmetry so the IFFT output is real. */
            cpx v = src[src_bin];
            if (sb == FFT_RX_LSB) v.i = -v.i;

            /* Poor Soul NR: deliberately gentle spectral suppression using the
               FFT we already paid for.  Track a slow per-audio-bin floor and
               attenuate only bins living close to it.  Strong CW/voice bins
               pass unchanged.  Maximum cut is about 9.5 dB. */
            if (nr_enabled) {
                float pwr = v.r*v.r + v.i*v.i;
                float *nf = &nr_floor[k];
                if (*nf <= 0.0f) *nf = pwr + 1.0f;
                float a = (pwr < *nf) ? 0.08f : 0.002f;
                *nf += a * (pwr - *nf);
                float ratio = pwr / (*nf + 1.0f);
                float ng = ratio < 1.6f ? 0.33f : (ratio < 3.0f ? 0.65f : 1.0f);
                v.r *= ng; v.i *= ng;
            }
            v.r *= w;
            v.i *= w;
            bins[k] = v;
            bins[N - k].r = v.r;
            bins[N - k].i = -v.i;
        }
    }

    fft(bins, true);

    /* 50% Hann analysis-window overlap-add.  No synthesis window is needed:
       adjacent Hann windows sum to unity at this hop size. */
    for (unsigned i = 0; i < N; ++i) {
        ola[i] += bins[i].r;
        if (sb == FFT_RX_AM) ola_i[i] += bins[i].i;
    }
    for (unsigned i = 0; i < HOP; ++i) {
        if (sb == FFT_RX_AM) {
            /* Conventional AM envelope detection on the reconstructed complex
             * channel.  The very-slow DC tracker removes the carrier/envelope
             * pedestal while retaining normal AM audio down to a few Hz. */
            const float env = sqrtf(ola[i] * ola[i] + ola_i[i] * ola_i[i]);
            am_env_dc += (env - am_env_dc) * (1.0f / 4096.0f);
            outq[i] = sat16(env - am_env_dc);
        } else {
            /* Conservative 0.5 scale leaves headroom before the existing AGC. */
            outq[i] = sat16(ola[i] * 0.5f);
        }
    }
    memmove(ola, ola + HOP, (N - HOP) * sizeof(float));
    memset(ola + (N - HOP), 0, HOP * sizeof(float));
    memmove(ola_i, ola_i + HOP, (N - HOP) * sizeof(float));
    memset(ola_i + (N - HOP), 0, HOP * sizeof(float));
    out_pos = 0;
    out_count = HOP;
}

void fft_ssb_rx_init(uint32_t sample_rate, uint32_t center_if_hz) {
    fs = sample_rate ? sample_rate : 96000u;
    if_hz = center_if_hz ? center_if_hz : 24000u;
    for (unsigned i = 0; i < N; ++i) {
        window[i] = 0.5f - 0.5f * cosf(2.0f * PI_F * (float)i / (float)N);
    }
    fft_ssb_rx_reset();
}

void fft_ssb_rx_set_if_hz(uint32_t center_if_hz) {
    if (center_if_hz >= 1000u && center_if_hz < fs / 2u) if_hz = center_if_hz;
}

void fft_ssb_rx_set_filter(uint16_t low_hz, uint16_t high_hz) {
    if (low_hz < 50u) low_hz = 50u;
    if (high_hz > 4000u) high_hz = 4000u;
    if (high_hz <= low_hz) return;
    filter_low_hz = low_hz;
    filter_high_hz = high_hz;
}

void fft_ssb_rx_reset(void) {
    memset(inbuf, 0, sizeof(inbuf));
    memset(hopbuf, 0, sizeof(hopbuf));
    memset(ola, 0, sizeof(ola));
    memset(ola_i, 0, sizeof(ola_i));
    am_env_dc = 0.0f;
    memset(outq, 0, sizeof(outq));
    in_count = 0;
    out_pos = 0;
    out_count = 0;
    primed = false;
}

int16_t fft_ssb_rx_process(int32_t sample, fft_rx_sideband_t sb) {
    const float s = (float)sample;

    if (!primed) {
        inbuf[in_count++] = s;
        if (in_count == N) {
            primed = true;
            in_count = 0;
            make_block(sb);
        }
    } else {
        hopbuf[in_count++] = s;
        if (in_count == HOP) {
            memmove(inbuf, inbuf + HOP, (N - HOP) * sizeof(float));
            memcpy(inbuf + (N - HOP), hopbuf, HOP * sizeof(float));
            in_count = 0;
            make_block(sb);
        }
    }

    if (out_count) {
        const int16_t y = outq[out_pos++];
        --out_count;
        return y;
    }
    return 0;
}

void fft_ssb_rx_set_noise_reduction(bool enabled) { nr_enabled=enabled; if(!enabled) memset(nr_floor,0,sizeof(nr_floor)); }
bool fft_ssb_rx_get_noise_reduction(void) { return nr_enabled; }
