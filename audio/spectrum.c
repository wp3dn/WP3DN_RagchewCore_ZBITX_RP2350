#include "audio/spectrum.h"

#include <math.h>
#include <string.h>

#include "hardware/sync.h"

#define FFT_N 64u
#define DISPLAY_HEIGHT 18u

static uint8_t display_bins_[SPECTRUM_BINS];
static volatile float fourier_power_[SPECTRUM_BINS];
static volatile float fourier_wide_power_[SPECTRUM_WIDE_BINS];
static uint8_t wide_display_bins_[SPECTRUM_WIDE_BINS];
static volatile uint32_t fourier_sequence_;
static volatile bool fourier_tx_source_;
static uint32_t fourier_processed_sequence_;
static int16_t offset_db_ = 5;
static float tracked_noise_db_;
static bool noise_valid_;

void spectrum_init(void) {
    spectrum_reset();
}

void spectrum_reset(void) {
    uint32_t irq = save_and_disable_interrupts();
    restore_interrupts(irq);
    fourier_sequence_ = 0;
    fourier_processed_sequence_ = 0;
    fourier_tx_source_ = false;
    memset((void *)fourier_power_, 0, sizeof(fourier_power_));
    memset((void *)fourier_wide_power_, 0, sizeof(fourier_wide_power_));
    memset(wide_display_bins_, 0, sizeof(wide_display_bins_));
    memset(display_bins_, 0, sizeof(display_bins_));
    tracked_noise_db_ = 0.0f;
    noise_valid_ = false;
}

/* Legacy sample-fed 64-point DFT removed: current spectrum uses Fourier power. */
void spectrum_feed(int32_t i_sample, int32_t q_sample) {
    (void)i_sample;
    (void)q_sample;
}

static void spectrum_feed_fourier_power_source(const float power[SPECTRUM_BINS], bool tx_source) {
    /* Called once per 256-sample Fourier hop.  Keep the source tag with the
       frame so RX and TX can use independent vertical display scales. */
    uint32_t irq = save_and_disable_interrupts();
    for (uint32_t i = 0; i < SPECTRUM_BINS; ++i) fourier_power_[i] = power[i];
    fourier_tx_source_ = tx_source;
    fourier_sequence_++;
    restore_interrupts(irq);
}

void spectrum_feed_fourier_power(const float power[SPECTRUM_BINS]) {
    spectrum_feed_fourier_power_source(power, false);
}

void spectrum_feed_fourier_wide_power(const float power[SPECTRUM_WIDE_BINS]) {
    uint32_t irq = save_and_disable_interrupts();
    for (uint32_t i = 0; i < SPECTRUM_WIDE_BINS; ++i) fourier_wide_power_[i] = power[i];
    restore_interrupts(irq);
}

void spectrum_feed_fourier_tx_power(const float power[SPECTRUM_BINS]) {
    spectrum_feed_fourier_power_source(power, true);
}

bool spectrum_process(void) {
    /* Invasion #4: Local Fourier SSB RX already owns a 512-point FFT.
       Consume its 64-bin +/-12 kHz power snapshot instead of running the
       legacy display DFT a second time. */
    if (fourier_sequence_ != fourier_processed_sequence_) {
        float db[FFT_N];
        uint32_t seq;
        bool tx_source;
        uint32_t irq = save_and_disable_interrupts();
        seq = fourier_sequence_;
        tx_source = fourier_tx_source_;
        for (uint32_t i = 0; i < FFT_N; ++i)
            db[i] = 10.0f * log10f(fourier_power_[i] + 1.0f);
        restore_interrupts(irq);
        fourier_processed_sequence_ = seq;

        float sorted[FFT_N];
        memcpy(sorted, db, sizeof(sorted));
        for (uint32_t i = 1; i < FFT_N; ++i) {
            float v = sorted[i]; uint32_t j = i;
            while (j > 0 && sorted[j - 1] > v) { sorted[j] = sorted[j - 1]; --j; }
            sorted[j] = v;
        }
        const float measured_noise = 0.5f * (sorted[FFT_N/2u-1u] + sorted[FFT_N/2u]);
        /* RX keeps the proven 24 dB scale/noise tracker.  TX deliberately does
           not disturb RX noise tracking and gets its own much wider 96 dB span. */
        if (!tx_source) {
            if (!noise_valid_) { tracked_noise_db_ = measured_noise; noise_valid_ = true; }
            else tracked_noise_db_ += 0.025f * (measured_noise - tracked_noise_db_);
        }
        const float floor_db = tx_source ? measured_noise : (tracked_noise_db_ - (float)offset_db_);
        const float visible_range_db = tx_source ? 96.0f : 24.0f;

        /* Stock ZBITX panel path: keep all 128 native +/-12 kHz RX bins.
           Deliberately no horizontal blur and no slow peak decay here: JJ's
           waterfall already supplies the visual history.  The old 64-bin
           FFT-fed 64-bin view is retained for compatibility. */
        if (!tx_source) {
            float wp[SPECTRUM_WIDE_BINS];
            uint32_t irq2 = save_and_disable_interrupts();
            for (uint32_t i=0; i<SPECTRUM_WIDE_BINS; ++i) wp[i]=fourier_wide_power_[i];
            restore_interrupts(irq2);
            for (uint32_t i=0; i<SPECTRUM_WIDE_BINS; ++i) {
                float wdb=10.0f*log10f(wp[i]+1.0f);
                float n=(wdb-floor_db)/visible_range_db;
                if(n<0.0f)n=0.0f; if(n>1.0f)n=1.0f;
                wide_display_bins_[i]=(uint8_t)(n*(float)DISPLAY_HEIGHT+0.5f);
            }
        }

        float raw_height[FFT_N];
        for (uint32_t x=0; x<FFT_N; ++x) {
            float normalized=(db[x]-floor_db)/visible_range_db;
            if(normalized<0.0f) normalized=0.0f; if(normalized>1.0f) normalized=1.0f;
            raw_height[x]=normalized*(float)DISPLAY_HEIGHT;
        }
        for(uint32_t x=0;x<FFT_N;++x){
            uint32_t l=x?x-1u:0u, r=(x+1u<FFT_N)?x+1u:FFT_N-1u;
            float h=(raw_height[l]+2.0f*raw_height[x]+raw_height[r])*0.25f;
            uint8_t target=(uint8_t)(h+0.5f), old=display_bins_[x];
            display_bins_[x]=target>=old?target:(uint8_t)((3u*old+target)/4u);
        }
        return true;
    }

    return false;
}

void spectrum_get_bins(uint8_t bins[SPECTRUM_BINS]) {
    memcpy(bins, display_bins_, SPECTRUM_BINS);
}

void spectrum_get_wide_bins(uint8_t bins[SPECTRUM_WIDE_BINS]) {
    memcpy(bins, wide_display_bins_, SPECTRUM_WIDE_BINS);
}

void spectrum_set_offset_db(int16_t offset_db) {
    if (offset_db < -20) offset_db = -20;
    if (offset_db > 20) offset_db = 20;
    offset_db_ = offset_db;
}

int16_t spectrum_get_offset_db(void) {
    return offset_db_;
}
