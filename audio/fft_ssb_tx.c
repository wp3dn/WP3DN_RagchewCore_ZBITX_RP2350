#include "audio/fft_ssb_tx.h"
#include "audio/spectrum.h"
#include <math.h>
#include <string.h>

/* RadioCore Fourier invasion #1
 * 512 point float STFT, 50% overlap, Hann analysis window.
 * At 96 kHz, bins are 187.5 Hz and 24 kHz is EXACTLY bin 128.
 * We keep voice bins 375..3000 Hz and move them directly around the IF bin.
 * No Hilbert transformer and no time-domain quadrature NCO are used.
 */
#define N 512
#define HOP 256
#define PI_F 3.14159265358979323846f

typedef struct { float r,i; } cpx;
static cpx a[N];
static float window[N];
static float inbuf[N];
static float hopbuf[HOP];
static float ola[N];
static int16_t outq[HOP];
static uint16_t in_count, out_pos, out_count;
static uint32_t fs=96000, if_hz=24000;
static uint16_t filter_low_hz=400, filter_high_hz=2800;
static volatile uint8_t am_carrier_percent=50u;
static bool primed;

static void fft(cpx *x, bool inverse){
    for(unsigned i=1,j=0;i<N;i++){
        unsigned bit=N>>1;
        for(; j&bit; bit>>=1) j^=bit;
        j^=bit;
        if(i<j){ cpx t=x[i]; x[i]=x[j]; x[j]=t; }
    }
    for(unsigned len=2;len<=N;len<<=1){
        float ang=(inverse?2.0f:-2.0f)*PI_F/(float)len;
        float wr0=cosf(ang), wi0=sinf(ang);
        for(unsigned i=0;i<N;i+=len){
            float wr=1.0f, wi=0.0f;
            for(unsigned j=0;j<len/2;j++){
                cpx u=x[i+j], v=x[i+j+len/2];
                float vr=v.r*wr-v.i*wi, vi=v.r*wi+v.i*wr;
                x[i+j].r=u.r+vr; x[i+j].i=u.i+vi;
                x[i+j+len/2].r=u.r-vr; x[i+j+len/2].i=u.i-vi;
                float nwr=wr*wr0-wi*wi0;
                wi=wr*wi0+wi*wr0; wr=nwr;
            }
        }
    }
    if(inverse) for(unsigned i=0;i<N;i++){ x[i].r/=N; x[i].i/=N; }
}

static int16_t sat16(float v){
    if(v>32767.0f) return 32767;
    if(v<-32768.0f) return -32768;
    return (int16_t)lrintf(v);
}


static void publish_tx_spectrum(const cpx *spec){
    /* Reuse the existing 64-bin +/-12 kHz display packet.  This snapshot is
       taken from the actual Fourier TX spectrum immediately before IFFT, so
       the front panel shows the modulation spectrum that is being sent to
       AUDIO-MOD rather than a synthetic meter. */
    float display_power[SPECTRUM_BINS];
    const int ifbin_display=(int)(((uint64_t)if_hz*N + fs/2u)/fs);
    const int first=ifbin_display-64;
    for(unsigned x=0;x<SPECTRUM_BINS;++x){
        int b0=first+(int)(2u*x);
        int b1=b0+1;
        float pwr=0.0f;
        if(b0>0 && b0<(int)(N/2)){ pwr += spec[b0].r*spec[b0].r + spec[b0].i*spec[b0].i; }
        if(b1>0 && b1<(int)(N/2)){ pwr += spec[b1].r*spec[b1].r + spec[b1].i*spec[b1].i; }
        display_power[x]=pwr;
    }
    spectrum_feed_fourier_tx_power(display_power);
}

static void make_block(fft_ssb_sideband_t sb){
    for(unsigned i=0;i<N;i++){ a[i].r=inbuf[i]*window[i]; a[i].i=0; }
    fft(a,false);
    cpx src[N/2+1];
    memcpy(src,a,sizeof(src));
    memset(a,0,sizeof(a));

    const unsigned ifbin=(unsigned)(((uint64_t)if_hz*N + fs/2)/fs);
    /* Invasion #2: use the existing RadioCore TX filter controls.
       The FFT bin spacing is 187.5 Hz at 96 kHz / 512, so the requested
       edges are mapped to the nearest usable spectral bins. */
    unsigned lo=(unsigned)(((uint64_t)filter_low_hz*N + fs/2)/fs);
    unsigned hi=(unsigned)(((uint64_t)filter_high_hz*N + fs/2)/fs);
    if(lo<1) lo=1;
    if(hi>N/2-1) hi=N/2-1;

    if(sb==FFT_SSB_AM){
        /* Fourier AM: carrier plus both sidebands.  Keep modulation
           conservative so carrier + peaks retain DAC headroom. */
        const float mod_scale=0.35f;
        for(unsigned k=lo;k<=hi;k++){
            int up=(int)ifbin+(int)k, dn=(int)ifbin-(int)k;
            if(up<=0 || up>=(int)N/2 || dn<=0 || dn>=(int)N/2) continue;
            cpx v=src[k]; v.r*=mod_scale; v.i*=mod_scale;
            a[up]=v;
            a[dn].r=v.r; a[dn].i=-v.i;
            a[N-up].r=a[up].r; a[N-up].i=-a[up].i;
            a[N-dn].r=a[dn].r; a[N-dn].i=-a[dn].i;
        }
        /* A real cosine of peak A has N*A/2 in each FFT carrier bin.
           100% here is intentionally capped at 12000 peak counts. */
        float camp=12000.0f*((float)am_carrier_percent/100.0f);
        a[ifbin].r += camp*(float)N*0.5f;
        a[N-ifbin].r += camp*(float)N*0.5f;
    }else{
        for(unsigned k=lo;k<=hi;k++){
            int d=(sb==FFT_SSB_USB) ? (int)ifbin+(int)k : (int)ifbin-(int)k;
            if(d<=0 || d>=(int)N/2) continue;
            /* Positive-frequency half of real microphone spectrum. */
            a[d]=src[k];
            a[N-d].r=src[k].r;
            a[N-d].i=-src[k].i;
        }
    }

    /* Invasion #10: publish the real Local Fourier TX spectrum. */
    publish_tx_spectrum(a);
    fft(a,true);

    for(unsigned i=0;i<N;i++) ola[i]+=a[i].r;
    for(unsigned i=0;i<HOP;i++) outq[i]=sat16(ola[i]);
    memmove(ola,ola+HOP,(N-HOP)*sizeof(float));
    memset(ola+(N-HOP),0,HOP*sizeof(float));
    out_pos=0; out_count=HOP;
}

void fft_ssb_tx_init(uint32_t sample_rate,uint32_t center_if){
    fs=sample_rate?sample_rate:96000;
    if_hz=center_if?center_if:24000;
    for(unsigned i=0;i<N;i++) window[i]=0.5f-0.5f*cosf(2.0f*PI_F*(float)i/(float)N);
    fft_ssb_tx_reset();
}

void fft_ssb_tx_set_filter(uint16_t low_hz, uint16_t high_hz){
    if(low_hz < 50u) low_hz=50u;
    if(high_hz > 4000u) high_hz=4000u;
    if(high_hz <= low_hz) return;
    filter_low_hz=low_hz;
    filter_high_hz=high_hz;
}

void fft_ssb_tx_set_am_carrier(uint8_t percent){ if(percent>100u) percent=100u; am_carrier_percent=percent; }
uint8_t fft_ssb_tx_get_am_carrier(void){ return am_carrier_percent; }

void fft_ssb_tx_reset(void){
    memset(inbuf,0,sizeof(inbuf)); memset(ola,0,sizeof(ola)); memset(outq,0,sizeof(outq));
    in_count=0; out_pos=0; out_count=0; primed=false;
}
int16_t fft_ssb_tx_process(int16_t sample, fft_ssb_sideband_t sb){
    /* modest safe input lift; unlike the old int16_t <<=3 this cannot wrap */
    float s=(float)sample*2.0f;
    if(!primed){
        inbuf[in_count++]=s;
        if(in_count==N){ primed=true; in_count=0; make_block(sb); }
    }else{
        hopbuf[in_count++]=s;
        if(in_count==HOP){
            memmove(inbuf,inbuf+HOP,(N-HOP)*sizeof(float));
            memcpy(inbuf+(N-HOP),hopbuf,HOP*sizeof(float));
            in_count=0;
            make_block(sb);
        }
    }
    if(out_count){ int16_t y=outq[out_pos++]; out_count--; return y; }
    return 0;
}
