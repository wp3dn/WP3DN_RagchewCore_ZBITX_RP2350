#include "audio/cw_decoder.h"
#include <math.h>
#include <string.h>

#define CWDEC_RING 2048u
#define CWDEC_BLOCK 240u          /* 10 ms at 24 kHz */
#define CWDEC_TEXT 43u            /* two 21-char OLED rows + NUL */
#define CWDEC_MAX_CODE 8u

static int16_t ringbuf[CWDEC_RING];
static volatile uint16_t wr_idx, rd_idx;
static volatile uint32_t drop_count;
static uint32_t fs_hz = 24000u;
static uint16_t center_hz = 700u;
static uint8_t ui_wpm = 20u;
static float dot_ticks = 6.0f;    /* 10-ms detector ticks */
static float noise_p, high_p;
static bool levels_ready, mark_state, signal_qualified, wpm_locked;
static uint16_t mark_ticks, space_ticks;
static char code[CWDEC_MAX_CODE + 1u];
static uint8_t code_len;
static char text[CWDEC_TEXT];
static bool space_emitted;
static int16_t tone_error_hz;
static uint8_t qualify_on_count, qualify_off_count, valid_mark_count;

struct morse_entry { const char *code; char ch; };
static const struct morse_entry morse[] = {
 {".-",'A'},{"-...",'B'},{"-.-.",'C'},{"-..",'D'},{".",'E'},
 {"..-.",'F'},{"--.",'G'},{"....",'H'},{"..",'I'},{".---",'J'},
 {"-.-",'K'},{".-..",'L'},{"--",'M'},{"-.",'N'},{"---",'O'},
 {".--.",'P'},{"--.-",'Q'},{".-.",'R'},{"...",'S'},{"-",'T'},
 {"..-",'U'},{"...-",'V'},{".--",'W'},{"-..-",'X'},{"-.--",'Y'},{"--..",'Z'},
 {".----",'1'},{"..---",'2'},{"...--",'3'},{"....-",'4'},{".....",'5'},
 {"-....",'6'},{"--...",'7'},{"---..",'8'},{"----.",'9'},{"-----",'0'},
 {"..--..",'?'},{"-..-.",'/'},{".-.-.-",'.'},{"--..--",','},{"-....-",'-'}
};

static void append_char(char c) {
    size_t n = strlen(text);
    if (n >= CWDEC_TEXT - 1u) {
        memmove(text, text + 1, CWDEC_TEXT - 2u);
        n = CWDEC_TEXT - 2u;
        text[n] = '\0';
    }
    text[n] = c;
    text[n + 1u] = '\0';
}

static void emit_code(void) {
    if (!code_len) return;
    code[code_len] = '\0';
    for (unsigned i = 0; i < sizeof(morse)/sizeof(morse[0]); ++i) {
        if (!strcmp(code, morse[i].code)) { append_char(morse[i].ch); break; }
    }
    code_len = 0u;
    code[0] = '\0';
    space_emitted = false;
}

static float goertzel_power(const int16_t *x, unsigned n, float hz) {
    float w = 2.0f * 3.14159265358979323846f * hz / (float)fs_hz;
    float coeff = 2.0f * cosf(w);
    float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f;
    for (unsigned i = 0; i < n; ++i) {
        s0 = (float)x[i] + coeff * s1 - s2;
        s2 = s1; s1 = s0;
    }
    float p = s1*s1 + s2*s2 - coeff*s1*s2;
    return p > 0.0f ? p : 0.0f;
}

static void process_detector_block(const int16_t *x) {
    static const int16_t off[5] = {-120,-60,0,60,120};
    float powers[5];
    for (int i=0;i<5;i++) {
        int f=(int)center_hz+off[i]; if(f<50)f=50;
        powers[i]=goertzel_power(x,CWDEC_BLOCK,(float)f);
    }

    /* The inner three bins are the useful decode passband.  The two outer
       bins are intentionally used as a local noise/QRM reference.  This
       makes broadband noise look uninteresting instead of "CW". */
    int best_i=1;
    float best=powers[1];
    for(int i=2;i<=3;i++) if(powers[i]>best){best=powers[i];best_i=i;}
    float outer_ref=0.5f*(powers[0]+powers[4]) + 1.0f;

    if(!levels_ready){
        noise_p=outer_ref;
        high_p=best;
        levels_ready=true;
    }
    /* Noise follows the outer-bin reference fairly quickly downward and
       conservatively upward.  It is not allowed to learn the wanted tone. */
    if(outer_ref<noise_p) noise_p=0.75f*noise_p+0.25f*outer_ref;
    else                  noise_p=0.985f*noise_p+0.015f*outer_ref;
    if(best>high_p) high_p=0.75f*high_p+0.25f*best;
    else            high_p=0.990f*high_p+0.010f*best;

    /* Qualification is deliberately stricter than mark detection:
       - useful-bin energy must stand well above the learned noise floor;
       - it must also beat the two outer reference bins;
       - several consecutive blocks are required before acquisition.
       This is what prevents plain receiver noise from driving the keyer
       timing estimator up into the 40-50 WPM region. */
    bool candidate=(best > noise_p*2.6f) && (best > outer_ref*1.8f);
    if(candidate){
        if(qualify_on_count<10u) qualify_on_count++;
        qualify_off_count=0u;
        if(qualify_on_count>=3u) signal_qualified=true;   /* 30 ms acquire */
    }else{
        qualify_on_count=0u;
        if(qualify_off_count<10u) qualify_off_count++;
        if(qualify_off_count>=2u) signal_qualified=false; /* 20 ms release */
    }

    tone_error_hz=signal_qualified?off[best_i]:0;

    /* No qualified carrier: force SPACE, but do not let noise alter WPM.
       If a real mark just ended, allow that mark to be classified once. */
    float span=high_p-noise_p;
    if(span<noise_p*0.5f) span=noise_p*0.5f;
    float on_th =noise_p+0.34f*span;
    float off_th=noise_p+0.20f*span;
    bool new_mark=signal_qualified && (mark_state?(best>off_th):(best>on_th));

    if(new_mark){
        mark_ticks++;
        if(!mark_state){
            float char_gap=2.6f*dot_ticks;
            float word_gap=6.2f*dot_ticks;
            if(code_len && (float)space_ticks>=char_gap)emit_code();
            if(!code_len && !space_emitted && (float)space_ticks>=word_gap && text[0]){
                append_char(' ');space_emitted=true;
            }
            space_ticks=0u;
        }
    }else{
        space_ticks++;
        if(mark_state){
            float threshold=1.75f*dot_ticks;
            bool dash=(float)mark_ticks>=threshold;
            if(code_len<CWDEC_MAX_CODE)code[code_len++]=dash?'-':'.';

            /* Adapt speed ONLY from marks that belonged to a qualified
               carrier.  The first few real elements acquire the WPM lock. */
            float est=dash?((float)mark_ticks/3.0f):(float)mark_ticks;
            if(est>=2.0f && est<=30.0f){
                dot_ticks=0.90f*dot_ticks+0.10f*est;
                if(valid_mark_count<255u)valid_mark_count++;
                if(valid_mark_count>=3u)wpm_locked=true;
            }
            mark_ticks=0u;
            space_ticks=0u;
        }
        float char_gap=2.8f*dot_ticks;
        float word_gap=6.5f*dot_ticks;
        if(code_len && (float)space_ticks>=char_gap)emit_code();
        if(!code_len && !space_emitted && (float)space_ticks>=word_gap && text[0]){
            append_char(' ');space_emitted=true;
        }
    }
    mark_state=new_mark;
}

void cw_decoder_init(uint32_t sample_rate_hz, uint16_t tone_hz, uint8_t initial_wpm) {
    fs_hz = sample_rate_hz ? sample_rate_hz : 24000u;
    center_hz = tone_hz ? tone_hz : 700u;
    cw_decoder_set_wpm(initial_wpm);
    cw_decoder_reset();
}
void cw_decoder_reset(void) {
    __atomic_store_n(&wr_idx,0u,__ATOMIC_RELEASE);
    __atomic_store_n(&rd_idx,0u,__ATOMIC_RELEASE);
    drop_count=0; levels_ready=false; mark_state=false; signal_qualified=false; wpm_locked=false;
    qualify_on_count=qualify_off_count=valid_mark_count=0; mark_ticks=space_ticks=0;
    noise_p=high_p=0.0f; code_len=0; code[0]='\0'; text[0]='\0'; space_emitted=false; tone_error_hz=0;
    dot_ticks = 120.0f / (float)(ui_wpm ? ui_wpm : 20u); /* 10ms ticks: 1200/WPM ms */
}
void cw_decoder_set_tone(uint16_t tone_hz) { if (tone_hz >= 100u && tone_hz <= 3000u) center_hz=tone_hz; }
void cw_decoder_set_wpm(uint8_t wpm) {
    if (wpm < 5u) wpm=5u;
    if (wpm > 60u) wpm=60u;
    ui_wpm=wpm;
    float expected = 120.0f/(float)wpm;
    if (!levels_ready) dot_ticks=expected;
}
void cw_decoder_feed(int32_t s) {
    if (s > 32767) s=32767; else if (s < -32768) s=-32768;
    uint16_t wr=__atomic_load_n(&wr_idx,__ATOMIC_RELAXED), next=(uint16_t)((wr+1u)%CWDEC_RING);
    uint16_t rd=__atomic_load_n(&rd_idx,__ATOMIC_ACQUIRE);
    if (next==rd) { drop_count++; return; }
    ringbuf[wr]=(int16_t)s; __atomic_store_n(&wr_idx,next,__ATOMIC_RELEASE);
}
void cw_decoder_process(void) {
    int16_t block[CWDEC_BLOCK];
    while (1) {
        uint16_t rd=__atomic_load_n(&rd_idx,__ATOMIC_RELAXED), wr=__atomic_load_n(&wr_idx,__ATOMIC_ACQUIRE);
        unsigned avail=(wr>=rd)?(unsigned)(wr-rd):(unsigned)(CWDEC_RING-rd+wr);
        if (avail < CWDEC_BLOCK) break;
        for (unsigned i=0;i<CWDEC_BLOCK;i++) { block[i]=ringbuf[rd]; rd=(uint16_t)((rd+1u)%CWDEC_RING); }
        __atomic_store_n(&rd_idx,rd,__ATOMIC_RELEASE);
        process_detector_block(block);
    }
}
const char *cw_decoder_text(void) { return text; }
uint8_t cw_decoder_wpm_estimate(void) {
    if (!wpm_locked) return 0u;
    if (dot_ticks < 0.5f) return 0u;
    int v=(int)(120.0f/dot_ticks+0.5f); if(v<5)v=5;if(v>60)v=60;return (uint8_t)v;
}
int16_t cw_decoder_tone_error_hz(void) { return tone_error_hz; }
uint32_t cw_decoder_drops(void) { return drop_count; }
