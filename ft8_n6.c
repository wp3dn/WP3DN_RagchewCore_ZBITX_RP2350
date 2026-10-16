#include "ft8_n6.h"
#include "ui/front_panel.h"
#include "i2cbb/zbitx_bus.h"
#include "pico/stdlib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ft8_lib/ft8/decode.h"
#include "ft8_lib/ft8/constants.h"
#include "fft_pico/kiss_fftr.h"

#define FS 6000
#define BS 960
#define TOSR 2
#define FOSR 2
#define NFFT 1920
#define NBINS 480
#define MAX_BLOCKS 93
#define WF_BYTES (MAX_BLOCKS*TOSR*FOSR*NBINS)
#define MAX_CAND 120
#define MAX_DEC 50
#define KISS_WORK 32768

typedef struct { float symbol_period,fft_norm; int block_size,subblock_size,nfft; float *window,*last_frame; waterfall_t wf; kiss_fftr_cfg cfg; float max_mag; } mon_t;
static float win[NFFT], last[NFFT], frame[BS];
static kiss_fft_scalar timedata[NFFT];
static kiss_fft_cpx freqdata[NFFT/2+1];
static uint8_t waterfall[WF_BYTES];
static uint8_t kisswork[KISS_WORK] __attribute__((aligned(8)));
static candidate_t cand[MAX_CAND];
static message_t seen[MAX_DEC];
static mon_t mon;
static bool inited=false;

static float hann(int i,int n){float x=sinf((float)M_PI*i/n);return x*x;}
static int bcd(uint8_t v){return ((v>>4)*10)+(v&15);}
static void rtc_text(char out[7]){uint8_t reg=0,r[3];int h=0,m=0,s=0;i2cbb_t*b=zbitx_bus();if(b&&i2cbb_write_read(b,0x68,&reg,1,r,3)){s=bcd(r[0]&0x7f);m=bcd(r[1]&0x7f);h=bcd(r[2]&0x3f);}snprintf(out,7,"%02d%02d%02d",h,m,s);}

void ft8_n6_init(void){
 memset(&mon,0,sizeof(mon)); mon.symbol_period=FT8_SYMBOL_PERIOD; mon.block_size=BS; mon.subblock_size=BS/TOSR; mon.nfft=NFFT; mon.fft_norm=2.0f/NFFT; mon.window=win; mon.last_frame=last;
 mon.wf.max_blocks=MAX_BLOCKS;mon.wf.num_blocks=0;mon.wf.num_bins=NBINS;mon.wf.time_osr=TOSR;mon.wf.freq_osr=FOSR;mon.wf.block_stride=TOSR*FOSR*NBINS;mon.wf.protocol=PROTO_FT8;mon.wf.mag=waterfall;
 size_t need=0;kiss_fftr_alloc(NFFT,0,NULL,&need);if(need>KISS_WORK){printf("FT8: N6 KISS work too small need=%u\n",(unsigned)need);return;}size_t supplied=KISS_WORK;mon.cfg=kiss_fftr_alloc(NFFT,0,kisswork,&supplied);if(!mon.cfg){printf("FT8: N6 FFT init failed\n");return;}
 for(int i=0;i<NFFT;i++)win[i]=hann(i,NFFT);inited=true;printf("FT8: N6 full RX ready fs=6000 nfft=1920 static=%u\n",(unsigned)need);
}
static void reset_mon(void){memset(last,0,sizeof(last));memset(waterfall,0,sizeof(waterfall));mon.wf.num_blocks=0;mon.max_mag=-120.0f;}
static void process_block(const float *f){if(mon.wf.num_blocks>=MAX_BLOCKS)return;int off=mon.wf.num_blocks*mon.wf.block_stride,fp=0;for(int ts=0;ts<TOSR;ts++){memmove(last,last+mon.subblock_size,(NFFT-mon.subblock_size)*sizeof(float));for(int p=NFFT-mon.subblock_size;p<NFFT;p++)last[p]=f[fp++];for(int p=0;p<NFFT;p++)timedata[p]=mon.fft_norm*win[p]*last[p];kiss_fftr(mon.cfg,timedata,freqdata);for(int fs=0;fs<FOSR;fs++)for(int bin=0;bin<NBINS;bin++){int sb=bin*FOSR+fs;float re=freqdata[sb].r,im=freqdata[sb].i;float db=10.0f*log10f(1e-12f+re*re+im*im);if(db>mon.max_mag)mon.max_mag=db;int v=(int)(2*db+240);waterfall[off++]=(uint8_t)(v<0?0:(v>255?255:v));}}mon.wf.num_blocks++;}
int ft8_n6_decode_capture(const int8_t*s,size_t count,const char *slot_time,void (*publish)(const char *)){if(!inited)return -1;absolute_time_t t0=get_absolute_time();reset_mon();size_t pos=0;while(pos+BS<=count&&mon.wf.num_blocks<MAX_BLOCKS){for(int i=0;i<BS;i++)frame[i]=(float)s[pos+i]/128.0f;process_block(frame);pos+=BS;}
printf("FT8: N6 waterfall blocks=%d samples=%u max=%.1f\n",mon.wf.num_blocks,(unsigned)count,(double)mon.max_mag);
 int nc=ft8_find_sync(&mon.wf,MAX_CAND,cand,10);printf("FT8: N6 candidates=%d\n",nc);int ns=0,sent=0;char tm[7];snprintf(tm,sizeof(tm),"%s",slot_time?slot_time:"000000");memset(seen,0,sizeof(seen));
 for(int k=0;k<nc&&ns<MAX_DEC;k++){message_t msg;decode_status_t st;if(!ft8_decode(&mon.wf,&cand[k],&msg,10,&st))continue;bool dup=false;for(int j=0;j<ns;j++)if(seen[j].hash==msg.hash&&!strcmp(seen[j].text,msg.text)){dup=true;break;}if(dup)continue;seen[ns++]=msg;float hz=(cand[k].freq_offset+(float)cand[k].freq_sub/FOSR)/mon.symbol_period;if(hz<100||hz>2950)continue;char line[96];
    /* FT8 nominal transmission begins about 0.5 s into its UTC slot. */
    float dt=((float)cand[k].time_offset+(float)cand[k].time_sub/TOSR)*mon.symbol_period-0.5f;
    /* Stock panel requires HHMMSS at offsets 0..5, slot seconds at 6..7,
       and renders from byte 12. Preserve its four whitespace-separated
       header tokens: time, confidence, strength, audio frequency. */
    snprintf(line,sizeof(line),"#G%s 0 0 %.0f ~ %+d %+.1f %s",tm,(double)hz,(int)cand[k].snr,(double)dt,msg.text);
    if(publish)publish(line);sent++;}
 int64_t us=absolute_time_diff_us(t0,get_absolute_time());printf("FT8: N6 decoded=%d sent=%d total=%lld ms\n",ns,sent,(long long)(us/1000));return sent;}
