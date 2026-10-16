#include "ui/front_panel.h"
#include "i2cbb/zbitx_bus.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Stock ZBITX front-panel adapter.  The panel is an I2C slave at 0x0a.
   Radio -> panel: one write beginning with '{', followed by LABEL value}
   fields. Panel -> radio: one read response, byte 0 length then text. */

#include "tusb.h"

static i2cbb_t *bb;
static absolute_time_t next_wf, next_poll, next_meter;
static bool full_state_pending=true;
static char state_cache[18][20];
static int last_usb_p1_mode=-1;
static char last_cw[43];
static uint32_t panel_span_hz=25000u;
static ui_event_t ft8_command_event(const char *cmd,const char *value){
 ui_event_t e={0};e.type=UI_EVENT_FT8_COMMAND;
 snprintf(e.ft8_command,sizeof(e.ft8_command),"%s%s%s",cmd,*value?" ":"",value);return e;
}

static const uint32_t steps[8]={10,50,100,500,1000,2500,5000,10000};

static bool send_payload(const char *p, size_t n){
    uint8_t b[1+300];
    if(n>300) return false;
    b[0]='{'; memcpy(b+1,p,n);
    return i2cbb_write(bb,ZBITX_PANEL_ADDR,b,(int)n+1);
}
static bool send_text(const char *p){ return send_payload(p,strlen(p)); }

static const char *panel_mode(uint8_t m){
    switch(m){case 0:return "USB";case 1:return "LSB";case 2:return "CW";case 3:return "CWR";case 4:return "AM";case 5:return "2TONE";case 6:return "FT8";default:return "USB";}
}
static int mode_from_panel(const char *v){
    if(!strcmp(v,"USB"))return 0;if(!strcmp(v,"LSB"))return 1;if(!strcmp(v,"CW")||!strcmp(v,"CWU"))return 2;
    if(!strcmp(v,"CWR")||!strcmp(v,"CWL"))return 3;if(!strcmp(v,"AM"))return 4;if(!strcmp(v,"2TONE"))return 5;if(!strcmp(v,"FT8"))return 6;return -1;
}
static int step_index(uint32_t x){for(int i=0;i<8;i++)if(steps[i]==x)return i;return -1;}
static uint32_t step_from_panel(const char *v){
    if(!strcmp(v,"10H"))return 10; if(!strcmp(v,"50H"))return 50; if(!strcmp(v,"100H"))return 100;
    if(!strcmp(v,"500H"))return 500; if(!strcmp(v,"1K"))return 1000; if(!strcmp(v,"2.5K"))return 2500;
    if(!strcmp(v,"5K"))return 5000; if(!strcmp(v,"10K"))return 10000; return (uint32_t)strtoul(v,NULL,10);
}
static const char *step_to_panel(uint32_t x){
    switch(x){case 10:return "10H";case 50:return "50H";case 100:return "100H";case 500:return "500H";case 1000:return "1K";case 2500:return "2.5K";case 5000:return "5K";case 10000:return "10K";default:return "100H";}
}
static uint32_t span_from_panel(const char *v){
    if(!strcmp(v,"25K"))return 25000u;if(!strcmp(v,"10K"))return 10000u;if(!strcmp(v,"6K"))return 6000u;if(!strcmp(v,"2.5K"))return 2500u;return 25000u;
}
static const char *span_to_panel(void){return panel_span_hz==10000u?"10K":panel_span_hz==6000u?"6K":panel_span_hz==2500u?"2.5K":"25K";}

bool front_panel_init(void){
    bb=zbitx_bus();
    if(!i2cbb_probe(bb,ZBITX_PANEL_ADDR)) return false;
    last_cw[0]=0;
    next_wf=get_absolute_time(); next_poll=get_absolute_time(); next_meter=get_absolute_time();
    full_state_pending=true; memset(state_cache,0,sizeof(state_cache));
    /* Populate stock-panel Setup/Options fields now so they are useful later.
       Only 2TONE is active in this build; the remaining option fields are
       intentionally truthful placeholders until their RadioCore backends exist. */
    send_text("9 Ragchew Machine}PIVERSION RP2350 Ragchew Machine}");
    send_text("PANELVER STOCK ZBITX}");
    send_text("TXMON 0}TNPWR 20}TNDUR 5}DSP OFF}ANR OFF}NOTCH OFF}COMP 0}VFOLK OFF}BFO 0}VFO A}SPLIT OFF}");
    return true;
}

static ui_event_t no_event(void){ui_event_t e={0};e.type=UI_EVENT_NONE;return e;}
static ui_event_t adjust(ui_item_t item,int delta){ui_event_t e={0};e.type=UI_EVENT_ADJUST;e.item=item;e.delta=delta;return e;}

static ui_event_t parse_one(char *line,const ui_snapshot_t *s){
    while(*line==' '||*line=='\r'||*line=='\n')line++;
    char *sp=strchr(line,' '); char *v=""; if(sp){*sp=0;v=sp+1;}
    if(!strcmp(line,"FT8"))return ft8_command_event("FT8",v);
    if(!strcmp(line,"WIPE"))return ft8_command_event("WIPE",v);
    if(!strcmp(line,"SAVE"))return ft8_command_event("SAVE",v);
    if(!strcmp(line,"OPEN"))return ft8_command_event("OPEN",v);
    if(!strcmp(line,"CALL")||!strcmp(line,"EXCH")||!strcmp(line,"RECV")||!strcmp(line,"SENT")||!strcmp(line,"NR"))return ft8_command_event(line,v);
    if(!strcmp(line,"F1")||!strcmp(line,"F2")||!strcmp(line,"F3")||!strcmp(line,"F4")||!strcmp(line,"F5"))return ft8_command_event(line,v);
    if(!strcmp(line,"TX1ST")||!strcmp(line,"FT8_TX1ST")||!strcmp(line,"AUTO")||!strcmp(line,"FT8_AUTO")||!strcmp(line,"REPEAT")||!strcmp(line,"FT8_REPEAT")||!strcmp(line,"MYCALLSIGN")||!strcmp(line,"MYGRID")||!strcmp(line,"TX_PITCH"))return ft8_command_event(line,v);
    if(!strcmp(line,"USB") && !strcmp(v,"toggle")) return adjust(UI_ITEM_ENGINE_MODE, s->protocol1_mode ? -1 : 1);
    if(!strcmp(line,"SYNC")){ full_state_pending=true; next_wf=get_absolute_time(); return no_event(); }
    if(!strcmp(line,"SHUTDOWN")){ ui_event_t e={0}; e.type=UI_EVENT_PROGRAM_MODE; return e; }
    if(!strcmp(line,"FREQ")){
        uint32_t f=(uint32_t)strtoul(v,NULL,10); uint32_t st=steps[s->step_index<8?s->step_index:2];
        int64_t d=(int64_t)f-(int64_t)s->frequency_hz; if(d){ui_event_t e={0};e.type=UI_EVENT_TUNE;e.delta=(int32_t)(d/(int64_t)st);if(e.delta==0)e.delta=d>0?1:-1;return e;}
    }
    if(!strcmp(line,"MODE")){
        int m=mode_from_panel(v);
        if(m>=0&&m!=(int)s->operating_mode){ui_event_t e={0};e.type=UI_EVENT_SET_MODE;e.delta=m;return e;}
    }
    if(!strcmp(line,"STEP")){int n=step_index(step_from_panel(v));if(n>=0&&n!=(int)s->step_index)return adjust(UI_ITEM_STEP,n-(int)s->step_index);}
    if(!strcmp(line,"SPAN")){panel_span_hz=span_from_panel(v);next_wf=get_absolute_time();return no_event();}
    if(!strcmp(line,"AGC")){int want=!strcmp(v,"OFF")?0:(!strcmp(v,"FAST")?1:2);if(want!=(int)s->agc_mode)return adjust(UI_ITEM_AGC,want-(int)s->agc_mode);}
    if(!strcmp(line,"BW")){int n=atoi(v);if(n<50)n=50;if(n>5000)n=5000;ui_event_t e={0};e.type=UI_EVENT_SET_BW;e.delta=n;return e;}
    if(!strcmp(line,"AUDIO")){int n=atoi(v);int d=(n-(int)s->volume)/5;if(d)return adjust(UI_ITEM_VOLUME,d);}
    if(!strcmp(line,"IF")){int n=atoi(v);if(n!=(int)s->if_gain)return adjust(UI_ITEM_IF_GAIN,n-(int)s->if_gain);}
    if(!strcmp(line,"LOW")){int n=atoi(v);int d=(n-(int)s->rx_low)/50;if(d)return adjust(UI_ITEM_RX_LOW,d);}
    if(!strcmp(line,"HIGH")){int n=atoi(v);int d=(n-(int)s->rx_high)/50;if(d)return adjust(UI_ITEM_RX_HIGH,d);}
    if(!strcmp(line,"DRIVE")){int n=atoi(v);if(n!=(int)s->tx_drive)return adjust(UI_ITEM_TX_DRIVE,n-(int)s->tx_drive);}
    if(!strcmp(line,"MIC")){int n=atoi(v);if(n!=(int)s->mic_codec_in)return adjust(UI_ITEM_MIC_CODEC_IN,n-(int)s->mic_codec_in);}
    if(!strcmp(line,"PITCH")){int n=atoi(v);if(n<400)n=400;if(n>1200)n=1200;ui_event_t e={0};e.type=UI_EVENT_SET_CW_PITCH;e.delta=n;return e;}
    if(!strcmp(line,"WPM")){int n=atoi(v);if(n!=(int)s->cw_wpm)return adjust(UI_ITEM_CW_SPEED,n-(int)s->cw_wpm);}
    if(!strcmp(line,"CW_DELAY")){int n=atoi(v);int d=(n-(int)s->cw_delay_ms)/25;if(d)return adjust(UI_ITEM_CW_DELAY,d);}
    if(!strcmp(line,"SIDETONE")){int n=atoi(v);int d=(n-(int)s->sidetone_level)/5;if(d)return adjust(UI_ITEM_SIDETONE,d);}
    if(!strcmp(line,"CW_INPUT")){int want=!strcmp(v,"STRAIGHT")||!strcmp(v,"BUG")?0:(!strcmp(v,"IAMBICB")?2:1);if(want!=(int)s->keyer_mode)return adjust(UI_ITEM_KEYER_MODE,want-(int)s->keyer_mode);}
    if(!strcmp(line,"VFO")){ui_event_t e={0};e.type=UI_EVENT_SET_VFO;e.delta=(!strcmp(v,"B"));return e;}
    if(!strcmp(line,"BFO")){int n=atoi(v);if(n<-3000)n=-3000;if(n>3000)n=3000;int d=(n-(int)s->bfo_delta_hz)/50;if(d)return adjust(UI_ITEM_BFO_DELTA,d);}
    if(!strcmp(line,"DSP")){int want=!strcmp(v,"ON");if(want!=(int)s->dsp_nr_enabled)return adjust(UI_ITEM_DSP_NR,want?1:-1);}
    if(!strcmp(line,"80M")||!strcmp(line,"60M")||!strcmp(line,"40M")||!strcmp(line,"30M")||!strcmp(line,"20M")||!strcmp(line,"17M")||!strcmp(line,"15M")||!strcmp(line,"12M")||!strcmp(line,"10M")){
        static const char *bn[9]={"80M","60M","40M","30M","20M","17M","15M","12M","10M"};int want=-1;for(int i=0;i<9;i++)if(!strcmp(line,bn[i]))want=i;if(want>=0&&want!=(int)s->band_index)return adjust(UI_ITEM_BAND,want-(int)s->band_index);
    }
    if(!strcmp(line,"TX")){ui_event_t e={0};e.type=UI_EVENT_REMOTE_PTT;e.delta=1;return e;}
    if(!strcmp(line,"RX")){ui_event_t e={0};e.type=UI_EVENT_REMOTE_PTT;e.delta=0;return e;}
    return no_event();
}

ui_event_t front_panel_poll(const ui_snapshot_t *s){
    ui_event_t e=no_event(); if(!bb)return e;
    absolute_time_t now=get_absolute_time();
    if(absolute_time_diff_us(next_poll,now)<0)return e;
    next_poll=make_timeout_time_ms(12);
    uint8_t r[200]; int n=i2cbb_read_rll(bb,ZBITX_PANEL_ADDR,r,sizeof(r)-1);
    if(n<=0)return e; r[n]=0;
    /* Telemetry is useful later for battery/SWR; it is intentionally consumed
       silently here. User changes are normally a single LABEL value record. */
    char *save=NULL; char *line=strtok_r((char*)r,"\n",&save);
    while(line){
        if(strncmp(line,"vbatt ",6)&&strncmp(line,"power ",6)&&strncmp(line,"vswr ",5)){
            e=parse_one(line,s); if(e.type!=UI_EVENT_NONE)return e;
        }
        line=strtok_r(NULL,"\n",&save);
    }
    return e;
}

static void send_field_cached(int slot,const char *label,const char *value,bool full){
    if(!full && !strcmp(state_cache[slot],value)) return;
    char b[80];
    snprintf(b,sizeof(b),"%s %s}",label,value);
    if(send_text(b))
        snprintf(state_cache[slot],sizeof(state_cache[slot]),"%s",value);
}

static void send_state(const ui_snapshot_t *s,bool full){
    char v[40];
    unsigned bw=(s->operating_mode==4)?(unsigned)(s->am_rx_bw_hz/2u):(unsigned)(s->rx_high>s->rx_low?s->rx_high-s->rx_low:s->rx_high);
    const char *agc=s->agc_mode==0?"OFF":(s->agc_mode==1?"FAST":"SLOW");
    const char *key=s->keyer_mode==0?"STRAIGHT":(s->keyer_mode==2?"IAMBICB":"IAMBIC");
    unsigned plow, phigh;
    if(s->operating_mode==4){ plow=bw; phigh=bw; }
    else if(s->operating_mode==2 || s->operating_mode==3){
        unsigned pitch=(unsigned)s->cw_decode_pitch_hz;
        plow=pitch>bw/2u?pitch-bw/2u:50u; phigh=pitch+bw/2u;
        if(plow<50u)plow=50u; if(phigh>5000u)phigh=5000u;
    }else{ plow=(unsigned)s->rx_low; phigh=(unsigned)s->rx_high; }

    snprintf(v,sizeof(v),"%lu",(unsigned long)s->frequency_hz); send_field_cached(0,"FREQ",v,full);
    send_field_cached(1,"MODE",panel_mode(s->operating_mode),full);
    send_field_cached(2,"STEP",step_to_panel(steps[s->step_index<8?s->step_index:2]),full);
    send_field_cached(3,"SPAN",span_to_panel(),full);
    send_field_cached(4,"AGC",agc,full);
    snprintf(v,sizeof(v),"%u",bw); send_field_cached(5,"BW",v,full);
    snprintf(v,sizeof(v),"%u",plow); send_field_cached(6,"LOW",v,full);
    snprintf(v,sizeof(v),"%u",phigh); send_field_cached(7,"HIGH",v,full);
    snprintf(v,sizeof(v),"%u",s->tx?1u:0u); send_field_cached(8,"IN_TX",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->volume); send_field_cached(9,"AUDIO",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->if_gain); send_field_cached(10,"IF",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->tx_drive); send_field_cached(11,"DRIVE",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->mic_codec_in); send_field_cached(12,"MIC",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->cw_decode_pitch_hz); send_field_cached(13,"PITCH",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->cw_wpm); send_field_cached(14,"WPM",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->cw_delay_ms); send_field_cached(15,"CW_DELAY",v,full);
    snprintf(v,sizeof(v),"%u",(unsigned)s->sidetone_level); send_field_cached(16,"SIDETONE",v,full);
    send_field_cached(17,"CW_INPUT",key,full);
    if(full || last_usb_p1_mode != (int)s->protocol1_mode) {
        if(send_text(s->protocol1_mode ? "USB_MODE P1}" : "USB_MODE LOCAL}")) last_usb_p1_mode=(int)s->protocol1_mode;
    }
}

/* Convert the proven live FFT/waterfall peak into the stock ZBITX panel's
   native SMETER encoding.  The panel divides SMETER by 200 when drawing its
   six-segment bar, so use 0..1200 rather than a 0..99 percentage. */
static unsigned spectrum_to_smeter(const ui_snapshot_t *s){
    /* v0.7g: meter from the live RX FFT/waterfall source, not the audio AGC.
       The wide spectrum is already noise-referenced and scaled 0..DISPLAY_HEIGHT.
       Use the strongest visible RX bin for this first bench test. */
    unsigned peak = 0u;
    for (unsigned i=0; i<SPECTRUM_WIDE_BINS; ++i) {
        if ((unsigned)s->spectrum_wide[i] > peak) peak = (unsigned)s->spectrum_wide[i];
    }
    if (peak > 18u) peak = 18u;

    /* Stock ZBITX front panel does NOT consume a 0..99 percentage here.
       Its smeter_draw() divides the received SMETER value by 200 and uses
       that result as the six-segment bar index.  Feed the panel its native
       0..1200-ish meter domain: peak 0 -> 0, peak 18 -> 1200. */
    return (peak * 1200u + 9u) / 18u;
}

static void send_meter(const ui_snapshot_t *s){
    char b[32];
    unsigned peak=0u;
    for(unsigned i=0;i<SPECTRUM_WIDE_BINS;++i)
        if((unsigned)s->spectrum_wide[i]>peak) peak=(unsigned)s->spectrum_wide[i];
    unsigned m=spectrum_to_smeter(s);
    snprintf(b,sizeof(b),"SMETER %u}",m);
    send_text(b);

}

static void send_waterfall(const ui_snapshot_t *s){
    char b[260]; strcpy(b,"WF ");
    /* v0.7: use the 128 native FFT bins across +/-12 kHz (187.5 Hz/bin)
       instead of stretching the legacy 64-bin OLED spectrum.  Smaller panel
       spans crop these native bins first, then interpolate only for pixel fill.
       JJ's panel draws frequency right-to-left, so source order is reversed. */
    int use=(int)((panel_span_hz*SPECTRUM_WIDE_BINS+12000u)/24000u);
    if(use<2)use=2; if(use>(int)SPECTRUM_WIDE_BINS)use=(int)SPECTRUM_WIDE_BINS;
    int first=((int)SPECTRUM_WIDE_BINS-use)/2;
    for(int x=0;x<250;x++){
        int pos=x*(use-1); int i=first+pos/249; int rem=pos%249;
        int j=i+1; if(j>=first+use)j=first+use-1;
        int ri=(int)SPECTRUM_WIDE_BINS-1-i, rj=(int)SPECTRUM_WIDE_BINS-1-j;
        int a=s->spectrum_wide[ri], c=s->spectrum_wide[rj];
        int h=(a*(249-rem)+c*rem)/249;
        int mag=(h*94+9)/18; if(mag>94)mag=94; if(mag<0)mag=0;
        b[3+x]=(char)(32+mag);
    }
    b[253]='}'; b[254]=0; send_text(b);
}

void front_panel_render(const ui_snapshot_t *s,bool force){
    if(!bb)return; absolute_time_t now=get_absolute_time();
    /* Match JJ's companion software: full population only at boot/SYNC.
       During normal operation only fields whose values changed are sent. */
    send_state(s,full_state_pending);
    full_state_pending=false;
    if(absolute_time_diff_us(next_meter,now)>=0){send_meter(s);next_meter=make_timeout_time_ms(250);}
    if(!s->protocol1_mode&&(force||absolute_time_diff_us(next_wf,now)>=0)){send_waterfall(s);next_wf=make_timeout_time_ms(20);}
    if(force){
        char b[160];
        snprintf(b,sizeof(b),"VFO %c}VFOA %lu}VFOB %lu}BFO %d}DSP %s}",
            s->active_vfo?'B':'A',(unsigned long)s->vfo_a_hz,(unsigned long)s->vfo_b_hz,
            (int)s->bfo_delta_hz,s->dsp_nr_enabled?"ON":"OFF");
        send_text(b);
    }
    if(s->cw_decode_active&&strcmp(last_cw,s->cw_decode_text)){
        char b[80];snprintf(b,sizeof(b),"TEXT %s}",s->cw_decode_text);send_text(b);snprintf(last_cw,sizeof(last_cw),"%s",s->cw_decode_text);
    }
}

/* Stock ZBITX panel has no 6 kHz UART audio transport. */
void front_panel_audio_capture_6k(int16_t sample){(void)sample;}
void front_panel_audio_service(void){}
bool front_panel_tx_audio_pop_6k(int16_t *sample){(void)sample;return false;}
void front_panel_tx_audio_flush(void){}
uint16_t front_panel_tx_audio_queued(void){return 0;}

void front_panel_ft8_message(const char *line){
    if(!bb||!line||!*line)return;
    char b[128]; snprintf(b,sizeof(b),"FT8_LIST %s}",line); send_text(b);
}

void front_panel_set_field(const char *label, const char *value) {
    if (!bb || !label || !value) return;
    char b[100];
    int n=snprintf(b,sizeof(b),"%s %s}",label,value);
    if(n>0 && (size_t)n<sizeof(b)) send_text(b);
}

void front_panel_logbook_qso(const char *record) {
    if (!bb || !record || !*record) return;
    char b[190];
    int n=snprintf(b,sizeof(b),"QSO %s}",record);
    if(n>0 && (size_t)n<sizeof(b)) send_text(b);
}
