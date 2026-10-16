#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include "tusb.h"
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "hardware/dma.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "wm8731_bidi_slave.pio.h"
#include "wm8731_adc_rx.pio.h"
#include "audio/wm8731_codec.h"
#include "audio/rx_agc.h"
#include "audio/ssb_tx.h"
#include "audio/fft_ssb_tx.h"
#include "audio/fft_ssb_rx.h"
#include "radio/si5351_radio.h"
#include "radio/power_calibration.h"
#include "radio/cw_envelope.h"
#include "hardware/radio_io.h"
#include "audio/audio_filter.h"
#include "audio/spectrum.h"
#include "audio/cw_decoder.h"
#include "settings.h"
#include "ui/front_panel.h"
#include "p1_network.h"
#include "p1_protocol.h"
#include "p1_dsp.h"
#include "ft8_n6.h"
#include "ft8_tx6.h"
#include "rtc_ds3231.h"
#include <time.h>
#include "pico/multicore.h"
#include "pico/mutex.h"

/* FT8/log helpers above the console implementation need the same USB
   diagnostics as the rest of the firmware. */
static int cdc_console_printf(const char *fmt, ...);
#define printf cdc_console_printf

#define PIN_SDA       2
#define PIN_SCL       3
#define PIN_BCLK      18
#define PIN_LRCLK     19
#define PIN_ADCDAT    20
#define PIN_DACDAT    21
#define SAMPLE_RATE   96000u
#define SINE_SIZE     256u

// Words, not stereo frames. Must be even.
#define HALF_WORDS 512
#define TOTAL_WORDS (HALF_WORDS * 2)

static PIO audio_pio = pio0;
static uint audio_sm;
static PIO adc_pio = pio1;
static uint adc_sm;
static int dma_in_ctrl, dma_in_data, dma_out_ctrl, dma_out_data;
static int32_t input_buffer[TOTAL_WORDS];
static int32_t output_buffer[TOTAL_WORDS];
static int32_t *in_blocks[2] __attribute__((aligned(8)));
static int32_t *out_blocks[2] __attribute__((aligned(8)));
static volatile uint32_t dma_irqs;
static volatile uint32_t completed_half;
static volatile int32_t adc_peak_l;
static volatile int32_t adc_peak_r;

typedef enum {
    AUDIO_MODE_SSB = 0,
    AUDIO_MODE_RAW_RECEIVER,
    AUDIO_MODE_MIC_MONITOR,
    AUDIO_MODE_TONE
} AudioMode;

typedef enum {
    SIDEBAND_USB = 0,
    SIDEBAND_LSB
} Sideband;

typedef enum {
    OPERATING_USB = 0,
    OPERATING_LSB,
    OPERATING_CWU,
    OPERATING_CWL,
    OPERATING_AM,
    OPERATING_2TONE,
    OPERATING_FT8
} OperatingMode;

static volatile AudioMode audio_mode = AUDIO_MODE_SSB;
static volatile Sideband sideband = SIDEBAND_LSB;
static volatile OperatingMode operating_mode = OPERATING_LSB;
static uint16_t am_rx_bw_hz = 6000u;

/* N5: deliberately boring FT8 proof.  Capture the existing 6 kHz stream
   in-place.  No FIFO, no Core1 handoff, no FFT, no decoder. */
#define FT8_N5_SAMPLES 78000u
static int8_t ft8_n5_capture[FT8_N5_SAMPLES];
static volatile int8_t ft8_decode_idx = -1;
static volatile bool ft8_worker_busy = false;
static volatile bool ft8_worker_done = false;
static char ft8_job_time[7] = "000000";
static mutex_t ft8_result_mutex;
static uint32_t ft8_core1_stack[2048] __attribute__((aligned(8)));
static char ft8_results[50][96];
static unsigned ft8_results_count = 0;
static void ft8_on_decoded(const char *line);
static char log_call[24]="", log_exch[24]="", log_recv[16]="", log_sent[16]="", log_nr[16]="";
static void ft8_qso_field(const char *name, const char *value) {
    front_panel_set_field(name, value ? value : "");
    char *dst=NULL; size_t cap=0;
    if(!strcmp(name,"CALL")){dst=log_call;cap=sizeof(log_call);}
    else if(!strcmp(name,"EXCH")){dst=log_exch;cap=sizeof(log_exch);}
    else if(!strcmp(name,"RECV")){dst=log_recv;cap=sizeof(log_recv);}
    else if(!strcmp(name,"SENT")){dst=log_sent;cap=sizeof(log_sent);}
    else if(!strcmp(name,"NR")){dst=log_nr;cap=sizeof(log_nr);}
    if(dst)snprintf(dst,cap,"%s",value?value:"");
}
static void ft8_qso_clear_fields(void) {
    ft8_qso_field("CALL", ""); ft8_qso_field("EXCH", "");
    ft8_qso_field("RECV", ""); ft8_qso_field("SENT", "");
    ft8_qso_field("NR", "");
}
static void ft8_qso_clear_state(void);

static void ft8_panel_rx_display(const char *line);
static void ft8_post_message(const char *msg) {
    mutex_enter_blocking(&ft8_result_mutex);
    if (ft8_results_count < 50) { snprintf(ft8_results[ft8_results_count++],96,"%s",msg); }
    mutex_exit(&ft8_result_mutex);
}
static void ft8_core1_worker(void) {
    for (;;) {
        uint32_t idx = multicore_fifo_pop_blocking();
        if (idx == 0) ft8_n6_decode_capture(ft8_n5_capture, FT8_N5_SAMPLES, ft8_job_time, ft8_post_message);
        __dmb();
        ft8_worker_done = true;
    }
}
static void __attribute__((noinline)) ft8_result_service(void) {
    /* Drain one result at a time
       instead of keeping a 4.8 KB copy on every main-loop stack frame. */
    char local[96]; unsigned n=0;
    mutex_enter_blocking(&ft8_result_mutex);
    n=ft8_results_count;
    mutex_exit(&ft8_result_mutex);
    for(unsigned i=0;i<n;i++) {
        mutex_enter_blocking(&ft8_result_mutex);
        memcpy(local,ft8_results[0],sizeof(local));
        --ft8_results_count;
        memmove(ft8_results,ft8_results+1,ft8_results_count*sizeof(ft8_results[0]));
        mutex_exit(&ft8_result_mutex);
        ft8_panel_rx_display(local); printf("FT8: %s\n",local); ft8_on_decoded(local);
    }
    if (ft8_worker_done) {
        __dmb(); ft8_worker_done=false; ft8_worker_busy=false; ft8_decode_idx=-1;
        /* Worker completion is routine, not a console event. */
    }
}

static volatile uint32_t ft8_n5_wr = 0;
static volatile bool ft8_n5_ready = false;
static uint32_t ft8_n5_reports = 0;
/* Junior-compatible PC UTC sync, anchored to RP2350 monotonic microseconds. */
static bool ft8_utc_valid = false;
static int64_t ft8_utc_epoch_ms = 0;
static uint64_t ft8_mono_epoch_us = 0;
static int64_t ft8_utc_now_ms(void) {
    if (!ft8_utc_valid) return -1;
    return ft8_utc_epoch_ms + (int64_t)((time_us_64() - ft8_mono_epoch_us) / 1000u);
}
static void ft8_utc_set_ms(int64_t ms) {
    ft8_utc_epoch_ms = ms;
    ft8_mono_epoch_us = time_us_64();
    ft8_utc_valid = true;
}
/* Defer DS3231 writes to a UTC second boundary on Core 0. */
static bool rtc_write_pending = false;
static int64_t rtc_write_second = 0;
static void rtc_sync_service(void) {
    if (!rtc_write_pending || !ft8_utc_valid) return;
    int64_t now=ft8_utc_now_ms();
    if (now < rtc_write_second*1000) return;
    rtc_write_pending=false;
    bool ok=rtc_ds3231_write(rtc_write_second);
    printf("RTC sync %s UTC=%lld\n",ok?"OK":"FAILED",(long long)rtc_write_second);
}
static void rtc_boot_restore(void) {
    int64_t sec=0;bool osf=true;
    if (!rtc_ds3231_read(&sec,&osf)) {printf("RTC unavailable or invalid\n");return;}
    if (osf) {printf("RTC oscillator-stop flag set; use TIME sync\n");return;}
    ft8_utc_set_ms(sec*1000);
    printf("RTC restored UTC=%lld (second phase uncalibrated; verify FT8 DT)\n",(long long)sec);
}
static int64_t ft8_slot_last = -1;
static bool ft8_tx6_queued = false;
static bool ft8_tx6_on = false;
static bool ft8_tx6_first = true;
static bool protocol1_mode; /* tentative declaration; initialized below */
/* FT8 panel bridge: no additional waveform/RAM buffers.  The stock panel's
   operator settings are not assumed until explicitly received from the panel. */
static void mark_settings_dirty(void);
static char ft8_mycall[20] = "";
static char ft8_mygrid[12] = "";
static char ft8_target[20] = "";
static char ft8_report[8] = "-10";
static bool ft8_panel_auto = false;
static unsigned ft8_panel_repeat = 5;
static unsigned ft8_repeat_left = 0;
static char ft8_repeat_message[96];
static char ft8_tx_message[96];
static bool ft8_qso_active = false;
static int ft8_qso_phase = 0; /* 0 idle, 1 sent grid, 2 sent report, 3 sent R-report, 4 sent RR73 */
static int64_t ft8_last_response_slot = -1;
static void ft8_qso_clear_state(void) {
    ft8_qso_active=false; ft8_qso_phase=0; ft8_target[0]=0;
    ft8_repeat_left=0; ft8_repeat_message[0]=0;
    ft8_tx6_queued=false;
    ft8_last_response_slot=-1;
}
static int ft8_tx_pitch = 2200;
static void ft8_on_decoded(const char *line);
/* The stock panel accepts FT8_LIST records with #H (our TX), #P (to us),
   and #Q (our callsign). Its FT8 filter understands these markers. */
static void ft8_panel_status(const char *status, const char *message) {
    if (!message || !*message || !ft8_utc_valid) return;
    int64_t sec = ft8_utc_now_ms() / 1000;
    int hh=(int)((sec/3600)%24), mm=(int)((sec/60)%60), ss=(int)(sec%60);
    char line[120];
    snprintf(line,sizeof(line),"#H%02d%02d%02d  00 +00 %04d ~ #S%s #H%s",
             hh,mm,ss,ft8_tx_pitch,status,message);
    front_panel_ft8_message(line);
}
static void ft8_panel_rx_display(const char *line) {
    if (!line || !*line) return;
    const char *tilde = strchr(line,'~');
    if (!tilde || !ft8_mycall[0]) {front_panel_ft8_message(line);return;}
    const char *payload=tilde+1;
    /* Decoder payload: SNR, DT, then decoded FT8 words. */
    char snr[12],dt[12],a[24],b[24],c[24];
    if (sscanf(payload,"%11s %11s %23s %23s %23s",snr,dt,a,b,c)!=5 ||
        (strcmp(a,ft8_mycall) && strcmp(b,ft8_mycall) && strcmp(c,ft8_mycall))) {
        front_panel_ft8_message(line);return;
    }
    char out[128]; size_t pos=0;
    /* #P tags incoming traffic for the CQ-only filter; it is not a color. */
    out[pos++]='#';out[pos++]='P';
    size_t prefix=(size_t)(tilde-line+1);
    if (prefix+pos>=sizeof(out)) {front_panel_ft8_message(line);return;}
    memcpy(out+pos,line,prefix);pos+=prefix;
    const char *p=payload;
    while (*p && pos+4<sizeof(out)) {
        if ((*p==' ' || *p=='\t')) {out[pos++]=*p++;continue;}
        const char *start=p;while(*p && *p!=' ' && *p!='\t')p++;
        size_t n=(size_t)(p-start);
        if (n==strlen(ft8_mycall) && !strncmp(start,ft8_mycall,n)) {
            if(pos+2>=sizeof(out))break;
            out[pos++]='#';out[pos++]='Q';
        }
        if(pos+n>=sizeof(out))break;
        memcpy(out+pos,start,n);pos+=n;
    }
    out[pos]=0;
    front_panel_ft8_message(out);
}

static void ft8_panel_queue(const char *msg) {
    if (operating_mode != OPERATING_FT8 || !ft8_utc_valid || protocol1_mode) { printf("FT8 panel: requires local FT8 and UTC\n"); return; }
    if (ft8_tx6_on || ft8_tx6_active()) { printf("FT8 panel: TX busy\n"); return; }
    if (!ft8_tx6_prepare(msg,(unsigned)ft8_tx_pitch)) { printf("FT8 panel: encode failed: %s\n",msg); return; }
    ft8_tx6_queued=true;
    snprintf(ft8_tx_message,sizeof(ft8_tx_message),"%s",msg);
    snprintf(ft8_repeat_message,sizeof(ft8_repeat_message),"%s",msg);
    ft8_repeat_left=(strstr(msg," RR73")||strstr(msg," 73")||strstr(msg," RRR"))?0:((ft8_panel_repeat>1 && ft8_panel_repeat<=10)?ft8_panel_repeat-1:0);
    printf("FT8 QUEUED: %s\n",msg);
    ft8_panel_status("QUEUED ",msg);
}
/* Logbook Stage 1: manual SAVE.  OPEN/viewer and automatic FT8 save follow
   after an on-hardware write test.  Never clear the QSO on write failure. */
static void adif_field(char *out,size_t cap,const char *key,const char *value) {
    if(!value || !*value)return;
    size_t len=strlen(out), n=strlen(value);
    if(len>=cap)return;
    snprintf(out+len,cap-len,"<%s:%u>%s",key,(unsigned)n,value);
}
static void __attribute__((noinline)) log_save_manual(void) {
    if(!log_call[0]){printf("ADIF ERROR: CALL is empty; QSO NOT saved!\n");ft8_panel_status("ERROR ","QSO NOT SAVED - NO CALL");return;}
    if(!ft8_utc_valid){printf("ADIF ERROR: UTC unavailable; QSO NOT saved!\n");ft8_panel_status("ERROR ","QSO NOT SAVED - NO UTC");return;}
    time_t seconds=(time_t)(ft8_utc_now_ms()/1000);
    struct tm *utc=gmtime(&seconds);
    if(!utc){printf("ADIF ERROR: invalid UTC; QSO NOT saved!\n");return;}
    char date[12],clock[8],freq[24],record[512]="";
    snprintf(date,sizeof(date),"%04d%02d%02d",utc->tm_year+1900,utc->tm_mon+1,utc->tm_mday);
    snprintf(clock,sizeof(clock),"%02d%02d%02d",utc->tm_hour,utc->tm_min,utc->tm_sec);
    uint32_t dial_hz=radio_get_dial_hz();
    snprintf(freq,sizeof(freq),"%lu.%06lu",(unsigned long)(dial_hz/1000000u),
             (unsigned long)(dial_hz%1000000u));
    const char *mode=operating_mode==OPERATING_FT8?"FT8":operating_mode==OPERATING_LSB||operating_mode==OPERATING_USB?"SSB":operating_mode==OPERATING_AM?"AM":operating_mode==OPERATING_CWL||operating_mode==OPERATING_CWU?"CW":"OTHER";
    adif_field(record,sizeof(record),"CALL",log_call);
    adif_field(record,sizeof(record),"QSO_DATE",date);
    adif_field(record,sizeof(record),"TIME_ON",clock);
    adif_field(record,sizeof(record),"FREQ",freq);
    adif_field(record,sizeof(record),"MODE",mode);
    adif_field(record,sizeof(record),"STATION_CALLSIGN",ft8_mycall);
    adif_field(record,sizeof(record),"RST_SENT",log_sent);
    adif_field(record,sizeof(record),"RST_RCVD",log_recv);
    if(operating_mode==OPERATING_FT8)adif_field(record,sizeof(record),"GRIDSQUARE",log_exch);
    else adif_field(record,sizeof(record),"SRX_STRING",log_exch);
    if(log_nr[0])adif_field(record,sizeof(record),"COMMENT",log_nr);
    if(strlen(record)+8>=sizeof(record)){printf("ADIF ERROR: record too long\n");return;}
    strcat(record,"<EOR>\r\n");
    if(!log_adif_append(record)){
        printf("ADIF ERROR: SD write failed - QSO NOT saved!\n");
        ft8_panel_status("ERROR ","QSO NOT SAVED - SD ERROR");return;
    }
    printf("ADIF: QSO saved - %s - %s - %s MHz\n",log_call,mode,freq);
    printf("ADIF: UTC %s %s - SENT %s - RCV %s\n",date,clock,log_sent,log_recv);
    char msg[64];snprintf(msg,sizeof(msg),"QSO SAVED %.23s",log_call);
    ft8_panel_status("LOG ",msg);
    ft8_qso_clear_state();ft8_qso_clear_fields();
}


/* Read-only logbook: bounded static snapshot avoids the SAVE stack regression.
   Send one panel QSO per main-loop pass. No SD work during startup. */
static char logview_data[1024];
static unsigned logview_offsets[16];
static unsigned logview_lengths[16];
static unsigned logview_count, logview_cursor;
static bool logview_pending;
static bool logview_open_requested;
static bool adif_get(const char *begin,const char *end,const char *key,char *dst,size_t cap){
    size_t klen=strlen(key);
    for(const char *p=begin;p<end;p++){
        if(*p!='<')continue;
        const char *colon=memchr(p,':',(size_t)(end-p));
        if(!colon || (size_t)(colon-p-1)!=klen || strncasecmp(p+1,key,klen))continue;
        const char *q=colon+1;unsigned n=0;
        if(q>=end || *q<'0'||*q>'9')continue;
        while(q<end && *q>='0' && *q<='9') {if(n>10000)return false;n=n*10+(unsigned)(*q++-'0');}
        /* Optional ADIF data type suffix. */
        while(q<end && *q!='>' && q-colon<18)q++;
        if(q>=end || *q!='>' || (size_t)(end-(q+1))<n)return false;
        if(n>=cap)n=(unsigned)cap-1;
        memcpy(dst,q+1,n);dst[n]=0;return true;
    }
    dst[0]=0;return false;
}
static void logview_open(void){
    logview_count=logview_cursor=0;logview_pending=false;
    int n=log_adif_read_snapshot(logview_data,sizeof(logview_data));
    if(n<0){printf("LOGVIEW: SD read error %d (missing file is -2)\n",n);return;}
    const char *start=logview_data,*end=logview_data+n;
    while(start<end && logview_count<16){
        const char *e=strstr(start,"<EOR>");
        if(!e || e>=end)break;
        logview_offsets[logview_count]=(unsigned)(start-logview_data);
        logview_lengths[logview_count]=(unsigned)(e-start);
        logview_count++;start=e+5;
    }
    logview_pending=logview_count>0;
    printf("LOGVIEW: %u records ready\n",logview_count);
}
static void logview_service(void){
    if(!logview_pending)return;
    if(logview_cursor>=logview_count){logview_pending=false;return;}
    unsigned idx=logview_count-1-logview_cursor;
    const char *start=logview_data+logview_offsets[idx];
    const char *end=start+logview_lengths[idx];
    char call[10],mode[10],freq[20],date[12],time[8],sent[10],recv[10],exch[10],station[10];
    if(!adif_get(start,end,"CALL",call,sizeof(call)) ||
       !adif_get(start,end,"QSO_DATE",date,sizeof(date)) ||
       !adif_get(start,end,"TIME_ON",time,sizeof(time))) {logview_cursor++;return;}
    adif_get(start,end,"MODE",mode,sizeof(mode));
    adif_get(start,end,"FREQ",freq,sizeof(freq));
    adif_get(start,end,"RST_SENT",sent,sizeof(sent));
    adif_get(start,end,"RST_RCVD",recv,sizeof(recv));
    adif_get(start,end,"STATION_CALLSIGN",station,sizeof(station));
    adif_get(start,end,"GRIDSQUARE",exch,sizeof(exch));
    if(!exch[0])adif_get(start,end,"SRX_STRING",exch,sizeof(exch));
    if(strlen(date)!=8 || strlen(time)<4 || !station[0]) {logview_cursor++;return;}
    char datefmt[11];snprintf(datefmt,sizeof(datefmt),"%.4s-%.2s-%.2s",date,date+4,date+6);
    unsigned khz=(unsigned)(atof(freq)*1000.0+0.5);
    char line[160];
    int n=snprintf(line,sizeof(line),"%u|%s|%u|%s|%.4s|%s|%s||%s|%s|%s|",
        logview_cursor+1,mode[0]?mode:"SSB",khz,datefmt,time,station,
        sent[0]?sent:"--",call,recv[0]?recv:"--",exch);
    if(n>0 && (size_t)n<sizeof(line))front_panel_logbook_qso(line);
    logview_cursor++;
}

static void ft8_panel_command(const char *command) {
    char label[20]={0}; const char *v=strchr(command,' ');
    size_t len=v?(size_t)(v-command):strlen(command);
    if(len>=sizeof(label))return;
    memcpy(label,command,len);v=v?v+1:"";
    if(!strcmp(label,"CALL")||!strcmp(label,"EXCH")||!strcmp(label,"RECV")||!strcmp(label,"SENT")||!strcmp(label,"NR")) {
        /* Manual editing: retain the panel value without echoing it back. */
        char *dst=NULL;size_t cap=0;
        if(!strcmp(label,"CALL")){dst=log_call;cap=sizeof(log_call);}
        else if(!strcmp(label,"EXCH")){dst=log_exch;cap=sizeof(log_exch);}
        else if(!strcmp(label,"RECV")){dst=log_recv;cap=sizeof(log_recv);}
        else if(!strcmp(label,"SENT")){dst=log_sent;cap=sizeof(log_sent);}
        else {dst=log_nr;cap=sizeof(log_nr);}
        snprintf(dst,cap,"%s",v);return;
    }
    if(!strcmp(label,"SAVE")){log_save_manual();return;}
    if(!strcmp(label,"OPEN")){logview_open_requested=true;printf("LOGVIEW: OPEN requested\n");return;}
    if(!strcmp(label,"WIPE")) {
        ft8_qso_clear_state();
        if (operating_mode==OPERATING_FT8 && ft8_tx6_on) {
            ft8_tx6_abort(); ft8_tx6_on=false;
            radio_io_set_manual_tx(false); radio_io_clear_manual_tx();
        }
        ft8_qso_clear_fields();
        printf("FT8 QSO wiped; AUTO ready for another caller\n");
        return;
    }
    if(!strcmp(label,"TX_PITCH")){int n=atoi(v);if(n>=300&&n<=3000 && ft8_tx_pitch!=n){ft8_tx_pitch=n;mark_settings_dirty();}return;}
    if(!strcmp(label,"MYCALLSIGN")) {snprintf(ft8_mycall,sizeof(ft8_mycall),"%s",v);mark_settings_dirty();return;}
    if(!strcmp(label,"MYGRID")) {snprintf(ft8_mygrid,sizeof(ft8_mygrid),"%s",v);mark_settings_dirty();return;}
    if(!strcmp(label,"TX1ST")||!strcmp(label,"FT8_TX1ST")) {ft8_tx6_first=!strcmp(v,"ON");mark_settings_dirty();return;}
    if(!strcmp(label,"AUTO")||!strcmp(label,"FT8_AUTO")) {ft8_panel_auto=!strcmp(v,"ON");mark_settings_dirty();printf("FT8 AUTO requested=%d (automatic response not enabled in this stage)\n",ft8_panel_auto);return;}
    if(!strcmp(label,"REPEAT")||!strcmp(label,"FT8_REPEAT")) {unsigned n=(unsigned)atoi(v);ft8_panel_repeat=n>10?10:(n<1?1:n);mark_settings_dirty();return;}
    if(!strcmp(label,"FT8")) {
        const char *msg=strchr(v,'~');if(!msg)return;msg++;while(*msg==' ')msg++;
        /* The stock panel strips #G colour tags, but retains the decoder's
           SNR and DT metadata after '~'. Skip both before reading CQ/call/grid. */
        char snr[20]={0},dt[20]={0},a[20]={0},b[20]={0},c[20]={0};
        if(sscanf(msg,"%19s %19s %19s %19s %19s",snr,dt,a,b,c)!=5)return;
        if(!ft8_mycall[0]||strlen(ft8_mygrid)<4){printf("FT8 panel: enter MYCALLSIGN/MYGRID first\n");return;}
        /* Selecting a CQ initiates a reply in the opposite parity slot. */
        if(!strcmp(a,"CQ")) {snprintf(ft8_target,sizeof(ft8_target),"%s",b);ft8_qso_active=true;ft8_qso_phase=1;
            ft8_qso_clear_fields(); ft8_qso_field("CALL",b);
            if (strlen(c)>=4) ft8_qso_field("EXCH",c);
            /* The selected CQ slot parity is derived from the decoded timestamp, not the button press time. */
            const char *t=strstr(v,"#G");
            if(t && strlen(t)>=8){int ss=(t[6]-'0')*10+(t[7]-'0');if(ss==0||ss==30)ft8_tx6_first=false;else if(ss==15||ss==45)ft8_tx6_first=true;}
            else if(strlen(v)>=6 && v[0]>='0'&&v[0]<='9' && v[4]>='0'&&v[4]<='9' && v[5]>='0'&&v[5]<='9'){
                /* Stock panel strips the #G prefix; HHMMSS remains. */
                int ss=(v[4]-'0')*10+(v[5]-'0');
                if(ss==0||ss==30)ft8_tx6_first=false;else if(ss==15||ss==45)ft8_tx6_first=true;
            }
            char tx[96];snprintf(tx,sizeof(tx),"%s %s %.4s",b,ft8_mycall,ft8_mygrid);ft8_panel_queue(tx);
        }
        return;
    }
    if(label[0]=='F'&&label[1]>='1'&&label[1]<='5'&&!label[2]) {
        char tx[96]={0};
        if(label[1]=='1') {if(!ft8_mycall[0]||strlen(ft8_mygrid)<4){printf("FT8: set callsign/grid on panel\n");return;}snprintf(tx,sizeof(tx),"CQ %s %.4s",ft8_mycall,ft8_mygrid);}
        else {if(!ft8_target[0]||!ft8_mycall[0]){printf("FT8: select a CQ first\n");return;}
            const char *end=label[1]=='2'?ft8_report:label[1]=='3'?"R-10":label[1]=='4'?"RR73":"73";
            snprintf(tx,sizeof(tx),"%s %s %s",ft8_target,ft8_mycall,end);
        }
        ft8_panel_queue(tx);
    }
}

/* Process actual decoded text after the Core1 worker has published it.
   Only address-specific exchanges; never call an arbitrary CQ automatically. */
static void ft8_on_decoded(const char *line) {
    if (!ft8_panel_auto || !ft8_mycall[0] || !ft8_utc_valid || operating_mode!=OPERATING_FT8 || protocol1_mode) return;
    const char *msg=strchr(line,'~'); if(!msg)return;
    ++msg; while(*msg==' ')++msg;
    /* N6 format after ~: SNR, DT, message */
    char snr[12]={0},dt[12]={0},a[24]={0},b[24]={0},c[24]={0};
    if(sscanf(msg,"%11s %11s %23s %23s %23s",snr,dt,a,b,c)!=5)return;
    if(strchr(a,'<')||strchr(b,'<')||strchr(c,'<'))return;
    if(strcmp(a,ft8_mycall)!=0)return;
    if(ft8_qso_active && strcmp(b,ft8_target)!=0)return;
    if(!ft8_qso_active){
        if(!strcmp(c,"RRR")||!strcmp(c,"RR73")||!strcmp(c,"73"))return;
        snprintf(ft8_target,sizeof(ft8_target),"%s",b);ft8_qso_active=true;ft8_qso_phase=0;
        ft8_qso_clear_fields(); ft8_qso_field("CALL",b);
    }
    /* The QSO workspace is independent of the FT8 decoded-message list. */
    ft8_qso_field("CALL", b);
    if (strlen(c)==4 && c[0]>='A' && c[0]<='R' && c[1]>='A' && c[1]<='R')
        ft8_qso_field("EXCH", c);
    if (c[0]=='-' || c[0]=='+' || (c[0]=='R' && (c[1]=='-' || c[1]=='+')))
        ft8_qso_field("RECV", c[0]=='R' ? c+1 : c);
    int64_t slot=ft8_utc_now_ms()/15000;
    if(slot==ft8_last_response_slot)return;
    char reply[96]={0};
    if(!strcmp(c,"73")){ft8_qso_active=false;ft8_target[0]=0;ft8_repeat_left=0;return;}
    if(!strcmp(c,"RR73")||!strcmp(c,"RRR")){
        snprintf(reply,sizeof(reply),"%s %s 73",b,ft8_mycall);ft8_qso_phase=5;
    }else if((c[0]=='R' && (c[1]=='-'||c[1]=='+'))){
        snprintf(reply,sizeof(reply),"%s %s RR73",b,ft8_mycall);ft8_qso_phase=4;
    }else if(c[0]=='-'||c[0]=='+'){
        snprintf(reply,sizeof(reply),"%s %s R%s",b,ft8_mycall,ft8_report);ft8_qso_phase=3;
    }else if(ft8_qso_phase==0){
        snprintf(reply,sizeof(reply),"%s %s %s",b,ft8_mycall,ft8_report);ft8_qso_phase=2;
    }else return;
    /* RX slot parity, not the decode-completion time, controls our next TX slot. */
    const char *ts=line; if(ts[0]=='#' && strlen(ts)>=8){int ss=(ts[6]-'0')*10+(ts[7]-'0');
        if(ss==0||ss==30)ft8_tx6_first=false;else if(ss==15||ss==45)ft8_tx6_first=true;
    }
    ft8_repeat_left=0;
    if(ft8_tx6_queued)ft8_tx6_queued=false; /* new addressed reply supersedes previous repeat */
    ft8_panel_queue(reply);
    if (ft8_tx6_queued && (strstr(reply," -") || strstr(reply," +") || strstr(reply," R-") || strstr(reply," R+"))) {
        const char *r=strrchr(reply,' ');
        if (r && r[1]) ft8_qso_field("SENT",r[1]=='R' ? r+2 : r+1);
    }
    ft8_last_response_slot=slot;
}

static int64_t ft8_tx6_slot = -1;
static void ft8_tx6_service(void);
static bool ft8_slot_capture_active = false;
static void ft8_slot_schedule(void) {
    /* A pending CQ repeat is NOT TX: keep receiving between repetitions. */
    if (operating_mode != OPERATING_FT8 || ft8_tx6_on || !ft8_utc_valid || ft8_n5_ready || ft8_slot_capture_active) return;
    int64_t now = ft8_utc_now_ms();
    int64_t slot = now / 15000;
    int64_t boundary = slot * 15000;
    if (now >= boundary && now < boundary + 200 && slot != ft8_slot_last) {
        ft8_slot_last = slot;
        if (ft8_worker_busy) {
            printf("FT8: G1 capture buffer busy; slot dropped\n"); return;
        }
        ft8_n5_wr = 0;
        ft8_slot_capture_active = true;
        /* Capture start is routine, not a console event. */
    }
}


static inline void ft8_n5_feed_6k(int16_t s) {
    if (operating_mode != OPERATING_FT8 || ft8_n5_ready || (ft8_utc_valid && !ft8_slot_capture_active)) return;
    uint32_t i = ft8_n5_wr;
    if (i < FT8_N5_SAMPLES) {
        int32_t v = ((int32_t)s) >> 5; /* proven M5 scaling */
        if (v > 127) v = 127;
        if (v < -128) v = -128;
        ft8_n5_capture[i] = (int8_t)v;
        i++;
        ft8_n5_wr = i;
        if (i >= FT8_N5_SAMPLES) { ft8_n5_ready = true; ft8_slot_capture_active = false; }
    }
}

static bool civ_remote_ptt=false;
static bool panel_remote_ptt=false;
static int16_t civ_usb_tx_cur=0,civ_usb_tx_next=0;
static uint8_t civ_usb_tx_phase=16u;
static bool civ_usb_tx_have=false;
static bool civ_usb_tx_running=false;
#define CIV_USB_TX_PREFILL_6K 96u   /* 16 ms jitter cushion before audio starts */
#define CIV_USB_TX_REFILL_6K  48u   /* after an underrun, wait one full UART frame */

static void civ_usb_tx_audio_reset(void){
    civ_usb_tx_cur=civ_usb_tx_next=0;
    civ_usb_tx_phase=16u;
    civ_usb_tx_have=false;
    civ_usb_tx_running=false;
}
static int16_t civ_usb_tx_audio_96k(void){
    /* Do not play individual UART frames as they arrive.  Build a small
       cushion first so USB/TFT/UART scheduling jitter cannot turn TX audio
       into 8 ms bursts separated by silence. */
    if(!civ_usb_tx_running){
        uint16_t need=civ_usb_tx_have ? CIV_USB_TX_REFILL_6K : CIV_USB_TX_PREFILL_6K;
        if(front_panel_tx_audio_queued()<need) return 0;
        civ_usb_tx_running=true;
    }
    if(!civ_usb_tx_have){
        if(!front_panel_tx_audio_pop_6k(&civ_usb_tx_cur)){ civ_usb_tx_running=false; return 0; }
        if(!front_panel_tx_audio_pop_6k(&civ_usb_tx_next)){ civ_usb_tx_running=false; return 0; }
        civ_usb_tx_phase=0u;
        civ_usb_tx_have=true;
    }
    if(civ_usb_tx_phase>=16u){
        civ_usb_tx_cur=civ_usb_tx_next;
        if(!front_panel_tx_audio_pop_6k(&civ_usb_tx_next)){
            civ_usb_tx_have=false;
            civ_usb_tx_running=false;
            civ_usb_tx_phase=16u;
            return 0;
        }
        civ_usb_tx_phase=0u;
    }
    int32_t d=(int32_t)civ_usb_tx_next-(int32_t)civ_usb_tx_cur;
    int32_t y=(int32_t)civ_usb_tx_cur + (d*(int32_t)civ_usb_tx_phase)/16;
    civ_usb_tx_phase++;
    if(y>32767) y=32767; else if(y<-32768) y=-32768;
    return (int16_t)y;
}
static volatile bool cw_key_down = false;

/* CW2 test keyer: PTT/GP14 is DIT, DASH/GP15 is DAH in CW modes.
 * Start with a fixed 20 WPM Iambic-B implementation; once paddle feel is
 * proven we can expose WPM/mode in the OLED menu. */
static uint8_t cw_keyer_wpm = 20u;
typedef enum { CW_KEY_STRAIGHT=0, CW_KEY_IAMBIC_A=1, CW_KEY_IAMBIC_B=2 } cw_keyer_mode_t;
static uint8_t cw_keyer_mode = CW_KEY_IAMBIC_B;
typedef enum { CWK_IDLE=0, CWK_DIT_DOWN, CWK_DIT_GAP, CWK_DAH_DOWN, CWK_DAH_GAP } cw_keyer_state_t;
static cw_keyer_state_t cw_keyer_state = CWK_IDLE;
static absolute_time_t cw_keyer_deadline;
static bool cw_keyer_pending_dit = false;
static bool cw_keyer_pending_dah = false;
static uint8_t cw_keyer_last = 0u; /* 1=dit, 2=dah */

static inline uint32_t cw_keyer_dot_ms(void) { return 1200u / (cw_keyer_wpm ? cw_keyer_wpm : 20u); }
static inline bool cw_keyer_deadline_passed(void) {
    return absolute_time_diff_us(get_absolute_time(), cw_keyer_deadline) <= 0;
}
static void cw_keyer_reset(void) {
    cw_keyer_state = CWK_IDLE;
    cw_keyer_pending_dit = false;
    cw_keyer_pending_dah = false;
    cw_keyer_last = 0u;
    cw_key_down = false;
}
static void cw_keyer_start(bool dit) {
    cw_key_down = true;
    cw_keyer_last = dit ? 1u : 2u;
    cw_keyer_state = dit ? CWK_DIT_DOWN : CWK_DAH_DOWN;
    cw_keyer_deadline = make_timeout_time_ms(cw_keyer_dot_ms() * (dit ? 1u : 3u));
}
static void cw_keyer_update(bool dit, bool dah) {
    /* Straight-key mode: GP14/PTT is the key, GP15/DASH is ignored.
       RF shaping, sidetone shaping and TX hang remain downstream exactly as
       they are for the electronic keyer. */
    if (cw_keyer_mode == CW_KEY_STRAIGHT) {
        cw_keyer_state = CWK_IDLE;
        cw_keyer_pending_dit = false;
        cw_keyer_pending_dah = false;
        cw_keyer_last = 0u;
        cw_key_down = dit;
        return;
    }

    /* Iambic-B keeps the proven opposite-element memory.  Iambic-A uses the
       same timing engine but deliberately does NOT latch an opposite paddle
       after a squeeze is released; it finishes the current element and stops.
       If the squeeze is still held at the element boundary, both A and B
       continue alternating normally. */
    if (cw_keyer_mode == CW_KEY_IAMBIC_B) {
        if (cw_keyer_state == CWK_DIT_DOWN && dah) cw_keyer_pending_dah = true;
        if (cw_keyer_state == CWK_DAH_DOWN && dit) cw_keyer_pending_dit = true;
    }

    if (cw_keyer_state == CWK_DIT_DOWN || cw_keyer_state == CWK_DAH_DOWN) {
        if (!cw_keyer_deadline_passed()) return;
        cw_key_down = false;
        cw_keyer_state = (cw_keyer_state == CWK_DIT_DOWN) ? CWK_DIT_GAP : CWK_DAH_GAP;
        cw_keyer_deadline = make_timeout_time_ms(cw_keyer_dot_ms());
        return;
    }

    if (cw_keyer_state == CWK_DIT_GAP || cw_keyer_state == CWK_DAH_GAP) {
        if (cw_keyer_mode == CW_KEY_IAMBIC_B) {
            if (cw_keyer_last == 1u && dah) cw_keyer_pending_dah = true;
            if (cw_keyer_last == 2u && dit) cw_keyer_pending_dit = true;
        }
        if (!cw_keyer_deadline_passed()) return;

        if (cw_keyer_mode == CW_KEY_IAMBIC_B && cw_keyer_pending_dit) {
            cw_keyer_pending_dit = false;
            cw_keyer_start(true);
        } else if (cw_keyer_mode == CW_KEY_IAMBIC_B && cw_keyer_pending_dah) {
            cw_keyer_pending_dah = false;
            cw_keyer_start(false);
        } else if (dit && dah) {
            cw_keyer_start(cw_keyer_last == 2u);
        } else if (dit) {
            cw_keyer_start(true);
        } else if (dah) {
            cw_keyer_start(false);
        } else {
            cw_keyer_state = CWK_IDLE;
            cw_keyer_pending_dit = false;
            cw_keyer_pending_dah = false;
        }
        return;
    }

    if (dit && dah) cw_keyer_start(cw_keyer_last == 1u ? false : true);
    else if (dit) cw_keyer_start(true);
    else if (dah) cw_keyer_start(false);
}

/* Local CW RF envelope.  0 = fully off, CW_ENVELOPE_SAMPLES = fully on. */
static uint16_t cw_envelope_pos = 0u;

static inline float cw_envelope_next(bool key_down) {
    if (key_down) {
        if (cw_envelope_pos < CW_ENVELOPE_SAMPLES) {
            float env = cw_envelope_data[cw_envelope_pos];
            cw_envelope_pos++;
            return env;
        }
        return 1.0f;
    }

    if (cw_envelope_pos > 0u) {
        cw_envelope_pos--;
        return cw_envelope_data[cw_envelope_pos];
    }
    return 0.0f;
}
static volatile bool ssb_tx_zero_mic = false;
// Always initialized LOCAL on every boot; never stored in settings.
/* protocol1_mode defined at file scope above */
static p1_dsp_t p1_dsp;
static int32_t p1_i_accum[P1_SAMPLES_PER_PACKET];
static int32_t p1_q_accum[P1_SAMPLES_PER_PACKET];
static uint16_t p1_accum_count;

// Phase 3k: zero-copy DMA handoff.  The IRQ queues only the completed
// ping-pong half index; foreground DSP reads that half directly before
// DMA returns to it (~5.33 ms later).
#define P1_RAW_BLOCK_SAMPLES (HALF_WORDS / 2u)
#define P1_RAW_INDEX_QUEUE_SLOTS 3u
static volatile uint8_t p1_raw_index_queue[P1_RAW_INDEX_QUEUE_SLOTS];
static volatile uint8_t p1_raw_wr;
static volatile uint8_t p1_raw_rd;
static volatile uint32_t p1_raw_drops;
static bool p1_tx_primed;
static bool p1_last_mox;
#define P1_TX_PRIME_FRAMES 504u
static uint16_t cw_offset_hz = 700u;
/* CW7 calibration control: decoder center is independent of radio CW offset. */
static uint16_t cw_decode_pitch_hz = 500u;

/* CW9: the experimental pitch-following monitor filter was removed.
 * CW and SSB now share the same sharpened RX Low/RX High filter path. */

static uint8_t cw_sidetone_level = 40u;
static uint16_t cw_delay_ms = 300u;
static absolute_time_t cw_release_deadline;
static uint32_t cw_mod_phase, cw_side_phase;
static uint32_t twotone_phase_a, twotone_phase_b;
static bool si5351_ok = false;
static int16_t sine_table[SINE_SIZE];

typedef struct {
    uint32_t phase;
    uint32_t phase_inc;
    uint8_t level;
    bool enabled;
} ToneChannel;

static ToneChannel tone = {0, 0, 0, false};
static uint32_t tone_hz = 3000u;

// ---- Proven RadioCore v0.9 receive SSB DSP, adapted to signed 16-bit I2S ----
// 96 kHz real IF input -> complex mixer at RX IF -> two-pole LPF ->
// 4:1 decimation -> 65-tap Hilbert phasing at 24 kHz -> USB/LSB audio.
#define RX_LPF_ALPHA_Q15 5650
#define RX_HILBERT_TAPS 65
#define RX_HILBERT_DELAY 32

static const int16_t rx_hilbert_q15[RX_HILBERT_TAPS] = {
    -54, 0, -64, 0, -91, 0, -136, 0, -202, 0, -295, 0, -417, 0, -577, 0,
    -783, 0, -1052, 0, -1408, 0, -1904, 0, -2649, 0, -3931, 0, -6807, 0,
    -20812, 0, 0, 0, 20812, 0, 6807, 0, 3931, 0, 2649, 0, 1904, 0, 1408, 0,
    1052, 0, 783, 0, 577, 0, 417, 0, 295, 0, 202, 0, 136, 0, 91, 0, 64, 0, 54
};

static int32_t rx_hilbert_q[RX_HILBERT_TAPS];
static int32_t rx_delay_i[RX_HILBERT_TAPS];
static uint8_t rx_hilbert_pos;
static uint8_t rx_sideband_decim;
static int32_t rx_sideband_audio;

/* Phase 2M decoder/audio stream: process_rx_ssb_16() produces a new filtered
   sideband sample at 24 kHz.  Four-sample boxcar/CIC decimation yields a
   lightweight 6 kHz mono stream. UART transmission happens later in main. */
static int32_t panel_audio6k_accum=0;
static uint8_t panel_audio6k_decim=0;
/* Fourier Invasion #3: 96 kHz IFFT audio is averaged to the same 24 kHz
   update cadence used by the proven legacy RX/AGC/display path. */
static int32_t fourier_rx_24k_accum=0;
static uint32_t rx_mixer_phase;
static uint32_t rx_mixer_phase_inc;
static int32_t rx_dc_q8;
static bool rx_dc_valid;
static int32_t rx_i_lpf1, rx_i_lpf2;
static int32_t rx_q_lpf1, rx_q_lpf2;
static volatile uint32_t rx_clip_count;
static volatile uint32_t rx_if_over_count;
static volatile int32_t rx_if_peak;
static volatile int32_t rx_dac_peak;
// Equivalent to the old RadioCore i2sshift normalization, but explicit.
// The 16-bit slave backend is already aligned, so we multiply before the mixer.
static volatile uint8_t rx_if_gain = 8;
static volatile uint16_t rx_volume_percent = 100;
static uint16_t rx_low_hz=300u, rx_high_hz=2500u;
static audio_rx_biquad_bandpass_t rx_filter1;
static audio_rx_biquad_bandpass_t rx_filter2;
static volatile bool settings_dirty=false;
static absolute_time_t settings_save_deadline;
static uint8_t ui_step_index = 2;
/* Stock-panel operator state. Calibration BFO remains the absolute Si5351 value;
   panel BFO is only a signed trim around it. */
static uint8_t ui_active_vfo = 0;
static uint32_t ui_vfo_hz[2] = {7100000u, 14200000u};
static int16_t ui_bfo_delta_hz = 0;
static uint32_t ui_bfo_base_hz = 40037000u;
static const uint32_t ui_step_hz[8] = {10u,50u,100u,500u,1000u,2500u,5000u,10000u};

static uint32_t phase_inc_for_hz(uint32_t hz);
static void mark_settings_dirty(void);
static bool operating_mode_is_cw(void);

static void rx_filter_config(void) {
    audio_rx_biquad_config(&rx_filter1, 24000u, rx_low_hz, rx_high_hz, 0u);
    audio_rx_biquad_config(&rx_filter2, 24000u, rx_low_hz, rx_high_hz, 1u);
    audio_rx_biquad_reset(&rx_filter1);
    audio_rx_biquad_reset(&rx_filter2);
}

static inline int32_t rx_filter_process(int32_t x) {
    return audio_rx_biquad_process(&rx_filter2,
           audio_rx_biquad_process(&rx_filter1, x));
}

static inline int16_t clamp16(int32_t sample) {
    if (sample > 32767) { rx_clip_count++; return 32767; }
    if (sample < -32768) { rx_clip_count++; return -32768; }
    return (int16_t)sample;
}

static void rx_dsp_reset(void) {
    rx_hilbert_pos = 0;
    rx_sideband_decim = 0;
    rx_sideband_audio = 0;
    panel_audio6k_accum = 0;
    panel_audio6k_decim = 0;
    fourier_rx_24k_accum = 0;
    fft_ssb_rx_reset();
    spectrum_reset();
    rx_mixer_phase = 0;
    rx_dc_q8 = 0;
    rx_dc_valid = false;
    rx_i_lpf1 = rx_i_lpf2 = 0;
    rx_q_lpf1 = rx_q_lpf2 = 0;
    rx_clip_count = 0;
    rx_if_over_count = 0;
    rx_if_peak = 0;
    rx_dac_peak = 0;
    audio_rx_biquad_reset(&rx_filter1);
    audio_rx_biquad_reset(&rx_filter2);
    rx_agc_reset();
    memset(rx_hilbert_q, 0, sizeof(rx_hilbert_q));
    memset(rx_delay_i, 0, sizeof(rx_delay_i));
}

static inline int32_t rx_sideband_phasing(int32_t i_sample, int32_t q_sample) {
    rx_hilbert_q[rx_hilbert_pos] = q_sample;
    rx_delay_i[rx_hilbert_pos] = i_sample;

    int64_t acc = 0;
    uint8_t rp = rx_hilbert_pos;
    for (uint8_t k = 0; k < RX_HILBERT_TAPS; ++k) {
        acc += (int64_t)rx_hilbert_q15[k] * rx_hilbert_q[rp];
        rp = (rp == 0) ? (RX_HILBERT_TAPS - 1) : (rp - 1);
    }

    uint8_t delay_pos = (uint8_t)((rx_hilbert_pos + RX_HILBERT_TAPS - RX_HILBERT_DELAY) % RX_HILBERT_TAPS);
    int32_t delayed_i = rx_delay_i[delay_pos];
    int32_t hilbert_q = (int32_t)(acc >> 15);
    rx_hilbert_pos++;
    if (rx_hilbert_pos >= RX_HILBERT_TAPS) rx_hilbert_pos = 0;

    // Same polarity as RadioCore v0.9. Swap signs here only if bench testing
    // proves the hardware spectrum is reversed.
    return (sideband == SIDEBAND_USB)
        ? ((delayed_i - hilbert_q) >> 1)
        : ((delayed_i + hilbert_q) >> 1);
}

static inline int16_t process_rx_ssb_16(int16_t input) {
    int32_t raw = input;
    if (!rx_dc_valid) {
        rx_dc_q8 = raw << 8;
        rx_dc_valid = true;
    }
    rx_dc_q8 += (((raw << 8) - rx_dc_q8) >> 12);
    int32_t centered = raw - (rx_dc_q8 >> 8);

    // Pre-demod IF normalization. This replaces the old packed-I2S
    // i2sshift control. Keep the DSP math wide; only the final DAC sample
    // is saturated to 16 bits. Count excursions beyond a nominal 16-bit
    // range so the selected IF gain can be evaluated on strong signals.
    centered *= (int32_t)rx_if_gain;
    int32_t abs_if = centered < 0 ? (int32_t)(-(int64_t)centered) : centered;
    if (abs_if > rx_if_peak) rx_if_peak = abs_if;
    if (centered > 32767 || centered < -32768) rx_if_over_count++;

    rx_mixer_phase += rx_mixer_phase_inc;
    uint8_t idx = (uint8_t)(rx_mixer_phase >> 24);
    int32_t cosv = sine_table[(uint8_t)(idx + 64)];
    int32_t sinv = sine_table[idx];
    int32_t mixed_i = (int32_t)(((int64_t)centered * cosv) >> 15);
    int32_t mixed_q = (int32_t)(-(((int64_t)centered * sinv) >> 15));

    rx_i_lpf1 += (int32_t)(((int64_t)(mixed_i - rx_i_lpf1) * RX_LPF_ALPHA_Q15) >> 15);
    rx_i_lpf2 += (int32_t)(((int64_t)(rx_i_lpf1 - rx_i_lpf2) * RX_LPF_ALPHA_Q15) >> 15);
    rx_q_lpf1 += (int32_t)(((int64_t)(mixed_q - rx_q_lpf1) * RX_LPF_ALPHA_Q15) >> 15);
    rx_q_lpf2 += (int32_t)(((int64_t)(rx_q_lpf1 - rx_q_lpf2) * RX_LPF_ALPHA_Q15) >> 15);

    /*
     * Little Spectrum has its own 96 kHz anti-alias FIR and 4:1 decimator.
     * Feed the mathematical mixer output before the narrow RX filters; the
     * display path is therefore independent of the listening bandwidth.
     */
    /* Legacy sample-fed spectrum removed; FFT power drives the display. */

    if (++rx_sideband_decim >= 4) {
        rx_sideband_decim = 0;
        rx_sideband_audio = rx_sideband_phasing(rx_i_lpf2, rx_q_lpf2);
        /* Phase 2N-FILTER: true 4th-order Butterworth HP + 4th-order
           Butterworth LP.  Operator audio and the 6 kHz display/modem stream
           receive this exact same filtered audio. */
        rx_sideband_audio = rx_filter_process(rx_sideband_audio);

        /* 24 kHz -> 6 kHz.  Keep this ISR path tiny: only add/shift and a
           non-blocking ring write every fourth 24 kHz sample. */
        panel_audio6k_accum += rx_sideband_audio;
        if (++panel_audio6k_decim >= 4u) {
            int32_t a6 = panel_audio6k_accum >> 2;
            panel_audio6k_accum = 0;
            panel_audio6k_decim = 0;
            if (a6 > 32767) a6 = 32767;
            if (a6 < -32768) a6 = -32768;
            front_panel_audio_capture_6k((int16_t)a6);
            ft8_n5_feed_6k((int16_t)a6);
        }

        /* Phase 2N: CW decoding is now offloaded to the Display Pico.
           The 6 kHz UART stream above carries the filtered audio. */
    }

    int32_t agc_audio = rx_agc_process(rx_sideband_audio);
    int32_t out = (int32_t)(((int64_t)agc_audio * rx_volume_percent) / 100);
    int32_t abs_out = out < 0 ? (int32_t)(-(int64_t)out) : out;
    if (abs_out > rx_dac_peak) rx_dac_peak = abs_out;
    return clamp16(out);
}

/* Fourier Invasion #3: contained Local USB/LSB receive path.
 * The old quadrature mixer is retained ONLY as a feed for the already-proven
 * main spectrum display.  Listening audio itself never enters the old mixer,
 * Hilbert phasing network, or time-domain RX filters. */
static inline int16_t process_rx_fourier_ssb_16(int16_t input) {
    int32_t raw = input;
    if (!rx_dc_valid) {
        rx_dc_q8 = raw << 8;
        rx_dc_valid = true;
    }
    rx_dc_q8 += (((raw << 8) - rx_dc_q8) >> 12);
    int32_t centered = raw - (rx_dc_q8 >> 8);

    centered *= (int32_t)rx_if_gain;
    int32_t abs_if = centered < 0 ? (int32_t)(-(int64_t)centered) : centered;
    if (abs_if > rx_if_peak) rx_if_peak = abs_if;
    if (centered > 32767 || centered < -32768) rx_if_over_count++;

    /* Invasion #4: no legacy visualization mixer here.  The 512-point
       Fourier RX FFT itself publishes the main-spectrum power frame. */

    int16_t audio96 = fft_ssb_rx_process(centered,
        operating_mode == OPERATING_AM ? FFT_RX_AM :
        (sideband == SIDEBAND_USB ? FFT_RX_USB : FFT_RX_LSB));

    /* Match the legacy sideband-audio update cadence: 96 kHz -> 24 kHz. */
    fourier_rx_24k_accum += audio96;
    if (++rx_sideband_decim >= 4u) {
        rx_sideband_decim = 0;
        rx_sideband_audio = fourier_rx_24k_accum >> 2;
        fourier_rx_24k_accum = 0;

        /* v0.7L: reconnect the proven pre-FFT CW decoder to the new receiver.
           It was written for 24 kHz audio, exactly this post-IFFT point. */
        if (operating_mode_is_cw()) cw_decoder_feed(rx_sideband_audio);

        /* Preserve the exact 24 kHz -> 6 kHz Display Pico decoder stream. */
        panel_audio6k_accum += rx_sideband_audio;
        if (++panel_audio6k_decim >= 4u) {
            int32_t a6 = panel_audio6k_accum >> 2;
            panel_audio6k_accum = 0;
            panel_audio6k_decim = 0;
            if (a6 > 32767) a6 = 32767;
            if (a6 < -32768) a6 = -32768;
            front_panel_audio_capture_6k((int16_t)a6);
            ft8_n5_feed_6k((int16_t)a6);
        }
    }

    int32_t agc_audio = rx_agc_process(rx_sideband_audio);
    int32_t out = (int32_t)(((int64_t)agc_audio * rx_volume_percent) / 100);
    int32_t abs_out = out < 0 ? (int32_t)(-(int64_t)out) : out;
    if (abs_out > rx_dac_peak) rx_dac_peak = abs_out;
    return clamp16(out);
}

static const char *operating_mode_name(void) {
    switch (operating_mode) {
        case OPERATING_USB: return "USB";
        case OPERATING_LSB: return "LSB";
        case OPERATING_CWU: return "CWU";
        case OPERATING_CWL: return "CWL";
        case OPERATING_AM: return "AM";
        case OPERATING_2TONE: return "2TONE";
        case OPERATING_FT8: return "FT8";
        default: return "?";
    }
}

static bool operating_mode_is_cw(void) {
    return operating_mode == OPERATING_CWU || operating_mode == OPERATING_CWL;
}

static void update_rx_mixer_frequency(void) {
    uint32_t hz = radio_get_if_hz();
    if (operating_mode == OPERATING_CWU) hz = hz > cw_offset_hz ? hz - cw_offset_hz : 10u;
    else if (operating_mode == OPERATING_CWL) hz += cw_offset_hz;
    rx_mixer_phase_inc = phase_inc_for_hz(hz);
    /* Fourier Invasion #5: CW joins the shared Fourier RX engine.  Center
       the Fourier translator exactly where the legacy CW mixer was centered:
       CWU below the global IF and CWL above it.  A carrier at the normal IF
       therefore appears at the selected CW offset (normally 700 Hz). */
    fft_ssb_rx_set_if_hz(hz);
    rx_dsp_reset();
}

static void set_operating_mode(OperatingMode m) {
    operating_mode = m;
    if (m != OPERATING_AM) sideband = (m == OPERATING_USB || m == OPERATING_CWU || m == OPERATING_2TONE || m == OPERATING_FT8) ? SIDEBAND_USB : SIDEBAND_LSB;
    update_rx_mixer_frequency();
    cw_decoder_reset();
    /* Display Pico owns normal frequency/mode/step persistence now.
       Manual Save Settings still captures the complete RadioCore state. */
}

// Radio tuning wrapper: ordinary VFO movement must stay lightweight.
// Do NOT reset RX DSP state on every encoder step.  Resetting the mixer/filter/
// AGC history for each detent makes LOCAL tuning sound stepped and stalls the
// local spectrum path.  DSP resets remain in the places where DSP configuration
// really changes (mode/IF/etc.).
static bool receiver_tune_hz(uint32_t new_dial_hz) {
    if (!radio_tune_hz(new_dial_hz)) return false;

    radio_io_set_frequency(new_dial_hz);

    // Keep TX calibration current for the new dial frequency.
    ssb_tx_set_band_multiplier_q16(power_calibration_get_multiplier_q16(new_dial_hz));

    // Tuning itself does not require restarting the receive DSP.
    audio_mode = AUDIO_MODE_SSB;
    tone.level = 0u;
    tone.enabled = false;

    // Intentionally no rx_dsp_reset() here.
    // Also avoid a printf on every tuning detent; console I/O can add visible/
    // audible latency during a fast encoder spin.

    return true;
}

static bool rit_rf_applied = false;
static void rit_sync_rf_state(void) {
    /* True RIT: apply offset only in LOCAL RX.  TX and Protocol 1 always use
       the unshifted dial frequency. */
    bool want_rx_offset = (!protocol1_mode && !radio_io_is_tx());
    if (want_rx_offset != rit_rf_applied) {
        if (radio_set_rit_rx_active(want_rx_offset))
            rit_rf_applied = want_rx_offset;
    }
}

static uint32_t phase_inc_for_hz(uint32_t hz) {
    return (uint32_t)(((uint64_t)hz << 32) / SAMPLE_RATE);
}

static void tone_set(uint32_t hz, uint8_t level) {
    if (hz < 10u || hz > 45000u) return;
    if (level > 100u) level = 100u;
    uint32_t new_inc = phase_inc_for_hz(hz);
    if (!tone.enabled || tone.phase_inc != new_inc) tone.phase = 0;
    tone.phase_inc = new_inc;
    tone.level = level;
    tone.enabled = level != 0u;
    tone_hz = hz;
}

static inline int32_t tone_word16(void) {
    if (!tone.enabled || tone.level == 0u) return 0;

    // Same DDS structure used by RadioCore: 32-bit phase accumulator and
    // the top eight bits selecting a 256-entry signed Q15 sine table.
    tone.phase += tone.phase_inc;
    uint8_t idx = (uint8_t)(tone.phase >> 24);

    // RadioCore scales the Q15 table with an integer percentage. For this
    // 16-bit diagnostic, retain a signed 16-bit sample and left-align it in
    // the 32-bit PIO/DMA word.
    int32_t sample = ((int32_t)sine_table[idx] * tone.level + 50) / 100;
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    return ((int32_t)(int16_t)sample) << 16;
}

static void fill_tone_half(uint half) {
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        int32_t word = tone_word16();
        dst[i + 0] = word; // DAC left: EAR
        dst[i + 1] = word; // DAC right: AUDIO-MOD (diagnostic mode only)
    }
}

static void fill_ssb_receiver_half(uint half) {
    const int32_t *src = &input_buffer[half * HALF_WORDS];
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    /* Invasion #3: existing RX LOW/HIGH controls now drive the Fourier mask. */
    if (operating_mode == OPERATING_AM) fft_ssb_rx_set_filter(50u, am_rx_bw_hz/2u);
    else fft_ssb_rx_set_filter(rx_low_hz, rx_high_hz);
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        // Proven hardware routing:
        // ADC LEFT = AUDIO-DEMOD; DAC RIGHT = EAR/headphones.
        int16_t real_if = (int16_t)(src[i + 0] & 0xffff);
        /* Invasion #5: USB/LSB/CWU/CWL now share the Fourier RX engine.
           CWU uses the USB spectral orientation and CWL the LSB orientation;
           update_rx_mixer_frequency() offsets the Fourier center so a tuned
           CW carrier lands at cw_offset_hz in the recovered audio. */
        int16_t audio = process_rx_fourier_ssb_16(real_if);
        dst[i + 0] = 0;                              // DAC LEFT / AUDIO-MOD muted
        dst[i + 1] = ((int32_t)audio) << 16;          // DAC RIGHT / EAR
    }
}

static void fill_raw_receiver_half(uint half) {
    const int32_t *src = &input_buffer[half * HALF_WORDS];
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        int16_t demod = (int16_t)(src[i + 0] & 0xffff);
        dst[i + 0] = 0;
        dst[i + 1] = ((int32_t)demod) << 16;
    }
}

static void fill_mic_monitor_half(uint half) {
    const int32_t *src = &input_buffer[half * HALF_WORDS];
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        int16_t mic = (int16_t)(src[i + 1] & 0xffff);
        dst[i + 0] = 0;                              // AUDIO-MOD muted
        dst[i + 1] = ((int32_t)mic) << 16;           // EAR/headphones
    }
}


/* TXCAL v1: independent band amplitude ceilings, never a 40m reference.
   Runtime changes are deliberately NOT persisted by ordinary settings saves. */
static const uint32_t txcal_low[9]={3500000u,5330000u,7000000u,10100000u,14000000u,18068000u,21000000u,24890000u,28000000u};
static const uint32_t txcal_high[9]={4000000u,5405000u,7300000u,10150000u,14350000u,18168000u,21450000u,24990000u,29700000u};
static const uint8_t txcal_band_m[9]={80,60,40,30,20,17,15,12,10};
static uint8_t txcal_runtime[9]={10,10,10,10,10,10,10,10,10};
static uint8_t txcal_committed[9]={10,10,10,10,10,10,10,10,10};
static uint8_t txcal_band_index(uint32_t hz) {
    for(uint8_t i=0;i<9;i++) if(hz>=txcal_low[i] && hz<=txcal_high[i]) return i;
    return 255;
}
static inline int16_t txcal_apply(int16_t sample) {
    uint8_t b=txcal_band_index(radio_get_dial_hz());
    if(b==255) return sample; /* Existing OOB inhibit remains responsible for TX. */
    int32_t n=(int32_t)sample*txcal_runtime[b];
    return (int16_t)(n/100);
}
static void txcal_print(void) {
    printf("TXCAL: runtime / saved amplitude ceilings (%%)\n");
    for(uint8_t i=0;i<9;i++)
        printf("  %um: %u / %u%s\n",(unsigned)txcal_band_m[i],
               (unsigned)txcal_runtime[i],(unsigned)txcal_committed[i],
               txcal_runtime[i]==txcal_committed[i]?"":" (unsaved)");
}

static void fill_ssb_transmitter_half(uint half) {
    const int32_t *src = &input_buffer[half * HALF_WORDS];
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    /* Fourier Invasion #2: mirror the existing TX LOW/HIGH controls into
       the contained Local SSB spectral mask.  CAT/P1 paths are untouched. */
    fft_ssb_tx_set_filter(ssb_tx_get_low_hz(), ssb_tx_get_high_hz());
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        // Proven routing: ADC RIGHT = microphone, DAC LEFT = AUDIO-MOD.
        int16_t mic;
        if(ft8_tx6_on && operating_mode == OPERATING_FT8){
            mic=ft8_tx6_next_96k();
        }else if(civ_remote_ptt){
            // CAT TX uses the Display Pico USB playback stream, not the codec mic.
            mic=civ_usb_tx_audio_96k();
        }else if(operating_mode == OPERATING_2TONE){
            /* Bench two-tone generator: equal 700 Hz + 1900 Hz tones at the
               normal 96 kHz TX input point.  Each tone is limited to 25%
               full scale so their sum has comfortable FFT-TX headroom. */
            twotone_phase_a += phase_inc_for_hz(700u);
            twotone_phase_b += phase_inc_for_hz(1900u);
            int32_t a = sine_table[(uint8_t)(twotone_phase_a >> 24)];
            int32_t b = sine_table[(uint8_t)(twotone_phase_b >> 24)];
            mic = (int16_t)((a + b) / 4);
        }else{
            mic=(int16_t)(src[i + 1] & 0xffff);
            /* Fourier path applies safe float gain; avoid old int16 wrap. */
            if (ssb_tx_zero_mic) mic = 0;
        }

        /* Physical PTT has already gone inactive, but radio_io deliberately
           holds the TX hardware on for ~2 ms.  During that guard interval,
           force AUDIO-MOD to exact zero instead of allowing one last SSB
           sample/filter tail to be chopped by the RF switching edge. */
        int16_t mod = 0;
        if (ft8_tx6_on && operating_mode == OPERATING_FT8) {
            mod = fft_ssb_tx_process(mic, FFT_SSB_USB);
        } else if (civ_remote_ptt) {
            /* Preserve the proven Display/USB CAT TX path. */
            mod = ssb_tx_process(mic,
                sideband == SIDEBAND_USB ? SSB_TX_USB : SSB_TX_LSB);
        } else if (operating_mode != OPERATING_FT8 && (radio_io_ptt() || panel_remote_ptt)) {
            /* Local microphone TX: physical PTT or stock-panel TX button. */
            mod = fft_ssb_tx_process(mic,
                operating_mode == OPERATING_AM ? FFT_SSB_AM :
                (sideband == SIDEBAND_USB ? FFT_SSB_USB : FFT_SSB_LSB));
        }
        dst[i + 0] = ((int32_t)txcal_apply(mod)) << 16; // DAC LEFT / AUDIO-MOD

        /* Phase 2CN diagnostic: while display/CAT TX is active, monitor the
           EXACT 96 kHz sample returned by the 6 kHz UART queue/interpolator
           on the EAR channel.  This is intentionally tapped BEFORE the SSB
           modulator so headphone continuity tells us whether the incoming
           stream itself is continuous.  Attenuate by 4 for comfortable test
           listening. */
        /* Use the existing CW sidetone setting as the display/CAT TX monitor
           volume.  0% = silent (useful for FreeDV), 100% = the previous
           proven /4 monitor level.  This changes EAR monitoring only; the
           unscaled mic sample above still feeds ssb_tx_process() and RF. */
        int16_t txmon = 0;
        if(civ_remote_ptt && cw_sidetone_level){
            int32_t m = ((int32_t)mic * (int32_t)cw_sidetone_level) / 400;
            if(m > 32767) m = 32767;
            if(m < -32768) m = -32768;
            txmon = (int16_t)m;
        }
        dst[i + 1] = ((int32_t)txmon) << 16; // DAC RIGHT / EAR TX monitor
    }
}

static inline int16_t cw_dds_sample(uint32_t *phase, uint32_t hz, uint8_t level) {
    *phase += phase_inc_for_hz(hz);
    int32_t sample = ((int32_t)sine_table[(uint8_t)(*phase >> 24)] * level + 50) / 100;
    if (sample > 32767) sample = 32767;
    if (sample < -32768) sample = -32768;
    return (int16_t)sample;
}

static void fill_cw_transmitter_half(uint half) {
    int32_t *dst = &output_buffer[half * HALF_WORDS];
    uint32_t tx_tone_hz = ssb_tx_get_if_hz();
    if (operating_mode == OPERATING_CWU) tx_tone_hz += cw_offset_hz;
    else if (tx_tone_hz > cw_offset_hz) tx_tone_hz -= cw_offset_hz;
    uint8_t drive = ssb_tx_get_drive();
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        /* Walk the proven sBITX Blackman-Harris table forward on key-down
           and backward on key-up.  Use the same envelope for RF and sidetone
           so repeated elements stay click-free and sound like the RF. */
        float env = cw_envelope_next(cw_key_down);
        int16_t raw_mod = (env > 0.0f) ? cw_dds_sample(&cw_mod_phase, tx_tone_hz, drive) : 0;
        int32_t shaped = (int32_t)((float)raw_mod * env);
        if (shaped > 32767) shaped = 32767;
        if (shaped < -32768) shaped = -32768;
        int16_t mod = (int16_t)shaped;

        int16_t raw_side = (env > 0.0f) ? cw_dds_sample(&cw_side_phase, cw_offset_hz, cw_sidetone_level) : 0;
        int32_t shaped_side = (int32_t)((float)raw_side * env);
        if (shaped_side > 32767) shaped_side = 32767;
        if (shaped_side < -32768) shaped_side = -32768;
        int16_t side = (int16_t)shaped_side;
        dst[i + 0] = ((int32_t)txcal_apply(mod)) << 16;
        dst[i + 1] = ((int32_t)side) << 16;
    }
}

static void audio_dma_irq(void) {
    dma_hw->ints0 = 1u << dma_in_data;
    uint half = completed_half;
    completed_half ^= 1u;
    dma_irqs++;

    int32_t *src = &input_buffer[half * HALF_WORDS];

    // Protocol 1 Phase 3k: keep the DMA IRQ as close to zero-cost as possible.
    // Queue only which ping-pong half completed; no sample copy, mixer, FIR,
    // packet work, USB or OLED activity occurs here.
    if (protocol1_mode) {
        uint8_t wr = __atomic_load_n(&p1_raw_wr, __ATOMIC_RELAXED);
        uint8_t next = (uint8_t)((wr + 1u) % P1_RAW_INDEX_QUEUE_SLOTS);
        uint8_t rd = __atomic_load_n(&p1_raw_rd, __ATOMIC_ACQUIRE);
        if (next != rd) {
            p1_raw_index_queue[wr] = (uint8_t)half;
            __atomic_store_n(&p1_raw_wr, next, __ATOMIC_RELEASE);
        } else {
            p1_raw_drops++;
        }
        return;
    }

    int32_t peak_l = 0, peak_r = 0;
    for (uint i = 0; i < HALF_WORDS; i += 2) {
        int32_t l = (int16_t)(src[i] & 0xffff);
        int32_t r = (int16_t)(src[i + 1] & 0xffff);
        int32_t al = l < 0 ? (int32_t)(-(int64_t)l) : l;
        int32_t ar = r < 0 ? (int32_t)(-(int64_t)r) : r;
        if (al > peak_l) peak_l = al;
        if (ar > peak_r) peak_r = ar;
    }
    adc_peak_l = peak_l;
    adc_peak_r = peak_r;

    // Phase 7: detect RX/TX edges, reset the TX phasing state once, then
    // route microphone ADC RIGHT through the SSB generator to DAC LEFT.
    static bool was_tx = false;
    bool now_tx = radio_io_is_tx();
    if (now_tx != was_tx) {
        if (now_tx) {
            ssb_tx_reset();
            fft_ssb_tx_reset();
            twotone_phase_a = twotone_phase_b = 0u;
        } else {
            /* CW must leave no generated audio/envelope state behind when
               the hang timer returns the radio to RX. */
            if (operating_mode_is_cw()) {
                cw_envelope_pos = 0u;
                cw_mod_phase = 0u;
                cw_side_phase = 0u;
            }
            rx_dsp_reset();
        }
        was_tx = now_tx;
    }
    if (now_tx) {
        if (operating_mode_is_cw()) fill_cw_transmitter_half(half);
        else fill_ssb_transmitter_half(half);
        return;
    }

    if (audio_mode == AUDIO_MODE_SSB) fill_ssb_receiver_half(half);
    else if (audio_mode == AUDIO_MODE_RAW_RECEIVER) fill_raw_receiver_half(half);
    else if (audio_mode == AUDIO_MODE_MIC_MONITOR) fill_mic_monitor_half(half);
    else fill_tone_half(half);
}

static void __attribute__((noinline)) p1_process_raw_blocks(void) {
    int16_t real96[P1_RAW_BLOCK_SAMPLES];
    int32_t i48[P1_RAW_BLOCK_SAMPLES / 2u];
    int32_t q48[P1_RAW_BLOCK_SAMPLES / 2u];
    int16_t tx_iq[(P1_RAW_BLOCK_SAMPLES / 2u) * 2u];
    int16_t tx_real[P1_RAW_BLOCK_SAMPLES];

    while (true) {
        uint8_t rd = __atomic_load_n(&p1_raw_rd, __ATOMIC_RELAXED);
        uint8_t wr = __atomic_load_n(&p1_raw_wr, __ATOMIC_ACQUIRE);
        if (rd == wr) break;

        uint8_t half = p1_raw_index_queue[rd];
        rd = (uint8_t)((rd + 1u) % P1_RAW_INDEX_QUEUE_SLOTS);
        __atomic_store_n(&p1_raw_rd, rd, __ATOMIC_RELEASE);

        const p1_network_status_t *pn = p1_network_status();
        bool tx = pn->mox_requested;
        int32_t *dst = &output_buffer[(uint)half * HALF_WORDS];

        if (tx) {
            /* P1 TX: 128 complex frames @48k -> 256 real IF samples @96k.
               Prime a small FIFO first so 126-frame EP2 packet boundaries
               never create a periodic two-sample underrun. */
            unsigned need48 = P1_RAW_BLOCK_SAMPLES / 2u;
            if (!p1_tx_primed && p1_network_tx_iq_available() >= (int)P1_TX_PRIME_FRAMES)
                p1_tx_primed = true;

            int got = 0;
            if (p1_tx_primed)
                got = p1_network_read_tx_iq(tx_iq, (int)need48);
            if (got < (int)need48) {
                memset(&tx_iq[got * 2], 0,
                       ((size_t)need48 - (size_t)got) * 2u * sizeof(int16_t));
                if (p1_tx_primed) p1_network_note_tx_underflow();
            }

            int out96 = p1_tx_iq48_to_real96(&p1_dsp, tx_iq, (int)need48,
                                               tx_real, P1_RAW_BLOCK_SAMPLES);
            for (uint n = 0; n < P1_RAW_BLOCK_SAMPLES; ++n) {
                int16_t mod = n < (uint)out96 ? tx_real[n] : 0;
                dst[n * 2u + 0u] = ((int32_t)txcal_apply(mod)) << 16; /* DAC LEFT = AUDIO-MOD; per-band ceiling */
                dst[n * 2u + 1u] = 0;                    /* EAR muted in P1 TX */
            }

            /* Keep EP6 cadence alive cheaply during TX without running the RX FIR. */
            for (uint n = 0; n < need48; ++n) {
                p1_i_accum[p1_accum_count] = 0;
                p1_q_accum[p1_accum_count] = 0;
                if (++p1_accum_count == P1_SAMPLES_PER_PACKET) {
                    (void)p1_network_queue_rx_iq(p1_i_accum, p1_q_accum);
                    p1_accum_count = 0;
                }
            }
            continue;
        }

        /* RX: ensure AUDIO-MOD stays silent, then run the proven 3K path. */
        for (uint n = 0; n < P1_RAW_BLOCK_SAMPLES; ++n) {
            dst[n * 2u + 0u] = 0;
            dst[n * 2u + 1u] = 0;
        }

        int32_t *src = &input_buffer[(uint)half * HALF_WORDS];
        for (uint n = 0; n < P1_RAW_BLOCK_SAMPLES; ++n)
            real96[n] = (int16_t)(src[n * 2u] & 0xffff);

        int frames = p1_rx_real96_to_iq48(&p1_dsp, real96,
                                           P1_RAW_BLOCK_SAMPLES, i48, q48,
                                           P1_RAW_BLOCK_SAMPLES / 2u);

        for (int n = 0; n < frames; ++n) {
            p1_i_accum[p1_accum_count] = i48[n];
            p1_q_accum[p1_accum_count] = -q48[n];
            if (++p1_accum_count == P1_SAMPLES_PER_PACKET) {
                (void)p1_network_queue_rx_iq(p1_i_accum, p1_q_accum);
                p1_accum_count = 0;
            }
        }
    }
}

static void setup_dma(void) {
    in_blocks[0] = &input_buffer[0];
    in_blocks[1] = &input_buffer[HALF_WORDS];
    out_blocks[0] = &output_buffer[0];
    out_blocks[1] = &output_buffer[HALF_WORDS];

    dma_out_ctrl = dma_claim_unused_channel(true);
    dma_out_data = dma_claim_unused_channel(true);
    dma_in_ctrl = dma_claim_unused_channel(true);
    dma_in_data = dma_claim_unused_channel(true);

    dma_channel_config c = dma_channel_get_default_config(dma_out_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(dma_out_ctrl, &c,
        &dma_hw->ch[dma_out_data].al3_read_addr_trig,
        out_blocks, 1, false);

    c = dma_channel_get_default_config(dma_out_data);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_chain_to(&c, dma_out_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(audio_pio, audio_sm, true));
    dma_channel_configure(dma_out_data, &c,
        &audio_pio->txf[audio_sm], NULL, HALF_WORDS, false);

    c = dma_channel_get_default_config(dma_in_ctrl);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 3);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    dma_channel_configure(dma_in_ctrl, &c,
        &dma_hw->ch[dma_in_data].al2_write_addr_trig,
        in_blocks, 1, false);

    c = dma_channel_get_default_config(dma_in_data);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_chain_to(&c, dma_in_ctrl);
    channel_config_set_dreq(&c, pio_get_dreq(adc_pio, adc_sm, false));
    dma_channel_configure(dma_in_data, &c,
        NULL, &adc_pio->rxf[adc_sm], HALF_WORDS, false);

    dma_channel_set_irq0_enabled(dma_in_data, true);
    irq_set_exclusive_handler(DMA_IRQ_0, audio_dma_irq);
    /* Phase 2CX: keep audio DMA below UART0 RX so incoming 2 Mbaud display
       bytes can interrupt the longer DSP handler instead of overflowing the
       UART FIFO. */
    irq_set_priority(DMA_IRQ_0,0x80);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_channel_start(dma_out_ctrl);
    dma_channel_start(dma_in_ctrl);
}

/* v0.7h: explicit TinyUSB CDC service console.  The firmware owns the USB
   device descriptors, so do not depend on Pico stdio_usb.  Route the existing
   command/status printf text through the already-enumerated CDC interface. */
static int cdc_console_printf(const char *fmt, ...) {
    char out[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(out, sizeof(out), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    size_t send = (size_t)n;
    if (send >= sizeof(out)) send = sizeof(out)-1u;
    if (tud_cdc_connected()) {
        tud_cdc_write(out, (uint32_t)send);
        tud_cdc_write_flush();
    }
    return n;
}
static void print_help(void) {
    printf("\nRagchewCore console (commands are lowercase except TIME/RTC?)\n");
    printf("  help | ?                this command list\n");
    printf("  status | s              live radio and configuration snapshot\n");
    printf("  freq <kHz|Hz>           dial frequency; mode usb|lsb|cwu|cwl|am\n");
    printf("  vfo a|b                 select VFO; vfoa/vfob <Hz> set memory\n");
    printf("  step <0..7>             tuning step index (10,50,100,500,1000,2500,5000,10000 Hz)\n");
    printf("  rit <-9999..9999>      receiver incremental tuning Hz\n");
    printf("  band <0..8>             select band (80,60,40,30,20,17,15,12,10m)\n");
    printf("  rxlow 50..2000 | rxhigh 150..4000 | txlow 50..1000 | txhigh 1200..4000\n");
    printf("  amrx 2000..10000       AM receive bandwidth Hz\n");
    printf("  amcarrier 0..100       AM transmit carrier percent\n");
    printf("  spectrum -20..20       waterfall/spectrum offset dB\n");
    printf("  dsp on|off             RX DSP noise reduction\n");
    printf("  agc on|off|fast|slow   AGC; agctarget 5..90; agcmax 1..64\n");
    printf("  ifgain 1..64 | volume 0..800\n");
    printf("  rxcodecin | miccodecin | earcodecout | modcodecout 0..100\n");
    printf("  micgain 0..1600 | txdrive 0..100 | txif 1000..47000\n");
    printf("  TXCAL SHOW | TXCAL <band> <0..100> | TXCAL SAVE | TXCAL RESET\n");
    printf("  txzero on|off          TX diagnostic zero mic\n");
    printf("  cwoffset 400..1200 | cwdecodepitch 200..1000\n");
    printf("  sidetone 0..100 | cwwpm 5..60 | cwdelay 0..2000\n");
    printf("  keyer straight|a|b     CW keyer mode\n");
    printf("  bfo 39000000..41000000 | bfodelta <-10000..10000>\n");
    printf("  globalif 1000..47000 | clk1drive 2|4|6|8 | clk2drive 2|4|6|8\n");
    printf("  ft8tx MESSAGE | ft8abort | ft8first on|off (EXPERIMENTAL RF TX)\n");
    printf("  tx on|off|auto         manual TX test / physical PTT (CAUTION RF)\n");
    printf("  lpf a|b|c|d|off|auto   LPF selection\n");
    printf("  TIME <unix_seconds.ms>  set UTC and queue DS3231 write\n");
    printf("  TIME? | RTC?           inspect clock and RTC\n");
    printf("  save                   write supported persistent settings to SD\n");
    printf("  bootsel | program      enter USB firmware programming\n");
    printf("  Note: some UI-only values are not yet included in the SD settings schema.\n\n");
}

static void print_status(void) {
    printf("\n%s  %lu.%03lu kHz  %s  PTT=%s  LPF=%s/%s\n",
           radio_io_is_tx() ? "TX" : "RX",
           (unsigned long)(radio_get_dial_hz() / 1000u),
           (unsigned long)(radio_get_dial_hz() % 1000u),
           operating_mode_name(),
           radio_io_ptt() ? "DOWN" : "UP",
           radio_io_lpf_name(radio_io_get_lpf()),
           radio_io_get_lpf_mode() == RADIO_LPF_MODE_AUTO ? "AUTO" : "MANUAL");
    printf("Filters: RX=%u-%u Hz TX=%u-%u Hz\n", rx_low_hz, rx_high_hz, ssb_tx_get_low_hz(), ssb_tx_get_high_hz());
    printf("RX: in=%u%% ifgain=%ux agc=%s/%s volume=%u%% ear=%u%%\n",
           wm8731_get_rx_input_percent(), (unsigned)rx_if_gain,
           rx_agc_get_enabled() ? "ON" : "OFF",
           rx_agc_get_speed() == RX_AGC_FAST ? "FAST" : "SLOW",
           (unsigned)rx_volume_percent, wm8731_get_ear_output_percent());
    printf("TX: armed=ON micin=%u%% fixedDSP=%u%% CWdrive=%u%% mod=%u%% IF=%lu Hz zero=%s\n",
           wm8731_get_mic_input_percent(), ssb_tx_get_mic_gain(),
           ssb_tx_get_drive(), wm8731_get_mod_output_percent(),
           (unsigned long)ssb_tx_get_if_hz(), ssb_tx_zero_mic ? "ON" : "OFF");
    printf("RF: band=%s scale=%.4f multiplier=%lu.%03lux %s\n",
           power_calibration_get_band_name(radio_get_dial_hz()),
           (double)power_calibration_get_scale(radio_get_dial_hz()),
           (unsigned long)(ssb_tx_get_band_multiplier_q16() >> 16),
           (unsigned long)(((ssb_tx_get_band_multiplier_q16() & 0xffffu) * 1000u) >> 16),
           power_calibration_tx_allowed(radio_get_dial_hz()) ? "ENABLED" : "OUT-OF-BAND/MUTED");
    printf("Meter: RXadc=%ld MIC=%ld MOD=%ld RXclips=%lu TXclips=%lu\n\n",
           (long)adc_peak_l, (long)ssb_tx_get_mic_peak(), (long)ssb_tx_get_dac_peak(),
           (unsigned long)rx_clip_count, (unsigned long)ssb_tx_get_clip_count());
}

static void print_status_extra(void) {
    printf("VFO: active=%c A=%lu Hz B=%lu Hz step=%u (%lu Hz) RIT=%d Hz\n",
           ui_active_vfo ? 'B':'A', (unsigned long)ui_vfo_hz[0],
           (unsigned long)ui_vfo_hz[1], (unsigned)ui_step_index,
           (unsigned long)ui_step_hz[ui_step_index], (int)radio_get_rit_offset_hz());
    printf("AM: RX bandwidth=%u Hz carrier=%u%% | DSP NR=%s | spectrum=%d dB\n",
           (unsigned)am_rx_bw_hz, (unsigned)fft_ssb_tx_get_am_carrier(),
           fft_ssb_rx_get_noise_reduction()?"ON":"OFF", (int)spectrum_get_offset_db());
    printf("CW: offset=%u Hz decode=%u Hz sidetone=%u%% speed=%u WPM delay=%u ms keyer=%s\n",
           (unsigned)cw_offset_hz, (unsigned)cw_decode_pitch_hz,
           (unsigned)cw_sidetone_level, (unsigned)cw_keyer_wpm,
           (unsigned)cw_delay_ms, cw_keyer_mode==CW_KEY_STRAIGHT?"STRAIGHT":
           cw_keyer_mode==CW_KEY_IAMBIC_A?"IAMBIC-A":"IAMBIC-B");
    printf("RF: BFO=%lu Hz delta=%ld Hz IF=%lu Hz CLK1=%u mA CLK2=%u mA\n",
           (unsigned long)radio_get_bfo_hz(), (long)ui_bfo_delta_hz,
           (unsigned long)radio_get_if_hz(), (unsigned)radio_get_clk_drive(1),
           (unsigned)radio_get_clk_drive(2));
    printf("AGC: target=%u%% max=%ux | UTC valid=%d unix_ms=%lld | SD dirty=%d\n\n",
           (unsigned)rx_agc_get_target_percent(), (unsigned)rx_agc_get_max_gain(),
           ft8_utc_valid?1:0, (long long)ft8_utc_now_ms(), settings_dirty?1:0);
}

static void mark_settings_dirty(void) {
    settings_dirty=true;
    settings_save_deadline=make_timeout_time_ms(5000);
}

static void clamp_rx_pair(uint16_t *low, uint16_t *high, bool changed_low) {
    /* CW needs genuinely narrow windows such as 350-650 Hz. */
    if (*low < 50u) *low = 50u;
    if (*low > 2000u) *low = 2000u;
    if (*high < 150u) *high = 150u;
    if (*high > 4000u) *high = 4000u;
    if ((int32_t)*high - (int32_t)*low < 100) {
        if (changed_low) {
            uint32_t h = (uint32_t)*low + 100u;
            if (h > 4000u) { h = 4000u; *low = (uint16_t)(h - 100u); }
            *high = (uint16_t)h;
        } else {
            int32_t l = (int32_t)*high - 100;
            if (l < 50) { l = 50; *high = 150u; }
            *low = (uint16_t)l;
        }
    }
}

static void clamp_pair(uint16_t *low, uint16_t *high, bool changed_low) {
    if (*low<50u) *low=50u; if (*low>1000u) *low=1000u;
    if (*high<1200u) *high=1200u; if (*high>4000u) *high=4000u;
    if (*high-*low<500u) {
        if (changed_low) {
            uint16_t h=*low+500u; if (h>4000u) { h=4000u; *low=h-500u; } *high=h;
        } else {
            int32_t l=(int32_t)*high-500; if (l<50) { l=50; *high=(uint16_t)l+500u; } *low=(uint16_t)l;
        }
    }
}

#define UI_BAND_COUNT RADIOCORE_BAND_COUNT
static const uint32_t ui_band_low[UI_BAND_COUNT]  = {3500000u, 5330000u, 7000000u, 10100000u, 14000000u, 18068000u, 21000000u, 24890000u, 28000000u};
static const uint32_t ui_band_high[UI_BAND_COUNT] = {4000000u, 5405000u, 7300000u, 10150000u, 14350000u, 18168000u, 21450000u, 24990000u, 29700000u};
static const uint32_t ui_band_default_frequency[UI_BAND_COUNT] = {3900000u, 5357000u, 7200000u, 10120000u, 14200000u, 18100000u, 21200000u, 24930000u, 28400000u};
static uint32_t ui_band_last[UI_BAND_COUNT] = {3900000u, 5357000u, 7200000u, 10120000u, 14200000u, 18100000u, 21200000u, 24930000u, 28400000u};
static int16_t ui_band_spectrum_offset[UI_BAND_COUNT] = {0};


static uint8_t ui_band_for_frequency(uint32_t hz);
static void ui_store_current_band_state(void);
static void ui_restore_band_state(uint8_t band);

static void collect_settings(radiocore_settings_t *c) {
    memset(c,0,sizeof(*c)); c->frequency_hz=radio_get_dial_hz(); c->sideband=(uint8_t)sideband;
    c->rx_low_hz=rx_low_hz; c->rx_high_hz=rx_high_hz; c->tx_low_hz=ssb_tx_get_low_hz(); c->tx_high_hz=ssb_tx_get_high_hz();
    c->rx_codec_in=wm8731_get_rx_input_percent(); c->mic_codec_in=wm8731_get_mic_input_percent();
    c->ear_codec_out=wm8731_get_ear_output_percent(); c->mod_codec_out=wm8731_get_mod_output_percent();
    c->if_gain=rx_if_gain; c->volume=rx_volume_percent; c->mic_gain=ssb_tx_get_mic_gain(); c->tx_drive=ssb_tx_get_drive();
    c->step_index=ui_step_index; c->agc_enabled=rx_agc_get_enabled()?1u:0u;
    c->agc_speed=rx_agc_get_speed()==RX_AGC_FAST?0u:1u;
    c->agc_target=rx_agc_get_target_percent(); c->agc_max=rx_agc_get_max_gain();
    c->tx_if_hz=ssb_tx_get_if_hz(); c->bfo_hz=radio_get_bfo_hz(); c->global_if_hz=radio_get_if_hz(); c->clk1_drive_ma=radio_get_clk_drive(1); c->clk2_drive_ma=radio_get_clk_drive(2); c->operating_mode=(uint8_t)operating_mode; c->cw_offset_hz=cw_offset_hz; c->sidetone_level=cw_sidetone_level; c->cw_delay_ms=cw_delay_ms; c->cw_wpm=cw_keyer_wpm;
    c->vfo_frequency_hz[0]=ui_vfo_hz[0];
    c->vfo_frequency_hz[1]=ui_vfo_hz[1];
    c->vfo_frequency_hz[ui_active_vfo]=radio_get_dial_hz();
    c->active_vfo=ui_active_vfo;
    c->dsp_nr_enabled=fft_ssb_rx_get_noise_reduction()?1u:0u;
    c->am_rx_bw_hz=am_rx_bw_hz;
    c->am_carrier_percent=fft_ssb_tx_get_am_carrier();
    c->cw_keyer_mode=cw_keyer_mode;
    c->cw_decode_pitch_hz=cw_decode_pitch_hz;
    c->bfo_delta_hz=ui_bfo_delta_hz;
    c->ft8_tx_first=ft8_tx6_first?1u:0u;
    c->ft8_auto=ft8_panel_auto?1u:0u;
    c->ft8_repeat=(uint8_t)ft8_panel_repeat;
    c->ft8_tx_pitch_hz=(uint16_t)ft8_tx_pitch;
    snprintf(c->station_callsign,sizeof(c->station_callsign),"%s",ft8_mycall);
    snprintf(c->station_grid,sizeof(c->station_grid),"%s",ft8_mygrid);
    c->reserved[0]=0xC7u;
    memcpy(&c->reserved[1],txcal_committed,9);
    /* Unimplemented panel-only controls remain reserved, not falsely persisted. */
    ui_store_current_band_state();
    for (uint8_t i=0;i<UI_BAND_COUNT;++i) {
        c->band_frequency_hz[i]=ui_band_last[i];
        c->band_spectrum_offset_db[i]=ui_band_spectrum_offset[i];
    }
}

static void __attribute__((noinline)) ft8_tx6_service(void) {
    if (ft8_tx6_on && !ft8_tx6_active()) {
        ft8_tx6_on = false;
        radio_io_set_manual_tx(false);
        radio_io_clear_manual_tx();
        printf("FT8 RX (TX ended)\n");
        if (ft8_repeat_left && !ft8_tx6_queued && operating_mode==OPERATING_FT8 && !protocol1_mode) {
            if (ft8_tx6_prepare(ft8_repeat_message,(unsigned)ft8_tx_pitch)) {
                ft8_tx6_queued=true; --ft8_repeat_left;
                printf("FT8 REPEAT QUEUED: %s (remaining %u)\n",ft8_repeat_message,ft8_repeat_left);
                ft8_panel_status("QUEUED ",ft8_repeat_message);
            } else { ft8_repeat_left=0; printf("FT8 repeat encode failed\n"); }
        }
    }
    if (!ft8_tx6_queued || !ft8_utc_valid || protocol1_mode || operating_mode != OPERATING_FT8) return;
    int64_t now = ft8_utc_now_ms();
    int64_t slot = now / 15000;
    if ((slot & 1) != (ft8_tx6_first ? 0 : 1) || slot == ft8_tx6_slot) return;
    if (now - slot*15000 > 80 || ft8_tx6_on) return;
    ft8_tx6_slot = slot;
    ft8_tx6_queued = false;
    ft8_slot_capture_active = false;
    ft8_tx6_start();
    ft8_tx6_on = true;
    radio_io_set_manual_tx(true);
    printf("FT8 TX: %s slot=%lld\n",ft8_tx_message,(long long)slot);
    ft8_panel_status("TX ",ft8_tx_message);
}


static void __attribute__((noinline)) apply_console_command(const char *line) {
    unsigned value = 0;
    if (!strncmp(line, "TIME ", 5)) {
        char *end = NULL;
        double seconds = strtod(line + 5, &end);
        if (end != line + 5 && seconds > 1000000000.0 && seconds < 4102444800.0) {
            ft8_utc_set_ms((int64_t)(seconds * 1000.0 + 0.5));
            ft8_slot_last = -1;
            ft8_slot_capture_active = false;
            ft8_n5_wr = 0;
            ft8_n5_ready = false;
            /* Align DS3231 write with the next UTC second tick. */
            rtc_write_second=(ft8_utc_now_ms()/1000)+1;
            rtc_write_pending=true;
            printf("TIME set unix_ms=%lld; RTC update queued\n", (long long)ft8_utc_now_ms());
        } else printf("ERR TIME: expected TIME <unix_seconds.ms>\n");
    } else if (!strncmp(line, "ft8tx ", 6)) {
        if (operating_mode != OPERATING_FT8 || !ft8_utc_valid || protocol1_mode)
            printf("FT8 TX requires local FT8 mode and valid UTC\n");
        else if (ft8_tx6_prepare(line+6, 1500u)) {
            ft8_tx6_queued = true;
            snprintf(ft8_tx_message,sizeof(ft8_tx_message),"%s",line+6);
            printf("FT8 TX queued: %s (1500 Hz, %s slots)\n",line+6,ft8_tx6_first?"0/30":"15/45");
        } else printf("FT8 TX: message cannot be encoded\n");
    } else if (!strcmp(line,"ft8abort")) {
        ft8_tx6_queued = false; ft8_tx6_on = false; ft8_repeat_left=0;ft8_qso_active=false;ft8_target[0]=0;ft8_tx6_abort();
        radio_io_set_manual_tx(false); radio_io_clear_manual_tx();
        printf("FT8 TX aborted\n");
    } else if (!strcmp(line,"ft8first on") || !strcmp(line,"ft8first off")) {
        ft8_tx6_first = !strcmp(line,"ft8first on");
        printf("FT8 TX1ST=%s\n",ft8_tx6_first?"ON":"OFF");
    } else if (!strcmp(line, "RTC?")) {
        int64_t sec=0;bool osf=true;
        if(rtc_ds3231_read(&sec,&osf))printf("RTC UTC=%lld OSF=%d sync_pending=%d\n",(long long)sec,osf?1:0,rtc_write_pending?1:0);
        else printf("RTC read failed\n");
    } else if (!strcmp(line, "TIME?")) {
        printf("TIME valid=%d unix_ms=%lld\n", ft8_utc_valid ? 1 : 0, (long long)ft8_utc_now_ms());
    } else if (!strcmp(line, "help") || !strcmp(line, "?")) print_help();
    else if (!strcmp(line, "status") || !strcmp(line, "s")) { print_status(); print_status_extra(); }
    else if (!strcmp(line, "mode usb") || !strcmp(line, "usb")) { set_operating_mode(OPERATING_USB); audio_mode=AUDIO_MODE_SSB; mark_settings_dirty(); printf("Mode=USB\n");
    } else if (!strcmp(line, "mode lsb") || !strcmp(line, "lsb")) { set_operating_mode(OPERATING_LSB); audio_mode=AUDIO_MODE_SSB; mark_settings_dirty(); printf("Mode=LSB\n");
    } else if (!strcmp(line, "mode cwu") || !strcmp(line, "cwu")) { set_operating_mode(OPERATING_CWU); audio_mode=AUDIO_MODE_SSB; mark_settings_dirty(); printf("Mode=CWU\n");
    } else if (!strcmp(line, "mode cwl") || !strcmp(line, "cwl")) { set_operating_mode(OPERATING_CWL); audio_mode=AUDIO_MODE_SSB; mark_settings_dirty(); printf("Mode=CWL\n");
    } else if (!strcmp(line, "mode am") || !strcmp(line, "am")) { set_operating_mode(OPERATING_AM); audio_mode=AUDIO_MODE_SSB; mark_settings_dirty(); printf("Mode=AM\n");
    } else if (sscanf(line, "cwoffset %u", &value)==1) { if(value<400)value=400;if(value>1200)value=1200;cw_offset_hz=(uint16_t)value;update_rx_mixer_frequency();mark_settings_dirty();printf("CW offset=%u Hz\n",cw_offset_hz);
    } else if (sscanf(line, "cwdecodepitch %u", &value)==1) { if(value<200)value=200;if(value>1000)value=1000;cw_decode_pitch_hz=(uint16_t)value;cw_decoder_set_tone(cw_decode_pitch_hz);cw_decoder_reset();printf("CW decode pitch=%u Hz\n",cw_decode_pitch_hz);
    } else if (sscanf(line, "sidetone %u", &value)==1) { if(value>100)value=100;cw_sidetone_level=(uint8_t)value;mark_settings_dirty();printf("Sidetone=%u%%\n",cw_sidetone_level);
    } else if (sscanf(line, "cwdelay %u", &value)==1) { if(value>2000)value=2000;cw_delay_ms=(uint16_t)value;mark_settings_dirty();printf("CW delay=%u ms\n",cw_delay_ms);
    } else if (sscanf(line, "cwwpm %u", &value)==1) { if(value<5)value=5;if(value>60)value=60;cw_keyer_wpm=(uint8_t)value;mark_settings_dirty();printf("CW speed=%u WPM\n",cw_keyer_wpm);
    } else if (sscanf(line, "rxlow %u", &value) == 1) {
        rx_low_hz=(uint16_t)value; clamp_rx_pair(&rx_low_hz,&rx_high_hz,true); rx_filter_config(); mark_settings_dirty(); printf("RX filter=%u-%u Hz\n",rx_low_hz,rx_high_hz);
    } else if (sscanf(line, "rxhigh %u", &value) == 1) {
        rx_high_hz=(uint16_t)value; clamp_rx_pair(&rx_low_hz,&rx_high_hz,false); rx_filter_config(); mark_settings_dirty(); printf("RX filter=%u-%u Hz\n",rx_low_hz,rx_high_hz);
    } else if (sscanf(line, "txlow %u", &value) == 1) {
        uint16_t l=(uint16_t)value,h=ssb_tx_get_high_hz(); clamp_pair(&l,&h,true); ssb_tx_set_filter(l,h); mark_settings_dirty(); printf("TX filter=%u-%u Hz\n",l,h);
    } else if (sscanf(line, "txhigh %u", &value) == 1) {
        uint16_t l=ssb_tx_get_low_hz(),h=(uint16_t)value; clamp_pair(&l,&h,false); ssb_tx_set_filter(l,h); mark_settings_dirty(); printf("TX filter=%u-%u Hz\n",l,h);
    } else if (sscanf(line, "freq %u", &value) == 1) {
        uint32_t f=(value < 1000000u ? value * 1000u : value);
        if (receiver_tune_hz(f)) {
            ui_band_last[ui_band_for_frequency(f)]=f;
            mark_settings_dirty();
        }
    } else if (sscanf(line, "rxcodecin %u", &value) == 1 || sscanf(line, "codecin %u", &value) == 1) {
        if (value <= 100u && wm8731_set_rx_input_percent((uint8_t)value)) { mark_settings_dirty(); printf("rxcodecin=%u%%\n", value); }
        else printf("Use: rxcodecin 0..100\n");
    } else if (sscanf(line, "miccodecin %u", &value) == 1) {
        if (value <= 100u && wm8731_set_mic_input_percent((uint8_t)value)) { mark_settings_dirty(); printf("miccodecin=%u%%\n", value); }
        else printf("Use: miccodecin 0..100\n");
    } else if (sscanf(line, "earcodecout %u", &value) == 1 || sscanf(line, "codecout %u", &value) == 1) {
        if (value <= 100u && wm8731_set_ear_output_percent((uint8_t)value)) { mark_settings_dirty(); printf("earcodecout=%u%%\n", value); }
        else printf("Use: earcodecout 0..100\n");
    } else if (sscanf(line, "modcodecout %u", &value) == 1) {
        if (value <= 100u && wm8731_set_mod_output_percent((uint8_t)value)) { mark_settings_dirty(); printf("modcodecout=%u%%\n", value); }
        else printf("Use: modcodecout 0..100\n");
    } else if (sscanf(line, "ifgain %u", &value) == 1) {
        if (value >= 1u && value <= 64u) { rx_if_gain = (uint8_t)value; mark_settings_dirty(); printf("ifgain=%ux\n", value); }
        else printf("Use: ifgain 1..64\n");
    } else if (sscanf(line, "volume %u", &value) == 1) {
        if (value <= 800u) { rx_volume_percent = (uint16_t)value; mark_settings_dirty(); printf("volume=%u%%\n", value); }
        else printf("Use: volume 0..800\n");
    } else if (!strcmp(line, "agc on")) { rx_agc_set_enabled(true); mark_settings_dirty(); printf("AGC=ON\n"); }
    else if (!strcmp(line, "agc off")) { rx_agc_set_enabled(false); mark_settings_dirty(); printf("AGC=OFF\n"); }
    else if (!strcmp(line, "agc fast")) { rx_agc_set_speed(RX_AGC_FAST); mark_settings_dirty(); printf("AGC=FAST\n"); }
    else if (!strcmp(line, "agc slow")) { rx_agc_set_speed(RX_AGC_SLOW); mark_settings_dirty(); printf("AGC=SLOW\n"); }
    else if (sscanf(line, "agctarget %u", &value) == 1) {
        if (rx_agc_set_target_percent((uint8_t)value)) { mark_settings_dirty(); printf("agctarget=%u%%\n", value); } else printf("Use: agctarget 5..90\n");
    } else if (sscanf(line, "agcmax %u", &value) == 1) {
        if (rx_agc_set_max_gain((uint8_t)value)) { mark_settings_dirty(); printf("agcmax=%ux\n", value); } else printf("Use: agcmax 1..64\n");
    } else if (sscanf(line, "micgain %u", &value) == 1) {
        if (ssb_tx_set_mic_gain((uint16_t)value)) { mark_settings_dirty(); printf("micgain=%u%%\n", value); } else printf("Use: micgain 0..1600\n");
    } else if (!strcmp(line, "txzero on")) {
        ssb_tx_zero_mic = true;
        ssb_tx_reset();
        printf("TX zero mic=ON\n");
    } else if (!strcmp(line, "txzero off")) {
        ssb_tx_zero_mic = false;
        ssb_tx_reset();
        printf("TX zero mic=OFF\n");
    } else if (sscanf(line, "txdrive %u", &value) == 1) {
        if (ssb_tx_set_drive((uint8_t)value)) { mark_settings_dirty(); printf("txdrive=%u%%\n", value); } else printf("Use: txdrive 0..100\n");
    } else if (sscanf(line, "txif %u", &value) == 1) {
        if (ssb_tx_set_if_hz(value)) { mark_settings_dirty(); printf("txif=%u Hz\n", value); } else printf("Use: txif 1000..47000\n");
    } else if (sscanf(line, "clk1drive %u", &value) == 1) {
        if (radio_set_clk_drive(1, (uint8_t)value)) { mark_settings_dirty(); printf("CLK1=%u mA\n", value); } else printf("Use: clk1drive 2|4|6|8\n");
    } else if (sscanf(line, "clk2drive %u", &value) == 1) {
        if (radio_set_clk_drive(2, (uint8_t)value)) { mark_settings_dirty(); printf("CLK2=%u mA\n", value); } else printf("Use: clk2drive 2|4|6|8\n");
    } else if (sscanf(line, "bfo %u", &value) == 1) {
        if (radio_set_bfo_hz(value)) { ui_bfo_base_hz=value; ui_bfo_delta_hz=0; mark_settings_dirty(); printf("BFO=%u Hz\n", value); } else printf("Use: bfo 39000000..41000000\n");
    } else if (sscanf(line, "globalif %u", &value) == 1) {
        if (radio_set_if_hz(value)) { ssb_tx_set_if_hz(value); update_rx_mixer_frequency(); mark_settings_dirty(); printf("Global IF=%u Hz\n", value); } else printf("Use: globalif 1000..47000\n");
    } else if (!strcmp(line,"TXCAL SHOW") || !strcmp(line,"txcal show")) {
        txcal_print();
    } else if (!strcmp(line,"TXCAL RESET") || !strcmp(line,"txcal reset")) {
        for(uint8_t i=0;i<9;i++) txcal_runtime[i]=10u;
        printf("TXCAL: runtime reset to 100%%; TXCAL SAVE to persist\n");
    } else if (!strcmp(line,"TXCAL SAVE") || !strcmp(line,"txcal save")) {
        uint8_t previous[9];
        memcpy(previous,txcal_committed,9);
        memcpy(txcal_committed,txcal_runtime,9);
        radiocore_settings_t c; collect_settings(&c);
        if(settings_save(&c)) { settings_dirty=false; printf("TXCAL: saved to SD\n"); }
        else { memcpy(txcal_committed,previous,9); printf("TXCAL: SD save FAILED; calibration remains temporary\n"); }
    } else if (!strncmp(line,"TXCAL ",6) || !strncmp(line,"txcal ",6)) {
        unsigned band=0,pct=0;char extra=0;
        if(sscanf(line+6,"%u %u %c",&band,&pct,&extra)==2 && pct<=100u) {
            uint8_t i=0;
            for(;i<9;i++) if(txcal_band_m[i]==band) break;
            if(i<9) { txcal_runtime[i]=(uint8_t)pct;
                printf("TXCAL: %um maximum amplitude=%u%% (not saved)\n",band,pct); }
            else printf("TXCAL: unknown band; use 80,60,40,30,20,17,15,12,10\n");
        } else printf("Use: TXCAL <band> <0..100>\n");
    } else if (!strcmp(line, "save")) {
        radiocore_settings_t c; collect_settings(&c);
        if (settings_save(&c)) { settings_dirty=false; printf("Settings saved to SD.\n"); }
        else printf("Settings save FAILED.\n");
    } else if (!strcmp(line, "bootsel") || !strcmp(line, "program")) {
        printf("Entering USB program mode...\n"); sleep_ms(100); reset_usb_boot(0,0);
    } else if (!strcmp(line, "tx on")) { radio_io_set_manual_tx(true); printf("Manual TX=ON\n"); }
    else if (!strcmp(line, "tx off")) { radio_io_set_manual_tx(false); printf("Manual TX=OFF\n"); }
    else if (!strcmp(line, "tx auto")) { radio_io_clear_manual_tx(); printf("TX=PTT AUTO\n"); }
    else if (!strcmp(line, "lpf auto")) { radio_io_set_lpf_auto(); printf("LPF=AUTO\n"); }
    else if (!strcmp(line, "lpf a")) { radio_io_set_lpf_manual(RADIO_LPF_A); printf("LPF=A\n"); }
    else if (!strcmp(line, "lpf b")) { radio_io_set_lpf_manual(RADIO_LPF_B); printf("LPF=B\n"); }
    else if (!strcmp(line, "lpf c")) { radio_io_set_lpf_manual(RADIO_LPF_C); printf("LPF=C\n"); }
    else if (!strcmp(line, "lpf d")) { radio_io_set_lpf_manual(RADIO_LPF_D); printf("LPF=D\n"); }
    else if (!strcmp(line, "lpf off")) { radio_io_set_lpf_manual(RADIO_LPF_OFF); printf("LPF=OFF\n"); }
    else if (!strcmp(line,"dsp on") || !strcmp(line,"dsp off")) {
        bool on=!strcmp(line,"dsp on"); fft_ssb_rx_set_noise_reduction(on);
        mark_settings_dirty(); printf("DSP NR=%s\n",on?"ON":"OFF");
    } else if (sscanf(line,"amrx %u",&value)==1) {
        if(value>=2000 && value<=10000){am_rx_bw_hz=(uint16_t)value;mark_settings_dirty();printf("AM RX=%u Hz\n",value);}
        else printf("Use: amrx 2000..10000\n");
    } else if (sscanf(line,"amcarrier %u",&value)==1) {
        if(value<=100){fft_ssb_tx_set_am_carrier((uint8_t)value);mark_settings_dirty();printf("AM carrier=%u%%\n",value);}
        else printf("Use: amcarrier 0..100\n");
    } else if (!strcmp(line,"keyer straight") || !strcmp(line,"keyer a") || !strcmp(line,"keyer b")) {
        cw_keyer_mode=!strcmp(line,"keyer straight")?CW_KEY_STRAIGHT:!strcmp(line,"keyer a")?CW_KEY_IAMBIC_A:CW_KEY_IAMBIC_B;
        cw_keyer_reset();mark_settings_dirty();printf("Keyer=%u\n",(unsigned)cw_keyer_mode);
    } else if (sscanf(line,"step %u",&value)==1) {
        if(value<8){ui_step_index=(uint8_t)value;mark_settings_dirty();printf("Step=%lu Hz\n",(unsigned long)ui_step_hz[value]);}
        else printf("Use: step 0..7\n");
    } else if (!strcmp(line,"vfo a") || !strcmp(line,"vfo b")) {
        uint8_t target=!strcmp(line,"vfo b")?1u:0u;
        ui_vfo_hz[ui_active_vfo]=radio_get_dial_hz();
        ui_active_vfo=target;receiver_tune_hz(ui_vfo_hz[target]);mark_settings_dirty();
        printf("VFO=%c\n",target?'B':'A');
    } else if (sscanf(line,"vfoa %u",&value)==1 || sscanf(line,"vfob %u",&value)==1) {
        uint8_t target=line[3]=='a'?0u:1u;
        if(value>=100000u && value<=60000000u){ui_vfo_hz[target]=value;
            if(ui_active_vfo==target)receiver_tune_hz(value);
            mark_settings_dirty();printf("VFO %c=%lu Hz\n",target?'B':'A',(unsigned long)value);
        } else printf("Use: vfoa/vfob 100000..60000000 (Hz)\n");
    } else if (sscanf(line,"band %u",&value)==1) {
        if(value<UI_BAND_COUNT){ui_store_current_band_state();ui_restore_band_state((uint8_t)value);
            ui_vfo_hz[ui_active_vfo]=radio_get_dial_hz();mark_settings_dirty();printf("Band=%u\n",value);
        } else printf("Use: band 0..8\n");
    } else {
        int signed_value=0;
        if(sscanf(line,"rit %d",&signed_value)==1){
            if(signed_value>=-9999 && signed_value<=9999){radio_set_rit_offset_hz((int16_t)signed_value);mark_settings_dirty();printf("RIT=%d Hz\n",signed_value);}
            else printf("Use: rit -9999..9999\n");
        } else if(sscanf(line,"spectrum %d",&signed_value)==1){
            if(signed_value>=-20 && signed_value<=20){spectrum_set_offset_db((int16_t)signed_value);
                ui_band_spectrum_offset[ui_band_for_frequency(radio_get_dial_hz())]=(int16_t)signed_value;
                mark_settings_dirty();printf("Spectrum=%d dB\n",signed_value);}
            else printf("Use: spectrum -20..20\n");
        } else if(sscanf(line,"bfodelta %d",&signed_value)==1){
            int64_t f=(int64_t)ui_bfo_base_hz+signed_value;
            if(signed_value>=-10000 && signed_value<=10000 && f>=39000000 && f<=41000000 && radio_set_bfo_hz((uint32_t)f)){
                ui_bfo_delta_hz=signed_value;mark_settings_dirty();printf("BFO delta=%d Hz\n",signed_value);}
            else printf("Use: bfodelta -10000..10000 (BFO must remain in range)\n");
        } else if(*line)printf("Unknown command. Type help.\n");
    }

}


static uint8_t ui_band_for_frequency(uint32_t hz) {
    for (uint8_t i=0; i<UI_BAND_COUNT; ++i) if (hz>=ui_band_low[i] && hz<=ui_band_high[i]) return i;
    uint8_t best=0; uint32_t best_diff=0xffffffffu;
    for (uint8_t i=0; i<UI_BAND_COUNT; ++i) { uint32_t mid=ui_band_low[i]+(ui_band_high[i]-ui_band_low[i])/2u; uint32_t d=hz>mid?hz-mid:mid-hz; if (d<best_diff) { best_diff=d; best=i; } }
    return best;
}

static void ui_store_current_band_state(void) {
    uint8_t band=ui_band_for_frequency(radio_get_dial_hz());
    ui_band_last[band]=radio_get_dial_hz();
    ui_band_spectrum_offset[band]=spectrum_get_offset_db();
}

static void ui_restore_band_state(uint8_t band) {
    if (band>=UI_BAND_COUNT) return;
    receiver_tune_hz(ui_band_last[band]);
    spectrum_set_offset_db(ui_band_spectrum_offset[band]);
}

static uint8_t ui_agc_mode(void) { if (!rx_agc_get_enabled()) return 0; return rx_agc_get_speed()==RX_AGC_FAST ? 1 : 2; }

static ui_snapshot_t ui_snapshot(void) {
    ui_snapshot_t s = {0};
    s.frequency_hz=radio_get_dial_hz(); s.operating_mode=(uint8_t)operating_mode; s.tx=radio_io_is_tx();
    s.volume=rx_volume_percent; s.if_gain=rx_if_gain; s.rx_low=rx_low_hz; s.rx_high=rx_high_hz; s.am_rx_bw_hz=am_rx_bw_hz; s.am_carrier_percent=fft_ssb_tx_get_am_carrier();
    s.tx_low=ssb_tx_get_low_hz(); s.tx_high=ssb_tx_get_high_hz(); s.tx_drive=ssb_tx_get_drive();
    s.mic_gain=ssb_tx_get_mic_gain(); s.tx_if_hz=ssb_tx_get_if_hz(); s.signal_envelope=rx_agc_get_envelope();
    s.band_index=ui_band_for_frequency(s.frequency_hz); s.agc_mode=ui_agc_mode();
    s.agc_target=rx_agc_get_target_percent(); s.agc_max=rx_agc_get_max_gain();
    s.rx_codec_in=wm8731_get_rx_input_percent(); s.mic_codec_in=wm8731_get_mic_input_percent();
    s.ear_codec_out=wm8731_get_ear_output_percent(); s.mod_codec_out=wm8731_get_mod_output_percent(); s.step_index=ui_step_index;
    s.rit_offset_hz=radio_get_rit_offset_hz();
    s.cw_offset_hz=cw_offset_hz; s.sidetone_level=cw_sidetone_level; s.cw_delay_ms=cw_delay_ms; s.cw_wpm=cw_keyer_wpm; s.keyer_mode=cw_keyer_mode; s.cw_decode_pitch_hz=cw_decode_pitch_hz; s.bfo_hz=radio_get_bfo_hz(); s.global_if_hz=radio_get_if_hz();
    s.bfo_delta_hz=ui_bfo_delta_hz; s.active_vfo=ui_active_vfo; s.vfo_a_hz=ui_vfo_hz[0]; s.vfo_b_hz=ui_vfo_hz[1]; s.dsp_nr_enabled=fft_ssb_rx_get_noise_reduction(); s.clk1_drive_ma=radio_get_clk_drive(1); s.clk2_drive_ma=radio_get_clk_drive(2);
    s.spectrum_offset_db = spectrum_get_offset_db();
    spectrum_get_bins(s.spectrum);
    spectrum_get_wide_bins(s.spectrum_wide);
    s.cw_decode_active = operating_mode_is_cw();
    snprintf(s.cw_decode_text, sizeof(s.cw_decode_text), "%s", cw_decoder_text());
    s.cw_decode_wpm = cw_decoder_wpm_estimate();
    s.cw_decode_tone_error_hz = cw_decoder_tone_error_hz();
    s.protocol1_mode = protocol1_mode;
    const p1_network_status_t *pn = p1_network_status();
    s.p1_net_initialized = pn->initialized;
    s.p1_link_up = pn->link_up;
    s.p1_discovered = pn->discovered;
    s.p1_control_seen = pn->control_seen;
    s.p1_rx_streaming = pn->rx_streaming;
    s.p1_mox_requested = pn->mox_requested;
    s.p1_rx_frequency_hz = pn->rx_frequency_hz;
    return s;
}

static void ui_factory_defaults(void) {
    for (uint8_t i=0;i<UI_BAND_COUNT;++i) {
        ui_band_last[i]=ui_band_default_frequency[i];
        ui_band_spectrum_offset[i]=0;
    }
    receiver_tune_hz(7100000u); spectrum_set_offset_db(0);
    operating_mode=OPERATING_LSB; sideband=SIDEBAND_LSB; ui_step_index=2; cw_offset_hz=700u; cw_sidetone_level=40u; cw_delay_ms=300u; cw_keyer_wpm=20u; cw_keyer_mode=CW_KEY_IAMBIC_B; cw_decoder_set_wpm(cw_keyer_wpm);
    rx_low_hz=300u; rx_high_hz=2500u; rx_filter_config();
    ssb_tx_set_filter(400u,2800u); rx_if_gain=8u; rx_volume_percent=100u;
    wm8731_set_rx_input_percent(70u); wm8731_set_mic_input_percent(30u); wm8731_set_ear_output_percent(100u); wm8731_set_mod_output_percent(100u);
    ssb_tx_set_mic_gain(50u); ssb_tx_set_drive(50u); ssb_tx_set_if_hz(radio_get_if_hz());
    rx_agc_set_enabled(true); rx_agc_set_speed(RX_AGC_FAST); rx_agc_set_target_percent(8u); rx_agc_set_max_gain(20u);
    update_rx_mixer_frequency(); mark_settings_dirty();
}

static void ui_apply_event(ui_event_t e) {
    if(e.type==UI_EVENT_FT8_COMMAND){ft8_panel_command(e.ft8_command);return;}
    if(e.type==UI_EVENT_REMOTE_PTT){
        /* Stock-panel TX/RX is LOCAL microphone PTT, not CI-V/USB-audio PTT.
           Keep civ_remote_ptt reserved for the proven CAT audio path. */
        panel_remote_ptt=(e.delta!=0);
        if(panel_remote_ptt && operating_mode != OPERATING_FT8) radio_io_set_manual_tx(true);
        else if(operating_mode != OPERATING_FT8) { radio_io_set_manual_tx(false); radio_io_clear_manual_tx(); }
        return;
    }
    if(e.type==UI_EVENT_SET_CW_PITCH){
        uint16_t hz=(uint16_t)e.delta;
        cw_offset_hz=hz;
        cw_decode_pitch_hz=hz>1000u?1000u:hz;
        cw_decoder_set_tone(cw_decode_pitch_hz); cw_decoder_reset(); update_rx_mixer_frequency(); mark_settings_dirty();
        return;
    }
    if(e.type==UI_EVENT_SET_BW){
        uint16_t bw=(uint16_t)e.delta;
        if(operating_mode==OPERATING_AM){
            uint32_t full=(uint32_t)bw*2u; if(full<2000u)full=2000u;if(full>10000u)full=10000u;
            am_rx_bw_hz=(uint16_t)full;
            uint16_t l=50u,h=bw; if(h<1200u)h=1200u;if(h>4000u)h=4000u; ssb_tx_set_filter(l,h);
        }else{
            uint16_t l=rx_low_hz; uint32_t hh=(uint32_t)l+bw; if(hh>4000u)hh=4000u; if(hh<150u)hh=150u;
            rx_high_hz=(uint16_t)hh; clamp_rx_pair(&rx_low_hz,&rx_high_hz,false); rx_filter_config();
            uint16_t tl=rx_low_hz,th=rx_high_hz; clamp_pair(&tl,&th,false); ssb_tx_set_filter(tl,th);
        }
        mark_settings_dirty();
        return;
    }
    if (e.type==UI_EVENT_APPLY_DISPLAY_SETTINGS) {
        const panel_settings_t *p=&e.settings;
        for(uint8_t i=0;i<UI_BAND_COUNT;i++){
            uint32_t bf=p->band_frequency_hz[i];
            if(bf>=ui_band_low[i]&&bf<=ui_band_high[i])ui_band_last[i]=bf;
            int16_t so=p->band_spectrum_offset_db[i];
            if(so>=-20&&so<=20)ui_band_spectrum_offset[i]=so;
        }
        if(p->bfo_hz>=39000000u&&p->bfo_hz<=41000000u)radio_set_bfo_hz(p->bfo_hz);
        if(p->global_if_hz>=1000u&&p->global_if_hz<=47000u)radio_set_if_hz(p->global_if_hz);
        if(p->clk1_drive_ma==2u||p->clk1_drive_ma==4u||p->clk1_drive_ma==6u||p->clk1_drive_ma==8u)radio_set_clk_drive(1,p->clk1_drive_ma);
        if(p->clk2_drive_ma==2u||p->clk2_drive_ma==4u||p->clk2_drive_ma==6u||p->clk2_drive_ma==8u)radio_set_clk_drive(2,p->clk2_drive_ma);
        if(p->frequency_hz>=100000u&&p->frequency_hz<=60000000u)receiver_tune_hz(p->frequency_hz);
        if(p->operating_mode<5u)set_operating_mode((OperatingMode)p->operating_mode);
        if(p->step_index<8u)ui_step_index=p->step_index;
        if(p->volume<=800u)rx_volume_percent=p->volume;
        if(p->if_gain>=1u&&p->if_gain<=64u)rx_if_gain=p->if_gain;
        if(p->agc_mode==0u)rx_agc_set_enabled(false);else{rx_agc_set_enabled(true);rx_agc_set_speed(p->agc_mode==1u?RX_AGC_FAST:RX_AGC_SLOW);}
        if(p->agc_target>=5u&&p->agc_target<=90u)rx_agc_set_target_percent(p->agc_target);
        if(p->agc_max>=1u&&p->agc_max<=64u)rx_agc_set_max_gain(p->agc_max);
        if(p->rx_codec_in<=100u)wm8731_set_rx_input_percent(p->rx_codec_in);
        if(p->mic_codec_in<=100u)wm8731_set_mic_input_percent(p->mic_codec_in);
        if(p->ear_codec_out<=100u)wm8731_set_ear_output_percent(p->ear_codec_out);
        if(p->mod_codec_out<=100u)wm8731_set_mod_output_percent(p->mod_codec_out);
        rx_low_hz=p->rx_low;rx_high_hz=p->rx_high;clamp_rx_pair(&rx_low_hz,&rx_high_hz,true);rx_filter_config();
        if(p->am_rx_bw_hz>=2000u&&p->am_rx_bw_hz<=10000u)am_rx_bw_hz=p->am_rx_bw_hz;
        if(p->am_carrier_percent<=100u)fft_ssb_tx_set_am_carrier(p->am_carrier_percent);
        uint16_t tl=p->tx_low,th=p->tx_high;clamp_pair(&tl,&th,true);ssb_tx_set_filter(tl,th);
        if(p->tx_if_hz>=1000u&&p->tx_if_hz<=47000u)ssb_tx_set_if_hz(p->tx_if_hz);
        if(p->tx_drive<=100u)ssb_tx_set_drive(p->tx_drive);
        if(p->cw_offset_hz>=400u&&p->cw_offset_hz<=1200u)cw_offset_hz=p->cw_offset_hz;
        if(p->sidetone_level<=100u)cw_sidetone_level=p->sidetone_level;
        if(p->cw_delay_ms<=2000u)cw_delay_ms=p->cw_delay_ms;
        if(p->cw_wpm>=5u&&p->cw_wpm<=60u){cw_keyer_wpm=p->cw_wpm;cw_decoder_set_wpm(cw_keyer_wpm);}
        if(p->keyer_mode<=CW_KEY_IAMBIC_B){cw_keyer_mode=p->keyer_mode;cw_keyer_reset();}
        if(p->cw_decode_pitch_hz>=200u&&p->cw_decode_pitch_hz<=1000u){cw_decode_pitch_hz=p->cw_decode_pitch_hz;cw_decoder_set_tone(cw_decode_pitch_hz);cw_decoder_reset();}
        if(p->spectrum_offset_db>=-20&&p->spectrum_offset_db<=20){spectrum_set_offset_db(p->spectrum_offset_db);ui_band_spectrum_offset[ui_band_for_frequency(radio_get_dial_hz())]=p->spectrum_offset_db;}
        update_rx_mixer_frequency();
        return;
    }
    if (e.type==UI_EVENT_TUNE) {
        int64_t f=(int64_t)radio_get_dial_hz()+(int64_t)e.delta*ui_step_hz[ui_step_index]; if(f<100000)f=100000;if(f>60000000)f=60000000;
        if(receiver_tune_hz((uint32_t)f)){
            uint8_t band=ui_band_for_frequency((uint32_t)f);
            ui_band_last[band]=(uint32_t)f;
            ui_vfo_hz[ui_active_vfo]=(uint32_t)f;
            ui_band_spectrum_offset[band]=spectrum_get_offset_db();
            /* Display Pico will persist the new dial frequency after idle. */
        } return;
    }
    if(e.type==UI_EVENT_CYCLE_STEP){ui_step_index=(uint8_t)((ui_step_index+1u)%8u);return;}
    if(e.type==UI_EVENT_SET_MODE){
        if(e.delta>=0 && e.delta<7) set_operating_mode((OperatingMode)e.delta);
        return;
    }
    if(e.type==UI_EVENT_SET_VFO){
        uint8_t want=(e.delta!=0)?1u:0u;
        if(want!=ui_active_vfo){
            ui_vfo_hz[ui_active_vfo]=radio_get_dial_hz();
            ui_active_vfo=want;
            receiver_tune_hz(ui_vfo_hz[ui_active_vfo]);
        }
        return;
    }
    if(e.type==UI_EVENT_FORCE_SAVE){radiocore_settings_t c;collect_settings(&c);if(settings_save(&c)){settings_dirty=false;printf("Settings saved.\n> ");}else printf("SD save failed.\n> ");return;}
    if(e.type==UI_EVENT_FACTORY_DEFAULTS){ui_factory_defaults();return;}
    if(e.type==UI_EVENT_PROGRAM_MODE){printf("Entering USB program mode...\n"); sleep_ms(100); reset_usb_boot(0,0); return;}
    if(e.type!=UI_EVENT_ADJUST)return;
    int32_t v;
    switch(e.item){
      case UI_ITEM_ENGINE_MODE:
          bool want_p1 = e.delta > 0 ? true : false;
          radio_io_clear_manual_tx();
          radio_io_enter_receive();
          cw_key_down = false;
          memset(output_buffer, 0, sizeof(output_buffer));
          if (want_p1 && !protocol1_mode) {
              p1_dsp_init(&p1_dsp, radio_get_if_hz());
              p1_accum_count = 0;
              p1_raw_wr = p1_raw_rd = 0;
              p1_raw_drops = 0;
              p1_tx_primed = false;
              p1_last_mox = false;
              radio_io_set_manual_tx(false);
              protocol1_mode = p1_network_start();
          } else if (!want_p1 && protocol1_mode) {
              p1_network_stop();
              radio_io_set_manual_tx(false);
              radio_io_clear_manual_tx();
              protocol1_mode = false;
              rx_dsp_reset();
              ssb_tx_reset();
          }
          return;
      case UI_ITEM_BAND:{
          uint8_t cur=ui_band_for_frequency(radio_get_dial_hz());
          ui_band_last[cur]=radio_get_dial_hz();
          ui_band_spectrum_offset[cur]=spectrum_get_offset_db();
          int next=(int)cur+e.delta;
          while(next<0)next+=UI_BAND_COUNT;
          while(next>=UI_BAND_COUNT)next-=UI_BAND_COUNT;
          ui_restore_band_state((uint8_t)next);
          break;
      }
      case UI_ITEM_STEP:v=(int32_t)ui_step_index+e.delta;while(v<0)v+=8;while(v>=8)v-=8;ui_step_index=(uint8_t)v;break;
      case UI_ITEM_RIT:
          v=(int32_t)radio_get_rit_offset_hz()+e.delta*10;
          if(v<-2000)v=-2000;if(v>2000)v=2000;
          radio_set_rit_offset_hz((int16_t)v);
          break;
      case UI_ITEM_MODE:{int m=(int)operating_mode+e.delta;while(m<0)m+=7;while(m>=7)m-=7;set_operating_mode((OperatingMode)m);break;}
      case UI_ITEM_VOLUME:v=(int32_t)rx_volume_percent+e.delta*5;if(v<0)v=0;if(v>800)v=800;rx_volume_percent=(uint16_t)v;break;
      case UI_ITEM_IF_GAIN:v=(int32_t)rx_if_gain+e.delta;if(v<1)v=1;if(v>64)v=64;rx_if_gain=(uint8_t)v;break;
      case UI_ITEM_AGC:{int m=(int)ui_agc_mode()+e.delta;while(m<0)m+=3;while(m>=3)m-=3;if(m==0)rx_agc_set_enabled(false);else{rx_agc_set_enabled(true);rx_agc_set_speed(m==1?RX_AGC_FAST:RX_AGC_SLOW);}break;}
      case UI_ITEM_AGC_TARGET:v=(int32_t)rx_agc_get_target_percent()+e.delta;if(v<5)v=5;if(v>90)v=90;rx_agc_set_target_percent((uint8_t)v);break;
      case UI_ITEM_AGC_MAX:v=(int32_t)rx_agc_get_max_gain()+e.delta;if(v<1)v=1;if(v>64)v=64;rx_agc_set_max_gain((uint8_t)v);break;
      case UI_ITEM_RX_CODEC_IN:v=(int32_t)wm8731_get_rx_input_percent()+e.delta;if(v<0)v=0;if(v>100)v=100;wm8731_set_rx_input_percent((uint8_t)v);break;
      case UI_ITEM_RX_LOW:v=(int32_t)rx_low_hz+e.delta*50;if(v<50)v=50;if(v>2000)v=2000;rx_low_hz=(uint16_t)v;clamp_rx_pair(&rx_low_hz,&rx_high_hz,true);rx_filter_config();break;
      case UI_ITEM_RX_HIGH:v=(int32_t)rx_high_hz+e.delta*50;if(v<150)v=150;if(v>4000)v=4000;rx_high_hz=(uint16_t)v;clamp_rx_pair(&rx_low_hz,&rx_high_hz,false);rx_filter_config();break;
      case UI_ITEM_AM_RX_BW:v=(int32_t)am_rx_bw_hz+e.delta*500;if(v<2000)v=2000;if(v>10000)v=10000;am_rx_bw_hz=(uint16_t)v;break;
      case UI_ITEM_SPECTRUM_OFFSET:
          v=(int32_t)spectrum_get_offset_db()+e.delta;
          if(v<-20)v=-20;if(v>20)v=20;
          spectrum_set_offset_db((int16_t)v);
          ui_band_spectrum_offset[ui_band_for_frequency(radio_get_dial_hz())]=(int16_t)v;
          break;
      case UI_ITEM_TX_DRIVE:v=(int32_t)ssb_tx_get_drive()+e.delta;if(v<0)v=0;if(v>100)v=100;ssb_tx_set_drive((uint8_t)v);break;
      case UI_ITEM_MIC_GAIN:v=(int32_t)ssb_tx_get_mic_gain()+e.delta*25;if(v<0)v=0;if(v>1600)v=1600;ssb_tx_set_mic_gain((uint16_t)v);break;
      case UI_ITEM_MIC_CODEC_IN:v=(int32_t)wm8731_get_mic_input_percent()+e.delta;if(v<0)v=0;if(v>100)v=100;wm8731_set_mic_input_percent((uint8_t)v);break;
      case UI_ITEM_MOD_CODEC_OUT:v=(int32_t)wm8731_get_mod_output_percent()+e.delta;if(v<0)v=0;if(v>100)v=100;wm8731_set_mod_output_percent((uint8_t)v);break;
      case UI_ITEM_TX_LOW:{v=(int32_t)ssb_tx_get_low_hz()+e.delta*50;if(v<50)v=50;if(v>1000)v=1000;uint16_t l=(uint16_t)v,h=ssb_tx_get_high_hz();clamp_pair(&l,&h,true);ssb_tx_set_filter(l,h);break;}
      case UI_ITEM_TX_HIGH:{v=(int32_t)ssb_tx_get_high_hz()+e.delta*50;if(v<1200)v=1200;if(v>4000)v=4000;uint16_t l=ssb_tx_get_low_hz(),h=(uint16_t)v;clamp_pair(&l,&h,false);ssb_tx_set_filter(l,h);break;}
      case UI_ITEM_AM_CARRIER:v=(int32_t)fft_ssb_tx_get_am_carrier()+e.delta*5;if(v<0)v=0;if(v>100)v=100;fft_ssb_tx_set_am_carrier((uint8_t)v);break;
      case UI_ITEM_TX_IF:v=(int32_t)ssb_tx_get_if_hz()+e.delta*100;if(v<1000)v=1000;if(v>47000)v=47000;ssb_tx_set_if_hz((uint32_t)v);break;
      case UI_ITEM_EAR_CODEC_OUT:v=(int32_t)wm8731_get_ear_output_percent()+e.delta;if(v<0)v=0;if(v>100)v=100;wm8731_set_ear_output_percent((uint8_t)v);break;
      case UI_ITEM_CW_OFFSET:v=(int32_t)cw_offset_hz+e.delta*10;if(v<400)v=400;if(v>1200)v=1200;cw_offset_hz=(uint16_t)v;update_rx_mixer_frequency();break;
      case UI_ITEM_CW_DECODE_PITCH:v=(int32_t)cw_decode_pitch_hz+e.delta*25;if(v<200)v=200;if(v>1000)v=1000;cw_decode_pitch_hz=(uint16_t)v;cw_decoder_set_tone(cw_decode_pitch_hz);cw_decoder_reset();break;
      case UI_ITEM_SIDETONE:v=(int32_t)cw_sidetone_level+e.delta*5;if(v<0)v=0;if(v>100)v=100;cw_sidetone_level=(uint8_t)v;break;
      case UI_ITEM_CW_DELAY:v=(int32_t)cw_delay_ms+e.delta*25;if(v<0)v=0;if(v>2000)v=2000;cw_delay_ms=(uint16_t)v;break;
      case UI_ITEM_CW_SPEED:v=(int32_t)cw_keyer_wpm+e.delta;if(v<5)v=5;if(v>60)v=60;cw_keyer_wpm=(uint8_t)v;cw_decoder_set_wpm(cw_keyer_wpm);break;
      case UI_ITEM_KEYER_MODE:v=(int32_t)cw_keyer_mode+e.delta;while(v<0)v+=3;while(v>=3)v-=3;cw_keyer_mode=(uint8_t)v;cw_keyer_reset();break;
      case UI_ITEM_BFO_FREQ:v=(int32_t)radio_get_bfo_hz()+e.delta*100;if(v<39000000)v=39000000;if(v>41000000)v=41000000;radio_set_bfo_hz((uint32_t)v);ui_bfo_base_hz=(uint32_t)v-ui_bfo_delta_hz;break;
      case UI_ITEM_BFO_DELTA:
          v=(int32_t)ui_bfo_delta_hz+e.delta*50;if(v<-3000)v=-3000;if(v>3000)v=3000;
          ui_bfo_delta_hz=(int16_t)v; radio_set_bfo_hz((uint32_t)((int32_t)ui_bfo_base_hz+v)); break;
      case UI_ITEM_DSP_NR: fft_ssb_rx_set_noise_reduction(e.delta>0); break;
      case UI_ITEM_GLOBAL_IF:v=(int32_t)radio_get_if_hz()+e.delta*100;if(v<1000)v=1000;if(v>47000)v=47000;if(radio_set_if_hz((uint32_t)v)){ssb_tx_set_if_hz((uint32_t)v);update_rx_mixer_frequency();}break;
      case UI_ITEM_CLK1_DRIVE:{int d=(int)radio_get_clk_drive(1)+e.delta*2;if(d<2)d=8;if(d>8)d=2;radio_set_clk_drive(1,(uint8_t)d);break;}
      case UI_ITEM_CLK2_DRIVE:{int d=(int)radio_get_clk_drive(2)+e.delta*2;if(d<2)d=8;if(d>8)d=2;radio_set_clk_drive(2,(uint8_t)d);break;}
      default:return;
    }
    /* All front-panel settings are display-owned. No automatic RadioCore flash write here. */
}

int main(void) {
    // N6 isolated A/B: increase RP2350 clock, leave FT8 capture and decoder unchanged.
    vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    set_sys_clock_khz(240000, true);
    protocol1_mode = false;
    stdio_init_all();
    printf("N6 CLOCK/O3 A-B: sysclk=%lu MHz\n", (unsigned long)(clock_get_hz(clk_sys)/1000000u));
    // v0.7d bench diagnostic: bring up existing TinyUSB device immediately.
    // P1 is intentionally not used during this SD test.
    tud_init(PICO_TUD_RHPORT);
    absolute_time_t usb_wait = make_timeout_time_ms(1500);
    while (absolute_time_diff_us(get_absolute_time(), usb_wait) > 0) { tud_task(); tight_loop_contents(); }

    i2c_init(i2c1, 400000);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);
    gpio_disable_pulls(PIN_SDA);
    gpio_disable_pulls(PIN_SCL);

    radiocore_settings_t saved={0};
    bool have_saved=settings_load(&saved);
    if(have_saved && saved.reserved[0]==0xC7u) {
        for(uint8_t i=0;i<9;i++) {
            uint8_t pct=saved.reserved[i+1];
            if(pct<=100u) txcal_runtime[i]=txcal_committed[i]=pct;
        }
    }

    /* Restore FT8 settings from the existing v7 SD record. */
    if (have_saved) {
        if (saved.station_callsign[0]) {
            memcpy(ft8_mycall,saved.station_callsign,sizeof(saved.station_callsign));
            ft8_mycall[sizeof(saved.station_callsign)]=0;
        }
        if (saved.station_grid[0]) {
            memcpy(ft8_mygrid,saved.station_grid,sizeof(saved.station_grid));
            ft8_mygrid[sizeof(ft8_mygrid)-1]=0;
        }
        if (saved.ft8_tx_pitch_hz>=300 && saved.ft8_tx_pitch_hz<=3000)
            ft8_tx_pitch=saved.ft8_tx_pitch_hz;
        if (saved.ft8_repeat>=1 && saved.ft8_repeat<=10) {
            ft8_panel_repeat=saved.ft8_repeat;
            ft8_tx6_first=saved.ft8_tx_first!=0;
            ft8_panel_auto=saved.ft8_auto!=0;
        }
    }

    for (uint8_t i=0;i<UI_BAND_COUNT;++i) {
        ui_band_last[i]=ui_band_default_frequency[i];
        ui_band_spectrum_offset[i]=0;
    }
    if (have_saved) {
        for (uint8_t i=0;i<UI_BAND_COUNT;++i) {
            uint32_t f=saved.band_frequency_hz[i];
            if (f>=ui_band_low[i] && f<=ui_band_high[i]) ui_band_last[i]=f;
            int16_t o=saved.band_spectrum_offset_db[i];
            if (o>=-20 && o<=20) ui_band_spectrum_offset[i]=o;
        }
        /* Migrated v4 settings have empty arrays. Preserve their saved dial. */
        if (saved.frequency_hz>=100000u && saved.frequency_hz<=60000000u) {
            uint8_t b=ui_band_for_frequency(saved.frequency_hz);
            if (saved.band_frequency_hz[b]==0u) ui_band_last[b]=saved.frequency_hz;
        }
    }

    uint32_t boot_freq=(have_saved && saved.frequency_hz>=100000u && saved.frequency_hz<=60000000u) ? saved.frequency_hz : 7100000u;
    uint32_t boot_bfo=(have_saved && saved.bfo_hz>=39000000u && saved.bfo_hz<=41000000u)?saved.bfo_hz:40037000u;
    uint32_t boot_if=(have_saved && saved.global_if_hz>=1000u && saved.global_if_hz<=47000u)?saved.global_if_hz:24000u;
    si5351_ok = radio_init(boot_bfo, boot_freq, boot_if);
    radio_set_clk_drive(1,(have_saved && (saved.clk1_drive_ma==2u||saved.clk1_drive_ma==4u||saved.clk1_drive_ma==6u||saved.clk1_drive_ma==8u))?saved.clk1_drive_ma:2u);
    radio_set_clk_drive(2,(have_saved && (saved.clk2_drive_ma==2u||saved.clk2_drive_ma==4u||saved.clk2_drive_ma==6u||saved.clk2_drive_ma==8u))?saved.clk2_drive_ma:6u);
    radio_io_init(radio_get_dial_hz());
    if (!si5351_ok) printf("Si5351: FAIL\n");

    uint8_t boot_rx=have_saved?saved.rx_codec_in:70u, boot_mic=have_saved?saved.mic_codec_in:30u;
    uint8_t boot_ear=have_saved?saved.ear_codec_out:100u, boot_mod=have_saved?saved.mod_codec_out:100u;
    if (!wm8731_init_master_96k_16bit(boot_rx, boot_mic, boot_ear, boot_mod)) {
        printf("WM8731: FAIL\n");
        while (true) tight_loop_contents();
    }

    for (uint i = 0; i < SINE_SIZE; ++i) {
        float a = (2.0f * (float)M_PI * (float)i) / (float)SINE_SIZE;
        sine_table[i] = (int16_t)lroundf(32767.0f * sinf(a));
    }
    rx_agc_init();
    spectrum_init();
    cw_decoder_init(24000u, cw_decode_pitch_hz, cw_keyer_wpm);
    spectrum_set_offset_db(ui_band_spectrum_offset[ui_band_for_frequency(boot_freq)]);
    ssb_tx_init(SAMPLE_RATE, radio_get_if_hz());
    fft_ssb_tx_init(SAMPLE_RATE, radio_get_if_hz());
    fft_ssb_rx_init(SAMPLE_RATE, radio_get_if_hz());
    p1_dsp_init(&p1_dsp, radio_get_if_hz());
    p1_accum_count = 0;
    p1_raw_wr = p1_raw_rd = 0;
    p1_raw_drops = 0;
    p1_tx_primed = false;
    p1_last_mox = false;
    if (have_saved) {
        operating_mode=saved.operating_mode<7u?(OperatingMode)saved.operating_mode:(saved.sideband?OPERATING_LSB:OPERATING_USB);
        sideband=(operating_mode==OPERATING_USB||operating_mode==OPERATING_CWU||operating_mode==OPERATING_2TONE||operating_mode==OPERATING_FT8)?SIDEBAND_USB:SIDEBAND_LSB;
        cw_offset_hz=(saved.cw_offset_hz>=400u&&saved.cw_offset_hz<=1200u)?saved.cw_offset_hz:700u;
        cw_sidetone_level=saved.sidetone_level<=100u?saved.sidetone_level:40u;
        cw_delay_ms=saved.cw_delay_ms<=2000u?saved.cw_delay_ms:300u;
        cw_keyer_wpm=(saved.cw_wpm>=5u&&saved.cw_wpm<=60u)?saved.cw_wpm:20u;
        rx_low_hz=saved.rx_low_hz; rx_high_hz=saved.rx_high_hz; clamp_rx_pair(&rx_low_hz,&rx_high_hz,true);
        uint16_t tl=saved.tx_low_hz,th=saved.tx_high_hz; clamp_pair(&tl,&th,true); ssb_tx_set_filter(tl,th);
        rx_if_gain=(saved.if_gain>=1u&&saved.if_gain<=64u)?saved.if_gain:8u;
        rx_volume_percent=(saved.volume<=800u)?saved.volume:100u;
        // Codec-only SSB gain scheme: fixed unity DSP microphone gain.
        ssb_tx_set_mic_gain(saved.mic_gain<=1000u?saved.mic_gain:50u);
        // Retained only as the CW RF-drive setting.
        ssb_tx_set_drive(saved.tx_drive<=100u?saved.tx_drive:50u);
        ui_step_index=saved.step_index<8u?saved.step_index:2u;
        rx_agc_set_enabled(saved.agc_enabled!=0u);
        rx_agc_set_speed(saved.agc_speed==0u?RX_AGC_FAST:RX_AGC_SLOW);
        rx_agc_set_target_percent((saved.agc_target>=5u&&saved.agc_target<=90u)?saved.agc_target:8u);
        rx_agc_set_max_gain((saved.agc_max>=1u&&saved.agc_max<=64u)?saved.agc_max:20u);
        if(saved.tx_if_hz>=1000u&&saved.tx_if_hz<=47000u)ssb_tx_set_if_hz(saved.tx_if_hz);
    } else { ssb_tx_set_filter(400u,2800u); ssb_tx_set_mic_gain(50u); ssb_tx_set_drive(50u); rx_agc_set_enabled(true); rx_agc_set_speed(RX_AGC_FAST); rx_agc_set_target_percent(8u); rx_agc_set_max_gain(20u); }
    rx_filter_config();
    ui_vfo_hz[0]=radio_get_dial_hz();
    ui_vfo_hz[1]=(ui_vfo_hz[0] >= 10000000u && ui_vfo_hz[0] <= 18000000u) ? 7100000u : 14200000u;
    ui_active_vfo=0; ui_bfo_delta_hz=0; ui_bfo_base_hz=radio_get_bfo_hz();
    if(have_saved) {
        for(uint8_t i=0;i<2;++i)
            if(saved.vfo_frequency_hz[i]>=100000u && saved.vfo_frequency_hz[i]<=60000000u)
                ui_vfo_hz[i]=saved.vfo_frequency_hz[i];
        ui_active_vfo=saved.active_vfo<=1u?saved.active_vfo:0u;
        if(ui_vfo_hz[ui_active_vfo]!=radio_get_dial_hz()) receiver_tune_hz(ui_vfo_hz[ui_active_vfo]);
        if(saved.am_rx_bw_hz>=2000u && saved.am_rx_bw_hz<=10000u) am_rx_bw_hz=saved.am_rx_bw_hz;
        if(saved.am_carrier_percent<=100u) fft_ssb_tx_set_am_carrier(saved.am_carrier_percent);
        if(saved.cw_keyer_mode<=CW_KEY_IAMBIC_B) cw_keyer_mode=saved.cw_keyer_mode;
        if(saved.cw_decode_pitch_hz>=200u && saved.cw_decode_pitch_hz<=1000u) cw_decode_pitch_hz=saved.cw_decode_pitch_hz;
        if(saved.bfo_delta_hz>=-3000 && saved.bfo_delta_hz<=3000) ui_bfo_delta_hz=saved.bfo_delta_hz;
    }
    fft_ssb_rx_set_noise_reduction(have_saved && saved.dsp_nr_enabled!=0u);
    ssb_tx_set_band_multiplier_q16(power_calibration_get_multiplier_q16(radio_get_dial_hz()));
    ssb_tx_set_enabled(true);  // Phase 7.6: always armed; PTT controls actual TX.
    cw_decoder_set_tone(cw_decode_pitch_hz);
    cw_decoder_set_wpm(cw_keyer_wpm);
    cw_decoder_reset();
    update_rx_mixer_frequency();
    audio_mode = AUDIO_MODE_SSB;

    memset(input_buffer, 0, sizeof(input_buffer));
    memset(output_buffer, 0, sizeof(output_buffer));

    audio_sm = pio_claim_unused_sm(audio_pio, true);
    uint offset = pio_add_program(audio_pio, &wm8731_bidi_slave_program);
    wm8731_bidi_slave_program_init(audio_pio, audio_sm, offset,
        PIN_ADCDAT, PIN_DACDAT, PIN_LRCLK, (float)clock_get_hz(clk_sys));

    adc_sm = pio_claim_unused_sm(adc_pio, true);
    uint adc_offset = pio_add_program(adc_pio, &wm8731_adc_rx_program);
    wm8731_adc_rx_program_init(adc_pio, adc_sm, adc_offset,
        PIN_ADCDAT, (float)clock_get_hz(clk_sys));

    setup_dma();
    pio_sm_set_enabled(audio_pio, audio_sm, true);
    pio_sm_set_enabled(adc_pio, adc_sm, true);

    bool panel_ok = front_panel_init();
    /* The panel is a display/control client; SD-restored RadioCore values win. */
    if (panel_ok) {
        char ft8_buf[24];
        front_panel_set_field("MYCALLSIGN",ft8_mycall);
        front_panel_set_field("MYGRID",ft8_mygrid);
        snprintf(ft8_buf,sizeof(ft8_buf),"%d",ft8_tx_pitch);
        front_panel_set_field("TX_PITCH",ft8_buf);
        front_panel_set_field("TX1ST",ft8_tx6_first?"ON":"OFF");
        front_panel_set_field("AUTO",ft8_panel_auto?"ON":"OFF");
        snprintf(ft8_buf,sizeof(ft8_buf),"%u",ft8_panel_repeat);
        front_panel_set_field("FT8_REPEAT",ft8_buf);
        printf("FT8 startup sync: call=%s grid=%s pitch=%d tx1st=%d auto=%d repeat=%u\n",
               ft8_mycall,ft8_mygrid,ft8_tx_pitch,ft8_tx6_first,ft8_panel_auto,ft8_panel_repeat);
    }
    rtc_boot_restore();
    ft8_n6_init();
    mutex_init(&ft8_result_mutex);
    multicore_launch_core1_with_stack(ft8_core1_worker, ft8_core1_stack, sizeof(ft8_core1_stack));
    printf("FT8: G1 dedicated Core1 decoder ready\n");
    if (!panel_ok) printf("Stock ZBITX front panel: not detected at I2CBB 0x0A\n");

    printf("\n******************************\n");
    printf("WP3DN Ragchew Machine\n");
    printf("Pico Digital Card\n");
    printf("RadioCore v1.2 CW Edition\n\n");
    printf("Ready to Ragchew.\n");
    printf("******************************\n");
    printf("SSB/CW TX armed; CW keyer %u WPM: STRAIGHT / IAMBIC-A / IAMBIC-B selectable; GP14=DIT/KEY, GP15=DAH. Type help.\n> ", cw_keyer_wpm);

    char command[64];
    uint8_t command_len = 0;
    while (true) {
        tud_task();
        /* In CW, GP14/PTT is the DIT paddle, not a direct TX request.
           Establish the manual override before polling GPIO so PTT cannot
           bypass the keyer and key the transmitter by itself. */
        if (!protocol1_mode && operating_mode_is_cw() && !radio_io_manual_tx_active())
            radio_io_set_manual_tx(false);
        if (!protocol1_mode) radio_io_poll();
        else {
            // Preserve the 3K ordering: completed codec halves first, then USB/lwIP.
            p1_process_raw_blocks();
            p1_network_task();
            const p1_network_status_t *pn = p1_network_status();

            bool mox = pn->mox_requested;
            if (mox != p1_last_mox) {
                if (mox) {
                    uint32_t tx_hz = pn->tx_frequency_hz;
                    if (tx_hz < 100000u || tx_hz > 60000000u) tx_hz = pn->rx_frequency_hz;
                    if (tx_hz >= 100000u && tx_hz <= 60000000u && tx_hz != radio_get_dial_hz())
                        receiver_tune_hz(tx_hz);
                    p1_dsp_reset_tx(&p1_dsp);
                    p1_tx_primed = false;
                    radio_io_set_manual_tx(true);
                } else {
                    radio_io_set_manual_tx(false);
                    p1_network_flush_tx_iq();
                    p1_tx_primed = false;
                    p1_dsp_reset_rx(&p1_dsp);
                    uint32_t rx_hz = pn->rx_frequency_hz;
                    if (rx_hz >= 100000u && rx_hz <= 60000000u && rx_hz != radio_get_dial_hz())
                        receiver_tune_hz(rx_hz);
                }
                p1_last_mox = mox;
            } else if (!mox && pn->control_seen &&
                       pn->rx_frequency_hz >= 100000u && pn->rx_frequency_hz <= 60000000u &&
                       pn->rx_frequency_hz != radio_get_dial_hz()) {
                receiver_tune_hz(pn->rx_frequency_hz);
            } else if (mox && pn->tx_frequency_hz >= 100000u && pn->tx_frequency_hz <= 60000000u &&
                       pn->tx_frequency_hz != radio_get_dial_hz()) {
                receiver_tune_hz(pn->tx_frequency_hz);
            }
        }
        if (!protocol1_mode && operating_mode_is_cw()) {
            /* CW paddle assignment on the ZBITX connector:
                 GP14 / PTT  -> DIT
                 GP15 / DASH -> DAH
               PTT is therefore consumed by the keyer in CW modes. */
            bool dit = radio_io_ptt();
            bool dah = radio_io_dash();
            cw_keyer_update(dit, dah);

            if (cw_key_down || cw_keyer_state != CWK_IDLE || dit || dah) {
                cw_release_deadline = make_timeout_time_ms(cw_delay_ms);
                radio_io_set_manual_tx(true);
            } else if (absolute_time_diff_us(get_absolute_time(), cw_release_deadline) > 0) {
                radio_io_set_manual_tx(true);
            } else {
                /* Keep the override engaged FALSE so GP14 remains a DIT
                   paddle and never becomes direct PTT while in CW mode. */
                radio_io_set_manual_tx(false);
            }
        } else {
            cw_keyer_reset();
            cw_envelope_pos = 0u;
            /* Do not clear the manual-TX override while display/CI-V modem PTT
               is active.  The previous code cleared it here and reasserted it
               a few lines later on every main-loop pass, physically toggling
               RX/TX continuously and producing the apparent audio pulses. */
            if (!protocol1_mode && operating_mode == OPERATING_FT8) {
                /* FT8 owns TX arbitration: ignore physical PTT, keep scheduled TX asserted. */
                radio_io_set_manual_tx(ft8_tx6_on);
            } else if (!protocol1_mode && !civ_remote_ptt && !panel_remote_ptt && radio_io_manual_tx_active()) {
                radio_io_clear_manual_tx();
            }
        }
        /* Phase 2BX: CI-V PTT is a persistent manual-TX request.  The normal
           non-CW path clears temporary manual overrides each pass, so reassert
           CAT PTT here until an explicit CI-V PTT OFF arrives. */
        if (!protocol1_mode && civ_remote_ptt && operating_mode != OPERATING_FT8) radio_io_set_manual_tx(true);
        if (!protocol1_mode && panel_remote_ptt && operating_mode != OPERATING_FT8) radio_io_set_manual_tx(true);
        rit_sync_rf_state();

        if (!protocol1_mode) {
            /* v0.7L: decoder consumes the post-IFFT 24 kHz stream. */
            cw_decoder_process();
            spectrum_process();
        }
        if (panel_ok) {
            ui_snapshot_t snap = ui_snapshot();
            ui_event_t event = front_panel_poll(&snap);
            ui_apply_event(event);
            /* Stock panel no longer owns persistence: SD does.  Debounce every
               real operator change, but never save merely because PTT changed. */
            if(event.type!=UI_EVENT_NONE && event.type!=UI_EVENT_REMOTE_PTT &&
               event.type!=UI_EVENT_PROGRAM_MODE) mark_settings_dirty();
            snap = ui_snapshot();
            // Send the POST-event state immediately for knob actions. This avoids
            // a stale pre-event frame and keeps fast tuning visually responsive.
            if (!(protocol1_mode && snap.p1_rx_streaming))
                front_panel_render(&snap, event.type != UI_EVENT_NONE);
        }

        rtc_sync_service();
        ft8_tx6_service();
        ft8_slot_schedule();

        /* N6: full receive decode from the proven N5 inline capture.
           No sample FIFO and no Core1 handoff. */
        ft8_result_service();
        if (ft8_n5_ready) {
            if (!ft8_worker_busy) {
                /* Use the captured slot boundary, not decode-dispatch time. */
                int64_t slot_start_seconds=ft8_slot_last*15;
                int sec=(int)(slot_start_seconds%86400); if(sec<0)sec+=86400;
                snprintf(ft8_job_time,sizeof(ft8_job_time),"%02d%02d%02d",sec/3600,(sec/60)%60,sec%60);
                ft8_decode_idx=0;
                ft8_worker_busy=true;
                ft8_worker_done=false;
                __dmb();
                multicore_fifo_push_blocking((uint32_t)ft8_decode_idx);
                /* Normal automatic FT8 dispatch is silent. */
            } else {
                printf("FT8: G1 worker still busy, dropping completed capture\n");
            }
            ft8_n5_wr=0;
            ft8_n5_ready=false;
        }

        /* Drain queued 6 kHz audio outside the DMA IRQ. One 48-sample frame
           is at most ~0.5 ms on the 2 Mbaud link and uses only ~7% bandwidth. */
        if (panel_ok && !protocol1_mode)
            front_panel_audio_service();

        int ch = tud_cdc_available() ? (int)tud_cdc_read_char() : -1;
        if (ch >= 0) {
            if (ch == '\r' || ch == '\n') {
                if (command_len > 0) {
                    command[command_len] = '\0';
                    apply_console_command(command);
                    command_len = 0;
                }
                printf("> ");
            } else if (ch == 8 || ch == 127) {
                if (command_len > 0) command_len--;
            } else if (ch == 27) {
                command_len = 0;
                printf("\n> ");
            } else if (ch >= 32 && ch <= 126 && command_len < sizeof(command) - 1) {
                command[command_len++] = (char)ch;
            }
        }
        if (settings_dirty && absolute_time_diff_us(get_absolute_time(),settings_save_deadline)<=0) {
            radiocore_settings_t c; collect_settings(&c); if(settings_save(&c)){settings_dirty=false;printf("Settings saved.\n> ");}else printf("SD save failed.\n> ");
        }
        if(logview_open_requested && !settings_dirty) {
            logview_open_requested=false;
            logview_open();
        }
        if(logview_pending) logview_service();
        tight_loop_contents();
    }
}
