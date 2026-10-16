#include "settings.h"
#include <string.h>
#include "ff.h"
#include "sd_card.h"
#include "tusb.h"
#include <stdio.h>
#include <stddef.h>

#define SETTINGS_MAGIC 0x52433737u
#define SETTINGS_VERSION 7u
#define SETTINGS_PREVIOUS_VERSION 6u
#define SETTINGS_PATH "0:/RAGCHEW/settings.bin"
#define SETTINGS_TEMP "0:/RAGCHEW/settings.tmp"
#define SETTINGS_BACKUP "0:/RAGCHEW/settings.bak"

typedef struct {
    uint32_t magic, version, checksum;
    radiocore_settings_t data;
} sd_record_t;

static FATFS fs;
static bool mounted;
static bool mount_tried;
static char sd_diag[112] = "[SD] not attempted\r\n";

static void diag_set(const char *msg){ snprintf(sd_diag,sizeof(sd_diag),"%s",msg); }
void settings_sd_diag_print(void){
    if (!tud_cdc_connected()) return;
    tud_cdc_write_str(sd_diag); tud_cdc_write_flush();
}

static uint32_t checksum_bytes(const void *data, uint32_t size) {
    const uint8_t *p=(const uint8_t*)data; uint32_t c=2166136261u;
    for(uint32_t i=0;i<size;++i){ c^=p[i]; c*=16777619u; }
    return c;
}

static bool ensure_mounted(void){
    if(mounted) return true;
    if(mount_tried) return false;
    mount_tried=true;
    diag_set("[SD] starting sd_init_driver()\r\n");
    if(!sd_init_driver()){ diag_set("[SD] sd_init_driver() FAILED\r\n"); return false; }
    FRESULT mfr=f_mount(&fs,"0:",1);
    if(mfr!=FR_OK){ snprintf(sd_diag,sizeof(sd_diag),"[SD] driver OK, f_mount FAILED: %d\r\n",(int)mfr); return false; }
    mounted=true;
    FRESULT fr=f_mkdir("0:/RAGCHEW");
    if(fr==FR_OK || fr==FR_EXIST){ snprintf(sd_diag,sizeof(sd_diag),"[SD] driver OK, mount OK, mkdir=%d (OK)\r\n",(int)fr); return true; }
    snprintf(sd_diag,sizeof(sd_diag),"[SD] mount OK, mkdir FAILED: %d\r\n",(int)fr); return false;
}

/* v6 is the exact prefix of v7, including compiler padding before the new fields. */
#define V6_DATA_SIZE ((uint32_t)offsetof(radiocore_settings_t, vfo_frequency_hz))

static bool read_record(const char *path, radiocore_settings_t *out) {
    FIL f; UINT got=0;
    if (f_open(&f,path,FA_READ)!=FR_OK) return false;
    struct { uint32_t magic, version, checksum; } header;
    FRESULT fr=f_read(&f,&header,sizeof(header),&got);
    if(fr!=FR_OK || got!=sizeof(header) || header.magic!=SETTINGS_MAGIC ||
       (header.version!=SETTINGS_VERSION && header.version!=SETTINGS_PREVIOUS_VERSION)) {
        f_close(&f); return false;
    }
    uint32_t length=(header.version==SETTINGS_VERSION)?sizeof(*out):V6_DATA_SIZE;
    memset(out,0,sizeof(*out));
    fr=f_read(&f,out,length,&got); f_close(&f);
    if(fr!=FR_OK || got!=length || header.checksum!=checksum_bytes(out,length)) return false;
    if(header.version==SETTINGS_PREVIOUS_VERSION) {
        out->vfo_frequency_hz[0]=out->frequency_hz;
        out->vfo_frequency_hz[1]=14200000u;
        out->am_rx_bw_hz=6000u;
        out->am_carrier_percent=50u;
        out->cw_keyer_mode=2u;
        out->cw_decode_pitch_hz=500u;
        out->ft8_tx_pitch_hz=1500u;
        /* Existing saved v6 values are retained; new fields get defaults. */
    }
    return true;
}

bool settings_load(radiocore_settings_t *out){
    if(!out || !ensure_mounted()) return false;
    if(read_record(SETTINGS_PATH,out)) return true;
    return read_record(SETTINGS_BACKUP,out);
}

bool settings_save(const radiocore_settings_t *in){
    if(!in || !ensure_mounted()) return false;
    sd_record_t r={0}; r.magic=SETTINGS_MAGIC; r.version=SETTINGS_VERSION;
    r.data=*in; r.checksum=checksum_bytes(&r.data,sizeof(r.data));
    FIL f; UINT wrote=0;
    if(f_open(&f,SETTINGS_TEMP,FA_CREATE_ALWAYS|FA_WRITE)!=FR_OK) return false;
    FRESULT fr=f_write(&f,&r,sizeof(r),&wrote);
    if(fr==FR_OK && wrote==sizeof(r)) fr=f_sync(&f);
    f_close(&f);
    if(fr!=FR_OK || wrote!=sizeof(r)){ f_unlink(SETTINGS_TEMP); return false; }
    /* Proven v6 SD save sequence; retain v7 record format and v6 loading. */
    f_unlink(SETTINGS_PATH);
    if(f_rename(SETTINGS_TEMP,SETTINGS_PATH)!=FR_OK){ f_unlink(SETTINGS_TEMP); return false; }
    return true;
}

/* Append-only ADIF log; uses the same mounted FatFs volume as settings. */
bool log_adif_append(const char *record) {
    if (!record || !*record || !ensure_mounted()) return false;
    FIL f; UINT n=0; FRESULT fr;
    const char *path="0:/RAGCHEW/ragchew_log.adi";
    fr=f_open(&f,path,FA_OPEN_ALWAYS|FA_WRITE);
    if(fr!=FR_OK) return false;
    FSIZE_t old=f_size(&f);
    fr=f_lseek(&f,old);
    if(fr==FR_OK && old==0) {
        const char *hdr="RagchewCore ADIF Log\r\n<ADIF_VER:5>3.1.4\r\n<PROGRAMID:11>RagchewCore\r\n<EOH>\r\n";
        fr=f_write(&f,hdr,(UINT)strlen(hdr),&n);
        if(fr==FR_OK && n!=strlen(hdr)) fr=FR_DISK_ERR;
    }
    if(fr==FR_OK) {
        size_t len=strlen(record);
        fr=f_write(&f,record,(UINT)len,&n);
        if(fr==FR_OK && n!=len) fr=FR_DISK_ERR;
    }
    if(fr==FR_OK) fr=f_sync(&f);
    FRESULT close_result=f_close(&f);
    return fr==FR_OK && close_result==FR_OK;
}

int log_adif_read_snapshot(char *out, unsigned capacity) {
    if (!out || capacity < 2 || !ensure_mounted()) return -1;
    FIL f; UINT n=0;
    FRESULT fr=f_open(&f,"0:/RAGCHEW/ragchew_log.adi",FA_READ);
    if(fr!=FR_OK) return -2;
    /* Snapshot is bounded; never allocate an SD file buffer on the stack. */
    FSIZE_t size=f_size(&f);
    if(size >= capacity) { f_close(&f); return -3; }
    fr=f_read(&f,out,(UINT)size,&n);
    FRESULT cl=f_close(&f);
    if(fr!=FR_OK || cl!=FR_OK || n!=size) return -4;
    out[n]=0;
    return (int)n;
}
