#include "settings.h"
#include <string.h>
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#define SETTINGS_MAGIC 0x52433737u
#define SETTINGS_VERSION 6u
#define SETTINGS_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t frequency_hz;
    uint8_t sideband;
    uint16_t rx_low_hz, rx_high_hz, tx_low_hz, tx_high_hz;
    uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out;
    uint8_t if_gain;
    uint16_t volume, mic_gain;
    uint8_t tx_drive;
    uint8_t step_index;
    uint8_t agc_enabled, agc_speed, agc_target, agc_max;
    uint32_t tx_if_hz;
} radiocore_settings_v2_t;

typedef struct {
    uint32_t frequency_hz;
    uint8_t sideband;
    uint16_t rx_low_hz, rx_high_hz, tx_low_hz, tx_high_hz;
    uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out;
    uint8_t if_gain;
    uint16_t volume, mic_gain;
    uint8_t tx_drive;
    uint8_t step_index;
    uint8_t agc_enabled, agc_speed, agc_target, agc_max;
    uint32_t tx_if_hz;
    uint8_t operating_mode;
    uint16_t cw_offset_hz;
    uint8_t sidetone_level;
    uint16_t cw_delay_ms;
    uint32_t bfo_hz, global_if_hz;
    uint8_t clk1_drive_ma, clk2_drive_ma;
} radiocore_settings_v4_t;

typedef struct {
    uint32_t frequency_hz;
    uint8_t sideband;
    uint16_t rx_low_hz, rx_high_hz, tx_low_hz, tx_high_hz;
    uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out;
    uint8_t if_gain;
    uint16_t volume, mic_gain;
    uint8_t tx_drive;
    uint8_t step_index;
    uint8_t agc_enabled, agc_speed, agc_target, agc_max;
    uint32_t tx_if_hz;
    uint8_t operating_mode;
    uint16_t cw_offset_hz;
    uint8_t sidetone_level;
    uint16_t cw_delay_ms;
    uint32_t bfo_hz, global_if_hz;
    uint8_t clk1_drive_ma, clk2_drive_ma;
    uint32_t band_frequency_hz[RADIOCORE_BAND_COUNT];
    int16_t band_spectrum_offset_db[RADIOCORE_BAND_COUNT];
} radiocore_settings_v5_t;

typedef struct {
    uint32_t magic, version, checksum;
    radiocore_settings_t data;
    uint8_t pad[FLASH_PAGE_SIZE - 12 - sizeof(radiocore_settings_t)];
} flash_record_t;

static uint32_t checksum_bytes(const void *data, uint32_t size) {
    const uint8_t *p=(const uint8_t*)data; uint32_t c=2166136261u;
    for (uint32_t i=0;i<size;++i) { c^=p[i]; c*=16777619u; }
    return c;
}

bool settings_load(radiocore_settings_t *out) {
    const uint8_t *base=(const uint8_t *)(XIP_BASE + SETTINGS_OFFSET);
    const uint32_t *header=(const uint32_t *)base;
    if (header[0]!=SETTINGS_MAGIC) return false;

    if (header[1]==SETTINGS_VERSION) {
        const flash_record_t *r=(const flash_record_t *)base;
        if (r->checksum!=checksum_bytes(&r->data,sizeof(r->data))) return false;
        *out=r->data;
        return true;
    }

    /* Upgrade v5: identical settings plus the new keyer speed field. */
    if (header[1]==5u) {
        const radiocore_settings_v5_t *old=(const radiocore_settings_v5_t *)(base+12);
        if (header[2]!=checksum_bytes(old,sizeof(*old))) return false;
        memset(out,0,sizeof(*out));
        /* v6 inserted cw_wpm before BFO/global-IF, so a raw memcpy would
           shift every field after cw_delay_ms.  Copy the common prefix,
           then map the tail explicitly. */
        memcpy(out, old, offsetof(radiocore_settings_t, cw_wpm));
        out->cw_wpm = 20u;
        out->bfo_hz = old->bfo_hz;
        out->global_if_hz = old->global_if_hz;
        out->clk1_drive_ma = old->clk1_drive_ma;
        out->clk2_drive_ma = old->clk2_drive_ma;
        memcpy(out->band_frequency_hz, old->band_frequency_hz, sizeof(out->band_frequency_hz));
        memcpy(out->band_spectrum_offset_db, old->band_spectrum_offset_db, sizeof(out->band_spectrum_offset_db));
        return true;
    }

    /* Upgrade the previous settings format. Per-band values are initialized
     * by main.c from its normal band defaults and current saved frequency. */
    if (header[1]==4u) {
        const radiocore_settings_v4_t *old=(const radiocore_settings_v4_t *)(base+12);
        if (header[2]!=checksum_bytes(old,sizeof(*old))) return false;
        memset(out,0,sizeof(*out));
        memcpy(out,old,sizeof(*old));
        return true;
    }

    // Preserve all v1.1 settings on the first boot of the CW Edition.
    if (header[1]==3u) {
        typedef struct {
            uint32_t frequency_hz; uint8_t sideband; uint16_t rx_low_hz, rx_high_hz, tx_low_hz, tx_high_hz;
            uint8_t rx_codec_in, mic_codec_in, ear_codec_out, mod_codec_out; uint8_t if_gain; uint16_t volume, mic_gain;
            uint8_t tx_drive, step_index, agc_enabled, agc_speed, agc_target, agc_max; uint32_t tx_if_hz;
            uint8_t operating_mode; uint16_t cw_offset_hz; uint8_t sidetone_level; uint16_t cw_delay_ms;
        } radiocore_settings_v3_t;
        const radiocore_settings_v3_t *old=(const radiocore_settings_v3_t *)(base+12);
        if (header[2]!=checksum_bytes(old,sizeof(*old))) return false;
        memset(out,0,sizeof(*out)); memcpy(out,old,sizeof(*old));
        out->bfo_hz=40035000u; out->global_if_hz=24000u; out->clk1_drive_ma=2u; out->clk2_drive_ma=6u;
        return true;
    }

    if (header[1]==2u) {
        const radiocore_settings_v2_t *old=(const radiocore_settings_v2_t *)(base+12);
        if (header[2]!=checksum_bytes(old,sizeof(*old))) return false;
        memset(out,0,sizeof(*out));
        memcpy(out,old,sizeof(*old));
        out->operating_mode=old->sideband ? 1u : 0u;
        out->cw_offset_hz=700u;
        out->sidetone_level=40u;
        out->cw_delay_ms=300u;
        return true;
    }
    return false;
}

bool settings_save(const radiocore_settings_t *in) {
    flash_record_t r; memset(&r,0xff,sizeof(r));
    r.magic=SETTINGS_MAGIC; r.version=SETTINGS_VERSION; r.data=*in;
    r.checksum=checksum_bytes(in,sizeof(*in));
    uint32_t ints=save_and_disable_interrupts();
    flash_range_erase(SETTINGS_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET, (const uint8_t*)&r, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    return true;
}
