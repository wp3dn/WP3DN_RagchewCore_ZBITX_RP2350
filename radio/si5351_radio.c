#include "si5351_radio.h"
#include <stdio.h>
#include "i2cbb/zbitx_bus.h"
#define SI5351_ADDR 0x60
static uint32_t bfo_hz_;
static uint32_t dial_hz_;
static uint32_t if_hz_;
static uint32_t vfo_hz_;
static int16_t rit_offset_hz_ = 0;
static bool rit_rx_active_ = false;
#define SI_CLK0_CONTROL 16
#define SI_CLK1_CONTROL 17
#define SI_CLK2_CONTROL 18
#define SI_SYNTH_PLL_A 26
#define SI_SYNTH_PLL_B 34
#define SI_SYNTH_MS_0 42
#define SI_SYNTH_MS_1 50
#define SI_SYNTH_MS_2 58
#define SI_PLL_RESET 177
#define SI_SYNTH_PLL_A_SRC SI_SYNTH_PLL_A
#define SI_SYNTH_PLL_B_SRC SI_SYNTH_PLL_B
#define SI_CLK_SRC_PLL_B 0x20
#define SI5351_CLK_INTEGER_MODE 0x40
#define SI5351_CLK_INPUT_MULTISYNTH_N 0x0c
#define SI5351_DIVBY4 0x0c
#define SI5351_DRIVE_2MA 0x00
#define SI5351_DRIVE_4MA 0x01
#define SI5351_DRIVE_6MA 0x02
#define SI5351_DRIVE_8MA 0x03
static uint8_t clk_drive_ma_[3] = {2, 8, 8};

static bool si_write(uint8_t reg, uint8_t value) {
    uint8_t b[2] = {reg, value};
    return i2cbb_write(zbitx_bus(), SI5351_ADDR, b, 2);
}

static bool si_read(uint8_t reg, uint8_t *value) {
    return i2cbb_write_read(zbitx_bus(), SI5351_ADDR, &reg, 1, value, 1);
}

static void si_setup_pll(uint8_t pll, uint8_t mult, uint32_t num, uint32_t denom) {
    uint32_t p1, p2, p3;
    if (num == 0) { p1 = 128u * mult - 512u; p2 = 0; p3 = 1; }
    else {
        p1 = 128u * mult + ((128u * num) / denom) - 512u;
        p2 = 128u * num - denom * ((128u * num) / denom);
        p3 = denom;
    }
    si_write(pll+0, (p3 >> 8) & 0xff); si_write(pll+1, p3 & 0xff);
    si_write(pll+2, (p1 >> 16) & 0x03); si_write(pll+3, (p1 >> 8) & 0xff);
    si_write(pll+4, p1 & 0xff);
    si_write(pll+5, ((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f));
    si_write(pll+6, (p2 >> 8) & 0xff); si_write(pll+7, p2 & 0xff);
}

static uint8_t drive_bits(uint8_t milliamps) {
    switch (milliamps) {
        case 2: return SI5351_DRIVE_2MA;
        case 4: return SI5351_DRIVE_4MA;
        case 6: return SI5351_DRIVE_6MA;
        case 8: return SI5351_DRIVE_8MA;
        default: return SI5351_DRIVE_2MA;
    }
}

static uint8_t clock_control_value(uint8_t clk, uint8_t pll) {
    uint8_t dat = drive_bits(clk_drive_ma_[clk]) |
                  SI5351_CLK_INPUT_MULTISYNTH_N |
                  SI5351_CLK_INTEGER_MODE;
    if (pll == SI_SYNTH_PLL_B_SRC) dat |= SI_CLK_SRC_PLL_B;
    return dat;
}

static void si_setup_ms(uint8_t clk, uint8_t pll, uint32_t div) {
    uint8_t synth = clk == 0 ? SI_SYNTH_MS_0 : (clk == 1 ? SI_SYNTH_MS_1 : SI_SYNTH_MS_2);
    uint8_t ctrl = SI_CLK0_CONTROL + clk;
    uint32_t p1, p2 = 0, p3 = 1;
    uint8_t div4 = 0;
    if (div == 4) { p1 = 0; div4 = SI5351_DIVBY4; }
    else p1 = 128u * div - 512u;
    si_write(synth+0, (p3 >> 8) & 0xff); si_write(synth+1, p3 & 0xff);
    si_write(synth+2, ((p1 >> 16) & 0x03) | div4);
    si_write(synth+3, (p1 >> 8) & 0xff); si_write(synth+4, p1 & 0xff);
    si_write(synth+5, ((p3 >> 12) & 0xf0) | ((p2 >> 16) & 0x0f));
    si_write(synth+6, (p2 >> 8) & 0xff); si_write(synth+7, p2 & 0xff);
    si_write(ctrl, clock_control_value(clk, pll));
}

static bool si_set_freq(uint8_t clk, uint32_t frequency) {
    if (clk > 2 || frequency < 8000u || frequency > 160000000u) return false;
    uint8_t pll = clk == 1 ? SI_SYNTH_PLL_B_SRC : SI_SYNTH_PLL_A_SRC;
    uint32_t div = (uint32_t)((650000000ull + frequency - 1ull) / frequency);
    if (div < 4) div = 4;
    if (div & 1u) div++;
    if (div > 1800) div = 1800;
    uint64_t pll_freq = (uint64_t)frequency * div;
    const uint32_t xtal = 25000000u, denom = 0x80000u;
    uint32_t mult = (uint32_t)(pll_freq / xtal);
    uint32_t num = (uint32_t)(((pll_freq % xtal) * denom) / xtal);
    si_setup_pll(pll, (uint8_t)mult, num, denom);
    si_setup_ms(clk, pll, div);
    si_write(SI_PLL_RESET, 0xa0);
    return true;
}


bool radio_init(uint32_t bfo_hz, uint32_t dial_hz, uint32_t if_hz) {
    bfo_hz_=bfo_hz; dial_hz_=dial_hz; if_hz_=if_hz;
    uint8_t status=0xff;
    if (!si_read(0,&status)) return false;
    si_write(SI_CLK0_CONTROL,0x80); si_write(SI_CLK1_CONTROL,0x80); si_write(SI_CLK2_CONTROL,0x80);
    vfo_hz_=bfo_hz_+dial_hz_-if_hz_;
    bool ok=si_set_freq(1,bfo_hz_); ok &= si_set_freq(2,vfo_hz_);
    printf("Si5351 status=0x%02x CLK1/BFO=%lu Hz CLK2/VFO=%lu Hz dial=%lu Hz IF=%lu Hz\n", status,
      (unsigned long)bfo_hz_,(unsigned long)vfo_hz_,(unsigned long)dial_hz_,(unsigned long)if_hz_);
    return ok;
}
static bool radio_apply_vfo(uint32_t base_dial_hz, int16_t rit_hz, bool use_rit) {
    int64_t actual = (int64_t)base_dial_hz + (use_rit ? (int64_t)rit_hz : 0);
    if (actual < 100000 || actual > 60000000) return false;
    uint32_t nv = bfo_hz_ + (uint32_t)actual - if_hz_;
    if (!si_set_freq(2, nv)) return false;
    vfo_hz_ = nv;
    return true;
}

bool radio_tune_hz(uint32_t new_dial_hz) {
    if (new_dial_hz < 100000u || new_dial_hz > 60000000u) {
        printf("Frequency out of range: use 100 kHz to 60 MHz\n");
        return false;
    }
    if (!radio_apply_vfo(new_dial_hz, rit_offset_hz_, rit_rx_active_)) {
        printf("Si5351 rejected VFO for dial %lu Hz\n",(unsigned long)new_dial_hz);
        return false;
    }
    dial_hz_ = new_dial_hz;
    return true;
}

bool radio_set_rit_offset_hz(int16_t offset_hz) {
    if (offset_hz < -2000 || offset_hz > 2000) return false;
    int16_t old = rit_offset_hz_;
    rit_offset_hz_ = offset_hz;
    if (rit_rx_active_ && !radio_apply_vfo(dial_hz_, rit_offset_hz_, true)) {
        rit_offset_hz_ = old;
        return false;
    }
    return true;
}

bool radio_set_rit_rx_active(bool active) {
    if (rit_rx_active_ == active) return true;
    if (!radio_apply_vfo(dial_hz_, rit_offset_hz_, active)) return false;
    rit_rx_active_ = active;
    return true;
}

int16_t radio_get_rit_offset_hz(void){ return rit_offset_hz_; }

uint32_t radio_get_rx_hz(void){
    int64_t f = (int64_t)dial_hz_ + (rit_rx_active_ ? (int64_t)rit_offset_hz_ : 0);
    if (f < 0) f = 0;
    return (uint32_t)f;
}

bool radio_set_bfo_hz(uint32_t new_bfo_hz) {
    if (new_bfo_hz < 39000000u || new_bfo_hz > 41000000u) return false;
    uint32_t actual = radio_get_rx_hz();
    uint32_t new_vfo = new_bfo_hz + actual - if_hz_;
    if (!si_set_freq(1, new_bfo_hz)) return false;
    if (!si_set_freq(2, new_vfo)) return false;
    bfo_hz_ = new_bfo_hz; vfo_hz_ = new_vfo; return true;
}
bool radio_set_if_hz(uint32_t new_if_hz) {
    if (new_if_hz < 1000u || new_if_hz > 47000u) return false;
    uint32_t actual = radio_get_rx_hz();
    uint32_t new_vfo = bfo_hz_ + actual - new_if_hz;
    if (!si_set_freq(2, new_vfo)) return false;
    if_hz_ = new_if_hz; vfo_hz_ = new_vfo; return true;
}
uint32_t radio_get_bfo_hz(void){return bfo_hz_;}
uint32_t radio_get_dial_hz(void){return dial_hz_;}
uint32_t radio_get_if_hz(void){return if_hz_;}
uint32_t radio_get_vfo_hz(void){return vfo_hz_;}


bool radio_set_clk_drive(uint8_t clk, uint8_t milliamps) {
    if (clk > 2 || !(milliamps == 2 || milliamps == 4 || milliamps == 6 || milliamps == 8)) return false;
    clk_drive_ma_[clk] = milliamps;
    uint8_t pll = clk == 1 ? SI_SYNTH_PLL_B_SRC : SI_SYNTH_PLL_A_SRC;
    return si_write((uint8_t)(SI_CLK0_CONTROL + clk), clock_control_value(clk, pll));
}

uint8_t radio_get_clk_drive(uint8_t clk) {
    return clk <= 2 ? clk_drive_ma_[clk] : 0;
}
