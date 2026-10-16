#include "i2cbb_pico.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "i2c_pio_edges.pio.h"
#include "pico/stdlib.h"
#include <assert.h>

// PIO-only prototype. CPU sends one command per bit, no GPIO bit banging.
// Dedicated RP2350 PIO2; PIO0/PIO1 are reserved for the radio audio.
static PIO bus_pio = pio2;
static uint bus_sm;
static uint bus_offset;
static bool bus_ready = false;
static volatile bool bus_fault = false;

// PIO-only: bounded FIFO waits. A stalled slave must not hang USB/audio forever.
static inline uint32_t run_cmd(uint cmd, uint sda_drive_low) {
    if (bus_fault || !bus_ready) return 0;
    absolute_time_t deadline = make_timeout_time_us(5000);
    while (pio_sm_is_tx_fifo_full(bus_pio, bus_sm)) {
        if (time_reached(deadline)) { bus_fault = true; return 0; }
        tight_loop_contents();
    }
    pio_sm_put(bus_pio, bus_sm, ((bus_offset + cmd) & 31u) | (sda_drive_low << 5));
    while (pio_sm_is_rx_fifo_empty(bus_pio, bus_sm)) {
        if (time_reached(deadline)) { bus_fault = true; return 0; }
        tight_loop_contents();
    }
    return pio_sm_get(bus_pio, bus_sm);
}
static inline void start(i2cbb_t *b) { (void)b; (void)run_cmd(i2c_pio_edges_offset_start_cycle, 0); }
static inline void stop(i2cbb_t *b) { (void)b; (void)run_cmd(i2c_pio_edges_offset_stop_cycle, 0); }
static inline bool bit(i2cbb_t *b, bool drive_low) {
    (void)b;
    return (run_cmd(i2c_pio_edges_offset_bit_cycle, drive_low ? 1u : 0u) & 1u) != 0;
}

void i2cbb_init(i2cbb_t *b, uint sda, uint scl, uint delay_us) {
    b->sda=sda; b->scl=scl; b->delay_us=delay_us;
    // The PIO program uses WAIT GPIO 6 and is dedicated to SDA GP13 / SCL GP6.
    assert(sda == 13 && scl == 6);
    assert(!bus_ready);
    gpio_init(sda); gpio_init(scl);
    gpio_put(sda, 0); gpio_put(scl, 0);
    gpio_set_dir(sda, GPIO_IN); gpio_set_dir(scl, GPIO_IN);
    gpio_pull_up(sda); gpio_pull_up(scl);
    uint offset=pio_add_program(bus_pio, &i2c_pio_edges_program);
    bus_offset = offset;
    bus_sm=pio_claim_unused_sm(bus_pio, true);
    pio_sm_config cfg=i2c_pio_edges_program_get_default_config(offset);
    sm_config_set_out_pins(&cfg, sda, 1);
    sm_config_set_set_pins(&cfg, sda, 1);
    sm_config_set_in_pins(&cfg, sda);
    sm_config_set_sideset_pins(&cfg, scl);
    sm_config_set_sideset(&cfg, 1, false, true);
    sm_config_set_out_shift(&cfg, true, false, 32); // command occupies bits 0..4, SDA drive bit 5
    sm_config_set_in_shift(&cfg, false, false, 32);
    sm_config_set_clkdiv(&cfg, 240.0f); // ~1MHz PIO at 240MHz sysclk; conservative I2C timing
    pio_gpio_init(bus_pio, sda);
    pio_gpio_init(bus_pio, scl);
    pio_sm_init(bus_pio, bus_sm, offset, &cfg);
    // PIO drives only LOW, never HIGH; external/internal pullups release lines.
    pio_sm_set_pins_with_mask(bus_pio, bus_sm, 0, (1u<<sda)|(1u<<scl));
    pio_sm_set_pindirs_with_mask(bus_pio, bus_sm, 0, (1u<<sda)|(1u<<scl));
    pio_sm_set_enabled(bus_pio, bus_sm, true);
    bus_fault=false;
    bus_ready=true;
}

static bool write_byte(i2cbb_t *b, uint8_t v) {
    for (int i=7; i>=0; --i) (void)bit(b, !(v & (1u<<i)));
    return !bit(b, false) && !bus_fault;
}
static uint8_t read_byte(i2cbb_t *b, bool ack) {
    uint8_t v=0;
    for (int i=0;i<8;++i) v=(uint8_t)((v<<1) | (bit(b,false)?1:0));
    (void)bit(b,ack);
    return v;
}
bool i2cbb_probe(i2cbb_t *b, uint8_t addr7) {
    start(b); bool ok=write_byte(b,(uint8_t)(addr7<<1)); stop(b); return ok && !bus_fault;
}
bool i2cbb_write(i2cbb_t *b, uint8_t addr7, const uint8_t *data, int len) {
    start(b);
    if (!write_byte(b,(uint8_t)(addr7<<1))) { stop(b); return false; }
    for(int i=0;i<len;i++) if(!write_byte(b,data[i])) {stop(b);return false;}
    stop(b);return !bus_fault;
}
bool i2cbb_read(i2cbb_t *b, uint8_t addr7, uint8_t *data, int len) {
    start(b);
    if (!write_byte(b,(uint8_t)((addr7<<1)|1))) {stop(b);return false;}
    for(int i=0;i<len;i++) data[i]=read_byte(b,i!=len-1);
    stop(b);return !bus_fault;
}
int i2cbb_read_rll(i2cbb_t *b, uint8_t addr7, uint8_t *out, int out_max) {
    start(b);
    if (!write_byte(b,(uint8_t)((addr7<<1)|1))) {stop(b);return -1;}
    uint8_t n=read_byte(b,true);
    if (!n) {stop(b);return 0;}
    if (n>out_max) n=(uint8_t)out_max;
    for(int i=0;i<n;i++) out[i]=read_byte(b,i!=n-1);
    stop(b);return bus_fault ? -1 : n;
}
bool i2cbb_write_read(i2cbb_t *b,uint8_t addr7,const uint8_t *w,int wn,uint8_t *r,int rn) {
    start(b);
    if(!write_byte(b,(uint8_t)(addr7<<1))) {stop(b);return false;}
    for(int i=0;i<wn;i++) if(!write_byte(b,w[i])) {stop(b);return false;}
    start(b);
    if(!write_byte(b,(uint8_t)((addr7<<1)|1))) {stop(b);return false;}
    for(int i=0;i<rn;i++) r[i]=read_byte(b,i!=rn-1);
    stop(b);return !bus_fault;
}
