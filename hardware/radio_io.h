#ifndef RADIOCORE_RADIO_IO_H
#define RADIOCORE_RADIO_IO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RADIO_LPF_OFF = 0,
    RADIO_LPF_A,
    RADIO_LPF_B,
    RADIO_LPF_C,
    RADIO_LPF_D
} radio_lpf_t;

typedef enum {
    RADIO_LPF_MODE_AUTO = 0,
    RADIO_LPF_MODE_MANUAL
} radio_lpf_mode_t;

typedef struct {
    bool ptt;
    bool dash;
    bool tx;
    uint32_t frequency_hz;
    radio_lpf_t lpf;
    radio_lpf_mode_t lpf_mode;
} radio_io_state_t;

void radio_io_init(uint32_t frequency_hz);
void radio_io_poll(void);

bool radio_io_ptt(void);
bool radio_io_dash(void);
bool radio_io_is_tx(void);

void radio_io_enter_receive(void);
void radio_io_enter_transmit(void);
void radio_io_set_manual_tx(bool tx);
void radio_io_clear_manual_tx(void);
bool radio_io_manual_tx_active(void);

void radio_io_set_frequency(uint32_t frequency_hz);
radio_lpf_t radio_io_lpf_for_frequency(uint32_t frequency_hz);
void radio_io_set_lpf_auto(void);
void radio_io_set_lpf_manual(radio_lpf_t lpf);
radio_lpf_t radio_io_get_lpf(void);
radio_lpf_mode_t radio_io_get_lpf_mode(void);

const radio_io_state_t *radio_io_get_state(void);
const char *radio_io_lpf_name(radio_lpf_t lpf);
void radio_io_print_status(void);

#endif
