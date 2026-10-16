#ifndef RADIOCORE_RX_AGC_H
#define RADIOCORE_RX_AGC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    RX_AGC_FAST = 0,
    RX_AGC_SLOW
} rx_agc_speed_t;

void rx_agc_init(void);
void rx_agc_reset(void);
int32_t rx_agc_process(int32_t sample);
void rx_agc_set_enabled(bool enabled);
bool rx_agc_get_enabled(void);
void rx_agc_set_speed(rx_agc_speed_t speed);
rx_agc_speed_t rx_agc_get_speed(void);
bool rx_agc_set_target_percent(uint8_t percent);
uint8_t rx_agc_get_target_percent(void);
bool rx_agc_set_max_gain(uint8_t gain);
uint8_t rx_agc_get_max_gain(void);
uint32_t rx_agc_get_gain_q16(void);
uint32_t rx_agc_get_envelope(void);

#endif
