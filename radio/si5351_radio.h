#ifndef RADIOCORE_SI5351_RADIO_H
#define RADIOCORE_SI5351_RADIO_H
#include <stdbool.h>
#include <stdint.h>
bool radio_init(uint32_t bfo_hz, uint32_t dial_hz, uint32_t if_hz);
bool radio_tune_hz(uint32_t dial_hz);
bool radio_set_rit_offset_hz(int16_t offset_hz);
bool radio_set_rit_rx_active(bool active);
int16_t radio_get_rit_offset_hz(void);
uint32_t radio_get_rx_hz(void);
bool radio_set_bfo_hz(uint32_t bfo_hz);
bool radio_set_if_hz(uint32_t if_hz);
bool radio_set_clk_drive(uint8_t clk, uint8_t milliamps);
uint8_t radio_get_clk_drive(uint8_t clk);
uint32_t radio_get_bfo_hz(void);
uint32_t radio_get_dial_hz(void);
uint32_t radio_get_if_hz(void);
uint32_t radio_get_vfo_hz(void);
#endif
