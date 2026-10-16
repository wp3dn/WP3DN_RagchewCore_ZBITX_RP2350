#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/types.h"

typedef struct {
  uint sda;
  uint scl;
  uint delay_us;
} i2cbb_t;

void i2cbb_init(i2cbb_t *b, uint sda, uint scl, uint delay_us);
bool i2cbb_probe(i2cbb_t *b, uint8_t addr7);
bool i2cbb_write(i2cbb_t *b, uint8_t addr7, const uint8_t *data, int len);
bool i2cbb_read(i2cbb_t *b, uint8_t addr7, uint8_t *data, int len);
int i2cbb_read_rll(i2cbb_t *b, uint8_t addr7, uint8_t *out, int out_max);
bool i2cbb_write_read(i2cbb_t *b, uint8_t addr7, const uint8_t *w, int wn, uint8_t *r, int rn);
