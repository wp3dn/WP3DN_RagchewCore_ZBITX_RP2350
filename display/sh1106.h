#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "hardware/i2c.h"

#define SH1106_WIDTH 128
#define SH1106_HEIGHT 64

bool sh1106_init(i2c_inst_t *i2c, uint8_t address);
void sh1106_clear(void);
void sh1106_show(void);
void sh1106_pixel(int x, int y, bool on);
void sh1106_text(int x, int y, const char *text, uint8_t scale);
void sh1106_text_center(int y, const char *text, uint8_t scale);
