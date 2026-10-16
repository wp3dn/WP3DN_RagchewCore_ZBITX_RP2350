#pragma once
#include "i2cbb/i2cbb_pico.h"
#define ZBITX_PANEL_ADDR 0x0a
#define ZBITX_SI5351_ADDR 0x60
#define ZBITX_RTC_ADDR 0x68
#define ZBITX_BB_SCL 6
#define ZBITX_BB_SDA 13
i2cbb_t *zbitx_bus(void);
