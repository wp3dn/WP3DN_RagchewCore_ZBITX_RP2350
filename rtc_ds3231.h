#pragma once
#include <stdbool.h>
#include <stdint.h>
/* All accesses from Core 0 only; UTC seconds since Unix epoch. */
bool rtc_ds3231_read(int64_t *epoch_seconds, bool *oscillator_stopped);
bool rtc_ds3231_write(int64_t epoch_seconds);
