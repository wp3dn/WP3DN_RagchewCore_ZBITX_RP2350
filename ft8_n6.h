#pragma once
#include <stdint.h>
#include <stddef.h>
void ft8_n6_init(void);
int ft8_n6_decode_capture(const int8_t *samples, size_t count, const char *slot_time, void (*publish)(const char *));
