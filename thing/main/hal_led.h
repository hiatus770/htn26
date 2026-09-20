#pragma once
#include <stdint.h>

typedef struct { uint8_t r, g, b; } rgb_t;

// index 0..5 follows the physical front layout from the HAL guide:
// 0 UpperLeft, 1 UpperRight, 2 MiddleRight, 3 BottomRight, 4 BottomLeft, 5 MiddleLeft
void hal_led_init(void);
void hal_led_set(const rgb_t colors[6]);
