#pragma once
#include <stdint.h>

#define MAP_W 16
#define MAP_H 16

// 0 = empty floor, 1..4 = wall id (each id gets its own color in raycaster.c)
extern const uint8_t g_map[MAP_H][MAP_W];
