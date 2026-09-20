#pragma once
#include <stdint.h>

// Internal render resolution. Scaled 2x by hal_display to fill the physical
// 320x240 panel. Kept low so the framebuffer (RENDER_W*RENDER_H*2 bytes)
// stays well within this chip's ~400KB SRAM budget (no PSRAM on this module).
#define RENDER_W 160
#define RENDER_H 120

typedef struct {
    float x, y;             // position, in map cells
    float dir_x, dir_y;     // normalized view direction
    float plane_x, plane_y; // camera plane, perpendicular to dir_*, sets FOV
} camera_t;

// Renders walls+floor+ceiling into fb (RENDER_W*RENDER_H RGB565).
// wall_dist_out[x] receives the perpendicular wall distance for column x,
// used by the sprite renderer in game.c for occlusion.
void raycast_render(uint16_t *fb, const camera_t *cam, float *wall_dist_out);
