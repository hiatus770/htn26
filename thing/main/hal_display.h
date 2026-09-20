#pragma once
#include <stdint.h>

#define DISP_W 320
#define DISP_H 240

void hal_display_init(void);

// Nearest-neighbor upscales an RGB565 buffer (src_w x src_h) by `scale` and
// blits it to the physical panel in horizontal stripes, so we never need a
// full 320x240 framebuffer in RAM (that alone would be 150KB on a chip with
// ~400KB total SRAM and no PSRAM).
void hal_display_blit_scaled(const uint16_t *src, int src_w, int src_h, int scale);
