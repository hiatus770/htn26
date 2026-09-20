#include "raycaster.h"
#include "map.h"
#include <math.h>

#define RGB565(r, g, b) ((uint16_t)(((r) & 0x1F) << 11 | ((g) & 0x3F) << 5 | ((b) & 0x1F)))

static uint16_t wall_color(uint8_t id, int dark_side)
{
    uint16_t c;
    switch (id) {
        case 1:  c = RGB565(22, 40, 10); break; // greenish stone
        case 2:  c = RGB565(24, 8, 8);   break; // dark red
        case 3:  c = RGB565(6, 20, 26);  break; // blue-teal
        case 4:  c = RGB565(26, 22, 4);  break; // brown/yellow
        default: c = RGB565(31, 0, 31);  break; // magenta: shouldn't happen
    }
    if (dark_side) {
        uint8_t r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
        c = RGB565(r * 2 / 3, g * 2 / 3, b * 2 / 3);
    }
    return c;
}

static uint16_t darken(uint16_t base)
{
    uint8_t r = (base >> 11) & 0x1F, g = (base >> 5) & 0x3F, b = base & 0x1F;
    return RGB565(r * 3 / 5, g * 3 / 5, b * 3 / 5);
}

// Procedural running-bond brick pattern, running along the wall-hit's
// fractional (wall_x, v) coordinate. Pure integer/fixed-point math: this
// chip's RISC-V core (rv32imc) has no hardware FPU, so a per-pixel floorf()
// here was the actual cause of the slowdown, not render resolution.
#define BRICK_ROWS 4
#define BRICK_FP_SHIFT 12
#define BRICK_FP_ONE (1 << BRICK_FP_SHIFT) // fixed-point 1.0
static uint16_t apply_brick(uint16_t base, uint16_t base_dark, int wall_x_fp, int y, int draw_start_raw, int line_h)
{
    int dy = y - draw_start_raw; // >= 0
    int row_num = dy * BRICK_ROWS;
    int row = row_num / line_h;
    int row_rem = row_num - row * line_h; // row_num % line_h
    if (row_rem * 10 < line_h) return base_dark; // near a horizontal mortar line

    int col_offset_fp = (row & 1) ? (BRICK_FP_ONE / 2) : 0;
    int cu_fp = (wall_x_fp * 2 + col_offset_fp) & (BRICK_FP_ONE - 1); // *COLS=2, wrap to [0,1)
    if (cu_fp * 10 < BRICK_FP_ONE) return base_dark; // near a vertical mortar line

    return base;
}

// Classic grid-DDA raycaster (per-column), see e.g. lodev.org's tutorial.
void raycast_render(uint16_t *fb, const camera_t *cam, float *wall_dist_out)
{
    const uint16_t ceil_color = RGB565(6, 10, 10);
    const uint16_t floor_color = RGB565(10, 10, 8);

    for (int x = 0; x < RENDER_W; x++) {
        float camera_x = 2.0f * x / (float)RENDER_W - 1.0f;
        float ray_dx = cam->dir_x + cam->plane_x * camera_x;
        float ray_dy = cam->dir_y + cam->plane_y * camera_x;

        int map_x = (int)cam->x;
        int map_y = (int)cam->y;

        float delta_dist_x = (ray_dx == 0) ? 1e30f : fabsf(1.0f / ray_dx);
        float delta_dist_y = (ray_dy == 0) ? 1e30f : fabsf(1.0f / ray_dy);

        int step_x, step_y, side = 0, wall_id = 0, hit = 0;
        float side_dist_x, side_dist_y;

        if (ray_dx < 0) { step_x = -1; side_dist_x = (cam->x - map_x) * delta_dist_x; }
        else             { step_x = 1;  side_dist_x = (map_x + 1.0f - cam->x) * delta_dist_x; }
        if (ray_dy < 0) { step_y = -1; side_dist_y = (cam->y - map_y) * delta_dist_y; }
        else             { step_y = 1;  side_dist_y = (map_y + 1.0f - cam->y) * delta_dist_y; }

        for (int guard = 0; !hit && guard < 64; guard++) {
            if (side_dist_x < side_dist_y) {
                side_dist_x += delta_dist_x;
                map_x += step_x;
                side = 0;
            } else {
                side_dist_y += delta_dist_y;
                map_y += step_y;
                side = 1;
            }
            if (map_x < 0 || map_x >= MAP_W || map_y < 0 || map_y >= MAP_H) {
                wall_id = 1;
                hit = 1;
                break;
            }
            if (g_map[map_y][map_x] != 0) {
                wall_id = g_map[map_y][map_x];
                hit = 1;
            }
        }

        float perp_dist = (side == 0) ? (side_dist_x - delta_dist_x) : (side_dist_y - delta_dist_y);
        if (perp_dist < 0.05f) perp_dist = 0.05f;
        wall_dist_out[x] = perp_dist;

        // Fractional position along the wall face this ray struck (0..1),
        // used only for the brick texture below. Converted to fixed-point
        // once per column so the per-pixel loop stays float-free.
        float wall_x = (side == 0) ? (cam->y + perp_dist * ray_dy) : (cam->x + perp_dist * ray_dx);
        wall_x -= floorf(wall_x);
        int wall_x_fp = (int)(wall_x * BRICK_FP_ONE);

        int line_h = (int)(RENDER_H / perp_dist);
        if (line_h < 1) line_h = 1;
        int draw_start_raw = -line_h / 2 + RENDER_H / 2;
        int draw_start = draw_start_raw;
        int draw_end = line_h / 2 + RENDER_H / 2;
        if (draw_start < 0) draw_start = 0;
        if (draw_end >= RENDER_H) draw_end = RENDER_H - 1;

        uint16_t wc = wall_color(wall_id, side == 1);
        uint16_t wc_dark = darken(wc);

        for (int y = 0; y < draw_start; y++) fb[y * RENDER_W + x] = ceil_color;
        for (int y = draw_start; y <= draw_end; y++) {
            fb[y * RENDER_W + x] = apply_brick(wc, wc_dark, wall_x_fp, y, draw_start_raw, line_h);
        }
        for (int y = draw_end + 1; y < RENDER_H; y++) fb[y * RENDER_W + x] = floor_color;
    }
}
