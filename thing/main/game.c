#include "game.h"
#include "raycaster.h"
#include "map.h"
#include "hal_buttons.h"
#include "hal_display.h"
#include "hal_led.h"
#include "hal_wifi.h"
#include "kart_race.h"
#include "duel.h"
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define RGB565(r, g, b) ((uint16_t)(((r) & 0x1F) << 11 | ((g) & 0x3F) << 5 | ((b) & 0x1F)))
#define MAX_ENEMIES 6

typedef enum {
    APP_MODE_WIFI,
    APP_MODE_GAME,
    APP_MODE_RACE,
    APP_MODE_DUEL,
} app_mode_t;

static app_mode_t s_mode = APP_MODE_WIFI;

typedef struct {
    float x, y;
    bool alive;
    int health;
} enemy_t;

static camera_t s_cam;
static uint16_t s_fb[RENDER_W * RENDER_H];
static float s_wall_dist[RENDER_W];
static enemy_t s_enemies[MAX_ENEMIES];
static int s_player_health;
static int s_muzzle_flash_frames;

static const float MOVE_SPEED = 2.2f; // map cells / sec
static const float TURN_SPEED = 2.4f; // rad / sec
static const float DT = 1.0f / 20.0f; // fixed step, matches app_main's ~20Hz loop

static bool is_wall(int mx, int my)
{
    if (mx < 0 || mx >= MAP_W || my < 0 || my >= MAP_H) return true;
    return g_map[my][mx] != 0;
}

// Spawn points are hand-picked open floor cells in g_map (see map.c).
static void spawn_enemies(void)
{
    static const float spots[MAX_ENEMIES][2] = {
        {3, 1}, {9, 1}, {7, 7}, {1, 10}, {9, 13}, {12, 3},
    };
    for (int i = 0; i < MAX_ENEMIES; i++) {
        s_enemies[i].x = spots[i][0] + 0.5f;
        s_enemies[i].y = spots[i][1] + 0.5f;
        s_enemies[i].alive = true;
        s_enemies[i].health = 2;
    }
}

static void reset_game(void)
{
    s_cam.x = 8.5f;
    s_cam.y = 14.5f;
    s_cam.dir_x = 0;
    s_cam.dir_y = -1;
    s_cam.plane_x = 0.66f;
    s_cam.plane_y = 0;
    s_player_health = 100;
    s_muzzle_flash_frames = 0;
    spawn_enemies();
}

void game_init(void)
{
    hal_buttons_init();
    hal_display_init();
    hal_led_init();
    kart_race_init(); // must run after WiFi is up (app_main calls hal_wifi_init() first)
    duel_init();
    reset_game();
}

static void try_move(float nx, float ny)
{
    if (!is_wall((int)nx, (int)s_cam.y)) s_cam.x = nx;
    if (!is_wall((int)s_cam.x, (int)ny)) s_cam.y = ny;
}

static int count_alive(void)
{
    int n = 0;
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (s_enemies[i].alive) n++;
    }
    return n;
}

static void fire_weapon(void)
{
    s_muzzle_flash_frames = 3;

    // Hit the nearest alive enemy within an ~30 degree forward cone and with
    // a clear (coarse, cell-stepped) line of sight.
    int best = -1;
    float best_dist = 1e9f;
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!s_enemies[i].alive) continue;

        float dx = s_enemies[i].x - s_cam.x;
        float dy = s_enemies[i].y - s_cam.y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > 8.0f || dist < 0.01f) continue;

        float ex = dx / dist, ey = dy / dist;
        float dot = ex * s_cam.dir_x + ey * s_cam.dir_y;
        if (dot < 0.85f) continue;

        bool blocked = false;
        int steps = (int)(dist * 4.0f) + 1;
        for (int s = 1; s < steps; s++) {
            float t = (float)s / steps;
            if (is_wall((int)(s_cam.x + dx * t), (int)(s_cam.y + dy * t))) {
                blocked = true;
                break;
            }
        }
        if (blocked) continue;

        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }

    if (best >= 0) {
        s_enemies[best].health--;
        if (s_enemies[best].health <= 0) s_enemies[best].alive = false;
    }
}

static void update_enemies(void)
{
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!s_enemies[i].alive) continue;

        float dx = s_cam.x - s_enemies[i].x;
        float dy = s_cam.y - s_enemies[i].y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist > 0.6f && dist < 6.0f) {
            float speed = 0.6f * DT;
            float nx = s_enemies[i].x + dx / dist * speed;
            float ny = s_enemies[i].y + dy / dist * speed;
            if (!is_wall((int)nx, (int)s_enemies[i].y)) s_enemies[i].x = nx;
            if (!is_wall((int)s_enemies[i].x, (int)ny)) s_enemies[i].y = ny;
        }
        if (dist < 0.6f) {
            s_player_health -= 1;
        }
    }
}

static void render_sprites(void)
{
    for (int i = 0; i < MAX_ENEMIES; i++) {
        if (!s_enemies[i].alive) continue;

        float rel_x = s_enemies[i].x - s_cam.x;
        float rel_y = s_enemies[i].y - s_cam.y;

        float inv_det = 1.0f / (s_cam.plane_x * s_cam.dir_y - s_cam.dir_x * s_cam.plane_y);
        float transform_x = inv_det * (s_cam.dir_y * rel_x - s_cam.dir_x * rel_y);
        float transform_y = inv_det * (-s_cam.plane_y * rel_x + s_cam.plane_x * rel_y); // depth

        if (transform_y <= 0.1f) continue; // behind camera

        int sprite_screen_x = (int)((RENDER_W / 2) * (1 + transform_x / transform_y));
        int sprite_size = abs((int)(RENDER_H / transform_y));

        int draw_start_y = -sprite_size / 2 + RENDER_H / 2;
        int draw_end_y = sprite_size / 2 + RENDER_H / 2;
        if (draw_start_y < 0) draw_start_y = 0;
        if (draw_end_y >= RENDER_H) draw_end_y = RENDER_H - 1;

        int draw_start_x = sprite_screen_x - sprite_size / 2;
        int draw_end_x = sprite_screen_x + sprite_size / 2;

        uint16_t body_color = (s_enemies[i].health > 1) ? (uint16_t)0xF800 : (uint16_t)0xFBE0;

        for (int sx = draw_start_x; sx < draw_end_x; sx++) {
            if (sx < 0 || sx >= RENDER_W) continue;
            if (transform_y >= s_wall_dist[sx]) continue; // occluded by a nearer wall

            float u = (float)(sx - draw_start_x) / (float)(sprite_size > 0 ? sprite_size : 1);
            float edge = fabsf(u - 0.5f) * 2.0f;

            for (int sy = draw_start_y; sy <= draw_end_y; sy++) {
                float v = (float)(sy - draw_start_y) / (float)((draw_end_y - draw_start_y) + 1);
                float vedge = fabsf(v - 0.5f) * 2.0f;
                if (edge * edge + vedge * vedge > 1.0f) continue; // ellipse silhouette
                s_fb[sy * RENDER_W + sx] = body_color;
            }
        }
    }
}

static void update_leds(void)
{
    rgb_t colors[6];
    if (s_muzzle_flash_frames > 0) {
        for (int i = 0; i < 6; i++) colors[i] = (rgb_t){40, 40, 40};
    } else {
        int alive = count_alive();
        for (int i = 0; i < 6; i++) {
            colors[i] = (i < alive) ? (rgb_t){20, 0, 0} : (rgb_t){0, 15, 0};
        }
    }
    hal_led_set(colors);
}

// Bounds-checked solid-rectangle blit into the shared framebuffer. Used for
// the WiFi status screen instead of text: there is no font-rendering code
// in this firmware, and hand-authoring one isn't worth the risk of subtly
// garbled glyphs with no way to visually test it on the real display.
static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    for (int yy = y; yy < y + h; yy++) {
        if (yy < 0 || yy >= RENDER_H) continue;
        for (int xx = x; xx < x + w; xx++) {
            if (xx < 0 || xx >= RENDER_W) continue;
            s_fb[yy * RENDER_W + xx] = color;
        }
    }
}

static void render_wifi_status(void)
{
    memset(s_fb, 0, sizeof(s_fb));

    wifi_status_t st;
    hal_wifi_get_status(&st);

    uint16_t status_color;
    rgb_t led_color;
    if (st.state == WIFI_STATE_CONNECTED) {
        status_color = RGB565(0, 60, 12);
        led_color = (rgb_t){ 0, 30, 0 };
    } else if (st.state == WIFI_STATE_FAILED) {
        status_color = RGB565(28, 4, 4);
        led_color = (rgb_t){ 30, 0, 0 };
    } else {
        status_color = RGB565(31, 40, 0);
        led_color = (rgb_t){ 20, 15, 0 };
    }

    // Big centered status swatch.
    fill_rect(55, 30, 50, 40, status_color);

    // Signal-strength bars, tallest on the right, filled count scales with
    // RSSI. Only meaningful once connected; all unlit otherwise.
    int bars = 0;
    if (st.state == WIFI_STATE_CONNECTED) {
        if (st.rssi >= -55) bars = 5;
        else if (st.rssi >= -65) bars = 4;
        else if (st.rssi >= -75) bars = 3;
        else if (st.rssi >= -85) bars = 2;
        else bars = 1;
    }
    for (int i = 0; i < 5; i++) {
        int bar_h = 6 + i * 5;
        uint16_t c = (i < bars) ? RGB565(0, 60, 15) : RGB565(6, 6, 7);
        fill_rect(60 + i * 12, 90 - bar_h, 8, bar_h, c);
    }

    rgb_t colors[6];
    for (int i = 0; i < 6; i++) colors[i] = led_color;
    // LED 5 lit white = turn direction is currently inverted on this badge
    // (Down toggles it -- see game_step's WiFi-mode dispatch).
    if (hal_buttons_turn_sign() < 0) colors[5] = (rgb_t){ 40, 40, 40 };
    hal_led_set(colors);

    hal_display_blit_scaled(s_fb, RENDER_W, RENDER_H, 2);
}

void game_step(void)
{
    button_state_t btn = hal_buttons_poll();

    if (s_mode == APP_MODE_WIFI) {
        if (button_pressed(&btn, BTN_A)) {
            s_mode = APP_MODE_GAME;
        } else if (button_pressed(&btn, BTN_B)) {
            s_mode = APP_MODE_RACE;
            kart_race_reset();
        } else if (button_pressed(&btn, BTN_UP)) {
            s_mode = APP_MODE_DUEL;
            duel_reset();
        } else if (button_pressed(&btn, BTN_DOWN)) {
            hal_buttons_toggle_turn_invert();
        }
        render_wifi_status();
        return;
    }

    if (s_mode == APP_MODE_RACE) {
        if (button_pressed(&btn, BTN_HOME)) {
            s_mode = APP_MODE_WIFI;
            return;
        }
        kart_race_step(&btn);
        return;
    }

    if (s_mode == APP_MODE_DUEL) {
        if (button_pressed(&btn, BTN_HOME)) {
            s_mode = APP_MODE_WIFI;
            return;
        }
        duel_step(&btn);
        return;
    }

    if (button_pressed(&btn, BTN_B)) {
        s_mode = APP_MODE_WIFI;
        return;
    }

    if (s_player_health <= 0 || button_pressed(&btn, BTN_START)) {
        reset_game();
    }

    // Aux1 used to gate strafe-vs-turn here, but it's a maintained toggle
    // switch (not momentary) per the badge's hardware docs -- whichever
    // physical position it's left in would permanently force one mode or
    // the other. Left/Right just turns now; no button-strafe.
    float move = 0, turn = 0;
    if (button_held(&btn, BTN_UP)) move += 1;
    if (button_held(&btn, BTN_DOWN)) move -= 1;
    if (button_held(&btn, BTN_LEFT)) turn -= 1;
    if (button_held(&btn, BTN_RIGHT)) turn += 1;
    turn *= hal_buttons_turn_sign();
    if (button_pressed(&btn, BTN_A)) fire_weapon();

    if (turn != 0) {
        float angle = turn * TURN_SPEED * DT;
        float ca = cosf(angle), sa = sinf(angle);
        float odx = s_cam.dir_x, opx = s_cam.plane_x;
        s_cam.dir_x = s_cam.dir_x * ca - s_cam.dir_y * sa;
        s_cam.dir_y = odx * sa + s_cam.dir_y * ca;
        s_cam.plane_x = s_cam.plane_x * ca - s_cam.plane_y * sa;
        s_cam.plane_y = opx * sa + s_cam.plane_y * ca;
    }
    if (move != 0) {
        try_move(s_cam.x + s_cam.dir_x * move * MOVE_SPEED * DT,
                 s_cam.y + s_cam.dir_y * move * MOVE_SPEED * DT);
    }

    update_enemies();
    if (s_muzzle_flash_frames > 0) s_muzzle_flash_frames--;

    raycast_render(s_fb, &s_cam, s_wall_dist);
    render_sprites();
    update_leds();

    hal_display_blit_scaled(s_fb, RENDER_W, RENDER_H, 2);
}
