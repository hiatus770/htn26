#include "kart_race.h"
#include "kart_net.h"
#include "hal_display.h"
#include "hal_led.h"
#include "raycaster.h" // reuse RENDER_W/RENDER_H, the shared low-res canvas size
#include <math.h>
#include <string.h>

#define RGB565(r, g, b) ((uint16_t)(((r) & 0x1F) << 11 | ((g) & 0x3F) << 5 | ((b) & 0x1F)))

// Track: a rounded-rectangle ring in the 160x120 render canvas.
#define TRACK_OX0 10.0f
#define TRACK_OY0 10.0f
#define TRACK_OX1 150.0f
#define TRACK_OY1 110.0f
#define TRACK_OR  25.0f
#define TRACK_IX0 45.0f
#define TRACK_IY0 35.0f
#define TRACK_IX1 115.0f
#define TRACK_IY1 85.0f
#define TRACK_IR  15.0f

// Checkpoint zones: A is the finish/start line (top straight), B is roughly
// opposite (bottom straight). A lap only counts on entering A after having
// passed through B, so reversing back over the line can't cheese a lap.
#define ZONE_A_X0 70
#define ZONE_A_X1 90
#define ZONE_A_Y0 10
#define ZONE_A_Y1 20
#define ZONE_B_X0 70
#define ZONE_B_X1 90
#define ZONE_B_Y0 100
#define ZONE_B_Y1 110

#define LAPS_TO_WIN 3

typedef struct {
    float x, y, heading, speed;
    uint8_t lap;
    bool finished;
    bool in_zone_a, in_zone_b, passed_b;
} kart_t;

static uint16_t s_fb[RENDER_W * RENDER_H];
static kart_t s_me;
static kart_peer_t s_peer;
static int s_net_tick;

static bool inside_rounded_rect(float px, float py, float x0, float y0, float x1, float y1, float r)
{
    float dx = (px < x0 + r) ? (x0 + r - px) : ((px > x1 - r) ? (px - (x1 - r)) : 0.0f);
    float dy = (py < y0 + r) ? (y0 + r - py) : ((py > y1 - r) ? (py - (y1 - r)) : 0.0f);
    return (dx * dx + dy * dy) <= r * r;
}

static bool is_track(float x, float y)
{
    if (!inside_rounded_rect(x, y, TRACK_OX0, TRACK_OY0, TRACK_OX1, TRACK_OY1, TRACK_OR)) return false;
    if (inside_rounded_rect(x, y, TRACK_IX0, TRACK_IY0, TRACK_IX1, TRACK_IY1, TRACK_IR)) return false;
    return true;
}

static bool in_zone(float x, float y, int x0, int y0, int x1, int y1)
{
    return x >= x0 && x <= x1 && y >= y0 && y <= y1;
}

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

static void reset_kart(kart_t *k, float x, float y, float heading)
{
    k->x = x;
    k->y = y;
    k->heading = heading;
    k->speed = 0;
    k->lap = 0;
    k->finished = false;
    k->in_zone_a = false;
    k->in_zone_b = false;
    k->passed_b = false;
}

void kart_race_reset(void)
{
    // Start on the finish line, facing along the top straight (not into the
    // hole in the middle of the ring).
    reset_kart(&s_me, 80.0f, 15.0f, 0.0f);
    memset(&s_peer, 0, sizeof(s_peer));
    s_net_tick = 0;
}

void kart_race_init(void)
{
    kart_net_init();
    kart_race_reset();
}

static void update_laps(kart_t *k)
{
    bool a = in_zone(k->x, k->y, ZONE_A_X0, ZONE_A_Y0, ZONE_A_X1, ZONE_A_Y1);
    bool b = in_zone(k->x, k->y, ZONE_B_X0, ZONE_B_Y0, ZONE_B_X1, ZONE_B_Y1);

    if (b && !k->in_zone_b) k->passed_b = true;
    if (a && !k->in_zone_a && k->passed_b) {
        k->lap++;
        k->passed_b = false;
        if (k->lap >= LAPS_TO_WIN) k->finished = true;
    }
    k->in_zone_a = a;
    k->in_zone_b = b;
}

static void draw_kart(float x, float y, float heading, uint16_t color)
{
    int cx = (int)x, cy = (int)y;
    fill_rect(cx - 2, cy - 2, 4, 4, color);
    int nx = cx + (int)(cosf(heading) * 5.0f);
    int ny = cy + (int)(sinf(heading) * 5.0f);
    fill_rect(nx - 1, ny - 1, 2, 2, RGB565(31, 63, 31));
}

static void update_leds(void)
{
    rgb_t colors[6];
    if (s_me.finished) {
        for (int i = 0; i < 6; i++) colors[i] = (rgb_t){ 0, 35, 0 };
    } else {
        int lit = s_me.lap + 1;
        if (lit > 6) lit = 6;
        for (int i = 0; i < 6; i++) {
            colors[i] = (i < lit) ? (rgb_t){ 0, 0, 35 } : (rgb_t){ 0, 0, 0 };
        }
    }
    hal_led_set(colors);
}

void kart_race_step(const button_state_t *btn)
{
    const float ACCEL = 0.12f;
    const float MAX_SPEED = 1.6f;
    const float FRICTION = 0.94f;
    const float TURN_RATE = 0.09f;

    if (button_pressed(btn, BTN_START)) {
        kart_race_reset();
    }

    if (!s_me.finished) {
        if (button_held(btn, BTN_UP)) {
            s_me.speed += ACCEL;
        } else if (button_held(btn, BTN_DOWN)) {
            s_me.speed -= ACCEL * 1.5f;
        } else {
            s_me.speed *= FRICTION;
        }
        if (s_me.speed > MAX_SPEED) s_me.speed = MAX_SPEED;
        if (s_me.speed < -MAX_SPEED * 0.5f) s_me.speed = -MAX_SPEED * 0.5f;

        if (fabsf(s_me.speed) > 0.05f) {
            float turn = 0;
            if (button_held(btn, BTN_LEFT)) turn -= TURN_RATE;
            if (button_held(btn, BTN_RIGHT)) turn += TURN_RATE;
            s_me.heading += turn * (s_me.speed > 0 ? 1.0f : -1.0f);
        }

        float nx = s_me.x + cosf(s_me.heading) * s_me.speed;
        float ny = s_me.y + sinf(s_me.heading) * s_me.speed;
        if (is_track(nx, ny)) {
            s_me.x = nx;
            s_me.y = ny;
        } else {
            s_me.speed *= 0.5f;
        }

        update_laps(&s_me);
    }

    kart_net_poll(&s_peer);
    s_net_tick++;
    if (s_net_tick >= 3) { // ~15Hz at this ~50Hz game loop
        s_net_tick = 0;
        kart_net_send(s_me.x, s_me.y, s_me.heading, s_me.lap, s_me.finished);
    }

    for (int y = 0; y < RENDER_H; y++) {
        for (int x = 0; x < RENDER_W; x++) {
            s_fb[y * RENDER_W + x] = is_track((float)x, (float)y) ? RGB565(18, 18, 18) : RGB565(4, 20, 6);
        }
    }
    fill_rect(ZONE_A_X0, ZONE_A_Y0, ZONE_A_X1 - ZONE_A_X0, ZONE_A_Y1 - ZONE_A_Y0, RGB565(28, 56, 28));

    if (s_peer.valid) {
        draw_kart(s_peer.x, s_peer.y, s_peer.heading, RGB565(28, 8, 8));
    }
    draw_kart(s_me.x, s_me.y, s_me.heading, RGB565(4, 20, 31));

    update_leds();
    hal_display_blit_scaled(s_fb, RENDER_W, RENDER_H, 2);
}
