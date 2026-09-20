#include "duel.h"
#include "duel_net.h"
#include "raycaster.h"
#include "map.h"
#include "hal_display.h"
#include "hal_led.h"
#include "hal_accel.h"
#include "sprite_doomguy.h"
#include "sprite_pistol.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define RGB565(r, g, b) ((uint16_t)(((r) & 0x1F) << 11 | ((g) & 0x3F) << 5 | ((b) & 0x1F)))

#define MAX_HP 100
#define DAMAGE_PER_HIT 20
#define HIT_RANGE 6.0f
#define HIT_CONE_COS 0.97f // ~14 degree half-angle aim cone -- tightened, was too forgiving at ~21 degrees

static const float MOVE_SPEED = 2.2f;
static const float TURN_SPEED = 2.4f;
static const float DT = 1.0f / 20.0f;

// Two spawn points on opposite sides of the maze, so the two badges never
// start on top of each other. Which one we use is decided the first time we
// hear from the peer (see maybe_assign_spawn below); until then we default
// to spawn 0.
typedef struct { float x, y, dir_x, dir_y; } spawn_t;
static const spawn_t SPAWNS[2] = {
    { 8.5f, 14.5f, 0.0f, -1.0f }, // near the south entrance, facing north
    { 8.5f, 1.5f, 0.0f, 1.0f },   // near the north wall, facing south
};

static camera_t s_cam;
static uint16_t s_fb[RENDER_W * RENDER_H];
static float s_wall_dist[RENDER_W];

static int s_hp;
static bool s_alive;
static uint32_t s_shot_seq;
static int s_net_tick;
static int s_muzzle_flash_frames;
static int s_gun_kick_frames;
static int s_hit_flash_frames;
static int s_screen_flash_frames;
static int16_t s_prev_peer_hp;
static bool s_prev_peer_alive;
static float s_last_tilt_x;

static duel_peer_t s_peer;
static uint8_t s_my_id[3];
static int s_spawn_index;
static bool s_spawn_decided;

// Session score -- persists across deaths/respawns within Duel mode, only
// reset when the firmware reboots (not by duel_reset()).
static int s_kills;
static int s_deaths;

static bool is_wall(int mx, int my)
{
    if (mx < 0 || mx >= MAP_W || my < 0 || my >= MAP_H) return true;
    return g_map[my][mx] != 0;
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

static void try_move(float nx, float ny)
{
    if (!is_wall((int)nx, (int)s_cam.y)) s_cam.x = nx;
    if (!is_wall((int)s_cam.x, (int)ny)) s_cam.y = ny;
}

void duel_reset(void)
{
    const spawn_t *sp = &SPAWNS[s_spawn_index];
    s_cam.x = sp->x;
    s_cam.y = sp->y;
    s_cam.dir_x = sp->dir_x;
    s_cam.dir_y = sp->dir_y;
    s_cam.plane_x = 0.66f;
    s_cam.plane_y = 0;
    s_hp = MAX_HP;
    s_alive = true;
    s_muzzle_flash_frames = 0;
    s_gun_kick_frames = 0;
    s_hit_flash_frames = 0;
    s_screen_flash_frames = 0;
    s_prev_peer_hp = MAX_HP;
}

// The first time we hear from the peer, deterministically pick opposite
// spawns by comparing badge ids -- both sides run this same comparison and
// agree on the same outcome without needing to negotiate over the network.
static void maybe_assign_spawn(void)
{
    if (s_spawn_decided || !s_peer.valid) return;
    s_spawn_decided = true;
    int cmp = memcmp(s_my_id, s_peer.badge_id, 3);
    int new_index = (cmp > 0) ? 1 : 0;
    if (new_index != s_spawn_index) {
        s_spawn_index = new_index;
        duel_reset();
    }
}

void duel_init(void)
{
    hal_accel_init();
    duel_net_init();
    duel_net_get_my_id(s_my_id);
    s_shot_seq = 0;
    s_net_tick = 0;
    s_last_tilt_x = 0;
    s_spawn_index = 0;
    s_spawn_decided = false;
    s_prev_peer_alive = true;
    s_kills = 0;
    s_deaths = 0;
    memset(&s_peer, 0, sizeof(s_peer));
    duel_reset();
}

static bool line_of_sight_clear(float x0, float y0, float x1, float y1)
{
    float dx = x1 - x0, dy = y1 - y0;
    float dist = sqrtf(dx * dx + dy * dy);
    int steps = (int)(dist * 4.0f) + 1;
    for (int s = 1; s < steps; s++) {
        float t = (float)s / steps;
        if (is_wall((int)(x0 + dx * t), (int)(y0 + dy * t))) return false;
    }
    return true;
}

// Each badge decides for itself whether it just got hit, checking its own
// (always current) position against the peer's broadcast fire origin/aim.
// This avoids trusting a remote badge's opinion about our position, and
// sidesteps the staleness of the peer's own position in our last packet.
static void apply_incoming_fire(void)
{
    float dx = s_cam.x - s_peer.x, dy = s_cam.y - s_peer.y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist < 0.05f || dist > HIT_RANGE) return;

    float ux = dx / dist, uy = dy / dist;
    float dot = ux * s_peer.dir_x + uy * s_peer.dir_y;
    if (dot < HIT_CONE_COS) return;

    if (!line_of_sight_clear(s_peer.x, s_peer.y, s_cam.x, s_cam.y)) return;

    s_hp -= DAMAGE_PER_HIT;
    s_screen_flash_frames = 5;
    if (s_hp <= 0) {
        s_hp = 0;
        s_alive = false;
        s_deaths++;
    }
}

static void draw_gun(void)
{
    int base_x = (RENDER_W - PISTOL_W) / 2;
    int base_y = RENDER_H - PISTOL_H;
    if (s_gun_kick_frames > 0) base_y -= 4;

    for (int ty = 0; ty < PISTOL_H; ty++) {
        int dy = base_y + ty;
        if (dy < 0 || dy >= RENDER_H) continue;
        for (int tx = 0; tx < PISTOL_W; tx++) {
            int dx = base_x + tx;
            if (dx < 0 || dx >= RENDER_W) continue;
            int idx = ty * PISTOL_W + tx;
            if (!((pistol_alpha[idx / 8] >> (idx % 8)) & 1)) continue;
            s_fb[dy * RENDER_W + dx] = pistol_rgb565[idx];
        }
    }

    if (s_muzzle_flash_frames > 0) {
        // Small circle centered exactly on the crosshair.
        int cx = RENDER_W / 2, cy = RENDER_H / 2;
        const int r = 4;
        for (int y = cy - r; y <= cy + r; y++) {
            if (y < 0 || y >= RENDER_H) continue;
            for (int x = cx - r; x <= cx + r; x++) {
                if (x < 0 || x >= RENDER_W) continue;
                int ddx = x - cx, ddy = y - cy;
                if (ddx * ddx + ddy * ddy > r * r) continue;
                s_fb[y * RENDER_W + x] = 0xFFF0;
            }
        }
    }
}

static void draw_opponent(void)
{
    if (!s_peer.valid || !s_peer.alive) return;

    float rel_x = s_peer.x - s_cam.x;
    float rel_y = s_peer.y - s_cam.y;

    float inv_det = 1.0f / (s_cam.plane_x * s_cam.dir_y - s_cam.dir_x * s_cam.plane_y);
    float transform_x = inv_det * (s_cam.dir_y * rel_x - s_cam.dir_x * rel_y);
    float transform_y = inv_det * (-s_cam.plane_y * rel_x + s_cam.plane_x * rel_y);

    if (transform_y <= 0.1f) return; // behind camera

    int screen_x = (int)((RENDER_W / 2) * (1 + transform_x / transform_y));
    int sprite_h = abs((int)(RENDER_H / transform_y));
    int sprite_w = sprite_h * DOOMGUY_W / DOOMGUY_H;
    if (sprite_w < 1) sprite_w = 1;

    int draw_start_y = -sprite_h / 2 + RENDER_H / 2;
    int draw_end_y = sprite_h / 2 + RENDER_H / 2;
    int draw_start_x = screen_x - sprite_w / 2;
    int draw_end_x = screen_x + sprite_w / 2;

    for (int sx = draw_start_x; sx < draw_end_x; sx++) {
        if (sx < 0 || sx >= RENDER_W) continue;
        if (transform_y >= s_wall_dist[sx]) continue; // occluded by a nearer wall

        int tex_x = (sx - draw_start_x) * DOOMGUY_W / (sprite_w > 0 ? sprite_w : 1);
        if (tex_x < 0) tex_x = 0;
        if (tex_x >= DOOMGUY_W) tex_x = DOOMGUY_W - 1;

        for (int sy = draw_start_y; sy <= draw_end_y; sy++) {
            if (sy < 0 || sy >= RENDER_H) continue;

            int tex_y = (sy - draw_start_y) * DOOMGUY_H / (sprite_h > 0 ? sprite_h : 1);
            if (tex_y < 0 || tex_y >= DOOMGUY_H) continue;

            int idx = tex_y * DOOMGUY_W + tex_x;
            if (!((doomguy_alpha[idx / 8] >> (idx % 8)) & 1)) continue;

            // Brief white tint over the whole silhouette right after landing
            // a hit, so it reads clearly even at small on-screen size.
            s_fb[sy * RENDER_W + sx] = (s_hit_flash_frames > 0) ? 0xFFFF : doomguy_rgb565[idx];
        }
    }
}

static void draw_crosshair(void)
{
    const uint16_t color = 0xFFFF;
    int cx = RENDER_W / 2, cy = RENDER_H / 2;
    for (int x = cx - 6; x <= cx - 2; x++) s_fb[cy * RENDER_W + x] = color;
    for (int x = cx + 2; x <= cx + 6; x++) s_fb[cy * RENDER_W + x] = color;
    for (int y = cy - 6; y <= cy - 2; y++) s_fb[y * RENDER_W + cx] = color;
    for (int y = cy + 2; y <= cy + 6; y++) s_fb[y * RENDER_W + cx] = color;
}

// Kill/death pip HUD, top-left/top-right, 5 pips each (caps display at 5;
// the underlying counts have no cap). No font in this firmware, so counts
// are shown as filled-vs-dim squares rather than digits.
static void draw_score(void)
{
    int kills_shown = s_kills > 5 ? 5 : s_kills;
    for (int i = 0; i < 5; i++) {
        uint16_t c = (i < kills_shown) ? RGB565(0, 60, 10) : RGB565(6, 8, 6);
        fill_rect(4 + i * 9, 4, 7, 7, c);
    }
    int deaths_shown = s_deaths > 5 ? 5 : s_deaths;
    for (int i = 0; i < 5; i++) {
        uint16_t c = (i < deaths_shown) ? RGB565(28, 4, 4) : RGB565(8, 6, 6);
        fill_rect(RENDER_W - 4 - 7 - i * 9, 4, 7, 7, c);
    }
}

// Blends the whole frame toward white when we just took a hit. 50% keeps
// the scene clearly visible underneath while still reading as a hit flash.
static void apply_screen_flash(void)
{
    if (s_screen_flash_frames <= 0) return;
    for (int i = 0; i < RENDER_W * RENDER_H; i++) {
        uint16_t c = s_fb[i];
        uint8_t r = (c >> 11) & 0x1F, g = (c >> 5) & 0x3F, b = c & 0x1F;
        r = r + (31 - r) / 2;
        g = g + (63 - g) / 2;
        b = b + (31 - b) / 2;
        s_fb[i] = RGB565(r, g, b);
    }
}

static void update_leds(void)
{
    rgb_t colors[6];
    if (!s_alive) {
        for (int i = 0; i < 6; i++) colors[i] = (rgb_t){ 25, 0, 0 };
    } else if (s_muzzle_flash_frames > 0) {
        for (int i = 0; i < 6; i++) colors[i] = (rgb_t){ 30, 30, 30 };
    } else {
        int lit = (s_hp * 6 + MAX_HP - 1) / MAX_HP; // ceil
        if (lit > 6) lit = 6;
        if (lit < 0) lit = 0;
        for (int i = 0; i < 6; i++) colors[i] = (i < lit) ? (rgb_t){ 0, 20, 0 } : (rgb_t){ 20, 0, 0 };
    }
    hal_led_set(colors);
}

void duel_step(const button_state_t *btn)
{
    if (button_pressed(btn, BTN_START)) {
        duel_reset();
    }

    bool peer_fired = false;
    duel_net_poll(&s_peer, &peer_fired);
    maybe_assign_spawn();
    if (peer_fired && s_alive) {
        apply_incoming_fire();
    }
    if (s_peer.valid) {
        if (s_peer.hp < s_prev_peer_hp) s_hit_flash_frames = 5;
        s_prev_peer_hp = s_peer.hp;
        // The opponent has no source of damage other than our own shots in
        // this 1v1, so a live->dead transition on their side credits us.
        if (s_prev_peer_alive && !s_peer.alive) s_kills++;
        s_prev_peer_alive = s_peer.alive;
    }

    if (s_alive) {
        // Aux1 used to gate strafe-vs-turn here, but it's a maintained
        // toggle switch (not momentary) per the badge's hardware docs --
        // whichever physical position it's left in would permanently force
        // one mode or the other. Left/Right just turns now; strafing is
        // tilt-only.
        float move = 0, turn = 0;
        if (button_held(btn, BTN_UP)) move += 1;
        if (button_held(btn, BTN_DOWN)) move -= 1;
        if (button_held(btn, BTN_LEFT)) turn -= 1;
        if (button_held(btn, BTN_RIGHT)) turn += 1;
        turn *= hal_buttons_turn_sign();

        // Tilt drives strafing continuously. Sign confirmed against real hardware.
        float tx, ty, tz;
        if (hal_accel_read(&tx, &ty, &tz)) s_last_tilt_x = -tx;
        const float DEADZONE = 150.0f;
        const float TILT_RANGE = 450.0f;
        float tilt = s_last_tilt_x;
        if (tilt > DEADZONE) tilt -= DEADZONE;
        else if (tilt < -DEADZONE) tilt += DEADZONE;
        else tilt = 0;
        tilt /= TILT_RANGE;
        if (tilt > 1.0f) tilt = 1.0f;
        if (tilt < -1.0f) tilt = -1.0f;
        float strafe = tilt;

        if (button_pressed(btn, BTN_A)) {
            s_shot_seq++;
            s_muzzle_flash_frames = 3;
            s_gun_kick_frames = 4;
        }

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
        if (strafe != 0) {
            try_move(s_cam.x + s_cam.plane_x * strafe * MOVE_SPEED * DT,
                     s_cam.y + s_cam.plane_y * strafe * MOVE_SPEED * DT);
        }
    }

    if (s_muzzle_flash_frames > 0) s_muzzle_flash_frames--;
    if (s_gun_kick_frames > 0) s_gun_kick_frames--;
    if (s_hit_flash_frames > 0) s_hit_flash_frames--;
    if (s_screen_flash_frames > 0) s_screen_flash_frames--;

    s_net_tick++;
    if (s_net_tick >= 3) { // ~15Hz at this ~50Hz game loop
        s_net_tick = 0;
        duel_net_send(s_cam.x, s_cam.y, s_cam.dir_x, s_cam.dir_y, (int16_t)s_hp, s_alive, s_shot_seq);
    }

    raycast_render(s_fb, &s_cam, s_wall_dist);
    draw_opponent();
    if (s_alive) draw_gun(); // no weapon to hold once dead
    draw_crosshair();
    draw_score();
    // Brief flash on a non-lethal hit; a sustained one while dead, until respawn.
    if (!s_alive) s_screen_flash_frames = 1;
    apply_screen_flash(); // last, so it tints everything drawn above
    update_leds();

    hal_display_blit_scaled(s_fb, RENDER_W, RENDER_H, 2);
}
