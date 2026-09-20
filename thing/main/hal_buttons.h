#pragma once
#include <stdint.h>
#include <stdbool.h>

// Order matches the 74HC165 shift order in the badge HAL guide: A shifts out
// first, Aux1 last. Start is a separate dedicated GPIO, folded into the same
// bitmask for convenience.
typedef enum {
    BTN_A = 0,
    BTN_B,
    BTN_HOME,
    BTN_DOWN,
    BTN_LEFT,
    BTN_RIGHT,
    BTN_UP,
    BTN_AUX1,
    BTN_START,
    BTN_COUNT
} button_id_t;

typedef struct {
    uint16_t held;    // bit i set = button i currently held down
    uint16_t pressed; // bit i set = button i transitioned pressed this poll
} button_state_t;

void hal_buttons_init(void);

// Bit-bangs the 74HC165 + reads the Start GPIO, edge-detects against the
// previous poll. Call this once per game frame (~10-50ms cadence).
button_state_t hal_buttons_poll(void);

// Some physical badge units have Left/Right wired differently than others,
// making rotation feel backwards. Toggle this (see BTN_DOWN on the WiFi
// screen) on whichever badge needs it; multiply hal_buttons_turn_sign()
// into turn/rotation direction to apply the correction.
void hal_buttons_toggle_turn_invert(void);
float hal_buttons_turn_sign(void);

static inline bool button_held(const button_state_t *s, button_id_t b) {
    return (s->held >> b) & 1;
}
static inline bool button_pressed(const button_state_t *s, button_id_t b) {
    return (s->pressed >> b) & 1;
}
