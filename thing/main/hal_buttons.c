#include "hal_buttons.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// Pin map from custom-firmware-hal.md.
#define PIN_HC165_DATA 7
#define PIN_HC165_LOAD 20
#define PIN_HC165_CLK  21
#define PIN_START      9 // strapping pin: held low at reset also enters download mode

static uint16_t s_prev_held = 0;

void hal_buttons_init(void)
{
    gpio_config_t inputs = {
        .pin_bit_mask = (1ULL << PIN_HC165_DATA) | (1ULL << PIN_START),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&inputs);

    gpio_config_t outputs = {
        .pin_bit_mask = (1ULL << PIN_HC165_LOAD) | (1ULL << PIN_HC165_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&outputs);

    gpio_set_level(PIN_HC165_CLK, 0);
    gpio_set_level(PIN_HC165_LOAD, 1);
}

// Latches the 8 parallel inputs and shifts them out, bit0 = A ... bit7 = Aux1.
// A sampled 0 on DATA means pressed (active-low), so we invert into the mask.
static uint8_t read_hc165(void)
{
    uint8_t bits = 0;

    gpio_set_level(PIN_HC165_LOAD, 0);
    esp_rom_delay_us(2);
    gpio_set_level(PIN_HC165_LOAD, 1);
    esp_rom_delay_us(2);

    for (int i = 0; i < 8; i++) {
        if (gpio_get_level(PIN_HC165_DATA) == 0) {
            bits |= (1 << i);
        }
        gpio_set_level(PIN_HC165_CLK, 1);
        esp_rom_delay_us(2);
        gpio_set_level(PIN_HC165_CLK, 0);
        esp_rom_delay_us(2);
    }
    return bits;
}

button_state_t hal_buttons_poll(void)
{
    uint16_t held = read_hc165();
    if (gpio_get_level(PIN_START) == 0) {
        held |= (1 << BTN_START);
    }

    button_state_t s = {
        .held = held,
        .pressed = held & (uint16_t)~s_prev_held,
    };
    s_prev_held = held;
    return s;
}

// Manually toggled (see BTN_DOWN on the WiFi screen), not auto-detected: a
// MAC-based per-unit guess turned out wrong, and there's no way to inspect
// this badge's physical button wiring remotely. Defaults to not inverted;
// persists only for this boot session.
static bool s_turn_inverted = false;

void hal_buttons_toggle_turn_invert(void)
{
    s_turn_inverted = !s_turn_inverted;
}

float hal_buttons_turn_sign(void)
{
    return s_turn_inverted ? -1.0f : 1.0f;
}
