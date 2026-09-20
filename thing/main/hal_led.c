#include "hal_led.h"
#include "driver/rmt_tx.h"

#define LED_GPIO 3
#define LED_COUNT 6
#define RMT_RESOLUTION_HZ 10000000 // 10 MHz -> 100 ns/tick

static rmt_channel_handle_t s_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;

void hal_led_init(void)
{
    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&chan_cfg, &s_chan));

    // WS2812 bit timings at 100ns/tick: 0-bit ~0.4us/0.9us, 1-bit ~0.8us/0.5us.
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .level0 = 1, .duration0 = 4, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 8, .level1 = 0, .duration1 = 5 },
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_cfg, &s_encoder));

    ESP_ERROR_CHECK(rmt_enable(s_chan));
}

// We only push updates once per game frame (tens of ms apart), and the RMT
// TX line idles low between transmits, so WS2812's >50us latch requirement
// is satisfied without an explicit reset symbol.
void hal_led_set(const rgb_t colors[6])
{
    uint8_t grb[LED_COUNT * 3];
    for (int i = 0; i < LED_COUNT; i++) {
        grb[i * 3 + 0] = colors[i].g;
        grb[i * 3 + 1] = colors[i].r;
        grb[i * 3 + 2] = colors[i].b;
    }

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(s_chan, s_encoder, grb, sizeof(grb), &tx_cfg));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_chan, 100));
}
