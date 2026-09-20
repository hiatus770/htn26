#include "hal_display.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"

// Pin map + init sequence from custom-firmware-hal.md section 4.
#define PIN_MOSI 10
#define PIN_SCLK 1
#define PIN_CS   2
#define PIN_DC   0
#define PIN_RST  4
#define LCD_HOST SPI2_HOST

// Source rows blitted per SPI transaction. At scale=2 this is a 320x20
// stripe buffer (~12.5KB), matching the "two ~30-row DMA stripes" guidance.
#define STRIPE_SRC_ROWS 10

static esp_lcd_panel_handle_t s_panel = NULL;
static esp_lcd_panel_io_handle_t s_io = NULL;
static SemaphoreHandle_t s_trans_done;

static bool on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                 esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_trans_done, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

void hal_display_init(void)
{
    s_trans_done = xSemaphoreCreateBinary();
    xSemaphoreGive(s_trans_done); // ready for the first blit

    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_SCLK,
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_W * STRIPE_SRC_ROWS * 2 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_DC,
        .cs_gpio_num = PIN_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 4,
        .on_color_trans_done = on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &s_io));

    // Three independent settings this panel needs, confirmed empirically
    // against real hardware via a manually-steppable diagnostic (combo 5 =
    // invert on, RGB order, little-endian data): rgb_ele_order is RGB565
    // channel order (R/B swap) -- this panel is RGB-native, not BGR as
    // originally guessed; data_endian is byte order for the 16-bit pixel
    // words (this panel expects LSB-first, matching the ESP32's native
    // little-endian uint16_t buffer -- the previous unset default assumed
    // big-endian and silently byte-swapped every pixel); invert_color is a
    // separate luminance-inversion command.
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_RST,
        .bits_per_pixel = 16,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(s_io, &panel_config, &s_panel));

    esp_lcd_panel_reset(s_panel);
    esp_lcd_panel_init(s_panel);
    // swap_xy/mirror are the fix if the image comes up rotated/mirrored, per
    // the HAL guide's bring-up notes.
    esp_lcd_panel_invert_color(s_panel, true);
    esp_lcd_panel_swap_xy(s_panel, true);
    esp_lcd_panel_mirror(s_panel, true, false);
    esp_lcd_panel_disp_on_off(s_panel, true);
}

void hal_display_blit_scaled(const uint16_t *src, int src_w, int src_h, int scale)
{
    static uint16_t stripe[DISP_W * STRIPE_SRC_ROWS * 2];
    int out_w = src_w * scale;

    for (int sy0 = 0; sy0 < src_h; sy0 += STRIPE_SRC_ROWS) {
        int rows = (sy0 + STRIPE_SRC_ROWS <= src_h) ? STRIPE_SRC_ROWS : (src_h - sy0);

        // Wait for the previous async transfer to finish before we touch the
        // (single, reused) stripe buffer again.
        xSemaphoreTake(s_trans_done, portMAX_DELAY);

        for (int ry = 0; ry < rows; ry++) {
            const uint16_t *srow = src + (size_t)(sy0 + ry) * src_w;
            for (int oy = 0; oy < scale; oy++) {
                uint16_t *drow = stripe + (size_t)(ry * scale + oy) * out_w;
                for (int x = 0; x < src_w; x++) {
                    uint16_t px = srow[x];
                    for (int ox = 0; ox < scale; ox++) {
                        drow[x * scale + ox] = px;
                    }
                }
            }
        }

        int y0 = sy0 * scale;
        int y1 = y0 + rows * scale;
        esp_lcd_panel_draw_bitmap(s_panel, 0, y0, out_w, y1, stripe);
    }
}
