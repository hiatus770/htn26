#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_rom_sys.h"

static const char *TAG = "badge_bridge";

#define LCD_MOSI 10
#define LCD_CLK 1
#define LCD_CS 2
#define LCD_DC 0
#define LCD_RST 4
#define BUTTON_DATA 7
#define BUTTON_LOAD 20
#define BUTTON_CLK 21
#define START_BUTTON 9
#define FRAME_MAGIC_VERSION 1
#define FRAME_KIND 1
#define INPUT_KIND 2
#define WIFI_CONNECTED (1U << 0)
#define INPUT_QUEUE_LEN 32

typedef struct {
    uint8_t id;
    bool pressed;
} button_event_t;

static EventGroupHandle_t wifi_events;
static SemaphoreHandle_t socket_lock;
static SemaphoreHandle_t lcd_done;
static QueueHandle_t input_queue;
static esp_lcd_panel_handle_t panel;
static int bridge_socket = -1;
static uint16_t lcd_stripes[2][320 * 16];

static const uint8_t *font_glyph(char c) {
    static const uint8_t space[5] = {0, 0, 0, 0, 0};
    static const uint8_t glyphs[][5] = {
        [0] = {0x7e, 0x09, 0x09, 0x09, 0x7e}, /* A */
        [1] = {0x7f, 0x49, 0x49, 0x49, 0x36}, /* B */
        [2] = {0x3e, 0x41, 0x41, 0x41, 0x22}, /* C */
        [3] = {0x7f, 0x41, 0x41, 0x22, 0x1c}, /* D */
        [4] = {0x7f, 0x49, 0x49, 0x49, 0x41}, /* E */
        [5] = {0x7f, 0x09, 0x09, 0x09, 0x01}, /* F */
        [6] = {0x3e, 0x41, 0x49, 0x49, 0x7a}, /* G */
        [7] = {0x7f, 0x08, 0x08, 0x08, 0x7f}, /* H */
        [8] = {0x00, 0x41, 0x7f, 0x41, 0x00}, /* I */
        [9] = {0x20, 0x40, 0x41, 0x3f, 0x01}, /* J */
        [10] = {0x7f, 0x08, 0x14, 0x22, 0x41}, /* K */
        [11] = {0x7f, 0x40, 0x40, 0x40, 0x40}, /* L */
        [12] = {0x7f, 0x02, 0x0c, 0x02, 0x7f}, /* M */
        [13] = {0x7f, 0x04, 0x08, 0x10, 0x7f}, /* N */
        [14] = {0x3e, 0x41, 0x41, 0x41, 0x3e}, /* O */
        [15] = {0x7f, 0x09, 0x09, 0x09, 0x06}, /* P */
        [16] = {0x3e, 0x41, 0x51, 0x21, 0x5e}, /* Q */
        [17] = {0x7f, 0x09, 0x19, 0x29, 0x46}, /* R */
        [18] = {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
        [19] = {0x01, 0x01, 0x7f, 0x01, 0x01}, /* T */
        [20] = {0x3f, 0x40, 0x40, 0x40, 0x3f}, /* U */
        [21] = {0x1f, 0x20, 0x40, 0x20, 0x1f}, /* V */
        [22] = {0x7f, 0x20, 0x18, 0x20, 0x7f}, /* W */
        [23] = {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
        [24] = {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
        [25] = {0x61, 0x51, 0x49, 0x45, 0x43}, /* Z */
    };
    if (c == ' ') return space;
    if (c >= 'A' && c <= 'Z') return glyphs[(unsigned)(c - 'A')];
    return space;
}

static void draw_status_screen(const char *message) {
    const int scale = 4;
    const int char_width = 6 * scale;
    const int text_width = (int)strlen(message) * char_width - scale;
    const int text_x = (320 - text_width) / 2;
    const int text_y = (240 - 7 * scale) / 2;
    const uint16_t background = 0x0000;
    const uint16_t foreground = 0xffff;

    for (int y = 0; y < 240; y += 16) {
        int rows = (240 - y < 16) ? 240 - y : 16;
        for (int row = 0; row < rows; ++row) {
            for (int x = 0; x < 320; ++x) {
                bool pixel = false;
                int glyph_y = y + row - text_y;
                if (glyph_y >= 0 && glyph_y < 7 * scale) {
                    int glyph_row = glyph_y / scale;
                    for (size_t i = 0; message[i]; ++i) {
                        int glyph_x = x - text_x - (int)i * char_width;
                        if (glyph_x >= 0 && glyph_x < 5 * scale) {
                            const uint8_t *glyph = font_glyph(message[i]);
                            pixel = (glyph[glyph_x / scale] & (1U << glyph_row)) != 0;
                            if (pixel) break;
                        }
                    }
                }
                lcd_stripes[0][row * 320 + x] = pixel ? foreground : background;
            }
        }
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, 320, y + rows, lcd_stripes[0]));
        ESP_ERROR_CHECK(xSemaphoreTake(lcd_done, pdMS_TO_TICKS(100)) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT);
    }
}

static uint16_t be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
static uint32_t be32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static bool recv_all(int fd, void *buf, size_t len) {
    uint8_t *p = buf;
    while (len) {
        int n = recv(fd, p, len, 0);
        if (n == 0) return false;
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += n;
        len -= (size_t)n;
    }
    return true;
}

static bool lcd_color_done(esp_lcd_panel_io_handle_t io,
                           esp_lcd_panel_io_event_data_t *edata,
                           void *user_ctx) {
    (void)io;
    (void)edata;
    (void)user_ctx;
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(lcd_done, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void display_init(void) {
    // Two DMA buffers may complete before the bridge task waits. A counting
    // semaphore preserves both completion callbacks; a binary semaphore can
    // lose the second event and break full-screen transfers.
    lcd_done = xSemaphoreCreateCounting(2, 0);
    assert(lcd_done);
    spi_bus_config_t bus = {
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 320 * 16 * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    esp_lcd_panel_io_handle_t io;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                             &io_config, &io));
    const esp_lcd_panel_io_callbacks_t callbacks = {
        .on_color_trans_done = lcd_color_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &callbacks, NULL));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
    ESP_LOGI(TAG, "ST7789 ready: SPI2 40MHz, 320x240 RGB565");
}

static void buttons_init(void) {
    gpio_config_t input = {
        .pin_bit_mask = (1ULL << BUTTON_DATA) | (1ULL << START_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input));
    gpio_config_t output = {
        .pin_bit_mask = (1ULL << BUTTON_LOAD) | (1ULL << BUTTON_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output));
    gpio_set_level(BUTTON_LOAD, 1);
    gpio_set_level(BUTTON_CLK, 0);
}

static uint8_t read_shift_buttons(void) {
    uint8_t state = 0;
    gpio_set_level(BUTTON_LOAD, 0);
    esp_rom_delay_us(1);
    gpio_set_level(BUTTON_LOAD, 1);
    for (int i = 0; i < 8; ++i) {
        if (gpio_get_level(BUTTON_DATA)) state |= (uint8_t)(1U << i);
        gpio_set_level(BUTTON_CLK, 1);
        esp_rom_delay_us(1);
        gpio_set_level(BUTTON_CLK, 0);
        esp_rom_delay_us(1);
    }
    return state;
}

static uint16_t read_buttons_active_low(void) {
    uint16_t state = (uint16_t)(~read_shift_buttons()) & 0xff;
    if (gpio_get_level(START_BUTTON) == 0) state |= 1U << 8;
    return state;
}

static void send_button(uint8_t id, bool pressed) {
    if (bridge_socket < 0) return;
    uint8_t packet[11] = {0};
    uint32_t len = 7;
    packet[0] = (uint8_t)(len >> 24); packet[1] = (uint8_t)(len >> 16);
    packet[2] = (uint8_t)(len >> 8); packet[3] = (uint8_t)len;
    packet[4] = FRAME_MAGIC_VERSION; packet[5] = INPUT_KIND;
    packet[8] = 1; packet[9] = id; packet[10] = pressed ? 1 : 0;
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    int result = send(bridge_socket, packet, sizeof(packet), 0);
    xSemaphoreGive(socket_lock);
    if (result < 0) {
        ESP_LOGW(TAG, "input send failed: errno=%d", errno);
    } else if (result == sizeof(packet)) {
        ESP_LOGI(TAG, "button id=%u %s", id, pressed ? "pressed" : "released");
    } else {
        ESP_LOGW(TAG, "short input send: %d/%u bytes", result, (unsigned)sizeof(packet));
    }
}

static void button_task(void *arg) {
    (void)arg;
    uint16_t stable = read_buttons_active_low();
    uint16_t candidate = stable;
    uint8_t same = 0;
    while (true) {
        uint16_t now = read_buttons_active_low();
        if (now == candidate) {
            if (same < 3) same++;
        } else {
            candidate = now;
            same = 0;
        }
        if (same >= 3 && candidate != stable) {
            uint16_t changed = candidate ^ stable;
            stable = candidate;
            for (uint8_t id = 0; id < 9; ++id) {
                if (changed & (1U << id)) send_button(id, (stable & (1U << id)) != 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool connect_bridge(void) {
    char port[8];
    snprintf(port, sizeof(port), "%d", CONFIG_BADGE_BRIDGE_PORT);
    struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
    struct addrinfo *result = NULL;
    if (getaddrinfo(CONFIG_BADGE_BRIDGE_HOST, port, &hints, &result) != 0) {
        ESP_LOGW(TAG, "cannot resolve bridge %s:%s", CONFIG_BADGE_BRIDGE_HOST, port);
        return false;
    }
    int fd = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    bool ok = fd >= 0 && connect(fd, result->ai_addr, result->ai_addrlen) == 0;
    freeaddrinfo(result);
    if (!ok) {
        if (fd >= 0) close(fd);
        ESP_LOGW(TAG, "bridge connect failed: errno=%d", errno);
        return false;
    }
    xSemaphoreTake(socket_lock, portMAX_DELAY);
    bridge_socket = fd;
    xSemaphoreGive(socket_lock);
    ESP_LOGI(TAG, "bridge connected to %s:%d", CONFIG_BADGE_BRIDGE_HOST, CONFIG_BADGE_BRIDGE_PORT);
    return true;
}

static bool receive_frame(void) {
    uint8_t length_bytes[4];
    if (!recv_all(bridge_socket, length_bytes, sizeof(length_bytes))) return false;
    uint32_t length = be32(length_bytes);
    if (length < 16 || length > 512 * 1024) {
        ESP_LOGE(TAG, "invalid frame packet length=%" PRIu32, length);
        return false;
    }
    uint8_t header[16];
    if (!recv_all(bridge_socket, header, sizeof(header))) return false;
    if (header[0] != FRAME_MAGIC_VERSION || header[1] != FRAME_KIND) {
        ESP_LOGE(TAG, "invalid frame header version=%d kind=%d", header[0], header[1]);
        return false;
    }
    uint16_t x = be16(&header[8]);
    uint16_t y = be16(&header[10]);
    uint16_t w = be16(&header[12]);
    uint16_t h = be16(&header[14]);
    if (!w || !h || x + w > 320 || y + h > 240 || length < 16 + (uint32_t)w * h * 2) {
        ESP_LOGE(TAG, "invalid rect x=%d y=%d w=%d h=%d", x, y, w, h);
        return false;
    }

    uint16_t rows_done = 0;
    uint8_t buffer_index = 0;
    uint8_t pending_transfers = 0;
    while (rows_done < h) {
        uint16_t rows = h - rows_done;
        if (rows > 16) rows = 16;
        // Keep two DMA buffers in flight: while the LCD sends one, the next
        // stripe can be received from TCP without overwriting display data.
        if (pending_transfers == 2) {
            if (xSemaphoreTake(lcd_done, pdMS_TO_TICKS(100)) != pdTRUE) {
                ESP_LOGE(TAG, "LCD DMA completion timed out");
                return false;
            }
            pending_transfers--;
        }
        size_t bytes = (size_t)w * rows * 2;
        if (!recv_all(bridge_socket, lcd_stripes[buffer_index], bytes)) return false;
        esp_err_t err = esp_lcd_panel_draw_bitmap(panel, x, y + rows_done,
                                                  x + w, y + rows_done + rows,
                                                  lcd_stripes[buffer_index]);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "LCD draw failed: %s", esp_err_to_name(err));
            return false;
        }
        pending_transfers++;
        buffer_index ^= 1;
        rows_done += rows;
    }
    while (pending_transfers) {
        if (xSemaphoreTake(lcd_done, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "LCD DMA completion timed out");
            return false;
        }
        pending_transfers--;
    }
    uint32_t expected = 16 + (uint32_t)w * h * 2;
    if (length != expected) {
        ESP_LOGE(TAG, "frame length mismatch expected=%" PRIu32 " actual=%" PRIu32, expected, length);
        return false;
    }
    return true;
}

static void bridge_task(void *arg) {
    (void)arg;
    while (true) {
        xEventGroupWaitBits(wifi_events, WIFI_CONNECTED, pdFALSE, pdTRUE, portMAX_DELAY);
        if (!connect_bridge()) {
            draw_status_screen("NOT CONNECTED");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        draw_status_screen("CONNECTED");
        while (receive_frame()) {
            static uint32_t frames;
            if ((++frames % 120) == 0) ESP_LOGI(TAG, "received frames=%" PRIu32, frames);
        }
        ESP_LOGW(TAG, "bridge disconnected");
        xSemaphoreTake(socket_lock, portMAX_DELAY);
        int disconnected_socket = bridge_socket;
        bridge_socket = -1;
        xSemaphoreGive(socket_lock);
        if (disconnected_socket >= 0) close(disconnected_socket);
        draw_status_screen("NOT CONNECTED");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg; (void)data;
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = data;
        xEventGroupClearBits(wifi_events, WIFI_CONNECTED);
        esp_wifi_connect();
        ESP_LOGW(TAG, "Wi-Fi disconnected reason=%d; retrying", event->reason);
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = data;
        ESP_LOGI(TAG, "Wi-Fi ready, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(TAG, "Wi-Fi AP=%.*s RSSI=%d channel=%d", (int)sizeof(ap.ssid),
                     ap.ssid, ap.rssi, ap.primary);
        }
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED);
    }
}

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL));
    wifi_config_t config = {0};
    snprintf((char *)config.sta.ssid, sizeof(config.sta.ssid), "%s", CONFIG_BADGE_WIFI_SSID);
    snprintf((char *)config.sta.password, sizeof(config.sta.password), "%s", CONFIG_BADGE_WIFI_PASSWORD);
    config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    // Android hotspots may advertise WPA3-only.  Accept either SAE PWE
    // derivation method so this also works with WPA2/WPA3 transition APs.
    config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi connecting to SSID '%s'", CONFIG_BADGE_WIFI_SSID);
}

void app_main(void) {
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);
    wifi_events = xEventGroupCreate();
    socket_lock = xSemaphoreCreateMutex();
    input_queue = xQueueCreate(INPUT_QUEUE_LEN, sizeof(button_event_t));
    display_init();
    draw_status_screen("NOT CONNECTED");
    buttons_init();
    wifi_init();
    xTaskCreate(button_task, "buttons", 3072, NULL, 5, NULL);
    xTaskCreate(bridge_task, "bridge", 8192, NULL, 6, NULL);
    ESP_LOGI(TAG, "badge bridge started; USB-Serial-JTAG logs enabled");
}
