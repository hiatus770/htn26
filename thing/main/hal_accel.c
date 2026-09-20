#include "hal_accel.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

// SC7A20 accelerometer, shared I2C bus per custom-firmware-hal.md section 5.
#define PIN_SDA 5
#define PIN_SCL 6
#define ACCEL_ADDR 0x19

static const char *TAG = "hal_accel";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_ok = false;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

// Bit 7 of the register address enables auto-increment for multi-byte
// reads; harmless (ignored) for the single-byte reads below.
static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t r = reg | 0x80;
    return i2c_master_transmit_receive(s_dev, &r, 1, buf, len, 100);
}

void hal_accel_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) {
        ESP_LOGW(TAG, "i2c bus init failed");
        return;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ACCEL_ADDR,
        .scl_speed_hz = 400000,
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) {
        ESP_LOGW(TAG, "i2c device add failed");
        return;
    }

    uint8_t who = 0;
    if (reg_read(0x0F, &who, 1) != ESP_OK || who != 0x11) {
        ESP_LOGW(TAG, "WHO_AM_I mismatch (got 0x%02x), accelerometer unavailable", who);
        return;
    }

    reg_write(0x20, 0x57); // CTRL_REG1: 100 Hz, all axes on
    reg_write(0x23, 0x80); // CTRL_REG4: block data update, +/-2g

    s_ok = true;
    ESP_LOGI(TAG, "SC7A20 online");
}

bool hal_accel_read(float *x_mg, float *y_mg, float *z_mg)
{
    if (!s_ok) return false;

    // CTRL_REG4's block-data-update bit (set in hal_accel_init) already
    // guarantees a coherent 6-byte read without needing to check the
    // STATUS/ZYXDA data-ready bit first, which this call used to gate on --
    // dropped after that path was never observed to return fresh data.
    uint8_t raw[6];
    if (reg_read(0x28, raw, 6) != ESP_OK) return false;

    int16_t rx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t ry = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t rz = (int16_t)((raw[5] << 8) | raw[4]);

    // 12-bit left-justified: 1 count = 1 mg at +/-2g.
    *x_mg = (float)(rx >> 4);
    *y_mg = (float)(ry >> 4);
    *z_mg = (float)(rz >> 4);
    return true;
}
