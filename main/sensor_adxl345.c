#include "sensor_adxl345.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_manager.h"
#include "project_config.h"

static const char *TAG = "sensor_adxl345";

#define ADXL345_REG_DEVID 0x00
#define ADXL345_REG_BW_RATE 0x2C
#define ADXL345_REG_POWER_CTL 0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_DATAX0 0x32

#define ADXL345_DEVICE_ID 0xE5

static bool s_initialized;
static uint8_t s_i2c_addr = ADXL345_I2C_ADDR;

static const uint8_t s_candidate_addresses[] = {
    ADXL345_I2C_ADDR,
    0x1D,
};

static esp_err_t adxl345_write_reg(uint8_t reg, uint8_t value)
{
    return i2c_manager_write_reg(s_i2c_addr, reg, &value, 1);
}

static esp_err_t adxl345_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(s_i2c_addr, reg, value, 1);
}

esp_err_t sensor_adxl345_init(void)
{
    esp_err_t err = ESP_FAIL;
    uint8_t devid = 0;
    bool found = false;

    for (size_t i = 0; i < (sizeof(s_candidate_addresses) / sizeof(s_candidate_addresses[0])); ++i) {
        s_i2c_addr = s_candidate_addresses[i];
        err = i2c_manager_probe_device(s_i2c_addr, pdMS_TO_TICKS(50));
        if (err != ESP_OK) {
            continue;
        }

        err = adxl345_read_reg(ADXL345_REG_DEVID, &devid);
        if (err != ESP_OK) {
            continue;
        }

        if (devid == ADXL345_DEVICE_ID) {
            found = true;
            break;
        }
    }

    if (!found) {
        s_initialized = false;
        if (err == ESP_OK) {
            return ESP_ERR_INVALID_RESPONSE;
        }
        return err;
    }

    err = adxl345_write_reg(ADXL345_REG_BW_RATE, 0x0A);
    if (err != ESP_OK) {
        return err;
    }

    err = adxl345_write_reg(ADXL345_REG_DATA_FORMAT, 0x08);
    if (err != ESP_OK) {
        return err;
    }

    err = adxl345_write_reg(ADXL345_REG_POWER_CTL, 0x08);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "ADXL345 detected at 0x%02X", s_i2c_addr);
    return ESP_OK;
}

esp_err_t sensor_adxl345_read_xyz(float *x_g, float *y_g, float *z_g)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (x_g == NULL || y_g == NULL || z_g == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[6] = {0};
    esp_err_t err = i2c_manager_read_reg(s_i2c_addr, ADXL345_REG_DATAX0, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    int16_t x = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t y = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t z = (int16_t)((raw[5] << 8) | raw[4]);

    const float scale_g_per_lsb = 0.0039f;
    *x_g = x * scale_g_per_lsb;
    *y_g = y * scale_g_per_lsb;
    *z_g = z * scale_g_per_lsb;

    return ESP_OK;
}
