#include "sensor_mlx90614.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_manager.h"
#include "project_config.h"

static const char *TAG = "sensor_mlx90614";

#define MLX90614_REG_AMBIENT_TEMP 0x06
#define MLX90614_REG_OBJECT_TEMP 0x07

static bool s_initialized;

static esp_err_t mlx90614_read_word(uint8_t reg, uint16_t *out_word)
{
    if (out_word == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[3] = {0};
    esp_err_t err = i2c_manager_read_reg(MLX90614_I2C_ADDR, reg, raw, sizeof(raw));
    if (err != ESP_OK) {
        return err;
    }

    *out_word = (uint16_t)(raw[0] | (raw[1] << 8));
    return ESP_OK;
}

esp_err_t sensor_mlx90614_init(void)
{
    esp_err_t err = i2c_manager_probe_device(MLX90614_I2C_ADDR, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "MLX90614 detected at 0x%02X", MLX90614_I2C_ADDR);
    return ESP_OK;
}

esp_err_t sensor_mlx90614_read_temperatures(float *object_temp_c, float *ambient_temp_c)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (object_temp_c == NULL || ambient_temp_c == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t ambient_raw = 0;
    uint16_t object_raw = 0;

    esp_err_t err = mlx90614_read_word(MLX90614_REG_AMBIENT_TEMP, &ambient_raw);
    if (err != ESP_OK) {
        return err;
    }

    err = mlx90614_read_word(MLX90614_REG_OBJECT_TEMP, &object_raw);
    if (err != ESP_OK) {
        return err;
    }

    *ambient_temp_c = (ambient_raw * 0.02f) - 273.15f;
    *object_temp_c = (object_raw * 0.02f) - 273.15f;
    return ESP_OK;
}
