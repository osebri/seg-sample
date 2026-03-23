#include "sensor_max30102.h"

#include <stdbool.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "project_config.h"

static const char *TAG = "sensor_max30102";

#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_OVF_COUNTER 0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CONFIG 0x08
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D
#define MAX30102_REG_PART_ID 0xFF

#define MAX30102_PART_ID_EXPECTED 0x15
#define MAX30102_INIT_RETRIES 5

static bool s_initialized;

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value)
{
    return i2c_manager_write_reg(MAX30102_I2C_ADDR, reg, &value, 1);
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(MAX30102_I2C_ADDR, reg, value, 1);
}

esp_err_t sensor_max30102_init(void)
{
    esp_err_t last_err = ESP_FAIL;

    for (int attempt = 0; attempt < MAX30102_INIT_RETRIES; ++attempt) {
        esp_err_t err = i2c_manager_probe_device(MAX30102_I2C_ADDR, pdMS_TO_TICKS(50));
        if (err != ESP_OK) {
            last_err = ESP_ERR_NOT_FOUND;
            vTaskDelay(pdMS_TO_TICKS(60));
            continue;
        }

        uint8_t part_id = 0;
        err = max30102_read_reg(MAX30102_REG_PART_ID, &part_id);
        if (err != ESP_OK) {
            last_err = err;
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        if (part_id != MAX30102_PART_ID_EXPECTED) {
            ESP_LOGW(TAG,
                     "MAX3010x at 0x%02X has part_id=0x%02X (expected 0x%02X for MAX30102)",
                     MAX30102_I2C_ADDR,
                     part_id,
                     MAX30102_PART_ID_EXPECTED);
            s_initialized = false;
            return ESP_ERR_NOT_SUPPORTED;
        }

        err = max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x40);
        if (err != ESP_OK) {
            last_err = err;
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(20));

        err = max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }
        err = max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }
        err = max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x4F);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_SPO2_CONFIG, 0x27);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_LED1_PA, 0x24);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }
        err = max30102_write_reg(MAX30102_REG_LED2_PA, 0x24);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x03);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        s_initialized = true;
        ESP_LOGI(TAG, "MAX30102 detected at 0x%02X (part_id=0x%02X)", MAX30102_I2C_ADDR, part_id);
        return ESP_OK;
    }

    s_initialized = false;
    ESP_LOGW(TAG,
             "MAX30102 init failed after %d attempts: %s",
             MAX30102_INIT_RETRIES,
             esp_err_to_name(last_err));
    return last_err;
}

esp_err_t sensor_max30102_read_raw(uint32_t *red_value, uint32_t *ir_value)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (red_value == NULL || ir_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    esp_err_t err = max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &write_ptr);
    if (err != ESP_OK) {
        return err;
    }
    err = max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &read_ptr);
    if (err != ESP_OK) {
        return err;
    }

    if (write_ptr == read_ptr) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t sample[6] = {0};
    err = i2c_manager_read_reg(MAX30102_I2C_ADDR, MAX30102_REG_FIFO_DATA, sample, sizeof(sample));
    if (err != ESP_OK) {
        return err;
    }

    // Phase-1 assumption: in SPO2 mode, FIFO sample order is Red then IR.
    *red_value = ((uint32_t)(sample[0] & 0x03) << 16) | ((uint32_t)sample[1] << 8) | sample[2];
    *ir_value = ((uint32_t)(sample[3] & 0x03) << 16) | ((uint32_t)sample[4] << 8) | sample[5];

    return ESP_OK;
}
