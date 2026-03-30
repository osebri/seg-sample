#include "i2c_manager.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "project_config.h"

static const char *TAG = "i2c_manager";

static SemaphoreHandle_t s_i2c_mutex;
static bool s_i2c_initialized;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_i2c_devices[128];

static esp_err_t i2c_lock(TickType_t timeout_ticks)
{
    if (s_i2c_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_i2c_mutex, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void i2c_unlock(void)
{
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }
}

static int ticks_to_ms(TickType_t timeout_ticks)
{
    uint32_t timeout_ms = pdTICKS_TO_MS(timeout_ticks);
    if (timeout_ms > INT_MAX) {
        return INT_MAX;
    }
    return (int)timeout_ms;
}

static esp_err_t i2c_get_device_locked(uint8_t address, i2c_master_dev_handle_t *out_handle)
{
    if (out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (address == 0 || address >= 0x80) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_i2c_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_i2c_devices[address] == NULL) {
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = address,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
            .scl_wait_us = 0,
        };

        esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_devices[address]);
        if (err != ESP_OK) {
            return err;
        }
    }

    *out_handle = s_i2c_devices[address];
    return ESP_OK;
}

esp_err_t i2c_manager_init(void)
{
    if (s_i2c_initialized) {
        return ESP_OK;
    }

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_GPIO,
        .scl_io_num = I2C_MASTER_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    s_i2c_initialized = true;
    ESP_LOGI(TAG, "I2C initialized on SDA=%d SCL=%d @ %dHz",
             I2C_MASTER_SDA_GPIO,
             I2C_MASTER_SCL_GPIO,
             I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

esp_err_t i2c_manager_scan(void)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lock_err = i2c_lock(pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    int found_count = 0;
    ESP_LOGI(TAG, "Scanning I2C bus...");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        esp_err_t err = i2c_master_probe(s_i2c_bus, addr, 20);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
            ++found_count;
        }
    }
    ESP_LOGI(TAG, "I2C scan complete, found %d device(s)", found_count);

    i2c_unlock();
    return ESP_OK;
}

esp_err_t i2c_manager_probe_device(uint8_t address, TickType_t timeout_ticks)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lock_err = i2c_lock(timeout_ticks);
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    esp_err_t err = i2c_master_probe(s_i2c_bus, address, ticks_to_ms(timeout_ticks));

    i2c_unlock();
    return err;
}

esp_err_t i2c_manager_write(uint8_t address, const uint8_t *data, size_t len)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lock_err = i2c_lock(pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t err = i2c_get_device_locked(address, &dev_handle);
    if (err == ESP_OK) {
        err = i2c_master_transmit(dev_handle, data, len, I2C_TRANSACTION_TIMEOUT_MS);
    }

    i2c_unlock();
    return err;
}

esp_err_t i2c_manager_write_reg(uint8_t address, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 0 && data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t tx_len = len + 1;
    uint8_t *tx_buf = (uint8_t *)malloc(tx_len);
    if (tx_buf == NULL) {
        return ESP_ERR_NO_MEM;
    }

    tx_buf[0] = reg;
    if (len > 0 && data != NULL) {
        memcpy(&tx_buf[1], data, len);
    }

    esp_err_t lock_err = i2c_lock(pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));
    if (lock_err != ESP_OK) {
        free(tx_buf);
        return lock_err;
    }

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t err = i2c_get_device_locked(address, &dev_handle);
    if (err == ESP_OK) {
        err = i2c_master_transmit(dev_handle, tx_buf, tx_len, I2C_TRANSACTION_TIMEOUT_MS);
    }

    i2c_unlock();
    free(tx_buf);
    return err;
}

esp_err_t i2c_manager_read_reg(uint8_t address, uint8_t reg, uint8_t *data, size_t len)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lock_err = i2c_lock(pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));
    if (lock_err != ESP_OK) {
        return lock_err;
    }

    i2c_master_dev_handle_t dev_handle = NULL;
    esp_err_t err = i2c_get_device_locked(address, &dev_handle);
    if (err == ESP_OK) {
        err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, I2C_TRANSACTION_TIMEOUT_MS);
    }

    i2c_unlock();
    return err;
}
