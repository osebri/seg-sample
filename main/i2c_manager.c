#include "i2c_manager.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/semphr.h"
#include "project_config.h"

static const char *TAG = "i2c_manager";

static SemaphoreHandle_t s_i2c_mutex;
static bool s_i2c_initialized;

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

static esp_err_t i2c_probe_locked(uint8_t address, TickType_t timeout_ticks)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, timeout_ticks);
    i2c_cmd_link_delete(cmd);
    return err;
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

    i2c_config_t conf = {0};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_GPIO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_GPIO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
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
        esp_err_t err = i2c_probe_locked(addr, pdMS_TO_TICKS(20));
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

    esp_err_t err = i2c_probe_locked(address, timeout_ticks);

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

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_PORT,
        address,
        data,
        len,
        pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));

    i2c_unlock();
    return err;
}

esp_err_t i2c_manager_write_reg(uint8_t address, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!s_i2c_initialized) {
        return ESP_ERR_INVALID_STATE;
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

    esp_err_t err = i2c_master_write_to_device(
        I2C_MASTER_PORT,
        address,
        tx_buf,
        tx_len,
        pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));

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

    esp_err_t err = i2c_master_write_read_device(
        I2C_MASTER_PORT,
        address,
        &reg,
        1,
        data,
        len,
        pdMS_TO_TICKS(I2C_TRANSACTION_TIMEOUT_MS));

    i2c_unlock();
    return err;
}
