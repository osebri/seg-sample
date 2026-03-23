#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

esp_err_t i2c_manager_init(void);
esp_err_t i2c_manager_scan(void);
esp_err_t i2c_manager_probe_device(uint8_t address, TickType_t timeout_ticks);

esp_err_t i2c_manager_write(uint8_t address, const uint8_t *data, size_t len);
esp_err_t i2c_manager_write_reg(uint8_t address, uint8_t reg, const uint8_t *data, size_t len);
esp_err_t i2c_manager_read_reg(uint8_t address, uint8_t reg, uint8_t *data, size_t len);
