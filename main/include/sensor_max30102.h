#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t sensor_max30102_init(void);
esp_err_t sensor_max30102_read_raw(uint32_t *red_value, uint32_t *ir_value);
