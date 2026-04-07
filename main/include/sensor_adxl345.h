#pragma once

#include <stdint.h>

#include "esp_err.h"

esp_err_t sensor_adxl345_init(void);
esp_err_t sensor_adxl345_read_xyz(int16_t *x_raw, int16_t *y_raw, int16_t *z_raw);
