#pragma once

#include "esp_err.h"

esp_err_t sensor_adxl345_init(void);
esp_err_t sensor_adxl345_read_xyz(float *x_g, float *y_g, float *z_g);
