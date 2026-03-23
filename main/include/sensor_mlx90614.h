#pragma once

#include "esp_err.h"

esp_err_t sensor_mlx90614_init(void);
esp_err_t sensor_mlx90614_read_temperatures(float *object_temp_c, float *ambient_temp_c);
