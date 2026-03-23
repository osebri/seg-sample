#pragma once

#include "esp_err.h"

esp_err_t sensor_ky037_init(void);
esp_err_t sensor_ky037_read_raw(int *raw_value);
