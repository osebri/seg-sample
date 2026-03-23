#pragma once

#include "esp_err.h"

esp_err_t sensor_mq2_init(void);
esp_err_t sensor_mq2_read_raw(int *raw_value);
