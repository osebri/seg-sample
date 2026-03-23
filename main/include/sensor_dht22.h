#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

esp_err_t sensor_dht22_init(gpio_num_t data_pin);
esp_err_t sensor_dht22_read(float *temp_c, float *humidity_pct);
