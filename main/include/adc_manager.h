#pragma once

#include "esp_err.h"

esp_err_t adc_manager_init(void);
esp_err_t adc_manager_read_ky037(int *raw_value);
esp_err_t adc_manager_read_mq2(int *raw_value);
