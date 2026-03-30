#pragma once

#include "esp_err.h"

typedef struct {
    int activity_pct;
    int peak_to_peak;
    int mean_abs_deviation;
} sensor_ky037_reading_t;

esp_err_t sensor_ky037_init(void);
esp_err_t sensor_ky037_read_activity(sensor_ky037_reading_t *out_reading);
