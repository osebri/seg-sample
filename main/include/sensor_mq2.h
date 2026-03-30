#pragma once

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    int filtered_raw;
    int baseline_raw;
    int delta_raw;
    int response_pct;
    bool warming_up;
} sensor_mq135_reading_t;

esp_err_t sensor_mq2_init(void);
esp_err_t sensor_mq2_read_metrics(sensor_mq135_reading_t *out_reading);
