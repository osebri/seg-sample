#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "sensor_registry.h"

esp_err_t display_ssd1306_init(void);
esp_err_t display_ssd1306_render_summary(const sensor_snapshot_t *snapshot);
bool display_ssd1306_is_ready(void);
