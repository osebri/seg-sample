#pragma once

#include "esp_err.h"
#include "sensor_registry.h"

esp_err_t blynk_bridge_init(void);
void blynk_bridge_task(void *arg);
void blynk_bridge_publish_snapshot(const sensor_snapshot_t *snapshot);
