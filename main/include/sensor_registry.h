#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SENSOR_ID_DHT22 = 0,
    SENSOR_ID_MLX90614,
    SENSOR_ID_MAX30102,
    SENSOR_ID_KY037,
    SENSOR_ID_MQ2,
    SENSOR_ID_ADXL345,
    SENSOR_ID_OLED,
    SENSOR_ID_COUNT
} sensor_id_t;

typedef struct {
    bool initialized;
    bool healthy;
    esp_err_t last_error;
    int64_t last_update_us;
} sensor_status_t;

typedef struct {
    float dht_temp_c;
    float dht_humidity_pct;

    float mlx_object_temp_c;
    float mlx_ambient_temp_c;

    uint32_t max30102_red;
    uint32_t max30102_ir;

    int ky037_raw;
    int mq2_raw;

    float adxl_x_g;
    float adxl_y_g;
    float adxl_z_g;

    sensor_status_t status[SENSOR_ID_COUNT];
    int64_t boot_time_us;
} sensor_snapshot_t;

void sensor_registry_init(void);
const char *sensor_registry_name(sensor_id_t sensor_id);

void sensor_registry_set_init_status(sensor_id_t sensor_id, bool initialized, esp_err_t init_error);
void sensor_registry_mark_error(sensor_id_t sensor_id, esp_err_t error_code);
void sensor_registry_mark_ok(sensor_id_t sensor_id);

void sensor_registry_update_dht22(float temp_c, float humidity_pct, esp_err_t error_code);
void sensor_registry_update_mlx90614(float object_temp_c, float ambient_temp_c, esp_err_t error_code);
void sensor_registry_update_max30102(uint32_t red, uint32_t ir, esp_err_t error_code);
void sensor_registry_update_ky037(int raw_value, esp_err_t error_code);
void sensor_registry_update_mq2(int raw_value, esp_err_t error_code);
void sensor_registry_update_adxl345(float x_g, float y_g, float z_g, esp_err_t error_code);

void sensor_registry_get_snapshot(sensor_snapshot_t *out_snapshot);
