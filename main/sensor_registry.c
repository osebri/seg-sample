#include "sensor_registry.h"

#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static sensor_snapshot_t s_snapshot;
static SemaphoreHandle_t s_registry_mutex;

static bool sensor_registry_lock(void)
{
    if (s_registry_mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_registry_mutex, portMAX_DELAY) == pdTRUE;
}

static void sensor_registry_unlock(void)
{
    if (s_registry_mutex != NULL) {
        xSemaphoreGive(s_registry_mutex);
    }
}

static void sensor_registry_set_ok_locked(sensor_id_t sensor_id)
{
    s_snapshot.status[sensor_id].initialized = true;
    s_snapshot.status[sensor_id].healthy = true;
    s_snapshot.status[sensor_id].last_error = ESP_OK;
    s_snapshot.status[sensor_id].last_update_us = esp_timer_get_time();
}

static void sensor_registry_set_error_locked(sensor_id_t sensor_id, esp_err_t error_code)
{
    if (error_code == ESP_OK) {
        error_code = ESP_FAIL;
    }
    s_snapshot.status[sensor_id].healthy = false;
    s_snapshot.status[sensor_id].last_error = error_code;
}

void sensor_registry_init(void)
{
    if (s_registry_mutex == NULL) {
        s_registry_mutex = xSemaphoreCreateMutex();
    }

    if (!sensor_registry_lock()) {
        return;
    }

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    for (int i = 0; i < SENSOR_ID_COUNT; ++i) {
        s_snapshot.status[i].initialized = false;
        s_snapshot.status[i].healthy = false;
        s_snapshot.status[i].last_error = ESP_FAIL;
        s_snapshot.status[i].last_update_us = 0;
    }
    s_snapshot.boot_time_us = esp_timer_get_time();

    sensor_registry_unlock();
}

const char *sensor_registry_name(sensor_id_t sensor_id)
{
    switch (sensor_id) {
    case SENSOR_ID_DHT22:
        return "DHT22";
    case SENSOR_ID_MLX90614:
        return "MLX90614";
    case SENSOR_ID_MAX30102:
        return "MAX30102";
    case SENSOR_ID_KY037:
        return "KY037";
    case SENSOR_ID_MQ2:
        return "MQ2";
    case SENSOR_ID_ADXL345:
        return "ADXL345";
    case SENSOR_ID_OLED:
        return "SSD1306";
    case SENSOR_ID_COUNT:
    default:
        return "UNKNOWN";
    }
}

void sensor_registry_set_init_status(sensor_id_t sensor_id, bool initialized, esp_err_t init_error)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.status[sensor_id].initialized = initialized;
    s_snapshot.status[sensor_id].healthy = initialized;
    s_snapshot.status[sensor_id].last_error = initialized ? ESP_OK : init_error;
    s_snapshot.status[sensor_id].last_update_us = initialized ? esp_timer_get_time() : 0;

    sensor_registry_unlock();
}

void sensor_registry_mark_error(sensor_id_t sensor_id, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    sensor_registry_set_error_locked(sensor_id, error_code);

    sensor_registry_unlock();
}

void sensor_registry_mark_ok(sensor_id_t sensor_id)
{
    if (!sensor_registry_lock()) {
        return;
    }

    sensor_registry_set_ok_locked(sensor_id);

    sensor_registry_unlock();
}

void sensor_registry_update_dht22(float temp_c, float humidity_pct, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.dht_temp_c = temp_c;
        s_snapshot.dht_humidity_pct = humidity_pct;
        sensor_registry_set_ok_locked(SENSOR_ID_DHT22);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_DHT22, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_update_mlx90614(float object_temp_c, float ambient_temp_c, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.mlx_object_temp_c = object_temp_c;
        s_snapshot.mlx_ambient_temp_c = ambient_temp_c;
        sensor_registry_set_ok_locked(SENSOR_ID_MLX90614);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_MLX90614, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_update_max30102(uint32_t red, uint32_t ir, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.max30102_red = red;
        s_snapshot.max30102_ir = ir;
        sensor_registry_set_ok_locked(SENSOR_ID_MAX30102);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_MAX30102, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_update_ky037(int raw_value, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.ky037_raw = raw_value;
        sensor_registry_set_ok_locked(SENSOR_ID_KY037);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_KY037, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_update_mq2(int raw_value, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.mq2_raw = raw_value;
        sensor_registry_set_ok_locked(SENSOR_ID_MQ2);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_MQ2, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_update_adxl345(float x_g, float y_g, float z_g, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        s_snapshot.adxl_x_g = x_g;
        s_snapshot.adxl_y_g = y_g;
        s_snapshot.adxl_z_g = z_g;
        sensor_registry_set_ok_locked(SENSOR_ID_ADXL345);
    } else {
        sensor_registry_set_error_locked(SENSOR_ID_ADXL345, error_code);
    }

    sensor_registry_unlock();
}

void sensor_registry_get_snapshot(sensor_snapshot_t *out_snapshot)
{
    if (out_snapshot == NULL) {
        return;
    }

    if (!sensor_registry_lock()) {
        memset(out_snapshot, 0, sizeof(*out_snapshot));
        return;
    }

    *out_snapshot = s_snapshot;

    sensor_registry_unlock();
}
