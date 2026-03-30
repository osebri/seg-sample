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

static void sensor_registry_apply_success_locked(sensor_id_t sensor_id, sensor_state_t state)
{
    sensor_status_t *status = &s_snapshot.status[sensor_id];

    status->initialized = true;
    status->state = state;
    status->last_error = ESP_OK;
    status->last_success_us = esp_timer_get_time();
    status->last_attempt_us = status->last_success_us;
    status->consecutive_failures = 0;
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
        s_snapshot.status[i].optional = false;
        s_snapshot.status[i].state = SENSOR_STATE_INIT_FAILED;
        s_snapshot.status[i].last_error = ESP_ERR_INVALID_STATE;
        s_snapshot.status[i].last_attempt_us = 0;
        s_snapshot.status[i].last_success_us = 0;
        s_snapshot.status[i].stale_after_us = 0;
        s_snapshot.status[i].consecutive_failures = 0;
    }
    s_snapshot.max30102_signal_hint = SENSOR_MAX30102_SIGNAL_UNKNOWN;
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
    case SENSOR_ID_MQ135:
        return "MQ135";
    case SENSOR_ID_ADXL345:
        return "ADXL345";
    case SENSOR_ID_OLED:
        return "SSD1306";
    case SENSOR_ID_COUNT:
    default:
        return "UNKNOWN";
    }
}

const char *sensor_registry_state_name(sensor_state_t state)
{
    switch (state) {
    case SENSOR_STATE_INIT_FAILED:
        return "INIT_FAILED";
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        return "WAITING_FIRST_SAMPLE";
    case SENSOR_STATE_WARMING_UP:
        return "WARMING_UP";
    case SENSOR_STATE_HEALTHY:
        return "HEALTHY";
    case SENSOR_STATE_STALE:
        return "STALE";
    case SENSOR_STATE_ERROR:
        return "ERROR";
    case SENSOR_STATE_ABSENT_OPTIONAL:
        return "ABSENT_OPTIONAL";
    default:
        return "UNKNOWN";
    }
}

void sensor_registry_configure(sensor_id_t sensor_id, bool optional, int64_t stale_after_us)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.status[sensor_id].optional = optional;
    s_snapshot.status[sensor_id].stale_after_us = stale_after_us;

    sensor_registry_unlock();
}

void sensor_registry_set_init_result(sensor_id_t sensor_id, esp_err_t init_error, sensor_state_t failure_state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    sensor_status_t *status = &s_snapshot.status[sensor_id];
    status->last_attempt_us = esp_timer_get_time();
    status->consecutive_failures = 0;

    if (init_error == ESP_OK) {
        status->initialized = true;
        status->state = SENSOR_STATE_WAITING_FIRST_SAMPLE;
        status->last_error = ESP_OK;
        status->last_success_us = 0;
    } else {
        status->initialized = false;
        status->state = failure_state;
        status->last_error = init_error;
    }

    sensor_registry_unlock();
}

void sensor_registry_note_attempt(sensor_id_t sensor_id)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.status[sensor_id].last_attempt_us = esp_timer_get_time();

    sensor_registry_unlock();
}

void sensor_registry_set_state(sensor_id_t sensor_id, sensor_state_t state, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    sensor_status_t *status = &s_snapshot.status[sensor_id];
    status->state = state;
    status->last_error = (error_code == ESP_OK) ? status->last_error : error_code;

    sensor_registry_unlock();
}

void sensor_registry_record_failure(sensor_id_t sensor_id, sensor_state_t state, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    sensor_status_t *status = &s_snapshot.status[sensor_id];
    status->state = state;
    status->last_error = (error_code == ESP_OK) ? ESP_FAIL : error_code;
    status->consecutive_failures++;
    status->last_attempt_us = esp_timer_get_time();

    sensor_registry_unlock();
}

void sensor_registry_update_oled(sensor_state_t state, esp_err_t error_code)
{
    if (!sensor_registry_lock()) {
        return;
    }

    if (error_code == ESP_OK) {
        sensor_registry_apply_success_locked(SENSOR_ID_OLED, state);
    } else {
        sensor_status_t *status = &s_snapshot.status[SENSOR_ID_OLED];
        status->state = state;
        status->last_error = error_code;
        status->consecutive_failures++;
        status->last_attempt_us = esp_timer_get_time();
    }

    sensor_registry_unlock();
}

void sensor_registry_update_dht22(float temp_c, float humidity_pct, sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.dht_temp_c = temp_c;
    s_snapshot.dht_humidity_pct = humidity_pct;
    sensor_registry_apply_success_locked(SENSOR_ID_DHT22, state);

    sensor_registry_unlock();
}

void sensor_registry_update_mlx90614(float object_temp_c, float ambient_temp_c, sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.mlx_object_temp_c = object_temp_c;
    s_snapshot.mlx_ambient_temp_c = ambient_temp_c;
    sensor_registry_apply_success_locked(SENSOR_ID_MLX90614, state);

    sensor_registry_unlock();
}

void sensor_registry_update_max30102(uint32_t red,
                                     uint32_t ir,
                                     uint32_t sample_age_ms,
                                     uint32_t samples_drained,
                                     sensor_max30102_signal_hint_t signal_hint,
                                     bool heart_rate_valid,
                                     uint16_t heart_rate_bpm,
                                     uint8_t heart_rate_confidence_pct,
                                     sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.max30102_red = red;
    s_snapshot.max30102_ir = ir;
    s_snapshot.max30102_sample_age_ms = (int32_t)sample_age_ms;
    s_snapshot.max30102_samples_drained = samples_drained;
    s_snapshot.max30102_signal_hint = signal_hint;
    s_snapshot.max30102_heart_rate_valid = heart_rate_valid;
    s_snapshot.max30102_heart_rate_bpm = heart_rate_bpm;
    s_snapshot.max30102_heart_rate_confidence_pct = heart_rate_confidence_pct;
    sensor_registry_apply_success_locked(SENSOR_ID_MAX30102, state);

    sensor_registry_unlock();
}

void sensor_registry_update_max30102_runtime(uint32_t sample_age_ms,
                                             uint32_t samples_drained,
                                             sensor_max30102_signal_hint_t signal_hint,
                                             bool heart_rate_valid,
                                             uint16_t heart_rate_bpm,
                                             uint8_t heart_rate_confidence_pct)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.max30102_sample_age_ms = (int32_t)sample_age_ms;
    s_snapshot.max30102_samples_drained = samples_drained;
    s_snapshot.max30102_signal_hint = signal_hint;
    s_snapshot.max30102_heart_rate_valid = heart_rate_valid;
    s_snapshot.max30102_heart_rate_bpm = heart_rate_bpm;
    s_snapshot.max30102_heart_rate_confidence_pct = heart_rate_confidence_pct;

    sensor_registry_unlock();
}

void sensor_registry_update_ky037(int activity_pct, int peak_to_peak, int mean_abs_deviation, sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.ky037_activity_pct = activity_pct;
    s_snapshot.ky037_peak_to_peak = peak_to_peak;
    s_snapshot.ky037_mean_abs_deviation = mean_abs_deviation;
    sensor_registry_apply_success_locked(SENSOR_ID_KY037, state);

    sensor_registry_unlock();
}

void sensor_registry_update_mq135(int filtered_raw,
                                  int baseline_raw,
                                  int delta_raw,
                                  int response_pct,
                                  sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.mq135_filtered_raw = filtered_raw;
    s_snapshot.mq135_baseline_raw = baseline_raw;
    s_snapshot.mq135_delta_raw = delta_raw;
    s_snapshot.mq135_response_pct = response_pct;
    sensor_registry_apply_success_locked(SENSOR_ID_MQ135, state);

    sensor_registry_unlock();
}

void sensor_registry_update_adxl345(float x_g, float y_g, float z_g, sensor_state_t state)
{
    if (!sensor_registry_lock()) {
        return;
    }

    s_snapshot.adxl_x_g = x_g;
    s_snapshot.adxl_y_g = y_g;
    s_snapshot.adxl_z_g = z_g;
    sensor_registry_apply_success_locked(SENSOR_ID_ADXL345, state);

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
