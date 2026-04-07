#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensor_max30102.h"

typedef enum {
    SENSOR_ID_DHT22 = 0,
    SENSOR_ID_MLX90614,
    SENSOR_ID_MAX30102,
    SENSOR_ID_KY037,
    SENSOR_ID_MQ135,
    SENSOR_ID_ADXL345,
    SENSOR_ID_OLED,
    SENSOR_ID_COUNT
} sensor_id_t;

typedef enum {
    SENSOR_STATE_INIT_FAILED = 0,
    SENSOR_STATE_WAITING_FIRST_SAMPLE,
    SENSOR_STATE_WARMING_UP,
    SENSOR_STATE_HEALTHY,
    SENSOR_STATE_STALE,
    SENSOR_STATE_ERROR,
    SENSOR_STATE_ABSENT_OPTIONAL,
} sensor_state_t;

typedef struct {
    bool initialized;
    bool optional;
    sensor_state_t state;
    esp_err_t last_error;
    int64_t last_attempt_us;
    int64_t last_success_us;
    int64_t stale_after_us;
    uint32_t consecutive_failures;
} sensor_status_t;

typedef struct {
    float dht_temp_c;
    float dht_humidity_pct;

    float mlx_object_temp_c;
    float mlx_ambient_temp_c;

    uint32_t max30102_red;
    uint32_t max30102_ir;
    uint8_t max30102_spo2_pct;
    bool max30102_spo2_valid;
    int32_t max30102_sample_age_ms;
    uint32_t max30102_samples_drained;
    sensor_max30102_signal_hint_t max30102_signal_hint;
    bool max30102_heart_rate_valid;
    uint16_t max30102_heart_rate_bpm;
    uint8_t max30102_heart_rate_confidence_pct;

    int ky037_activity_pct;
    int ky037_peak_to_peak;
    int ky037_mean_abs_deviation;

    int mq135_filtered_raw;
    int mq135_baseline_raw;
    int mq135_delta_raw;
    int mq135_response_pct;

    int16_t adxl_x_raw;
    int16_t adxl_y_raw;
    int16_t adxl_z_raw;

    sensor_status_t status[SENSOR_ID_COUNT];
    int64_t boot_time_us;
} sensor_snapshot_t;

void sensor_registry_init(void);
const char *sensor_registry_name(sensor_id_t sensor_id);
const char *sensor_registry_state_name(sensor_state_t state);

void sensor_registry_configure(sensor_id_t sensor_id, bool optional, int64_t stale_after_us);
void sensor_registry_set_init_result(sensor_id_t sensor_id, esp_err_t init_error, sensor_state_t failure_state);
void sensor_registry_note_attempt(sensor_id_t sensor_id);
void sensor_registry_set_state(sensor_id_t sensor_id, sensor_state_t state, esp_err_t error_code);
void sensor_registry_record_failure(sensor_id_t sensor_id, sensor_state_t state, esp_err_t error_code);
void sensor_registry_update_oled(sensor_state_t state, esp_err_t error_code);

void sensor_registry_update_dht22(float temp_c, float humidity_pct, sensor_state_t state);
void sensor_registry_update_mlx90614(float object_temp_c, float ambient_temp_c, sensor_state_t state);
void sensor_registry_update_max30102(uint32_t red,
                                     uint32_t ir,
                                     uint8_t spo2_pct,
                                     bool spo2_valid,
                                     uint32_t sample_age_ms,
                                     uint32_t samples_drained,
                                     sensor_max30102_signal_hint_t signal_hint,
                                     bool heart_rate_valid,
                                     uint16_t heart_rate_bpm,
                                     uint8_t heart_rate_confidence_pct,
                                     sensor_state_t state);
void sensor_registry_update_max30102_runtime(uint32_t sample_age_ms,
                                             uint8_t spo2_pct,
                                             bool spo2_valid,
                                             uint32_t samples_drained,
                                             sensor_max30102_signal_hint_t signal_hint,
                                             bool heart_rate_valid,
                                             uint16_t heart_rate_bpm,
                                             uint8_t heart_rate_confidence_pct);
void sensor_registry_update_ky037(int activity_pct, int peak_to_peak, int mean_abs_deviation, sensor_state_t state);
void sensor_registry_update_mq135(int filtered_raw,
                                  int baseline_raw,
                                  int delta_raw,
                                  int response_pct,
                                  sensor_state_t state);
void sensor_registry_update_adxl345(int16_t x_raw, int16_t y_raw, int16_t z_raw, sensor_state_t state);

void sensor_registry_get_snapshot(sensor_snapshot_t *out_snapshot);
