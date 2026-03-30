#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SENSOR_MAX30102_SIGNAL_UNKNOWN = 0,
    SENSOR_MAX30102_SIGNAL_NO_CONTACT,
    SENSOR_MAX30102_SIGNAL_WEAK,
    SENSOR_MAX30102_SIGNAL_OK,
    SENSOR_MAX30102_SIGNAL_SATURATED,
} sensor_max30102_signal_hint_t;

typedef struct {
    bool initialized;
    uint8_t fifo_write_ptr;
    uint8_t fifo_read_ptr;
    uint8_t fifo_overflow_count;
    uint8_t led_current_code;
    uint32_t consecutive_empty_reads;
    uint32_t consecutive_read_errors;
    uint32_t last_samples_drained;
    uint16_t hr_candidate_peaks;
    uint16_t hr_consistent_intervals;
    uint32_t ir_dc_level;
    uint32_t ir_ac_envelope;
    int64_t last_success_us;
} sensor_max30102_debug_t;

typedef struct {
    uint32_t red;
    uint32_t ir;
    uint32_t samples_drained;
    sensor_max30102_signal_hint_t signal_hint;
    bool heart_rate_valid;
    uint16_t heart_rate_bpm;
    uint8_t heart_rate_confidence_pct;
} sensor_max30102_reading_t;

esp_err_t sensor_max30102_init(void);
esp_err_t sensor_max30102_read_latest(sensor_max30102_reading_t *out_reading);
const char *sensor_max30102_signal_hint_name(sensor_max30102_signal_hint_t hint);
void sensor_max30102_get_debug_state(sensor_max30102_debug_t *out_state);
