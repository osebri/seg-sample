#include "sensor_max30102.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "project_config.h"

static const char *TAG = "sensor_max30102";

#define MAX30102_REG_INTR_STATUS_1 0x00
#define MAX30102_REG_INTR_STATUS_2 0x01
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_OVF_COUNTER 0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA 0x07
#define MAX30102_REG_FIFO_CONFIG 0x08
#define MAX30102_REG_MODE_CONFIG 0x09
#define MAX30102_REG_SPO2_CONFIG 0x0A
#define MAX30102_REG_LED1_PA 0x0C
#define MAX30102_REG_LED2_PA 0x0D
#define MAX30102_REG_PART_ID 0xFF

#define MAX30102_PART_ID_EXPECTED 0x15
#define MAX30102_INIT_RETRIES 5
#define MAX30102_MAX_SAMPLES_PER_READ 16
#define MAX30102_SAMPLE_PERIOD_MS 20.0f
#define MAX30102_SAMPLES_PER_SECOND 50.0f
#define MAX30102_HR_BUFFER_SIZE 256U
#define MAX30102_HR_MIN_WINDOW_SAMPLES 180U
#define MAX30102_MAX_PEAKS 16U
#define MAX30102_MIN_BEAT_INTERVAL_SAMPLES 15U
#define MAX30102_MAX_BEAT_INTERVAL_SAMPLES 75U
#define MAX30102_MIN_HR_BPM 45.0f
#define MAX30102_MAX_HR_BPM 180.0f
#define MAX30102_MIN_THRESHOLD 120.0f
#define MAX30102_MIN_PROMINENCE 180.0f
#define MAX30102_LED_CURRENT_START 0x32
#define MAX30102_LED_CURRENT_MIN 0x18
#define MAX30102_LED_CURRENT_MAX 0x50
#define MAX30102_LED_CURRENT_STEP 0x08
#define MAX30102_LED_ADJUST_INTERVAL_US (2000000LL)
#define MAX30102_HR_LOCK_HOLD_US (4000000LL)

static bool s_initialized;
static sensor_max30102_debug_t s_debug_state;

typedef struct {
    bool dc_ready;
    float dc_estimate;
    float ac_envelope;
    uint32_t ir_buffer[MAX30102_HR_BUFFER_SIZE];
    uint16_t sample_count;
    uint16_t write_index;
    sensor_max30102_signal_hint_t signal_hint;
    bool heart_rate_valid;
    uint16_t heart_rate_bpm;
    uint8_t heart_rate_confidence_pct;
    uint8_t led_current_code;
    int64_t last_led_adjust_us;
    int64_t last_hr_lock_us;
} sensor_max30102_hr_state_t;

static sensor_max30102_hr_state_t s_hr_state;
static float s_hr_centered[MAX30102_HR_BUFFER_SIZE];
static float s_hr_smoothed[MAX30102_HR_BUFFER_SIZE];
static uint16_t s_peak_indices[MAX30102_MAX_PEAKS];
static float s_peak_values[MAX30102_MAX_PEAKS];

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value)
{
    return i2c_manager_write_reg(MAX30102_I2C_ADDR, reg, &value, 1);
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(MAX30102_I2C_ADDR, reg, value, 1);
}

static float max30102_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float max30102_minf(float lhs, float rhs)
{
    return (lhs < rhs) ? lhs : rhs;
}

static float max30102_maxf(float lhs, float rhs)
{
    return (lhs > rhs) ? lhs : rhs;
}

static void max30102_reset_debug_state(void)
{
    memset(&s_debug_state, 0, sizeof(s_debug_state));
}

static void max30102_reset_heart_rate_state(void)
{
    memset(&s_hr_state, 0, sizeof(s_hr_state));
    s_hr_state.led_current_code = MAX30102_LED_CURRENT_START;
}

static void max30102_publish_no_lock(sensor_max30102_signal_hint_t hint)
{
    int64_t now_us = esp_timer_get_time();

    if ((hint == SENSOR_MAX30102_SIGNAL_OK || hint == SENSOR_MAX30102_SIGNAL_WEAK) &&
        s_hr_state.heart_rate_valid &&
        s_hr_state.last_hr_lock_us != 0 &&
        (now_us - s_hr_state.last_hr_lock_us) <= MAX30102_HR_LOCK_HOLD_US) {
        if (s_hr_state.heart_rate_confidence_pct > 8U) {
            s_hr_state.heart_rate_confidence_pct = (uint8_t)(s_hr_state.heart_rate_confidence_pct - 8U);
        } else {
            s_hr_state.heart_rate_confidence_pct = 0U;
        }
        return;
    }

    s_hr_state.heart_rate_valid = false;
    s_hr_state.heart_rate_bpm = 0;
    s_hr_state.heart_rate_confidence_pct = 0;
}

static void max30102_publish_lock(uint16_t bpm, uint8_t confidence_pct)
{
    s_hr_state.heart_rate_valid = true;
    s_hr_state.heart_rate_bpm = bpm;
    s_hr_state.heart_rate_confidence_pct = confidence_pct;
    s_hr_state.last_hr_lock_us = esp_timer_get_time();
}

static esp_err_t max30102_set_led_current(uint8_t led_current_code)
{
    esp_err_t err = max30102_write_reg(MAX30102_REG_LED1_PA, led_current_code);
    if (err != ESP_OK) {
        return err;
    }

    err = max30102_write_reg(MAX30102_REG_LED2_PA, led_current_code);
    if (err != ESP_OK) {
        return err;
    }

    s_hr_state.led_current_code = led_current_code;
    s_debug_state.led_current_code = led_current_code;
    return ESP_OK;
}

static void max30102_track_signal(uint32_t ir_value)
{
    if (!s_hr_state.dc_ready) {
        s_hr_state.dc_ready = true;
        s_hr_state.dc_estimate = (float)ir_value;
        s_hr_state.ac_envelope = 0.0f;
    } else {
        s_hr_state.dc_estimate = (s_hr_state.dc_estimate * 0.97f) + ((float)ir_value * 0.03f);
        s_hr_state.ac_envelope =
            (s_hr_state.ac_envelope * 0.96f) + (max30102_absf((float)ir_value - s_hr_state.dc_estimate) * 0.04f);
    }

    s_debug_state.ir_dc_level = (uint32_t)(s_hr_state.dc_estimate + 0.5f);
    s_debug_state.ir_ac_envelope = (uint32_t)(s_hr_state.ac_envelope + 0.5f);
}

static void max30102_push_ir_sample(uint32_t ir_value)
{
    s_hr_state.ir_buffer[s_hr_state.write_index] = ir_value;
    s_hr_state.write_index = (uint16_t)((s_hr_state.write_index + 1U) % MAX30102_HR_BUFFER_SIZE);
    if (s_hr_state.sample_count < MAX30102_HR_BUFFER_SIZE) {
        s_hr_state.sample_count++;
    }
}

static sensor_max30102_signal_hint_t max30102_classify_signal(uint32_t ir_value)
{
    float dc_level = s_hr_state.dc_ready ? s_hr_state.dc_estimate : (float)ir_value;
    float ac_envelope = s_hr_state.ac_envelope;
    float ac_ratio = (dc_level > 1.0f) ? (ac_envelope / dc_level) : 0.0f;

    if (dc_level > 245000.0f || ir_value > 250000U) {
        return SENSOR_MAX30102_SIGNAL_SATURATED;
    }
    if (dc_level < 12000.0f) {
        return SENSOR_MAX30102_SIGNAL_NO_CONTACT;
    }
    if (dc_level < 30000.0f || ac_envelope < 110.0f || ac_ratio < 0.0025f) {
        return SENSOR_MAX30102_SIGNAL_WEAK;
    }
    return SENSOR_MAX30102_SIGNAL_OK;
}

static void max30102_copy_recent_window(uint16_t window_samples)
{
    uint16_t start = 0;
    if (s_hr_state.sample_count == MAX30102_HR_BUFFER_SIZE) {
        start = s_hr_state.write_index;
    }

    if (window_samples < s_hr_state.sample_count) {
        start = (uint16_t)((start + (s_hr_state.sample_count - window_samples)) % MAX30102_HR_BUFFER_SIZE);
    } else {
        window_samples = s_hr_state.sample_count;
    }

    for (uint16_t i = 0; i < window_samples; ++i) {
        uint16_t index = (uint16_t)((start + i) % MAX30102_HR_BUFFER_SIZE);
        s_hr_centered[i] = (float)s_hr_state.ir_buffer[index];
    }
}

static float max30102_interval_jitter(const float *intervals, uint16_t count, float mean_interval)
{
    if (intervals == NULL || count == 0U || mean_interval <= 0.0f) {
        return 1.0f;
    }

    float jitter_sum = 0.0f;
    for (uint16_t i = 0; i < count; ++i) {
        jitter_sum += max30102_absf(intervals[i] - mean_interval) / mean_interval;
    }

    return jitter_sum / (float)count;
}

static void max30102_estimate_heart_rate(sensor_max30102_signal_hint_t latest_hint)
{
    uint16_t window_samples = s_hr_state.sample_count;
    if (window_samples > MAX30102_HR_BUFFER_SIZE) {
        window_samples = MAX30102_HR_BUFFER_SIZE;
    }
    if (window_samples < MAX30102_HR_MIN_WINDOW_SAMPLES) {
        s_debug_state.hr_candidate_peaks = 0;
        s_debug_state.hr_consistent_intervals = 0;
        max30102_publish_no_lock(latest_hint);
        return;
    }

    max30102_copy_recent_window(window_samples);

    float mean_raw = 0.0f;
    for (uint16_t i = 0; i < window_samples; ++i) {
        mean_raw += s_hr_centered[i];
    }
    mean_raw /= (float)window_samples;

    for (uint16_t i = 0; i < window_samples; ++i) {
        s_hr_centered[i] -= mean_raw;
    }

    float envelope_sum = 0.0f;
    float peak_abs = 0.0f;
    for (uint16_t i = 0; i < window_samples; ++i) {
        float smooth_sum = 0.0f;
        uint16_t smooth_count = 0U;

        for (int tap = -2; tap <= 2; ++tap) {
            int index = (int)i + tap;
            if (index < 0 || index >= (int)window_samples) {
                continue;
            }

            smooth_sum += s_hr_centered[index];
            smooth_count++;
        }

        float smoothed = (smooth_count > 0U) ? (smooth_sum / (float)smooth_count) : s_hr_centered[i];
        s_hr_smoothed[i] = smoothed;
        float abs_value = max30102_absf(smoothed);
        envelope_sum += abs_value;
        if (abs_value > peak_abs) {
            peak_abs = abs_value;
        }
    }

    float envelope = envelope_sum / (float)window_samples;
    float threshold = max30102_maxf(MAX30102_MIN_THRESHOLD, envelope * 1.10f);
    float prominence_threshold = max30102_maxf(MAX30102_MIN_PROMINENCE, envelope * 1.35f);

    uint16_t peak_count = 0U;
    int last_peak_index = -(int)MAX30102_MAX_BEAT_INTERVAL_SAMPLES;
    for (uint16_t i = 2U; i + 2U < window_samples && peak_count < MAX30102_MAX_PEAKS; ++i) {
        float current = s_hr_smoothed[i];
        if (current <= threshold) {
            continue;
        }

        if (!(current >= s_hr_smoothed[i - 1U] &&
              current > s_hr_smoothed[i - 2U] &&
              current >= s_hr_smoothed[i + 1U] &&
              current > s_hr_smoothed[i + 2U])) {
            continue;
        }

        float left_min = s_hr_smoothed[i - 1U];
        float right_min = s_hr_smoothed[i + 1U];
        for (uint16_t offset = 2U; offset <= 6U; ++offset) {
            if (i >= offset) {
                left_min = max30102_minf(left_min, s_hr_smoothed[i - offset]);
            }
            if ((i + offset) < window_samples) {
                right_min = max30102_minf(right_min, s_hr_smoothed[i + offset]);
            }
        }

        float prominence = current - ((left_min + right_min) * 0.5f);
        if (prominence < prominence_threshold) {
            continue;
        }

        int spacing = (int)i - last_peak_index;
        if (spacing < (int)MAX30102_MIN_BEAT_INTERVAL_SAMPLES) {
            if (peak_count > 0U && current > s_peak_values[peak_count - 1U]) {
                s_peak_indices[peak_count - 1U] = i;
                s_peak_values[peak_count - 1U] = current;
                last_peak_index = (int)i;
            }
            continue;
        }

        s_peak_indices[peak_count] = i;
        s_peak_values[peak_count] = current;
        last_peak_index = (int)i;
        peak_count++;
    }

    s_debug_state.hr_candidate_peaks = peak_count;

    if (peak_count < 4U || peak_abs < threshold) {
        s_debug_state.hr_consistent_intervals = 0;
        max30102_publish_no_lock(latest_hint);
        return;
    }

    float candidate_intervals[MAX30102_MAX_PEAKS - 1U] = {0.0f};
    uint16_t interval_count = 0U;
    for (uint16_t i = 1U; i < peak_count; ++i) {
        uint16_t interval_samples = (uint16_t)(s_peak_indices[i] - s_peak_indices[i - 1U]);
        if (interval_samples < MAX30102_MIN_BEAT_INTERVAL_SAMPLES ||
            interval_samples > MAX30102_MAX_BEAT_INTERVAL_SAMPLES) {
            continue;
        }

        candidate_intervals[interval_count++] = (float)interval_samples;
    }

    if (interval_count < 3U) {
        s_debug_state.hr_consistent_intervals = 0;
        max30102_publish_no_lock(latest_hint);
        return;
    }

    float interval_sum = 0.0f;
    for (uint16_t i = 0; i < interval_count; ++i) {
        interval_sum += candidate_intervals[i];
    }
    float mean_interval = interval_sum / (float)interval_count;

    float consistent_intervals[MAX30102_MAX_PEAKS - 1U] = {0.0f};
    uint16_t consistent_count = 0U;
    for (uint16_t i = 0; i < interval_count; ++i) {
        float interval = candidate_intervals[i];
        if (interval >= (mean_interval * 0.82f) && interval <= (mean_interval * 1.18f)) {
            consistent_intervals[consistent_count++] = interval;
        }
    }

    s_debug_state.hr_consistent_intervals = consistent_count;

    if (consistent_count < 3U) {
        max30102_publish_no_lock(latest_hint);
        return;
    }

    interval_sum = 0.0f;
    for (uint16_t i = 0; i < consistent_count; ++i) {
        interval_sum += consistent_intervals[i];
    }
    mean_interval = interval_sum / (float)consistent_count;

    float bpm = (60.0f * MAX30102_SAMPLES_PER_SECOND) / mean_interval;
    if (bpm < MAX30102_MIN_HR_BPM || bpm > MAX30102_MAX_HR_BPM) {
        max30102_publish_no_lock(latest_hint);
        return;
    }

    float jitter = max30102_interval_jitter(consistent_intervals, consistent_count, mean_interval);
    if (jitter > 0.18f) {
        max30102_publish_no_lock(latest_hint);
        return;
    }

    uint8_t confidence_pct = 45U;
    confidence_pct = (uint8_t)(confidence_pct + (consistent_count * 10U));
    if (latest_hint == SENSOR_MAX30102_SIGNAL_OK) {
        confidence_pct = (uint8_t)(confidence_pct + 10U);
    }
    if (envelope > 700.0f) {
        confidence_pct = (uint8_t)(confidence_pct + 8U);
    }
    if (jitter < 0.08f) {
        confidence_pct = (uint8_t)(confidence_pct + 7U);
    }
    if (confidence_pct > 100U) {
        confidence_pct = 100U;
    }

    max30102_publish_lock((uint16_t)(bpm + 0.5f), confidence_pct);
}

static void max30102_maybe_adjust_led_current(sensor_max30102_signal_hint_t latest_hint)
{
    int64_t now_us = esp_timer_get_time();
    if (s_hr_state.last_led_adjust_us != 0 &&
        (now_us - s_hr_state.last_led_adjust_us) < MAX30102_LED_ADJUST_INTERVAL_US) {
        return;
    }

    uint8_t next_led_current = s_hr_state.led_current_code;
    if (latest_hint == SENSOR_MAX30102_SIGNAL_SATURATED &&
        next_led_current > MAX30102_LED_CURRENT_MIN) {
        if (next_led_current > (MAX30102_LED_CURRENT_MIN + MAX30102_LED_CURRENT_STEP)) {
            next_led_current = (uint8_t)(next_led_current - MAX30102_LED_CURRENT_STEP);
        } else {
            next_led_current = MAX30102_LED_CURRENT_MIN;
        }
    } else if (latest_hint == SENSOR_MAX30102_SIGNAL_WEAK &&
               s_hr_state.dc_estimate >= 18000.0f &&
               s_hr_state.dc_estimate <= 50000.0f &&
               next_led_current < MAX30102_LED_CURRENT_MAX) {
        if (next_led_current < (MAX30102_LED_CURRENT_MAX - MAX30102_LED_CURRENT_STEP)) {
            next_led_current = (uint8_t)(next_led_current + MAX30102_LED_CURRENT_STEP);
        } else {
            next_led_current = MAX30102_LED_CURRENT_MAX;
        }
    }

    if (next_led_current == s_hr_state.led_current_code) {
        return;
    }

    esp_err_t err = max30102_set_led_current(next_led_current);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED current adjustment failed: %s", esp_err_to_name(err));
        return;
    }

    s_hr_state.last_led_adjust_us = now_us;
    ESP_LOGI(TAG,
             "Adjusted LED current to 0x%02X (dc=%lu env=%lu hint=%s)",
             next_led_current,
             (unsigned long)s_debug_state.ir_dc_level,
             (unsigned long)s_debug_state.ir_ac_envelope,
             sensor_max30102_signal_hint_name(latest_hint));
}

static esp_err_t max30102_read_fifo_state(uint8_t *write_ptr, uint8_t *read_ptr, uint8_t *overflow_count)
{
    esp_err_t err = max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, write_ptr);
    if (err != ESP_OK) {
        return err;
    }

    err = max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, read_ptr);
    if (err != ESP_OK) {
        return err;
    }

    return max30102_read_reg(MAX30102_REG_OVF_COUNTER, overflow_count);
}

const char *sensor_max30102_signal_hint_name(sensor_max30102_signal_hint_t hint)
{
    switch (hint) {
    case SENSOR_MAX30102_SIGNAL_NO_CONTACT:
        return "NO_CONTACT";
    case SENSOR_MAX30102_SIGNAL_WEAK:
        return "WEAK";
    case SENSOR_MAX30102_SIGNAL_OK:
        return "OK";
    case SENSOR_MAX30102_SIGNAL_SATURATED:
        return "SATURATED";
    case SENSOR_MAX30102_SIGNAL_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

esp_err_t sensor_max30102_init(void)
{
    esp_err_t last_err = ESP_FAIL;

    max30102_reset_debug_state();
    max30102_reset_heart_rate_state();

    for (int attempt = 0; attempt < MAX30102_INIT_RETRIES; ++attempt) {
        esp_err_t err = i2c_manager_probe_device(MAX30102_I2C_ADDR, pdMS_TO_TICKS(50));
        if (err != ESP_OK) {
            last_err = ESP_ERR_NOT_FOUND;
            vTaskDelay(pdMS_TO_TICKS(60));
            continue;
        }

        uint8_t part_id = 0;
        err = max30102_read_reg(MAX30102_REG_PART_ID, &part_id);
        if (err != ESP_OK) {
            last_err = err;
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        if (part_id != MAX30102_PART_ID_EXPECTED) {
            ESP_LOGW(TAG,
                     "MAX3010x at 0x%02X has part_id=0x%02X (expected 0x%02X for MAX30102)",
                     MAX30102_I2C_ADDR,
                     part_id,
                     MAX30102_PART_ID_EXPECTED);
            s_initialized = false;
            return ESP_ERR_NOT_SUPPORTED;
        }

        err = max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x40);
        if (err != ESP_OK) {
            last_err = err;
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        vTaskDelay(pdMS_TO_TICKS(20));

        err = max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }
        err = max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }
        err = max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_FIFO_CONFIG, 0x4F);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_SPO2_CONFIG, 0x03);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_set_led_current(MAX30102_LED_CURRENT_START);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        err = max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x03);
        if (err != ESP_OK) {
            last_err = err;
            continue;
        }

        uint8_t clear_reg = 0;
        (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &clear_reg);
        (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &clear_reg);

        s_initialized = true;
        s_debug_state.initialized = true;
        ESP_LOGI(TAG,
                 "MAX30102 detected at 0x%02X (part_id=0x%02X, 50sps, led=0x%02X)",
                 MAX30102_I2C_ADDR,
                 part_id,
                 MAX30102_LED_CURRENT_START);
        return ESP_OK;
    }

    s_initialized = false;
    ESP_LOGW(TAG,
             "MAX30102 init failed after %d attempts: %s",
             MAX30102_INIT_RETRIES,
             esp_err_to_name(last_err));
    return last_err;
}

esp_err_t sensor_max30102_read_latest(sensor_max30102_reading_t *out_reading)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t write_ptr = 0;
    uint8_t read_ptr = 0;
    uint8_t overflow_count = 0;
    esp_err_t err = max30102_read_fifo_state(&write_ptr, &read_ptr, &overflow_count);
    if (err != ESP_OK) {
        s_debug_state.consecutive_read_errors++;
        return err;
    }

    s_debug_state.initialized = true;
    s_debug_state.fifo_write_ptr = write_ptr;
    s_debug_state.fifo_read_ptr = read_ptr;
    s_debug_state.fifo_overflow_count = overflow_count;

    uint8_t samples_available = (uint8_t)((write_ptr - read_ptr) & 0x1FU);
    if (samples_available == 0U) {
        s_debug_state.consecutive_empty_reads++;
        s_debug_state.last_samples_drained = 0;
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t samples_to_read = samples_available;
    if (samples_to_read > MAX30102_MAX_SAMPLES_PER_READ) {
        samples_to_read = MAX30102_MAX_SAMPLES_PER_READ;
    }

    uint8_t fifo_data[MAX30102_MAX_SAMPLES_PER_READ * 6U] = {0};
    err = i2c_manager_read_reg(MAX30102_I2C_ADDR, MAX30102_REG_FIFO_DATA, fifo_data, (size_t)samples_to_read * 6U);
    if (err != ESP_OK) {
        s_debug_state.consecutive_read_errors++;
        return err;
    }

    memset(out_reading, 0, sizeof(*out_reading));
    sensor_max30102_signal_hint_t latest_hint = SENSOR_MAX30102_SIGNAL_UNKNOWN;

    for (uint8_t sample_idx = 0; sample_idx < samples_to_read; ++sample_idx) {
        const uint8_t *sample = &fifo_data[sample_idx * 6U];
        out_reading->red = ((uint32_t)(sample[0] & 0x03U) << 16) | ((uint32_t)sample[1] << 8) | sample[2];
        out_reading->ir = ((uint32_t)(sample[3] & 0x03U) << 16) | ((uint32_t)sample[4] << 8) | sample[5];

        max30102_track_signal(out_reading->ir);
        latest_hint = max30102_classify_signal(out_reading->ir);
        s_hr_state.signal_hint = latest_hint;
        max30102_push_ir_sample(out_reading->ir);
    }

    max30102_estimate_heart_rate(latest_hint);
    max30102_maybe_adjust_led_current(latest_hint);

    out_reading->samples_drained = samples_to_read;
    out_reading->signal_hint = latest_hint;
    out_reading->heart_rate_valid = s_hr_state.heart_rate_valid;
    out_reading->heart_rate_bpm = s_hr_state.heart_rate_bpm;
    out_reading->heart_rate_confidence_pct = s_hr_state.heart_rate_confidence_pct;

    s_debug_state.consecutive_empty_reads = 0;
    s_debug_state.consecutive_read_errors = 0;
    s_debug_state.last_samples_drained = samples_to_read;
    s_debug_state.last_success_us = esp_timer_get_time();

    return ESP_OK;
}

void sensor_max30102_get_debug_state(sensor_max30102_debug_t *out_state)
{
    if (out_state == NULL) {
        return;
    }

    *out_state = s_debug_state;
}
