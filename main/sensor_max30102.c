#include "sensor_max30102.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "i2c_manager.h"
#include "project_config.h"
static const char *TAG = "sensor_max30102";

#define MAX30102_REG_INTR_STATUS_1 0x00
#define MAX30102_REG_INTR_STATUS_2 0x01
#define MAX30102_REG_INTR_ENABLE_1 0x02
#define MAX30102_REG_INTR_ENABLE_2 0x03
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

#define MAX30102_PART_ID 0x15
#define MAX30102_MODE_RESET 0x40
#define MAX30102_MODE_SPO2 0x03

#define MAX30102_FIFO_SAMPLE_BYTES 6
#define MAX30102_MAX_BATCH_SAMPLES 8
#define MAX30102_SAMPLE_MASK 0x03FFFFUL
#define MAX30102_FIFO_CONFIG_VALUE 0x4F
#define MAX30102_SPO2_CONFIG_VALUE 0x27
#define MAX30102_LED_CURRENT_DEFAULT 0x24

#define MAX30102_NO_FINGER_THRESHOLD 50000UL
#define MAX30102_BEAT_THRESHOLD 70000UL
#define MAX30102_MIN_BPM 20U
#define MAX30102_MAX_BPM 255U
#define MAX30102_MIN_BEAT_INTERVAL_US 250000LL
#define MAX30102_RATE_SIZE 4U
#define MAX30102_ASSUMED_SAMPLE_PERIOD_US 10000LL
static bool s_initialized;
static sensor_max30102_debug_t s_debug;
static uint32_t s_last_red;
static uint32_t s_last_ir;
static uint8_t s_led_current_code = MAX30102_LED_CURRENT_DEFAULT;
static uint16_t s_rates[MAX30102_RATE_SIZE];
static uint8_t s_rate_spot;
static int64_t s_last_beat_us;
static float s_beats_per_minute;
static uint16_t s_beat_avg;
static bool s_heart_rate_valid;
static uint8_t s_confidence_pct;
static uint8_t s_spo2_pct;
static bool s_spo2_valid;
static uint32_t s_prev_ir2;
static uint32_t s_prev_ir1;
static bool s_have_prev_ir2;
static bool s_have_prev_ir1;

static esp_err_t max30102_write_u8(uint8_t reg, uint8_t value)
{
    return i2c_manager_write_reg(MAX30102_I2C_ADDR, reg, &value, 1);
}

static esp_err_t max30102_read_u8(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(MAX30102_I2C_ADDR, reg, value, 1);
}

static esp_err_t max30102_read_multi(uint8_t reg, uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return i2c_manager_read_reg(MAX30102_I2C_ADDR, reg, data, len);
}

static uint32_t max30102_decode_sample(const uint8_t *buf)
{
    return ((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | (uint32_t)buf[2]) & MAX30102_SAMPLE_MASK;
}

static void max30102_reset_runtime_state(void)
{
    memset(&s_debug, 0, sizeof(s_debug));
    memset(s_rates, 0, sizeof(s_rates));
    s_last_red = 0;
    s_last_ir = 0;
    s_rate_spot = 0;
    s_last_beat_us = 0;
    s_beats_per_minute = 0.0f;
    s_beat_avg = 0;
    s_heart_rate_valid = false;
    s_confidence_pct = 0;
    s_spo2_pct = 0;
    s_spo2_valid = false;
    s_prev_ir2 = 0;
    s_prev_ir1 = 0;
    s_have_prev_ir2 = false;
    s_have_prev_ir1 = false;
    s_debug.led_current_code = s_led_current_code;
}

static sensor_max30102_signal_hint_t max30102_classify_signal(uint32_t ir, uint32_t red)
{
    uint32_t stronger = (ir > red) ? ir : red;

    if (stronger < MAX30102_NO_FINGER_THRESHOLD) {
        return SENSOR_MAX30102_SIGNAL_NO_CONTACT;
    }
    if (stronger < MAX30102_BEAT_THRESHOLD) {
        return SENSOR_MAX30102_SIGNAL_WEAK;
    }
    if (stronger > 240000UL) {
        return SENSOR_MAX30102_SIGNAL_SATURATED;
    }
    return SENSOR_MAX30102_SIGNAL_OK;
}

static void max30102_clear_heart_rate(void)
{
    memset(s_rates, 0, sizeof(s_rates));
    s_rate_spot = 0;
    s_last_beat_us = 0;
    s_beats_per_minute = 0.0f;
    s_beat_avg = 0;
    s_heart_rate_valid = false;
    s_confidence_pct = 0;
    s_spo2_pct = 0;
    s_spo2_valid = false;
    s_prev_ir2 = 0;
    s_prev_ir1 = 0;
    s_have_prev_ir2 = false;
    s_have_prev_ir1 = false;
    s_debug.hr_candidate_peaks = 0;
    s_debug.hr_consistent_intervals = 0;
}

static void max30102_store_rate(uint16_t bpm)
{
    s_rates[s_rate_spot] = bpm;
    s_rate_spot = (uint8_t)((s_rate_spot + 1U) % MAX30102_RATE_SIZE);

    uint32_t sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX30102_RATE_SIZE; ++i) {
        if (s_rates[i] != 0U) {
            sum += s_rates[i];
            count++;
        }
    }

    if (count == 0U) {
        s_beat_avg = 0;
        s_heart_rate_valid = false;
        s_confidence_pct = 0;
        return;
    }

    s_beat_avg = (uint16_t)(sum / count);
    s_heart_rate_valid = count >= 2U;
    s_confidence_pct = (uint8_t)((count * 100U) / MAX30102_RATE_SIZE);
}

static void max30102_update_spo2(uint32_t red_min,
                                 uint32_t red_max,
                                 uint32_t red_sum,
                                 uint32_t ir_min,
                                 uint32_t ir_max,
                                 uint32_t ir_sum,
                                 uint8_t sample_count,
                                 sensor_max30102_signal_hint_t signal_hint)
{
    s_spo2_valid = false;
    s_spo2_pct = 0;

    if (sample_count == 0U || signal_hint != SENSOR_MAX30102_SIGNAL_OK) {
        return;
    }

    uint32_t red_dc = red_sum / sample_count;
    uint32_t ir_dc = ir_sum / sample_count;
    uint32_t red_ac = red_max - red_min;
    uint32_t ir_ac = ir_max - ir_min;
    if (red_dc == 0U || ir_dc == 0U || red_ac == 0U || ir_ac == 0U) {
        return;
    }

    float ratio = ((float)red_ac / (float)red_dc) / ((float)ir_ac / (float)ir_dc);
    int spo2 = (int)(110.0f - (25.0f * ratio));
    if (spo2 < 0) {
        spo2 = 0;
    } else if (spo2 > 100) {
        spo2 = 100;
    }

    s_spo2_pct = (uint8_t)spo2;
    s_spo2_valid = (spo2 >= 70 && spo2 <= 100);
}

static void max30102_process_ir_sample(uint32_t ir, int64_t sample_time_us)
{
    if (!s_have_prev_ir1) {
        s_prev_ir1 = ir;
        s_have_prev_ir1 = true;
        return;
    }

    if (!s_have_prev_ir2) {
        s_prev_ir2 = s_prev_ir1;
        s_prev_ir1 = ir;
        s_have_prev_ir2 = true;
        return;
    }

    if (s_prev_ir1 > s_prev_ir2 &&
        s_prev_ir1 > ir &&
        s_prev_ir1 > MAX30102_BEAT_THRESHOLD) {
        s_debug.hr_candidate_peaks++;

        int64_t beat_time_us = sample_time_us - MAX30102_ASSUMED_SAMPLE_PERIOD_US;
        if (s_last_beat_us != 0) {
            int64_t delta_us = beat_time_us - s_last_beat_us;
            if (delta_us >= MAX30102_MIN_BEAT_INTERVAL_US) {
                float bpm = 60.0f / ((float)delta_us / 1000000.0f);
                if (bpm > (float)MAX30102_MIN_BPM && bpm < (float)MAX30102_MAX_BPM) {
                    s_beats_per_minute = bpm;
                    max30102_store_rate((uint16_t)bpm);
                    s_debug.hr_consistent_intervals++;
                }
            }
        }
        s_last_beat_us = beat_time_us;
    }

    s_prev_ir2 = s_prev_ir1;
    s_prev_ir1 = ir;
}

static esp_err_t max30102_read_fifo_pointers(uint8_t *write_ptr, uint8_t *read_ptr, uint8_t *overflow)
{
    esp_err_t err = max30102_read_u8(MAX30102_REG_FIFO_WR_PTR, write_ptr);
    if (err != ESP_OK) {
        return err;
    }

    err = max30102_read_u8(MAX30102_REG_FIFO_RD_PTR, read_ptr);
    if (err != ESP_OK) {
        return err;
    }

    err = max30102_read_u8(MAX30102_REG_OVF_COUNTER, overflow);
    if (err != ESP_OK) {
        return err;
    }

    *write_ptr &= 0x1F;
    *read_ptr &= 0x1F;
    *overflow &= 0x1F;
    return ESP_OK;
}

esp_err_t sensor_max30102_init(void)
{
    esp_err_t err = i2c_manager_probe_device(MAX30102_I2C_ADDR, pdMS_TO_TICKS(50));
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    uint8_t part_id = 0;
    err = max30102_read_u8(MAX30102_REG_PART_ID, &part_id);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }
    if (part_id != MAX30102_PART_ID) {
        s_initialized = false;
        return ESP_ERR_INVALID_RESPONSE;
    }


    err = max30102_write_u8(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    err = max30102_write_u8(MAX30102_REG_INTR_ENABLE_1, 0x00);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_INTR_ENABLE_2, 0x00);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_OVF_COUNTER, 0x00);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_FIFO_CONFIG, MAX30102_FIFO_CONFIG_VALUE);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_SPO2_CONFIG, MAX30102_SPO2_CONFIG_VALUE);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_LED1_PA, s_led_current_code);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_LED2_PA, s_led_current_code);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    err = max30102_write_u8(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    uint8_t clear_interrupts[2] = {0};
    err = max30102_read_multi(MAX30102_REG_INTR_STATUS_1, clear_interrupts, sizeof(clear_interrupts));
    if (err != ESP_OK) {
        s_initialized = false;
        return err;
    }

    s_initialized = true;
    max30102_reset_runtime_state();
    s_debug.initialized = true;
    ESP_LOGI(TAG, "MAX30102 detected at 0x%02X (part=0x%02X)", MAX30102_I2C_ADDR, part_id);
    return ESP_OK;
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
    uint8_t overflow = 0;
    esp_err_t err = max30102_read_fifo_pointers(&write_ptr, &read_ptr, &overflow);
    if (err != ESP_OK) {
        s_debug.consecutive_read_errors++;
        return err;
    }

    s_debug.fifo_write_ptr = write_ptr;
    s_debug.fifo_read_ptr = read_ptr;
    s_debug.fifo_overflow_count = overflow;

    uint8_t available_samples = (uint8_t)((write_ptr - read_ptr) & 0x1F);
    if (available_samples == 0U) {
        s_debug.consecutive_empty_reads++;
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t samples_to_read = available_samples;
    if (samples_to_read > MAX30102_MAX_BATCH_SAMPLES) {
        samples_to_read = MAX30102_MAX_BATCH_SAMPLES;
    }

    uint8_t raw[MAX30102_MAX_BATCH_SAMPLES * MAX30102_FIFO_SAMPLE_BYTES] = {0};
    err = max30102_read_multi(MAX30102_REG_FIFO_DATA,
                              raw,
                              (size_t)samples_to_read * MAX30102_FIFO_SAMPLE_BYTES);
    if (err != ESP_OK) {
        s_debug.consecutive_read_errors++;
        return err;
    }
    int64_t now_us = esp_timer_get_time();
    uint32_t red_min = UINT32_MAX;
    uint32_t red_max = 0;
    uint32_t ir_min = UINT32_MAX;
    uint32_t ir_max = 0;
    uint32_t red_sum = 0;
    uint32_t ir_sum = 0;
    for (uint8_t i = 0; i < samples_to_read; ++i) {
        const uint8_t *sample = &raw[i * MAX30102_FIFO_SAMPLE_BYTES];
        s_last_red = max30102_decode_sample(sample);
        s_last_ir = max30102_decode_sample(sample + 3);
        if (s_last_red < red_min) {
            red_min = s_last_red;
        }
        if (s_last_red > red_max) {
            red_max = s_last_red;
        }
        if (s_last_ir < ir_min) {
            ir_min = s_last_ir;
        }
        if (s_last_ir > ir_max) {
            ir_max = s_last_ir;
        }
        red_sum += s_last_red;
        ir_sum += s_last_ir;
        int64_t sample_time_us =
            now_us - ((int64_t)(samples_to_read - 1U - i) * MAX30102_ASSUMED_SAMPLE_PERIOD_US);
        max30102_process_ir_sample(s_last_ir, sample_time_us);
    }

    s_debug.consecutive_empty_reads = 0;
    s_debug.consecutive_read_errors = 0;
    s_debug.last_samples_drained = available_samples;
    s_debug.ir_dc_level = s_last_ir;
    s_debug.ir_ac_envelope = (s_last_ir > s_last_red) ? (s_last_ir - s_last_red) : (s_last_red - s_last_ir);
    s_debug.last_success_us = now_us;

    sensor_max30102_signal_hint_t signal_hint = max30102_classify_signal(s_last_ir, s_last_red);
    if (signal_hint != SENSOR_MAX30102_SIGNAL_OK) {
        max30102_clear_heart_rate();
    }
    max30102_update_spo2(red_min, red_max, red_sum, ir_min, ir_max, ir_sum, samples_to_read, signal_hint);

    out_reading->red = s_last_red;
    out_reading->ir = s_last_ir;
    out_reading->spo2_pct = s_spo2_pct;
    out_reading->spo2_valid = s_spo2_valid;
    out_reading->samples_drained = available_samples;
    out_reading->signal_hint = signal_hint;
    out_reading->heart_rate_valid = s_heart_rate_valid;
    out_reading->heart_rate_bpm = s_beat_avg;
    out_reading->heart_rate_confidence_pct = s_confidence_pct;
    return ESP_OK;
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

void sensor_max30102_get_debug_state(sensor_max30102_debug_t *out_state)
{
    if (out_state == NULL) {
        return;
    }

    *out_state = s_debug;
    out_state->initialized = s_initialized;
    out_state->led_current_code = s_led_current_code;
}
