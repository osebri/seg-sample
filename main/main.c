#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "adc_manager.h"
#include "blynk_bridge.h"
#include "display_ssd1306.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_manager.h"
#include "nvs_flash.h"
#include "project_config.h"
#include "sensor_adxl345.h"
#include "sensor_dht22.h"
#include "sensor_ky037.h"
#include "sensor_max30102.h"
#include "sensor_mlx90614.h"
#include "sensor_mq2.h"
#include "sensor_registry.h"

static const char *TAG = "phase1_main";

#define LED_POWER_GPIO GPIO_NUM_19
#define LED_SAFE_GPIO GPIO_NUM_18
#define LED_WARNING_GPIO GPIO_NUM_5
#define LED_CRITICAL_GPIO GPIO_NUM_23

#define DHT22_TASK_PERIOD_MS 3000
#define ANALOG_TASK_PERIOD_MS 50
#define ANALOG_KY037_PERIOD_MS 250
#define ANALOG_MQ135_PERIOD_MS 1000
#define I2C_TASK_PERIOD_MS 100
#define I2C_MLX90614_PERIOD_MS 2000
#define I2C_ADXL345_PERIOD_MS 1000
#define I2C_OLED_PERIOD_MS 1000
#define LOG_TASK_PERIOD_MS 3000
#define INIT_RETRY_PERIOD_MS 15000
#define I2C_RESCAN_PERIOD_MS 30000
#define DHT22_STALE_AFTER_MS 9000
#define MLX90614_STALE_AFTER_MS 6000
#define MAX30102_STALE_AFTER_MS 500
#define KY037_STALE_AFTER_MS 1500
#define MQ135_STALE_AFTER_MS 4000
#define ADXL345_STALE_AFTER_MS 4000
#define OLED_STALE_AFTER_MS 4000
#define LED_TASK_PERIOD_MS 100

#define WARNING_HUMIDITY_THRESHOLD_PCT 55.0f
#define WARNING_SOUND_ACTIVITY_THRESHOLD_PCT 0
#define WARNING_MOVEMENT_THRESHOLD_MG 120

#define CRITICAL_BODY_TEMP_LOW_C 36.0f
#define CRITICAL_BODY_TEMP_HIGH_C 37.5f
#define CRITICAL_ROOM_TEMP_LOW_C 20.0f
#define CRITICAL_ROOM_TEMP_HIGH_C 22.2f
#define CRITICAL_AIR_QUALITY_THRESHOLD_PCT 70

typedef enum {
    LED_STATE_UNKNOWN = 0,
    LED_STATE_NORMAL,
    LED_STATE_WARNING,
    LED_STATE_CRITICAL,
} led_state_t;

static void initialize_leds(void)
{
    const uint64_t pin_mask = (1ULL << LED_POWER_GPIO) |
                              (1ULL << LED_SAFE_GPIO) |
                              (1ULL << LED_WARNING_GPIO) |
                              (1ULL << LED_CRITICAL_GPIO);

    const gpio_config_t io_cfg = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&io_cfg));
    ESP_ERROR_CHECK(gpio_set_level(LED_POWER_GPIO, 1));
    ESP_ERROR_CHECK(gpio_set_level(LED_SAFE_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(LED_WARNING_GPIO, 0));
    ESP_ERROR_CHECK(gpio_set_level(LED_CRITICAL_GPIO, 0));
}

static bool sensor_has_led_value(const sensor_snapshot_t *snapshot, sensor_id_t sensor_id)
{
    if (snapshot == NULL) {
        return false;
    }

    sensor_state_t state = snapshot->status[sensor_id].state;
    return state == SENSOR_STATE_HEALTHY || state == SENSOR_STATE_WARMING_UP;
}

static bool movement_warning_detected(const sensor_snapshot_t *snapshot)
{
    if (!sensor_has_led_value(snapshot, SENSOR_ID_ADXL345)) {
        return false;
    }

    return snapshot->adxl_motion_delta_mg >= WARNING_MOVEMENT_THRESHOLD_MG;
}

static bool led_warning_condition_exists(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    if (sensor_has_led_value(snapshot, SENSOR_ID_DHT22) &&
        snapshot->dht_humidity_pct > WARNING_HUMIDITY_THRESHOLD_PCT) {
        return true;
    }

    if (sensor_has_led_value(snapshot, SENSOR_ID_KY037) &&
        snapshot->ky037_activity_pct > WARNING_SOUND_ACTIVITY_THRESHOLD_PCT) {
        return true;
    }

    return movement_warning_detected(snapshot);
}

static bool led_critical_condition_exists(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    if (sensor_has_led_value(snapshot, SENSOR_ID_MLX90614) &&
        (snapshot->mlx_object_temp_c < CRITICAL_BODY_TEMP_LOW_C ||
         snapshot->mlx_object_temp_c > CRITICAL_BODY_TEMP_HIGH_C)) {
        return true;
    }

    if (sensor_has_led_value(snapshot, SENSOR_ID_DHT22) &&
        (snapshot->dht_temp_c < CRITICAL_ROOM_TEMP_LOW_C ||
         snapshot->dht_temp_c > CRITICAL_ROOM_TEMP_HIGH_C)) {
        return true;
    }

    if (sensor_has_led_value(snapshot, SENSOR_ID_MQ135) &&
        snapshot->mq135_response_pct > CRITICAL_AIR_QUALITY_THRESHOLD_PCT) {
        return true;
    }

    return false;
}

static bool led_normal_state_ready(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    if (!sensor_has_led_value(snapshot, SENSOR_ID_DHT22) ||
        !sensor_has_led_value(snapshot, SENSOR_ID_MLX90614) ||
        !sensor_has_led_value(snapshot, SENSOR_ID_KY037) ||
        !sensor_has_led_value(snapshot, SENSOR_ID_MQ135)) {
        return false;
    }

    const sensor_status_t *adxl_status = &snapshot->status[SENSOR_ID_ADXL345];
    return sensor_has_led_value(snapshot, SENSOR_ID_ADXL345) ||
           adxl_status->state == SENSOR_STATE_ABSENT_OPTIONAL;
}

static led_state_t determine_led_state(const sensor_snapshot_t *snapshot)
{
    if (led_critical_condition_exists(snapshot)) {
        return LED_STATE_CRITICAL;
    }

    if (led_warning_condition_exists(snapshot)) {
        return LED_STATE_WARNING;
    }

    if (led_normal_state_ready(snapshot)) {
        return LED_STATE_NORMAL;
    }

    return LED_STATE_UNKNOWN;
}

static void apply_led_state(led_state_t state)
{
    ESP_ERROR_CHECK(gpio_set_level(LED_POWER_GPIO, 1));
    ESP_ERROR_CHECK(gpio_set_level(LED_SAFE_GPIO, state == LED_STATE_NORMAL));
    ESP_ERROR_CHECK(gpio_set_level(LED_WARNING_GPIO, state == LED_STATE_WARNING));
    ESP_ERROR_CHECK(gpio_set_level(LED_CRITICAL_GPIO, state == LED_STATE_CRITICAL));
}

static void initialize_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static bool sensor_has_success(const sensor_status_t *status)
{
    return status != NULL && status->last_success_us != 0;
}

static int64_t sensor_age_ms(const sensor_status_t *status, int64_t now_us)
{
    if (!sensor_has_success(status)) {
        return -1;
    }

    return (now_us - status->last_success_us) / 1000LL;
}

static bool sensor_is_stale(const sensor_status_t *status, int64_t now_us)
{
    return status != NULL &&
           sensor_has_success(status) &&
           status->stale_after_us > 0 &&
           (now_us - status->last_success_us) > status->stale_after_us;
}

static sensor_state_t next_state_after_hard_failure(const sensor_status_t *status, int64_t now_us)
{
    if (status == NULL || !status->initialized || !sensor_has_success(status)) {
        return SENSOR_STATE_ERROR;
    }
    if (status->state == SENSOR_STATE_WARMING_UP) {
        return SENSOR_STATE_WARMING_UP;
    }
    if (sensor_is_stale(status, now_us)) {
        return SENSOR_STATE_STALE;
    }
    return SENSOR_STATE_HEALTHY;
}

static sensor_state_t next_state_after_no_sample(const sensor_status_t *status,
                                                 int64_t now_us,
                                                 sensor_state_t waiting_state)
{
    if (status == NULL || !status->initialized || !sensor_has_success(status)) {
        return waiting_state;
    }
    if (status->state == SENSOR_STATE_WARMING_UP) {
        return SENSOR_STATE_WARMING_UP;
    }
    if (sensor_is_stale(status, now_us)) {
        return SENSOR_STATE_STALE;
    }
    return SENSOR_STATE_HEALTHY;
}

static void register_sensor_configs(void)
{
    sensor_registry_configure(SENSOR_ID_DHT22, false, DHT22_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_MLX90614, false, MLX90614_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_MAX30102, false, MAX30102_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_KY037, false, KY037_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_MQ135, false, MQ135_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_ADXL345, true, ADXL345_STALE_AFTER_MS * 1000LL);
    sensor_registry_configure(SENSOR_ID_OLED, false, OLED_STALE_AFTER_MS * 1000LL);
}

static void register_init_result(sensor_id_t sensor_id, esp_err_t err, bool optional_absent)
{
    sensor_state_t failure_state = optional_absent ? SENSOR_STATE_ABSENT_OPTIONAL : SENSOR_STATE_INIT_FAILED;
    sensor_registry_set_init_result(sensor_id, err, failure_state);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[%s] init OK", sensor_registry_name(sensor_id));
        return;
    }

    if (optional_absent) {
        ESP_LOGI(TAG, "[%s] optional sensor absent: %s", sensor_registry_name(sensor_id), esp_err_to_name(err));
        return;
    }

    ESP_LOGW(TAG, "[%s] init FAILED: %s", sensor_registry_name(sensor_id), esp_err_to_name(err));
}

static bool adxl345_is_optional_absent_error(esp_err_t err)
{
    return err == ESP_ERR_NOT_FOUND || err == ESP_ERR_INVALID_RESPONSE || err == ESP_ERR_TIMEOUT;
}

static bool any_primary_i2c_sensor_initialized(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return false;
    }

    return snapshot->status[SENSOR_ID_MLX90614].initialized ||
           snapshot->status[SENSOR_ID_MAX30102].initialized ||
           snapshot->status[SENSOR_ID_OLED].initialized;
}

static void retry_i2c_sensor_inits(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }

    if (snapshot->status[SENSOR_ID_MLX90614].state == SENSOR_STATE_INIT_FAILED) {
        register_init_result(SENSOR_ID_MLX90614, sensor_mlx90614_init(), false);
    }

    if (snapshot->status[SENSOR_ID_MAX30102].state == SENSOR_STATE_INIT_FAILED) {
        register_init_result(SENSOR_ID_MAX30102, sensor_max30102_init(), false);
    }

    const sensor_state_t adxl_state = snapshot->status[SENSOR_ID_ADXL345].state;
    if (adxl_state == SENSOR_STATE_INIT_FAILED ||
        adxl_state == SENSOR_STATE_ABSENT_OPTIONAL) {
        esp_err_t adxl_err = sensor_adxl345_init();
        register_init_result(SENSOR_ID_ADXL345,
                             adxl_err,
                             adxl345_is_optional_absent_error(adxl_err));
    }

#if ENABLE_OLED_SUMMARY
    if (snapshot->status[SENSOR_ID_OLED].state == SENSOR_STATE_INIT_FAILED) {
        register_init_result(SENSOR_ID_OLED, display_ssd1306_init(), false);
    }
#endif
}

static void maybe_retry_dht22_init(const sensor_status_t *status, int64_t now_us)
{
    if (status == NULL || status->state != SENSOR_STATE_INIT_FAILED) {
        return;
    }

    if (status->last_attempt_us != 0 &&
        (now_us - status->last_attempt_us) < (INIT_RETRY_PERIOD_MS * 1000LL)) {
        return;
    }

    register_init_result(SENSOR_ID_DHT22, sensor_dht22_init(DHT22_DATA_GPIO), false);
}

static void task_dht22(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        sensor_snapshot_t snapshot;
        sensor_registry_get_snapshot(&snapshot);

        int64_t now_us = esp_timer_get_time();
        const sensor_status_t *status = &snapshot.status[SENSOR_ID_DHT22];
        maybe_retry_dht22_init(status, now_us);

        if (status->initialized) {
            sensor_registry_note_attempt(SENSOR_ID_DHT22);

            float temp_c = 0.0f;
            float humidity_pct = 0.0f;
            esp_err_t err = sensor_dht22_read(&temp_c, &humidity_pct);
            if (err == ESP_OK) {
                sensor_registry_update_dht22(temp_c, humidity_pct, SENSOR_STATE_HEALTHY);
            } else if (err == ESP_ERR_INVALID_STATE) {
                if (!sensor_has_success(status)) {
                    sensor_registry_set_state(SENSOR_ID_DHT22, SENSOR_STATE_WAITING_FIRST_SAMPLE, ESP_OK);
                } else if (sensor_is_stale(status, now_us)) {
                    sensor_registry_record_failure(SENSOR_ID_DHT22, SENSOR_STATE_STALE, ESP_ERR_TIMEOUT);
                }
            } else {
                sensor_state_t next_state = next_state_after_hard_failure(status, now_us);
                uint32_t next_failures = status->consecutive_failures + 1U;
                sensor_registry_record_failure(SENSOR_ID_DHT22, next_state, err);

                if (next_failures == 1U || (next_failures % 4U) == 0U) {
                    ESP_LOGW(TAG, "[DHT22] read error: %s (failures=%lu)", esp_err_to_name(err), (unsigned long)next_failures);
                }

                if (next_failures >= 4U) {
                    register_init_result(SENSOR_ID_DHT22, sensor_dht22_init(DHT22_DATA_GPIO), false);
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DHT22_TASK_PERIOD_MS));
    }
}

static void task_analog(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_ky037_us = 0;
    int64_t last_mq135_us = 0;

    while (true) {
        int64_t now_us = esp_timer_get_time();
        sensor_snapshot_t snapshot;
        sensor_registry_get_snapshot(&snapshot);

        const sensor_status_t *ky037_status = &snapshot.status[SENSOR_ID_KY037];
        if (ky037_status->initialized &&
            (now_us - last_ky037_us) >= (ANALOG_KY037_PERIOD_MS * 1000LL)) {
            last_ky037_us = now_us;
            sensor_registry_note_attempt(SENSOR_ID_KY037);

            sensor_ky037_reading_t reading = {0};
            esp_err_t err = sensor_ky037_read_activity(&reading);
            if (err == ESP_OK) {
                sensor_registry_update_ky037(reading.activity_pct,
                                             reading.peak_to_peak,
                                             reading.mean_abs_deviation,
                                             SENSOR_STATE_HEALTHY);
            } else {
                uint32_t next_failures = ky037_status->consecutive_failures + 1U;
                sensor_registry_record_failure(SENSOR_ID_KY037,
                                               next_state_after_hard_failure(ky037_status, now_us),
                                               err);
                if (next_failures == 1U || (next_failures % 8U) == 0U) {
                    ESP_LOGW(TAG, "[KY037] read error: %s (failures=%lu)", esp_err_to_name(err), (unsigned long)next_failures);
                }
            }
        }

        const sensor_status_t *mq135_status = &snapshot.status[SENSOR_ID_MQ135];
        if (mq135_status->initialized &&
            (now_us - last_mq135_us) >= (ANALOG_MQ135_PERIOD_MS * 1000LL)) {
            last_mq135_us = now_us;
            sensor_registry_note_attempt(SENSOR_ID_MQ135);

            sensor_mq135_reading_t reading = {0};
            esp_err_t err = sensor_mq2_read_metrics(&reading);
            if (err == ESP_OK) {
                sensor_registry_update_mq135(reading.filtered_raw,
                                             reading.baseline_raw,
                                             reading.delta_raw,
                                             reading.response_pct,
                                             reading.warming_up ? SENSOR_STATE_WARMING_UP : SENSOR_STATE_HEALTHY);
            } else {
                uint32_t next_failures = mq135_status->consecutive_failures + 1U;
                sensor_state_t next_state = next_state_after_hard_failure(mq135_status, now_us);
                if (mq135_status->state == SENSOR_STATE_WARMING_UP) {
                    next_state = SENSOR_STATE_WARMING_UP;
                }
                sensor_registry_record_failure(SENSOR_ID_MQ135, next_state, err);
                if (next_failures == 1U || (next_failures % 6U) == 0U) {
                    ESP_LOGW(TAG, "[MQ135] read error: %s (failures=%lu)", esp_err_to_name(err), (unsigned long)next_failures);
                }
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ANALOG_TASK_PERIOD_MS));
    }
}

static void task_i2c_sensors(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_retry_us = 0;
    int64_t last_scan_us = 0;
    int64_t last_mlx90614_us = 0;
    int64_t last_adxl345_us = 0;
    int64_t last_oled_us = 0;
    bool have_last_adxl_sample = false;
    int16_t last_adxl_x_mg = 0;
    int16_t last_adxl_y_mg = 0;
    int16_t last_adxl_z_mg = 0;

    while (true) {
        int64_t now_us = esp_timer_get_time();
        sensor_snapshot_t snapshot;
        sensor_registry_get_snapshot(&snapshot);

        if ((now_us - last_retry_us) >= (INIT_RETRY_PERIOD_MS * 1000LL)) {
            retry_i2c_sensor_inits(&snapshot);
            last_retry_us = now_us;
            sensor_registry_get_snapshot(&snapshot);
        }

        if (!any_primary_i2c_sensor_initialized(&snapshot) &&
            (now_us - last_scan_us) >= (I2C_RESCAN_PERIOD_MS * 1000LL)) {
            ESP_LOGW(TAG, "[I2C] no primary I2C sensors initialized, rescanning shared bus");
            (void)i2c_manager_scan();
            last_scan_us = now_us;
        }

        const sensor_status_t *mlx_status = &snapshot.status[SENSOR_ID_MLX90614];
        if (mlx_status->initialized &&
            (now_us - last_mlx90614_us) >= (I2C_MLX90614_PERIOD_MS * 1000LL)) {
            last_mlx90614_us = now_us;
            sensor_registry_note_attempt(SENSOR_ID_MLX90614);

            float object_temp_c = 0.0f;
            float ambient_temp_c = 0.0f;
            esp_err_t err = sensor_mlx90614_read_temperatures(&object_temp_c, &ambient_temp_c);
            if (err == ESP_OK) {
                sensor_registry_update_mlx90614(object_temp_c, ambient_temp_c, SENSOR_STATE_HEALTHY);
            } else {
                uint32_t next_failures = mlx_status->consecutive_failures + 1U;
                sensor_registry_record_failure(SENSOR_ID_MLX90614,
                                               next_state_after_hard_failure(mlx_status, now_us),
                                               err);
                if (next_failures == 1U || (next_failures % 5U) == 0U) {
                    ESP_LOGW(TAG, "[MLX90614] read error: %s (failures=%lu)", esp_err_to_name(err), (unsigned long)next_failures);
                }
            }
        }

        const sensor_status_t *max_status = &snapshot.status[SENSOR_ID_MAX30102];
        if (max_status->initialized) {
            sensor_registry_note_attempt(SENSOR_ID_MAX30102);

            sensor_max30102_reading_t reading = {0};
            esp_err_t err = sensor_max30102_read_latest(&reading);
            if (err == ESP_OK) {
                sensor_registry_update_max30102(reading.red,
                                                reading.ir,
                                                reading.spo2_pct,
                                                reading.spo2_valid,
                                                0U,
                                                reading.samples_drained,
                                                reading.signal_hint,
                                                reading.heart_rate_valid,
                                                reading.heart_rate_bpm,
                                                reading.heart_rate_confidence_pct,
                                                SENSOR_STATE_HEALTHY);
            } else {
                int64_t age_ms = sensor_age_ms(max_status, now_us);
                if (age_ms < 0) {
                    age_ms = 0;
                }
                sensor_registry_update_max30102_runtime((uint32_t)age_ms,
                                                        snapshot.max30102_spo2_pct,
                                                        snapshot.max30102_spo2_valid,
                                                        0U,
                                                        snapshot.max30102_signal_hint,
                                                        snapshot.max30102_heart_rate_valid,
                                                        snapshot.max30102_heart_rate_bpm,
                                                        snapshot.max30102_heart_rate_confidence_pct);

                if (err == ESP_ERR_NOT_FOUND) {
                    sensor_registry_set_state(SENSOR_ID_MAX30102,
                                              next_state_after_no_sample(max_status,
                                                                         now_us,
                                                                         SENSOR_STATE_WAITING_FIRST_SAMPLE),
                                              ESP_OK);
                } else {
                    uint32_t next_failures = max_status->consecutive_failures + 1U;
                    sensor_registry_record_failure(SENSOR_ID_MAX30102,
                                                   next_state_after_hard_failure(max_status, now_us),
                                                   err);
                    if (next_failures == 1U || (next_failures % 5U) == 0U) {
                        ESP_LOGW(TAG, "[MAX30102] read error: %s (failures=%lu)", esp_err_to_name(err), (unsigned long)next_failures);
                    }

                    if (next_failures >= 5U) {
                        register_init_result(SENSOR_ID_MAX30102, sensor_max30102_init(), false);
                    }
                }
            }
        }

        const sensor_status_t *adxl_status = &snapshot.status[SENSOR_ID_ADXL345];
        if (adxl_status->initialized &&
            (now_us - last_adxl345_us) >= (I2C_ADXL345_PERIOD_MS * 1000LL)) {
            last_adxl345_us = now_us;
            sensor_registry_note_attempt(SENSOR_ID_ADXL345);

            int16_t x_raw = 0;
            int16_t y_raw = 0;
            int16_t z_raw = 0;
            esp_err_t err = sensor_adxl345_read_xyz(&x_raw, &y_raw, &z_raw);
            if (err == ESP_OK) {
                int16_t x_mg = sensor_adxl345_raw_to_mg(x_raw);
                int16_t y_mg = sensor_adxl345_raw_to_mg(y_raw);
                int16_t z_mg = sensor_adxl345_raw_to_mg(z_raw);
                uint16_t motion_delta_mg = 0;

                if (have_last_adxl_sample) {
                    int dx_mg = abs((int)x_mg - (int)last_adxl_x_mg);
                    int dy_mg = abs((int)y_mg - (int)last_adxl_y_mg);
                    int dz_mg = abs((int)z_mg - (int)last_adxl_z_mg);
                    int max_delta_mg = dx_mg;

                    if (dy_mg > max_delta_mg) {
                        max_delta_mg = dy_mg;
                    }
                    if (dz_mg > max_delta_mg) {
                        max_delta_mg = dz_mg;
                    }

                    motion_delta_mg = (uint16_t)max_delta_mg;
                } else {
                    have_last_adxl_sample = true;
                }

                last_adxl_x_mg = x_mg;
                last_adxl_y_mg = y_mg;
                last_adxl_z_mg = z_mg;

                sensor_registry_update_adxl345(x_mg, y_mg, z_mg, motion_delta_mg, SENSOR_STATE_HEALTHY);
            } else {
                sensor_registry_record_failure(SENSOR_ID_ADXL345,
                                               next_state_after_hard_failure(adxl_status, now_us),
                                               err);
            }
        }

#if ENABLE_OLED_SUMMARY
        const sensor_status_t *oled_status = &snapshot.status[SENSOR_ID_OLED];
        if (oled_status->initialized &&
            (now_us - last_oled_us) >= (I2C_OLED_PERIOD_MS * 1000LL)) {
            last_oled_us = now_us;
            sensor_registry_get_snapshot(&snapshot);
            sensor_registry_note_attempt(SENSOR_ID_OLED);

            esp_err_t err = display_ssd1306_render_summary(&snapshot);
            if (err == ESP_OK) {
                sensor_registry_update_oled(SENSOR_STATE_HEALTHY, ESP_OK);
            } else {
                uint32_t next_failures = oled_status->consecutive_failures + 1U;
                sensor_registry_update_oled(SENSOR_STATE_ERROR, err);
                if (next_failures >= 3U) {
                    register_init_result(SENSOR_ID_OLED, display_ssd1306_init(), false);
                }
            }
        }
#endif

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(I2C_TASK_PERIOD_MS));
    }
}

static void log_state_transition(sensor_id_t sensor_id, const sensor_status_t *status)
{
    static bool initialized[SENSOR_ID_COUNT];
    static sensor_state_t last_state[SENSOR_ID_COUNT];

    if (status == NULL) {
        return;
    }

    if (!initialized[sensor_id]) {
        initialized[sensor_id] = true;
        last_state[sensor_id] = status->state;
        return;
    }

    if (last_state[sensor_id] == status->state) {
        return;
    }

    const char *err_name = esp_err_to_name(status->last_error);
    if (status->state == SENSOR_STATE_STALE ||
        status->state == SENSOR_STATE_ERROR ||
        status->state == SENSOR_STATE_INIT_FAILED) {
        ESP_LOGW(TAG,
                 "[%s] transition=%s err=%s failures=%lu",
                 sensor_registry_name(sensor_id),
                 sensor_registry_state_name(status->state),
                 err_name,
                 (unsigned long)status->consecutive_failures);
    } else {
        ESP_LOGI(TAG,
                 "[%s] transition=%s err=%s failures=%lu",
                 sensor_registry_name(sensor_id),
                 sensor_registry_state_name(status->state),
                 err_name,
                 (unsigned long)status->consecutive_failures);
    }

    last_state[sensor_id] = status->state;
}

static void log_dht22_summary(const sensor_snapshot_t *snapshot, int64_t now_us)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_DHT22];
    int64_t age_ms = sensor_age_ms(status, now_us);

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        if (status->consecutive_failures > 0U && status->last_error != ESP_OK) {
            ESP_LOGI(TAG,
                     "[DHT22] state=HEALTHY temp=%.1fC hum=%.1f%% age=%lldms recent_err=%s fail=%lu",
                     snapshot->dht_temp_c,
                     snapshot->dht_humidity_pct,
                     (long long)age_ms,
                     esp_err_to_name(status->last_error),
                     (unsigned long)status->consecutive_failures);
        } else {
            ESP_LOGI(TAG,
                     "[DHT22] state=HEALTHY temp=%.1fC hum=%.1f%% age=%lldms",
                     snapshot->dht_temp_c,
                     snapshot->dht_humidity_pct,
                     (long long)age_ms);
        }
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG, "[DHT22] state=WAITING_FIRST_SAMPLE");
        break;
    case SENSOR_STATE_STALE:
        ESP_LOGW(TAG,
                 "[DHT22] state=STALE age=%lldms last_temp=%.1fC last_hum=%.1f%% err=%s",
                 (long long)age_ms,
                 snapshot->dht_temp_c,
                 snapshot->dht_humidity_pct,
                 esp_err_to_name(status->last_error));
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG, "[DHT22] state=%s err=%s", sensor_registry_state_name(status->state), esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void log_mlx90614_summary(const sensor_snapshot_t *snapshot, int64_t now_us)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_MLX90614];
    int64_t age_ms = sensor_age_ms(status, now_us);

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG,
                 "[MLX90614] state=HEALTHY object=%.1fC ambient=%.1fC age=%lldms",
                 snapshot->mlx_object_temp_c,
                 snapshot->mlx_ambient_temp_c,
                 (long long)age_ms);
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG, "[MLX90614] state=WAITING_FIRST_SAMPLE");
        break;
    case SENSOR_STATE_STALE:
        ESP_LOGW(TAG,
                 "[MLX90614] state=STALE age=%lldms last_object=%.1fC last_ambient=%.1fC err=%s",
                 (long long)age_ms,
                 snapshot->mlx_object_temp_c,
                 snapshot->mlx_ambient_temp_c,
                 esp_err_to_name(status->last_error));
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG,
                 "[MLX90614] state=%s err=%s",
                 sensor_registry_state_name(status->state),
                 esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void log_ky037_summary(const sensor_snapshot_t *snapshot, int64_t now_us)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_KY037];
    int64_t age_ms = sensor_age_ms(status, now_us);

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG,
                 "[KY037] state=HEALTHY activity=%d%% p2p=%d mad=%d age=%lldms",
                 snapshot->ky037_activity_pct,
                 snapshot->ky037_peak_to_peak,
                 snapshot->ky037_mean_abs_deviation,
                 (long long)age_ms);
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG, "[KY037] state=WAITING_FIRST_SAMPLE");
        break;
    case SENSOR_STATE_STALE:
        ESP_LOGW(TAG,
                 "[KY037] state=STALE age=%lldms last_activity=%d%% last_p2p=%d err=%s",
                 (long long)age_ms,
                 snapshot->ky037_activity_pct,
                 snapshot->ky037_peak_to_peak,
                 esp_err_to_name(status->last_error));
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG,
                 "[KY037] state=%s err=%s",
                 sensor_registry_state_name(status->state),
                 esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void log_mq135_summary(const sensor_snapshot_t *snapshot, int64_t now_us)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_MQ135];
    int64_t age_ms = sensor_age_ms(status, now_us);

    switch (status->state) {
    case SENSOR_STATE_WARMING_UP:
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG,
                 "[MQ135] state=%s response=%d%% delta=%+d filtered=%d baseline=%d age=%lldms",
                 sensor_registry_state_name(status->state),
                 snapshot->mq135_response_pct,
                 snapshot->mq135_delta_raw,
                 snapshot->mq135_filtered_raw,
                 snapshot->mq135_baseline_raw,
                 (long long)age_ms);
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG, "[MQ135] state=WAITING_FIRST_SAMPLE");
        break;
    case SENSOR_STATE_STALE:
        ESP_LOGW(TAG,
                 "[MQ135] state=STALE age=%lldms last_response=%d%% last_delta=%+d err=%s",
                 (long long)age_ms,
                 snapshot->mq135_response_pct,
                 snapshot->mq135_delta_raw,
                 esp_err_to_name(status->last_error));
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG,
                 "[MQ135] state=%s err=%s",
                 sensor_registry_state_name(status->state),
                 esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void log_max30102_summary(const sensor_snapshot_t *snapshot)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_MAX30102];
    sensor_max30102_debug_t debug_state = {0};
    sensor_max30102_get_debug_state(&debug_state);

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG,
                 "[MAX30102] red=%lu ir=%lu spo2=%u bpm=%u conf=%u%%",
                 (unsigned long)snapshot->max30102_red,
                 (unsigned long)snapshot->max30102_ir,
                 (unsigned int)(snapshot->max30102_spo2_valid ? snapshot->max30102_spo2_pct : 0U),
                 (unsigned int)snapshot->max30102_heart_rate_bpm,
                 (unsigned int)snapshot->max30102_heart_rate_confidence_pct);
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG,
                 "[MAX30102] state=WAITING_FIRST_SAMPLE empty=%lu",
                 (unsigned long)debug_state.consecutive_empty_reads);
        break;
    case SENSOR_STATE_STALE:
        ESP_LOGW(TAG,
                 "[MAX30102] state=STALE age=%ldms last_red=%lu last_ir=%lu hint=%s fifo_wr=%u fifo_rd=%u ovf=%u empty=%lu",
                 (long)snapshot->max30102_sample_age_ms,
                 (unsigned long)snapshot->max30102_red,
                 (unsigned long)snapshot->max30102_ir,
                 sensor_max30102_signal_hint_name(snapshot->max30102_signal_hint),
                 debug_state.fifo_write_ptr,
                 debug_state.fifo_read_ptr,
                 debug_state.fifo_overflow_count,
                 (unsigned long)debug_state.consecutive_empty_reads);
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG,
                 "[MAX30102] state=%s err=%s age=%ldms fifo_wr=%u fifo_rd=%u ovf=%u read_err=%lu",
                 sensor_registry_state_name(status->state),
                 esp_err_to_name(status->last_error),
                 (long)snapshot->max30102_sample_age_ms,
                 debug_state.fifo_write_ptr,
                 debug_state.fifo_read_ptr,
                 debug_state.fifo_overflow_count,
                 (unsigned long)debug_state.consecutive_read_errors);
        break;
    default:
        break;
    }
}

static void log_adxl345_summary(const sensor_snapshot_t *snapshot, int64_t now_us)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_ADXL345];
    int64_t age_ms = sensor_age_ms(status, now_us);

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG,
                 "[ADXL345] state=HEALTHY x=%dmg y=%dmg z=%dmg d=%umg age=%lldms",
                 snapshot->adxl_x_mg,
                 snapshot->adxl_y_mg,
                 snapshot->adxl_z_mg,
                 (unsigned int)snapshot->adxl_motion_delta_mg,
                 (long long)age_ms);
        break;
    case SENSOR_STATE_ABSENT_OPTIONAL:
        ESP_LOGI(TAG, "[ADXL345] state=ABSENT_OPTIONAL err=%s", esp_err_to_name(status->last_error));
        break;
    case SENSOR_STATE_STALE:
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG,
                 "[ADXL345] state=%s err=%s",
                 sensor_registry_state_name(status->state),
                 esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void log_oled_summary(const sensor_snapshot_t *snapshot)
{
    const sensor_status_t *status = &snapshot->status[SENSOR_ID_OLED];

    switch (status->state) {
    case SENSOR_STATE_HEALTHY:
        ESP_LOGI(TAG, "[OLED] state=HEALTHY");
        break;
    case SENSOR_STATE_WAITING_FIRST_SAMPLE:
        ESP_LOGI(TAG, "[OLED] state=WAITING_FIRST_SAMPLE");
        break;
    case SENSOR_STATE_ERROR:
    case SENSOR_STATE_INIT_FAILED:
        ESP_LOGW(TAG, "[OLED] state=%s err=%s", sensor_registry_state_name(status->state), esp_err_to_name(status->last_error));
        break;
    default:
        break;
    }
}

static void task_logger(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        sensor_snapshot_t snapshot;
        sensor_registry_get_snapshot(&snapshot);

        for (int sensor_id = 0; sensor_id < SENSOR_ID_COUNT; ++sensor_id) {
            log_state_transition((sensor_id_t)sensor_id, &snapshot.status[sensor_id]);
        }

        int64_t now_us = esp_timer_get_time();
        log_dht22_summary(&snapshot, now_us);
        log_mlx90614_summary(&snapshot, now_us);
        log_ky037_summary(&snapshot, now_us);
        log_mq135_summary(&snapshot, now_us);
        log_max30102_summary(&snapshot);
        log_adxl345_summary(&snapshot, now_us);
        log_oled_summary(&snapshot);
        blynk_bridge_publish_snapshot(&snapshot);

        int64_t uptime_s = (esp_timer_get_time() - snapshot.boot_time_us) / 1000000LL;
        ESP_LOGI(TAG, "[HEARTBEAT] uptime=%llds", (long long)uptime_s);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
    }
}

static void task_leds(void *arg)
{
    (void)arg;

    TickType_t last_wake = xTaskGetTickCount();
    led_state_t last_state = LED_STATE_UNKNOWN;

    while (true) {
        sensor_snapshot_t snapshot;
        sensor_registry_get_snapshot(&snapshot);

        led_state_t next_state = determine_led_state(&snapshot);
        apply_led_state(next_state);

        if (next_state != last_state) {
            ESP_LOGI(TAG,
                     "[LED] power=ON safe=%s warning=%s critical=%s",
                     next_state == LED_STATE_NORMAL ? "ON" : "OFF",
                     next_state == LED_STATE_WARNING ? "ON" : "OFF",
                     next_state == LED_STATE_CRITICAL ? "ON" : "OFF");
            last_state = next_state;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LED_TASK_PERIOD_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Production-hardening firmware booting...");

    initialize_nvs();
    initialize_leds();
    sensor_registry_init();
    register_sensor_configs();

    esp_err_t blynk_err = blynk_bridge_init();
    if (blynk_err != ESP_OK) {
        ESP_LOGW(TAG, "Blynk bridge init incomplete: %s", esp_err_to_name(blynk_err));
    }

    esp_err_t i2c_err = i2c_manager_init();
    if (i2c_err == ESP_OK) {
        (void)i2c_manager_scan();
    } else {
        ESP_LOGW(TAG, "I2C init failed: %s", esp_err_to_name(i2c_err));
    }

    esp_err_t adc_err = adc_manager_init();
    if (adc_err != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed: %s", esp_err_to_name(adc_err));
    }

    register_init_result(SENSOR_ID_DHT22, sensor_dht22_init(DHT22_DATA_GPIO), false);

    if (adc_err == ESP_OK) {
        register_init_result(SENSOR_ID_KY037, sensor_ky037_init(), false);
        register_init_result(SENSOR_ID_MQ135, sensor_mq2_init(), false);
    } else {
        register_init_result(SENSOR_ID_KY037, adc_err, false);
        register_init_result(SENSOR_ID_MQ135, adc_err, false);
    }

    if (i2c_err == ESP_OK) {
        register_init_result(SENSOR_ID_MLX90614, sensor_mlx90614_init(), false);
        register_init_result(SENSOR_ID_MAX30102, sensor_max30102_init(), false);

        esp_err_t adxl_err = sensor_adxl345_init();
        register_init_result(SENSOR_ID_ADXL345, adxl_err, adxl345_is_optional_absent_error(adxl_err));

#if ENABLE_OLED_SUMMARY
        register_init_result(SENSOR_ID_OLED, display_ssd1306_init(), false);
#else
        register_init_result(SENSOR_ID_OLED, ESP_ERR_NOT_SUPPORTED, false);
#endif
    } else {
        register_init_result(SENSOR_ID_MLX90614, i2c_err, false);
        register_init_result(SENSOR_ID_MAX30102, i2c_err, false);
        register_init_result(SENSOR_ID_ADXL345, i2c_err, false);
        register_init_result(SENSOR_ID_OLED, i2c_err, false);
    }

    xTaskCreate(task_dht22, "task_dht22", 4096, NULL, 5, NULL);
    xTaskCreate(task_analog, "task_analog", 4096, NULL, 5, NULL);
    xTaskCreate(task_i2c_sensors, "task_i2c", 4096, NULL, 5, NULL);
    xTaskCreate(task_logger, "task_logger", 6144, NULL, 4, NULL);
    xTaskCreate(task_leds, "task_leds", 3072, NULL, 4, NULL);
    xTaskCreate(blynk_bridge_task, "task_blynk", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Production-hardening tasks started.");
}
