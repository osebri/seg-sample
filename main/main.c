#include <stdbool.h>
#include <stdint.h>

#include "adc_manager.h"
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

#define DHT22_TASK_PERIOD_MS 3000
#define I2C_TASK_PERIOD_MS 2000
#define ANALOG_TASK_PERIOD_MS 1500
#define LOG_TASK_PERIOD_MS 3000
#define OLED_TASK_PERIOD_MS 1500
#define I2C_RETRY_INIT_PERIOD_MS 15000
#define I2C_RESCAN_PERIOD_MS 30000

static void initialize_nvs(void)
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);
}

static void register_init_result(sensor_id_t sensor_id, esp_err_t err)
{
	sensor_registry_set_init_status(sensor_id, err == ESP_OK, err);
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "[%s] init OK", sensor_registry_name(sensor_id));
	} else {
		ESP_LOGW(TAG,
				 "[%s] init FAILED: %s",
				 sensor_registry_name(sensor_id),
				 esp_err_to_name(err));
	}
}

static bool i2c_sensor_is_initialized(const sensor_snapshot_t *snapshot)
{
	if (snapshot == NULL) {
		return false;
	}

	return snapshot->status[SENSOR_ID_MLX90614].initialized ||
		   snapshot->status[SENSOR_ID_MAX30102].initialized ||
		   snapshot->status[SENSOR_ID_ADXL345].initialized ||
		   snapshot->status[SENSOR_ID_OLED].initialized;
}

static void retry_i2c_sensor_inits(const sensor_snapshot_t *snapshot)
{
	if (snapshot == NULL) {
		return;
	}

	if (!snapshot->status[SENSOR_ID_MLX90614].initialized) {
		register_init_result(SENSOR_ID_MLX90614, sensor_mlx90614_init());
	}

	if (!snapshot->status[SENSOR_ID_MAX30102].initialized) {
		register_init_result(SENSOR_ID_MAX30102, sensor_max30102_init());
	}

	if (!snapshot->status[SENSOR_ID_ADXL345].initialized) {
		register_init_result(SENSOR_ID_ADXL345, sensor_adxl345_init());
	}

#if ENABLE_OLED_SUMMARY
	if (!snapshot->status[SENSOR_ID_OLED].initialized) {
		register_init_result(SENSOR_ID_OLED, display_ssd1306_init());
	}
#endif
}

static void task_dht22(void *arg)
{
	(void)arg;

	TickType_t last_wake = xTaskGetTickCount();
	uint32_t error_count = 0;
	uint32_t consecutive_errors = 0;

	while (true) {
		sensor_snapshot_t snapshot;
		sensor_registry_get_snapshot(&snapshot);

		if (snapshot.status[SENSOR_ID_DHT22].initialized) {
			float temp_c = 0.0f;
			float humidity_pct = 0.0f;
			esp_err_t err = sensor_dht22_read(&temp_c, &humidity_pct);
			if (err == ESP_OK) {
				sensor_registry_update_dht22(temp_c, humidity_pct, ESP_OK);
				error_count = 0;
				consecutive_errors = 0;
			} else if (err != ESP_ERR_INVALID_STATE) {
				sensor_registry_update_dht22(0.0f, 0.0f, err);
				++consecutive_errors;
				if ((error_count++ % 10U) == 0U) {
					ESP_LOGW(TAG, "[DHT22] read error: %s", esp_err_to_name(err));
				}

				if (consecutive_errors >= 6U) {
					esp_err_t reinit_err = sensor_dht22_init(DHT22_DATA_GPIO);
					register_init_result(SENSOR_ID_DHT22, reinit_err);
					consecutive_errors = 0;
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

	while (true) {
		sensor_snapshot_t snapshot;
		sensor_registry_get_snapshot(&snapshot);

		if (snapshot.status[SENSOR_ID_KY037].initialized) {
			int raw = 0;
			esp_err_t err = sensor_ky037_read_raw(&raw);
			sensor_registry_update_ky037(raw, err);
		}

		if (snapshot.status[SENSOR_ID_MQ2].initialized) {
			int raw = 0;
			esp_err_t err = sensor_mq2_read_raw(&raw);
			sensor_registry_update_mq2(raw, err);
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

	while (true) {
		sensor_snapshot_t snapshot;
		sensor_registry_get_snapshot(&snapshot);

		int64_t now_us = esp_timer_get_time();
		if ((now_us - last_retry_us) >= (I2C_RETRY_INIT_PERIOD_MS * 1000LL)) {
			retry_i2c_sensor_inits(&snapshot);
			last_retry_us = now_us;
			sensor_registry_get_snapshot(&snapshot);
		}

		if (!i2c_sensor_is_initialized(&snapshot) &&
			(now_us - last_scan_us) >= (I2C_RESCAN_PERIOD_MS * 1000LL)) {
			ESP_LOGW(TAG, "[I2C] no I2C sensors initialized yet, rescanning bus");
			(void)i2c_manager_scan();
			last_scan_us = now_us;
		}

		if (snapshot.status[SENSOR_ID_MLX90614].initialized) {
			float object_temp_c = 0.0f;
			float ambient_temp_c = 0.0f;
			esp_err_t err = sensor_mlx90614_read_temperatures(&object_temp_c, &ambient_temp_c);
			sensor_registry_update_mlx90614(object_temp_c, ambient_temp_c, err);
		}

		if (snapshot.status[SENSOR_ID_MAX30102].initialized) {
			uint32_t red = 0;
			uint32_t ir = 0;
			esp_err_t err = sensor_max30102_read_raw(&red, &ir);
			if (err == ESP_OK) {
				sensor_registry_update_max30102(red, ir, ESP_OK);
			} else if (err != ESP_ERR_NOT_FOUND) {
				sensor_registry_update_max30102(0, 0, err);
			}
		}

		if (snapshot.status[SENSOR_ID_ADXL345].initialized) {
			float x_g = 0.0f;
			float y_g = 0.0f;
			float z_g = 0.0f;
			esp_err_t err = sensor_adxl345_read_xyz(&x_g, &y_g, &z_g);
			sensor_registry_update_adxl345(x_g, y_g, z_g, err);
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(I2C_TASK_PERIOD_MS));
	}
}

static void task_logger(void *arg)
{
	(void)arg;

	TickType_t last_wake = xTaskGetTickCount();

	while (true) {
		sensor_snapshot_t snapshot;
		sensor_registry_get_snapshot(&snapshot);

		if (snapshot.status[SENSOR_ID_DHT22].initialized && snapshot.status[SENSOR_ID_DHT22].healthy) {
			ESP_LOGI(TAG, "[DHT22] temp=%.1fC hum=%.1f%%", snapshot.dht_temp_c, snapshot.dht_humidity_pct);
		} else if (snapshot.status[SENSOR_ID_DHT22].initialized) {
			ESP_LOGW(TAG,
					 "[DHT22] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_DHT22].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[DHT22] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_DHT22].last_error));
		}

		if (snapshot.status[SENSOR_ID_MLX90614].initialized && snapshot.status[SENSOR_ID_MLX90614].healthy) {
			ESP_LOGI(TAG,
					 "[MLX90614] object=%.1fC ambient=%.1fC",
					 snapshot.mlx_object_temp_c,
					 snapshot.mlx_ambient_temp_c);
		} else if (snapshot.status[SENSOR_ID_MLX90614].initialized) {
			ESP_LOGW(TAG,
					 "[MLX90614] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MLX90614].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[MLX90614] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MLX90614].last_error));
		}

		if (snapshot.status[SENSOR_ID_KY037].initialized && snapshot.status[SENSOR_ID_KY037].healthy) {
			ESP_LOGI(TAG, "[KY037] raw=%d", snapshot.ky037_raw);
		} else if (snapshot.status[SENSOR_ID_KY037].initialized) {
			ESP_LOGW(TAG,
					 "[KY037] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_KY037].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[KY037] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_KY037].last_error));
		}

		if (snapshot.status[SENSOR_ID_MQ2].initialized && snapshot.status[SENSOR_ID_MQ2].healthy) {
			ESP_LOGI(TAG, "[MQ2] raw=%d", snapshot.mq2_raw);
		} else if (snapshot.status[SENSOR_ID_MQ2].initialized) {
			ESP_LOGW(TAG,
					 "[MQ2] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MQ2].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[MQ2] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MQ2].last_error));
		}

		if (snapshot.status[SENSOR_ID_MAX30102].initialized && snapshot.status[SENSOR_ID_MAX30102].healthy) {
			ESP_LOGI(TAG,
					 "[MAX30102] red=%lu ir=%lu",
					 (unsigned long)snapshot.max30102_red,
					 (unsigned long)snapshot.max30102_ir);
		} else if (snapshot.status[SENSOR_ID_MAX30102].initialized) {
			ESP_LOGW(TAG,
					 "[MAX30102] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MAX30102].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[MAX30102] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_MAX30102].last_error));
		}

		if (snapshot.status[SENSOR_ID_ADXL345].initialized && snapshot.status[SENSOR_ID_ADXL345].healthy) {
			ESP_LOGI(TAG,
					 "[ADXL345] x=%.3fg y=%.3fg z=%.3fg",
					 snapshot.adxl_x_g,
					 snapshot.adxl_y_g,
					 snapshot.adxl_z_g);
		} else if (snapshot.status[SENSOR_ID_ADXL345].initialized) {
			ESP_LOGW(TAG,
					 "[ADXL345] status=ERR err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_ADXL345].last_error));
		} else {
			ESP_LOGW(TAG,
					 "[ADXL345] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_ADXL345].last_error));
		}

		if (snapshot.status[SENSOR_ID_OLED].initialized) {
			const char *oled_state = snapshot.status[SENSOR_ID_OLED].healthy ? "OK" : "ERR";
			ESP_LOGI(TAG, "[OLED] status=%s", oled_state);
		} else {
			ESP_LOGW(TAG,
					 "[OLED] status=NOT_INIT err=%s",
					 esp_err_to_name(snapshot.status[SENSOR_ID_OLED].last_error));
		}

		int64_t uptime_s = (esp_timer_get_time() - snapshot.boot_time_us) / 1000000;
		ESP_LOGI(TAG, "[HEARTBEAT] uptime=%llds", (long long)uptime_s);

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));
	}
}

static void task_oled(void *arg)
{
	(void)arg;

	TickType_t last_wake = xTaskGetTickCount();

	while (true) {
		sensor_snapshot_t snapshot;
		sensor_registry_get_snapshot(&snapshot);

		if (snapshot.status[SENSOR_ID_OLED].initialized) {
			esp_err_t err = display_ssd1306_render_summary(&snapshot);
			if (err == ESP_OK) {
				sensor_registry_mark_ok(SENSOR_ID_OLED);
			} else {
				sensor_registry_mark_error(SENSOR_ID_OLED, err);
			}
		}

		vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(OLED_TASK_PERIOD_MS));
	}
}

void app_main(void)
{
	ESP_LOGI(TAG, "Phase-1 baby monitor prototype booting...");

	initialize_nvs();
	sensor_registry_init();

	// Shared I2C bus (GPIO21 SDA, GPIO22 SCL) for MAX30102/MLX90614/SSD1306/(potentially ADXL345).
	esp_err_t i2c_err = i2c_manager_init();
	if (i2c_err == ESP_OK) {
		(void)i2c_manager_scan();
	} else {
		ESP_LOGW(TAG, "I2C init failed: %s", esp_err_to_name(i2c_err));
	}

	// ADC1 channels on GPIO34 (KY-037 AO) and GPIO35 (MQ-2 AO via divider).
	esp_err_t adc_err = adc_manager_init();
	if (adc_err != ESP_OK) {
		ESP_LOGW(TAG, "ADC init failed: %s", esp_err_to_name(adc_err));
	}

	register_init_result(SENSOR_ID_DHT22, sensor_dht22_init(DHT22_DATA_GPIO));

	if (adc_err == ESP_OK) {
		register_init_result(SENSOR_ID_KY037, sensor_ky037_init());
		register_init_result(SENSOR_ID_MQ2, sensor_mq2_init());
	} else {
		register_init_result(SENSOR_ID_KY037, adc_err);
		register_init_result(SENSOR_ID_MQ2, adc_err);
	}

	if (i2c_err == ESP_OK) {
		register_init_result(SENSOR_ID_MLX90614, sensor_mlx90614_init());
		register_init_result(SENSOR_ID_MAX30102, sensor_max30102_init());
		register_init_result(SENSOR_ID_ADXL345, sensor_adxl345_init());

#if ENABLE_OLED_SUMMARY
		register_init_result(SENSOR_ID_OLED, display_ssd1306_init());
#else
		register_init_result(SENSOR_ID_OLED, ESP_ERR_NOT_SUPPORTED);
#endif
	} else {
		register_init_result(SENSOR_ID_MLX90614, i2c_err);
		register_init_result(SENSOR_ID_MAX30102, i2c_err);
		register_init_result(SENSOR_ID_ADXL345, i2c_err);
		register_init_result(SENSOR_ID_OLED, i2c_err);
	}

	xTaskCreate(task_dht22, "task_dht22", 4096, NULL, 5, NULL);
	xTaskCreate(task_analog, "task_analog", 3072, NULL, 5, NULL);
	xTaskCreate(task_i2c_sensors, "task_i2c", 4096, NULL, 5, NULL);
	xTaskCreate(task_logger, "task_logger", 4096, NULL, 4, NULL);

#if ENABLE_OLED_SUMMARY
	xTaskCreate(task_oled, "task_oled", 4096, NULL, 3, NULL);
#endif

	ESP_LOGI(TAG, "Phase-1 tasks started.");
}
