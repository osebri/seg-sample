//Some of the code here could be dead or inappropriate
// Delete once sensor usage has been understood
#include "sensor_dht22.h"

#include <stdint.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensor_dht22";

#define DHT22_MIN_INTERVAL_US (2000000)
#define DHT22_START_SIGNAL_MS (2)
#define DHT11_START_SIGNAL_MS (20)
#define DHT22_STARTUP_SETTLE_US (2500000)
#define DHT22_RESPONSE_TIMEOUT_US (400)
#define DHT22_BIT_TIMEOUT_US (260)

static gpio_num_t s_data_pin = GPIO_NUM_NC;
static int64_t s_last_read_us;
static int64_t s_ready_after_us;
static portMUX_TYPE s_dht_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bool s_reported_dht11_mode;

static esp_err_t dht22_wait_idle_high(uint32_t timeout_us)
{
    int64_t start_us = esp_timer_get_time();
    while (gpio_get_level(s_data_pin) == 0) {
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

static esp_err_t dht22_wait_for_level(int expected_level, uint32_t timeout_us)
{
    int64_t start_us = esp_timer_get_time();
    while (gpio_get_level(s_data_pin) != expected_level) {
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

static esp_err_t dht22_measure_level_high_us(uint32_t timeout_us, uint32_t *duration_us)
{
    if (duration_us == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t start_us = esp_timer_get_time();
    while (gpio_get_level(s_data_pin) == 1) {
        if ((esp_timer_get_time() - start_us) > timeout_us) {
            return ESP_ERR_TIMEOUT;
        }
    }

    *duration_us = (uint32_t)(esp_timer_get_time() - start_us);
    return ESP_OK;
}

static esp_err_t dht22_read_once(float *temp_c, float *humidity_pct, uint32_t start_signal_ms)
{
    uint8_t data[5] = {0};

    gpio_set_direction(s_data_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_data_pin, GPIO_PULLUP_ONLY);
    esp_err_t idle_err = dht22_wait_idle_high(2000);
    if (idle_err != ESP_OK) {
        return idle_err;
    }

    gpio_set_direction(s_data_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(s_data_pin, 1);
    esp_rom_delay_us(2000);
    gpio_set_level(s_data_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(start_signal_ms));
    gpio_set_level(s_data_pin, 1);
    esp_rom_delay_us(40);

    esp_err_t err = ESP_OK;
    taskENTER_CRITICAL(&s_dht_spinlock);

    gpio_set_direction(s_data_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(s_data_pin, GPIO_PULLUP_ONLY);

    err = dht22_wait_for_level(0, DHT22_RESPONSE_TIMEOUT_US);
    if (err != ESP_OK) {
        goto exit_read;
    }
    err = dht22_wait_for_level(1, DHT22_RESPONSE_TIMEOUT_US);
    if (err != ESP_OK) {
        goto exit_read;
    }
    err = dht22_wait_for_level(0, DHT22_RESPONSE_TIMEOUT_US);
    if (err != ESP_OK) {
        goto exit_read;
    }

    for (int bit = 0; bit < 40; ++bit) {
        err = dht22_wait_for_level(1, DHT22_BIT_TIMEOUT_US);
        if (err != ESP_OK) {
            goto exit_read;
        }

        uint32_t high_time_us = 0;
        err = dht22_measure_level_high_us(DHT22_BIT_TIMEOUT_US, &high_time_us);
        if (err != ESP_OK) {
            goto exit_read;
        }

        data[bit / 8] <<= 1;
        if (high_time_us > 40) {
            data[bit / 8] |= 1;
        }
    }

exit_read:
    taskEXIT_CRITICAL(&s_dht_spinlock);

    if (err != ESP_OK) {
        return err;
    }

    uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_humidity = (uint16_t)((data[0] << 8) | data[1]);
    uint16_t raw_temp = (uint16_t)(((data[2] & 0x7F) << 8) | data[3]);

    float humidity = raw_humidity / 10.0f;
    float temp = raw_temp / 10.0f;
    if (data[2] & 0x80) {
        temp = -temp;
    }

    // Some modules sold as DHT22 are DHT11-compatible parts; decode fallback format when needed.
    if (humidity > 100.0f || humidity < 0.0f || temp > 85.0f || temp < -40.0f) {
        float dht11_humidity = (float)data[0] + ((float)data[1] * 0.1f);
        float dht11_temp = (float)data[2] + ((float)data[3] * 0.1f);

        if (dht11_humidity >= 0.0f && dht11_humidity <= 100.0f && dht11_temp >= -20.0f && dht11_temp <= 80.0f) {
            humidity = dht11_humidity;
            temp = dht11_temp;
            if (!s_reported_dht11_mode) {
                ESP_LOGW(TAG, "DHT payload matches DHT11-style format; using compatibility decode");
                s_reported_dht11_mode = true;
            }
        } else {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    *humidity_pct = humidity;
    *temp_c = temp;
    return ESP_OK;
}

esp_err_t sensor_dht22_init(gpio_num_t data_pin)
{
    if (data_pin < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    s_data_pin = data_pin;

    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << s_data_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_last_read_us = 0;
    s_ready_after_us = esp_timer_get_time() + DHT22_STARTUP_SETTLE_US;
    s_reported_dht11_mode = false;
    ESP_LOGI(TAG, "DHT22 initialized on GPIO%d", s_data_pin);
    return ESP_OK;
}

esp_err_t sensor_dht22_read(float *temp_c, float *humidity_pct)
{
    if (s_data_pin == GPIO_NUM_NC) {
        return ESP_ERR_INVALID_STATE;
    }
    if (temp_c == NULL || humidity_pct == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t now_us = esp_timer_get_time();
    if (now_us < s_ready_after_us) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((now_us - s_last_read_us) < DHT22_MIN_INTERVAL_US) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t last_err = ESP_FAIL;
    const uint32_t start_pulses_ms[4] = {
        DHT22_START_SIGNAL_MS,
        DHT22_START_SIGNAL_MS,
        DHT11_START_SIGNAL_MS,
        DHT11_START_SIGNAL_MS,
    };

    for (int attempt = 0; attempt < 4; ++attempt) {
        esp_err_t err = dht22_read_once(temp_c, humidity_pct, start_pulses_ms[attempt]);
        if (err == ESP_OK) {
            s_last_read_us = esp_timer_get_time();
            return ESP_OK;
        }
        last_err = err;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGW(TAG, "DHT22 read failed: %s", esp_err_to_name(last_err));
    return last_err;
}
