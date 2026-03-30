#include "sensor_mq2.h"

#include <stdbool.h>
#include <stdint.h>

#include "adc_manager.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "project_config.h"

#define MQ135_SAMPLE_COUNT 32
#define MQ135_SAMPLE_SPACING_US 1000
#define MQ135_WARMUP_US (90000000LL)

static bool s_filters_ready;
static float s_filtered_raw;
static float s_baseline_raw;
static int64_t s_warmup_until_us;

static int clamp_pct(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return value;
}

esp_err_t sensor_mq2_init(void)
{
    s_filters_ready = false;
    s_filtered_raw = 0.0f;
    s_baseline_raw = 0.0f;
    s_warmup_until_us = esp_timer_get_time() + MQ135_WARMUP_US;
    return ESP_OK;
}

esp_err_t sensor_mq2_read_metrics(sensor_mq135_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int64_t sum_value = 0;
    for (int i = 0; i < MQ135_SAMPLE_COUNT; ++i) {
        int sample = 0;
        esp_err_t err = adc_manager_read_mq2(&sample);
        if (err != ESP_OK) {
            return err;
        }

        sum_value += sample;
        esp_rom_delay_us(MQ135_SAMPLE_SPACING_US);
    }

    float mean_raw = (float)sum_value / (float)MQ135_SAMPLE_COUNT;
    if (!s_filters_ready) {
        s_filters_ready = true;
        s_filtered_raw = mean_raw;
        s_baseline_raw = mean_raw;
    } else {
        s_filtered_raw = (s_filtered_raw * 0.65f) + (mean_raw * 0.35f);

        float delta = s_filtered_raw - s_baseline_raw;
        bool warming_up = esp_timer_get_time() < s_warmup_until_us;
        float freeze_threshold = (s_baseline_raw * 0.015f) + 24.0f;
        float baseline_alpha = warming_up ? 0.05f : 0.01f;

        if (delta < 0.0f) {
            baseline_alpha = warming_up ? 0.08f : 0.02f;
        }

        if (warming_up || delta <= freeze_threshold) {
            s_baseline_raw = (s_baseline_raw * (1.0f - baseline_alpha)) + (s_filtered_raw * baseline_alpha);
        }
    }

    int filtered_raw = (int)(s_filtered_raw + 0.5f);
    int baseline_raw = (int)(s_baseline_raw + 0.5f);
    int delta_raw = filtered_raw - baseline_raw;
    int response_pct = 0;
    int normalizer = (int)(s_baseline_raw * 0.08f);
    if (normalizer < 48) {
        normalizer = 48;
    }
    if (delta_raw > 0) {
        response_pct = clamp_pct((delta_raw * 100) / normalizer);
    }

    out_reading->filtered_raw = filtered_raw;
    out_reading->baseline_raw = baseline_raw;
    out_reading->delta_raw = delta_raw;
    out_reading->response_pct = response_pct;
    out_reading->warming_up = esp_timer_get_time() < s_warmup_until_us;

    return ESP_OK;
}
