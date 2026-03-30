#include "sensor_ky037.h"

#include <stdbool.h>
#include <stdint.h>

#include "adc_manager.h"
#include "esp_rom_sys.h"

#define KY037_WINDOW_SAMPLES 256
#define KY037_SAMPLE_SPACING_US 125

static bool s_noise_floor_ready;
static float s_noise_floor;

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

esp_err_t sensor_ky037_init(void)
{
    s_noise_floor_ready = false;
    s_noise_floor = 0.0f;
    return ESP_OK;
}

esp_err_t sensor_ky037_read_activity(sensor_ky037_reading_t *out_reading)
{
    if (out_reading == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int samples[KY037_WINDOW_SAMPLES] = {0};
    int min_value = 4095;
    int max_value = 0;
    int64_t sum_value = 0;

    for (int i = 0; i < KY037_WINDOW_SAMPLES; ++i) {
        int sample = 0;
        esp_err_t err = adc_manager_read_ky037(&sample);
        if (err != ESP_OK) {
            return err;
        }

        samples[i] = sample;
        if (sample < min_value) {
            min_value = sample;
        }
        if (sample > max_value) {
            max_value = sample;
        }
        sum_value += sample;

        esp_rom_delay_us(KY037_SAMPLE_SPACING_US);
    }

    int mean_value = (int)(sum_value / KY037_WINDOW_SAMPLES);
    int64_t abs_sum = 0;
    for (int i = 0; i < KY037_WINDOW_SAMPLES; ++i) {
        int deviation = samples[i] - mean_value;
        if (deviation < 0) {
            deviation = -deviation;
        }
        abs_sum += deviation;
    }

    int peak_to_peak = max_value - min_value;
    int mean_abs_deviation = (int)(abs_sum / KY037_WINDOW_SAMPLES);
    int composite_energy = peak_to_peak + (mean_abs_deviation * 2);

    if (!s_noise_floor_ready) {
        s_noise_floor_ready = true;
        s_noise_floor = (float)composite_energy;
    } else if ((float)composite_energy <= s_noise_floor) {
        s_noise_floor = (s_noise_floor * 0.92f) + ((float)composite_energy * 0.08f);
    } else {
        s_noise_floor = (s_noise_floor * 0.995f) + ((float)composite_energy * 0.005f);
    }

    int response = composite_energy - (int)(s_noise_floor + 0.5f);
    if (response < 0) {
        response = 0;
    }

    int normalizer = (int)(s_noise_floor * 0.65f);
    if (normalizer < 24) {
        normalizer = 24;
    }

    out_reading->activity_pct = clamp_pct((response * 100) / normalizer);
    out_reading->peak_to_peak = peak_to_peak;
    out_reading->mean_abs_deviation = mean_abs_deviation;

    return ESP_OK;
}
