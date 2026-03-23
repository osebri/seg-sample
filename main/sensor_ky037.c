#include "sensor_ky037.h"

#include "adc_manager.h"
#include "esp_rom_sys.h"

#define KY037_WINDOW_SAMPLES 256
#define KY037_SAMPLE_SPACING_US 200

static int s_last_mean = -1;

esp_err_t sensor_ky037_init(void)
{
    s_last_mean = -1;
    return ESP_OK;
}

esp_err_t sensor_ky037_read_raw(int *raw_value)
{
    if (raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int min_value = 4095;
    int max_value = 0;
    int64_t sum_value = 0;

    for (int i = 0; i < KY037_WINDOW_SAMPLES; ++i) {
        int sample = 0;
        esp_err_t err = adc_manager_read_ky037(&sample);
        if (err != ESP_OK) {
            return err;
        }

        if (sample < min_value) {
            min_value = sample;
        }
        if (sample > max_value) {
            max_value = sample;
        }
        sum_value += sample;

        esp_rom_delay_us(KY037_SAMPLE_SPACING_US);
    }

    const int mean_value = (int)(sum_value / KY037_WINDOW_SAMPLES);
    const int amplitude = max_value - min_value;

    if (max_value == 0 && min_value == 0) {
        *raw_value = 0;
        s_last_mean = 0;
        return ESP_OK;
    }

    // KY-037 analog output is often (or could be) inverted (louder sound -> lower voltage).
    // We report a robust activity metric combining AC amplitude with slow DC movement for the moment
    // however this logic is subject to change once we fully understand how the sensor operates
    int baseline_delta = 0;
    if (s_last_mean >= 0) {
        baseline_delta = mean_value - s_last_mean;
        if (baseline_delta < 0) {
            baseline_delta = -baseline_delta;
        }
    }
    s_last_mean = mean_value;

    int activity = amplitude + baseline_delta;
    if (activity <= 0) {
        // Fallback to an inverted DC level so a connected quiet sensor is still observable.
        activity = 4095 - mean_value;
    }

    *raw_value = activity;
    return ESP_OK;
}
