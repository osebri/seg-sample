#include "adc_manager.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "project_config.h"

static const char *TAG = "adc_manager";

static bool s_adc_initialized;
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_channel_t s_ky037_channel;
static adc_channel_t s_mq2_channel;

static esp_err_t adc_manager_config_channel(adc_channel_t channel)
{
    adc_oneshot_chan_cfg_t channel_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    return adc_oneshot_config_channel(s_adc_handle, channel, &channel_cfg);
}

esp_err_t adc_manager_init(void)
{
    if (s_adc_initialized) {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t init_cfg = {0};
    init_cfg.unit_id = ADC_UNIT_1;

    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        return err;
    }

    adc_unit_t ky037_unit = ADC_UNIT_1;
    err = adc_oneshot_io_to_channel(KY037_ADC_GPIO, &ky037_unit, &s_ky037_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map KY037 GPIO%d to ADC channel: %s", KY037_ADC_GPIO, esp_err_to_name(err));
        return err;
    }
    if (ky037_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "KY037 GPIO%d mapped to non-ADC1 unit, unsupported in this project", KY037_ADC_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    adc_unit_t mq2_unit = ADC_UNIT_1;
    err = adc_oneshot_io_to_channel(MQ135_ADC_GPIO, &mq2_unit, &s_mq2_channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to map MQ135 GPIO%d to ADC channel: %s", MQ135_ADC_GPIO, esp_err_to_name(err));
        return err;
    }
    if (mq2_unit != ADC_UNIT_1) {
        ESP_LOGE(TAG, "MQ135 GPIO%d mapped to non-ADC1 unit, unsupported in this project", MQ135_ADC_GPIO);
        return ESP_ERR_INVALID_ARG;
    }

    err = adc_manager_config_channel(s_ky037_channel);
    if (err != ESP_OK) {
        return err;
    }

    err = adc_manager_config_channel(s_mq2_channel);
    if (err != ESP_OK) {
        return err;
    }

    s_adc_initialized = true;

    ESP_LOGI(TAG,
             "ADC initialized (oneshot, 12-bit, 12dB) KY037 GPIO%d->CH%d, MQ135 GPIO%d->CH%d",
             KY037_ADC_GPIO,
             (int)s_ky037_channel,
             MQ135_ADC_GPIO,
             (int)s_mq2_channel);
    return ESP_OK;
}

static esp_err_t adc_manager_read_channel(adc_channel_t channel, int *raw_value)
{
    if (!s_adc_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (raw_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return adc_oneshot_read(s_adc_handle, channel, raw_value);
}

esp_err_t adc_manager_read_ky037(int *raw_value)
{
    return adc_manager_read_channel(s_ky037_channel, raw_value);
}

esp_err_t adc_manager_read_mq2(int *raw_value)
{
    return adc_manager_read_channel(s_mq2_channel, raw_value);
}
