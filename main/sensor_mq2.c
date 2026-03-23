#include "sensor_mq2.h"

#include "adc_manager.h"

esp_err_t sensor_mq2_init(void)
{
    // the actual sensor is mq135 and not mq2
    // please verify the voltage before using (3.3v or 5v)
    return ESP_OK;
}

esp_err_t sensor_mq2_read_raw(int *raw_value)
{
    return adc_manager_read_mq2(raw_value);
}
