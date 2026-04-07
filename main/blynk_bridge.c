#include "blynk_bridge.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "blynk_config.h"

static const char *TAG = "blynk_bridge";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
#define BLYNK_HTTP_TIMEOUT_MS 5000
#define BLYNK_WIFI_MAX_RETRIES 5
#define ALERT_COOLDOWN_MS 120000

static TickType_t last_body_temp_alert = 0;
static TickType_t last_heart_rate_alert = 0;
static TickType_t last_room_temp_alert = 0;
static TickType_t last_humidity_alert = 0;
static TickType_t last_cry_alert = 0;
static TickType_t last_gas_alert = 0;
static bool s_initialized;
static bool s_logged_disabled;
static bool s_wifi_ready;
static int s_wifi_retry_count;
static EventGroupHandle_t s_wifi_event_group;
static esp_event_handler_instance_t s_wifi_event_handler;
static esp_event_handler_instance_t s_ip_event_handler;

static void blynk_wifi_event_handler(void *arg,
                                     esp_event_base_t event_base,
                                     int32_t event_id,
                                     void *event_data)
{
    (void)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        ESP_LOGI(TAG, "Wi-Fi started, connecting to '%s'", BLYNK_WIFI_SSID);
        (void)esp_wifi_connect();
        return;
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        s_wifi_ready = false;
        if (s_wifi_retry_count < BLYNK_WIFI_MAX_RETRIES)
        {
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "Wi-Fi disconnected, retrying (%d/%d)", s_wifi_retry_count, BLYNK_WIFI_MAX_RETRIES);
            (void)esp_wifi_connect();
        }
        else if (s_wifi_event_group != NULL)
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        return;
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)event_data;
        s_wifi_retry_count = 0;
        s_wifi_ready = true;
        if (s_wifi_event_group != NULL)
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        ESP_LOGI(TAG, "Wi-Fi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static esp_err_t blynk_wifi_init_sta(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    static bool s_netif_created;
    if (!s_netif_created)
    {
        esp_netif_t *netif = esp_netif_create_default_wifi_sta();
        if (netif == NULL)
        {
            return ESP_FAIL;
        }
        s_netif_created = true;
    }

    wifi_init_config_t wifi_init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&wifi_init_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    if (s_wifi_event_group == NULL)
    {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    if (s_wifi_event_handler == NULL)
    {
        err = esp_event_handler_instance_register(WIFI_EVENT,
                                                  ESP_EVENT_ANY_ID,
                                                  &blynk_wifi_event_handler,
                                                  NULL,
                                                  &s_wifi_event_handler);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    if (s_ip_event_handler == NULL)
    {
        err = esp_event_handler_instance_register(IP_EVENT,
                                                  IP_EVENT_STA_GOT_IP,
                                                  &blynk_wifi_event_handler,
                                                  NULL,
                                                  &s_ip_event_handler);
        if (err != ESP_OK)
        {
            return err;
        }
    }

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    strlcpy((char *)wifi_config.sta.ssid, BLYNK_WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, BLYNK_WIFI_PASSWORD, sizeof(wifi_config.sta.password));

    err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (err != ESP_OK)
    {
        return err;
    }

    err = esp_wifi_start();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        return err;
    }

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(BLYNK_WIFI_CONNECT_TIMEOUT_MS));
    if (bits & WIFI_CONNECTED_BIT)
    {
        return ESP_OK;
    }
    if (bits & WIFI_FAIL_BIT)
    {
        return ESP_ERR_WIFI_NOT_CONNECT;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t blynk_http_publish(const char *url)
{
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = BLYNK_HTTP_TIMEOUT_MS,
        .transport_type = HTTP_TRANSPORT_OVER_SSL,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL)
    {
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code != 200)
        {
            ESP_LOGW(TAG, "Blynk HTTP update failed with status %d", status_code);
            err = ESP_FAIL;
        }
    }

    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t blynk_log_event(const char *event_code)
{
    if (event_code == NULL || !s_initialized || !s_wifi_ready)
    {
        return ESP_ERR_INVALID_STATE;
    }

    char url[256];
    int written = snprintf(url,
                           sizeof(url),
                           "https://%s/external/api/logEvent?token=%s&code=%s",
                           BLYNK_SERVER_HOST,
                           BLYNK_AUTH_TOKEN,
                           event_code);

    if (written <= 0 || written >= (int)sizeof(url))
    {
        ESP_LOGW(TAG, "Blynk event URL too long");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Logging Blynk event: %s", event_code);
    ESP_LOGI(TAG, "Event URL: %s", url);

    return blynk_http_publish(url);
}

static bool sensor_has_publishable_value(const sensor_snapshot_t *snapshot, sensor_id_t sensor_id)
{
    if (snapshot == NULL)
    {
        return false;
    }

    sensor_state_t state = snapshot->status[sensor_id].state;
    return state == SENSOR_STATE_HEALTHY || state == SENSOR_STATE_WARMING_UP;
}

esp_err_t blynk_bridge_init(void)
{
#if BLYNK_ENABLE
    if (strlen(BLYNK_AUTH_TOKEN) == 0 || strlen(BLYNK_WIFI_SSID) == 0)
    {
        ESP_LOGW(TAG, "Blynk enabled, but auth token or Wi-Fi SSID is missing");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = blynk_wifi_init_sta();
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Wi-Fi init/connect failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Blynk bridge ready. Template=%s Name=%s Host=%s",
             BLYNK_TEMPLATE_ID,
             BLYNK_TEMPLATE_NAME,
             BLYNK_SERVER_HOST);
    return ESP_OK;
#else
    s_initialized = false;
    if (!s_logged_disabled)
    {
        ESP_LOGI(TAG, "Blynk bridge disabled. Enable it in project_config.h when ready.");
        s_logged_disabled = true;
    }
    return ESP_OK;
#endif
}

void blynk_bridge_task(void *arg)
{
    (void)arg;

    while (true)
    {
#if BLYNK_ENABLE
        if (!s_wifi_ready)
        {
            (void)esp_wifi_connect();
        }
        vTaskDelay(pdMS_TO_TICKS(BLYNK_LOOP_PERIOD_MS));
#else
        vTaskDelay(pdMS_TO_TICKS(1000));
#endif
    }
}
void blynk_bridge_publish_alerts(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

#if BLYNK_ENABLE
    if (!s_initialized || !s_wifi_ready)
    {
        return;
    }

    TickType_t now = xTaskGetTickCount();

    //  Body temperature
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_MLX90614))
    {
        float t = snapshot->mlx_object_temp_c;
        if ((t < 36.0f || t > 37.5f) &&
            (now - last_body_temp_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("body_temp_low_high");
            last_body_temp_alert = now;
        }
    }

    // Heart rate
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_MAX30102) &&
        snapshot->max30102_heart_rate_valid)
    {
        int hr = snapshot->max30102_heart_rate_bpm;
        if ((hr < 120 || hr > 180) &&
            (now - last_heart_rate_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("heart_rate_alert");
            last_heart_rate_alert = now;
        }
    }

    //  Room temperature
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_DHT22))
    {
        float rt = snapshot->dht_temp_c;
        if ((rt < 20.0f || rt > 22.2f) &&
            (now - last_room_temp_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("room_temp_low_high");
            last_room_temp_alert = now;
        }
    }

    //  Humidity
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_DHT22))
    {
        float h = snapshot->dht_humidity_pct;
        if ((h > 55.0f) &&
            (now - last_humidity_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("humidity_high");
            last_humidity_alert = now;
        }
    }

    //  Cry detection
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_KY037))
    {
        int cry = snapshot->ky037_activity_pct;
        if ((cry > 50) &&
            (now - last_cry_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("cry_alert");
            last_cry_alert = now;
        }
    }

    // Gas / air quality
    if (sensor_has_publishable_value(snapshot, SENSOR_ID_MQ135))
    {
        int gas = snapshot->mq135_response_pct;
        if ((gas > 50) &&
            (now - last_gas_alert > pdMS_TO_TICKS(ALERT_COOLDOWN_MS)))
        {
            blynk_log_event("gas_alert");
            last_gas_alert = now;
        }
    }

#endif
}

void blynk_bridge_publish_snapshot(const sensor_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

#if BLYNK_ENABLE
    if (!s_initialized || !s_wifi_ready)
    {
        return;
    }

    char url[512];
    int written = snprintf(url,
                           sizeof(url),
                           "https://%s/external/api/batch/update?token=%s"
                           "&v%d=%.2f"
                           "&v%d=%.2f"
                           "&v%d=%u"
                           "&v%d=%d"
                           "&v%d=%d"
                           "&v%d=%.2f"
                           "&v%d=%u"
                           "&v%d=%.3f"
                           "&v%d=%.3f"
                           "&v%d=%.3f",
                           BLYNK_SERVER_HOST,
                           BLYNK_AUTH_TOKEN,
                           BLYNK_VPIN_ROOM_TEMP,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_DHT22) ? snapshot->dht_temp_c : 0.0f,
                           BLYNK_VPIN_BODY_TEMP,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_MLX90614) ? snapshot->mlx_object_temp_c : 0.0f,
                           BLYNK_VPIN_HEART_RATE,
                           (unsigned int)((sensor_has_publishable_value(snapshot, SENSOR_ID_MAX30102) &&
                                           snapshot->max30102_heart_rate_valid)
                                              ? snapshot->max30102_heart_rate_bpm
                                              : 0U),
                           BLYNK_VPIN_SMOKE,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_MQ135) ? snapshot->mq135_response_pct : 0,
                           BLYNK_VPIN_CRY,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_KY037) ? snapshot->ky037_activity_pct : 0,
                           BLYNK_VPIN_HUMIDITY,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_DHT22) ? snapshot->dht_humidity_pct : 0.0f,
                           BLYNK_VPIN_SPO2,
                           (unsigned int)((sensor_has_publishable_value(snapshot, SENSOR_ID_MAX30102) &&
                                           snapshot->max30102_spo2_valid)
                                              ? snapshot->max30102_spo2_pct
                                              : 0U),
                           BLYNK_VPIN_ACCEL_X,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_ADXL345) ? (double)snapshot->adxl_x_raw : 0.0f,
                           BLYNK_VPIN_ACCEL_Y,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_ADXL345) ? (double)snapshot->adxl_y_raw : 0.0f,
                           BLYNK_VPIN_ACCEL_Z,
                           sensor_has_publishable_value(snapshot, SENSOR_ID_ADXL345) ? (double)snapshot->adxl_z_raw : 0.0f);

    blynk_bridge_publish_alerts(snapshot);
    if (written <= 0 || written >= (int)sizeof(url))
    {
        ESP_LOGW(TAG, "Blynk request buffer too small");
        return;
    }

    esp_err_t err = blynk_http_publish(url);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Blynk publish failed: %s", esp_err_to_name(err));
    }
#else
    (void)snapshot;
#endif
}
