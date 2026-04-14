#include "mqtt_publisher.h"
#include "credentials.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "mqtt";

#define MQTT_CONNECTED_BIT  BIT0
#define CONNECT_TIMEOUT_MS  15000

static esp_mqtt_client_handle_t s_client = NULL;
static EventGroupHandle_t s_mqtt_event_group;
static bool s_connected = false;

static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT conectado ao broker");
        s_connected = true;
        xEventGroupSetBits(s_mqtt_event_group, MQTT_CONNECTED_BIT);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT desconectado — reconectando...");
        s_connected = false;
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT erro (type=%d)", event->error_handle->error_type);
        break;

    default:
        break;
    }
}

esp_err_t mqtt_publisher_init(void)
{
    s_mqtt_event_group = xEventGroupCreate();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "Falha ao criar cliente MQTT");
        return ESP_FAIL;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID,
                                   mqtt_event_handler, NULL);

    esp_err_t ret = esp_mqtt_client_start(s_client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao iniciar cliente MQTT: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Conectando ao broker MQTT...");

    EventBits_t bits = xEventGroupWaitBits(
        s_mqtt_event_group,
        MQTT_CONNECTED_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(CONNECT_TIMEOUT_MS)
    );

    if (bits & MQTT_CONNECTED_BIT) {
        return ESP_OK;
    }

    ESP_LOGE(TAG, "Timeout ao conectar MQTT");
    return ESP_ERR_TIMEOUT;
}

esp_err_t mqtt_publish_data(const lora_packet_t *pkt, int8_t rssi, int8_t snr,
                            float vibration_g, bool gust_alert)
{
    if (!s_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    char json[600];
    int len = snprintf(json, sizeof(json),
        "{"
        "\"temperatura\":%.2f,"
        "\"umidade\":%.1f,"
        "\"pressao\":%.2f,"
        "\"lux\":%.1f,"
        "\"mq7_raw\":%d,"
        "\"mq7_voltage\":%.2f,"
        "\"mq2_raw\":%d,"
        "\"mq2_voltage\":%.2f,"
        "\"chuva_raw\":%d,"
        "\"chuva_voltage\":%.2f,"
        "\"chuva_status\":%d,"
        "\"accel_x\":%.2f,"
        "\"accel_y\":%.2f,"
        "\"accel_z\":%.2f,"
        "\"gyro_x\":%.1f,"
        "\"gyro_y\":%.1f,"
        "\"gyro_z\":%.1f,"
        "\"imu_temp\":%.1f,"
        "\"vibration_g\":%.3f,"
        "\"gust_alert\":%d,"
        "\"rssi\":%d,"
        "\"snr\":%d"
        "}",
        pkt->temperature_c, pkt->humidity_pct, pkt->pressure_hpa,
        pkt->lux,
        pkt->mq7_raw, pkt->mq7_voltage,
        pkt->mq2_raw, pkt->mq2_voltage,
        pkt->rain_raw, pkt->rain_voltage, pkt->rain_status,
        pkt->accel_x, pkt->accel_y, pkt->accel_z,
        pkt->gyro_x, pkt->gyro_y, pkt->gyro_z,
        pkt->imu_temp_c,
        vibration_g, gust_alert ? 1 : 0,
        rssi, snr
    );

    int msg_id = esp_mqtt_client_publish(s_client, MQTT_TOPIC_DATA,
                                         json, len, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Falha ao publicar MQTT");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "MQTT pub OK (msg_id=%d, %d bytes)", msg_id, len);
    return ESP_OK;
}

bool mqtt_publisher_is_connected(void)
{
    return s_connected;
}