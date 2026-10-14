#include "mqtt_publisher.h"
#include "credentials.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "mqtt";

#ifndef MQTT_TOPIC_HIST
#define MQTT_TOPIC_HIST  "estacao/sjc/hist"   /* credentials.h antigo sem o define */
#endif

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

/* ---------- Montagem do JSON ---------- */

typedef struct {
    char   *buf;
    size_t  size;
    size_t  len;
    bool    overflow;
} json_buf_t;

/**
 * @brief Acrescenta "key":valor, (ou "key":null, se !ok) ao buffer.
 */
static void json_num(json_buf_t *j, const char *key, bool ok, double v, int dec)
{
    if (j->overflow) { return; }

    ok = ok && isfinite(v);   /* NaN/Inf viraria JSON inválido — publica null */

    size_t room = j->size - j->len;
    int n = ok ? snprintf(j->buf + j->len, room, "\"%s\":%.*f,", key, dec, v)
               : snprintf(j->buf + j->len, room, "\"%s\":null,", key);

    if (n < 0 || (size_t)n >= room) {
        j->overflow = true;
        return;
    }
    j->len += (size_t)n;
}

esp_err_t mqtt_publish_data(const lora_packet_t *pkt, int8_t rssi, int8_t snr)
{
    if (!s_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    char json[512];
    json_buf_t j = { .buf = json, .size = sizeof(json), .len = 1 };
    json[0] = '{';

    const bool bme  = lora_packet_has(pkt, LORA_VALID_BME280);
    const bool bh   = lora_packet_has(pkt, LORA_VALID_BH1750);
    const bool mq7  = lora_packet_has(pkt, LORA_VALID_MQ7);
    const bool mq2  = lora_packet_has(pkt, LORA_VALID_MQ2);
    const bool rain = lora_packet_has(pkt, LORA_VALID_RAIN);
    const bool wind  = lora_packet_has(pkt, LORA_VALID_WIND);
    const bool gauge = lora_packet_has(pkt, LORA_VALID_GAUGE);

    json_num(&j, "temperatura",   bme,  pkt->temperature_c, 2);
    json_num(&j, "umidade",       bme,  pkt->humidity_pct,  1);
    json_num(&j, "pressao",       bme,  pkt->pressure_hpa,  2);
    json_num(&j, "lux",           bh,   pkt->lux,           1);
    json_num(&j, "mq7_raw",       mq7,  pkt->mq7_raw,       0);
    json_num(&j, "mq7_voltage",   mq7,  pkt->mq7_voltage,   2);
    json_num(&j, "mq2_raw",       mq2,  pkt->mq2_raw,       0);
    json_num(&j, "mq2_voltage",   mq2,  pkt->mq2_voltage,   2);
    json_num(&j, "chuva_raw",     rain, pkt->rain_raw,      0);
    json_num(&j, "chuva_voltage", rain, pkt->rain_voltage,  2);
    json_num(&j, "chuva_status",  rain, pkt->rain_status,   0);
    json_num(&j, "vento_ms",        wind,  pkt->wind_avg_ms,   1);
    json_num(&j, "vento_rajada_ms", wind,  pkt->wind_gust_ms,  1);
    json_num(&j, "chuva_acum_mm",   gauge, pkt->rain_total_mm, 2);
    json_num(&j, "rssi",          true, rssi,               0);
    json_num(&j, "snr",           true, snr,                0);

    if (j.overflow) {
        ESP_LOGE(TAG, "JSON excedeu o buffer (%zu bytes)", sizeof(json));
        return ESP_ERR_NO_MEM;
    }
    json[j.len - 1] = '}';   /* Troca a vírgula final pelo fechamento */

    int msg_id = esp_mqtt_client_publish(s_client, MQTT_TOPIC_DATA,
                                         json, (int)j.len, 1, 0);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Falha ao publicar MQTT");
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "MQTT pub OK (msg_id=%d, %zu bytes)", msg_id, j.len);
    return ESP_OK;
}

esp_err_t mqtt_publish_history(const uint8_t *buf, size_t len)
{
    if (!s_connected || s_client == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buf == NULL || len == 0 || len > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    /* QoS 0 + retain: com QoS 1 o esp-mqtt copiaria o restante do payload
       fragmentado (~48 KB) para o outbox. O snapshot é republicado a cada
       5 min, então perder um envio não tem custo. */
    int msg_id = esp_mqtt_client_publish(s_client, MQTT_TOPIC_HIST,
                                         (const char *)buf, (int)len, 0, 1);
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Falha ao publicar historico (%zu bytes)", len);
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool mqtt_publisher_is_connected(void)
{
    return s_connected;
}
