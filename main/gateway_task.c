#include "gateway_task.h"
#include "pin_config.h"
#include "credentials.h"
#include "i2c_bus.h"
#include "oled_ssd1306.h"
#include "lora_sx1262.h"
#include "lora_packet.h"
#include "wifi_manager.h"
#include "mqtt_publisher.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const char *TAG = "gateway";

/* Limiar de vibração para alerta de rajada (em g) */
#define GUST_VIBRATION_THRESHOLD  0.15f

static i2c_master_bus_handle_t  sensor_bus;
static i2c_master_bus_handle_t  oled_bus;
static oled_handle_t            oled;
static lora_sx1262_handle_t     lora;

static bool     oled_available = false;
static uint32_t pkt_count      = 0;

/* ---------- Inicialização ---------- */

esp_err_t gateway_init(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Gateway LoRa + MQTT — Init");
    ESP_LOGI(TAG, "  Heltec WiFi LoRa 32 V3 | ESP-IDF v6.0");
    ESP_LOGI(TAG, "========================================");

    /* 1. I2C (apenas OLED) */
    ret = i2c_bus_init(&sensor_bus, &oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init I2C");
        goto err_init;
    }

    /* 2. OLED */
    ret = oled_init(&oled, oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "OLED nao inicializado");
    } else {
        oled_available = true;
        oled_clear(&oled);
        oled_draw_text(&oled, 0, 0, "Gateway LoRa");
        oled_draw_text(&oled, 0, 1, "Conectando WiFi...");
        oled_update(&oled);
    }

    /* 3. WiFi */
    ret = wifi_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao conectar WiFi");
        goto err_init;
    }
    if (oled_available) {
        oled_draw_text(&oled, 0, 2, "WiFi OK!");
        oled_draw_text(&oled, 0, 3, "Conectando MQTT...");
        oled_update(&oled);
    }

    /* 4. MQTT */
    ret = mqtt_publisher_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao conectar MQTT");
        goto err_init;
    }
    if (oled_available) {
        oled_draw_text(&oled, 0, 4, "MQTT OK!");
        oled_draw_text(&oled, 0, 5, "Iniciando LoRa...");
        oled_update(&oled);
    }

    /* 5. LoRa SX1262 */
    ret = lora_sx1262_init(&lora);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na init LoRa SX1262");
        goto err_init;
    }

    ESP_LOGI(TAG, "Gateway completo: LoRa + WiFi + MQTT");
    return ESP_OK;

err_init:
    return ret;
}

/* ---------- Log serial ---------- */

static void log_packet(const lora_packet_t *pkt, int8_t rssi, int8_t snr)
{
    ESP_LOGI(TAG, "=== Pacote #%lu (RSSI=%d dBm, SNR=%d dB) ===",
             (unsigned long)pkt_count, rssi, snr);
    ESP_LOGI(TAG, "BME280  | Temp=%.2f C | Umid=%.1f %% | Press=%.2f hPa",
             pkt->temperature_c, pkt->humidity_pct, pkt->pressure_hpa);
    ESP_LOGI(TAG, "BH1750  | Lux=%.1f", pkt->lux);
    ESP_LOGI(TAG, "MQ-7    | Raw=%d | Voltage=%.2f V",
             pkt->mq7_raw, pkt->mq7_voltage);
    ESP_LOGI(TAG, "MQ-2    | Raw=%d | Voltage=%.2f V",
             pkt->mq2_raw, pkt->mq2_voltage);
    ESP_LOGI(TAG, "Chuva   | Raw=%d | Voltage=%.2f V | %s",
             pkt->rain_raw, pkt->rain_voltage,
             pkt->rain_status ? "CHOVENDO" : "SECO");
    ESP_LOGI(TAG, "MPU6050 | Accel=(%.2f, %.2f, %.2f) g",
             pkt->accel_x, pkt->accel_y, pkt->accel_z);
    ESP_LOGI(TAG, "MPU6050 | Gyro=(%.1f, %.1f, %.1f) deg/s",
             pkt->gyro_x, pkt->gyro_y, pkt->gyro_z);
    ESP_LOGI(TAG, "MPU6050 | Temp interna=%.1f C", pkt->imu_temp_c);
}

/* ---------- OLED ---------- */

static void update_oled(const lora_packet_t *pkt, int8_t rssi, int8_t snr,
                        bool mqtt_ok, float vibration_g, bool gust_alert)
{
    if (!oled_available) { return; }

    char line[22];
    oled_clear(&oled);

    snprintf(line, sizeof(line), "T:%.1fC  H:%.0f%%",
             pkt->temperature_c, pkt->humidity_pct);
    oled_draw_text(&oled, 0, 0, line);

    snprintf(line, sizeof(line), "P:%.1f hPa", pkt->pressure_hpa);
    oled_draw_text(&oled, 0, 1, line);

    snprintf(line, sizeof(line), "Lux:%.0f  %s",
             pkt->lux, pkt->rain_status ? "CHUVA" : "SECO");
    oled_draw_text(&oled, 0, 2, line);

    snprintf(line, sizeof(line), "CO:%d GLP:%d",
             pkt->mq7_raw, pkt->mq2_raw);
    oled_draw_text(&oled, 0, 3, line);

    snprintf(line, sizeof(line), "Vib:%.2fg %s",
             vibration_g, gust_alert ? "RAJADA!" : "OK");
    oled_draw_text(&oled, 0, 4, line);

    snprintf(line, sizeof(line), "LoRa:%ddBm SNR:%d", rssi, snr);
    oled_draw_text(&oled, 0, 5, line);

    snprintf(line, sizeof(line), "MQTT:%s WiFi:%s",
             mqtt_ok ? "OK" : "FAIL",
             wifi_manager_is_connected() ? "OK" : "OFF");
    oled_draw_text(&oled, 0, 6, line);

    snprintf(line, sizeof(line), "Pkts:%lu | GW v1.1",
             (unsigned long)pkt_count);
    oled_draw_text(&oled, 0, 7, line);

    oled_update(&oled);
}

/* ---------- Task principal ---------- */

void gateway_task(void *arg)
{
    (void)arg;

    uint8_t   rx_buf[256];
    uint8_t   rx_len = 0;
    int8_t    rssi   = 0;
    int8_t    snr    = 0;
    bool      mqtt_ok = false;
    esp_err_t ret;

    ESP_LOGI(TAG, "Entrando em modo RX continuo...");

    if (oled_available) {
        oled_clear(&oled);
        oled_draw_text(&oled, 0, 0, "Gateway LoRa+MQTT");
        oled_draw_text(&oled, 0, 2, "WiFi: OK");
        oled_draw_text(&oled, 0, 3, "MQTT: OK");
        oled_draw_text(&oled, 0, 5, "Aguardando dados");
        oled_draw_text(&oled, 0, 6, "do no sensor...");
        oled_update(&oled);
    }

    while (1) {
        /* Aguarda pacote LoRa (RX contínuo) */
        rx_len = 0;
        ret = lora_sx1262_receive(&lora, rx_buf, &rx_len, &rssi, &snr, 0);

        if (ret == ESP_ERR_TIMEOUT) {
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Erro na recepcao: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Verifica tamanho */
        if (rx_len != sizeof(lora_packet_t)) {
            ESP_LOGW(TAG, "Pacote tamanho invalido: %d (esperado %zu)",
                     rx_len, sizeof(lora_packet_t));
            continue;
        }

        /* Desserializa e valida */
        lora_packet_t pkt;
        memcpy(&pkt, rx_buf, sizeof(pkt));

        if (!lora_packet_validate(&pkt)) {
            ESP_LOGW(TAG, "Pacote CRC invalido — descartado");
            continue;
        }

        pkt_count++;

        /* Calcula vibração: magnitude da aceleração - 1g (gravidade) */
        float accel_mag = sqrtf(pkt.accel_x * pkt.accel_x +
                                pkt.accel_y * pkt.accel_y +
                                pkt.accel_z * pkt.accel_z);
        float vibration_g = fabsf(accel_mag - 1.0f);
        bool gust_alert = (vibration_g > GUST_VIBRATION_THRESHOLD);

        /* 1. Log serial */
        log_packet(&pkt, rssi, snr);
        ESP_LOGI(TAG, "Vibracao | %.3f g | %s",
                 vibration_g, gust_alert ? "ALERTA RAJADA!" : "Normal");

        /* 2. Publica via MQTT */
        mqtt_ok = (mqtt_publish_data(&pkt, rssi, snr, vibration_g, gust_alert) == ESP_OK);
        if (mqtt_ok) {
            ESP_LOGI(TAG, "MQTT publicado no topico '%s'", MQTT_TOPIC_DATA);
        } else {
            ESP_LOGW(TAG, "MQTT publish falhou");
        }

        /* 3. Atualiza OLED */
        update_oled(&pkt, rssi, snr, mqtt_ok, vibration_g, gust_alert);
    }
}