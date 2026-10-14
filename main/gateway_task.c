#include "gateway_task.h"
#include "pin_config.h"
#include "credentials.h"
#include "i2c_bus.h"
#include "oled_ssd1306.h"
#include "lora_sx1262.h"
#include "lora_packet.h"
#include "wifi_manager.h"
#include "mqtt_publisher.h"
#include "time_sync.h"
#include "history.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "gateway";

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
    ESP_LOGI(TAG, "  Heltec WiFi LoRa 32 V3 | ESP-IDF v6.1");
    ESP_LOGI(TAG, "  Pacote LoRa v%d (%zu bytes)", LORA_PKT_VERSION, sizeof(lora_packet_t));
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

    /* 3b. Relógio via SNTP: data os registros do histórico.
           Sem sincronizar, o gateway segue e o histórico espera o relógio. */
    ret = time_sync_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SNTP indisponivel: historico aguarda o relogio");
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

    /* 4b. Histórico de 7 dias (flash + snapshot retido no MQTT).
           Falha aqui não impede o repasse ao vivo. */
    ret = history_init(mqtt_publish_history);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Historico desativado: %s", esp_err_to_name(ret));
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

/* ---------- Formatação ---------- */

/**
 * @brief Formata um valor numérico ou "--" se o sensor não tem leitura válida.
 */
static const char *fmt_val(char *buf, size_t size, bool ok, const char *f, double v)
{
    if (ok) {
        snprintf(buf, size, f, v);
    } else {
        snprintf(buf, size, "--");
    }
    return buf;
}

/* ---------- Log serial ---------- */

static void log_packet(const lora_packet_t *pkt, int8_t rssi, int8_t snr)
{
    const bool bme  = lora_packet_has(pkt, LORA_VALID_BME280);
    const bool bh   = lora_packet_has(pkt, LORA_VALID_BH1750);
    const bool mq7  = lora_packet_has(pkt, LORA_VALID_MQ7);
    const bool mq2  = lora_packet_has(pkt, LORA_VALID_MQ2);
    const bool rain = lora_packet_has(pkt, LORA_VALID_RAIN);
    const bool wind = lora_packet_has(pkt, LORA_VALID_WIND);
    const bool gauge = lora_packet_has(pkt, LORA_VALID_GAUGE);
    char a[16], b[16], c[16];

    ESP_LOGI(TAG, "=== Pacote #%lu (RSSI=%d dBm, SNR=%d dB, valid=0x%02X) ===",
             (unsigned long)pkt_count, rssi, snr, pkt->valid);
    ESP_LOGI(TAG, "BME280  | Temp=%s C | Umid=%s %% | Press=%s hPa",
             fmt_val(a, sizeof(a), bme, "%.2f", pkt->temperature_c),
             fmt_val(b, sizeof(b), bme, "%.1f", pkt->humidity_pct),
             fmt_val(c, sizeof(c), bme, "%.2f", pkt->pressure_hpa));
    ESP_LOGI(TAG, "BH1750  | Lux=%s",
             fmt_val(a, sizeof(a), bh, "%.1f", pkt->lux));
    ESP_LOGI(TAG, "MQ-7    | Raw=%s | Voltage=%s V",
             fmt_val(a, sizeof(a), mq7, "%.0f", pkt->mq7_raw),
             fmt_val(b, sizeof(b), mq7, "%.2f", pkt->mq7_voltage));
    ESP_LOGI(TAG, "MQ-2    | Raw=%s | Voltage=%s V",
             fmt_val(a, sizeof(a), mq2, "%.0f", pkt->mq2_raw),
             fmt_val(b, sizeof(b), mq2, "%.2f", pkt->mq2_voltage));
    ESP_LOGI(TAG, "Chuva   | Raw=%s | Voltage=%s V | %s",
             fmt_val(a, sizeof(a), rain, "%.0f", pkt->rain_raw),
             fmt_val(b, sizeof(b), rain, "%.2f", pkt->rain_voltage),
             rain ? (pkt->rain_status ? "CHOVENDO" : "SECO") : "--");
    ESP_LOGI(TAG, "Vento   | Media=%s m/s | Rajada=%s m/s",
             fmt_val(a, sizeof(a), wind, "%.1f", pkt->wind_avg_ms),
             fmt_val(b, sizeof(b), wind, "%.1f", pkt->wind_gust_ms));
    ESP_LOGI(TAG, "Pluvio  | Acumulado=%s mm",
             fmt_val(a, sizeof(a), gauge, "%.2f", pkt->rain_total_mm));
}

/* ---------- OLED ---------- */

static void update_oled(const lora_packet_t *pkt, int8_t rssi, int8_t snr, bool mqtt_ok)
{
    if (!oled_available) { return; }

    const bool bme  = lora_packet_has(pkt, LORA_VALID_BME280);
    const bool bh   = lora_packet_has(pkt, LORA_VALID_BH1750);
    const bool mq7  = lora_packet_has(pkt, LORA_VALID_MQ7);
    const bool mq2  = lora_packet_has(pkt, LORA_VALID_MQ2);
    const bool rain = lora_packet_has(pkt, LORA_VALID_RAIN);
    const bool wind = lora_packet_has(pkt, LORA_VALID_WIND);
    char line[22];
    char a[10], b[10];

    oled_clear(&oled);

    snprintf(line, sizeof(line), "T:%sC  H:%s%%",
             fmt_val(a, sizeof(a), bme, "%.1f", pkt->temperature_c),
             fmt_val(b, sizeof(b), bme, "%.0f", pkt->humidity_pct));
    oled_draw_text(&oled, 0, 0, line);

    snprintf(line, sizeof(line), "P:%s hPa",
             fmt_val(a, sizeof(a), bme, "%.1f", pkt->pressure_hpa));
    oled_draw_text(&oled, 0, 1, line);

    snprintf(line, sizeof(line), "Lux:%s  %s",
             fmt_val(a, sizeof(a), bh, "%.0f", pkt->lux),
             rain ? (pkt->rain_status ? "CHUVA" : "SECO") : "--");
    oled_draw_text(&oled, 0, 2, line);

    snprintf(line, sizeof(line), "CO:%s GLP:%s",
             fmt_val(a, sizeof(a), mq7, "%.0f", pkt->mq7_raw),
             fmt_val(b, sizeof(b), mq2, "%.0f", pkt->mq2_raw));
    oled_draw_text(&oled, 0, 3, line);

    snprintf(line, sizeof(line), "Vento:%s m/s",
             fmt_val(a, sizeof(a), wind, "%.1f", pkt->wind_avg_ms));
    oled_draw_text(&oled, 0, 4, line);

    snprintf(line, sizeof(line), "LoRa:%ddBm SNR:%d", rssi, snr);
    oled_draw_text(&oled, 0, 5, line);

    snprintf(line, sizeof(line), "MQTT:%s WiFi:%s",
             mqtt_ok ? "OK" : "FAIL",
             wifi_manager_is_connected() ? "OK" : "OFF");
    oled_draw_text(&oled, 0, 6, line);

    snprintf(line, sizeof(line), "Pkts:%lu | GW v2.0",
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
            ESP_LOGW(TAG, "Pacote tamanho invalido: %d (esperado %zu — no sensor com firmware antigo?)",
                     rx_len, sizeof(lora_packet_t));
            continue;
        }

        /* Desserializa e valida */
        lora_packet_t pkt;
        memcpy(&pkt, rx_buf, sizeof(pkt));

        if (!lora_packet_validate(&pkt)) {
            ESP_LOGW(TAG, "Pacote invalido (magic/versao/CRC) — descartado");
            continue;
        }

        pkt_count++;

        /* 1. Log serial */
        log_packet(&pkt, rssi, snr);

        /* 2. Publica via MQTT */
        mqtt_ok = (mqtt_publish_data(&pkt, rssi, snr) == ESP_OK);
        if (mqtt_ok) {
            ESP_LOGI(TAG, "MQTT publicado no topico '%s'", MQTT_TOPIC_DATA);
        } else {
            ESP_LOGW(TAG, "MQTT publish falhou");
        }

        /* 3. Histórico de 7 dias (agrega em 5 min; independe do MQTT estar no ar) */
        history_add(&pkt);

        /* 4. Atualiza OLED */
        update_oled(&pkt, rssi, snr, mqtt_ok);
    }
}
