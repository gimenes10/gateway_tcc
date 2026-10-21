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

static void log_packet(const lora_packet_t *p, int8_t rssi, int8_t snr)
{
    const bool thp  = lora_packet_has(p, LORA_VALID_THP);
    const bool gas  = lora_packet_has(p, LORA_VALID_GAS);
    const bool lux  = lora_packet_has(p, LORA_VALID_LUX);
    const bool dust = lora_packet_has(p, LORA_VALID_DUST);
    const bool rain = lora_packet_has(p, LORA_VALID_RAIN);
    const bool wind = lora_packet_has(p, LORA_VALID_WIND);
    const bool gaug = lora_packet_has(p, LORA_VALID_GAUGE);
    char a[16], b[16], c[16], d[16];

    ESP_LOGI(TAG, "=== Pacote #%lu seq=%u (RSSI=%d dBm, SNR=%d dB, valid=0x%02X) ===",
             (unsigned long)pkt_count, p->seq, rssi, snr, p->valid);
    ESP_LOGI(TAG, "BME680  | T=%s C | UR=%s %% | P=%s hPa | gas=%s kOhm",
             fmt_val(a, sizeof(a), thp, "%.2f", p->temp_c100 / 100.0),
             fmt_val(b, sizeof(b), thp, "%.1f", p->hum_x100 / 100.0),
             fmt_val(c, sizeof(c), thp, "%.1f", p->press_x10 / 10.0),
             fmt_val(d, sizeof(d), gas, "%.1f", p->gas_ohm / 1000.0));
    ESP_LOGI(TAG, "BH1750  | %s lux | GP2Y %s ug/m3 (Vo %s mV)",
             fmt_val(a, sizeof(a), lux, "%.0f", p->lux_x10 / 10.0),
             fmt_val(b, sizeof(b), dust, "%.0f", p->dust_ugm3),
             fmt_val(c, sizeof(c), dust, "%.0f", p->dust_vo_mv));
    ESP_LOGI(TAG, "Chuva   | FC-22 %s mV %s | pluvio %s mm",
             fmt_val(a, sizeof(a), rain, "%.0f", p->rain_mv),
             rain ? ((p->flags & LORA_FLAG_RAIN_WET) ? "MOLHADO" : "SECO") : "--",
             fmt_val(b, sizeof(b), gaug, "%.2f", p->rain_x100 / 100.0));
    ESP_LOGI(TAG, "Vento   | media %s m/s | rajada %s m/s",
             fmt_val(a, sizeof(a), wind, "%.2f", p->wind_cms / 100.0),
             fmt_val(b, sizeof(b), wind, "%.2f", p->gust_cms / 100.0));
}

/* ---------- OLED ---------- */

static void update_oled(const lora_packet_t *p, int8_t rssi, int8_t snr, bool mqtt_ok)
{
    if (!oled_available) { return; }

    const bool thp  = lora_packet_has(p, LORA_VALID_THP);
    const bool lux  = lora_packet_has(p, LORA_VALID_LUX);
    const bool dust = lora_packet_has(p, LORA_VALID_DUST);
    const bool gas  = lora_packet_has(p, LORA_VALID_GAS);
    const bool rain = lora_packet_has(p, LORA_VALID_RAIN);
    const bool wind = lora_packet_has(p, LORA_VALID_WIND);
    char line[22];
    char a[10], b[10];

    oled_clear(&oled);

    snprintf(line, sizeof(line), "T:%sC  H:%s%%",
             fmt_val(a, sizeof(a), thp, "%.1f", p->temp_c100 / 100.0),
             fmt_val(b, sizeof(b), thp, "%.0f", p->hum_x100 / 100.0));
    oled_draw_text(&oled, 0, 0, line);

    snprintf(line, sizeof(line), "P:%s hPa",
             fmt_val(a, sizeof(a), thp, "%.1f", p->press_x10 / 10.0));
    oled_draw_text(&oled, 0, 1, line);

    snprintf(line, sizeof(line), "Lux:%s  %s",
             fmt_val(a, sizeof(a), lux, "%.0f", p->lux_x10 / 10.0),
             rain ? ((p->flags & LORA_FLAG_RAIN_WET) ? "CHUVA" : "SECO") : "--");
    oled_draw_text(&oled, 0, 2, line);

    snprintf(line, sizeof(line), "Po:%s VOC:%sk",
             fmt_val(a, sizeof(a), dust, "%.0f", p->dust_ugm3),
             fmt_val(b, sizeof(b), gas, "%.0f", p->gas_ohm / 1000.0));
    oled_draw_text(&oled, 0, 3, line);

    snprintf(line, sizeof(line), "Vento:%s Raj:%s",
             fmt_val(a, sizeof(a), wind, "%.1f", p->wind_cms / 100.0),
             fmt_val(b, sizeof(b), wind, "%.1f", p->gust_cms / 100.0));
    oled_draw_text(&oled, 0, 4, line);

    snprintf(line, sizeof(line), "LoRa:%ddBm SNR:%d", rssi, snr);
    oled_draw_text(&oled, 0, 5, line);

    snprintf(line, sizeof(line), "MQTT:%s WiFi:%s",
             mqtt_ok ? "OK" : "FAIL",
             wifi_manager_is_connected() ? "OK" : "OFF");
    oled_draw_text(&oled, 0, 6, line);

    snprintf(line, sizeof(line), "Pkts:%lu | GW v4",
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
