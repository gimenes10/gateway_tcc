/**
 * @file main.c
 * @brief Ponto de entrada do gateway LoRa da estação meteorológica.
 *
 * Projeto: TCC — Estação Meteorológica IoT com comunicação LoRa
 * Hardware: Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262)
 * Framework: ESP-IDF v6.0
 *
 * Função: Recebe pacotes LoRa do nó sensor, decodifica e exibe
 * os dados no monitor serial e no display OLED.
 * Futuramente: WiFi + MQTT para repasse dos dados ao broker.
 */

#include "gateway_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

#define GATEWAY_TASK_STACK_SIZE  8192
#define GATEWAY_TASK_PRIORITY    5

void app_main(void)
{
    esp_err_t ret = gateway_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha na inicializacao: %s", esp_err_to_name(ret));
        return;
    }

    BaseType_t xret = xTaskCreate(
        gateway_task,
        "gateway_task",
        GATEWAY_TASK_STACK_SIZE,
        NULL,
        GATEWAY_TASK_PRIORITY,
        NULL
    );

    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Falha ao criar gateway_task");
        return;
    }

    ESP_LOGI(TAG, "gateway_task criada — RX ativo");
}
