/**
 * @file wifi_manager.h
 * @brief Conexão WiFi STA com reconexão automática.
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Inicializa NVS, netif, event loop e conecta ao WiFi.
 *        Bloqueia até conectar ou timeout de 30s.
 * @return ESP_OK quando conectado
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Retorna true se o WiFi está conectado e com IP.
 */
bool wifi_manager_is_connected(void);
