/**
 * @file gateway_task.h
 * @brief Task principal do gateway: recebe pacotes LoRa e exibe no serial + OLED.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Inicializa LoRa RX e OLED.
 */
esp_err_t gateway_init(void);

/**
 * @brief Task FreeRTOS que aguarda pacotes LoRa e decodifica.
 */
void gateway_task(void *arg);
