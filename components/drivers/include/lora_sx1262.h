/**
 * @file lora_sx1262.h
 * @brief Driver do transceiver LoRa SX1262 via SPI para o Heltec WiFi LoRa 32 V3.
 *
 * Implementa inicialização, configuração, transmissão e recepção de pacotes LoRa.
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    void *spi_dev;  /* spi_device_handle_t (opaco) */
} lora_sx1262_handle_t;

/**
 * @brief Inicializa o SX1262: reset, configura SPI, define parâmetros RF.
 */
esp_err_t lora_sx1262_init(lora_sx1262_handle_t *handle);

/**
 * @brief Transmite um pacote LoRa e aguarda conclusão.
 */
esp_err_t lora_sx1262_transmit(lora_sx1262_handle_t *handle,
                               const uint8_t *data, uint8_t len);

/**
 * @brief Coloca o rádio em modo RX e aguarda recepção de um pacote.
 *
 * @param handle      Handle do driver
 * @param[out] data   Buffer para receber os dados (mín. 255 bytes)
 * @param[out] len    Tamanho do pacote recebido
 * @param[out] rssi   RSSI do pacote em dBm (pode ser NULL)
 * @param[out] snr    SNR do pacote em dB (pode ser NULL)
 * @param timeout_ms  Timeout de espera (0 = RX contínuo até receber)
 * @return ESP_OK em sucesso, ESP_ERR_TIMEOUT se expirou
 */
esp_err_t lora_sx1262_receive(lora_sx1262_handle_t *handle,
                              uint8_t *data, uint8_t *len,
                              int8_t *rssi, int8_t *snr,
                              uint32_t timeout_ms);
