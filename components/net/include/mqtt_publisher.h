/**
 * @file mqtt_publisher.h
 * @brief Cliente MQTT para publicação dos dados da estação meteorológica.
 */
#pragma once

#include "esp_err.h"
#include "lora_packet.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Conecta ao broker MQTT (HiveMQ Cloud via TLS).
 *        Bloqueia até conectar ou timeout de 15s.
 * @return ESP_OK quando conectado
 */
esp_err_t mqtt_publisher_init(void);

/**
 * @brief Publica os dados de um pacote no tópico MQTT como JSON.
 *        Campos de sensores sem bit em `pkt->valid` são publicados como null.
 * @param pkt   Pacote decodificado (v4)
 * @param rssi  RSSI do LoRa em dBm
 * @param snr   SNR do LoRa em dB
 * @return ESP_OK em sucesso
 */
esp_err_t mqtt_publish_data(const lora_packet_t *pkt, int8_t rssi, int8_t snr);

/**
 * @brief Publica o snapshot binário do histórico como mensagem retida (QoS 0).
 *        O broker entrega a última versão a cada cliente que assina o tópico.
 * @param buf  Snapshot no formato descrito em history.h
 * @param len  Tamanho em bytes (até ~48 KB; o cliente fragmenta o envio)
 * @return ESP_OK em sucesso
 */
esp_err_t mqtt_publish_history(const uint8_t *buf, size_t len);

/**
 * @brief Retorna true se o MQTT está conectado.
 */
bool mqtt_publisher_is_connected(void);