/**
 * @file history.h
 * @brief Histórico de 7 dias da estação: agrega em intervalos de 5 min,
 *        persiste em flash (buffer circular) e publica um snapshot binário.
 *
 * Snapshot publicado (little-endian), tópico MQTT retido:
 *   [0]   'H'
 *   [1]   'S'
 *   [2]   versão do formato (HISTORY_FMT_VERSION)
 *   [3]   tamanho do registro em bytes (24)
 *   [4:5] período de agregação em segundos (uint16)
 *   [6:7] quantidade de registros N (uint16)
 *   [8..] N registros, do mais antigo para o mais novo
 *
 * Registro (24 bytes):
 *   uint32 ts           início do intervalo, Unix UTC (s)
 *   int16  temp_c100    °C ×100            (INT16_MIN = sem dado)
 *   uint16 hum_x10      % ×10              (0xFFFF   = sem dado)
 *   uint16 press_x10    hPa ×10
 *   uint16 lux          lux (satura em 65534)
 *   uint16 mq7_mv       tensão MQ-7 em mV
 *   uint16 mq2_mv       tensão MQ-2 em mV
 *   uint16 wind_x10     vento médio, m/s ×10
 *   uint16 gust_x10     maior rajada, m/s ×10
 *   uint16 rain_mm_x10  chuva do pluviômetro no intervalo, mm ×10
 *   uint8  rain_pct     % das amostras com chuva no FC-22 (0xFF = sem dado)
 *   uint8  crc8         CRC-8 (poli 0x07) dos 23 bytes anteriores
 */
#pragma once

#include "esp_err.h"
#include "lora_packet.h"
#include <stddef.h>
#include <stdint.h>

#define HISTORY_FMT_VERSION   1
#define HISTORY_PERIOD_S      300
#define HISTORY_DAYS          7
#define HISTORY_MAX_RECORDS   (HISTORY_DAYS * 86400 / HISTORY_PERIOD_S)   /* 2016 */

/**
 * @brief Função que publica o snapshot (ex.: mqtt_publish_history).
 * @return ESP_OK se a mensagem foi entregue ao cliente MQTT.
 */
typedef esp_err_t (*history_publish_fn_t)(const uint8_t *buf, size_t len);

/**
 * @brief Localiza a partição "hist", recupera o histórico salvo e cria a task
 *        que fecha intervalos e publica o snapshot.
 * @param publish Callback de publicação (não pode ser NULL).
 * @return ESP_ERR_NOT_FOUND se a partição não existir na tabela.
 */
esp_err_t history_init(history_publish_fn_t publish);

/**
 * @brief Acumula um pacote válido no intervalo corrente.
 *        Ignora a amostra enquanto o relógio não foi sincronizado por SNTP.
 */
void history_add(const lora_packet_t *pkt);
