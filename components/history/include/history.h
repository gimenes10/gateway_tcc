/**
 * @file history.h
 * @brief Histórico de 7 dias da estação: agrega em intervalos de 5 min,
 *        persiste em flash (buffer circular) e publica um snapshot binário.
 *
 * Snapshot publicado (little-endian), tópico MQTT retido:
 *   [0]   'H'
 *   [1]   'S'
 *   [2]   versão do formato (HISTORY_FMT_VERSION)
 *   [3]   tamanho do registro em bytes (HISTORY_REC_SIZE)
 *   [4:5] período de agregação em segundos (uint16)
 *   [6:7] quantidade de registros N (uint16)
 *   [8..] N registros, do mais antigo para o mais novo
 *
 * Registro v2 (24 bytes). Médias do intervalo, exceto onde indicado;
 * 0xFFFF (INT16_MIN nos campos com sinal) = sem dado.
 *   uint32 ts            início do intervalo, Unix UTC (s)
 *   int16  temp_c100     °C ×100
 *   uint16 hum_x10       % ×10
 *   uint16 press_x10     hPa ×10
 *   uint16 lux_2         lux ÷ 2 (satura em 131068 lux)
 *   uint16 gas_kohm_x10  resistência do gás do BME680, kΩ ×10
 *   uint16 dust_ugm3     poeira/fumaça do GP2Y, µg/m³
 *   uint16 wind_x10      vento médio, m/s ×10
 *   uint16 gust_x10      maior rajada do intervalo, m/s ×10
 *   uint16 rain_mm_x10   chuva do pluviômetro no intervalo (soma), mm ×10
 *   uint8  rain_pct      % das amostras com chuva no FC-22 (0xFF = sem dado)
 *   uint8  crc8          CRC-8 (poli 0x07, semente = versão) dos 23 bytes anteriores
 */
#pragma once

#include "esp_err.h"
#include "lora_packet.h"
#include <stddef.h>
#include <stdint.h>

#define HISTORY_FMT_VERSION   2
#define HISTORY_REC_SIZE      24
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
