/**
 * @file lora_packet.h
 * @brief Formato do pacote de dados da estação meteorológica para transmissão LoRa.
 *
 * Pacote binário compacto (v2, 45 bytes) com os dados dos sensores.
 * Compartilhado entre nó sensor (TX) e nó gateway (RX) — manter os dois
 * arquivos idênticos.
 *
 * Cada sensor tem um bit em `valid`. Campo com bit em 0 = sem leitura
 * (sensor ausente ou falhou); o valor do campo deve ser ignorado e o
 * gateway publica `null` no JSON.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Magic bytes + versão para validar integridade e compatibilidade */
#define LORA_PKT_MAGIC_0   0xCA
#define LORA_PKT_MAGIC_1   0xFE
#define LORA_PKT_VERSION   2

/* Bits de validade por sensor (campo `valid`) */
#define LORA_VALID_BME280  (1U << 0)   /* temperatura, umidade, pressão */
#define LORA_VALID_BH1750  (1U << 1)   /* lux */
#define LORA_VALID_MQ7     (1U << 2)   /* mq7_raw, mq7_voltage */
#define LORA_VALID_MQ2     (1U << 3)   /* mq2_raw, mq2_voltage */
#define LORA_VALID_RAIN    (1U << 4)   /* rain_raw, rain_voltage, rain_status */
#define LORA_VALID_WIND    (1U << 5)   /* wind_ms — anemômetro (futuro) */

/**
 * @brief Estrutura do pacote da estação meteorológica (v2).
 *
 * Layout binário (little-endian, packed):
 *   [2B magic][1B version][1B valid]
 *   [4B temp][4B umid][4B press][4B lux]
 *   [2B mq7_raw][4B mq7_v][2B mq2_raw][4B mq2_v]
 *   [2B rain_raw][4B rain_v][1B rain_status]
 *   [4B wind_ms]
 *   [2B crc16]
 *   Total: 45 bytes
 */
typedef struct __attribute__((packed)) {
    /* Header */
    uint8_t  magic[2];
    uint8_t  version;
    uint8_t  valid;         /* Bitmask LORA_VALID_* */

    /* BME280 */
    float    temperature_c;
    float    humidity_pct;
    float    pressure_hpa;

    /* BH1750 */
    float    lux;

    /* MQ-7 */
    int16_t  mq7_raw;
    float    mq7_voltage;

    /* MQ-2 */
    int16_t  mq2_raw;
    float    mq2_voltage;

    /* Chuva FC-22 */
    int16_t  rain_raw;
    float    rain_voltage;
    uint8_t  rain_status;   /* 1 = chovendo, 0 = seco */

    /* Anemômetro (futuro) */
    float    wind_ms;

    /* Integridade */
    uint16_t crc16;
} lora_packet_t;

_Static_assert(sizeof(lora_packet_t) == 45, "lora_packet_t v2 deve ter 45 bytes");

/* Tamanho do payload sem CRC (para cálculo do CRC) */
#define LORA_PKT_CRC_OFFSET  (sizeof(lora_packet_t) - sizeof(uint16_t))

/**
 * @brief Calcula CRC-16/CCITT do payload.
 */
static inline uint16_t lora_packet_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

/**
 * @brief Preenche o header e calcula o CRC de um pacote.
 *        O campo `valid` deve ser preenchido antes.
 */
static inline void lora_packet_finalize(lora_packet_t *pkt)
{
    pkt->magic[0] = LORA_PKT_MAGIC_0;
    pkt->magic[1] = LORA_PKT_MAGIC_1;
    pkt->version  = LORA_PKT_VERSION;
    pkt->crc16 = lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
}

/**
 * @brief Valida magic, versão e CRC de um pacote recebido.
 * @return true se válido
 */
static inline bool lora_packet_validate(const lora_packet_t *pkt)
{
    if (pkt->magic[0] != LORA_PKT_MAGIC_0 || pkt->magic[1] != LORA_PKT_MAGIC_1) {
        return false;
    }
    if (pkt->version != LORA_PKT_VERSION) {
        return false;
    }
    uint16_t expected = lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
    return (pkt->crc16 == expected);
}

/**
 * @brief Retorna true se o sensor indicado pelo bit tem leitura válida.
 */
static inline bool lora_packet_has(const lora_packet_t *pkt, uint8_t bit)
{
    return (pkt->valid & bit) != 0;
}
