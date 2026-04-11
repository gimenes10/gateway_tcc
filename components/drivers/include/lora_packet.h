/**
 * @file lora_packet.h
 * @brief Formato do pacote de dados da estação meteorológica para transmissão LoRa.
 *
 * Pacote binário compacto (65 bytes) com todos os dados dos sensores.
 * Compartilhado entre nó sensor (TX) e nó gateway (RX).
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Magic bytes para validar integridade do pacote */
#define LORA_PKT_MAGIC_0   0xCA
#define LORA_PKT_MAGIC_1   0xFE

/**
 * @brief Estrutura do pacote da estação meteorológica.
 *
 * Layout binário (little-endian, packed):
 *   [2B magic][4B temp][4B umid][4B press][4B lux]
 *   [2B mq7_raw][4B mq7_v][2B mq2_raw][4B mq2_v]
 *   [2B rain_raw][4B rain_v][1B rain_status]
 *   [12B accel][12B gyro][4B imu_temp]
 *   [2B crc16]
 *   Total: 65 bytes
 */
typedef struct __attribute__((packed)) {
    /* Header */
    uint8_t  magic[2];

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

    /* MPU-6050 */
    float    accel_x;
    float    accel_y;
    float    accel_z;
    float    gyro_x;
    float    gyro_y;
    float    gyro_z;
    float    imu_temp_c;

    /* Integridade */
    uint16_t crc16;
} lora_packet_t;

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
 */
static inline void lora_packet_finalize(lora_packet_t *pkt)
{
    pkt->magic[0] = LORA_PKT_MAGIC_0;
    pkt->magic[1] = LORA_PKT_MAGIC_1;
    pkt->crc16 = lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
}

/**
 * @brief Valida magic bytes e CRC de um pacote recebido.
 * @return true se válido
 */
static inline bool lora_packet_validate(const lora_packet_t *pkt)
{
    if (pkt->magic[0] != LORA_PKT_MAGIC_0 || pkt->magic[1] != LORA_PKT_MAGIC_1) {
        return false;
    }
    uint16_t expected = lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
    return (pkt->crc16 == expected);
}
