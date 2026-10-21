/**
 * @file lora_packet.h
 * @brief Pacote LoRa da estação meteorológica (v4, 37 bytes).
 *
 * Compartilhado entre o nó sensor (TX) e o gateway (RX): manter os dois
 * arquivos idênticos.
 *
 * Grandezas em inteiros escalados (sem float no ar). Cada grupo tem um bit
 * em `valid`; bit em 0 = sem leitura, o campo deve ser ignorado e o gateway
 * publica null no JSON.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define LORA_PKT_MAGIC_0    0xCA
#define LORA_PKT_MAGIC_1    0xFE
#define LORA_PKT_VERSION    4

/* Bits de `valid` */
#define LORA_VALID_THP      (1U << 0)   /* BME680: temp_c100, hum_x100, press_x10 */
#define LORA_VALID_GAS      (1U << 1)   /* BME680: gas_ohm */
#define LORA_VALID_LUX      (1U << 2)   /* BH1750: lux_x10 */
#define LORA_VALID_DUST     (1U << 3)   /* GP2Y1014: dust_ugm3, dust_vo_mv */
#define LORA_VALID_RAIN     (1U << 4)   /* FC-22: rain_mv, LORA_FLAG_RAIN_WET */
#define LORA_VALID_WIND     (1U << 5)   /* Anemômetro: wind_cms, gust_cms */
#define LORA_VALID_GAUGE    (1U << 6)   /* Pluviômetro: rain_x100 */

/* Bits de `flags` */
#define LORA_FLAG_RAIN_WET  (1U << 0)   /* FC-22 abaixo do limiar */

typedef struct __attribute__((packed)) {
    uint8_t  magic[2];
    uint8_t  version;
    uint8_t  flags;
    uint8_t  valid;
    uint16_t seq;           /* Sequência desde o boot (perda de pacotes, reboot) */

    int16_t  temp_c100;     /* °C ×100 */
    uint16_t hum_x100;      /* % ×100 */
    uint16_t press_x10;     /* hPa ×10, pressão na estação */
    uint32_t gas_ohm;       /* Resistência do gás, Ω (maior = ar mais limpo) */
    uint32_t lux_x10;       /* lux ×10 */
    uint16_t dust_ugm3;     /* Densidade de poeira/fumaça, µg/m³ */
    uint16_t dust_vo_mv;    /* Vo do GP2Y, mV (para calibração) */
    uint16_t rain_mv;       /* AO do FC-22, mV */
    uint16_t wind_cms;      /* Vento médio no intervalo, cm/s */
    uint16_t gust_cms;      /* Rajada (maior média de 3 s), cm/s */
    uint32_t rain_x100;     /* Acumulado do pluviômetro, mm ×100 (cai = reboot) */

    uint16_t crc16;         /* CRC-16/CCITT dos bytes anteriores */
} lora_packet_t;

_Static_assert(sizeof(lora_packet_t) == 37, "lora_packet_t v4 deve ter 37 bytes");

#define LORA_PKT_CRC_OFFSET  (sizeof(lora_packet_t) - sizeof(uint16_t))

static inline uint16_t lora_packet_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/** @brief Preenche magic, versão e CRC. `valid` e os campos vêm antes. */
static inline void lora_packet_finalize(lora_packet_t *pkt)
{
    pkt->magic[0] = LORA_PKT_MAGIC_0;
    pkt->magic[1] = LORA_PKT_MAGIC_1;
    pkt->version  = LORA_PKT_VERSION;
    pkt->crc16    = lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
}

/** @brief Valida magic, versão e CRC. */
static inline bool lora_packet_validate(const lora_packet_t *pkt)
{
    return pkt->magic[0] == LORA_PKT_MAGIC_0 &&
           pkt->magic[1] == LORA_PKT_MAGIC_1 &&
           pkt->version  == LORA_PKT_VERSION &&
           pkt->crc16    == lora_packet_crc16((const uint8_t *)pkt, LORA_PKT_CRC_OFFSET);
}

static inline bool lora_packet_has(const lora_packet_t *pkt, uint8_t bit)
{
    return (pkt->valid & bit) != 0;
}
