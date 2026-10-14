/**
 * @file time_sync.h
 * @brief Sincronização do relógio via SNTP (necessária para datar o histórico).
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Inicia o SNTP e espera a primeira sincronização por até 10 s.
 *        Se não sincronizar a tempo, segue tentando em segundo plano.
 * @return Erro apenas se o cliente SNTP não puder ser criado.
 */
esp_err_t time_sync_init(void);

/**
 * @brief Retorna true se o relógio já foi acertado.
 */
bool time_sync_is_valid(void);
