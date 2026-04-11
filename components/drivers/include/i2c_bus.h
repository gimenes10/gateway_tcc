/**
 * @file i2c_bus.h
 * @brief Inicialização dos barramentos I2C (sensores e OLED).
 *
 * Cria dois barramentos I2C independentes:
 * - I2C_NUM_1 para sensores externos (BME280, MPU-6050, BH1750)
 * - I2C_NUM_0 para o display OLED SSD1306
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/**
 * @brief Inicializa ambos os barramentos I2C.
 *
 * @param[out] sensor_bus  Handle do barramento dos sensores
 * @param[out] oled_bus    Handle do barramento do OLED
 * @return ESP_OK em sucesso, código de erro caso contrário
 */
esp_err_t i2c_bus_init(i2c_master_bus_handle_t *sensor_bus,
                       i2c_master_bus_handle_t *oled_bus);

/**
 * @brief Adiciona um dispositivo I2C ao barramento especificado.
 *
 * @param bus_handle  Handle do barramento
 * @param addr        Endereço I2C de 7 bits
 * @param speed_hz    Velocidade do clock SCL em Hz
 * @param[out] dev    Handle do dispositivo criado
 * @return ESP_OK em sucesso
 */
esp_err_t i2c_bus_add_device(i2c_master_bus_handle_t bus_handle,
                             uint8_t addr, uint32_t speed_hz,
                             i2c_master_dev_handle_t *dev);
