/**
 * @file oled_ssd1306.h
 * @brief Driver mínimo do display OLED SSD1306 128x64 via I2C.
 *
 * Usa framebuffer de 1024 bytes (128x64/8) e fonte 5x7 embutida.
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#define OLED_WIDTH   128
#define OLED_HEIGHT  64
#define OLED_PAGES   (OLED_HEIGHT / 8)

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t framebuf[OLED_WIDTH * OLED_PAGES];
} oled_handle_t;

esp_err_t oled_init(oled_handle_t *oled, i2c_master_bus_handle_t bus);
void      oled_clear(oled_handle_t *oled);
esp_err_t oled_update(oled_handle_t *oled);

/**
 * @brief Escreve texto no framebuffer usando fonte 5x7.
 * @param oled  Handle do display
 * @param x     Posição X (0..127)
 * @param y     Posição Y em páginas (0..7), cada página = 8 pixels
 * @param text  String a escrever
 */
void oled_draw_text(oled_handle_t *oled, uint8_t x, uint8_t y, const char *text);
