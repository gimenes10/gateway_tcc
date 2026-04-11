#include "i2c_bus.h"
#include "pin_config.h"
#include "esp_log.h"

static const char *TAG = "i2c_bus";

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *sensor_bus,
                       i2c_master_bus_handle_t *oled_bus)
{
    esp_err_t ret;

    /* --- Barramento dos sensores (I2C_NUM_1) --- */
    i2c_master_bus_config_t sensor_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = SENSOR_I2C_PORT,
        .scl_io_num = SENSOR_I2C_SCL,
        .sda_io_num = SENSOR_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  /* Pull-ups nos módulos breakout */
    };

    ret = i2c_new_master_bus(&sensor_cfg, sensor_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar barramento de sensores: %s", esp_err_to_name(ret));
        goto err_sensor;
    }
    ESP_LOGI(TAG, "Barramento I2C sensores OK (SDA=%d, SCL=%d)",
             SENSOR_I2C_SDA, SENSOR_I2C_SCL);

    /* --- Barramento do OLED (I2C_NUM_0) --- */
    i2c_master_bus_config_t oled_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port   = OLED_I2C_PORT,
        .scl_io_num = OLED_I2C_SCL,
        .sda_io_num = OLED_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* OLED usa pull-up interno */
    };

    ret = i2c_new_master_bus(&oled_cfg, oled_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar barramento OLED: %s", esp_err_to_name(ret));
        goto err_oled;
    }
    ESP_LOGI(TAG, "Barramento I2C OLED OK (SDA=%d, SCL=%d)",
             OLED_I2C_SDA, OLED_I2C_SCL);

    return ESP_OK;

err_oled:
    i2c_del_master_bus(*sensor_bus);
err_sensor:
    return ret;
}

esp_err_t i2c_bus_add_device(i2c_master_bus_handle_t bus_handle,
                             uint8_t addr, uint32_t speed_hz,
                             i2c_master_dev_handle_t *dev)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = speed_hz,
    };
    return i2c_master_bus_add_device(bus_handle, &cfg, dev);
}
