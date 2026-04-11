idf_component_register(
    SRCS
        "i2c_bus.c"
        "oled_ssd1306.c"
        "lora_sx1262.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_spi
    PRIV_REQUIRES board_config
)
