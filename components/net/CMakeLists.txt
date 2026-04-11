idf_component_register(
    SRCS
        "wifi_manager.c"
        "mqtt_publisher.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif mqtt nvs_flash mbedtls
    PRIV_REQUIRES board_config drivers
)