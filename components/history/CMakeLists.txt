idf_component_register(
    SRCS "history.c"
    INCLUDE_DIRS "include"
    REQUIRES drivers
    PRIV_REQUIRES esp_partition
)
