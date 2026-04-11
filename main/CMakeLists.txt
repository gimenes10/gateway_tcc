idf_component_register(
    SRCS
        "main.c"
        "gateway_task.c"
    INCLUDE_DIRS "include"
    REQUIRES board_config drivers net
)
