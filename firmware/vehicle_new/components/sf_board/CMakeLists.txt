idf_component_register(
    SRCS
        "board.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_event
        esp_netif
        nvs_flash
        sf_core
)
