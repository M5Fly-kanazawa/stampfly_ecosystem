idf_component_register(
    SRCS
        "board.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_event
        esp_netif
        esp_wifi
        nvs_flash
        sf_core
        sf_hal_led
)
