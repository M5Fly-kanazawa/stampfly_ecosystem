idf_component_register(
    SRCS "comm.cpp"
    INCLUDE_DIRS "include"
    REQUIRES protocol sf_core sf_board esp_wifi esp_event esp_netif esp_timer nvs_flash
)
