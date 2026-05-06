idf_component_register(
    SRCS "comm.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sf_core esp_wifi esp_event esp_netif esp_timer nvs_flash
)
