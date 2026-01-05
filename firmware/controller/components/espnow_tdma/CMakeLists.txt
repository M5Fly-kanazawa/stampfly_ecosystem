idf_component_register(
    SRCS "espnow_tdma.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES
        esp_wifi
        esp_event
        esp_timer
        esp_netif
        nvs_flash
        spiffs
        vfs
        buzzer
)
