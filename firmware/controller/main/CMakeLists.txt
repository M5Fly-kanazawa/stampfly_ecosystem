idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_wifi
        esp_timer
        driver
        freertos
        nvs_flash
        spiffs
        buzzer
        atoms3joy
        espnow_tdma
        menu_system
        usb_hid
)
