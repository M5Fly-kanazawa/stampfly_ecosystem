idf_component_register(
    SRCS "led.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver led_strip esp_timer nvs_flash
)
