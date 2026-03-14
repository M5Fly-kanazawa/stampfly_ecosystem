idf_component_register(
    SRCS "buzzer.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
    PRIV_REQUIRES nvs_flash
)
