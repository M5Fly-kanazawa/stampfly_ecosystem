idf_component_register(
    SRCS "bmp280.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)
