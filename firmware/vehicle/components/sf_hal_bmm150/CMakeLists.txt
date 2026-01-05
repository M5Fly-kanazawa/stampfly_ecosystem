idf_component_register(
    SRCS "bmm150.cpp" "mag_calibration.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer nvs_flash
)
