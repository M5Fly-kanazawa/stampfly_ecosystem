idf_component_register(
    SRCS "controller_comm.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event nvs_flash freertos
)
