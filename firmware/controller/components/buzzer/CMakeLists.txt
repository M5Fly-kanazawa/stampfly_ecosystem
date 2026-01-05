idf_component_register(
    SRCS "buzzer.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver freertos esp_driver_ledc
)
