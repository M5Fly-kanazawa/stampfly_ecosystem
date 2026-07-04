idf_component_register(
    SRCS "actuator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sf_core sf_hal_motor esp_timer
)
