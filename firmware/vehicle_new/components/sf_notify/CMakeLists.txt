idf_component_register(
    SRCS "notify.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sf_core sf_state sf_hal_led sf_hal_buzzer
)
