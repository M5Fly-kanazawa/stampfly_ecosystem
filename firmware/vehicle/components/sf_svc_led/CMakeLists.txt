idf_component_register(
    SRCS "led_manager.cpp"
    INCLUDE_DIRS "include"
    REQUIRES sf_hal_led sf_svc_state freertos esp_timer
)
