idf_component_register(
    SRCS "failsafe.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer sf_core sf_state
)
