idf_component_register(
    SRCS "takeoff_landing.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        esp_timer sf_core sf_state
)
