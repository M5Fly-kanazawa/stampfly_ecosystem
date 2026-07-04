idf_component_register(
    SRCS "stampfly_state.cpp" "system_manager.cpp" "system_state.cpp" "state_transitions.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos sf_algo_filter sf_svc_control_arbiter
    PRIV_REQUIRES esp_timer
)
