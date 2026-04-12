idf_component_register(
    SRCS
        "pid_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_state
        sf_controller
)
