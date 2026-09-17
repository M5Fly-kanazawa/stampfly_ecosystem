idf_component_register(
    SRCS
        "snn_controller.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_state
        sf_controller
        sf_math
        sf_snn_core
)
