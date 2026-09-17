idf_component_register(
    SRCS
        "snn_estimator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_estimator
        sf_math
        sf_snn_core
)
