idf_component_register(
    SRCS
        "eskf_estimator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_estimator
)
