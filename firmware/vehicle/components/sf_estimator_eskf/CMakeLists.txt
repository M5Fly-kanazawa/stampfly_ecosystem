idf_component_register(
    SRCS
        "eskf_estimator.cpp"
        "eskf_core.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_estimator
        sf_math
)
