idf_component_register(
    SRCS
        "complementary_estimator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES
        sf_core
        sf_estimator
        sf_math
)
