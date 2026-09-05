idf_component_register(SRCS "main.cpp"
                             "internal_sensor_feed.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES sf_api
                                sf_math
                                sf_estimator
                                sf_estimator_complementary
                                sf_hal_bmi270)
