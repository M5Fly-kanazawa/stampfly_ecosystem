idf_component_register(SRCS "main.cpp"
                             "learner_controller.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES sf_controller
                                sf_controller_pid)
