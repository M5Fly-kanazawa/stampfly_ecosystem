idf_component_register(
    SRCS
        "main.cpp"
        "../tasks/imu_task.cpp"
        "../tasks/control_task.cpp"
        "../tasks/state_task.cpp"
    INCLUDE_DIRS "." "../tasks"
    REQUIRES
        esp_timer
        nvs_flash
        sf_core
        sf_state
        sf_estimator
        sf_estimator_eskf
        sf_controller
        sf_controller_pid
)
