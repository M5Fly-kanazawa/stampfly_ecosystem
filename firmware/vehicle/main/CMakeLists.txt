# Application sources (L1 entry point, see docs/plans/sf-app-sils-plan.md):
# when SF_APP_DIR is set, every *.cpp in it is compiled into this component and
# provides sf::app::* (sf_app_hooks then omits its default implementation).
# アプリのソース（L1 の入口）: SF_APP_DIR が与えられたら、その *.cpp を本コンポーネントに
# 取り込み sf::app::* の実装とする（sf_app_hooks は既定実装を外す）。
set(SF_APP_SRCS "")
set(SF_APP_INCS "")
if(SF_APP_DIR)
    file(GLOB SF_APP_SRCS CONFIGURE_DEPENDS "${SF_APP_DIR}/*.cpp")
    if(NOT SF_APP_SRCS)
        message(FATAL_ERROR "SF_APP_DIR=${SF_APP_DIR} contains no *.cpp")
    endif()
    set(SF_APP_INCS "${SF_APP_DIR}")
    message(STATUS "[vehicle] Application sources from ${SF_APP_DIR}")
endif()

idf_component_register(
    SRCS
        "main.cpp"
        "../tasks/tasks.cpp"
        "../tasks/imu_task.cpp"
        "../tasks/control_task.cpp"
        "../tasks/state_task.cpp"
        "../tasks/flow_task.cpp"
        "../tasks/mag_task.cpp"
        "../tasks/baro_task.cpp"
        "../tasks/tof_task.cpp"
        "../tasks/power_task.cpp"
        "../tasks/comm_task.cpp"
        "../tasks/telemetry_task.cpp"
        "../tasks/button_task.cpp"
        "../tasks/notify_task.cpp"
        "../tasks/cli_task.cpp"
        "../tasks/log_task.cpp"
         "../tasks/api_task.cpp"
        ${SF_APP_SRCS}
    INCLUDE_DIRS "." "../tasks" ${SF_APP_INCS}
    REQUIRES
        esp_timer
        nvs_flash
        console
        sf_board
        sf_core
        sf_api
        sf_math
        sf_state
        sf_estimator
        sf_estimator_eskf
        sf_estimator_complementary
        sf_controller
        sf_controller_pid
        sf_autotune
        sf_actuator
        sf_app_hooks
        sf_comm
        sf_command
        sf_telemetry
        sf_logger
        sf_notify
        sf_failsafe
        sf_takeoff_landing
        sf_calibration
        sf_hal_bmi270
        sf_hal_bmm150
        sf_hal_bmp280
        sf_hal_vl53l3cx
        sf_hal_pmw3901
        sf_hal_motor
        sf_hal_led
        sf_hal_buzzer
        sf_hal_button
        sf_hal_power
)
