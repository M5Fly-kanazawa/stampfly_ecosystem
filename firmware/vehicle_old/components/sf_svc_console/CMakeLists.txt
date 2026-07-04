idf_component_register(
    SRCS
        "console.cpp"
        "commands/cmd_system.cpp"
        "commands/cmd_sensor.cpp"
        "commands/cmd_motor.cpp"
        "commands/cmd_control.cpp"
        "commands/cmd_comm.cpp"
        "commands/cmd_calib.cpp"
        "commands/cmd_misc.cpp"
        "commands/cmd_flight.cpp"
        "commands/cmd_query.cpp"
        "commands/cmd_wifi_bench.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        console
        nvs_flash
        freertos
        sf_svc_state
        sf_hal_motor
        sf_hal_buzzer
        sf_hal_led
        sf_hal_bmm150
        sf_svc_led
        sf_algo_fusion
        sf_svc_logger
        sf_svc_comm
        sf_svc_udp
        sf_svc_control_arbiter
        sf_algo_control
        sf_algo_pid
        sf_svc_flight_command
        sf_svc_telemetry
)

# vehicle/main/rate_controller.hpp へのアクセス（コンソールからゲイン調整用）
# Access to vehicle/main/rate_controller.hpp (for gain adjustment from console)
# Use path relative to component location so it works for both vehicle and workshop builds
target_include_directories(${COMPONENT_LIB} PRIVATE ${COMPONENT_DIR}/../../main)
