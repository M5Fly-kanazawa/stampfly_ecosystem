idf_component_register(
    SRCS
        "flight_command.cpp"
        "command_queue.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        freertos
        esp_timer
        sf_svc_state
        sf_svc_control_arbiter
        sf_svc_comm
        sf_algo_fusion
        sf_algo_pid
)

# Access to vehicle/main/ directory for globals.hpp, landing_handler.hpp, etc.
# vehicle/main/ディレクトリへのアクセス（globals.hpp, landing_handler.hpp等）
# Use path relative to component location so it works for both vehicle and workshop builds
target_include_directories(${COMPONENT_LIB} PRIVATE ${COMPONENT_DIR}/../../main)
