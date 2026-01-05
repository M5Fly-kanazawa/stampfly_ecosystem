idf_component_register(
    SRCS "cli.cpp"
    INCLUDE_DIRS "include"
    REQUIRES freertos esp_system sf_svc_state esp_vfs_console vfs sf_hal_bmm150 sf_svc_logger sf_svc_comm sf_svc_led sf_hal_motor sf_hal_buzzer sf_algo_fusion sf_algo_pid nvs_flash
)

# main/rate_controller.hpp へのアクセス（CLIからゲイン調整用）
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_SOURCE_DIR}/main)
