# Workshop main component
# Builds workshop-specific files on top of the SAME vehicle sensor/peripheral
# tasks used by firmware/vehicle (R12, workshop_migration.md M5). Only
# ControlTask is replaced (workshop_control_task.cpp calls the learner's
# setup()/loop_400Hz() instead of running sf::PidController).
# ワークショップ固有ファイルを、firmware/vehicle と同一のセンサ/周辺タスクの上に
# ビルドする（R12、workshop_migration.md M5）。ControlTask だけを差し替え
# （workshop_control_task.cpp が sf::PidController の代わりに学習者の
# setup()/loop_400Hz() を呼ぶ）。

set(VEHICLE_MAIN_DIR  "${CMAKE_CURRENT_SOURCE_DIR}/../../vehicle/main")
set(VEHICLE_TASKS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../vehicle/tasks")

# Auto-bootstrap user_code.cpp from the Lesson 0 seed if absent.
# This file is gitignored; `sf lesson switch` overwrites it on demand,
# and existing edits are preserved (NOT EXISTS guard).
# user_code.cpp が無ければ Lesson 0 テンプレートから生成する（gitignore 対象）
# `sf lesson switch` での上書きは従来通り、編集済みファイルは保護される
set(USER_CODE "${CMAKE_CURRENT_SOURCE_DIR}/user_code.cpp")
set(USER_CODE_SEED "${CMAKE_CURRENT_SOURCE_DIR}/../lessons/lesson_00_setup/student.cpp")
if(NOT EXISTS "${USER_CODE}")
    if(NOT EXISTS "${USER_CODE_SEED}")
        message(FATAL_ERROR
            "user_code.cpp bootstrap failed: seed not found at ${USER_CODE_SEED}")
    endif()
    configure_file("${USER_CODE_SEED}" "${USER_CODE}" COPYONLY)
    message(STATUS "[workshop] Bootstrapped user_code.cpp from Lesson 0 template")
endif()

idf_component_register(
    SRCS
        "workshop_main.cpp"
        "workshop_tasks.cpp"
        "workshop_control_task.cpp"
        "workshop_api.cpp"
        "user_code.cpp"
        # Reuse vehicle's sensor/service tasks verbatim (compiled from vehicle
        # source). control_task.cpp and tasks.cpp are EXCLUDED — this component
        # provides its own workshop_control_task.cpp / workshop_tasks.cpp instead.
        # vehicle のセンサ/サービスタスクをそのまま再利用（vehicle のソースから
        # コンパイル）。control_task.cpp と tasks.cpp は除外 — 本コンポーネントが
        # 独自の workshop_control_task.cpp / workshop_tasks.cpp を提供する。
        "${VEHICLE_TASKS_DIR}/imu_task.cpp"
        "${VEHICLE_TASKS_DIR}/state_task.cpp"
        "${VEHICLE_TASKS_DIR}/flow_task.cpp"
        "${VEHICLE_TASKS_DIR}/mag_task.cpp"
        "${VEHICLE_TASKS_DIR}/baro_task.cpp"
        "${VEHICLE_TASKS_DIR}/tof_task.cpp"
        "${VEHICLE_TASKS_DIR}/power_task.cpp"
        "${VEHICLE_TASKS_DIR}/comm_task.cpp"
        "${VEHICLE_TASKS_DIR}/telemetry_task.cpp"
        "${VEHICLE_TASKS_DIR}/button_task.cpp"
        "${VEHICLE_TASKS_DIR}/notify_task.cpp"
        "${VEHICLE_TASKS_DIR}/cli_task.cpp"
        "${VEHICLE_TASKS_DIR}/log_task.cpp"
        "${VEHICLE_TASKS_DIR}/api_task.cpp"
    INCLUDE_DIRS "." "${VEHICLE_MAIN_DIR}" "${VEHICLE_TASKS_DIR}"
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
