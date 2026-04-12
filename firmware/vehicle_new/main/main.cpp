/**
 * @file main.cpp
 * @brief StampFly vehicle_new — Main Entry Point
 *        StampFly vehicle_new — メインエントリポイント
 *
 * Executes the INIT phase sequentially, then starts tasks
 * and transitions to IDLE.
 *
 * INITフェーズを逐次実行し、タスクを起動後にIDLEへ遷移する。
 *
 * @design requirements.md §2 — INIT: sequential, no sub-modes         [--]
 * @design architecture.md §4 — State machine: INIT → IDLE             [--]
 * @design detailed_design.md §3 — onEnter(IDLE): start calibration    [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "config.hpp"
#include "topics.hpp"
#include "tasks.hpp"

static const char* TAG = "main";

/// Control task handle (used by ImuTask for sync notification)
/// 制御タスクハンドル（ImuTaskの同期通知に使用）
extern TaskHandle_t g_control_task_handle;

/// State task handle (used for event notification from failsafe)
/// 状態タスクハンドル（フェイルセーフからのイベント通知に使用）
TaskHandle_t g_state_task_handle = nullptr;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== StampFly vehicle_new starting ===");

    // =========================================================================
    // Phase 0: System initialization
    // Phase 0: システム初期化
    // =========================================================================

    // Initialize NVS (required for WiFi and parameter persistence)
    // NVS初期化（WiFiおよびパラメータ永続化に必要）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Phase 0: NVS initialized");

    // Initialize all Pub-Sub topics
    // 全Pub-Subトピックを初期化
    sf::topics_init();
    ESP_LOGI(TAG, "Phase 0: Topics initialized");

    // =========================================================================
    // Phase 1: Sensor and peripheral initialization
    // Phase 1: センサおよび周辺機器の初期化
    // =========================================================================

    // TODO: Initialize I2C bus
    // TODO: Initialize SPI bus
    // TODO: Initialize sensors (IMU, ToF, Flow, Mag, Baro, Power)
    // TODO: Initialize actuators (Motor, LED, Buzzer, Button)
    // TODO: Initialize communication (ESP-NOW, WiFi, UDP)
    // TODO: Initialize parameter system (load from NVS)
    ESP_LOGI(TAG, "Phase 1: Peripherals initialized (stub)");

    // =========================================================================
    // Phase 2: Wait for sensor stabilization
    // Phase 2: センサ安定待ち
    // =========================================================================

    // TODO: Wait for sensor buffers to fill
    // TODO: Compute initial biases
    ESP_LOGI(TAG, "Phase 2: Sensor stabilization (stub)");

    // =========================================================================
    // Phase 3: Start tasks
    // Phase 3: タスク起動
    //
    // @design architecture.md §6 — Task list                          [--]
    // @design detailed_design.md §8 — Task priorities and stacks      [--]
    // =========================================================================

    // Start state task first (event-driven, manages transitions)
    // 状態タスクを最初に起動（イベント駆動、遷移を管理）
    xTaskCreatePinnedToCore(StateTask, "StateTask",
        config::STACK_STATE, nullptr,
        config::PRIORITY_STATE, &g_state_task_handle, 1);

    // Start control task (waits for IMU notification)
    // 制御タスクを起動（IMU通知を待つ）
    xTaskCreatePinnedToCore(ControlTask, "ControlTask",
        config::STACK_CONTROL, nullptr,
        config::PRIORITY_CONTROL, &g_control_task_handle, 1);

    // Start IMU task (highest priority, drives the pipeline)
    // IMUタスクを起動（最高優先度、パイプラインを駆動）
    xTaskCreatePinnedToCore(ImuTask, "ImuTask",
        config::STACK_IMU, nullptr,
        config::PRIORITY_IMU, nullptr, 1);

    // Start sensor tasks
    // センサタスクを起動
    xTaskCreatePinnedToCore(FlowTask, "FlowTask",
        config::STACK_OPTFLOW, nullptr,
        config::PRIORITY_OPTFLOW, nullptr, 0);

    xTaskCreatePinnedToCore(MagTask, "MagTask",
        config::STACK_MAG, nullptr,
        config::PRIORITY_MAG, nullptr, 0);

    xTaskCreatePinnedToCore(BaroTask, "BaroTask",
        config::STACK_BARO, nullptr,
        config::PRIORITY_BARO, nullptr, 0);

    xTaskCreatePinnedToCore(TofTask, "TofTask",
        config::STACK_TOF, nullptr,
        config::PRIORITY_TOF, nullptr, 0);

    xTaskCreatePinnedToCore(PowerTask, "PowerTask",
        config::STACK_POWER, nullptr,
        config::PRIORITY_POWER, nullptr, 0);

    // Start communication and service tasks
    // 通信・サービスタスクを起動
    xTaskCreatePinnedToCore(CommTask, "CommTask",
        config::STACK_COMM, nullptr,
        config::PRIORITY_COMM, nullptr, 0);

    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask",
        config::STACK_TELEMETRY, nullptr,
        config::PRIORITY_TELEMETRY, nullptr, 0);

    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask",
        config::STACK_BUTTON, nullptr,
        config::PRIORITY_BUTTON, nullptr, 0);

    xTaskCreatePinnedToCore(NotifyTask, "NotifyTask",
        config::STACK_NOTIFY, nullptr,
        config::PRIORITY_NOTIFY, nullptr, 0);

    xTaskCreatePinnedToCore(CLITask, "CLITask",
        config::STACK_CLI, nullptr,
        config::PRIORITY_CLI, nullptr, 0);

    xTaskCreatePinnedToCore(LogTask, "LogTask",
        config::STACK_LOG, nullptr,
        config::PRIORITY_LOG, nullptr, 0);

    ESP_LOGI(TAG, "=== Phase 3: All 14 tasks started — INIT complete ===");
}
