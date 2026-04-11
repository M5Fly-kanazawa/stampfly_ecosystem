/**
 * @file main.cpp
 * @brief StampFly vehicle_new — Main Entry Point
 *        StampFly vehicle_new — メインエントリポイント
 *
 * Executes the INIT phase sequentially, then transitions to IDLE.
 * All initialization runs here in app_main() before tasks are started.
 *
 * INITフェーズを逐次実行し、完了後にIDLEへ遷移する。
 * タスク起動前の全初期化はここ（app_main）で行う。
 *
 * @design requirements.md §2 — INIT: sequential, no sub-modes         [--]
 * @design architecture.md §4 — State machine: INIT → IDLE             [--]
 * @design detailed_design.md §3 — onEnter(IDLE): start calibration    [--]
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "config.hpp"

static const char* TAG = "main";

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
    ESP_LOGI(TAG, "NVS initialized");

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

    // =========================================================================
    // Phase 2: Estimator and controller initialization
    // Phase 2: 推定器および制御器の初期化
    // =========================================================================

    // TODO: Initialize state estimator (ESKF)
    // TODO: Initialize controller (PID)
    // TODO: Wait for sensor stabilization

    // =========================================================================
    // Phase 3: Start tasks and transition to IDLE
    // Phase 3: タスク起動およびIDLEへ遷移
    // =========================================================================

    // TODO: Start all FreeRTOS tasks
    // TODO: Transition state to IDLE_GROUND

    ESP_LOGI(TAG, "=== Initialization complete ===");
}
