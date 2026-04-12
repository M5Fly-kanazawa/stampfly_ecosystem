/**
 * @file imu_task.cpp
 * @brief IMU reading and state estimation task (400Hz)
 *        IMU読み取りおよび状態推定タスク（400Hz）
 *
 * Reads IMU data at 400Hz, runs the state estimator prediction step,
 * processes async sensor observations (ToF, Flow, Mag, Baro),
 * publishes the state estimate, and notifies the control task.
 *
 * 400HzでIMUデータを読み取り、状態推定器の予測ステップを実行し、
 * 非同期センサ観測（ToF、Flow、Mag、Baro）を処理し、
 * 状態推定値を発行し、制御タスクに通知する。
 *
 * @design architecture.md §5 — Main pipeline: IMU → Estimation        [--]
 * @design architecture.md §6 — ImuTask: Sensing(IMU) + Estimation     [--]
 * @design detailed_design.md §8 — ImuTask: 400Hz, priority 24        [--]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [--]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "estimator.hpp"
#include "eskf_estimator.hpp"
#include "config.hpp"

static const char* TAG = "ImuTask";

/// Task handle for control task notification
/// 制御タスク通知用のタスクハンドル
extern TaskHandle_t g_control_task_handle;

/// Estimator instance (global for callback access)
/// 推定器インスタンス（コールバックアクセス用にグローバル）
static sf::EskfEstimator estimator;

/// Process async sensor observations from queues
/// キューからの非同期センサ観測を処理する
static void processAsyncSensors()
{
    sf::TofData tof;
    while (sf::sensor_tof.read(tof)) {
        estimator.updateTof(tof);
    }

    sf::FlowData flow;
    while (sf::sensor_flow.read(flow)) {
        estimator.updateFlow(flow);
    }

    sf::MagData mag;
    while (sf::sensor_mag.read(mag)) {
        estimator.updateMag(mag);
    }

    sf::BaroData baro;
    while (sf::sensor_baro.read(baro)) {
        estimator.updateBaro(baro);
    }
}

void ImuTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ImuTask started");

    // Initialize estimator
    // 推定器を初期化
    estimator.init();

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(2);  // ~400Hz (2.5ms → 2ms tick granularity)

    while (true) {
        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());

        // =====================================================================
        // Step 1: Read IMU sensor data
        // Step 1: IMUセンサデータを読み取る
        // =====================================================================

        // TODO: Read from BMI270 hardware
        // TODO: BMI270ハードウェアから読み取る
        sf::ImuData imu = {};
        imu.timestamp = now;

        // Publish raw IMU data to topic
        // 生IMUデータをトピックに発行
        sf::sensor_imu.publish(imu);

        // =====================================================================
        // Step 2: Run estimator prediction step
        // Step 2: 推定器の予測ステップを実行
        // =====================================================================

        estimator.predict(imu, config::IMU_DT);

        // =====================================================================
        // Step 3: Process async sensor observations
        // Step 3: 非同期センサ観測を処理
        // =====================================================================

        processAsyncSensors();

        // =====================================================================
        // Step 4: Publish state estimate
        // Step 4: 状態推定値を発行
        // =====================================================================

        sf::StateEstimate state = estimator.getState();
        sf::estimate_state.publish(state);

        // =====================================================================
        // Step 5: Notify control task
        // Step 5: 制御タスクに通知
        //
        // @design architecture.md §5 — IMU-synced control pipeline    [--]
        // =====================================================================

        if (g_control_task_handle != nullptr) {
            xTaskNotifyGive(g_control_task_handle);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
