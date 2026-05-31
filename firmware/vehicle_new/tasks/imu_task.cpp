/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

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
 * @design architecture.md §5 — Main pipeline: IMU → Estimation        [OK]
 * @design architecture.md §6 — ImuTask: Sensing(IMU) + Estimation     [OK]
 * @design detailed_design.md §8 — ImuTask: 400Hz, priority 24        [OK]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "estimator.hpp"
#include "eskf_estimator.hpp"
#include "bmi270_wrapper.hpp"
#include "config.hpp"

static const char* TAG = "ImuTask";

/// Conversion factor: 1 g to m/s² (standard gravity)
/// 換算係数: 1 g → m/s²（標準重力）
static constexpr float G_TO_MPS2 = 9.80665f;

/// Temperature is read every N IMU cycles (400Hz / 100 = 4Hz)
/// 温度は N サイクルごとに読み取る（400Hz / 100 = 4Hz）
static constexpr uint32_t TEMPERATURE_READ_INTERVAL = 100;

/// Read-failure log throttle: warn at most once every N cycles
/// 読み取り失敗ログの抑制: N サイクルに 1 回まで警告
static constexpr uint32_t READ_FAIL_LOG_INTERVAL = 400;

/// Task handle for control task notification
/// 制御タスク通知用のタスクハンドル
extern TaskHandle_t g_control_task_handle;

/// Estimator instance
/// 推定器インスタンス
static sf::EskfEstimator estimator;

/// BMI270 IMU driver instance (file-scope, not exposed as global)
/// BMI270 IMU ドライバインスタンス（ファイルスコープ、グローバル非公開）
///
/// @design architecture.md §6 — Component encapsulation, no globals    [OK]
/// @design coding_and_education.md §3 — No global singletons            [OK]
static stampfly::BMI270Wrapper imu_wrapper;

/// Last cached temperature [°C], updated every TEMPERATURE_READ_INTERVAL cycles
/// 最後にキャッシュした温度 [°C]、TEMPERATURE_READ_INTERVAL サイクルごとに更新
static float cached_temperature_c = 0.0f;

/// IMU SPI driver debug breadcrumb (referenced by sf_hal_bmi270/bmi270_spi.c).
/// The BMI270 C driver writes checkpoint codes (40..44) for crash-dump analysis.
/// Define here with C linkage so the HAL's extern declaration resolves.
///
/// IMU SPI ドライバのデバッグ用ブレッドクラム（sf_hal_bmi270/bmi270_spi.c が参照）。
/// BMI270 C ドライバがクラッシュダンプ解析用にチェックポイント値（40..44）を書き込む。
/// HAL の extern 宣言を解決するため C リンケージで定義する。
extern "C" volatile uint8_t g_imu_checkpoint = 0;

/// Apply BMI270→body-frame coordinate transform and unit conversion.
/// BMI270→機体座標系変換と単位変換を適用する。
///
/// StampFly body frame is NED (X forward, Y right, Z down).
/// BMI270 sensor axes are mapped as follows:
///
/// StampFly機体座標系は NED（X前方、Y右、Z下方）。
/// BMI270 のセンサ軸マッピングは以下の通り:
///
///   ┌─────────────────────────────────────────┐
///   │   BMI270 axis  →  body axis             │
///   │   ───────────────────────────           │
///   │   X (sensor)   →  Y body  (right)       │
///   │   Y (sensor)   →  X body  (forward)     │
///   │   Z (sensor)   →  −Z body (down, sign-flip)
///   └─────────────────────────────────────────┘
///
/// Accel is converted from g to m/s² (× G_TO_MPS2).
/// Gyro stays in rad/s (already in SI units from the wrapper).
///
/// 加速度は g から m/s² へ変換（× G_TO_MPS2）。
/// 角速度は rad/s のまま（ラッパが既に SI 単位で返す）。
///
/// @design detailed_design.md §3.1 — IMU body-frame convention (NED)   [OK]
/// @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
static void applyImuTransform(const stampfly::AccelData& accel_sensor,
                               const stampfly::GyroData& gyro_sensor,
                               sf::ImuData& imu_out)
{
    // Accel: sensor g → body m/s² with axis remap
    // 加速度: センサ g → 機体 m/s²、軸を再マッピング
    imu_out.accel[0] =  accel_sensor.y * G_TO_MPS2;  // body X (forward)  / 機体X（前方）
    imu_out.accel[1] =  accel_sensor.x * G_TO_MPS2;  // body Y (right)    / 機体Y（右）
    imu_out.accel[2] = -accel_sensor.z * G_TO_MPS2;  // body Z (down)     / 機体Z（下方、符号反転）

    // Gyro: sensor rad/s → body rad/s with same axis remap
    // 角速度: センサ rad/s → 機体 rad/s、同じ軸マッピング
    imu_out.gyro[0] =  gyro_sensor.y;   // body X (roll rate)   / 機体X（ロールレート）
    imu_out.gyro[1] =  gyro_sensor.x;   // body Y (pitch rate)  / 機体Y（ピッチレート）
    imu_out.gyro[2] = -gyro_sensor.z;   // body Z (yaw rate)    / 機体Z（ヨーレート、符号反転）
}

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

/// esp_timer callback: paces the IMU loop at 400Hz by notifying the IMU task.
/// Runs in the esp_timer task context (not an ISR), so xTaskNotifyGive is used.
/// esp_timer コールバック: IMU タスクに通知して IMU ループを 400Hz で刻む。
/// esp_timer タスク文脈（ISR ではない）で動くので xTaskNotifyGive を使う。
static void imuTimerCallback(void* arg)
{
    TaskHandle_t imu_task = static_cast<TaskHandle_t>(arg);
    if (imu_task != nullptr) {
        xTaskNotifyGive(imu_task);
    }
}

void ImuTask(void* pvParameters)
{
    ESP_LOGI(TAG, "ImuTask started");

    // -------------------------------------------------------------------------
    // Setup: initialize BMI270 IMU sensor (must succeed before entering loop)
    // セットアップ: BMI270 IMU を初期化（ループ開始前に成功必須）
    //
    // @design detailed_design.md §8 — IMU init in task setup phase     [OK]
    // -------------------------------------------------------------------------
    esp_err_t imu_init_result = imu_wrapper.init(
        stampfly::BMI270Wrapper::Config::defaultStampFly());
    if (imu_init_result != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 init failed: %s — task aborting",
                 esp_err_to_name(imu_init_result));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "BMI270 init OK (400Hz read loop starting)");

    // Initialize estimator
    // 推定器を初期化
    estimator.init();

    // Drive the loop at a true 400Hz with an esp_timer periodic (2500us). The
    // FreeRTOS tick cannot express 2.5ms, so a hardware timer paces the loop;
    // this makes the wall period match config::IMU_DT (eliminating the old
    // 2ms/500Hz vs 2.5ms dt mismatch that mis-scaled the ESKF/PID timing).
    // ループを esp_timer periodic（2500us）で真の 400Hz に駆動する。FreeRTOS tick
    // では 2.5ms を表現できないためハードウェアタイマでループを刻む。これで実周期が
    // config::IMU_DT と一致する（旧 2ms/500Hz vs 2.5ms の不一致＝ESKF/PID のタイミング
    // 誤差を解消）。
    TaskHandle_t self = xTaskGetCurrentTaskHandle();
    const esp_timer_create_args_t imu_timer_args = {
        .callback = &imuTimerCallback,
        .arg = self,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "imu_400hz",
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t imu_timer = nullptr;
    if (esp_timer_create(&imu_timer_args, &imu_timer) != ESP_OK ||
        esp_timer_start_periodic(imu_timer, config::IMU_PERIOD_US) != ESP_OK) {
        ESP_LOGE(TAG, "IMU 400Hz timer setup failed — task aborting");
        vTaskDelete(NULL);
        return;
    }

    uint32_t cycle_count = 0;          // For temperature/log throttling
                                       // 温度・ログ抑制カウンタ
    uint32_t last_fail_log_cycle = 0;  // Last cycle a read-fail warning was logged
                                       // 直近で読み取り失敗を警告したサイクル

    while (true) {
        // Block until the 400Hz esp_timer ticks (it notifies this task).
        // 400Hz の esp_timer がティック（このタスクに通知）するまでブロック。
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        ++cycle_count;

        // =====================================================================
        // Step 1: Read IMU sensor data
        // Step 1: IMUセンサデータを読み取る
        //
        // @design architecture.md §5 — Sensing(IMU) stage              [OK]
        // =====================================================================
        sf::ImuData imu = {};
        imu.timestamp = now;

        // Read accel + gyro from BMI270 (single SPI burst transaction)
        // BMI270 から加速度+角速度を読み取り（単一 SPI バースト）
        stampfly::AccelData accel_sensor = {};
        stampfly::GyroData  gyro_sensor  = {};
        esp_err_t read_result = imu_wrapper.readSensorData(accel_sensor, gyro_sensor);
        if (read_result != ESP_OK) {
            // Rate-limited warning to avoid log flooding at 400Hz
            // 400Hz でのログ氾濫を避けるため、ログを抑制
            if (cycle_count - last_fail_log_cycle >= READ_FAIL_LOG_INTERVAL) {
                ESP_LOGW(TAG, "BMI270 read failed: %s",
                         esp_err_to_name(read_result));
                last_fail_log_cycle = cycle_count;
            }
            continue;  // wait for the next 400Hz tick / 次の 400Hz ティックを待つ
        }

        // Apply body-frame coordinate transform and unit conversion
        // 機体座標系変換と単位変換を適用
        applyImuTransform(accel_sensor, gyro_sensor, imu);

        // Refresh cached temperature periodically (not every cycle)
        // 温度を定期的に更新（毎サイクルではない）
        if ((cycle_count % TEMPERATURE_READ_INTERVAL) == 0) {
            float temperature_c = 0.0f;
            if (imu_wrapper.readTemperature(temperature_c) == ESP_OK) {
                cached_temperature_c = temperature_c;
            }
        }
        imu.temperature = cached_temperature_c;

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
        // @design architecture.md §5 — IMU-synced control pipeline    [OK]
        // =====================================================================

        if (g_control_task_handle != nullptr) {
            xTaskNotifyGive(g_control_task_handle);
        }
        // The esp_timer paces the loop; the ulTaskNotifyTake at the top blocks
        // until the next 400Hz tick, so no explicit delay is needed here.
        // ループは esp_timer が刻む。先頭の ulTaskNotifyTake が次の 400Hz ティック
        // までブロックするので、ここで明示的な待ちは不要。
    }
}
