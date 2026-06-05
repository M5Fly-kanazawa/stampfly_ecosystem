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
#include "complementary_estimator.hpp"
#include "takeoff_landing.hpp"
#include "bmi270_wrapper.hpp"
#include "config.hpp"
#include "params.hpp"

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

/// Active estimator, selected by the estimator.type parameter via the factory
/// below. Held as an IEstimator* so the implementation is swappable WITHOUT
/// touching this task — the SIL bench picks ESKF or complementary via a param
/// (RESET_PLAN P2: algorithm-independence).
/// アクティブな推定器（下のファクトリが estimator.type で選ぶ）。実装を差し替え可能に
/// するため IEstimator* で持つ。SIL ベンチは param で ESKF/相補を選ぶ（P2）。
static sf::IEstimator* g_estimator = nullptr;

/// Takeoff/landing manager — derives the on-ground/airborne state from ToF altitude
/// (ToF-only, no baro). imu_task owns it (not state_task) because the vertical
/// ground→airborne handoff is an ESTIMATION concern: it tells the estimator when to
/// anchor the vertical state on the ground and when to hand off to ToF at takeoff.
/// This mirrors the proven firmware/vehicle pattern (landing handler run in imu_task).
/// 離着陸マネージャ — ToF 高度から接地/空中状態を導く（ToF のみ、baro なし）。
/// 鉛直の接地→空中ハンドオフは推定の関心事（いつ地上に錨を打ち、いつ離陸で ToF に
/// 渡すか）なので imu_task が所有する。実証済みの firmware/vehicle（landing handler を
/// imu_task で回す）と同じ構造。
static sf::TakeoffLandingMgr g_takeoff_landing;

/// Whether the vertical ground-hold/handoff applies — true when ToF is the vertical
/// sensor. On the ground the ToF sits below its min range and returns invalid, so
/// the vertical channel has NO observation and would drift without the hold. When a
/// barometer anchors altitude from the ground instead (eskf.use_tof=false), the hold
/// is neither needed nor wanted (the ToF-driven airborne detector would never release
/// it), so it is disabled. Read once from eskf.use_tof at task start.
/// 鉛直の地上ホールド/ハンドオフを適用するか — ToF が鉛直センサなら true。接地中 ToF は
/// 最小レンジ未満で無効を返し鉛直チャネルに観測が無くホールド無しではドリフトする。気圧計が
/// 地上から高度を錨付けする構成（eskf.use_tof=false）ではホールドは不要かつ有害（ToF 駆動の
/// 空中検出が解除されない）なので無効化する。タスク開始時に eskf.use_tof から1回読む。
static bool g_tof_vertical = true;

/// Estimator factory: select by estimator.type (0 = ESKF, 1 = complementary),
/// construct statically (no heap), initialize, and return via IEstimator. This is
/// the only place that names the concrete types; everything else uses IEstimator.
/// 推定器ファクトリ: estimator.type（0=ESKF, 1=相補）で選び、静的生成・初期化して
/// IEstimator で返す。具象型を知るのはここだけ。
static sf::IEstimator* createEstimator()
{
    static sf::EskfEstimator eskf;
    static sf::ComplementaryEstimator comp;
    int32_t type = 0;
    sf::params::get_int("estimator.type", type);
    if (type == 1) {
        comp.init();
        ESP_LOGI(TAG, "Estimator: complementary filter (attitude + rate)");
        return &comp;
    }
    eskf.init();
    ESP_LOGI(TAG, "Estimator: ESKF (15-state)");
    return &eskf;
}

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
    // Arm state gates the takeoff/landing manager's ground detection (disarmed →
    // on the ground). Read once per cycle from the system_mode topic (Latest, non-
    // consuming, no queue competition).
    // arm 状態が離着陸マネージャの接地判定を制御（disarmed→接地）。system_mode トピック
    // （Latest, 非消費）から毎サイクル1回読む。
    const sf::SystemMode mode = sf::system_mode.latest();
    const bool armed = sf::isArmed(static_cast<sf::FlightState>(mode.state));

    sf::TofData tof;
    while (sf::sensor_tof.read(tof)) {
        g_estimator->updateTof(tof);
        // Inject the SAME sample into the takeoff/landing manager so it does not
        // compete with the estimator for the sensor_tof queue (single consumer).
        // 同じサンプルを離着陸マネージャに注入し、sensor_tof キューを推定器と
        // 奪い合わないようにする（単一 consumer）。
        g_takeoff_landing.update(tof, armed);
    }

    sf::FlowData flow;
    while (sf::sensor_flow.read(flow)) {
        g_estimator->updateFlow(flow);
    }

    sf::MagData mag;
    while (sf::sensor_mag.read(mag)) {
        g_estimator->updateMag(mag);
    }

    sf::BaroData baro;
    while (sf::sensor_baro.read(baro)) {
        g_estimator->updateBaro(baro);
    }
}

/// Vertical ground→airborne handoff — anchor the vertical estimate on the ground
/// and hand off to ToF at takeoff (ToF-only vertical; no baro).
///
/// On the ground the only vertical observation (ToF) is invalid below its minimum
/// range, so without an anchor the predict-only vertical state drifts (a residual
/// vel_z integrates into a pos_z ramp). By takeoff the drift exceeds the ToF
/// innovation gate, so the first airborne ToF reading is rejected and the estimate
/// never recovers — ALT_HOLD then chases a diverged altitude. Holding pos/vel at
/// zero while grounded kills the drift; resetting at the ground→airborne edge gives
/// ToF a clean lock (innovation ≈ true altitude, well inside the gate).
///
/// This mirrors the proven firmware/vehicle handoff (hold while grounded, reset at
/// takeoff). Must run AFTER predict + ToF update so the hold overrides any drift.
///
/// 鉛直の接地→空中ハンドオフ — 接地中は鉛直推定を錨で固定し、離陸で ToF に渡す
/// （鉛直は ToF のみ、baro なし）。接地中は唯一の鉛直観測 ToF が最小レンジ未満で無効
/// ゆえ、錨が無いと予測のみの鉛直状態がドリフトし（残差 vel_z が pos_z ランプに積分）、
/// 離陸時には ToF innovation ゲートを超えて最初の空中 ToF が棄却され回復不能になる
/// （ALT_HOLD が発散した高度を追う）。接地中 pos/vel をゼロ保持でドリフトを殺し、
/// 接地→空中エッジで reset すれば ToF がクリーンにロックする。実証済みの
/// firmware/vehicle と同じ。predict + ToF 更新の後に実行（hold がドリフトを上書き）。
///
/// @design development_roadmap.md §3 Layer 3 — ToF-only vertical handoff  [OK]
static void applyVerticalGroundHandoff()
{
    const bool on_ground = g_takeoff_landing.isOnGround();
    static bool was_on_ground = true;   // boot state: on the ground / 起動時は接地

    // Ground→airborne edge: reset pos/vel (and covariance) for a clean ToF lock.
    // 接地→空中エッジ: クリーンな ToF ロックのため pos/vel（と共分散）をリセット。
    if (was_on_ground && !on_ground) {
        g_estimator->resetPositionVelocity();
        ESP_LOGI(TAG, "Vertical handoff: takeoff — position tracking enabled");
    }

    // While on the ground: clamp pos/vel to zero each cycle to kill predict-only drift.
    // 接地中: 毎サイクル pos/vel をゼロ固定して予測のみのドリフトを殺す。
    if (on_ground) {
        g_estimator->holdPositionVelocity();
    }

    was_on_ground = on_ground;
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

    // Create + initialize the estimator selected by estimator.type.
    // estimator.type で選ばれた推定器を生成・初期化。
    g_estimator = createEstimator();

    // Initialize the takeoff/landing manager (ToF-altitude ground/airborne detection).
    // 離着陸マネージャを初期化（ToF 高度で接地/空中を判定）。
    g_takeoff_landing.init();

    // The vertical ground-hold applies only when ToF is the vertical sensor (a
    // barometer would anchor altitude from the ground, making the hold unnecessary
    // and its ToF-driven release impossible). Read eskf.use_tof once.
    // 地上ホールドは ToF が鉛直センサの時のみ適用（気圧計があれば地上から錨付けでき
    // ホールドは不要かつ ToF 駆動の解除が不能になる）。eskf.use_tof を1回読む。
    bool use_tof = true;
    sf::params::get_bool("eskf.use_tof", use_tof);
    g_tof_vertical = use_tof;

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

        g_estimator->predict(imu, config::IMU_DT);

        // =====================================================================
        // Step 3: Process async sensor observations
        // Step 3: 非同期センサ観測を処理
        // =====================================================================

        processAsyncSensors();

        // =====================================================================
        // Step 3.5: Vertical ground→airborne handoff (ToF-only)
        // Step 3.5: 鉛直の接地→空中ハンドオフ（ToF のみ）
        //
        // Runs after predict + ToF update so the on-ground hold overrides any
        // predict-only vertical drift before the state is published. Skipped when a
        // barometer (not ToF) anchors altitude from the ground (g_tof_vertical).
        // predict + ToF 更新の後に実行し、接地ホールドが予測のみの鉛直ドリフトを
        // 上書きしてから状態を発行する。気圧計が地上から錨付けする構成では飛ばす。
        // =====================================================================

        if (g_tof_vertical) {
            applyVerticalGroundHandoff();
        }

        // =====================================================================
        // Step 4: Publish state estimate
        // Step 4: 状態推定値を発行
        // =====================================================================

        sf::StateEstimate state = g_estimator->getState();
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
