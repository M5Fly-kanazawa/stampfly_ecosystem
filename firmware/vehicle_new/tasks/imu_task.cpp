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
 * @publisher sensor_imu, estimate_state, system_status
 * @subscriber sensor_tof, sensor_flow, sensor_mag, sensor_baro, estimator_command, system_mode
 * @design architecture.md §5 — Main pipeline: IMU → Estimation        [OK]
 * @design architecture.md §6 — ImuTask: Sensing(IMU) + Estimation     [OK]
 * @design detailed_design.md §8 — ImuTask: 400Hz, priority 24        [OK]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
 * @design architecture.md §3 — R13 @publisher/@subscriber annotation  [OK]
 */

#include <atomic>     // s_imu_spi_init_done (FlowTask SPI-init serialization)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "tasks.hpp"
#include "estimator.hpp"
#include "eskf_estimator.hpp"
#include "complementary_estimator.hpp"
#include "takeoff_landing.hpp"
#include "calibration.hpp"
#include "failsafe.hpp"     // ImuAnomalyDetector (400Hz impact/gyro checks)
#include "bmi270_wrapper.hpp"
#include "config.hpp"
#include "params.hpp"

static const char* TAG = "ImuTask";

/// Conversion factor: 1 g to m/s² (standard gravity, SSOT: sf::math::kGravity)
/// 換算係数: 1 g → m/s²（標準重力, SSOT: sf::math::kGravity）
static constexpr float G_TO_MPS2 = sf::math::kGravity;

/// Temperature is read every N IMU cycles (400Hz / 100 = 4Hz)
/// 温度は N サイクルごとに読み取る（400Hz / 100 = 4Hz）
static constexpr uint32_t TEMPERATURE_READ_INTERVAL = 100;

/// Read-failure log throttle: warn at most once every N cycles
/// 読み取り失敗ログの抑制: N サイクルに 1 回まで警告
static constexpr uint32_t READ_FAIL_LOG_INTERVAL = 400;

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

/// IMU-rate anomaly detector (impact / gyro anomaly, requirements §9). Owned by
/// this task and fed per-sample at 400Hz — the previous 10Hz latest() peek in
/// PowerTask missed millisecond-scale crash spikes almost certainly.
/// IMU レート異常検出器（衝撃/ジャイロ異常、要件§9）。本タスクが所有し 400Hz で
/// サンプル毎に与える — 従来の PowerTask での 10Hz latest() 覗き見はミリ秒オーダーの
/// 墜落スパイクをほぼ確実に取りこぼしていた。
static sf::ImuAnomalyDetector g_imu_anomaly;

/// CalibrationMgr — boot gyro/accel bias calibration, owned by ImuTask because it has
/// the IMU samples. Measures the at-rest bias on the ground and seeds the estimator
/// with it before flight (an uncalibrated accel bias destabilizes the ESKF attitude).
/// CalibrationMgr — 起動時のジャイロ/加速度バイアス校正。IMU サンプルを持つ ImuTask が
/// 所有。地上静止のバイアスを測り、飛行前に推定器へ種付けする（未校正の加速度バイアスは
/// ESKF 姿勢を不安定化させる）。
static sf::CalibrationMgr g_calib;

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

/// Set once the BMI270 init on the shared SPI bus has finished (success OR
/// terminal failure). FlowTask polls this via sf::tasks::imu_spi_init_done()
/// before touching the bus — see tasks.hpp for the CS/MISO race rationale.
/// Plain atomic (no board dependency) because this source is shared with the
/// partial SIL test programs that do not link sf_board.
/// 共有 SPI バス上の BMI270 初期化が終わったら set（成功・確定失敗とも）。FlowTask は
/// バスに触れる前に sf::tasks::imu_spi_init_done() でこれをポーリングする — CS/MISO
/// 競合の理由は tasks.hpp 参照。sf_board をリンクしない部分 SIL 試験プログラムと
/// ソース共有のため、board 依存のない素の atomic で表現する。
static std::atomic<bool> s_imu_spi_init_done{false};

namespace sf {
namespace tasks {
bool imu_spi_init_done() { return s_imu_spi_init_done.load(std::memory_order_acquire); }
}  // namespace tasks
}  // namespace sf

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

/// Convert the BMI270 driver's body-frame reading into ImuData (unit conversion only).
/// The axis remap (chip → body FRD) is done in the BMI270 driver (bmi270_wrapper) per
/// project policy, so AccelData/GyroData are ALREADY body forward/right/down. Here we
/// only convert accel from g to m/s² (gyro is already rad/s). No axis swap, no sign.
/// BMI270 ドライバの機体座標読みを ImuData へ（単位変換のみ）。軸 remap（チップ→機体FRD）は
/// 方針によりドライバ(bmi270_wrapper)で実施済みゆえ、AccelData/GyroData は既に機体
/// 前方/右/下。ここでは加速度を g→m/s² 変換するのみ（ジャイロは既に rad/s）。軸入替・符号なし。
///
/// @design detailed_design.md §3.1 — IMU body-frame convention (NED)   [OK]
/// @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
static void applyImuTransform(const stampfly::AccelData& accel_body,
                               const stampfly::GyroData& gyro_body,
                               sf::ImuData& imu_out)
{
    // Accel: body g → body m/s² (driver already supplied body FRD axes).
    // 加速度: 機体 g → 機体 m/s²（ドライバが機体 FRD 軸で供給済み）。
    imu_out.accel[0] = accel_body.x * G_TO_MPS2;  // body X (forward)
    imu_out.accel[1] = accel_body.y * G_TO_MPS2;  // body Y (right)
    imu_out.accel[2] = accel_body.z * G_TO_MPS2;  // body Z (down)

    // Gyro: already body FRD rad/s — pass through.
    // 角速度: 既に機体 FRD rad/s — そのまま。
    imu_out.gyro[0] = gyro_body.x;   // body X (roll rate)
    imu_out.gyro[1] = gyro_body.y;   // body Y (pitch rate)
    imu_out.gyro[2] = gyro_body.z;   // body Z (yaw rate)
}

// Mag boot policy — one-shot at boot-calibration completion (and after every
// ground recalibration): the craft is still and the attitude has converged.
// CALIBRATED mag → capture the NED reference (ref = R(q̂)·m_body, defining
// "yaw 0 = boot heading") so the ESKF mag update is meaningful. UNCALIBRATED
// mag → force the mag OUT of the fusion regardless of eskf.use_mag: hard-iron
// offsets are commonly as large as the field itself, and fusing them would
// inject exactly the slow yaw-reference contamination the gyro-bias clamp
// guards against. Run `magcal`, then reboot (or recalibrate) to enable.
// 磁気の起動ポリシー — 起動校正完了時（と接地再校正のたび）の one-shot: 機体は静止し
// 姿勢は収束済み。「校正済み」磁気 → NED 参照を捕捉（ref = R(q̂)·m_body、
// 「ヨー0 = 起動方位」を定義）し ESKF 磁気更新を意味のあるものにする。「未校正」
// 磁気 → eskf.use_mag に関わらず融合から強制除外: ハードアイアンオフセットは磁場
// 自体と同程度の大きさになることが多く、融合すればジャイロバイアスクランプが守って
// いる「遅いヨー参照汚染」をまさに注入してしまう。`magcal` 実行後、再起動（または
// 再校正）で有効化される。
static bool g_mag_ref_pending = false;

static void applyMagBootPolicy(const sf::MagData& mag)
{
    if (!g_mag_ref_pending) {
        return;
    }
    g_mag_ref_pending = false;

    if (!mag.calibrated) {
        // Drop the calibration gate (group 2). It is independent of eskf.use_mag
        // and survives reloadParams, so live tuning/recalibration cannot silently
        // re-admit an uncalibrated mag (L-5).
        // 校正ゲート(group 2)を下ろす。eskf.use_mag と独立で reloadParams を生き延びる
        // ため、ライブチューニング/再校正で未校正磁気が黙って復帰しない (L-5)。
        g_estimator->setSensorEnabled(2 /*MAG calib gate*/, false);
        ESP_LOGW(TAG, "Mag uncalibrated — yaw aiding disabled (run 'magcal')");
        return;
    }

    // Calibrated: raise the calibration gate (it may have been dropped on a prior
    // boot/recal when the mag was still uncalibrated). Mag still fuses only when
    // eskf.use_mag is also set. / 校正済み: 校正ゲートを上げる（以前の起動/再校正で
    // 未校正だったとき下りている可能性）。融合は eskf.use_mag も成立時のみ。
    g_estimator->setSensorEnabled(2 /*MAG calib gate*/, true);

    // ref_ned = R(q) · m_body, with q the converged boot attitude.
    // ref_ned = R(q)·m_body。q は収束済みの起動姿勢。
    const sf::StateEstimate st = g_estimator->getState();
    sf::math::Quat q(st.attitude[0], st.attitude[1], st.attitude[2], st.attitude[3]);
    sf::math::Vec3 m_body(mag.mag[0], mag.mag[1], mag.mag[2]);
    sf::math::Vec3 ref = q.rotate(m_body);
    const float ned[3] = {ref.x, ref.y, ref.z};
    g_estimator->setMagReference(ned);
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
    // True while the autonomous landing descent is commanded — enables the stalled-descent
    // touchdown branch (a stall near the ground means "landed" only when we are descending).
    // 自動着陸降下の指令中に true — 降下停滞による接地判定を有効化（接地近傍の停滞が「着陸」を
    // 意味するのは降下中のみ）。
    const bool in_landing_descent =
        (static_cast<sf::FlightState>(mode.state) == sf::FlightState::LANDING);

    // Mirror the latest consumed async-sensor values into sensor_snapshot (Latest) so
    // monitors (CLI `sensor`, telemetry) can peek without stealing from these SPSC
    // queues. Persisted across cycles so a slow sensor keeps its last value visible.
    // 消費した非同期センサの最新値を sensor_snapshot(Latest) にミラーし、監視側（CLI/テレ）が
    // SPSC キューから奪わず覗けるようにする。低レートセンサの最終値を保つよう周期跨ぎで保持。
    static sf::SensorSnapshot snap{};
    bool snap_dirty = false;

    sf::TofData tof;
    while (sf::sensor_tof.read(tof)) {
        g_estimator->updateTof(tof);
        // Inject the SAME sample into the takeoff/landing manager so it does not compete
        // with the estimator for the sensor_tof queue (single consumer). Also inject the
        // current vertical velocity (NED down) so the landing detector stays a pure
        // function of its inputs (no topic read inside the manager).
        // 同じサンプルを離着陸マネージャに注入し、sensor_tof キューを推定器と奪い合わない
        // ようにする（単一 consumer）。鉛直速度（NED down）も注入し、着陸検出器を入力の純粋な
        // 関数に保つ（マネージャ内でトピックを読まない）。
        g_takeoff_landing.update(tof, armed, g_estimator->getState().velocity[2],
                                 in_landing_descent);
        snap.tof_distance  = tof.distance;
        snap.tof_status    = tof.status;
        snap.tof_valid     = tof.valid;
        snap.tof_timestamp = tof.timestamp;   // per-sensor stamp for the Data Stream / Data Stream 用センサ別時刻
        snap_dirty = true;
    }

    sf::FlowData flow;
    while (sf::sensor_flow.read(flow)) {
        g_estimator->updateFlow(flow);
        // Mirror EVERY flow sample for the Data Stream: dx/dy are incremental
        // (displacement since the previous read), so a latest()-style 50Hz peek
        // would silently drop half the displacement — offline replay needs all.
        // Data Stream 用に「全」フローサンプルをミラー: dx/dy は差分量（前回読み
        // 出しからの変位）ゆえ、50Hz の latest() 覗き見では変位の半分が黙って
        // 失われる — オフライン再生には全量が要る。
        sf::log_flow.publish(flow);
        snap.flow_dx = flow.dx; snap.flow_dy = flow.dy; snap.flow_squal = flow.squal;
        snap.flow_timestamp = flow.timestamp;
        snap_dirty = true;
    }

    sf::MagData mag;
    while (sf::sensor_mag.read(mag)) {
        applyMagBootPolicy(mag);
        g_estimator->updateMag(mag);
        snap.mag[0] = mag.mag[0]; snap.mag[1] = mag.mag[1]; snap.mag[2] = mag.mag[2];
        snap.mag_timestamp = mag.timestamp;
        snap_dirty = true;
    }

    sf::BaroData baro;
    while (sf::sensor_baro.read(baro)) {
        g_estimator->updateBaro(baro);
        snap.baro_pressure = baro.pressure; snap.baro_altitude = baro.altitude;
        snap.baro_timestamp = baro.timestamp;
        snap_dirty = true;
    }

    if (snap_dirty) {
        snap.timestamp = static_cast<uint32_t>(esp_timer_get_time());
        sf::sensor_snapshot.publish(snap);
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
    // Vertical ground→airborne handoff — an ESTIMATION-internal concern owned by ImuTask
    // because it is tightly coupled to the ToF sensor and must fire the instant the ToF
    // detects airborne (a one-cycle-precise event). It is deliberately NOT moved to the
    // onEnter(FLYING) transition callback: that fires ~20 ms later (StateTask's RC-poll
    // tick after system_status.airborne), and the α-β accel-compensation tracker is
    // sensitive enough that a 20 ms-late pos/vel reset measurably degrades POS_HOLD
    // attitude (pos_flight/pos_yaw att_rmse). So the state machine owns the CONTROLLER
    // resets and the full-state ESKF reset (architecture §4), while this ToF-synced
    // vertical handoff (a one-shot reset at the airborne edge + the on-ground hold) stays
    // here in estimation where the ToF event is observed directly.
    // 鉛直の接地→空中ハンドオフ — ToF センサと密結合で、ToF が空中を検知した瞬間（1サイクル
    // 精度のイベント）に発火する必要がある estimation 内部の関心事ゆえ ImuTask が所有する。
    // onEnter(FLYING) 遷移コールバックに移さない: それは ~20ms 遅れて発火し（system_status.
    // airborne 後の StateTask の RC ポーリングティック）、α-β 運動加速度補償トラッカは 20ms
    // 遅れの pos/vel reset で POS_HOLD 姿勢が測定可能なほど劣化するほど敏感（pos_flight/
    // pos_yaw の att_rmse）。よって状態機械は CONTROLLER リセットと ESKF full-state reset を
    // 所有し（architecture §4）、この ToF 同期の鉛直ハンドオフ（空中エッジの one-shot reset ＋
    // 接地ホールド）は ToF イベントを直接観測する estimation 側のここに残す。
    //
    // @design development_roadmap.md §3 Layer 3 — ToF-only vertical handoff        [OK]
    // @design architecture.md §4 — controller/full-state reset is owned by the state
    //         machine; the timing-critical ToF-synced vertical handoff stays here    [OK]
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

// Boot-calibration loop state. The calibration runs INSIDE the 400Hz loop (not as a
// blocking setup phase) so it never delays the estimator/loop start — a blocking phase
// desyncs the estimator from the rest of the system and the scenario timeline. While
// on the ground the estimate is anchored, so accumulating in parallel is harmless.
// 起動校正のループ状態。校正は 400Hz ループ「内」で実行し（ブロッキングな setup フェーズに
// しない）、推定器/ループ開始を遅らせない — ブロッキングは推定器を他系やシナリオ時系列と
// desync させる。接地中は推定が錨付けされるので並行蓄積は無害。
static bool     g_calib_active        = false;  // settling/collecting now / 整定・収集中
static uint32_t g_calib_settle_cycles = 0;      // remaining settle cycles to discard / 破棄する整定残り
static bool     g_calibrated          = false;  // latched: calibration no longer pending / 校正が保留中でない

// Applied calibration biases, retained so a full estimator reset (estimator_command
// Reset — e.g. ARM or crash-return in Phase 2) can re-seed them: reset() zeroes the bias
// states (estimator.hpp), so the calibration must be re-applied AFTER the reset.
// 適用した校正バイアス。full reset（estimator_command Reset — 例: Phase 2 の ARM や墜落復帰）後に
// 再注入できるよう保持する: reset() はバイアス状態をゼロ化する（estimator.hpp）ので、reset 後に
// 校正を再適用する必要がある。
static bool     g_calib_applied         = false;
static float    g_applied_gyro_bias[3]  = {0, 0, 0};
static float    g_applied_accel_bias[3] = {0, 0, 0};

/// Publish the boot/system readiness status so other tasks can gate on it via the topic
/// (R16-style, not a cross-task object): StateManager::requestArm() reads `calibrated`,
/// StateTask reads `airborne` (ToF takeoff detection) to drive TAKEOFF → FLYING for the
/// MANUAL takeoff (ACRO/STABILIZE) only — the ALT/POS auto-takeoff completes on the
/// controller capturing its target altitude (controller_status.takeoff_reached), so the
/// ToF airborne edge there serves the ESKF vertical handoff alone (applyVerticalGroundHandoff,
/// above). Called every cycle from the loop so `airborne` tracks continuously.
/// 起動/システム準備状態をトピックで発行し、他タスクがゲートできるようにする（R16 流、
/// クロスタスクのオブジェクトでなく）: requestArm() は calibrated を読む。StateTask は
/// airborne（ToF 離陸検出）を読み「手動」離陸（ACRO/STABILIZE）の TAKEOFF→FLYING を駆動する
/// — ALT/POS 自動離陸は制御器の目標高度捕捉（controller_status.takeoff_reached）で完了するため、
/// そこでの ToF 空中エッジは ESKF 鉛直ハンドオフ（上の applyVerticalGroundHandoff）専用。
/// airborne が連続追従するよう毎周期発行。
static void publishSystemStatus(uint32_t now_us)
{
    sf::SystemStatus st{};
    st.calibrated = g_calibrated;
    st.airborne   = !g_takeoff_landing.isOnGround();
    st.held       = g_takeoff_landing.isHeld();           // IDLE_GROUND ↔ IDLE_HELD
    st.landing    = g_takeoff_landing.isLandingDetected(); // LANDING → IDLE_GROUND
    st.timestamp  = now_us;
    sf::system_status.publish(st);
}

/// Set up the boot gyro/accel bias calibration (called once in ImuTask setup). Reads
/// calibration.enable; when on, primes the CalibrationMgr and arms the in-loop feed.
/// Does NOT block — feedBootCalibration() does the per-cycle work.
/// 起動時のジャイロ/加速度バイアス校正をセットアップ（ImuTask setup で1回）。
/// calibration.enable を読み、ON なら CalibrationMgr を準備しループ内 feed を起動。
/// ブロックしない — 周期処理は feedBootCalibration() が行う。
///
/// @design detailed_design.md §3 — onEnter(IDLE): start calibration    [OK]
static void startBootCalibration()
{
    bool enable = true;
    sf::params::get_bool("calibration.enable", enable);
    if (!enable) {
        ESP_LOGW(TAG, "Boot calibration DISABLED (calibration.enable=0) "
                      "— estimator starts at zero bias");
        g_calibrated = true;        // nothing to wait for → ARM not gated on calibration
        return;
    }

    // Load any persisted NVS calibration first (overwritten by the fresh measure).
    // まず保存済み NVS 校正を読む（新規測定で上書きされる）。
    g_calib.init();

    // Stillness gate (config.hpp is the SSOT; sf_calibration is a leaf component
    // that cannot include it, so the thresholds are passed in here).
    // 静止ゲート（SSOT は config.hpp。sf_calibration は leaf コンポーネントで include
    // できないため、ここで閾値を渡す）。
    sf::StillnessConfig still{};
    still.gyro_max       = config::CALIB_STILL_GYRO_MAX;
    still.gyro_var_max   = config::CALIB_STILL_GYRO_VAR_MAX;
    still.accel_norm_min = config::CALIB_STILL_ACCEL_NORM_MIN;
    still.accel_norm_max = config::CALIB_STILL_ACCEL_NORM_MAX;
    still.accel_var_max  = config::CALIB_STILL_ACCEL_VAR_MAX;
    still.ema_alpha      = config::CALIB_STILL_EMA_ALPHA;
    g_calib.startGyroCal(config::CALIB_GYRO_SAMPLES, still);
    g_calib_active        = true;
    g_calib_settle_cycles = config::CALIB_SETTLE_MS * 1000u / config::IMU_PERIOD_US;
    g_calibrated          = false;  // calibration pending → pre-arm check rejects ARM
    ESP_LOGI(TAG, "Boot calibration: settle %lu cycles, then average %lu samples",
             static_cast<unsigned long>(g_calib_settle_cycles),
             static_cast<unsigned long>(config::CALIB_GYRO_SAMPLES));
}

/// Per-cycle boot-calibration step (called from the 400Hz loop while active). Discards
/// the settling-transient cycles, then averages CALIB_GYRO_SAMPLES at-rest samples and
/// seeds the estimator with the measured bias exactly once on completion.
/// 周期ごとの起動校正ステップ（active 中は 400Hz ループから呼ぶ）。整定過渡の周期を捨て、
/// 静止サンプルを CALIB_GYRO_SAMPLES 個平均し、完了時に一度だけ推定器へ種付けする。
static void feedBootCalibration(const sf::ImuData& imu)
{
    // Calibration is valid only at rest. If the craft is armed (taking off or already
    // flying) before the average completes, abort and leave the estimator untouched —
    // averaging through flight motion (e.g. a yaw spin) yields a garbage bias. Read the
    // arm state from system_mode (Latest, non-consuming).
    // 校正は静止時のみ有効。平均完了前に arm された（離陸中/飛行中）なら中止し推定器を
    // 触らない — 飛行運動（例: ヨー回転）を平均するとバイアスがゴミになる。arm 状態は
    // system_mode（Latest, 非消費）から読む。
    const sf::SystemMode mode = sf::system_mode.latest();
    if (sf::isArmed(static_cast<sf::FlightState>(mode.state))) {
        g_calib_active = false;
        g_calibrated   = true;      // no longer pending (aborted)
        ESP_LOGW(TAG, "Boot calibration aborted — armed before completion "
                      "(estimator left untouched)");
        return;
    }

    // Skip the settling transient so the average reflects gravity + true bias only.
    // 整定過渡を捨て、平均が重力＋真のバイアスのみを反映するようにする。
    if (g_calib_settle_cycles > 0) {
        --g_calib_settle_cycles;
        return;
    }

    const float gyro[3]  = {imu.gyro[0],  imu.gyro[1],  imu.gyro[2]};
    const float accel[3] = {imu.accel[0], imu.accel[1], imu.accel[2]};
    if (!g_calib.feedSample(gyro, accel)) {
        return;   // still accumulating / まだ蓄積中
    }

    // Complete. Stop the in-loop feed and mark boot readiness regardless of what we
    // decide below (apply or deadband-skip) — the calibration is no longer pending.
    // 完了。以降（適用 or デッドバンドスキップ）に関わらずループ内 feed を止め、起動準備
    // 完了を通知する — 校正はもう保留中でない。
    g_calib_active = false;
    g_calibrated   = true;

    // The craft is verified-still and the attitude converged: (re-)capture the
    // mag NED reference from the next mag sample (see applyMagBootPolicy).
    // 機体は静止確認済みで姿勢は収束済み: 次の磁気サンプルで NED 参照を（再）捕捉
    // する（applyMagBootPolicy 参照）。
    g_mag_ref_pending = true;

    const sf::CalibrationData& d = g_calib.data();

    // Largest per-axis absolute bias (|x| inline, no <cmath> dependency).
    // 軸ごとの絶対バイアスの最大（<cmath> 依存を避け |x| をインラインで）。
    auto absf = [](float v) { return v < 0.0f ? -v : v; };
    float gyro_mag = absf(d.gyro_bias[0]);
    float accel_mag = absf(d.accel_bias[0]);
    for (int i = 1; i < 3; ++i) {
        if (absf(d.gyro_bias[i])  > gyro_mag)  gyro_mag  = absf(d.gyro_bias[i]);
        if (absf(d.accel_bias[i]) > accel_mag) accel_mag = absf(d.accel_bias[i]);
    }

    // Deadband: a negligible measured bias (a clean IMU) is NOT seeded. Seeding a
    // near-zero "calibration" mid-run would overwrite the filter's own converged bias
    // estimate and perturb the marginal POSITION_HOLD entry, so leave the estimator
    // untouched (a true no-op) when there is nothing meaningful to correct.
    // デッドバンド: 無視可能な測定バイアス（クリーンな IMU）は種付けしない。ほぼゼロの
    // 「校正」を実行中に種付けするとフィルタの収束済みバイアス推定を上書きし、脆弱な
    // POSITION_HOLD 入口を撹乱する — 補正すべきものが無ければ推定器に触れない（真の no-op）。
    if (gyro_mag < config::CALIB_GYRO_DEADBAND && accel_mag < config::CALIB_ACCEL_DEADBAND) {
        ESP_LOGI(TAG,
                 "Boot calibration: bias negligible (gyro %.4f<%.4f, accel %.3f<%.3f) "
                 "— estimator left untouched",
                 gyro_mag, config::CALIB_GYRO_DEADBAND, accel_mag, config::CALIB_ACCEL_DEADBAND);
        return;
    }

    // Significant bias: seed the estimator (gravity already removed for accel Z).
    // 有意なバイアス: 推定器に種付け（加速度 Z の重力は除去済み）。
    g_estimator->applyCalibration(d.gyro_bias, d.accel_bias);
    g_calib_applied = true;
    for (int i = 0; i < 3; ++i) {
        g_applied_gyro_bias[i]  = d.gyro_bias[i];
        g_applied_accel_bias[i] = d.accel_bias[i];
    }
    ESP_LOGI(TAG,
             "Boot calibration applied: gyro_bias=[%.4f %.4f %.4f] rad/s, "
             "accel_bias=[%.3f %.3f %.3f] m/s2",
             d.gyro_bias[0], d.gyro_bias[1], d.gyro_bias[2],
             d.accel_bias[0], d.accel_bias[1], d.accel_bias[2]);
}

/// Re-apply the retained boot-calibration biases after a full estimator reset. A full
/// reset() zeroes the bias states (estimator.hpp), so without this a post-reset estimator
/// starts from zero bias and re-converges through the next takeoff transient — the very
/// instability the boot calibration removes. No-op if no calibration was applied.
/// full reset 後に保持した起動校正バイアスを再適用する。full reset() はバイアス状態をゼロ化
/// する（estimator.hpp）ので、これが無いと reset 後の推定器がゼロバイアスから始まり次の離陸
/// 過渡で再収束する — 起動校正が除去するまさにその不安定。校正未適用なら no-op。
static void reseedCalibration()
{
    if (!g_calib_applied) {
        return;
    }
    g_estimator->applyCalibration(g_applied_gyro_bias, g_applied_accel_bias);
    ESP_LOGI(TAG, "Calibration re-seeded after estimator reset");
}

/// Consume estimator commands published by the StateManager transition callbacks
/// (architecture §4). This is the estimator-side endpoint of the reset consolidation:
/// the state machine decides WHEN to reset; ImuTask (which owns the estimator) decides
/// HOW. Replaces the ad-hoc edge detection that used to live in this task.
/// StateManager の遷移コールバックが発行した推定器コマンドを消費する（architecture §4）。
/// reset 集約の推定器側エンドポイント: いつ reset するかは状態機械が、どう reset するかは
/// 推定器を所有する ImuTask が決める。本タスクにあった旧来のエッジ検出を置き換える。
///
/// @design architecture.md §4 — reset consolidation (estimator side)   [OK]
static void processEstimatorCommands()
{
    sf::EstimatorCommand cmd;
    while (sf::estimator_command.read(cmd)) {
        switch (static_cast<sf::EstimatorCmd>(cmd.command)) {
        case sf::EstimatorCmd::Reset:
            g_estimator->reset();
            reseedCalibration();   // reset zeroed the bias — re-apply the calibration
            break;
        case sf::EstimatorCmd::ResetPosVel:
            g_estimator->resetPositionVelocity();
            ESP_LOGI(TAG, "Estimator pos/vel reset (takeoff handoff)");
            break;
        case sf::EstimatorCmd::FreezeBias:
            g_estimator->freezeBias();
            break;
        case sf::EstimatorCmd::UnfreezeBias:
            g_estimator->unfreezeBias();
            break;
        case sf::EstimatorCmd::InflateCov: {
            // Map the semantic CovScope (carried in cmd.arg) to the estimator's state-bit
            // mask, then inflate. State bit layout (StateIdx): pos 0-2, vel 3-5, att 6-8,
            // bg 9-11, ba 12-14. Keeps state_task free of estimator-internal indices.
            // 意味的な CovScope（cmd.arg）を推定器の状態ビットマスクに変換して膨張。状態ビット
            // 配置（StateIdx）: 位置0-2, 速度3-5, 姿勢6-8, ジャイロbias9-11, 加速度bias12-14。
            // state_task を推定器内部インデックスから切り離す。
            uint16_t mask = 0x7FFF;   // CovScope::All
            switch (static_cast<sf::CovScope>(cmd.arg)) {
            case sf::CovScope::PosVel:     mask = 0x003F; break;  // pos+vel
            case sf::CovScope::PosVelBias: mask = 0x7E3F; break;  // pos+vel+bias (keep att)
            case sf::CovScope::Attitude:   mask = 0x01C0; break;  // attitude only
            case sf::CovScope::All:        mask = 0x7FFF; break;  // all 15
            }
            g_estimator->inflateCovariance(mask);
            ESP_LOGI(TAG, "Estimator covariance inflated (scope=%u mask=0x%04X)",
                     static_cast<unsigned>(cmd.arg), static_cast<unsigned>(mask));
            break;
        }
        case sf::EstimatorCmd::Recalibrate:
            startBootCalibration();
            break;
        case sf::EstimatorCmd::ReloadParams:
            // Live tuning: re-read eskf.* in THIS task's context (the params
            // callback only publishes the verb — no cross-task touching).
            // ライブチューニング: 本タスクの文脈で eskf.* を読み直す（params
            // コールバックは verb を発行するだけ — タスク跨ぎで触らない）。
            g_estimator->reloadParams();
            break;
        default:
            break;
        }
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
    // sf_board owns the SPI bus (R1): it called spi_bus_initialize() in Phase 1,
    // so the IMU driver must only add its device, not re-init the bus. We set the
    // flag here (a plain bool) rather than calling sf::internal::board::imu_spi()
    // because this source is shared with the partial SIL test programs that do
    // not link sf_board; spi_host stays SPI2_HOST = board::imu_spi() by design.
    // SPI バスは sf_board が所有(R1)し Phase 1 で spi_bus_initialize() 済み。よって
    // IMU ドライバは device add のみ行う。この .cpp は sf_board をリンクしない部分
    // SIL 試験プログラムと共有のため board::imu_spi() を呼ばず bool で設定する。
    auto imu_cfg = stampfly::BMI270Wrapper::Config::defaultStampFly();
    imu_cfg.skip_bus_init = true;
    esp_err_t imu_init_result = imu_wrapper.init(imu_cfg);

    // Release FlowTask: the shared SPI bus is no longer mid-init (success or
    // failure alike — what matters is that BMI270 transactions are not in flight).
    // FlowTask を解放: 共有 SPI バスは初期化中ではなくなった（成功・失敗を問わず、
    // BMI270 のトランザクションが飛んでいないことが重要）。
    s_imu_spi_init_done.store(true, std::memory_order_release);

    if (imu_init_result != ESP_OK) {
        // IMU is Critical (R4, hardware_init.md §5): no attitude estimation means
        // no flight. Do NOT vTaskDelete and let the rest of the system limp —
        // halt loudly (no esp_restart, so the cause stays observable; the LED
        // error pattern is wired in Phase 6 when LED ownership lands).
        // IMU は Critical (R4, §5): 姿勢推定不能＝飛行不能。vTaskDelete で他を片肺
        // 運転させず、大きく停止する (esp_restart せず原因を保つ。LED は Phase 6)。
        ESP_LOGE(TAG, "CRITICAL: BMI270 init failed: %s — vehicle cannot fly. Halting.",
                 esp_err_to_name(imu_init_result));
        for (;;) {
            vTaskDelay(portMAX_DELAY);
        }
    }
    ESP_LOGI(TAG, "BMI270 init OK (400Hz read loop starting)");

    // Create + initialize the estimator selected by estimator.type.
    // estimator.type で選ばれた推定器を生成・初期化。
    g_estimator = createEstimator();

    // Initialize the takeoff/landing manager (ToF-altitude ground/airborne detection).
    // 離着陸マネージャを初期化（ToF 高度で接地/空中を判定）。
    g_takeoff_landing.init();

    // Initialize the IMU-rate anomaly detector (impact / gyro anomaly) with the
    // thresholds from the safety.* parameters (params SSOT, not struct defaults).
    // IMU レート異常検出器を初期化（衝撃/ジャイロ異常）。閾値は safety.* パラメータ
    // （params SSOT）から取得し、struct 既定値の暗黙使用にしない。
    g_imu_anomaly.init(sf::loadFailsafeConfigFromParams());

    // The vertical ground-hold applies only when ToF is the vertical sensor (a
    // barometer would anchor altitude from the ground, making the hold unnecessary
    // and its ToF-driven release impossible). Read eskf.use_tof once.
    // 地上ホールドは ToF が鉛直センサの時のみ適用（気圧計があれば地上から錨付けでき
    // ホールドは不要かつ ToF 駆動の解除が不能になる）。eskf.use_tof を1回読む。
    bool use_tof = true;
    sf::params::get_bool("eskf.use_tof", use_tof);
    g_tof_vertical = use_tof;

    // The boot gyro/accel bias calibration is now started by the onEnter(IDLE_GROUND)
    // transition callback (architecture §6 / detailed_design §3 — calibration is
    // callback-driven), which publishes estimator_command(Recalibrate); this task
    // consumes it in processEstimatorCommands() and arms startBootCalibration(). The
    // in-loop feed (feedBootCalibration) then runs while the craft rests on the ground,
    // measuring the IMU bias and seeding the estimator before takeoff. (Previously this
    // was called directly here in setup — an ad-hoc bypass of the onEnter callback path.)
    // 起動バイアス校正は onEnter(IDLE_GROUND) 遷移コールバックが開始する（architecture §6 /
    // detailed_design §3 — 校正はコールバック駆動）。コールバックが estimator_command(Recalibrate)
    // を publish し、本タスクが processEstimatorCommands() で消費して startBootCalibration() を
    // 起動する。以降ループ内 feed（feedBootCalibration）が地上静止中に走り、IMU バイアスを測って
    // 離陸前に推定器へ種付けする。（以前はここ setup で直接呼んでいた — onEnter 経路の場当たり迂回。）

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

        // IMU-rate safety checks (requirements §9): impact / gyro anomaly at the
        // sample rate, so "2 consecutive" means 5 ms — fast enough for a real
        // crash spike. Alerts go to system_alert; the StateManager decides.
        // IMU レートの安全監視（要件§9）: 衝撃/ジャイロ異常をサンプルレートで判定。
        // 「連続2回」= 5ms で実墜落スパイクに十分速い。アラートは system_alert へ、
        // 判断は StateManager。
        g_imu_anomaly.feedSample(
            imu.accel, imu.gyro,
            sf::isArmed(static_cast<sf::FlightState>(sf::system_mode.latest().state)));

        // Process estimator commands from the StateManager transition callbacks
        // (reset / pos-vel reset / bias freeze / recalibrate). Done before predict so a
        // reset takes effect this cycle (architecture §4 reset consolidation).
        // StateManager の遷移コールバックからの推定器コマンドを処理（reset/pos-vel reset/
        // bias freeze/recalibrate）。predict 前に行い reset がこの周期で効くようにする
        // （architecture §4 reset 集約）。
        processEstimatorCommands();

        // Boot calibration: while still active (on the ground at boot), accumulate the
        // at-rest bias and seed the estimator once on completion. Runs in parallel with
        // the normal predict below (the on-ground estimate is anchored), so it adds no
        // delay and the 400Hz timing is unchanged.
        // 起動校正: active 中（起動時の地上）は静止バイアスを蓄積し、完了時に一度だけ推定器へ
        // 種付け。下の通常 predict と並行（接地推定は錨付け）で遅延を生まず 400Hz は不変。
        if (g_calib_active) {
            feedBootCalibration(imu);
        }

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

        // Publish boot/system readiness (calibrated + ToF airborne) for the pre-arm
        // check (requestArm) and the takeoff sequencer (StateTask: TAKEOFF → FLYING).
        // 起動/システム準備状態（calibrated + ToF 空中）を発行: ARM 前チェック（requestArm）と
        // 離陸シーケンサ（StateTask: TAKEOFF→FLYING）が読む。
        publishSystemStatus(now);

        // Boot-complete chime: on the calibration false→true edge (boot bias done →
        // ready to arm), fire NotifyEvent::Ready once. NotifyTask plays the 3-beep
        // readyTone (legacy vehicle's Phase-3 chime). One-shot via the edge latch.
        // 起動完了音: 校正の false→true エッジ（起動バイアス完了→ARM 可能）で NotifyEvent::Ready
        // を1回発火。NotifyTask が 3連ビープ readyTone（旧 vehicle の Phase3 音）を鳴らす。
        static bool s_prev_calibrated = false;
        if (g_calibrated && !s_prev_calibrated) {
            sf::notify_command.publish(
                {static_cast<uint8_t>(sf::NotifyEvent::Ready), static_cast<uint32_t>(now)});
        }
        s_prev_calibrated = g_calibrated;

        // CPU-load watermark: per-cycle busy time vs the 2500µs budget, logged
        // once a minute. KEPT (not a temporary diag): core-1 saturation at -Og
        // starved StateTask into the INIT-stuck bug, and this number is the
        // early warning if estimator/control cost ever creeps back up.
        // CPU 負荷ウォーターマーク: 周期あたり busy 時間 vs 予算 2500µs を毎分ログ。
        // 恒久採用（一時診断ではない）: -Og 時のコア1 飽和が StateTask を飢餓させ
        // INIT 停止バグになった。推定・制御のコストが再び肥大したときの早期警報。
        {
            static uint32_t s_busy_sum_us = 0;
            static uint32_t s_busy_max_us = 0;
            constexpr uint32_t kLoadLogEveryCycles = 24000;   // 60 s at 400Hz
            const uint32_t busy_us =
                static_cast<uint32_t>(esp_timer_get_time()) - now;
            s_busy_sum_us += busy_us;
            if (busy_us > s_busy_max_us) {
                s_busy_max_us = busy_us;
            }
            if ((cycle_count % kLoadLogEveryCycles) == 0u) {
                ESP_LOGI(TAG, "400Hz load: busy_avg=%luus max=%luus (budget 2500us)",
                         static_cast<unsigned long>(s_busy_sum_us / kLoadLogEveryCycles),
                         static_cast<unsigned long>(s_busy_max_us));
                s_busy_sum_us = 0;
                s_busy_max_us = 0;
            }
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

        TaskHandle_t control = sf::tasks::control_handle();
        if (control != nullptr) {
            xTaskNotifyGive(control);
        }
        // The esp_timer paces the loop; the ulTaskNotifyTake at the top blocks
        // until the next 400Hz tick, so no explicit delay is needed here.
        // ループは esp_timer が刻む。先頭の ulTaskNotifyTake が次の 400Hz ティック
        // までブロックするので、ここで明示的な待ちは不要。
    }
}
