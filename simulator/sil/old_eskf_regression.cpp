/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL — M11 regression challenge).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file old_eskf_regression.cpp
 * @brief M11-2/M11-3 regression challenge — reproduce legacy ESKF bugs, then
 *        show the redesign fixes them (legacy vs new ESKF on identical inputs)
 *        M11-2/M11-3 回帰チャレンジ — 旧ESKFの既知バグを再現し、redesign が
 *        それを直すことを示す(旧vs新ESKFを同一入力で比較)
 *
 * The regression challenge is the north star of the diagnostic-instrument SIL:
 * "can the SIL actually reproduce the known bugs of the legacy estimator?"
 * If it can, the SIL is proven to catch real estimator failures; the differential
 * diagnosis (reproduced → software cause / not reproduced → HW・concurrency cause)
 * then has teeth.
 *
 * 回帰チャレンジは診断計器SILの北極星: 「SILは旧推定器の既知バグを本当に
 * 再現できるか?」 再現できれば、SILが実際の推定器故障を検出できる証明になり、
 * 差分診断(再現→ソフト要因 / 非再現→HW・並行性要因)が意味を持つ。
 *
 * This harness drives BOTH the UNMODIFIED legacy ESKF (firmware/vehicle,
 * stampfly::) and the new redesign (firmware/vehicle_new, sf::EskfCore) with the
 * SAME quad_physics + sensor synthesis as integrated_sil. Both ESKFs observe the
 * identical sensor stream as open-loop observers (not in the control loop; truth
 * drives a PID hover), so this is a rigorous A/B: legacy reproduces the bug, the
 * redesign should not. Code Identity is preserved on both sides
 * (git diff firmware/ stays empty).
 *
 * 本ハーネスは無改変の旧ESKF(firmware/vehicle, stampfly::)と新redesign
 * (firmware/vehicle_new, sf::EskfCore)を、integrated_sil と同じ quad_physics +
 * センサ合成で並走駆動する。両ESKFは同一センサ列をオープンループ観測(制御は
 * 真値PIDホバリング)するため、厳密なA/B: 旧はバグ再現、新は再現しないはず。
 * Code Identity は両側で維持(git diff firmware/ は空)。
 *
 * Modes (--mode):
 *   pcollapse : Bug A root cause — sustained ToF collapses P(POS_Z) below R.
 *               Shared by both filters (normal Kalman); BENIGN under absolute
 *               ToF gates. The operational bug lives in the flow channel.
 *               バグA根本原因 — ToF長時間観測でP(POS_Z)がR以下に崩壊。両者
 *               共通(正常Kalman)で絶対値ゲート下では無害。運用バグはflow側。
 *   flow      : Bug A operational — legacy P(VEL) collapse breaks the flow chi2
 *               gate → velocity diverges; redesign clamps innovation → bounded.
 *               --no-flow-gate disables the legacy chi2 gate (control: bounded).
 *               バグA運用 — 旧P(VEL)崩壊でflow χ²ゲート破綻→速度発散; 新は
 *               イノベーションクランプで有界。--no-flow-gate で旧χ²無効(対照)。
 *   ba        : Bug C — accel bias drift when BA left unfrozen (--free-ba).
 *               Legacy drifts; redesign stays bounded.
 *               バグC — BA未凍結時(--free-ba)の漂流。旧は漂流、新は有界。
 *   negcal    : Negative calibration — why bugs D/E are out of SIL scope.
 *               陰性較正 — バグD/EがSIL射程外である根拠。
 *
 * Build & run: make old_eskf_regression
 *   ./old_eskf_regression --mode flow > flow.csv    # OLD diverges, NEW bounded
 *   ./old_eskf_regression --mode ba --free-ba       # OLD drifts, NEW bounded
 *   ./old_eskf_regression --mode flow --no-flow-gate # control: OLD also bounded
 *   stdout = CSV (both estimators), stderr = A/B verdict / debug
 *
 * @design project_sil_architecture (memory) §regression challenge (north star) [--]
 * @design development_roadmap.md §M11 — legacy bug reproduction + redesign A/B  [--]
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

// New-side: physics, math, control (reused unmodified for Code Identity)
// 新側: 物理・数学・制御(無改変で再利用、Code Identity)
#include "sf_math.hpp"
#include "pid.hpp"
#include "quad_physics.hpp"
#include "eskf_core.hpp"   // new ESKF (sf::EskfCore) — the redesign under comparison

// Legacy-side: the ESKF under test (firmware/vehicle, stampfly:: namespace)
// 旧側: 試験対象ESKF(firmware/vehicle, stampfly:: 名前空間)
#include "eskf.hpp"

using namespace sf;
using namespace sf::math;
using namespace sf::sim;

// =============================================================================
// Simulation constants (all named — no magic numbers)
// シミュレーション定数(全て命名 — マジックナンバー禁止)
// =============================================================================

namespace {

constexpr float DT          = 0.0025f;   // 400 Hz control/IMU step [s]
constexpr int   TOF_DIV     = 13;        // ToF update divider → ~30 Hz
constexpr int   FLOW_DIV    = 4;         // Flow update divider → 100 Hz
constexpr float LPF_ALPHA   = 0.32f;     // ESKF input LPF (matches integrated_sil)
constexpr float HOVER_ALT   = 0.5f;      // Hover altitude target [m]

constexpr float T_SETTLE_END = 1.0f;     // On-ground sensor settle window end [s]
constexpr float T_CALIB_END  = 3.0f;     // Calibration window end [s]
constexpr int   CALIB_SAMPLES = 400;     // Calibration average sample count

constexpr float T_EVAL_START = 6.0f;     // Skip takeoff transient before judging [s]

// Bug-A (altitude) verdict: P(POS_Z) below R_tof (=tof_noise^2) means the
// covariance collapsed below the measurement noise, breaking chi2 gating.
// バグA(高度)判定: P(POS_Z) が R_tof(=tof_noise^2)未満 = 共分散が観測ノイズを
// 下回って崩壊し χ² ゲートが破綻する閾値。
constexpr float PCOLLAPSE_THRESH = 1e-4f;
// Bug-A (velocity) verdict: |ESKF vel_xy| above this (truth~0) is divergence.
// バグA(速度)判定: |ESKF vel_xy| がこれ超(真値~0)なら発散。
constexpr float VEL_DIVERGE_THRESH = 5.0f;
// Bug-C verdict: |accel bias drift| above this from calibration is corruption.
// バグC判定: 較正値からの |加速度バイアス漂流| がこれ超なら推定値の汚染。
constexpr float BA_DRIFT_THRESH = 0.2f;

}  // namespace

// =============================================================================
// Vec3 (new sf::math) → Vector3 (legacy stampfly::math) conversion
// 両者ともフィールド x/y/z 同名・両 NED 座標系なので単純コピー
// Both have identically-named x/y/z fields and both are NED → plain copy.
// =============================================================================

static stampfly::math::Vector3 toOld(const Vec3& v)
{
    return stampfly::math::Vector3(v.x, v.y, v.z);
}

// =============================================================================
// Faithful optical-flow forward model — synthesize PMW3901 pixel deltas that
// are CONSISTENT with the true motion, by inverting updateFlowRaw's decode
// (eskf.cpp:1065-1094) with the legacy default flow calibration.
//
// 忠実なオプティカルフロー順モデル — updateFlowRaw のデコード(eskf.cpp:1065-1094)
// を旧既定フローキャリブで反転し、真の運動と整合する PMW3901 画素変位を合成。
//
// The pixels encode TRUE motion (translation + rotation); updateFlowRaw then
// removes rotation using the MEASURED gyro. A naive px=0 stream is WRONG — it
// makes the decoder attribute (−rotation) as translation, injecting a spurious
// velocity that diverges. Encoding the true rotation component fixes this.
//
// 画素は真の運動(並進+回転)を符号化し、updateFlowRaw が測定ジャイロで回転を
// 除去する。px=0 のナイーブ合成は誤り — デコーダが(−回転)を並進と誤認し
// 速度を発散させる。真の回転成分を符号化することで解消。
// =============================================================================

struct FlowPixels { int16_t dx, dy; };

static FlowPixels synthFlow(const QuadState& truth, float distance)
{
    constexpr float RAD_PER_PIXEL = 0.00222f;  // legacy defaultConfig
    constexpr float C2B_XX = 0.943f, C2B_YY = 1.015f;
    constexpr float FLOW_DT = FLOW_DIV * DT;

    // True body-frame horizontal velocity (NED velocity → body)
    // 真の機体水平速度(NED速度 → 機体)
    Vec3 v_body = truth.attitude.inv_rotate(truth.velocity);

    // Translation flow (body) → camera, undo cam-to-body (diagonal)
    // 並進フロー(機体) → カメラ、cam-to-body(対角)を逆変換
    float ftx_cam = (v_body.x / distance) / C2B_XX;
    float fty_cam = (v_body.y / distance) / C2B_YY;

    // Add rotation flow the camera physically sees (gyro_scale = 1)
    // カメラが物理的に見る回転フローを加算(gyro_scale = 1)
    float fx_cam = ftx_cam + truth.angular_rate.y;
    float fy_cam = fty_cam - truth.angular_rate.x;

    // Camera angular rate → pixel delta
    // カメラ角速度 → 画素変位
    FlowPixels px;
    px.dx = static_cast<int16_t>(lroundf(fx_cam * FLOW_DT / RAD_PER_PIXEL));
    px.dy = static_cast<int16_t>(lroundf(fy_cam * FLOW_DT / RAD_PER_PIXEL));
    return px;
}

// =============================================================================
// Mixer: thrust/torque → motor duty (identical to integrated_sil / sf_actuator)
// ミキサー: 推力/トルク → モーターduty(integrated_sil / sf_actuator と同一)
// =============================================================================

static void mixer(float thrust_cmd, const float torque[3], float duty[4],
                  float k_thrust)
{
    const float L = 0.023f;
    const float kappa = 0.00971f;

    float t = thrust_cmd * 0.25f;
    float r = torque[0] / (4.0f * L);
    float p = torque[1] / (4.0f * L);
    float y = torque[2] / (4.0f * kappa);

    float t1 = t - r + p + y;  // M1 FR CCW
    float t2 = t - r - p - y;  // M2 RR CW
    float t3 = t + r - p + y;  // M3 RL CCW
    float t4 = t + r + p - y;  // M4 FL CW

    for (int i = 0; i < 4; i++) {
        float ti = (i == 0) ? t1 : (i == 1) ? t2 : (i == 2) ? t3 : t4;
        if (ti < 0) ti = 0;
        duty[i] = sqrtf(ti / k_thrust);
        if (duty[i] > 0.95f) duty[i] = 0.95f;
    }
}

// =============================================================================
// Truth-based hover controller — keeps the quad hovering so the legacy ESKF
// observes a realistic sustained-flight sensor stream. The controller uses the
// TRUTH state (not the ESKF), so the trajectory is deterministic and identical
// across modes. Same cascade & gains as integrated_sil (M3 baseline).
//
// 真値ベース・ホバリング制御器 — 旧ESKFが現実的な飛行センサ列を観測できるよう
// quad をホバリングさせる。制御は真値を使う(ESKF非投入)ので軌道は決定論的で
// 全モード共通。カスケード・ゲインは integrated_sil(M3基準)と同一。
// =============================================================================

struct HoverControl {
    PID rate_r, rate_p, rate_y;
    PID att_r, att_p;
    PID alt_p, alt_v;

    void init()
    {
        rate_r.kp = 1.365e-3f; rate_r.ti = 0.7f; rate_r.td = 0.01f;
        rate_r.output_limit = 5.2e-3f; rate_r.eta = 0.125f;
        rate_p = rate_r; rate_p.kp = 1.995e-3f;
        rate_y.kp = 5.31e-3f; rate_y.ti = 1.6f; rate_y.output_limit = 2.2e-3f;
        att_r.kp = 5.0f; att_r.ti = 4.0f; att_r.output_limit = 3.0f;
        att_p = att_r;
        alt_p.kp = 0.6f; alt_p.ti = 7.0f; alt_p.output_limit = 0.5f;
        alt_v.kp = 0.1f; alt_v.ti = 2.5f; alt_v.output_limit = 0.15f;
    }

    // Compute motor duty from the truth state for a level hover at HOVER_ALT.
    // 真値からレベルホバリング(HOVER_ALT)のモーターdutyを計算。
    void compute(const QuadState& truth, float hover_thrust, float k_thrust,
                 float duty[4])
    {
        float est_height = -truth.position.z;
        float est_climb  = -truth.velocity.z;
        float vel_sp = alt_p.compute(HOVER_ALT - est_height, DT);
        float thrust = hover_thrust + alt_v.compute(vel_sp - est_climb, DT);

        Vec3 euler = truth.attitude.to_euler();
        float rate_sp_r = att_r.compute(0 - euler.x, DT);
        float rate_sp_p = att_p.compute(0 - euler.y, DT);

        float torque[3];
        torque[0] = rate_r.compute(rate_sp_r - truth.angular_rate.x, DT);
        torque[1] = rate_p.compute(rate_sp_p - truth.angular_rate.y, DT);
        torque[2] = rate_y.compute(0 - truth.angular_rate.z, DT);

        mixer(thrust, torque, duty, k_thrust);
    }
};

// =============================================================================
// Mode dispatch
// モード分岐
// =============================================================================

enum class Mode { PCOLLAPSE, FLOW, BA, NEGCAL };

// -----------------------------------------------------------------------------
// Negative calibration — bugs D (landing detection) and E (arbiter contention)
// are OUT OF SCOPE for an ESKF-only SIL. State this explicitly with 3 grounds.
// 陰性較正 — バグD(着陸検出)・E(arbiter競合)はESKF単体SILの射程外。3根拠で明示。
// -----------------------------------------------------------------------------

static int runNegCal()
{
    fprintf(stderr,
        "[NEG-CAL] bugs D(landing)/E(arbiter) are NOT in ESKF-only SIL scope.\n"
        "  ground 1 (link boundary): this binary links eskf.cpp ONLY. "
        "landing_handler.hpp / stationary_detector.hpp (main/) and "
        "control_arbiter.cpp (sf_svc_control_arbiter/) are NOT linked.\n"
        "  ground 2 (dependency closure): the legacy ESKF depends only on "
        "esp_err.h / esp_log.h / stampfly_math.hpp (no FreeRTOS). "
        "control_arbiter depends on SemaphoreHandle_t / xSemaphoreTake / "
        "xTaskGetTickCount; landing_handler depends on armed-state + ToF timeout. "
        "Those trigger conditions do not exist in this single-thread loop.\n"
        "  ground 3 (determinism): this SIL is a single-threaded deterministic "
        "loop with a fixed seed. Bug E (race / priority inversion / mutex "
        "timeout) is a concurrency phenomenon that, by definition, cannot occur "
        "under deterministic sequential execution. Bug D is state-machine x "
        "timing and has no context here.\n"
        "  => non-reproduction is EXPECTED and is itself the diagnostic signal: "
        "D/E originate OUTSIDE the ESKF (state / arbiter / concurrency).\n");
    printf("mode,result\n");
    printf("negcal,D/E out-of-scope (link boundary + dependency closure + determinism)\n");
    return 0;
}

// -----------------------------------------------------------------------------
// Build the legacy ESKF config to match the firmware (TOF-only, mag/flow per
// the requested mode). Returns a defaultConfig() with sensor enables overridden.
// ファーム相当の旧ESKF設定を構築(TOFのみ、mag/flowはモード依存)。
// defaultConfig() のセンサ有効フラグを上書きして返す。
// -----------------------------------------------------------------------------

static stampfly::ESKF::Config makeLegacyConfig(bool use_flow, bool flow_gate)
{
    stampfly::ESKF::Config cfg = stampfly::ESKF::Config::defaultConfig();
    // Match integrated_sil's new ESKF: ToF only, baro/mag off (+flow per mode).
    // integrated_sil の新ESKFに合わせる: ToFのみ、baro/mag OFF(+flowはモード依存)。
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_MAG]  = false;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_BARO] = false;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_TOF]  = true;
    cfg.sensor_enabled[stampfly::ESKF::SENSOR_FLOW] = use_flow;
    cfg.tof_noise = 0.01f;  // match integrated_sil cfg.tof_noise
    // --no-flow-gate disables the flow chi2 gate. This is the DISCRIMINATING
    // control for the velocity-channel P-collapse: with the gate ON, the
    // collapsed P(VEL) makes valid flow updates fail chi2 → velocity diverges;
    // with the gate OFF, flow correctly pins velocity → bounded. Proves the
    // divergence is the chi2-gate-breaking consequence of P-collapse, not a
    // synthesis artifact.
    // --no-flow-gate は flow χ²ゲートを無効化。速度チャネルP崩壊の判別用対照:
    // ゲートONだと崩壊した P(VEL) が正常な flow を χ² 棄却→速度発散、OFFだと
    // flow が速度を正しく固定→有界。発散がP崩壊のχ²ゲート破壊の帰結であり
    // 合成人工産物でないことを証明する。
    if (!flow_gate) cfg.flow_chi2_gate = 0.0f;
    return cfg;
}

// -----------------------------------------------------------------------------
// Build the NEW ESKF (sf::EskfCore) config — mirrors integrated_sil's validated
// firmware config (ToF only + adaptive-R + innovation-clamp + ToF-velocity), so
// the comparison is "old design vs new design" running on identical inputs.
// 新ESKF(sf::EskfCore)設定 — integrated_sil の検証済みファーム設定(ToFのみ +
// 適応R + イノベーションクランプ + ToF速度観測)を踏襲。比較は同一入力上の
// 「旧設計 vs 新設計」。
// -----------------------------------------------------------------------------

static sf::EskfConfig makeNewConfig(bool use_flow)
{
    sf::EskfConfig cfg;  // defaults carry the redesign (k_adaptive, innov_clamp)
    cfg.use_tof  = true;
    cfg.use_baro = false;
    cfg.use_mag  = false;
    cfg.use_flow = use_flow;
    cfg.accel_noise = 0.3f;   // matches integrated_sil default
    cfg.gyro_noise  = 0.009655f;
    cfg.tof_noise   = 0.01f;  // matches integrated_sil
    return cfg;
}

// -----------------------------------------------------------------------------
// Main simulation: hover the quad, drive the legacy ESKF as an observer, and
// record the per-mode diagnostic to CSV (stdout). Verdict goes to stderr.
// 本シミュレーション: quad をホバリングさせ旧ESKFをオブザーバ駆動、モード別
// 診断量を CSV(stdout)に記録。判定は stderr。
// -----------------------------------------------------------------------------

static int runSim(Mode mode, float duration, bool free_ba, bool flow_gate,
                  unsigned seed)
{
    srand(seed);  // deterministic
    const int steps = static_cast<int>(duration / DT);
    const bool use_flow = (mode == Mode::FLOW);
    // FLOW mode isolates the velocity-channel P-collapse, so keep BA frozen
    // (firmware config); BA drift is the separate concern of the BA mode.
    // FLOW モードは速度チャネルP崩壊を分離するため BA は凍結(ファーム設定);
    // BA 漂流は BA モードの別関心事。
    if (mode == Mode::FLOW) free_ba = false;

    // --- Physics with full vibration noise model (N2) ---
    QuadPhysics quad;
    QuadParams qp;
    quad.init(qp, ContactParams{}, SensorNoiseParams{});
    const float hover_thrust = qp.mass * qp.gravity;

    // --- Both ESKFs observe the SAME sensor stream (open-loop), so this is a
    //     rigorous A/B on identical inputs: old (under test) vs new (redesign).
    // --- 旧・新ESKFが同一センサ列を観測(オープンループ) = 同一入力上の厳密A/B:
    //     旧(試験対象) vs 新(redesign)。
    stampfly::ESKF eskf;       // legacy (firmware/vehicle)
    eskf.init(makeLegacyConfig(use_flow, flow_gate));
    sf::EskfCore neweskf;      // redesign (firmware/vehicle_new)
    neweskf.init(makeNewConfig(use_flow));

    HoverControl ctrl;
    ctrl.init();

    // --- ESKF-input LPF (matches integrated_sil) ---
    Vec3 gyro_lpf = {}, accel_lpf = {};
    bool lpf_init = false;

    // --- Calibration accumulators ---
    Vec3 cal_accel_sum = {}, cal_gyro_sum = {};
    int cal_count = 0;
    bool eskf_ready = false;

    // --- Verdict accumulators (old = legacy under test, new = redesign) ---
    float min_pzz = 1e30f, last_kz = 0;        // bug A/altitude old
    float min_pzz_n = 1e30f, last_kz_n = 0;    // bug A/altitude new
    float max_vel_xy = 0, max_vel_xy_n = 0;    // bug A/velocity old, new
    Vec3 ba_calib = {}, ba_calib_n = {};       // bug C reference
    float max_ba_drift = 0, max_ba_drift_n = 0;// bug C old, new

    // --- CSV header (mode-specific; _old = legacy, _new = redesign) ---
    if (mode == Mode::PCOLLAPSE) {
        printf("time,true_z,tof,p_zz_old,k_z_old,p_zz_new,k_z_new\n");
    } else if (mode == Mode::FLOW) {
        printf("time,true_z,vel_old_x,vel_old_y,vel_new_x,vel_new_y,"
               "true_vx,true_vy\n");
    } else {  // BA
        printf("time,ba_old_x,ba_old_y,ba_old_drift,ba_new_x,ba_new_y,"
               "ba_new_drift\n");
    }

    for (int step = 0; step < steps; step++) {
        float t = step * DT;
        float zero_cmd[4] = {0, 0, 0, 0};

        // --- Sensors + LPF (same path as integrated_sil) ---
        SimSensors sens = quad.getSensors(DT);
        quad.updateBiases(DT);
        Vec3 accel_raw(sens.accel.x, sens.accel.y, sens.accel.z);
        Vec3 gyro_raw(sens.gyro.x, sens.gyro.y, sens.gyro.z);
        if (!lpf_init) { gyro_lpf = gyro_raw; accel_lpf = accel_raw; lpf_init = true; }
        gyro_lpf  = gyro_lpf  * (1.0f - LPF_ALPHA) + gyro_raw  * LPF_ALPHA;
        accel_lpf = accel_lpf * (1.0f - LPF_ALPHA) + accel_raw * LPF_ALPHA;
        Vec3 accel = accel_lpf;
        Vec3 gyro  = gyro_lpf;

        // --- Phase 1: on-ground settle (no control, no ESKF) ---
        if (t < T_SETTLE_END) { quad.step(zero_cmd, DT); continue; }

        // --- Phase 2: calibration (on ground, average accel/gyro) ---
        if (t < T_CALIB_END) {
            quad.step(zero_cmd, DT);
            if (cal_count < CALIB_SAMPLES) {
                cal_accel_sum += accel; cal_gyro_sum += gyro; cal_count++;
            }
            if (cal_count == CALIB_SAMPLES && !eskf_ready) {
                Vec3 accel_avg = cal_accel_sum * (1.0f / cal_count);
                Vec3 gyro_avg  = cal_gyro_sum  * (1.0f / cal_count);
                // setAttitudeReference sets level attitude + accel bias and
                // UNFREEZES accel bias (eskf.cpp:394). The firmware then always
                // re-freezes it; bug C is exposed by OMITTING that re-freeze.
                // setAttitudeReference はレベル姿勢+加速度バイアスを設定し
                // freeze を解除(eskf.cpp:394)。ファームは必ず再凍結する;
                // バグCはその再凍結を省くことで露出する。
                Vec3 accel_bias(accel_avg.x, accel_avg.y,
                                accel_avg.z - (-qp.gravity));
                // -- Legacy: setAttitudeReference sets level attitude + accel
                //    bias and UNFREEZES (eskf.cpp:394); firmware always re-freezes.
                //    Bug C is exposed by OMITTING that re-freeze.
                // -- 旧: setAttitudeReference はレベル姿勢+加速度バイアスを設定し
                //    freeze 解除(eskf.cpp:394); ファームは必ず再凍結。バグCは
                //    その再凍結を省くことで露出。
                eskf.setGyroBias(toOld(gyro_avg));
                eskf.setAttitudeReference(toOld(accel_avg), toOld(gyro_avg));
                if (!free_ba) eskf.setFreezeAccelBias(true);
                // -- New: same calibration as integrated_sil (firmware startup).
                // -- 新: integrated_sil(ファーム起動)と同じ較正手順。
                neweskf.setGyroBias(gyro_avg);
                neweskf.setAccelBias(accel_bias);
                neweskf.setAttitudeFromGravity(accel_avg);
                neweskf.shrinkBiasCovariance(0.01f);
                if (!free_ba) neweskf.setFreezeAccelBias(true);
                eskf_ready = true;
                // Record calibrated accel bias as the bug-C drift reference.
                // 較正済み加速度バイアスをバグC漂流の基準値として記録。
                stampfly::ESKF::State s0 = eskf.getState();
                ba_calib = Vec3(s0.accel_bias.x, s0.accel_bias.y, s0.accel_bias.z);
                ba_calib_n = neweskf.getAccelBias();
                fprintf(stderr,
                        "=== ESKFs ready: legacy mask=0x%04x | both freeze_ba=%d ===\n",
                        eskf.getActiveMask(), free_ba ? 0 : 1);
            }
            continue;
        }

        // --- Both ESKFs predict + observe identically (open-loop) ---
        // The redesign adds updateToFVelocity (vertical-velocity observation),
        // which the legacy lacks — a key reason the new filter stays consistent.
        // 両ESKFを同一に予測+観測(オープンループ)。redesign は updateToFVelocity
        // (垂直速度観測)を追加 — 旧にはなく、新が整合を保つ主因の一つ。
        eskf.predict(toOld(accel), toOld(gyro), DT);
        eskf.updateAccelAttitude(toOld(accel));
        neweskf.predict(accel, gyro, DT);
        neweskf.updateAccelAttitude(accel);
        if (step % TOF_DIV == 0) {
            eskf.updateToF(sens.tof_bottom);
            neweskf.updateToF(sens.tof_bottom);
            neweskf.updateToFVelocity(sens.tof_bottom, TOF_DIV * DT);  // new only
        }
        if (use_flow && step % FLOW_DIV == 0) {
            // Faithful flow consistent with truth (pixels encode true motion),
            // decoded against the measured gyro — the only flow↔gyro mismatch
            // is realistic sensor noise, not a synthesis artifact. Same pixels
            // feed both decoders (legacy: chi2 gate; redesign: innovation clamp).
            // 真値と整合する忠実フロー(画素は真の運動を符号化)を測定ジャイロで
            // デコード — flow↔gyro の不一致は現実的なセンサノイズのみ。同じ画素
            // を両デコーダに投入(旧: χ²ゲート; 新: イノベーションクランプ)。
            FlowPixels px = synthFlow(quad.getState(), sens.tof_bottom);
            eskf.updateFlowRaw(px.dx, px.dy, sens.tof_bottom, FLOW_DIV * DT,
                               gyro.x, gyro.y);
            neweskf.updateFlowRaw(px.dx, px.dy, sens.tof_bottom, FLOW_DIV * DT,
                                  gyro.x, gyro.y);
        }

        // --- Drive physics ---
        // Bug A (P-collapse) is a pure covariance-recursion property: drive it
        // with the quad STATIONARY on the ground so the vertical velocity stays
        // ~0 and the sustained, consistent ToF observation isolates the collapse
        // (a hover injects velocity uncertainty that masks it with divergence).
        // Bug C needs vibration → fly a hover (duty>0 excites the throttle-coupled
        // vibration that drives the unfrozen accel bias).
        // バグA(P崩壊)は純粋な共分散漸化の性質: quad を地上静止で駆動し垂直速度を
        // ~0 に保ち、持続的で一貫したToF観測で崩壊を分離する(ホバリングは速度
        // 不確かさで崩壊を発散により覆い隠す)。バグCは振動が必要 → ホバリング
        // (duty>0 がスロットル結合振動を励起し未凍結の加速度バイアスを駆動)。
        QuadState truth = quad.getState();
        if (mode == Mode::PCOLLAPSE) {
            quad.step(zero_cmd, DT);  // stay grounded
        } else {
            float duty[4];
            ctrl.compute(truth, hover_thrust, qp.k_thrust, duty);
            quad.step(duty, DT);
        }

        // --- Read both ESKFs' diagnostics ---
        const auto& P = eskf.getCovariance();           // legacy P-matrix
        stampfly::ESKF::State est = eskf.getState();
        const float R_TOF = 0.01f * 0.01f;              // tof_noise^2

        if (mode == Mode::PCOLLAPSE) {
            // Bug A (altitude channel): sustained ToF collapses P(POS_Z) < R_tof.
            float pzz   = P(stampfly::ESKF::POS_Z, stampfly::ESKF::POS_Z);
            float pzz_n = neweskf.getPDiag(POS_Z);
            float kz   = pzz   / (pzz   + R_TOF);
            float kz_n = pzz_n / (pzz_n + R_TOF);
            if (t >= T_EVAL_START) {
                if (pzz   < min_pzz)   min_pzz   = pzz;
                if (pzz_n < min_pzz_n) min_pzz_n = pzz_n;
            }
            last_kz = kz; last_kz_n = kz_n;
            if (step % 8 == 0)
                printf("%.4f,%.4f,%.4f,%.3e,%.3e,%.3e,%.3e\n",
                       t, truth.position.z, sens.tof_bottom,
                       pzz, kz, pzz_n, kz_n);
        } else if (mode == Mode::FLOW) {
            // Bug A (velocity channel): legacy P(VEL) collapse → flow chi2
            // false-reject → velocity diverges; redesign clamps innovation.
            float vx = est.velocity.x, vy = est.velocity.y;
            Vec3  vn = neweskf.getVelocity();
            if (t >= T_EVAL_START) {
                float m  = sqrtf(vx * vx + vy * vy);
                float mn = sqrtf(vn.x * vn.x + vn.y * vn.y);
                if (m  > max_vel_xy)   max_vel_xy   = m;
                if (mn > max_vel_xy_n) max_vel_xy_n = mn;
            }
            if (step % 8 == 0)
                printf("%.4f,%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                       t, truth.position.z, vx, vy, vn.x, vn.y,
                       truth.velocity.x, truth.velocity.y);
        } else {  // BA — bug C: unfrozen accel bias drifts from its calibration
            float ba_x = est.accel_bias.x, ba_y = est.accel_bias.y;
            Vec3  ba_n = neweskf.getAccelBias();
            float drift   = sqrtf(powf(ba_x - ba_calib.x, 2) +
                                  powf(ba_y - ba_calib.y, 2));
            float drift_n = sqrtf(powf(ba_n.x - ba_calib_n.x, 2) +
                                  powf(ba_n.y - ba_calib_n.y, 2));
            if (t >= T_EVAL_START) {
                if (drift   > max_ba_drift)   max_ba_drift   = drift;
                if (drift_n > max_ba_drift_n) max_ba_drift_n = drift_n;
            }
            if (step % 8 == 0)
                printf("%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
                       t, ba_x, ba_y, drift, ba_n.x, ba_n.y, drift_n);
        }

        // --- Debug (stderr, 1 Hz) ---
        if (step % 400 == 0) {
            fprintf(stderr, "t=%.1f h=%.3f ", t, -truth.position.z);
            if (mode == Mode::PCOLLAPSE)
                fprintf(stderr, "P_zz old=%.2e new=%.2e\n",
                        P(stampfly::ESKF::POS_Z, stampfly::ESKF::POS_Z),
                        neweskf.getPDiag(POS_Z));
            else if (mode == Mode::FLOW)
                fprintf(stderr, "vel old=(%.2f,%.2f) new=(%.2f,%.2f) [truth~0]\n",
                        est.velocity.x, est.velocity.y,
                        neweskf.getVelocity().x, neweskf.getVelocity().y);
            else
                fprintf(stderr, "ba old=(%.3f,%.3f) new=(%.3f,%.3f)\n",
                        est.accel_bias.x, est.accel_bias.y,
                        neweskf.getAccelBias().x, neweskf.getAccelBias().y);
        }
    }

    // --- Verdict (stderr): legacy reproduces the bug, redesign should not ---
    // The pass condition is the A/B contrast: OLD reproduces, NEW is fixed.
    // 判定: 旧はバグ再現、新は修正済みであること(A/B対比)が合格条件。
    if (mode == Mode::PCOLLAPSE) {
        // The P-collapse PHENOMENON (covariance shrinks below R) is the root
        // cause documented at eskf.hpp:135 — and it is shared by BOTH filters
        // (normal Kalman behavior under consistent ToF). It is BENIGN here
        // because both use ABSOLUTE ToF innovation gates. It becomes the bug
        // only where a chi2 gate sits atop the collapsed P — the legacy
        // flow/velocity channel (--mode flow). So this mode confirms the root
        // cause; --mode flow shows the operational divergence and the fix.
        // P-collapse 現象(共分散がR以下に縮む)は eskf.hpp:135 の根本原因で、
        // 両フィルタ共通(一貫ToF下の正常なKalman挙動)。ToFは新旧とも絶対値
        // イノベーションゲートを使うのでここでは無害。崩壊したPの上にχ²ゲートが
        // 乗る旧flow/速度チャネル(--mode flow)でのみバグ化する。本モードは
        // 根本原因の確認、--mode flow が運用上の発散と修正を示す。
        bool collapsed   = (min_pzz   < PCOLLAPSE_THRESH);
        bool collapsed_n = (min_pzz_n < PCOLLAPSE_THRESH);
        fprintf(stderr,
                "[BUG-A/altitude] P(POS_Z) min  OLD=%.3e (K_z=%.2e)  "
                "NEW=%.3e (K_z=%.2e)  R_tof=%.0e\n"
                "  P-collapse phenomenon: OLD %s, NEW %s "
                "(BENIGN — both use absolute ToF gates)\n"
                "  => root cause confirmed; operational bug is the chi2 "
                "consequence in the flow channel (see --mode flow) → %s\n",
                min_pzz, last_kz, min_pzz_n, last_kz_n, PCOLLAPSE_THRESH,
                collapsed ? "collapsed" : "bounded",
                collapsed_n ? "collapsed" : "bounded",
                collapsed ? "phenomenon reproduced ✅" : "NOT reproduced");
        return collapsed ? 0 : 2;
    } else if (mode == Mode::FLOW) {
        bool old_div = (max_vel_xy   > VEL_DIVERGE_THRESH);
        bool new_ok  = (max_vel_xy_n <= VEL_DIVERGE_THRESH);
        const char* concl;
        bool pass;
        if (flow_gate) {
            // Default: legacy chi2 gate ON → expect OLD diverges, NEW bounded.
            concl = (old_div && new_ok) ? "redesign fixes bug A/velocity ✅"
                                        : "INCONCLUSIVE";
            pass = (old_div && new_ok);
        } else {
            // Control: legacy chi2 gate OFF → expect OLD bounded too, proving
            // the divergence is the chi2-gate-breaking consequence of P-collapse.
            concl = (!old_div && new_ok)
                    ? "control ✅ — gate off bounds OLD too (divergence is chi2-caused)"
                    : "control INCONCLUSIVE";
            pass = (!old_div && new_ok);
        }
        fprintf(stderr,
                "[BUG-A/velocity] max|vel_xy| (truth~0)  OLD=%.2f m/s  "
                "NEW=%.3f m/s  (flow_gate=%s)\n"
                "  OLD: %s | NEW: bounded → %s\n",
                max_vel_xy, max_vel_xy_n, flow_gate ? "ON" : "OFF",
                old_div ? "DIVERGED (P-collapse breaks flow chi2) — reproduced"
                        : "bounded", concl);
        return pass ? 0 : 2;
    } else {  // BA
        bool old_bug = (max_ba_drift   > BA_DRIFT_THRESH);
        bool new_ok  = (max_ba_drift_n <= BA_DRIFT_THRESH);
        fprintf(stderr,
                "[BUG-C] freeze_ba=%d  max|ba_drift|  OLD=%.4f  NEW=%.4f m/s^2\n"
                "  OLD: %s | NEW: %s → %s\n",
                free_ba ? 0 : 1, max_ba_drift, max_ba_drift_n,
                old_bug ? "DRIFTED from calibration — reproduced" : "bounded",
                new_ok ? "bounded — redesign more robust" : "ALSO drifted",
                free_ba ? ((old_bug && new_ok) ? "redesign fixes bug C ✅"
                                               : "INCONCLUSIVE")
                        : "control: both frozen/bounded (firmware config)");
        // Frozen (control): both should be bounded. Unfrozen: old drifts, new ok.
        bool pass = free_ba ? (old_bug && new_ok) : (!old_bug && new_ok);
        return pass ? 0 : 2;
    }
}

// =============================================================================
// CLI
// =============================================================================

int main(int argc, char** argv)
{
    Mode mode = Mode::PCOLLAPSE;
    float duration = 30.0f;
    bool free_ba = false;
    bool flow_gate = true;
    unsigned seed = 1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--mode") && i + 1 < argc) {
            const char* m = argv[++i];
            if (!strcmp(m, "pcollapse")) mode = Mode::PCOLLAPSE;
            else if (!strcmp(m, "flow")) mode = Mode::FLOW;
            else if (!strcmp(m, "ba"))   mode = Mode::BA;
            else if (!strcmp(m, "negcal")) mode = Mode::NEGCAL;
        } else if (!strcmp(argv[i], "--dur") && i + 1 < argc) {
            duration = static_cast<float>(atof(argv[++i]));
        } else if (!strcmp(argv[i], "--free-ba")) {
            free_ba = true;
        } else if (!strcmp(argv[i], "--no-flow-gate")) {
            flow_gate = false;  // FLOW-mode control: disable flow chi2 gate
        } else if (!strcmp(argv[i], "--seed") && i + 1 < argc) {
            seed = static_cast<unsigned>(atoi(argv[++i]));
        }
    }

    if (mode == Mode::NEGCAL) return runNegCal();
    return runSim(mode, duration, free_ba, flow_gate, seed);
}
