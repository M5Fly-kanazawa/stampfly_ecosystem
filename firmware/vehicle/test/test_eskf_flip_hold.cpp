/**
 * @file test_eskf_flip_hold.cpp
 * @brief Unit tests for the flip maneuver's ESKF attitude-correction hold and
 *        ToF re-acquisition (flip-maneuver-plan.md §3.6, detailed_design.md §3
 *        FLIP row).
 *        宙返りマニューバの ESKF 姿勢補正ホールドと ToF 再取り込みの単体テスト
 *        （flip-maneuver-plan.md §3.6, detailed_design.md §3 FLIP 行）。
 *
 * Runs on host PC (not ESP32), same pattern as test_main.cpp: a minimal
 * self-contained TEST()/ASSERT_* framework, built as its OWN binary (see
 * Makefile) so it does not need to share test_main.cpp's runner list.
 * ホストPC上で実行（ESP32ではない）。test_main.cpp と同じ流儀: 最小の自己完結
 * TEST()/ASSERT_* フレームワークを、独自バイナリとしてビルドする（Makefile 参照。
 * test_main.cpp のランナー一覧を共有する必要がない）。
 *
 * @design docs/plans/flip-maneuver-plan.md §3.6 — estimator handling during flip [OK]
 * @design detailed_design.md §3 — FLIP row: HoldAttitudeCorrection/ResumeAttitudeCorrection [OK]
 * @design coding_and_education.md §2 — Bilingual comments                        [OK]
 */

#include <cstdio>
#include <cmath>

#include "sf_math.hpp"
#include "eskf_core.hpp"

// =============================================================================
// Test framework (minimal — mirrors test_main.cpp)
// テストフレームワーク（最小 — test_main.cpp と同型）
// =============================================================================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-45s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_NEAR(a, b, tol) \
    if (fabsf((a) - (b)) > (tol)) { \
        printf("FAIL: %s:%d: %.6f != %.6f (tol=%.6f)\n", \
               __FILE__, __LINE__, (float)(a), (float)(b), (float)(tol)); \
        throw 1; \
    }

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

// A quiet ESKF config for these tests: only the sensor under test is enabled,
// so unrelated observation paths (flow/mag/baro) cannot perturb the result.
// これらのテスト用の静かな ESKF 設定: テスト対象のセンサだけを有効にし、無関係な
// 観測経路（フロー/磁気/気圧）が結果を乱さないようにする。
static sf::EskfConfig quietConfig()
{
    sf::EskfConfig cfg;
    cfg.use_flow = false;
    cfg.use_mag  = false;
    cfg.use_baro = false;
    cfg.use_tof  = false;
    return cfg;
}

// Walk a ToF-height convergence up to target_height in steps no larger than
// `step`, converging tightly at each waypoint before advancing. A single jump
// larger than the absolute-innovation gate would itself be rejected by the
// very mechanism these tests exercise, so plain "call updateToF(target) 200x
// from a height-0 prior" does not converge — this helper is test SETUP, not
// the behavior under test (only used to reach a known pre-flip height).
// ToF 高度の収束を target_height まで `step` 以下の刻みで歩かせ、各中継点で
// しっかり収束させてから次へ進める。判定より大きい1回のジャンプは、これらの
// テストが検証する機構そのものによって棄却されてしまうため、高度0の事前状態から
// 素朴に updateToF(target) を200回呼んでも収束しない — このヘルパはテストの
// 「準備」であり、検証対象の挙動ではない（飛行前の既知高度へ到達させるためだけに使う）。
static void convergeTofHeight(sf::EskfCore& eskf, float target_height,
                               float step, int iters_per_step)
{
    // predict() runs between observations (level accel — no net motion, just
    // process noise) so the covariance does not collapse to ~0 across many
    // repeated updateToF() calls at the SAME waypoint; without it the Kalman
    // gain on the NEXT waypoint would be negligible and convergence would
    // stall partway (this is test setup, not the mechanism under test).
    // predict() を観測の合間に走らせる（水平加速度 — 正味の運動はなく、プロセス
    // ノイズだけ）。これがないと同一中継点への繰り返し updateToF() で共分散が
    // ~0 まで潰れ、次の中継点でのカルマンゲインが無視できるほど小さくなり、
    // 収束が途中で止まってしまう（これはテストの準備であり検証対象の機構ではない）。
    const sf::math::Vec3 level_accel(0.0f, 0.0f, -sf::math::kGravity);
    const sf::math::Vec3 zero_gyro(0.0f, 0.0f, 0.0f);

    float waypoint = -eskf.getPosition().z;   // NED: height = -pos_z
    while (target_height - waypoint > 1e-3f) {
        waypoint = (waypoint + step > target_height) ? target_height : waypoint + step;
        for (int i = 0; i < iters_per_step; i++) {
            eskf.predict(level_accel, zero_gyro, 0.0025f);
            eskf.updateToF(waypoint);
        }
    }

    // Settle at the final height: the ramp's climbing sequence of readings leaves
    // the POS_Z<->VEL_Z cross-covariance with a residual "still climbing" vel_z
    // estimate. A long run of readings that stop changing drives that residual
    // back toward zero (a real static hover would do the same) — needed here so
    // the flip window's predict-only phase does not inherit a spurious velocity
    // and drift on its own.
    // 最終高度で整定させる: ランプの「上昇し続ける」読み値の並びは POS_Z<->VEL_Z の
    // クロス共分散に「まだ上昇中」という残留 vel_z 推定を残す。値が変化しない読みを
    // 長く続けるとその残差はゼロへ戻る（実際の静止ホバーでも同様）— 宙返り窓の
    // predict のみフェーズが偽の速度を引き継いで自ら漂流しないよう、ここで整定させる。
    for (int i = 0; i < 400; i++) {
        eskf.predict(level_accel, zero_gyro, 0.0025f);
        eskf.updateToF(target_height);
    }
}

// =============================================================================
// (1) Hold blocks the accel-attitude correction; without Hold the same
//     gravity-mismatched observations visibly rotate the attitude.
// (1) Hold は加速度姿勢補正を止める。Hold 無しでは同じ重力不整合観測が姿勢を
//     目に見えて回転させる。
// =============================================================================

TEST(hold_blocks_accel_attitude_correction)
{
    sf::EskfConfig cfg = quietConfig();

    // A gravity-mismatched accelerometer reading — e.g. the maneuver's own
    // thrust/centripetal signature during a flip, NOT a real tilt.
    // 重力と食い違う加速度計の測定値 — 実際の傾きでなく、宙返り中のマニューバ
    // 自身の推力/遠心力の比力を模す。
    const sf::math::Vec3 tilted_accel(2.5f, 0.0f, -9.4f);

    sf::EskfCore held;
    held.init(cfg);
    held.holdAttitudeCorrection(true);

    sf::EskfCore free_running;
    free_running.init(cfg);

    for (int i = 0; i < 200; i++) {
        held.updateAccelAttitude(tilted_accel);
        free_running.updateAccelAttitude(tilted_accel);
    }

    // Held: attitude must stay EXACTLY at the initial identity quaternion —
    // updateAccelAttitude() returns before touching anything.
    // Hold あり: 姿勢は初期の単位クォータニオンのまま厳密に変わらない —
    // updateAccelAttitude() は何にも触れずに戻る。
    auto q_held = held.getAttitude();
    ASSERT_NEAR(q_held.w, 1.0f, 1e-9f);
    ASSERT_NEAR(q_held.x, 0.0f, 1e-9f);
    ASSERT_NEAR(q_held.y, 0.0f, 1e-9f);
    ASSERT_NEAR(q_held.z, 0.0f, 1e-9f);

    // Not held: 200 corrective updates against a mismatched gravity reference
    // must have rotated the attitude measurably away from identity.
    // Hold なし: 不整合な重力参照に対する200回の補正更新は姿勢を単位姿勢から
    // 目に見えて回転させているはず。
    auto q_free = free_running.getAttitude();
    ASSERT_TRUE(fabsf(q_free.x) > 0.01f || fabsf(q_free.y) > 0.01f);
}

// =============================================================================
// (2) After Resume, correct accelerometer observations re-converge the
//     attitude that drifted while Hold was active.
// (2) Resume 後、正しい加速度観測が Hold 中にドリフトした姿勢を再収束させる。
// =============================================================================

TEST(resume_reconverges_attitude)
{
    sf::EskfConfig cfg = quietConfig();
    sf::EskfCore eskf;
    eskf.init(cfg);

    eskf.holdAttitudeCorrection(true);

    // 0.5s of gyro-only integration with a small residual rate on roll — models
    // the gyro sensitivity-error residual the plan calls out (§3.6, "ジャイロ
    // 感度誤差") that is left uncorrected while attitude correction is held.
    // predict() is called every cycle exactly as EskfEstimator::predict() does
    // in production, together with updateAccelAttitude() (a no-op while held).
    // 0.5秒のジャイロのみ積分、ロールに小さな残留レートを与える — 計画（§3.6
    // 「ジャイロ感度誤差」）が挙げる、姿勢補正ホールド中は補正されない残差を模す。
    // predict() は本番の EskfEstimator::predict() と同じく毎サイクル呼び、
    // updateAccelAttitude()（ホールド中は no-op）も同様に呼ぶ。
    const sf::math::Vec3 level_accel(0.0f, 0.0f, -sf::math::kGravity);
    const sf::math::Vec3 drift_gyro(0.15f, 0.0f, 0.0f);   // [rad/s]
    for (int i = 0; i < 200; i++) {                        // 0.5s @ 400Hz
        eskf.predict(level_accel, drift_gyro, 0.0025f);
        eskf.updateAccelAttitude(level_accel);
    }

    auto euler_before = eskf.getAttitude().to_euler();
    ASSERT_TRUE(fabsf(euler_before.x) > 0.05f);   // meaningfully rolled off level

    eskf.holdAttitudeCorrection(false);   // Resume

    // Clean, level accelerometer samples (gyro back to zero) for up to the
    // plan's "1〜3s" recovery-time budget (§5 ESKF実装の特性).
    // 水平の清浄な加速度計サンプル（ジャイロはゼロに戻す）を、計画の「復帰
    // 時間1〜3s」（§5 ESKF実装の特性）の範囲で与える。
    const sf::math::Vec3 zero_gyro(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 2000; i++) {                        // 5s @ 400Hz
        eskf.predict(level_accel, zero_gyro, 0.0025f);
        eskf.updateAccelAttitude(level_accel);
    }

    auto euler_after = eskf.getAttitude().to_euler();
    ASSERT_NEAR(euler_after.x, 0.0f, 0.05f);
    ASSERT_NEAR(euler_after.y, 0.0f, 0.05f);
}

// =============================================================================
// (3) Resume re-acquires ToF after an altitude drift the plan calls the
//     boundary case (§3.6, "高度損失0.3〜0.5mは境界上"); without Resume the
//     absolute-innovation gate keeps rejecting the same ToF forever.
// (3) Resume は計画が境界事例と呼ぶ高度ドリフト（§3.6「高度損失0.3〜0.5mは
//     境界上」）の後に ToF を再取り込みする。Resume 無しでは絶対値イノベーション
//     判定が同じ ToF を棄却し続ける。
// =============================================================================

TEST(resume_reacquires_tof_after_altitude_drift)
{
    sf::EskfConfig cfg = quietConfig();
    cfg.use_tof = true;
    // Narrow the gate for this test so the scenario's ~0.4m estimate/ToF gap
    // deterministically exceeds it, independent of the production default
    // (0.5m) — the mechanism under test (gate suspension on Resume) is the
    // same updateToF() code path either way.
    // このテスト用にゲートを狭め、シナリオの ~0.4m の推定値/ToF ギャップが
    // 本番既定値（0.5m）に関わらず確実に判定を超えるようにする — テスト対象
    // の機構（Resume によるゲート一時停止）はどちらの値でも同じ updateToF()
    // コード経路。
    cfg.tof_innov_gate = 0.35f;

    sf::EskfCore eskf;
    eskf.init(cfg);
    // This test is about the POS_Z/ToF gate mechanism only — freeze the accel
    // bias so it cannot pick up the ToF-update cross-covariance (predict()
    // couples vel<->ba) and leak into the vertical channel as a phantom net
    // acceleration during the "predict-only" flip window below.
    // このテストは POS_Z/ToF 判定の機構だけが対象 — 加速度バイアスをフリーズし、
    // ToF 更新のクロス共分散（predict() が vel<->ba を結合する）を拾って、下の
    // 「predict のみ」の宙返り窓中に鉛直チャネルへ見かけの正味加速度として
    // 漏れ込まないようにする。
    eskf.setFreezeAccelBias(true);

    // Converge at 1.0m height (NED pos_z = -1.0) before the flip.
    // 宙返り前に高度1.0m（NED pos_z = -1.0）へ収束させる。
    convergeTofHeight(eskf, 1.0f, 0.3f, 60);
    ASSERT_NEAR(-eskf.getPosition().z, 1.0f, 0.05f);

    // Flip window: hold attitude correction; ToF stalls for 0.4s (in the real
    // system this is the tilt gate rejecting it — modeled here by simply not
    // calling updateToF). predict-only with level accel leaves the ESTIMATE
    // parked at 1.0m while the plan's worst case has the TRUE height drop
    // under it to 0.6m.
    // 宙返り窓: 姿勢補正をホールド。ToF は0.4秒停滞（実系では傾き判定による
    // 棄却。ここでは単に updateToF を呼ばないことで模す）。水平加速度での
    // predict のみは「推定値」を1.0mに留め置くが、計画の最悪ケースでは
    // その下で「真値」高度が0.6mまで下がっている。
    eskf.holdAttitudeCorrection(true);
    const sf::math::Vec3 level_accel(0.0f, 0.0f, -sf::math::kGravity);
    const sf::math::Vec3 zero_gyro(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 160; i++) {   // 0.4s @ 400Hz
        eskf.predict(level_accel, zero_gyro, 0.0025f);
    }

    // --- (a) WITHOUT Resume: the gate keeps rejecting the true ToF forever. ---
    // --- (a) Resume 無し: 判定が真の ToF を棄却し続ける。 ---
    sf::EskfCore no_resume = eskf;   // snapshot state at the moment the window ends
    for (int i = 0; i < 40; i++) {   // 0.1s worth of ToF samples
        no_resume.updateToF(0.6f);
    }
    ASSERT_NEAR(-no_resume.getPosition().z, 1.0f, 0.05f);   // unchanged: still rejected

    // --- (b) WITH Resume: the gate is suspended for exactly the next sample. ---
    // --- (b) Resume あり: 判定は次のサンプルだけ一時停止される。 ---
    eskf.holdAttitudeCorrection(false);   // Resume
    for (int i = 0; i < 40; i++) {        // <= 0.1s of predict+ToF cycles
        eskf.predict(level_accel, zero_gyro, 0.0025f);
        eskf.updateToF(0.6f);
    }
    ASSERT_NEAR(-eskf.getPosition().z, 0.6f, 0.1f);
}

// A stuck suspension must not disable the gate forever: if no ToF is ever
// accepted, cfg_.tof_reacquire_timeout_s restores the gate on its own.
// 判定停止の永久化防止: ToF が一度も採用されなければ、cfg_.tof_reacquire_timeout_s
// が自動的に判定を復帰させる。
TEST(resume_tof_suspension_times_out)
{
    sf::EskfConfig cfg = quietConfig();
    cfg.use_tof = true;
    cfg.tof_innov_gate = 0.1f;             // deliberately tight — the probe below always misses
    cfg.tof_reacquire_timeout_s = 0.2f;    // short timeout for a fast test

    sf::EskfCore eskf;
    eskf.init(cfg);
    eskf.setFreezeAccelBias(true);   // isolate POS_Z from bias cross-covariance (see test 3)
    convergeTofHeight(eskf, 1.0f, 0.08f, 20);
    eskf.holdAttitudeCorrection(false);   // Resume — suspends the gate

    const sf::math::Vec3 level_accel(0.0f, 0.0f, -sf::math::kGravity);
    const sf::math::Vec3 zero_gyro(0.0f, 0.0f, 0.0f);

    // Advance predict() WITHOUT ever calling updateToF, past the timeout.
    // updateToF を一度も呼ばずに、タイムアウトを超えて predict() だけ進める。
    for (int i = 0; i < 120; i++) {   // 0.3s @ 400Hz > tof_reacquire_timeout_s (0.2s)
        eskf.predict(level_accel, zero_gyro, 0.0025f);
    }

    // The gate must be back in force: a ToF far outside it (>0.1m from the
    // converged 1.0m) is rejected instead of being force-accepted.
    // 判定は復帰しているはず: 収束済み1.0mから0.1mを超えて外れたToFは、
    // 強制採用されず棄却される。
    eskf.updateToF(2.0f);
    ASSERT_NEAR(-eskf.getPosition().z, 1.0f, 0.05f);   // still 1.0m — the 2.0m sample was rejected
}

int main()
{
    printf("=== ESKF Flip Hold/Resume Unit Tests ===\n\n");

    printf("[flip hold/resume]\n");
    run_hold_blocks_accel_attitude_correction();
    run_resume_reconverges_attitude();
    run_resume_reacquires_tof_after_altitude_drift();
    run_resume_tof_suspension_times_out();

    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           tests_passed, tests_run, tests_failed);

    return tests_failed > 0 ? 1 : 0;
}
