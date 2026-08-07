/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_state_manager.cpp
 * @brief Deterministic unit tests for StateManager and Failsafe
 *        StateManager と Failsafe の決定論ユニットテスト
 *
 * Runs on host PC against the UNMODIFIED firmware sources
 * (state_manager.cpp, failsafe.cpp) compiled through the simulator/sils/compat
 * shim. Covers every transition edge, every guard rejection, every AlertType
 * branch, callback ordering, and Failsafe thresholds — the state-transition
 * coverage that the integrated SILS alone cannot guarantee.
 *
 * ホストPCで無改変のファームソース(state_manager.cpp, failsafe.cpp)を
 * simulator/sils/compat シム経由でコンパイルして実行する。全遷移エッジ・
 * 全ガード拒否・全AlertType分岐・コールバック順序・Failsafe閾値を網羅 —
 * 統合SILS単独では保証できない状態遷移カバレッジを担保する。
 *
 * @design requirements.md §10 — Unit testing on PC                    [--]
 * @design architecture.md §2 — Sole transition owner                  [--]
 */

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <vector>
#include <string>

#include "topics.hpp"
#include "state_manager.hpp"
#include "failsafe.hpp"

using namespace sf;

// =============================================================================
// Minimal test framework (mirrors test_main.cpp)
// 最小テストフレームワーク（test_main.cpp と同じ）
// =============================================================================

static int tests_run = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define TEST(name) \
    static void test_##name(); \
    static void run_##name() { \
        tests_run++; \
        printf("  [TEST] %-40s ", #name); \
        try { test_##name(); tests_passed++; printf("PASS\n"); } \
        catch (...) { tests_failed++; printf("FAIL\n"); } \
    } \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    if (!(cond)) { \
        printf("FAIL: %s:%d: condition false\n", __FILE__, __LINE__); \
        throw 1; \
    }

// =============================================================================
// Helpers
// ヘルパー
// =============================================================================

static constexpr float G = 9.80665f;
static constexpr float DEG2RAD = 3.14159265358979f / 180.0f;

/// Drive a freshly-initialized StateManager all the way to FLYING.
/// 初期化直後の StateManager を FLYING まで遷移させる。
static void toFlying(StateManager& sm)
{
    sm.init();
    sm.forceIdle();           // INIT → IDLE_GROUND
    sm.requestArm();          // → ARMED_GROUND
    sm.notifyTakeoff();       // → TAKEOFF
    sm.notifyTakeoffComplete();  // → FLYING
}

/// Build a SystemAlert message.
/// SystemAlert メッセージを生成する。
static SystemAlert mkAlert(AlertType t, AlertSeverity s)
{
    SystemAlert a = {};
    a.type = static_cast<uint8_t>(t);
    a.severity = static_cast<uint8_t>(s);
    a.timestamp = 0;
    return a;
}

/// Publish an IMU sample (accel in m/s², gyro in rad/s).
/// IMUサンプルを発行する（加速度 m/s²、ジャイロ rad/s）。
static void publishImu(float ax, float ay, float az, float gx, float gy, float gz)
{
    ImuData imu = {};
    imu.accel[0] = ax; imu.accel[1] = ay; imu.accel[2] = az;
    imu.gyro[0] = gx;  imu.gyro[1] = gy;  imu.gyro[2] = gz;
    imu.temperature = 25.0f;
    imu.timestamp = 0;
    sensor_imu.publish(imu);
}

/// Publish a battery voltage sample.
/// バッテリー電圧サンプルを発行する。
static void publishVoltage(float v)
{
    PowerData p = {};
    p.voltage = v;
    sensor_power.publish(p);
}

// Callback recording (cleared per test)
// コールバック記録（テスト毎にクリア）
static std::vector<std::string> g_cb_log;

// =============================================================================
// A. Normal transitions (11)
// A. 正常遷移（11）
// =============================================================================

TEST(a01_init_state) {
    topics_init();
    StateManager sm; sm.init();
    ASSERT_TRUE(sm.getState() == FlightState::INIT);
    ASSERT_TRUE(sm.getMode() == FlightMode::STABILIZE);
    ASSERT_TRUE(!sm.isArmed());
}

TEST(a02_force_idle) {
    topics_init();
    StateManager sm; sm.init();
    sm.forceIdle();
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(a03_arm) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    bool ok = sm.requestArm();
    ASSERT_TRUE(ok);
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
    ASSERT_TRUE(sm.isArmed());
}

TEST(a04_takeoff) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.notifyTakeoff();
    ASSERT_TRUE(sm.getState() == FlightState::TAKEOFF);
}

TEST(a05_takeoff_complete) {
    topics_init();
    StateManager sm; toFlying(sm);
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
}

TEST(a06_landing_request) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.notifyLandingRequest();
    ASSERT_TRUE(sm.getState() == FlightState::LANDING);
}

TEST(a07_landing_complete) {
    topics_init();
    StateManager sm; toFlying(sm); sm.notifyLandingRequest();
    sm.notifyLandingComplete();
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(a08_disarm) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    bool ok = sm.requestDisarm();
    ASSERT_TRUE(ok);
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(a09_soft_landing) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.notifySoftLanding();
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
}

TEST(a10_held) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    sm.notifyIdleGroundHeld(true);
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_HELD);
}

TEST(a11_unheld) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    sm.notifyIdleGroundHeld(true);   // → IDLE_HELD
    sm.notifyIdleGroundHeld(false);  // → IDLE_GROUND
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

// =============================================================================
// B. Guard rejections (12)
// B. ガード拒否（12）
// =============================================================================

TEST(b01_arm_from_init) {
    topics_init();
    StateManager sm; sm.init();
    bool ok = sm.requestArm();
    ASSERT_TRUE(!ok);
    ASSERT_TRUE(sm.getState() == FlightState::INIT);
}

TEST(b02_arm_from_flying) {
    topics_init();
    StateManager sm; toFlying(sm);
    bool ok = sm.requestArm();
    ASSERT_TRUE(!ok);
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
}

TEST(b03_double_arm) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    bool ok = sm.requestArm();  // already ARMED_GROUND
    ASSERT_TRUE(!ok);
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
}

TEST(b04_disarm_from_idle) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    bool ok = sm.requestDisarm();
    ASSERT_TRUE(!ok);
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(b05_disarm_from_flying) {
    topics_init();
    StateManager sm; toFlying(sm);
    bool ok = sm.requestDisarm();
    ASSERT_TRUE(!ok);
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
}

TEST(b06_takeoff_from_idle) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    sm.notifyTakeoff();  // guard: ARMED_GROUND only
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(b07_takeoff_complete_from_armed) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.notifyTakeoffComplete();  // guard: TAKEOFF only
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
}

TEST(b08_landing_request_from_armed) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.notifyLandingRequest();  // guard: FLYING only
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
}

TEST(b09_landing_complete_from_flying) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.notifyLandingComplete();  // guard: LANDING only
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
}

TEST(b10_soft_landing_from_armed) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.notifySoftLanding();  // guard: FLYING only
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);
}

TEST(b11_mode_change_not_flying) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();
    bool ok1 = sm.requestModeChange(FlightMode::ALT_HOLD);  // IDLE_GROUND
    sm.requestArm();
    bool ok2 = sm.requestModeChange(FlightMode::ALT_HOLD);  // ARMED_GROUND
    ASSERT_TRUE(!ok1 && !ok2);
}

TEST(b12_held_from_flying) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.notifyIdleGroundHeld(true);  // guard: IDLE_GROUND/IDLE_HELD only
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
}

// =============================================================================
// C. Mode change within FLYING (3)
// C. FLYING内モード変更（3）
// =============================================================================

TEST(c01_mode_change_alt_hold) {
    topics_init();
    StateManager sm; toFlying(sm);
    bool ok = sm.requestModeChange(FlightMode::ALT_HOLD);
    ASSERT_TRUE(ok);
    ASSERT_TRUE(sm.getMode() == FlightMode::ALT_HOLD);
}

TEST(c02_mode_change_idempotent) {
    topics_init();
    StateManager sm; toFlying(sm);  // mode = STABILIZE
    int fired = 0;
    sm.onModeChange([&fired](FlightMode, FlightMode){ fired++; });
    bool ok = sm.requestModeChange(FlightMode::STABILIZE);  // same mode
    ASSERT_TRUE(ok);
    ASSERT_TRUE(fired == 0);  // no callback for same mode
}

TEST(c03_mode_change_all) {
    topics_init();
    StateManager sm; toFlying(sm);
    ASSERT_TRUE(sm.requestModeChange(FlightMode::ACRO) && sm.getMode() == FlightMode::ACRO);
    ASSERT_TRUE(sm.requestModeChange(FlightMode::STABILIZE) && sm.getMode() == FlightMode::STABILIZE);
    ASSERT_TRUE(sm.requestModeChange(FlightMode::ALT_HOLD) && sm.getMode() == FlightMode::ALT_HOLD);
    ASSERT_TRUE(sm.requestModeChange(FlightMode::POS_HOLD) && sm.getMode() == FlightMode::POS_HOLD);
}

// =============================================================================
// D. All AlertType branches in handleAlert (10)
// D. handleAlert の全AlertType分岐（10）
// =============================================================================

TEST(d01_impact_airborne) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.handleAlert(mkAlert(AlertType::IMPACT, AlertSeverity::EMERGENCY));
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(d02_impact_ground) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();  // ARMED_GROUND (not airborne)
    sm.handleAlert(mkAlert(AlertType::IMPACT, AlertSeverity::EMERGENCY));
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);  // unchanged
}

TEST(d03_gyro_anomaly_airborne) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.handleAlert(mkAlert(AlertType::GYRO_ANOMALY, AlertSeverity::CRITICAL));
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

TEST(d04_gyro_anomaly_ground) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.handleAlert(mkAlert(AlertType::GYRO_ANOMALY, AlertSeverity::CRITICAL));
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);  // unchanged
}

TEST(d05_comm_lost_flying) {
    topics_init();
    StateManager sm; toFlying(sm);
    // COMM_LOST while FLYING arms the hover-grace timer but does NOT land immediately
    // (requirements §9: hover hold 3 s → auto landing). mkAlert stamps timestamp=0, so
    // update(now) crosses the 3 s grace at now == 3'000'000 us. update() drives the
    // deferred transition because the failsafe raises COMM_LOST only once (rising edge).
    // COMM_LOST は FLYING 中にホバー猶予タイマを起動するが即着陸しない（要件§9: ホバー
    // 維持3秒→自動着陸）。mkAlert は timestamp=0 ゆえ update(now) は now==3'000'000us で
    // 3秒猶予を越える。failsafe は COMM_LOST を1回しか出さないので遅延遷移は update() が駆動。
    sm.handleAlert(mkAlert(AlertType::COMM_LOST, AlertSeverity::CRITICAL));
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);   // still hovering, timer armed
    sm.update(1000000);                                  // +1 s: grace not yet elapsed
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);
    sm.update(3000000);                                  // +3 s: grace elapsed → LANDING
    ASSERT_TRUE(sm.getState() == FlightState::LANDING);
}

TEST(d06_comm_lost_not_flying) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm(); sm.notifyTakeoff();  // TAKEOFF
    sm.handleAlert(mkAlert(AlertType::COMM_LOST, AlertSeverity::CRITICAL));
    sm.update(3000000);                                  // even past the grace window
    ASSERT_TRUE(sm.getState() == FlightState::TAKEOFF);  // COMM_LOST acts only in FLYING
}

TEST(d07_low_battery) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.handleAlert(mkAlert(AlertType::LOW_BATTERY, AlertSeverity::WARNING));
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);  // warning only, no state change
}

TEST(d08_usb_power) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle(); sm.requestArm();
    sm.handleAlert(mkAlert(AlertType::USB_POWER, AlertSeverity::WARNING));
    ASSERT_TRUE(sm.getState() == FlightState::ARMED_GROUND);  // unchanged
}

TEST(d09_eskf_diverged) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.handleAlert(mkAlert(AlertType::ESKF_DIVERGED, AlertSeverity::WARNING));
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);  // reset only, no state change
}

TEST(d10_alert_none) {
    topics_init();
    StateManager sm; toFlying(sm);
    sm.handleAlert(mkAlert(AlertType::NONE, AlertSeverity::INFO));
    ASSERT_TRUE(sm.getState() == FlightState::FLYING);  // unchanged
}

// =============================================================================
// E. Callback ordering and registration (5)
// E. コールバック順序・登録（5）
// =============================================================================

TEST(e01_exit_enter_order) {
    topics_init();
    g_cb_log.clear();
    StateManager sm; sm.init();
    sm.onExit([](FlightState s){ g_cb_log.push_back(std::string("exit:") + flightStateName(s)); });
    sm.onEnter([](FlightState s){ g_cb_log.push_back(std::string("enter:") + flightStateName(s)); });
    sm.forceIdle();  // INIT → IDLE_GROUND
    ASSERT_TRUE(g_cb_log.size() == 2);
    ASSERT_TRUE(g_cb_log[0] == "exit:INIT");
    ASSERT_TRUE(g_cb_log[1] == "enter:IDLE_GROUND");
}

TEST(e02_same_state_no_callback) {
    topics_init();
    StateManager sm; sm.init(); sm.forceIdle();  // IDLE_GROUND
    int count = 0;
    sm.onEnter([&count](FlightState){ count++; });
    sm.forceIdle();  // IDLE_GROUND → IDLE_GROUND (old == new)
    ASSERT_TRUE(count == 0);
}

TEST(e03_mode_change_callback) {
    topics_init();
    StateManager sm; toFlying(sm);
    bool fired = false;
    FlightMode old_m = FlightMode::ACRO, new_m = FlightMode::ACRO;
    sm.onModeChange([&](FlightMode o, FlightMode n){ fired = true; old_m = o; new_m = n; });
    sm.requestModeChange(FlightMode::ALT_HOLD);
    ASSERT_TRUE(fired);
    ASSERT_TRUE(old_m == FlightMode::STABILIZE && new_m == FlightMode::ALT_HOLD);
}

TEST(e04_multi_callback_order) {
    topics_init();
    StateManager sm; sm.init();
    std::vector<int> order;
    sm.onEnter([&order](FlightState){ order.push_back(1); });
    sm.onEnter([&order](FlightState){ order.push_back(2); });
    sm.onEnter([&order](FlightState){ order.push_back(3); });
    sm.forceIdle();
    ASSERT_TRUE(order.size() == 3 && order[0] == 1 && order[1] == 2 && order[2] == 3);
}

TEST(e05_max_callbacks_limit) {
    topics_init();
    StateManager sm; sm.init();
    int count = 0;
    for (int i = 0; i < 9; i++) {  // 9 registered, only MAX_CALLBACKS(8) kept
        sm.onEnter([&count](FlightState){ count++; });
    }
    sm.forceIdle();
    ASSERT_TRUE(count == 8);
}

// =============================================================================
// F. Failsafe thresholds (5)
// F. Failsafe 閾値（5）
// =============================================================================

TEST(f01_impact_latched) {
    topics_init();
    publishImu(0, 0, 5.0f * G, 0, 0, 0);  // 5G, no gyro
    Failsafe fs; fs.init();
    fs.update();
    SystemAlert a;
    ASSERT_TRUE(system_alert.read(a));
    ASSERT_TRUE(a.type == static_cast<uint8_t>(AlertType::IMPACT));
    ASSERT_TRUE(a.severity == static_cast<uint8_t>(AlertSeverity::EMERGENCY));
    // Latch: a second high-G update must NOT re-fire
    // ラッチ: 2回目の高Gでは再発火しない
    publishImu(0, 0, 5.0f * G, 0, 0, 0);
    fs.update();
    SystemAlert b;
    ASSERT_TRUE(!system_alert.read(b));
}

TEST(f02_gyro_anomaly) {
    topics_init();
    publishImu(0, 0, G, 1100.0f * DEG2RAD, 0, 0);  // 1G accel, 1100 dps roll
    Failsafe fs; fs.init();
    fs.update();
    SystemAlert a;
    ASSERT_TRUE(system_alert.read(a));
    ASSERT_TRUE(a.type == static_cast<uint8_t>(AlertType::GYRO_ANOMALY));
    ASSERT_TRUE(a.severity == static_cast<uint8_t>(AlertSeverity::CRITICAL));
}

TEST(f03_battery_warning) {
    topics_init();
    publishVoltage(3.2f);  // below low_battery_v(3.3), above critical(3.0)
    Failsafe fs; fs.init();
    fs.update();
    SystemAlert a;
    ASSERT_TRUE(system_alert.read(a));
    ASSERT_TRUE(a.type == static_cast<uint8_t>(AlertType::LOW_BATTERY));
    ASSERT_TRUE(a.severity == static_cast<uint8_t>(AlertSeverity::WARNING));
}

TEST(f04_battery_critical) {
    topics_init();
    publishVoltage(2.9f);  // below critical_battery_v(3.0)
    Failsafe fs; fs.init();
    fs.update();
    SystemAlert a;
    ASSERT_TRUE(system_alert.read(a));
    ASSERT_TRUE(a.type == static_cast<uint8_t>(AlertType::LOW_BATTERY));
    ASSERT_TRUE(a.severity == static_cast<uint8_t>(AlertSeverity::EMERGENCY));
}

TEST(f05_battery_disconnected) {
    topics_init();
    publishVoltage(0.0f);  // <= 0.1 → treated as no battery, must not fire
    Failsafe fs; fs.init();
    fs.update();
    SystemAlert a;
    ASSERT_TRUE(!system_alert.read(a));
}

// =============================================================================
// G. End-to-end: Failsafe → StateManager (1)
// G. エンドツーエンド: Failsafe → StateManager（1）
// =============================================================================

TEST(g01_impact_to_idle_e2e) {
    topics_init();
    StateManager sm; toFlying(sm);          // FLYING
    Failsafe fs; fs.init();
    publishImu(0, 0, 6.0f * G, 0, 0, 0);    // impact
    fs.update();                            // → system_alert
    SystemAlert a;
    ASSERT_TRUE(system_alert.read(a));      // consume like state_task does
    sm.handleAlert(a);                      // → emergency IDLE
    ASSERT_TRUE(sm.getState() == FlightState::IDLE_GROUND);
}

// =============================================================================
// Runner
// ランナー
// =============================================================================

int main()
{
    printf("=== StateManager / Failsafe unit tests ===\n");

    // A. Normal transitions
    run_a01_init_state(); run_a02_force_idle(); run_a03_arm(); run_a04_takeoff();
    run_a05_takeoff_complete(); run_a06_landing_request(); run_a07_landing_complete();
    run_a08_disarm(); run_a09_soft_landing(); run_a10_held(); run_a11_unheld();

    // B. Guard rejections
    run_b01_arm_from_init(); run_b02_arm_from_flying(); run_b03_double_arm();
    run_b04_disarm_from_idle(); run_b05_disarm_from_flying(); run_b06_takeoff_from_idle();
    run_b07_takeoff_complete_from_armed(); run_b08_landing_request_from_armed();
    run_b09_landing_complete_from_flying(); run_b10_soft_landing_from_armed();
    run_b11_mode_change_not_flying(); run_b12_held_from_flying();

    // C. Mode change
    run_c01_mode_change_alt_hold(); run_c02_mode_change_idempotent(); run_c03_mode_change_all();

    // D. Alert branches
    run_d01_impact_airborne(); run_d02_impact_ground(); run_d03_gyro_anomaly_airborne();
    run_d04_gyro_anomaly_ground(); run_d05_comm_lost_flying(); run_d06_comm_lost_not_flying();
    run_d07_low_battery(); run_d08_usb_power(); run_d09_eskf_diverged(); run_d10_alert_none();

    // E. Callbacks
    run_e01_exit_enter_order(); run_e02_same_state_no_callback(); run_e03_mode_change_callback();
    run_e04_multi_callback_order(); run_e05_max_callbacks_limit();

    // F. Failsafe thresholds
    run_f01_impact_latched(); run_f02_gyro_anomaly(); run_f03_battery_warning();
    run_f04_battery_critical(); run_f05_battery_disconnected();

    // G. End-to-end
    run_g01_impact_to_idle_e2e();

    printf("\n=== %d run, %d passed, %d failed ===\n",
           tests_run, tests_passed, tests_failed);
    return tests_failed == 0 ? 0 : 1;
}
