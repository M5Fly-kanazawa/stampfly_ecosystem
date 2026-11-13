/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_failsafe.cpp
 * @brief Host tests of the Failsafe thresholds: comm timeout, battery voltages, impact, gyro anomaly, ToF silence
 *        Failsafe の閾値のホスト試験: 通信タイムアウト・電池電圧・衝撃・ジャイロ異常・ToF 出力停止
 *
 * failsafe.cpp is compiled unmodified. The topics (sf_core/params.cpp) and the clock, log and NVS shims come from
 * simulator/sils/compat, the same host shims the SILS uses. Time is virtual. Every case checks one threshold at the
 * boundary, just below it and just above it, or one timing rule (latch, hold, re-raise).
 * The expected values are written out from requirements.md §9 and from the constants of FailsafeConfig, not
 * recomputed with the production code.
 *
 * What sf_failsafe does NOT check: the tilt / attitude limits are not Failsafe thresholds. The attitude-versus-gravity
 * check on the ground is TiltConsistencyMonitor (test_tilt_consistency.cpp); the ESKF divergence reset and the
 * 3 s hover hold after COMM_LOST belong to the StateManager (test_state_manager.cpp).
 * sf_failsafe が検査「しない」もの: 傾き・姿勢の限界は Failsafe の閾値ではない。地上での姿勢と重力の不一致は
 * TiltConsistencyMonitor（test_tilt_consistency.cpp）、ESKF 発散のリセットと COMM_LOST 後の 3 秒のホバー猶予は
 * StateManager（test_state_manager.cpp）が担う。
 *
 * @design requirements.md §9 — Safety requirements                    [OK]
 * @design architecture.md §4 — Failsafe subsystem, R16 level semantics [OK]
 */

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <esp_log.h>
#include <esp_timer.h>

#include "failsafe.hpp"
#include "flight_limits.hpp"
#include "topics.hpp"

using namespace sf;

// The SILS semaphore and queue shims call vTaskDelay() only to wait for a lock another task holds. The tests run in one thread, so
// the wait never happens; the symbol is defined here instead of linking the SILS scheduler.
// SILS のセマフォ・キューのシムは、他のタスクが持つロックを待つときだけ vTaskDelay() を呼ぶ。試験は 1 スレッドなので待つことは
// 起きない。SILS のスケジューラをリンクする代わりに、シンボルだけをここで定義する。
extern "C" void vTaskDelay(TickType_t) {}

// =============================================================================
// Test framework (self-registering TEST, own main)
// 試験の枠組み（自己登録の TEST、独自の main）
// =============================================================================

struct TestCase {
    const char* name;
    void (*body)();
};

static std::vector<TestCase>& registry()
{
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, void (*body)()) { registry().push_back({name, body}); }
};

struct AssertionFailure {};

#define TEST(name) \
    static void test_##name(); \
    static Registrar registrar_##name(#name, test_##name); \
    static void test_##name()

#define ASSERT_MSG(cond, ...) \
    do { \
        if (!(cond)) { \
            printf("\n      FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            printf(__VA_ARGS__); \
            printf("\n"); \
            throw AssertionFailure{}; \
        } \
    } while (0)

// =============================================================================
// Named constants
// 名前つき定数
// =============================================================================

static constexpr uint32_t kStartTimeUs = 1000000;      ///< first virtual time (a stamp of 0 means "never") / 仮想時間の開始（時刻印 0 は「一度も無い」）
static constexpr uint32_t kTickUs = 100000;            ///< Failsafe::update() period, 10 Hz [us] / Failsafe::update() の周期 10 Hz
static constexpr uint32_t kImuSamplePeriodUs = 2500;   ///< ImuAnomalyDetector::feedSample() period, 400 Hz [us] / 400 Hz の周期
static constexpr float kGravity = 9.80665f;            ///< [m/s^2] / 重力加速度
static constexpr float kDegToRad = 0.0174532925f;      ///< [rad/deg] / 度からラジアン

// requirements.md §9 / FailsafeConfig defaults. / 要件 §9・FailsafeConfig の既定値。
static constexpr uint32_t kCommTimeoutMs = 500;
static constexpr float kLowVoltageV = 3.4f;
static constexpr float kCriticalVoltageV = 3.0f;
static constexpr float kImpactG = 3.0f;
static constexpr float kGyroLimitDps = 800.0f;
static constexpr uint32_t kConsecutiveSamples = 2;
static constexpr uint32_t kGyroReraiseSamples = 400;   ///< one re-raise per second at 400 Hz / 400 Hz で毎秒 1 回
static constexpr float kNoReadingVoltageV = 0.1f;      ///< at or below this the battery reading counts as absent / 以下は読み値なし

static constexpr uint32_t kWarningReraiseUpdates = 50;     ///< BatteryAlertPolicy: about 5 s at 10 Hz / 10 Hz で約 5 秒
static constexpr uint32_t kEmergencyReraiseUpdates = 50;
static constexpr uint32_t kCommReraiseUpdates = 10;        ///< about once a second / 約 1 秒ごと
static constexpr uint32_t kTofReraiseUpdates = 10;         ///< about once a second / 約 1 秒ごと

// =============================================================================
// Fixture helpers
// 試験台の補助
// =============================================================================

static uint32_t nowUs()
{
    return static_cast<uint32_t>(esp_timer_get_time());
}

/// All alerts of one type published since the last call, oldest first.
/// 前回の呼び出し以降に発行された、ある種別のアラート（古い順）。
static std::vector<SystemAlert> drainAlerts(AlertType wanted)
{
    std::vector<SystemAlert> found;
    SystemAlert alert = {};
    while (system_alert.read(alert)) {
        if (alert.type == static_cast<uint8_t>(wanted)) {
            found.push_back(alert);
        }
    }
    return found;
}

static void publishVoltage(float voltage)
{
    PowerData power = {};
    power.voltage = voltage;
    sensor_power.publish(power);
}

static void publishSetpointStampedNow()
{
    CommandSetpoint setpoint = {};
    setpoint.timestamp = nowUs();
    command_setpoint.publish(setpoint);
}

static void publishFlightState(FlightState state)
{
    SystemMode mode = {};
    mode.state = static_cast<uint8_t>(state);
    system_mode.publish(mode);
}

/// A ToF sample stamped now, with the estimator-side facts given.
/// 今の時刻印を持つ ToF サンプルと、与えられた推定器側の事実。
static void publishToFSampleNow(bool altitude_reliable, bool invalid_stuck)
{
    SensorSnapshot snapshot = {};
    snapshot.tof_timestamp = nowUs();
    sensor_snapshot.publish(snapshot);
    ToFHealth health = {};
    health.altitude_reliable = altitude_reliable;
    health.invalid_stuck = invalid_stuck;
    tof_health.publish(health);
}

/// Topics and clock reset; the pack is charged, the link is up, the ToF is alive, the craft is not armed.
/// トピックと時計を初期化する。電池は充電済み、リンクは生きている、ToF は動作中、機体は ARM されていない。
struct Fixture {
    Failsafe failsafe;

    Fixture()
    {
        topics_init();
        sils::compat::set_virtual_time_us(kStartTimeUs);
        publishVoltage(4.0f);
        publishSetpointStampedNow();
        publishToFSampleNow(true, false);
        publishFlightState(FlightState::IDLE_GROUND);
        failsafe.init();
        failsafe.update();
        drainAlerts(AlertType::NONE);   // empties the queue / キューを空にする
    }

    /// Move the virtual time forward and run one Failsafe cycle.
    /// 仮想時間を進めて Failsafe の 1 周期を実行する。
    void stepUs(uint32_t duration_us)
    {
        sils::compat::advance_virtual_time_us(duration_us);
        failsafe.update();
    }

    /// Run n 10 Hz cycles, the link and the ToF kept alive, and count the alerts of one type.
    /// 10 Hz の周期を n 回実行し（リンクと ToF は生かしたまま）、ある種別のアラートを数える。
    int countAlertsOverUpdates(AlertType type, uint32_t updates)
    {
        int count = 0;
        for (uint32_t i = 0; i < updates; ++i) {
            sils::compat::advance_virtual_time_us(kTickUs);
            publishSetpointStampedNow();
            publishToFSampleNow(true, false);
            failsafe.update();
            count += static_cast<int>(drainAlerts(type).size());
        }
        return count;
    }
};

// =============================================================================
// Comm timeout (requirements §9: 500 ms)
// 通信タイムアウト（要件 §9: 500 ms）
// =============================================================================

TEST(comm_timeout_boundary_is_500_ms_counted_in_whole_milliseconds)
{
    // Age 499 ms and 500 ms (also 500.999 ms: whole milliseconds) keep the link; 501 ms loses it.
    // 経過 499 ms・500 ms（500.999 ms も: ミリ秒の整数部）はリンク維持、501 ms で喪失。
    struct Case { uint32_t age_us; bool lost; };
    const Case cases[] = {
        {499000, false}, {500000, false}, {500999, false}, {501000, true},
    };
    for (const Case& c : cases) {
        Fixture fixture;
        publishSetpointStampedNow();
        fixture.failsafe.update();
        drainAlerts(AlertType::NONE);
        fixture.stepUs(c.age_us);
        const auto alerts = drainAlerts(AlertType::COMM_LOST);
        ASSERT_MSG(alerts.size() == (c.lost ? 1u : 0u), "age %u us: %zu COMM_LOST alerts, expected %d", c.age_us,
                   alerts.size(), c.lost ? 1 : 0);
        if (c.lost) {
            ASSERT_MSG(alerts[0].severity == static_cast<uint8_t>(AlertSeverity::CRITICAL), "COMM_LOST severity %u",
                       alerts[0].severity);
        }
    }
}

TEST(comm_timeout_is_not_judged_before_the_first_packet)
{
    // A stamp that has never been non-zero means no packet yet (boot, bench): no alert however long it lasts.
    // 時刻印が一度も 0 以外になっていなければ未受信（起動時・ベンチ）: いくら続いても発報しない。
    topics_init();
    sils::compat::set_virtual_time_us(kStartTimeUs);
    publishVoltage(4.0f);
    Failsafe failsafe;
    failsafe.init();
    for (int i = 0; i < 100; ++i) {
        sils::compat::advance_virtual_time_us(kTickUs);
        failsafe.update();
    }
    ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).empty(), "COMM_LOST before any packet");
}

TEST(comm_lost_is_raised_once_on_the_edge_and_re_raised_every_second)
{
    Fixture fixture;
    fixture.stepUs((kCommTimeoutMs + 1) * 1000);
    ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).size() == 1, "no alert on the rising edge");
    // The next kCommReraiseUpdates - 1 cycles stay quiet, the kCommReraiseUpdates-th re-raises.
    // 次の kCommReraiseUpdates - 1 周期は静か、kCommReraiseUpdates 周期目で再発報。
    for (uint32_t i = 1; i < kCommReraiseUpdates; ++i) {
        fixture.stepUs(kTickUs);
        ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).empty(), "re-raised early at cycle %u", i);
    }
    fixture.stepUs(kTickUs);
    ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).size() == 1, "no re-raise at cycle %u", kCommReraiseUpdates);
}

TEST(comm_lost_clears_on_a_new_packet_and_a_second_loss_raises_at_once)
{
    Fixture fixture;
    fixture.stepUs((kCommTimeoutMs + 1) * 1000);
    drainAlerts(AlertType::COMM_LOST);
    // Link back: the re-raise divider restarts and nothing is raised while the link is up.
    // リンク復帰: 再発報の分周が戻り、リンクが生きている間は発報しない。
    fixture.stepUs(kTickUs);
    publishSetpointStampedNow();
    fixture.failsafe.update();
    ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).empty(), "alert while the link is up");
    fixture.stepUs((kCommTimeoutMs + 1) * 1000);
    ASSERT_MSG(drainAlerts(AlertType::COMM_LOST).size() == 1, "second loss is not raised on its edge");
}

// =============================================================================
// Battery (requirements §9: low 3.4 V warning only, critical 3.0 V emergency)
// 電池（要件 §9: 3.4 V 警告のみ、3.0 V 危険）
// =============================================================================

/// Publish a voltage, run one cycle and return how many alerts of each severity LOW_BATTERY raised.
/// 電圧を発行して 1 周期実行し、LOW_BATTERY の重要度別の発報数を返す。
struct BatteryResult { int warnings; int emergencies; };

static BatteryResult runBatteryCycle(Fixture& fixture, float voltage)
{
    publishVoltage(voltage);
    fixture.stepUs(kTickUs);
    publishSetpointStampedNow();   // keep the link alive in the next cycle / 次の周期でもリンクを生かす
    BatteryResult result = {0, 0};
    for (const SystemAlert& alert : drainAlerts(AlertType::LOW_BATTERY)) {
        result.warnings += (alert.severity == static_cast<uint8_t>(AlertSeverity::WARNING)) ? 1 : 0;
        result.emergencies += (alert.severity == static_cast<uint8_t>(AlertSeverity::EMERGENCY)) ? 1 : 0;
    }
    return result;
}

TEST(low_battery_warning_threshold_is_strictly_below_3_4_volts)
{
    // Code: voltage < low. requirements §9 writes "<= 3.4 V": at exactly 3.4 V the code stays silent.
    // コード: voltage < low。要件 §9 は「≤3.4V」: ちょうど 3.4 V ではコードは発報しない（要件との食い違い）。
    struct Case { float voltage; int warnings; };
    const Case cases[] = {{kLowVoltageV + 0.01f, 0}, {kLowVoltageV, 0}, {kLowVoltageV - 0.01f, 1}};
    for (const Case& c : cases) {
        Fixture fixture;
        const BatteryResult result = runBatteryCycle(fixture, c.voltage);
        ASSERT_MSG(result.warnings == c.warnings && result.emergencies == 0, "%.2f V: %d warnings %d emergencies",
                   c.voltage, result.warnings, result.emergencies);
    }
}

TEST(critical_battery_emergency_threshold_is_strictly_below_3_0_volts)
{
    // requirements §9: "< 3.0 V". At 3.0 V the pack is in the warning band; just below it is the emergency
    // (and no WARNING in the same cycle).
    // 要件 §9: 「<3.0V」。3.0 V は警告帯、それより下は非常（同じ周期に WARNING は出さない）。
    struct Case { float voltage; int warnings; int emergencies; };
    const Case cases[] = {
        {kCriticalVoltageV + 0.01f, 1, 0}, {kCriticalVoltageV, 1, 0}, {kCriticalVoltageV - 0.01f, 0, 1},
    };
    for (const Case& c : cases) {
        Fixture fixture;
        const BatteryResult result = runBatteryCycle(fixture, c.voltage);
        ASSERT_MSG(result.warnings == c.warnings && result.emergencies == c.emergencies,
                   "%.2f V: %d warnings %d emergencies, expected %d / %d", c.voltage, result.warnings,
                   result.emergencies, c.warnings, c.emergencies);
    }
}

TEST(battery_reading_of_0_1_volt_or_less_is_no_reading)
{
    Fixture fixture;
    BatteryResult result = runBatteryCycle(fixture, kNoReadingVoltageV);
    ASSERT_MSG(result.warnings == 0 && result.emergencies == 0, "alert on a no-reading value");
    result = runBatteryCycle(fixture, kNoReadingVoltageV + 0.01f);
    ASSERT_MSG(result.emergencies == 1, "a reading just above 0.1 V is a real (empty) pack");
}

TEST(battery_warning_is_latched_re_raised_every_50_updates_and_released_with_hysteresis)
{
    Fixture fixture;
    ASSERT_MSG(runBatteryCycle(fixture, kLowVoltageV - 0.1f).warnings == 1, "no warning on entry");
    int later_warnings = 0;
    for (uint32_t i = 1; i < kWarningReraiseUpdates; ++i) {
        later_warnings += runBatteryCycle(fixture, kLowVoltageV - 0.1f).warnings;
    }
    ASSERT_MSG(later_warnings == 0, "re-raised before %u updates", kWarningReraiseUpdates);
    ASSERT_MSG(runBatteryCycle(fixture, kLowVoltageV - 0.1f).warnings == 1, "no re-raise at update %u",
               kWarningReraiseUpdates);
    // 3.49 V is inside the 0.1 V recovery hysteresis: the latch stays, a dip below 3.4 V is silent.
    // 3.49 V は回復ヒステリシス 0.1 V の内側: ラッチは残り、3.4 V 未満へ戻っても静か。
    runBatteryCycle(fixture, kLowVoltageV + 0.09f);
    ASSERT_MSG(runBatteryCycle(fixture, kLowVoltageV - 0.1f).warnings == 0, "warned again inside the hysteresis");
    // 3.51 V releases the latch: the next dip warns at once.
    // 3.51 V でラッチが外れる: 次に下がるとすぐ警告する。
    runBatteryCycle(fixture, kLowVoltageV + 0.11f);
    ASSERT_MSG(runBatteryCycle(fixture, kLowVoltageV - 0.1f).warnings == 1, "no warning after the latch was released");
}

TEST(battery_emergency_is_raised_on_entry_and_re_raised_every_50_updates)
{
    Fixture fixture;
    ASSERT_MSG(runBatteryCycle(fixture, kCriticalVoltageV - 0.2f).emergencies == 1, "no emergency on entry");
    int later_emergencies = 0;
    for (uint32_t i = 1; i < kEmergencyReraiseUpdates; ++i) {
        later_emergencies += runBatteryCycle(fixture, kCriticalVoltageV - 0.2f).emergencies;
    }
    ASSERT_MSG(later_emergencies == 0, "re-raised before %u updates", kEmergencyReraiseUpdates);
    ASSERT_MSG(runBatteryCycle(fixture, kCriticalVoltageV - 0.2f).emergencies == 1, "no re-raise at update %u",
               kEmergencyReraiseUpdates);
}

// =============================================================================
// ToF silence (requirements §9: no new sample for 0.3 s, only while armed it raises the alert)
// ToF 出力停止（要件 §9: 新サンプルが 0.3 s 来ない。アラートは ARM 中だけ）
// =============================================================================

TEST(tof_silence_boundary_is_the_silence_window_and_the_status_is_published_every_update)
{
    // No new sample for window - 1 us: fresh. For exactly the window: silent (strict "<" for fresh).
    // 新サンプルが無い時間が window - 1 us: 健全。ちょうど window: 出力停止（健全は厳密な "<"）。
    const uint32_t window_us = limits::kTofSilenceUs;
    struct Case { uint32_t quiet_us; bool silent; };
    const Case cases[] = {{window_us - 1, false}, {window_us, true}, {window_us + 1, true}};
    for (const Case& c : cases) {
        Fixture fixture;
        fixture.stepUs(c.quiet_us);
        const FailsafeStatus status = failsafe_status.latest();
        ASSERT_MSG(status.tof_silent == c.silent, "quiet %u us: tof_silent=%d", c.quiet_us, status.tof_silent ? 1 : 0);
        ASSERT_MSG(status.timestamp == nowUs(), "failsafe_status is not stamped with this update");
        ASSERT_MSG(status.tof_reason == static_cast<uint8_t>(c.silent ? ToFSilenceReason::NoSample : ToFSilenceReason::None),
                   "quiet %u us: reason %u", c.quiet_us, status.tof_reason);
    }
}

TEST(tof_never_seen_counts_as_silent)
{
    topics_init();
    sils::compat::set_virtual_time_us(kStartTimeUs);
    publishVoltage(4.0f);
    Failsafe failsafe;
    failsafe.init();
    failsafe.update();
    ASSERT_MSG(failsafe_status.latest().tof_silent, "a sensor that never sampled is not silent");
}

TEST(tof_episode_reasons_follow_the_estimator_facts)
{
    // A fresh sample is not enough: only invalid samples while flying, or no re-acquisition, is still an episode.
    // 新しいサンプルがあるだけでは足りない: 飛行中に無効だけ・取り込み直し未了もエピソード。
    Fixture fixture;
    publishToFSampleNow(true, true);
    fixture.failsafe.update();
    ASSERT_MSG(failsafe_status.latest().tof_reason == static_cast<uint8_t>(ToFSilenceReason::InvalidStuck),
               "invalid_stuck reason %u", failsafe_status.latest().tof_reason);
    publishToFSampleNow(false, false);
    fixture.failsafe.update();
    ASSERT_MSG(failsafe_status.latest().tof_reason == static_cast<uint8_t>(ToFSilenceReason::Reacquiring),
               "reacquiring reason %u", failsafe_status.latest().tof_reason);
    publishToFSampleNow(true, false);
    fixture.failsafe.update();
    ASSERT_MSG(!failsafe_status.latest().tof_silent, "episode did not end after the re-acquisition");
}

TEST(tof_silent_alert_is_raised_only_while_armed_at_once_and_every_second)
{
    {   // Disarmed: the level is published, no alert. / DISARM 中: 水準は発行するがアラートは無し。
        Fixture fixture;
        fixture.stepUs(limits::kTofSilenceUs);
        ASSERT_MSG(failsafe_status.latest().tof_silent, "no silence level");
        ASSERT_MSG(drainAlerts(AlertType::TOF_SILENT).empty(), "alert while disarmed");
    }
    Fixture fixture;
    publishFlightState(FlightState::FLYING);
    fixture.stepUs(limits::kTofSilenceUs);
    ASSERT_MSG(drainAlerts(AlertType::TOF_SILENT).size() == 1, "no alert at the first armed silent update");
    for (uint32_t i = 1; i < kTofReraiseUpdates; ++i) {
        fixture.stepUs(kTickUs);
        publishSetpointStampedNow();
        ASSERT_MSG(drainAlerts(AlertType::TOF_SILENT).empty(), "re-raised early at cycle %u", i);
    }
    fixture.stepUs(kTickUs);
    ASSERT_MSG(drainAlerts(AlertType::TOF_SILENT).size() == 1, "no re-raise at cycle %u", kTofReraiseUpdates);
}

// =============================================================================
// IMU-rate checks (requirements §9: impact 3.0 G x 2, gyro 800 deg/s x 2)
// IMU レートの監視（要件 §9: 衝撃 3.0 G x 2 連続、角速度 800 deg/s x 2 連続）
// =============================================================================

struct ImuFixture {
    ImuAnomalyDetector detector;

    ImuFixture()
    {
        topics_init();
        sils::compat::set_virtual_time_us(kStartTimeUs);
        detector.init();
    }

    void feed(float accel_g, float gyro_dps, bool armed = true)
    {
        const float accel[3] = {0.0f, 0.0f, accel_g * kGravity};
        const float gyro[3] = {gyro_dps * kDegToRad, 0.0f, 0.0f};
        sils::compat::advance_virtual_time_us(kImuSamplePeriodUs);
        detector.feedSample(accel, gyro, armed);
    }

    /// Feed n samples and return how many alerts of one type were raised.
    /// n サンプル与え、ある種別のアラートの発報数を返す。
    int feedMany(uint32_t samples, float accel_g, float gyro_dps, AlertType type)
    {
        for (uint32_t i = 0; i < samples; ++i) {
            feed(accel_g, gyro_dps);
        }
        return static_cast<int>(drainAlerts(type).size());
    }
};

static constexpr float kOneG = 1.0f;       ///< resting craft / 静止した機体
static constexpr float kMargin = 0.01f;    ///< margin for "just below / above" [G] / 「直下・直上」の余白

TEST(impact_threshold_is_3_g_strictly_over)
{
    // Over for the two consecutive samples: just below and at the boundary stay silent, just above raises.
    // 連続 2 サンプル超過: 直下と境界ちょうどは静か、直上で発報。
    struct Case { float g; int alerts; };
    const Case cases[] = {{kImpactG - kMargin, 0}, {kImpactG, 0}, {kImpactG + kMargin, 1}};
    for (const Case& c : cases) {
        ImuFixture fixture;
        const int alerts = fixture.feedMany(kConsecutiveSamples, c.g, 0.0f, AlertType::IMPACT);
        ASSERT_MSG(alerts == c.alerts, "%.2f G: %d impact alerts, expected %d", c.g, alerts, c.alerts);
    }
}

TEST(impact_needs_two_consecutive_samples_and_is_latched_until_disarm)
{
    ImuFixture fixture;
    ASSERT_MSG(fixture.feedMany(1, 4.0f, 0.0f, AlertType::IMPACT) == 0, "one sample raised an alert");
    fixture.feed(kOneG, 0.0f);   // a sample below the threshold restarts the count / 閾値未満のサンプルで数え直し
    ASSERT_MSG(fixture.feedMany(1, 4.0f, 0.0f, AlertType::IMPACT) == 0, "the count was not restarted");
    ASSERT_MSG(fixture.feedMany(1, 4.0f, 0.0f, AlertType::IMPACT) == 1, "no alert on the second consecutive sample");
    // Latched: a crash raises one alert however long the high G lasts.
    // ラッチ: 高 G が続いても 1 回の墜落でアラートは 1 回。
    ASSERT_MSG(fixture.feedMany(kGyroReraiseSamples, 4.0f, 0.0f, AlertType::IMPACT) == 0, "latched impact re-raised");
    // Disarm clears the latch: the next flight is protected again.
    // DISARM でラッチが外れる: 次の飛行は再び保護される。
    fixture.feed(4.0f, 0.0f, false);
    ASSERT_MSG(fixture.feedMany(kConsecutiveSamples, 4.0f, 0.0f, AlertType::IMPACT) == 1, "no alert after re-arming");
}

TEST(nothing_is_detected_while_disarmed)
{
    ImuFixture fixture;
    for (uint32_t i = 0; i < kConsecutiveSamples * 2; ++i) {
        fixture.feed(5.0f, 2000.0f, false);
    }
    SystemAlert unexpected = {};
    ASSERT_MSG(!system_alert.read(unexpected), "alert of type %u while disarmed", unexpected.type);
}

TEST(gyro_anomaly_threshold_is_800_dps_on_any_axis_either_sign)
{
    // Just below and just above on each axis and sign. The boundary value itself depends on the float
    // conversion rad/s -> deg/s, so only the two sides are asserted.
    // 各軸・各符号の直下と直上。境界値そのものは rad/s → deg/s の float 変換に左右されるので、両側だけを検査する。
    const float below_dps = kGyroLimitDps - 1.0f;
    const float above_dps = kGyroLimitDps + 1.0f;
    for (int axis = 0; axis < 3; ++axis) {
        for (float sign : {1.0f, -1.0f}) {
            for (float dps : {below_dps, above_dps}) {
                ImuFixture fixture;
                const float accel[3] = {0.0f, 0.0f, kOneG * kGravity};
                float gyro[3] = {0.0f, 0.0f, 0.0f};
                gyro[axis] = sign * dps * kDegToRad;
                for (uint32_t i = 0; i < kConsecutiveSamples; ++i) {
                    fixture.detector.feedSample(accel, gyro, true);
                }
                const size_t alerts = drainAlerts(AlertType::GYRO_ANOMALY).size();
                const size_t expected = (dps > kGyroLimitDps) ? 1u : 0u;
                ASSERT_MSG(alerts == expected, "axis %d sign %+.0f %.0f dps: %zu alerts, expected %zu", axis, sign, dps,
                           alerts, expected);
            }
        }
    }
}

TEST(gyro_anomaly_needs_two_consecutive_samples)
{
    ImuFixture fixture;
    const float over_dps = kGyroLimitDps + 100.0f;
    ASSERT_MSG(fixture.feedMany(1, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 0, "one sample raised an alert");
    fixture.feed(kOneG, 0.0f);
    ASSERT_MSG(fixture.feedMany(1, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 0, "the count was not restarted");
    ASSERT_MSG(fixture.feedMany(1, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 1, "no alert on the second sample");
}

TEST(gyro_anomaly_is_re_raised_at_most_once_per_400_samples)
{
    ImuFixture fixture;
    const float over_dps = kGyroLimitDps + 100.0f;
    ASSERT_MSG(fixture.feedMany(kConsecutiveSamples, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 1, "no first alert");
    // Samples 3 .. 401 are suppressed; sample 402 (400 samples after the first alert) raises again.
    // サンプル 3〜401 は抑制され、サンプル 402（最初の発報の 400 サンプル後）で再発報する。
    ASSERT_MSG(fixture.feedMany(kGyroReraiseSamples - 1, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 0,
               "re-raised inside the suppression window");
    ASSERT_MSG(fixture.feedMany(1, kOneG, over_dps, AlertType::GYRO_ANOMALY) == 1, "no re-raise after the window");
}

// =============================================================================
// Runner
// 実行部
// =============================================================================

static int silentLogOutput(const char*, va_list)
{
    return 0;
}

int main()
{
    esp_log_set_vprintf(silentLogOutput);   // the firmware log lines are not part of the test output / ファームのログ行は試験の出力に含めない
    printf("=== Failsafe host tests (requirements §9 thresholds) ===\n");
    int passed = 0;
    int failed = 0;
    for (const TestCase& test_case : registry()) {
        printf("  [TEST] %-84s ", test_case.name);
        try {
            test_case.body();
            printf("PASS\n");
            ++passed;
        } catch (const AssertionFailure&) {
            printf("  FAIL  %s\n", test_case.name);
            ++failed;
        }
    }
    printf("\n=== %d run, %d passed, %d failed ===\n", passed + failed, passed, failed);
    return failed == 0 ? 0 : 1;
}
