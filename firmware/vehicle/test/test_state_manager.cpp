/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_state_manager.cpp
 * @brief Host tests for StateManager against the normative table (detailed_design.md §3.1)
 *        規範表（detailed_design.md §3.1）に対する StateManager のホスト試験
 *
 * state_manager.cpp is compiled unmodified. The topics (sf_core/params.cpp) and the clock, log and NVS shims come from
 * simulator/sils/compat, the same host shims the SILS uses. Time is virtual: a test moves it with Fixture::advance().
 *
 * The expected values are written out from the table and the design text, one function per table column or row
 * (expectedAfterDisarm, expectedAfterLandingRequest, ...), not recomputed with the production decision functions.
 *
 *  Table row / column                                    Tests
 *  ----------------------------------------------------  -------------------------------------------------------
 *  Mode arbitration, "edge" column (all 8 states)        mode_change_is_accepted_only_where_the_table_applies
 *  Mode arbitration, "API setting" column                api_takeoff_mode_is_applied_only_on_the_ground,
 *                                                        api_takeoff_refused_arm_restores_the_previous_mode,
 *                                                        api_takeoff_refused_arm_keeps_mode_when_the_switch_was_refused,
 *                                                        api_takeoff_in_flying_leaves_the_mode_alone (disabled)
 *  Mode arbitration, "ground reset" column               returning_to_the_ground_resets_the_mode_to_stabilize
 *  Mode arbitration, "Flip request" column               flip_request_is_accepted_only_in_flying,
 *                                                        flip_direction_is_kept_for_the_enter_callback
 *  Transitions among INIT / IDLE / ARMED / TAKEOFF /     each_notification_acts_only_in_its_source_state,
 *   FLYING / LANDING / FLIP rows                         notifications_end_in_the_table_destination
 *  ARMED_GROUND -> IDLE_GROUND, TAKEOFF -> IDLE_GROUND,  disarm_follows_the_table_for_every_state_and_mode,
 *   FLYING -> LANDING / IDLE_GROUND, LANDING -> IDLE,    emergency_stop_cuts_every_armed_state,
 *   FLIP -> IDLE_GROUND                                  landing_request_follows_the_table_for_every_state_and_mode
 *  Pre-arm gates (notes 9, 10, 12)                       each_pre_arm_gate_refuses_arm, pre_arm_gates_report_the_first_failing_reason,
 *                                                        unknown_battery_voltage_does_not_refuse_arm
 *  Pairing (parallel state machine)                      pairing_can_start_only_on_the_ground_or_held,
 *                                                        pairing_completes_and_pairing_again_is_allowed,
 *                                                        arm_is_refused_while_pairing
 *  Alert rows: IMPACT / GYRO_ANOMALY                     impact_disarms_every_armed_state, gyro_anomaly_disarms_except_in_flip
 *  Alert rows: COMM_LOST                                 comm_loss_lands_after_the_hover_grace_from_takeoff_and_flying,
 *                                                        comm_loss_before_flight_does_not_land_a_later_flight,
 *                                                        comm_loss_in_flip_waits_for_flying
 *  Alert rows: LOW_BATTERY                               battery_emergency_lands_takeoff_and_flying_only,
 *                                                        battery_warning_changes_nothing,
 *                                                        battery_emergency_in_flip_is_deferred_until_flying,
 *                                                        deferred_battery_emergency_is_dropped_when_flip_ends_on_the_ground
 *  Alert rows: ESKF_DIVERGED / USB_POWER / NONE          eskf_divergence_commands_an_estimator_reset,
 *                                                        usb_power_and_none_alerts_change_nothing
 *  ToF silence rows (per state and flight mode)          tof_silence_alert_follows_the_table_for_every_state_and_mode,
 *                                                        tof_silence_warning_tone_is_rate_limited,
 *                                                        tof_silence_in_flip_is_decided_when_flip_ends,
 *                                                        tof_silence_level_without_an_alert_is_decided_when_flip_ends
 *  ToF silence, LANDING rows (blind landing)             blind_landing_ends_by_time_only_after_both_limits,
 *                                                        landing_with_a_live_tof_never_ends_by_time,
 *                                                        blind_landing_ends_by_the_silent_time_when_the_tof_dies_late,
 *                                                        impact_while_landing_with_a_silent_tof_is_a_touchdown,
 *                                                        last_landing_end_is_recorded_and_cleared
 *  ToF silence, IDLE_GROUND / IDLE_HELD / INIT rows      tof_silence_refuses_arm_in_every_mode,
 *                                                        stale_failsafe_status_reads_as_tof_silent
 *  ToF silence column of the mode arbitration table      altitude_modes_are_refused_on_the_ground_and_the_edge_persists,
 *                                                        altitude_modes_are_refused_in_flight_and_the_edge_is_consumed
 *  Callbacks and published system_mode                   callbacks_run_exit_then_enter_with_both_endpoints,
 *                                                        callbacks_do_not_run_for_refused_requests,
 *                                                        mode_callbacks_receive_old_and_new_mode,
 *                                                        callback_registration_is_limited,
 *                                                        system_mode_publishes_state_mode_and_entry_time
 *  Initial state                                         init_starts_in_init_stabilize_not_paired
 *
 * Not covered here (StateTask / ImuTask / ApiTask rows, not host-buildable): the "mismatch (level)" column, the execution of
 * decideTakeoff() (completion / abort of TAKEOFF), the attitude_mismatch and bench-test events, the API `land` parking.
 * 対象外（StateTask / ImuTask / ApiTask の行でホストでビルドできない）: 「不一致（レベル）」の列、decideTakeoff() の実行
 * （TAKEOFF の完了・中止）、attitude_mismatch とベンチ試験のイベント、API `land` の保留。
 *
 * @design architecture.md INV-4 — every (state x input) cell of the table is checked  [OK]
 * @design detailed_design.md §3.1 — normative table                                   [OK]
 * @design coding_and_education.md §2 — Bilingual comments                              [OK]
 */

#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

// Angle brackets: the SILS compat headers on the include path, not the local stubs next to this file.
// 山括弧: この隣にあるローカルの代用ではなく、インクルードパス上の SILS の compat ヘッダを使う。
#include <esp_log.h>
#include <esp_timer.h>

#include "flight_limits.hpp"
#include "state_manager.hpp"
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
    const char* disabled_reason;   ///< nullptr = enabled / nullptr = 有効
};

static std::vector<TestCase>& registry()
{
    static std::vector<TestCase> cases;
    return cases;
}

struct Registrar {
    Registrar(const char* name, void (*body)(), const char* disabled_reason)
    {
        registry().push_back({name, body, disabled_reason});
    }
};

struct AssertionFailure {};

#define TEST(name) \
    static void test_##name(); \
    static Registrar registrar_##name(#name, test_##name, nullptr); \
    static void test_##name()

#define DISABLED_TEST(name, reason) \
    static void test_##name(); \
    static Registrar registrar_##name(#name, test_##name, reason); \
    static void test_##name()

#define ASSERT_TRUE(cond) \
    do { \
        if (!(cond)) { \
            printf("\n      FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            throw AssertionFailure{}; \
        } \
    } while (0)

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

static constexpr uint32_t kStartTimeUs = 1000000;          ///< first virtual time (a stamp of 0 means "never") / 仮想時間の開始（時刻印 0 は「一度も無い」）
static constexpr uint32_t kTickUs = 100000;                ///< one StateTask / Failsafe cycle [us] / StateTask・Failsafe の 1 周期
static constexpr uint32_t kCommLossGraceUs = 3000000;      ///< hover grace after COMM_LOST (requirements §9: 3 s) / COMM_LOST 後のホバー猶予
static constexpr float kHealthyVoltageV = 4.0f;            ///< a charged pack / 充電済みの電池
static constexpr float kUsbRailVoltageV = 3.0f;            ///< at or below safety.battery.usb_v (3.3 V default) / 既定 3.3 V 以下
static constexpr uint32_t kWarnToneEveryAlerts = 5;        ///< "about every 5 s", alerts come about once a second / 約 5 s ごと（アラートは約 1 s ごと）
static constexpr int kMaxCallbacks = 8;                    ///< callbacks one StateManager keeps per kind / 種類ごとに保持するコールバック数
static constexpr FlipDirection kFlipDirection = FlipDirection::Forward;

static constexpr FlightState kAllStates[] = {
    FlightState::INIT,    FlightState::IDLE_GROUND, FlightState::IDLE_HELD, FlightState::ARMED_GROUND,
    FlightState::TAKEOFF, FlightState::FLYING,      FlightState::FLIP,      FlightState::LANDING,
};

static constexpr FlightMode kAllModes[] = {
    FlightMode::ACRO, FlightMode::STABILIZE, FlightMode::ALT_HOLD, FlightMode::POS_HOLD,
};

static bool isAltitudeMode(FlightMode mode)
{
    return mode == FlightMode::ALT_HOLD || mode == FlightMode::POS_HOLD;
}

static bool isArmedState(FlightState state)
{
    return state == FlightState::ARMED_GROUND || state == FlightState::TAKEOFF || state == FlightState::FLYING ||
           state == FlightState::FLIP || state == FlightState::LANDING;
}

// =============================================================================
// Fixture: one StateManager with the topics, the clock and the failsafe status around it
// 試験台: StateManager と、その周りのトピック・時計・failsafe_status
// =============================================================================

class Fixture {
public:
    StateManager manager;

    Fixture()
    {
        topics_init();
        sils::compat::set_virtual_time_us(kStartTimeUs);
        publishReadySystemStatus(true, false, true);
        publishVoltage(kHealthyVoltageV);
        tof_silent_ = false;
        publishFailsafeStatus();
        manager.init();
    }

    uint32_t nowUs() const { return static_cast<uint32_t>(esp_timer_get_time()); }

    /// Publish system_status: boot calibration, attitude mismatch and the positive attitude verdict.
    /// system_status を発行する: 起動校正・姿勢の不一致・姿勢の肯定判定。
    void publishReadySystemStatus(bool calibrated, bool mismatch, bool verified)
    {
        SystemStatus status = {};
        status.calibrated = calibrated;
        status.attitude_mismatch = mismatch;
        status.attitude_verified = verified;
        system_status.publish(status);
    }

    void publishVoltage(float voltage)
    {
        PowerData power = {};
        power.voltage = voltage;
        sensor_power.publish(power);
    }

    /// Set the ToF-silence level Failsafe publishes and publish it with the current time.
    /// Failsafe が発行する ToF 出力停止のレベルを設定し、現在時刻で発行する。
    void setToFSilent(bool silent)
    {
        tof_silent_ = silent;
        publishFailsafeStatus();
    }

    void publishFailsafeStatus()
    {
        FailsafeStatus status = {};
        status.tof_silent = tof_silent_;
        status.tof_reason = static_cast<uint8_t>(tof_silent_ ? ToFSilenceReason::NoSample : ToFSilenceReason::None);
        status.timestamp = nowUs();
        failsafe_status.publish(status);
    }

    /// One StateTask cycle at the current time: refresh the failsafe status, then update().
    /// 現在時刻での StateTask の 1 周期: failsafe_status を更新してから update()。
    void updateNow()
    {
        publishFailsafeStatus();
        manager.update(nowUs());
    }

    /// Move the virtual time forward in whole cycles, running a StateTask cycle in each.
    /// 仮想時間を周期の整数倍だけ進め、各周期で StateTask の 1 周期を実行する。
    void advance(uint32_t duration_us)
    {
        ASSERT_MSG(duration_us % kTickUs == 0, "duration %u is not a whole number of cycles", duration_us);
        for (uint32_t elapsed_us = 0; elapsed_us < duration_us; elapsed_us += kTickUs) {
            sils::compat::advance_virtual_time_us(kTickUs);
            updateNow();
        }
    }

    void raise(AlertType type, AlertSeverity severity = AlertSeverity::CRITICAL)
    {
        SystemAlert alert = {};
        alert.type = static_cast<uint8_t>(type);
        alert.severity = static_cast<uint8_t>(severity);
        alert.timestamp = nowUs();
        manager.handleAlert(alert);
    }

    /// Bring the manager to `target` through the public requests only, with `mode` selected on the ground first.
    /// 公開された要求だけで `target` まで進める。先に地上で `mode` を選ぶ。
    void driveTo(FlightState target, FlightMode mode = FlightMode::STABILIZE)
    {
        if (target == FlightState::INIT) {
            return;
        }
        manager.notifyInitComplete();
        ASSERT_TRUE(manager.requestModeChange(mode));
        if (target == FlightState::IDLE_GROUND) {
            return;
        }
        if (target == FlightState::IDLE_HELD) {
            manager.notifyIdleGroundHeld(true);
        } else {
            driveFromIdleToAirborne(target);
        }
        ASSERT_MSG(manager.getState() == target, "wanted %s, reached %s", flightStateName(target),
                   flightStateName(manager.getState()));
    }

    SystemMode publishedMode() const { return system_mode.latest(); }

    std::vector<NotifyEvent> drainNotifyEvents()
    {
        std::vector<NotifyEvent> events;
        NotifyCommand command = {};
        while (notify_command.read(command)) {
            events.push_back(static_cast<NotifyEvent>(command.event));
        }
        return events;
    }

    int countNotifyEvents(NotifyEvent wanted)
    {
        int count = 0;
        for (NotifyEvent event : drainNotifyEvents()) {
            count += (event == wanted) ? 1 : 0;
        }
        return count;
    }

private:
    void driveFromIdleToAirborne(FlightState target)
    {
        ASSERT_TRUE(manager.requestArm());
        if (target == FlightState::ARMED_GROUND) {
            return;
        }
        manager.notifyTakeoff();
        if (target == FlightState::TAKEOFF) {
            return;
        }
        manager.notifyTakeoffComplete();
        if (target == FlightState::LANDING) {
            manager.notifyLandingRequest();
        } else if (target == FlightState::FLIP) {
            ASSERT_TRUE(manager.requestFlip(kFlipDirection));
        }
    }

    bool tof_silent_ = false;
};

// =============================================================================
// Expected values written out from the table
// 表から書き出した期待値
// =============================================================================

/// Disarm column: the state after requestDisarm() (the table rows of notes 5 and the FLYING / FLIP / LANDING rows).
/// DISARM の列: requestDisarm() の後の状態（注5 の行と FLYING / FLIP / LANDING の行）。
static FlightState expectedAfterDisarm(FlightState state, FlightMode mode)
{
    if (!isArmedState(state)) {
        return state;   // INIT / IDLE_GROUND / IDLE_HELD: not armed, refused / 武装していない: 拒否
    }
    if (state == FlightState::FLYING && isAltitudeMode(mode)) {
        return FlightState::LANDING;   // ALT_HOLD / POS_HOLD in flight: auto-landing / 飛行中の高度モード: 自動着陸
    }
    return FlightState::IDLE_GROUND;   // ARMED_GROUND, TAKEOFF (any mode), FLYING ACRO/STABILIZE, FLIP, LANDING: motor cut / モータ停止
}

/// API `land` column: the state after notifyLandingRequest().
/// API `land` の列: notifyLandingRequest() の後の状態。
static FlightState expectedAfterLandingRequest(FlightState state, FlightMode mode)
{
    if (state == FlightState::FLYING) {
        return FlightState::LANDING;
    }
    if (state == FlightState::ARMED_GROUND && isAltitudeMode(mode)) {
        return FlightState::IDLE_GROUND;   // the motors have only begun to spool: immediate DISARM / モータは回り始めただけ: 即時 DISARM
    }
    return state;   // manual-mode ARMED_GROUND, TAKEOFF, FLIP, LANDING and the ground states: ignored / 無視
}

/// Downward-ToF silence rows: the state after TOF_SILENT is raised (LANDING has not reached its time limit).
/// 下向き ToF 出力停止の行: TOF_SILENT を発報した後の状態（LANDING は時間切れに達していない）。
static FlightState expectedAfterToFSilence(FlightState state, FlightMode mode)
{
    switch (state) {
        case FlightState::ARMED_GROUND:
            return FlightState::IDLE_GROUND;   // any mode: never launch blind / どのモードでも: 見えないまま離陸しない
        case FlightState::TAKEOFF:
        case FlightState::FLYING:
            return isAltitudeMode(mode) ? FlightState::LANDING : state;   // ACRO / STABILIZE: warn only / 警告のみ
        default:
            return state;   // INIT, IDLE_*, FLIP (decided at the end of the flip), LANDING (by time) / 遷移なし
    }
}

// =============================================================================
// Initial state
// 初期状態
// =============================================================================

TEST(init_starts_in_init_stabilize_not_paired)
{
    Fixture fixture;
    ASSERT_TRUE(fixture.manager.getState() == FlightState::INIT);
    ASSERT_TRUE(fixture.manager.getMode() == FlightMode::STABILIZE);
    ASSERT_TRUE(fixture.manager.getPairingState() == PairingState::NotPaired);
    ASSERT_TRUE(!fixture.manager.isArmed());
}

// =============================================================================
// Notifications and ARM: each acts only in its source state
// 通知と ARM: それぞれ送り元の状態でだけ作用する
// =============================================================================

struct Edge {
    const char* name;
    FlightState from;
    FlightState to;
    std::function<void(StateManager&)> action;
};

static std::vector<Edge> notificationEdges()
{
    return {
        {"notifyInitComplete", FlightState::INIT, FlightState::IDLE_GROUND,
         [](StateManager& manager) { manager.notifyInitComplete(); }},
        {"notifyIdleGroundHeld(true)", FlightState::IDLE_GROUND, FlightState::IDLE_HELD,
         [](StateManager& manager) { manager.notifyIdleGroundHeld(true); }},
        {"notifyIdleGroundHeld(false)", FlightState::IDLE_HELD, FlightState::IDLE_GROUND,
         [](StateManager& manager) { manager.notifyIdleGroundHeld(false); }},
        {"requestArm", FlightState::IDLE_GROUND, FlightState::ARMED_GROUND,
         [](StateManager& manager) { manager.requestArm(); }},
        {"notifyTakeoff", FlightState::ARMED_GROUND, FlightState::TAKEOFF,
         [](StateManager& manager) { manager.notifyTakeoff(); }},
        {"notifyTakeoffComplete", FlightState::TAKEOFF, FlightState::FLYING,
         [](StateManager& manager) { manager.notifyTakeoffComplete(); }},
        {"notifyTakeoffEndedByLanding", FlightState::TAKEOFF, FlightState::LANDING,
         [](StateManager& manager) { manager.notifyTakeoffEndedByLanding(); }},
        {"notifyLandingRequest", FlightState::FLYING, FlightState::LANDING,
         [](StateManager& manager) { manager.notifyLandingRequest(); }},
        {"notifySoftLanding", FlightState::FLYING, FlightState::ARMED_GROUND,
         [](StateManager& manager) { manager.notifySoftLanding(); }},
        {"notifyLandingComplete", FlightState::LANDING, FlightState::IDLE_GROUND,
         [](StateManager& manager) { manager.notifyLandingComplete(); }},
        {"requestFlip", FlightState::FLYING, FlightState::FLIP,
         [](StateManager& manager) { manager.requestFlip(kFlipDirection); }},
        {"notifyFlipComplete", FlightState::FLIP, FlightState::FLYING,
         [](StateManager& manager) { manager.notifyFlipComplete(); }},
    };
}

TEST(each_notification_acts_only_in_its_source_state)
{
    // STABILIZE is selected so the altitude-mode rows of notifyLandingRequest do not apply here.
    // STABILIZE を選ぶので、notifyLandingRequest の高度モードの行はここでは当てはまらない。
    for (const Edge& edge : notificationEdges()) {
        for (FlightState state : kAllStates) {
            Fixture fixture;
            fixture.driveTo(state);
            edge.action(fixture.manager);
            const FlightState expected = (state == edge.from) ? edge.to : state;
            ASSERT_MSG(fixture.manager.getState() == expected, "%s from %s: expected %s, got %s", edge.name,
                       flightStateName(state), flightStateName(expected), flightStateName(fixture.manager.getState()));
        }
    }
}

TEST(notifications_end_in_the_table_destination)
{
    // The path INIT -> ... -> LANDING -> IDLE_GROUND and the FLIP loop, each hop checked on its own.
    // INIT から LANDING・IDLE_GROUND までの経路と FLIP の往復を、1 区間ずつ確かめる。
    Fixture fixture;
    const FlightState path[] = {FlightState::IDLE_GROUND, FlightState::ARMED_GROUND, FlightState::TAKEOFF,
                                FlightState::FLYING,      FlightState::FLIP,         FlightState::FLYING,
                                FlightState::LANDING,     FlightState::IDLE_GROUND};
    const std::function<void()> hops[] = {
        [&] { fixture.manager.notifyInitComplete(); },
        [&] { fixture.manager.requestArm(); },
        [&] { fixture.manager.notifyTakeoff(); },
        [&] { fixture.manager.notifyTakeoffComplete(); },
        [&] { fixture.manager.requestFlip(kFlipDirection); },
        [&] { fixture.manager.notifyFlipComplete(); },
        [&] { fixture.manager.notifyLandingRequest(); },
        [&] { fixture.manager.notifyLandingComplete(); },
    };
    for (size_t index = 0; index < sizeof(path) / sizeof(path[0]); ++index) {
        hops[index]();
        ASSERT_MSG(fixture.manager.getState() == path[index], "hop %zu: expected %s, got %s", index,
                   flightStateName(path[index]), flightStateName(fixture.manager.getState()));
    }
    ASSERT_TRUE(!fixture.manager.isArmed());
}

// =============================================================================
// DISARM, emergency stop, landing request
// DISARM・緊急停止・着陸要求
// =============================================================================

TEST(disarm_follows_the_table_for_every_state_and_mode)
{
    for (FlightState state : kAllStates) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            const bool accepted = fixture.manager.requestDisarm();
            const FlightState expected = expectedAfterDisarm(state, mode);
            ASSERT_MSG(fixture.manager.getState() == expected, "DISARM in %s/%s: expected %s, got %s",
                       flightStateName(state), flightModeName(mode), flightStateName(expected),
                       flightStateName(fixture.manager.getState()));
            ASSERT_MSG(accepted == isArmedState(state), "DISARM in %s/%s: accepted=%d", flightStateName(state),
                       flightModeName(mode), accepted);
        }
    }
}

TEST(emergency_stop_cuts_every_armed_state)
{
    // Unconditional: even ALT_HOLD / POS_HOLD in FLYING (no auto-landing) and FLIP. Refused when not armed.
    // 無条件: FLYING の ALT_HOLD / POS_HOLD（自動着陸なし）と FLIP でも。武装していなければ拒否。
    for (FlightState state : kAllStates) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            const bool accepted = fixture.manager.requestEmergencyStop();
            const FlightState expected = isArmedState(state) ? FlightState::IDLE_GROUND : state;
            ASSERT_MSG(fixture.manager.getState() == expected && accepted == isArmedState(state),
                       "emergency in %s/%s: state %s accepted=%d", flightStateName(state), flightModeName(mode),
                       flightStateName(fixture.manager.getState()), accepted);
        }
    }
}

TEST(landing_request_follows_the_table_for_every_state_and_mode)
{
    for (FlightState state : kAllStates) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            fixture.manager.notifyLandingRequest();
            const FlightState expected = expectedAfterLandingRequest(state, mode);
            ASSERT_MSG(fixture.manager.getState() == expected, "land in %s/%s: expected %s, got %s",
                       flightStateName(state), flightModeName(mode), flightStateName(expected),
                       flightStateName(fixture.manager.getState()));
        }
    }
}

// =============================================================================
// Mode arbitration table
// モード調停表
// =============================================================================

TEST(mode_change_is_accepted_only_where_the_table_applies)
{
    // "Edge" column: applied in IDLE_GROUND, ARMED_GROUND, FLYING; refused in INIT, IDLE_HELD, TAKEOFF, FLIP, LANDING.
    // 「エッジ」の列: IDLE_GROUND・ARMED_GROUND・FLYING で適用、INIT・IDLE_HELD・TAKEOFF・FLIP・LANDING で拒否。
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        const bool applies = state == FlightState::IDLE_GROUND || state == FlightState::ARMED_GROUND ||
                             state == FlightState::FLYING;
        const bool accepted = fixture.manager.requestModeChange(FlightMode::ALT_HOLD);
        const FlightMode expected = applies ? FlightMode::ALT_HOLD : FlightMode::STABILIZE;
        ASSERT_MSG(accepted == applies && fixture.manager.getMode() == expected, "mode change in %s: accepted=%d mode=%s",
                   flightStateName(state), accepted, flightModeName(fixture.manager.getMode()));
    }
}

TEST(api_takeoff_mode_is_applied_only_on_the_ground)
{
    // "API setting" column: applied in IDLE_GROUND (then ARM) and ARMED_GROUND (already armed, mode only); refused elsewhere.
    // 「API設定」の列: IDLE_GROUND（続けて ARM）と ARMED_GROUND（既に武装済み、モードのみ）で適用、他は拒否。
    const FlightState refused_states[] = {FlightState::INIT, FlightState::IDLE_HELD, FlightState::TAKEOFF,
                                          FlightState::FLIP, FlightState::LANDING};
    for (FlightState state : refused_states) {
        Fixture fixture;
        fixture.driveTo(state);
        ASSERT_TRUE(!fixture.manager.requestArmForTakeoff(FlightMode::ALT_HOLD));
        ASSERT_MSG(fixture.manager.getMode() == FlightMode::STABILIZE && fixture.manager.getState() == state,
                   "API takeoff in %s changed the mode or the state", flightStateName(state));
    }
    Fixture idle;
    idle.driveTo(FlightState::IDLE_GROUND);
    ASSERT_TRUE(idle.manager.requestArmForTakeoff(FlightMode::ALT_HOLD));
    ASSERT_TRUE(idle.manager.getState() == FlightState::ARMED_GROUND && idle.manager.getMode() == FlightMode::ALT_HOLD);

    Fixture armed;
    armed.driveTo(FlightState::ARMED_GROUND);
    armed.drainNotifyEvents();
    ASSERT_TRUE(armed.manager.requestArmForTakeoff(FlightMode::POS_HOLD));
    ASSERT_TRUE(armed.manager.getState() == FlightState::ARMED_GROUND && armed.manager.getMode() == FlightMode::POS_HOLD);
    ASSERT_MSG(armed.drainNotifyEvents().empty(), "an ARM was requested although the craft is armed already");
}

DISABLED_TEST(api_takeoff_in_flying_leaves_the_mode_alone,
              "table cell is '(guidance target only)'; requestArmForTakeoff() applies the mode change in FLYING "
              "(whether ApiTask keeps the call from FLYING is not checked here)")
{
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING);
    ASSERT_TRUE(!fixture.manager.requestArmForTakeoff(FlightMode::ALT_HOLD));
    ASSERT_TRUE(fixture.manager.getMode() == FlightMode::STABILIZE);
}

TEST(api_takeoff_refused_arm_restores_the_previous_mode)
{
    // A refused ARM (here: boot calibration not complete) leaves no trace: the mode before the call is put back.
    // 拒否された ARM（ここでは起動校正が未完了）は痕跡を残さない: 呼び出し前のモードに戻す。
    for (FlightMode previous : {FlightMode::STABILIZE, FlightMode::ACRO}) {
        Fixture fixture;
        fixture.driveTo(FlightState::IDLE_GROUND, previous);
        fixture.publishReadySystemStatus(false, false, true);
        ASSERT_TRUE(!fixture.manager.requestArmForTakeoff(FlightMode::POS_HOLD));
        ASSERT_MSG(fixture.manager.getState() == FlightState::IDLE_GROUND && fixture.manager.getMode() == previous,
                   "mode after the refusal is %s, expected %s", flightModeName(fixture.manager.getMode()),
                   flightModeName(previous));
    }
}

TEST(api_takeoff_refused_arm_restores_the_previous_mode_through_the_mode_callbacks)
{
    // The controller learns the mode only through the mode callbacks: both the switch and the restore must be announced.
    // 制御器はモードをモードのコールバック経由でしか知らない: 切り替えと復元の両方を通知しなければならない。
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND, FlightMode::ACRO);
    std::vector<std::pair<FlightMode, FlightMode>> changes;
    fixture.manager.onModeChange([&](FlightMode old_mode, FlightMode new_mode) { changes.push_back({old_mode, new_mode}); });
    fixture.publishReadySystemStatus(false, false, true);
    ASSERT_TRUE(!fixture.manager.requestArmForTakeoff(FlightMode::ALT_HOLD));
    ASSERT_TRUE(changes.size() == 2);
    ASSERT_TRUE(changes[0].first == FlightMode::ACRO && changes[0].second == FlightMode::ALT_HOLD);
    ASSERT_TRUE(changes[1].first == FlightMode::ALT_HOLD && changes[1].second == FlightMode::ACRO);
}

TEST(api_takeoff_refused_arm_keeps_mode_when_the_switch_was_refused)
{
    // The ToF is silent: the switch into ALT_HOLD is refused first, so there is no mode to put back; ARM is refused as well.
    // ToF が出力停止: ALT_HOLD への切り替えがまず拒否されるので戻すモードは無い。ARM も拒否される。
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND, FlightMode::ACRO);
    fixture.setToFSilent(true);
    ASSERT_TRUE(!fixture.manager.requestArmForTakeoff(FlightMode::ALT_HOLD));
    ASSERT_TRUE(fixture.manager.getState() == FlightState::IDLE_GROUND && fixture.manager.getMode() == FlightMode::ACRO);
}

TEST(returning_to_the_ground_resets_the_mode_to_stabilize)
{
    // "Ground reset" column: every airborne state into IDLE_GROUND forces STABILIZE and announces it to the mode callbacks.
    // 「接地リセット」の列: 空中の全状態から IDLE_GROUND に入ると STABILIZE に強制し、モードのコールバックへ通知する。
    const FlightState airborne[] = {FlightState::TAKEOFF, FlightState::FLYING, FlightState::FLIP, FlightState::LANDING};
    for (FlightState state : airborne) {
        Fixture fixture;
        fixture.driveTo(state, FlightMode::POS_HOLD);
        std::vector<std::pair<FlightMode, FlightMode>> changes;
        fixture.manager.onModeChange([&](FlightMode old_mode, FlightMode new_mode) { changes.push_back({old_mode, new_mode}); });
        fixture.manager.requestEmergencyStop();
        ASSERT_MSG(fixture.manager.getState() == FlightState::IDLE_GROUND && fixture.manager.getMode() == FlightMode::STABILIZE,
                   "from %s: mode %s", flightStateName(state), flightModeName(fixture.manager.getMode()));
        ASSERT_MSG(changes.size() == 1 && changes[0].first == FlightMode::POS_HOLD && changes[0].second == FlightMode::STABILIZE,
                   "from %s: the mode callback was not run once with POS_HOLD -> STABILIZE", flightStateName(state));
    }
}

TEST(flip_request_is_accepted_only_in_flying)
{
    // "Flip request" column: accepted in FLYING; refused in FLIP (busy) and in every other state (not flying).
    // 「Flip 要求」の列: FLYING で受理、FLIP（busy）とその他の全状態（not flying）で拒否。
    for (FlightState state : kAllStates) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            const bool accepted = fixture.manager.requestFlip(kFlipDirection);
            const FlightState expected = (state == FlightState::FLYING) ? FlightState::FLIP : state;
            ASSERT_MSG(accepted == (state == FlightState::FLYING) && fixture.manager.getState() == expected,
                       "flip in %s/%s: accepted=%d state=%s", flightStateName(state), flightModeName(mode), accepted,
                       flightStateName(fixture.manager.getState()));
        }
    }
}

TEST(flip_direction_is_kept_for_the_enter_callback)
{
    const FlipDirection directions[] = {FlipDirection::Left, FlipDirection::Right, FlipDirection::Forward,
                                        FlipDirection::Back};
    for (FlipDirection direction : directions) {
        Fixture fixture;
        fixture.driveTo(FlightState::FLYING);
        FlipDirection seen_in_callback = FlipDirection::Left;
        fixture.manager.onEnter([&](FlightState, FlightState to) {
            if (to == FlightState::FLIP) {
                seen_in_callback = fixture.manager.getFlipDirection();
            }
        });
        ASSERT_TRUE(fixture.manager.requestFlip(direction));
        ASSERT_MSG(seen_in_callback == direction && fixture.manager.getFlipDirection() == direction,
                   "direction %s was not available to the onEnter(FLIP) callback", flipDirectionName(direction));
    }
}

// =============================================================================
// Pre-arm gates
// ARM 前判定
// =============================================================================

/// One way of making a single pre-arm gate fail, and the reason it must report.
/// ARM 前判定を 1 つだけ落とす方法と、報告すべき理由。
struct GateCase {
    const char* name;
    ArmBlock reason;
    std::function<void(Fixture&)> make_fail;
};

static std::vector<GateCase> gateCases()
{
    return {
        {"pairing", ArmBlock::Pairing, [](Fixture& fixture) { fixture.manager.requestPairing(); }},
        {"battery on the USB rail", ArmBlock::Battery, [](Fixture& fixture) { fixture.publishVoltage(kUsbRailVoltageV); }},
        {"calibration pending", ArmBlock::Calibrating,
         [](Fixture& fixture) { fixture.publishReadySystemStatus(false, false, true); }},
        {"bench re-level pending", ArmBlock::BenchRelevel,
         [](Fixture& fixture) { fixture.manager.setBenchRelevelPending(true); }},
        {"motor test active", ArmBlock::BenchRelevel,
         [](Fixture&) {
             MotorTest test = {};
             test.active = true;
             motor_test.publish(test);
         }},
        {"ToF silent", ArmBlock::TofSilent, [](Fixture& fixture) { fixture.setToFSilent(true); }},
        {"attitude mismatch", ArmBlock::TiltMismatch,
         [](Fixture& fixture) { fixture.publishReadySystemStatus(true, true, false); }},
        {"attitude not verified yet", ArmBlock::TiltPending,
         [](Fixture& fixture) { fixture.publishReadySystemStatus(true, false, false); }},
    };
}

TEST(each_pre_arm_gate_refuses_arm)
{
    for (const GateCase& gate : gateCases()) {
        Fixture fixture;
        fixture.driveTo(FlightState::IDLE_GROUND);
        fixture.drainNotifyEvents();
        gate.make_fail(fixture);
        ASSERT_MSG(!fixture.manager.requestArm(), "ARM was accepted although the gate '%s' fails", gate.name);
        ASSERT_MSG(fixture.manager.getState() == FlightState::IDLE_GROUND, "state changed on the refusal by '%s'", gate.name);
        ASSERT_MSG(fixture.countNotifyEvents(NotifyEvent::ArmRejected) == 1, "no single error tone for '%s'", gate.name);
        fixture.updateNow();
        ASSERT_MSG(fixture.publishedMode().arm_block == static_cast<uint8_t>(gate.reason),
                   "'%s': arm_block %u, expected %u", gate.name, fixture.publishedMode().arm_block,
                   static_cast<unsigned>(gate.reason));
    }
}

TEST(pre_arm_gates_report_the_first_failing_reason)
{
    // Priority: pairing, battery, calibration, bench test, ToF silence, attitude mismatch, attitude not verified.
    // 優先順: ペアリング・電池・校正・ベンチ試験・ToF 出力停止・姿勢の不一致・姿勢が未検証。
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    fixture.manager.requestPairing();
    fixture.publishVoltage(kUsbRailVoltageV);
    fixture.publishReadySystemStatus(false, true, false);
    fixture.manager.setBenchRelevelPending(true);
    fixture.setToFSilent(true);

    const std::function<void()> fix_in_priority_order[] = {
        [&] { fixture.manager.notifyPairingComplete(); },
        [&] { fixture.publishVoltage(kHealthyVoltageV); },
        [&] { fixture.publishReadySystemStatus(true, true, false); },
        [&] { fixture.manager.setBenchRelevelPending(false); },
        [&] { fixture.setToFSilent(false); },
        [&] { fixture.publishReadySystemStatus(true, false, false); },
        [&] { fixture.publishReadySystemStatus(true, false, true); },
    };
    const ArmBlock expected_reasons[] = {ArmBlock::Pairing,   ArmBlock::Battery,      ArmBlock::Calibrating,
                                         ArmBlock::BenchRelevel, ArmBlock::TofSilent, ArmBlock::TiltMismatch,
                                         ArmBlock::TiltPending};
    for (size_t index = 0; index < sizeof(expected_reasons) / sizeof(expected_reasons[0]); ++index) {
        fixture.updateNow();
        ASSERT_MSG(fixture.publishedMode().arm_block == static_cast<uint8_t>(expected_reasons[index]),
                   "step %zu: arm_block %u, expected %u", index, fixture.publishedMode().arm_block,
                   static_cast<unsigned>(expected_reasons[index]));
        fix_in_priority_order[index]();
    }
    fixture.updateNow();
    ASSERT_TRUE(fixture.publishedMode().arm_block == static_cast<uint8_t>(ArmBlock::None));
    ASSERT_TRUE(fixture.manager.requestArm());
}

TEST(unknown_battery_voltage_does_not_refuse_arm)
{
    // A reading of 0 means "no power monitor": power monitoring is optional, so it must not block ARM.
    // 読み 0 は「電源モニタが無い」: 電源監視は任意なので ARM を阻んではならない。
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    fixture.publishVoltage(0.0f);
    ASSERT_TRUE(fixture.manager.requestArm());
}

// =============================================================================
// Pairing
// ペアリング
// =============================================================================

TEST(pairing_can_start_only_on_the_ground_or_held)
{
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.drainNotifyEvents();
        fixture.manager.requestPairing();
        const bool allowed = state == FlightState::IDLE_GROUND || state == FlightState::IDLE_HELD;
        const PairingState expected = allowed ? PairingState::Pairing : PairingState::NotPaired;
        ASSERT_MSG(fixture.manager.getPairingState() == expected, "pairing from %s", flightStateName(state));
        ASSERT_MSG(static_cast<PairingState>(pairing_state.latest().state) == expected, "pairing topic from %s",
                   flightStateName(state));
        ASSERT_MSG(fixture.countNotifyEvents(NotifyEvent::PairingMode) == (allowed ? 1 : 0), "pairing tone from %s",
                   flightStateName(state));
    }
}

TEST(pairing_completes_and_pairing_again_is_allowed)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    fixture.manager.requestPairing();
    fixture.drainNotifyEvents();
    fixture.manager.requestPairing();   // already searching: idempotent, no second tone / 既に探索中: 冪等で 2 回目の音なし
    ASSERT_TRUE(fixture.countNotifyEvents(NotifyEvent::PairingMode) == 0);

    fixture.manager.notifyPairingComplete();
    ASSERT_TRUE(fixture.manager.getPairingState() == PairingState::Paired);
    ASSERT_TRUE(static_cast<PairingState>(pairing_state.latest().state) == PairingState::Paired);
    fixture.manager.notifyPairingComplete();   // idempotent / 冪等
    ASSERT_TRUE(fixture.manager.getPairingState() == PairingState::Paired);

    fixture.manager.requestPairing();   // pairing again from Paired (long press) / Paired からの再ペア（長押し）
    ASSERT_TRUE(fixture.manager.getPairingState() == PairingState::Pairing);
}

TEST(arm_is_refused_while_pairing)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_HELD);
    fixture.manager.requestPairing();   // started while held / 手持ちで開始
    fixture.manager.notifyIdleGroundHeld(false);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::IDLE_GROUND);
    ASSERT_TRUE(!fixture.manager.requestArm());
    fixture.manager.notifyPairingComplete();
    ASSERT_TRUE(fixture.manager.requestArm());
}

// =============================================================================
// Alerts: IMPACT, GYRO_ANOMALY
// アラート: IMPACT・GYRO_ANOMALY
// =============================================================================

TEST(impact_disarms_every_armed_state)
{
    // Every armed state, ARMED_GROUND and FLIP included. With a live ToF, LANDING is cut as well.
    // 武装している全状態（ARMED_GROUND と FLIP を含む）。ToF が生きていれば LANDING も停止する。
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::IMPACT);
        const FlightState expected = isArmedState(state) ? FlightState::IDLE_GROUND : state;
        ASSERT_MSG(fixture.manager.getState() == expected, "IMPACT in %s: got %s", flightStateName(state),
                   flightStateName(fixture.manager.getState()));
    }
}

TEST(gyro_anomaly_disarms_except_in_flip)
{
    // FLIP: a commanded flip spins past the crash threshold, so the alert is ignored there.
    // FLIP: 指令された宙返りは墜落のしきい値を超えて回るので、そこでは無視する。
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::GYRO_ANOMALY);
        const bool disarms = isArmedState(state) && state != FlightState::FLIP;
        const FlightState expected = disarms ? FlightState::IDLE_GROUND : state;
        ASSERT_MSG(fixture.manager.getState() == expected, "GYRO_ANOMALY in %s: got %s", flightStateName(state),
                   flightStateName(fixture.manager.getState()));
    }
}

// =============================================================================
// Alerts: COMM_LOST
// アラート: COMM_LOST
// =============================================================================

TEST(comm_loss_lands_after_the_hover_grace_from_takeoff_and_flying)
{
    for (FlightState state : {FlightState::TAKEOFF, FlightState::FLYING}) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            fixture.updateNow();
            fixture.raise(AlertType::COMM_LOST);
            fixture.advance(kCommLossGraceUs - kTickUs);
            ASSERT_MSG(fixture.manager.getState() == state, "%s/%s landed before the grace", flightStateName(state),
                       flightModeName(mode));
            fixture.advance(kTickUs);
            ASSERT_MSG(fixture.manager.getState() == FlightState::LANDING, "%s/%s did not land after the grace",
                       flightStateName(state), flightModeName(mode));
        }
    }
}

TEST(comm_loss_before_flight_does_not_land_a_later_flight)
{
    // A COMM_LOST on the ground starts no timer: the later takeoff is not landed by it.
    // 地上での COMM_LOST はタイマを起動しない: 後の離陸は、それで着陸させられない。
    for (FlightState state : {FlightState::INIT, FlightState::IDLE_GROUND, FlightState::IDLE_HELD, FlightState::ARMED_GROUND}) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::COMM_LOST);
        fixture.advance(kCommLossGraceUs + kTickUs);
        fixture.manager.requestEmergencyStop();
        fixture.manager.forceIdle();
        fixture.manager.requestArm();
        fixture.manager.notifyTakeoff();
        fixture.manager.notifyTakeoffComplete();
        fixture.advance(kCommLossGraceUs + kTickUs);
        ASSERT_MSG(fixture.manager.getState() == FlightState::FLYING, "COMM_LOST on %s landed a later flight",
                   flightStateName(state));
    }
}

TEST(comm_loss_in_flip_waits_for_flying)
{
    // The flip is not interrupted; the existing rule applies once FLIP has returned to FLYING.
    // 宙返りは中断しない。FLIP が FLYING に戻ってから既存の規則を適用する。
    Fixture fixture;
    fixture.driveTo(FlightState::FLIP);
    fixture.updateNow();
    fixture.raise(AlertType::COMM_LOST);
    fixture.advance(kCommLossGraceUs + kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLIP);
    fixture.manager.notifyFlipComplete();
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLYING);
    fixture.advance(kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
}

TEST(pending_comm_loss_is_dropped_on_the_ground)
{
    // A flight ended by DISARM cancels the pending landing: the next flight is not landed by the old loss.
    // DISARM で終えた飛行は保留中の着陸を取り消す: 次の飛行は古い喪失で着陸させられない。
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING);
    fixture.updateNow();
    fixture.raise(AlertType::COMM_LOST);
    fixture.manager.requestEmergencyStop();
    fixture.advance(kTickUs);   // on the ground: the timer is dropped / 地上: タイマは破棄される
    fixture.manager.requestArm();
    fixture.manager.notifyTakeoff();
    fixture.manager.notifyTakeoffComplete();
    fixture.advance(kCommLossGraceUs + kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLYING);
}

// =============================================================================
// Alerts: LOW_BATTERY
// アラート: LOW_BATTERY
// =============================================================================

TEST(battery_emergency_lands_takeoff_and_flying_only)
{
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::LOW_BATTERY, AlertSeverity::EMERGENCY);
        const bool lands = state == FlightState::TAKEOFF || state == FlightState::FLYING;
        const FlightState expected = lands ? FlightState::LANDING : state;   // FLIP defers, LANDING is already landing / FLIP は保留、LANDING は既に着陸
        ASSERT_MSG(fixture.manager.getState() == expected, "battery emergency in %s: got %s", flightStateName(state),
                   flightStateName(fixture.manager.getState()));
    }
}

TEST(battery_warning_changes_nothing)
{
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::LOW_BATTERY, AlertSeverity::WARNING);
        ASSERT_MSG(fixture.manager.getState() == state, "battery warning in %s: got %s", flightStateName(state),
                   flightStateName(fixture.manager.getState()));
    }
}

TEST(battery_emergency_in_flip_is_deferred_until_flying)
{
    Fixture fixture;
    fixture.driveTo(FlightState::FLIP);
    fixture.raise(AlertType::LOW_BATTERY, AlertSeverity::EMERGENCY);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLIP);
    fixture.manager.notifyFlipComplete();
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
}

TEST(deferred_battery_emergency_is_dropped_when_flip_ends_on_the_ground)
{
    // A DISARM in the flip lands the craft; the old emergency must not land a later, unrelated flip.
    // 宙返りの中の DISARM で機体は接地する。古い緊急事態が、後の無関係な宙返りを着陸させてはならない。
    Fixture fixture;
    fixture.driveTo(FlightState::FLIP);
    fixture.raise(AlertType::LOW_BATTERY, AlertSeverity::EMERGENCY);
    fixture.manager.requestEmergencyStop();
    ASSERT_TRUE(fixture.manager.getState() == FlightState::IDLE_GROUND);
    fixture.manager.requestArm();
    fixture.manager.notifyTakeoff();
    fixture.manager.notifyTakeoffComplete();
    ASSERT_TRUE(fixture.manager.requestFlip(kFlipDirection));
    fixture.manager.notifyFlipComplete();
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLYING);
}

// =============================================================================
// Alerts: ESKF_DIVERGED, USB_POWER, NONE
// アラート: ESKF_DIVERGED・USB_POWER・NONE
// =============================================================================

TEST(eskf_divergence_commands_an_estimator_reset)
{
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING);
    EstimatorCommand command = {};
    while (estimator_command.read(command)) {}   // drop what the transitions published / 遷移が発行した分を捨てる
    fixture.raise(AlertType::ESKF_DIVERGED);
    ASSERT_TRUE(estimator_command.read(command));
    ASSERT_TRUE(command.command == static_cast<uint8_t>(EstimatorCmd::Reset));
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLYING);
}

TEST(usb_power_and_none_alerts_change_nothing)
{
    for (FlightState state : kAllStates) {
        Fixture fixture;
        fixture.driveTo(state);
        fixture.raise(AlertType::USB_POWER);
        fixture.raise(AlertType::NONE);
        ASSERT_MSG(fixture.manager.getState() == state, "USB_POWER / NONE changed %s", flightStateName(state));
    }
}

// =============================================================================
// Downward-ToF silence
// 下向き ToF の出力停止
// =============================================================================

TEST(tof_silence_alert_follows_the_table_for_every_state_and_mode)
{
    for (FlightState state : kAllStates) {
        for (FlightMode mode : kAllModes) {
            Fixture fixture;
            fixture.driveTo(state, mode);
            fixture.setToFSilent(true);
            fixture.raise(AlertType::TOF_SILENT);
            const FlightState expected = expectedAfterToFSilence(state, mode);
            ASSERT_MSG(fixture.manager.getState() == expected, "ToF silent in %s/%s: expected %s, got %s",
                       flightStateName(state), flightModeName(mode), flightStateName(expected),
                       flightStateName(fixture.manager.getState()));
        }
    }
}

TEST(tof_silence_warning_tone_is_rate_limited)
{
    // ACRO / STABILIZE: a warning tone on the first alert, then about every 5 s (alerts come about once a second).
    // ACRO / STABILIZE: 最初のアラートで警告音、以後約 5 s ごと（アラートは約 1 s ごと）。
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::STABILIZE);
    fixture.setToFSilent(true);
    fixture.drainNotifyEvents();
    int tones = 0;
    for (uint32_t alert_index = 0; alert_index < kWarnToneEveryAlerts + 1; ++alert_index) {
        fixture.raise(AlertType::TOF_SILENT);
        const int expected_new = (alert_index == 0 || alert_index == kWarnToneEveryAlerts) ? 1 : 0;
        const int new_tones = fixture.countNotifyEvents(NotifyEvent::TofSilentWarn);
        ASSERT_MSG(new_tones == expected_new, "alert %u: %d new tones, expected %d", alert_index, new_tones, expected_new);
        tones += new_tones;
    }
    ASSERT_TRUE(tones == 2);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::FLYING);   // never a forced landing / 強制着陸はしない

    // A new episode starts the count again: its first alert sounds.
    // 次のエピソードは数え直す: その最初のアラートで鳴る。
    fixture.setToFSilent(false);
    fixture.advance(kTickUs);
    fixture.setToFSilent(true);
    fixture.raise(AlertType::TOF_SILENT);
    ASSERT_TRUE(fixture.countNotifyEvents(NotifyEvent::TofSilentWarn) == 1);
}

TEST(tof_silence_in_flip_is_decided_when_flip_ends)
{
    // The alert in FLIP is ignored; when FLIP -> FLYING the standing level decides on the spot.
    // FLIP 中のアラートは無視する。FLIP → FLYING のとき、継続状態（レベル）がその場で決める。
    for (FlightMode mode : kAllModes) {
        Fixture fixture;
        fixture.driveTo(FlightState::FLIP, mode);
        fixture.setToFSilent(true);
        fixture.raise(AlertType::TOF_SILENT);
        ASSERT_MSG(fixture.manager.getState() == FlightState::FLIP, "the flip was interrupted in %s", flightModeName(mode));
        fixture.manager.notifyFlipComplete();
        const FlightState expected = isAltitudeMode(mode) ? FlightState::LANDING : FlightState::FLYING;
        ASSERT_MSG(fixture.manager.getState() == expected, "after the flip in %s: got %s", flightModeName(mode),
                   flightStateName(fixture.manager.getState()));
    }
}

TEST(tof_silence_level_without_an_alert_is_decided_when_flip_ends)
{
    // The next re-raised alert (up to about 1 s away) is not waited for: the level alone decides.
    // 次に再発報されるアラート（最大約 1 s 先）は待たない: レベルだけで決める。
    Fixture fixture;
    fixture.driveTo(FlightState::FLIP, FlightMode::ALT_HOLD);
    fixture.setToFSilent(true);
    fixture.manager.notifyFlipComplete();
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
}

/// Start a landing by the ToF-silence rule from FLYING in ALT_HOLD, with an update at the very moment it begins.
/// FLYING の ALT_HOLD から、ToF 出力停止の規則で着陸を始める。始まったその時刻に update を 1 回行う。
static void startBlindLanding(Fixture& fixture)
{
    fixture.driveTo(FlightState::FLYING, FlightMode::ALT_HOLD);
    fixture.updateNow();
    fixture.setToFSilent(true);
    fixture.raise(AlertType::TOF_SILENT);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
    fixture.updateNow();
}

TEST(blind_landing_ends_by_time_only_after_both_limits)
{
    // LANDING with a silent ToF ends by time once 10 s have passed since the LANDING began AND the ToF has been silent for 5.5 s.
    // ToF 出力停止中の LANDING は、LANDING の開始から 10 s かつ ToF の出力停止が 5.5 s 続いて初めて時間で終わる。
    Fixture fixture;
    startBlindLanding(fixture);
    fixture.advance(limits::kBlindLandingTimeoutUs - kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
    fixture.advance(kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::IDLE_GROUND);
    ASSERT_TRUE(fixture.manager.lastLandingEnd() == StateManager::LandingEnd::ToFSilentTimeout);
}

TEST(landing_with_a_live_tof_never_ends_by_time)
{
    // With a live ToF the touchdown comes from the detector (ImuTask), not from a time limit.
    // ToF が生きていれば接地は検出器（ImuTask）が決める。時間切れでは終わらない。
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::ALT_HOLD);
    fixture.manager.notifyLandingRequest();
    fixture.updateNow();
    fixture.advance(2 * limits::kBlindLandingTimeoutUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
}

TEST(blind_landing_ends_by_the_silent_time_when_the_tof_dies_late)
{
    // A ToF that dies late in a long LANDING still gets the minimum silent time before the motors are cut.
    // 長い LANDING の終わりごろに ToF が止まっても、モータを切るまでに最低の出力停止時間を与える。
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::ALT_HOLD);
    fixture.manager.notifyLandingRequest();
    fixture.updateNow();
    fixture.advance(limits::kBlindLandingTimeoutUs + kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
    fixture.setToFSilent(true);
    fixture.updateNow();
    fixture.advance(limits::kBlindLandingMinSilentUs - kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::LANDING);
    fixture.advance(kTickUs);
    ASSERT_TRUE(fixture.manager.getState() == FlightState::IDLE_GROUND);
}

TEST(impact_while_landing_with_a_silent_tof_is_a_touchdown)
{
    // IMPACT: the touchdown of a descent without a ToF, not an emergency. GYRO_ANOMALY stays an emergency.
    // IMPACT: ToF なしの降下の接地であって非常ではない。GYRO_ANOMALY は非常のまま。
    Fixture impact;
    startBlindLanding(impact);
    impact.raise(AlertType::IMPACT);
    ASSERT_TRUE(impact.manager.getState() == FlightState::IDLE_GROUND);
    ASSERT_TRUE(impact.manager.lastLandingEnd() == StateManager::LandingEnd::ToFSilentImpact);

    Fixture gyro;
    startBlindLanding(gyro);
    gyro.raise(AlertType::GYRO_ANOMALY);
    ASSERT_TRUE(gyro.manager.getState() == FlightState::IDLE_GROUND);
    ASSERT_TRUE(gyro.manager.lastLandingEnd() != StateManager::LandingEnd::ToFSilentImpact);

    Fixture alive;
    alive.driveTo(FlightState::FLYING, FlightMode::ALT_HOLD);
    alive.manager.notifyLandingRequest();
    alive.raise(AlertType::IMPACT);
    ASSERT_TRUE(alive.manager.getState() == FlightState::IDLE_GROUND);
    ASSERT_TRUE(alive.manager.lastLandingEnd() != StateManager::LandingEnd::ToFSilentImpact);
}

TEST(last_landing_end_is_recorded_and_cleared)
{
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::ALT_HOLD);
    ASSERT_TRUE(fixture.manager.lastLandingEnd() == StateManager::LandingEnd::None);
    fixture.manager.notifyLandingRequest();
    fixture.manager.notifyLandingComplete();
    ASSERT_TRUE(fixture.manager.lastLandingEnd() == StateManager::LandingEnd::Detected);

    fixture.manager.requestArm();
    fixture.manager.notifyTakeoff();
    fixture.manager.notifyTakeoffComplete();
    fixture.manager.notifyLandingRequest();   // the next LANDING forgets the last end / 次の LANDING は直近の終わり方を忘れる
    ASSERT_TRUE(fixture.manager.lastLandingEnd() == StateManager::LandingEnd::None);
}

TEST(tof_silence_refuses_arm_in_every_mode)
{
    // IDLE_GROUND rows: ARM refused in every mode, ACRO included; the reason is published; ARM is accepted when the episode ends.
    // IDLE_GROUND の行: ACRO を含む全モードで ARM を拒否し、理由を発行する。エピソードが終われば受理する。
    for (FlightMode mode : kAllModes) {
        Fixture fixture;
        fixture.driveTo(FlightState::IDLE_GROUND, mode);
        fixture.setToFSilent(true);
        ASSERT_MSG(!fixture.manager.requestArm(), "ARM accepted in %s with a silent ToF", flightModeName(mode));
        fixture.updateNow();
        ASSERT_TRUE(fixture.publishedMode().arm_block == static_cast<uint8_t>(ArmBlock::TofSilent));
        fixture.setToFSilent(false);
        fixture.updateNow();
        ASSERT_TRUE(fixture.publishedMode().arm_block == static_cast<uint8_t>(ArmBlock::None));
        ASSERT_MSG(fixture.manager.requestArm(), "ARM refused in %s after the episode", flightModeName(mode));
    }
}

TEST(stale_failsafe_status_reads_as_tof_silent)
{
    // A status that nobody refreshes reads as silent, so a stalled Failsafe cannot leave the last "not silent" in force.
    // 誰も更新しない状態は出力停止と読む。Failsafe が停滞しても、最後の「出力停止でない」が効き続けることはない。
    Fixture stale;
    stale.driveTo(FlightState::IDLE_GROUND);
    ASSERT_TRUE(!stale.manager.isToFSilent());
    sils::compat::advance_virtual_time_us(limits::kFailsafeStatusStaleUs);   // no republish / 再発行しない
    ASSERT_TRUE(stale.manager.isToFSilent());
    ASSERT_TRUE(!stale.manager.requestArm());

    Fixture never;
    never.driveTo(FlightState::IDLE_GROUND);
    FailsafeStatus unpublished = {};   // timestamp 0: never published / 時刻印 0: 一度も発行されていない
    failsafe_status.publish(unpublished);
    ASSERT_TRUE(never.manager.isToFSilent());
}

TEST(altitude_modes_are_refused_on_the_ground_and_the_edge_persists)
{
    // Ground rows: ALT_HOLD / POS_HOLD refused during an episode (ModeBlock::TofSilent, request persists); ACRO / STABILIZE accepted;
    // the refusal ends with the episode.
    // 地上の行: エピソード中の ALT_HOLD / POS_HOLD は拒否（ModeBlock::TofSilent、要求は持続）。ACRO / STABILIZE は受理。拒否はエピソードとともに終わる。
    for (FlightState state : {FlightState::IDLE_GROUND, FlightState::ARMED_GROUND}) {
        for (FlightMode altitude_mode : {FlightMode::ALT_HOLD, FlightMode::POS_HOLD}) {
            Fixture fixture;
            fixture.driveTo(state);
            fixture.setToFSilent(true);
            ASSERT_TRUE(!fixture.manager.requestModeChange(altitude_mode));
            ASSERT_MSG(fixture.manager.getMode() == FlightMode::STABILIZE && !fixture.manager.isModeRequestConsumed(),
                       "%s/%s: mode %s consumed=%d", flightStateName(state), flightModeName(altitude_mode),
                       flightModeName(fixture.manager.getMode()), fixture.manager.isModeRequestConsumed());
            ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::TofSilent));
            ASSERT_TRUE(fixture.manager.requestModeChange(FlightMode::ACRO));   // leaving the altitude modes is never refused / 高度モードから出る方向は拒否しない
            fixture.manager.requestModeChange(FlightMode::STABILIZE);
            fixture.setToFSilent(false);
            fixture.updateNow();
            ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::None));
            ASSERT_TRUE(fixture.manager.requestModeChange(altitude_mode));
        }
    }
}

TEST(altitude_modes_are_refused_in_flight_and_the_edge_is_consumed)
{
    // FLYING row: the refused edge is CONSUMED; the mode does not switch by itself when the ToF recovers, the pilot flips the switch again.
    // FLYING の行: 拒否したエッジは「消費」する。ToF が復帰してもモードは自分では切り替わらず、パイロットがスイッチを入れ直す。
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::STABILIZE);
    fixture.setToFSilent(true);
    ASSERT_TRUE(!fixture.manager.requestModeChange(FlightMode::POS_HOLD));
    ASSERT_TRUE(fixture.manager.isModeRequestConsumed());
    ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::TofSwitchConsumed));
    ASSERT_TRUE(fixture.manager.getMode() == FlightMode::STABILIZE);

    fixture.setToFSilent(false);
    fixture.advance(kTickUs);
    ASSERT_TRUE(fixture.manager.getMode() == FlightMode::STABILIZE);   // no automatic switch / 自動では切り替わらない
    ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::TofSwitchConsumed));   // shown until a request is accepted / 要求が受理されるまで表示
    ASSERT_TRUE(fixture.manager.requestModeChange(FlightMode::POS_HOLD));   // the pilot flips it again / パイロットが入れ直す
    ASSERT_TRUE(!fixture.manager.isModeRequestConsumed());
    ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::None));
}

TEST(consumed_mode_request_notice_ends_on_the_ground)
{
    Fixture fixture;
    fixture.driveTo(FlightState::FLYING, FlightMode::STABILIZE);
    fixture.setToFSilent(true);
    fixture.manager.requestModeChange(FlightMode::ALT_HOLD);
    ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::TofSwitchConsumed));
    fixture.manager.requestEmergencyStop();
    fixture.updateNow();
    ASSERT_TRUE(fixture.publishedMode().mode_block == static_cast<uint8_t>(ModeBlock::None));
}

// =============================================================================
// Callbacks and the published system_mode
// コールバックと発行される system_mode
// =============================================================================

TEST(callbacks_run_exit_then_enter_with_both_endpoints)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    std::vector<std::string> log;
    fixture.manager.onExit([&](FlightState from, FlightState to) {
        log.push_back(std::string("exit ") + flightStateName(from) + ">" + flightStateName(to) + " state=" +
                      flightStateName(fixture.manager.getState()));
    });
    fixture.manager.onEnter([&](FlightState from, FlightState to) {
        log.push_back(std::string("enter ") + flightStateName(from) + ">" + flightStateName(to) + " state=" +
                      flightStateName(fixture.manager.getState()));
    });
    ASSERT_TRUE(fixture.manager.requestArm());
    ASSERT_TRUE(log.size() == 2);
    ASSERT_MSG(log[0] == "exit IDLE_GROUND>ARMED_GROUND state=IDLE_GROUND", "%s", log[0].c_str());   // before the state changes / 状態が変わる前
    ASSERT_MSG(log[1] == "enter IDLE_GROUND>ARMED_GROUND state=ARMED_GROUND", "%s", log[1].c_str());   // after / 後
}

TEST(callbacks_do_not_run_for_refused_requests)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    int calls = 0;
    fixture.manager.onExit([&](FlightState, FlightState) { ++calls; });
    fixture.manager.onEnter([&](FlightState, FlightState) { ++calls; });
    fixture.manager.notifyTakeoff();            // refused: not ARMED_GROUND / 拒否: ARMED_GROUND でない
    fixture.manager.notifyLandingComplete();    // refused: not LANDING / 拒否: LANDING でない
    ASSERT_TRUE(!fixture.manager.requestDisarm());
    fixture.publishVoltage(kUsbRailVoltageV);
    ASSERT_TRUE(!fixture.manager.requestArm());
    ASSERT_MSG(calls == 0, "%d callbacks ran for refused requests", calls);
}

TEST(mode_callbacks_receive_old_and_new_mode)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    std::vector<std::pair<FlightMode, FlightMode>> changes;
    fixture.manager.onModeChange([&](FlightMode old_mode, FlightMode new_mode) { changes.push_back({old_mode, new_mode}); });
    fixture.manager.requestModeChange(FlightMode::ALT_HOLD);
    fixture.manager.requestModeChange(FlightMode::ALT_HOLD);   // already in it: no callback / 既にそのモード: コールバックなし
    ASSERT_TRUE(changes.size() == 1);
    ASSERT_TRUE(changes[0].first == FlightMode::STABILIZE && changes[0].second == FlightMode::ALT_HOLD);
}

TEST(callback_registration_is_limited)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    int enter_calls = 0;
    for (int index = 0; index < kMaxCallbacks + 1; ++index) {
        fixture.manager.onEnter([&](FlightState, FlightState) { ++enter_calls; });
    }
    ASSERT_TRUE(fixture.manager.requestArm());
    ASSERT_MSG(enter_calls == kMaxCallbacks, "%d callbacks ran, the limit is %d", enter_calls, kMaxCallbacks);
}

TEST(system_mode_publishes_state_mode_and_entry_time)
{
    Fixture fixture;
    fixture.driveTo(FlightState::IDLE_GROUND);
    fixture.manager.requestModeChange(FlightMode::ALT_HOLD);
    sils::compat::advance_virtual_time_us(kTickUs);
    ASSERT_TRUE(fixture.manager.requestArm());
    const uint32_t armed_at_us = fixture.nowUs();
    SystemMode published = fixture.publishedMode();
    ASSERT_TRUE(published.state == static_cast<uint8_t>(FlightState::ARMED_GROUND));
    ASSERT_TRUE(published.sub_mode == static_cast<uint8_t>(FlightMode::ALT_HOLD));
    ASSERT_TRUE(published.armed);
    ASSERT_TRUE(published.state_since_us == armed_at_us);

    sils::compat::advance_virtual_time_us(kTickUs);
    fixture.manager.requestModeChange(FlightMode::POS_HOLD);   // not a state change: the entry time stays / 状態遷移ではない: 入った時刻は変わらない
    published = fixture.publishedMode();
    ASSERT_TRUE(published.sub_mode == static_cast<uint8_t>(FlightMode::POS_HOLD));
    ASSERT_TRUE(published.state_since_us == armed_at_us);
}

// =============================================================================
// Runner
// 実行部
// =============================================================================

static int silentLogOutput(const char*, va_list)
{
    return 0;
}

static bool runCase(const TestCase& test_case)
{
    printf("  [TEST] %-82s ", test_case.name);
    try {
        test_case.body();
        printf("PASS\n");
        return true;
    } catch (const AssertionFailure&) {
        printf("  FAIL  %s\n", test_case.name);
        return false;
    }
}

int main()
{
    esp_log_set_vprintf(silentLogOutput);   // the firmware log lines are not part of the test output / ファームのログ行は試験の出力に含めない
    printf("=== StateManager host tests (normative table) ===\n");
    int passed = 0;
    int failed = 0;
    int skipped = 0;
    for (const TestCase& test_case : registry()) {
        if (test_case.disabled_reason != nullptr) {
            printf("  [SKIP] %-82s %s\n", test_case.name, test_case.disabled_reason);
            ++skipped;
        } else if (runCase(test_case)) {
            ++passed;
        } else {
            ++failed;
        }
    }
    printf("\n=== %d run, %d passed, %d failed, %d skipped ===\n", passed + failed, passed, failed, skipped);
    return failed == 0 ? 0 : 1;
}
