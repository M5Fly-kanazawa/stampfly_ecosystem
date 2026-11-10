/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file battery_alert_policy.hpp
 * @brief When the failsafe raises the low-battery WARNING and EMERGENCY alerts (pure logic)
 *        フェイルセーフが電池の WARNING / EMERGENCY アラートを発報するタイミング（純粋ロジック）
 *
 * Level semantics, like COMM_LOST and TOF_SILENT: the StateManager acts on a battery EMERGENCY
 * only while airborne, so an alert consumed on the ground (or left over from an earlier flight)
 * must not be the only one. While the voltage stays below the critical threshold the EMERGENCY
 * is raised on entry into that region and then again every kEmergencyReraiseEveryNUpdates. After
 * leaving the region (recovery to the warning band, or a voltage that chatters around the
 * threshold) the next entry raises it again, but never closer than kEmergencyMinSpacingUpdates to
 * the previous one, so a chattering voltage cannot flood the depth-4 system_alert queue.
 * The WARNING is raised on entry into the band below the low threshold and re-raised every
 * kWarningReraiseEveryNUpdates; it is latched until the voltage rises clearly above the
 * threshold (kRecoveryHysteresisV).
 * Pure function of the voltage sequence (no ESP-IDF, no topics): checked on the host in
 * test/test_battery_alert_policy.cpp. Failsafe::checkBattery() calls it and publishes the result.
 * COMM_LOST・TOF_SILENT と同じレベル意味論: StateManager は電池の EMERGENCY を空中でしか処理しない
 * ので、地上で消費された（または前の飛行から残った）アラートが唯一であってはならない。電圧が危険
 * 閾値を下回っている間、EMERGENCY はその領域へ入ったときに発報し、以後 kEmergencyReraiseEveryNUpdates
 * ごとに再発報する。領域を出た後（警告帯への回復、閾値まわりで電圧がばたつく場合）の次の突入でも
 * 再び発報するが、前回の発報から kEmergencyMinSpacingUpdates より近づけず、電圧のばたつきが深さ 4 の
 * system_alert キューを溢れさせないようにする。WARNING は低電圧閾値を下回る帯へ入ったときに発報し、
 * kWarningReraiseEveryNUpdates ごとに再発報する。電圧が閾値を明確に上回る（kRecoveryHysteresisV）まで
 * ラッチされる。
 * 電圧列だけの純関数（ESP-IDF・トピック不要）で、test/test_battery_alert_policy.cpp でホスト検査する。
 * Failsafe::checkBattery() が呼び、結果を発行する。
 *
 * @design requirements.md §9 — low battery warning / emergency landing   [OK]
 * @design architecture.md §3 — R16 level semantics of failsafe alerts    [OK]
 */

#pragma once

#include <cstdint>

namespace sf {

/// What Failsafe::checkBattery() must raise after one voltage sample.
/// 1 回の電圧サンプルの後に Failsafe::checkBattery() が発報すべきもの。
enum class BatteryAlertAction : uint8_t {
    None,        ///< nothing to raise / 発報なし
    Warning,     ///< raise LOW_BATTERY / WARNING
    Emergency,   ///< raise LOW_BATTERY / EMERGENCY
};

/// Decides the battery alerts from the 10 Hz voltage sequence.
/// 10 Hz の電圧列から電池アラートを決める。
class BatteryAlertPolicy {
public:
    /// The voltage must rise this far above the low threshold [V] before the warning latch clears.
    /// 警告のラッチが解けるために、電圧が低電圧閾値をこれだけ上回る必要がある [V]。
    static constexpr float kRecoveryHysteresisV = 0.1f;

    /// Re-raise period of a standing WARNING [updates]: 50 updates = about 5 s at the 10 Hz rate.
    /// A single in-flight warble is missed over motor noise, so it repeats.
    /// 継続中の WARNING の再発報周期 [更新回数]: 10 Hz で 50 回 = 約 5 s。飛行中の単発の
    /// 警告音はモータ騒音に埋もれるので繰り返す。
    static constexpr uint8_t kWarningReraiseEveryNUpdates = 50;

    /// Re-raise period of a standing EMERGENCY [updates]: 50 updates = about 5 s.
    /// 継続中の EMERGENCY の再発報周期 [更新回数]: 50 回 = 約 5 s。
    static constexpr uint8_t kEmergencyReraiseEveryNUpdates = 50;

    /// Shortest spacing between two EMERGENCY alerts [updates]: 20 updates = 2 s. Bounds the rate
    /// when the voltage chatters around the critical threshold (the system_alert queue holds 4).
    /// 2 つの EMERGENCY の最短間隔 [更新回数]: 20 回 = 2 s。電圧が危険閾値まわりでばたつくときの
    /// 発報の頻度を抑える（system_alert キューは 4 件）。
    static constexpr uint8_t kEmergencyMinSpacingUpdates = 20;

    /// Feed one voltage sample [V] (call at the failsafe rate, 10 Hz) with the thresholds [V].
    /// 電圧サンプル [V] を 1 つ与える（フェイルセーフの周期 10 Hz で呼ぶ）。閾値 [V] も渡す。
    BatteryAlertAction update(float voltage, float low_threshold_v, float critical_threshold_v)
    {
        if (updates_since_emergency_ < kEmergencyReraiseEveryNUpdates) {
            ++updates_since_emergency_;
        }
        if (voltage < critical_threshold_v) {
            return updateCritical();
        }
        emergency_raised_in_region_ = false;   // left the critical region / 危険領域を出た
        if (voltage < low_threshold_v) {
            return updateWarning();
        }
        if (voltage > low_threshold_v + kRecoveryHysteresisV) {
            warning_latched_        = false;
            warning_updates_since_  = 0;
        }
        return BatteryAlertAction::None;
    }

    /// True while the WARNING latch is set (for the host tests).
    /// WARNING のラッチが立っている間 true（ホスト試験用）。
    bool warningLatched() const { return warning_latched_; }

private:
    // Below the critical threshold: raise on entry (after the minimum spacing) and re-raise on the cadence.
    // 危険閾値の下: 突入時（最短間隔の後）に発報し、周期で再発報する。
    BatteryAlertAction updateCritical()
    {
        warning_latched_       = true;   // the warning band is below us too / 警告帯もこの下にある
        warning_updates_since_ = 0;
        const uint8_t required = emergency_raised_in_region_ ? kEmergencyReraiseEveryNUpdates
                                                             : kEmergencyMinSpacingUpdates;
        if (updates_since_emergency_ < required) {
            return BatteryAlertAction::None;
        }
        updates_since_emergency_    = 0;
        emergency_raised_in_region_ = true;
        return BatteryAlertAction::Emergency;
    }

    // In the band below the low threshold: raise on entry, re-raise on the cadence.
    // 低電圧閾値より下の帯: 突入時に発報し、周期で再発報する。
    BatteryAlertAction updateWarning()
    {
        if (!warning_latched_) {
            warning_latched_       = true;
            warning_updates_since_ = 0;
            return BatteryAlertAction::Warning;
        }
        if (++warning_updates_since_ >= kWarningReraiseEveryNUpdates) {
            warning_updates_since_ = 0;
            return BatteryAlertAction::Warning;
        }
        return BatteryAlertAction::None;
    }

    bool    warning_latched_            = false;   ///< WARNING raised, not yet recovered / WARNING 発報済み・未回復
    uint8_t warning_updates_since_      = 0;       ///< updates since the last WARNING / 前回の WARNING からの更新回数
    // Starts saturated so that the first entry raises at once.
    // 飽和した値から始め、最初の突入で直ちに発報する。
    uint8_t updates_since_emergency_    = kEmergencyReraiseEveryNUpdates;
    bool    emergency_raised_in_region_ = false;   ///< raised since entering the current critical region / 現在の危険領域に入ってから発報済み
};

}  // namespace sf
