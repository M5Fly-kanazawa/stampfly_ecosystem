/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (host SIL).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file scenario.hpp
 * @brief Flight scenario selection for the integrated SIL
 *        統合SIL用のフライトシナリオ選択
 *
 * A scenario selects which fault (if any) is injected on top of the common
 * nominal flight progression (ground → arm → takeoff → hover → land). The
 * integrated_sil loop owns the actual injection timing; this header only
 * defines the enum and CLI parsing so the loop stays readable.
 *
 * シナリオは、共通の正常フライト進行（地上→ARM→離陸→ホバー→着陸）に
 * 重畳して注入する障害（あれば）を選ぶ。注入タイミングは integrated_sil の
 * ループが持つ。本ヘッダは enum と CLI 解析のみを定義し、ループを読みやすく保つ。
 */

#pragma once

#include <cstring>

namespace sf {
namespace sil {

/// Fault scenario injected on top of nominal flight
/// 正常フライトに重畳する障害シナリオ
enum class Scenario {
    NOMINAL,        // No fault: full takeoff → hover → land / 障害なし
    COMM_LOST,      // Comm timeout while FLYING → auto-land / 飛行中の通信途絶 → 自動着陸
    IMPACT,         // High-G impact while airborne → emergency idle / 空中衝撃 → 緊急IDLE
    LOW_BATTERY,    // Battery sag → low-battery warning / 電圧低下 → 低電圧警告
    ESKF_DIVERGED,  // Estimator divergence alert (state unchanged) / 推定器発散アラート（状態不変）
};

/// Parse a scenario name from the CLI (defaults to NOMINAL)
/// CLI からシナリオ名を解析する（既定は NOMINAL）
inline Scenario parseScenario(const char* name)
{
    if (!name) return Scenario::NOMINAL;
    if (std::strcmp(name, "comm_lost") == 0)     return Scenario::COMM_LOST;
    if (std::strcmp(name, "impact") == 0)        return Scenario::IMPACT;
    if (std::strcmp(name, "low_battery") == 0)   return Scenario::LOW_BATTERY;
    if (std::strcmp(name, "eskf_diverged") == 0) return Scenario::ESKF_DIVERGED;
    return Scenario::NOMINAL;
}

/// Human-readable scenario name
/// 可読なシナリオ名
inline const char* scenarioName(Scenario s)
{
    switch (s) {
        case Scenario::COMM_LOST:     return "comm_lost";
        case Scenario::IMPACT:        return "impact";
        case Scenario::LOW_BATTERY:   return "low_battery";
        case Scenario::ESKF_DIVERGED: return "eskf_diverged";
        default:                      return "nominal";
    }
}

}  // namespace sil
}  // namespace sf
