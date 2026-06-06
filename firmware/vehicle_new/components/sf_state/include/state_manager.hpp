/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_manager.hpp
 * @brief Centralized state management with transition callbacks
 *        状態遷移コールバック付き一元的状態管理
 *
 * The StateManager is the sole authority for mode transitions.
 * No other component may change the flight state directly.
 * Transitions trigger onExit/onEnter callbacks that consolidate
 * all reset processing in one place.
 *
 * StateManagerはモード遷移の唯一の権限者。
 * 他のコンポーネントがフライト状態を直接変更してはならない。
 * 遷移時にonExit/onEnterコールバックが発火し、
 * 全リセット処理を1箇所に集約する。
 *
 * @design requirements.md §4 — Component #3: State Management         [--]
 * @design architecture.md §2 — State Management: sole transition owner [--]
 * @design architecture.md §4 — onExit/onEnter callbacks               [--]
 * @design detailed_design.md §3 — State transition table              [--]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [--]
 */

#pragma once

#include <functional>
#include "flight_state.hpp"
#include "topics.hpp"

namespace sf {

// =============================================================================
// Transition Callback Types
// 遷移コールバック型
// =============================================================================

/// Callback invoked on a transition BEFORE the state changes. Receives both endpoints
/// (from, to) so a handler can act on the specific transition pair — e.g.
/// "FLYING → IDLE_GROUND" resets the ESKF but "ARMED_GROUND → IDLE_GROUND" does not.
/// 状態遷移時（状態変更の前）に呼ばれるコールバック。両端 (from, to) を受け取り、特定の遷移
/// ペアに応じた処理ができる（例: FLYING→IDLE_GROUND は ESKF reset するが ARMED_GROUND→
/// IDLE_GROUND はしない）。
using OnExitCallback = std::function<void(FlightState from, FlightState to)>;

/// Callback invoked on a transition AFTER the state changes. Receives both endpoints
/// (from, to) for the same reason as OnExitCallback.
/// 状態遷移時（状態変更の後）に呼ばれるコールバック。OnExitCallback と同じ理由で両端を受け取る。
using OnEnterCallback = std::function<void(FlightState from, FlightState to)>;

/// Callback invoked when flight mode changes within FLYING
/// FLYING内でフライトモードが変わった時に呼ばれるコールバック
using OnModeChangeCallback = std::function<void(FlightMode old_mode, FlightMode new_mode)>;

// =============================================================================
// StateManager
// 状態管理クラス
// =============================================================================

class StateManager {
public:
    /// Initialize the state manager (starts in INIT state)
    /// 状態管理を初期化する（INIT状態から開始）
    ///
    /// @design detailed_design.md §3 — Initial state: INIT            [--]
    void init();

    // =========================================================================
    // State Query
    // 状態参照
    // =========================================================================

    /// Get the current flight state
    /// 現在のフライト状態を取得する
    FlightState getState() const { return state_; }

    /// Get the current flight mode (valid only when FLYING)
    /// 現在のフライトモードを取得する（FLYING時のみ有効）
    FlightMode getMode() const { return mode_; }

    /// Check if the vehicle is armed
    /// 機体がARMされているか確認する
    bool isArmed() const { return sf::isArmed(state_); }

    // =========================================================================
    // Transition Requests
    // 遷移リクエスト
    //
    // These methods validate preconditions before executing transitions.
    // Invalid requests are silently ignored (logged at debug level).
    //
    // 遷移前に前提条件を検証する。
    // 無効なリクエストは無視される（デバッグレベルでログ出力）。
    // =========================================================================

    /// Notify initialization complete (INIT → IDLE_GROUND). Called once by StateTask
    /// after the sensors + estimator are up and producing valid output. No-op unless
    /// still in INIT. 初期化完了通知（INIT → IDLE_GROUND）。センサ＋推定器が立ち上がり
    /// 有効出力を出した後に StateTask が1回呼ぶ。INIT 以外では無操作。
    ///
    /// @design requirements.md §2 — INIT → IDLE_GROUND on init complete [--]
    void notifyInitComplete();

    /// Request ARM (IDLE_GROUND → ARMED_GROUND)
    /// ARMリクエスト（IDLE_GROUND → ARMED_GROUND）
    ///
    /// @return true if transition succeeded
    ///
    /// @design requirements.md §2 — ARM from GROUND only              [--]
    /// @design requirements.md §9 — USB power: ARM prohibited         [--]
    bool requestArm();

    /// Request DISARM (ARMED_GROUND → IDLE_GROUND)
    /// DISARMリクエスト（ARMED_GROUND → IDLE_GROUND）
    ///
    /// @return true if transition succeeded
    bool requestDisarm();

    /// Notify takeoff detected (ARMED_GROUND → TAKEOFF)
    /// 離陸検出通知（ARMED_GROUND → TAKEOFF）
    void notifyTakeoff();

    /// Notify takeoff complete (TAKEOFF → FLYING)
    /// 離陸完了通知（TAKEOFF → FLYING）
    void notifyTakeoffComplete();

    /// Notify landing requested (FLYING → LANDING)
    /// 着陸リクエスト通知（FLYING → LANDING）
    void notifyLandingRequest();

    /// Notify landing complete (LANDING → IDLE_GROUND)
    /// 着陸完了通知（LANDING → IDLE_GROUND）
    void notifyLandingComplete();

    /// Notify soft landing (FLYING → ARMED_GROUND)
    /// ソフトランディング通知（FLYING → ARMED_GROUND）
    ///
    /// @design requirements.md §2 — Soft landing / touch-and-go       [--]
    void notifySoftLanding();

    /// Notify idle ground/held transition based on ToF
    /// ToFに基づくIDLE地上/手持ち遷移通知
    ///
    /// @design requirements.md §2 — IDLE_GROUND ↔ IDLE_HELD by ToF    [--]
    void notifyIdleGroundHeld(bool is_held);

    /// Request flight mode change within FLYING
    /// FLYING内のフライトモード変更リクエスト
    ///
    /// @return true if mode change succeeded
    bool requestModeChange(FlightMode new_mode);

    /// Handle system alert from failsafe
    /// フェイルセーフからのシステムアラートを処理する
    ///
    /// @design architecture.md §4 — FAILSAFE as event                 [--]
    /// @design requirements.md §9 — Safety requirements               [--]
    void handleAlert(const SystemAlert& alert);

    /// Force transition to IDLE_GROUND (emergency use only)
    /// IDLE_GROUNDへ強制遷移（緊急用のみ）
    void forceIdle();

    // =========================================================================
    // Callback Registration
    // コールバック登録
    //
    // Register callbacks to be notified of state/mode transitions.
    // Multiple callbacks can be registered; they are called in order.
    //
    // 状態/モード遷移の通知を受けるコールバックを登録する。
    // 複数登録可能、登録順に呼ばれる。
    // =========================================================================

    /// Register an onExit callback
    /// onExitコールバックを登録する
    void onExit(OnExitCallback callback);

    /// Register an onEnter callback
    /// onEnterコールバックを登録する
    void onEnter(OnEnterCallback callback);

    /// Register a flight mode change callback
    /// フライトモード変更コールバックを登録する
    void onModeChange(OnModeChangeCallback callback);

private:
    /// Execute state transition with callbacks
    /// コールバック付きで状態遷移を実行する
    void transition(FlightState new_state);

    /// Publish current state to system.mode topic
    /// 現在の状態をsystem.modeトピックに発行する
    void publishMode();

    // Current state
    // 現在の状態
    FlightState state_ = FlightState::INIT;
    FlightMode mode_ = FlightMode::STABILIZE;

    // Callback lists
    // コールバックリスト
    static constexpr int MAX_CALLBACKS = 8;

    OnExitCallback exit_callbacks_[MAX_CALLBACKS] = {};
    int exit_callback_count_ = 0;

    OnEnterCallback enter_callbacks_[MAX_CALLBACKS] = {};
    int enter_callback_count_ = 0;

    OnModeChangeCallback mode_callbacks_[MAX_CALLBACKS] = {};
    int mode_callback_count_ = 0;
};

}  // namespace sf
