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
 * @design requirements.md §4 — Component #3: State Management         [OK]
 * @design architecture.md §2 — State Management: sole transition owner [OK]
 * @design architecture.md §4 — onExit/onEnter callbacks               [OK]
 * @design detailed_design.md §3 — State transition table              [OK]
 * @design coding_and_education.md §2 — 1 function 1 responsibility    [OK]
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
    /// @design detailed_design.md §3 — Initial state: INIT            [OK]
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

    /// Get the current pairing state (parallel state machine, see requestPairing).
    /// 現在のペアリング状態を取得する（並行状態機械、requestPairing 参照）。
    PairingState getPairingState() const { return pairing_state_; }

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
    /// @design requirements.md §2 — INIT → IDLE_GROUND on init complete [OK]
    void notifyInitComplete();

    /// Request ARM (IDLE_GROUND → ARMED_GROUND)
    /// ARMリクエスト（IDLE_GROUND → ARMED_GROUND）
    ///
    /// @return true if transition succeeded
    ///
    /// @design requirements.md §2 — ARM from GROUND only              [OK]
    /// @design requirements.md §9 — USB power: ARM prohibited         [OK]
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
    /// @design requirements.md §2 — Soft landing / touch-and-go       [OK]
    void notifySoftLanding();

    /// Notify idle ground/held transition based on ToF
    /// ToFに基づくIDLE地上/手持ち遷移通知
    ///
    /// @design requirements.md §2 — IDLE_GROUND ↔ IDLE_HELD by ToF    [OK]
    void notifyIdleGroundHeld(bool is_held);

    /// Request flight mode change within FLYING
    /// FLYING内のフライトモード変更リクエスト
    ///
    /// @return true if mode change succeeded
    bool requestModeChange(FlightMode new_mode);

    /// Handle system alert from failsafe
    /// フェイルセーフからのシステムアラートを処理する
    ///
    /// @design architecture.md §4 — FAILSAFE as event                 [OK]
    /// @design requirements.md §9 — Safety requirements               [OK]
    void handleAlert(const SystemAlert& alert);

    // =========================================================================
    // Pairing — parallel state machine (NotPaired / Pairing / Paired)
    // ペアリング — 並行状態機械（NotPaired / Pairing / Paired）
    //
    // Independent of FlightState; this manager is its single owner (architecture
    // §4). sf_comm EXECUTES the radio side (broadcast/learn/filter) and reports
    // the bind status fact; this manager DECIDES the PairingState and publishes it.
    // FlightState とは独立。本マネージャが単一所有する（architecture §4）。sf_comm が
    // 無線側（送出/学習/フィルタ）を実行しバインド状態の事実を報告、本マネージャが
    // PairingState を判断して発行する。
    // =========================================================================

    /// Request entering Pairing (search). Valid only on the ground (IDLE_GROUND /
    /// IDLE_HELD) — used for auto-enter when unpaired and for button re-pairing.
    /// Rejected while INIT / armed / airborne. Idempotent if already Pairing.
    /// Pairing（探索）への突入要求。地上（IDLE_GROUND / IDLE_HELD）でのみ有効 — 未ペア時の
    /// 自動突入とボタン再ペアに使う。INIT/武装/空中では拒否。既に Pairing なら冪等。
    ///
    /// @design requirements.md §2 — Pairing on the ground only          [OK]
    void requestPairing();

    /// Reflect that sf_comm has bound to a controller → Paired. Idempotent.
    /// sf_comm が相手にバインドした事実を反映する → Paired。冪等。
    ///
    /// @design requirements.md §2 — Pairing → Paired on bind            [OK]
    void notifyPairingComplete();

    /// Periodic update for TIME-DEFERRED transitions (call once per StateTask cycle).
    /// Today this drives the comm-loss failsafe: a COMM_LOST while FLYING does not land
    /// immediately — it arms a timer (handleAlert) and this method commands the
    /// FLYING → LANDING transition only after the hover grace period elapses
    /// (requirements §9: "hover hold 3 s → auto landing"). Kept separate from
    /// handleAlert because the failsafe raises COMM_LOST only once (rising edge), so the
    /// elapsed-time check needs an independent periodic tick, not another alert.
    /// 時間遅延つき遷移の周期更新（StateTask の各サイクルで1回呼ぶ）。現状は通信断
    /// フェイルセーフを駆動する: FLYING 中の COMM_LOST は即着陸せず、タイマを起動
    /// （handleAlert）し、ホバー猶予が経過してから本メソッドが FLYING → LANDING を
    /// 指令する（要件§9「ホバー維持3秒→自動着陸」）。failsafe は COMM_LOST を立ち上がり
    /// エッジで1回だけ発報するので、経過判定にはアラートでなく独立した周期ティックが要る。
    ///
    /// @param now_us  current time [us] (esp_timer_get_time) supplied by the caller
    /// @design requirements.md §9 — comm loss: hover 3 s → LANDING     [OK]
    /// @design architecture.md §4 — FAILSAFE as event                 [OK]
    void update(uint32_t now_us);

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

    /// Publish current PairingState to the pairing_state topic (comm/notify read it)
    /// 現在の PairingState を pairing_state トピックに発行する（comm/notify が読む）
    void publishPairingState();

    // Current state
    // 現在の状態
    FlightState state_ = FlightState::INIT;
    FlightMode mode_ = FlightMode::STABILIZE;

    // Pairing state (parallel to FlightState). Owned here; comm reflects it.
    // ペアリング状態（FlightState と並行）。ここが所有し comm が反映する。
    PairingState pairing_state_ = PairingState::NotPaired;

    // Comm-loss failsafe timer (requirements §9: hover hold 3 s → auto landing).
    // Armed by handleAlert(COMM_LOST) while FLYING; update() lands once the grace
    // period from comm_lost_time_us_ elapses. Cleared on landing or on leaving FLYING.
    // 通信断フェイルセーフのタイマ（要件§9: ホバー維持3秒→自動着陸）。FLYING 中の
    // handleAlert(COMM_LOST) で起動し、comm_lost_time_us_ から猶予経過で update() が
    // 着陸させる。着陸時または FLYING を外れた時にクリアする。
    bool     comm_lost_pending_ = false;
    uint32_t comm_lost_time_us_ = 0;

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
