/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
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
 * 遷移時にonExit/onEnterコールバックが呼び出され、
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

    /// Request DISARM. On the ground / in ACRO·STABILIZE it is an immediate motor cut
    /// (→ IDLE_GROUND). In ALT_HOLD/POS_HOLD while FLYING it starts an AUTO-LANDING
    /// (→ LANDING, gradual descent) instead of dropping the craft; touchdown then disarms.
    /// A DISARM pressed again while already LANDING is the emergency cut (see below).
    /// DISARM リクエスト。地上 / ACRO・STABILIZE では即モータ停止（→IDLE_GROUND）。
    /// ALT_HOLD/POS_HOLD の FLYING 中は機体を落とさず自動着陸を開始（→LANDING, 緩降下）し、
    /// 接地で DISARM する。LANDING 中の再 DISARM は緊急カット（下記）。
    ///
    /// @return true if transition succeeded
    bool requestDisarm();

    /// Emergency stop: unconditional immediate motor cut from any armed state
    /// (→ IDLE_GROUND), bypassing the ALT/POS auto-landing. For the API `emergency`
    /// verb and for aborting an in-progress auto-landing.
    /// 緊急停止: 任意の armed 状態から無条件で即モータ停止（→IDLE_GROUND）。ALT/POS の
    /// 自動着陸を迂回する。API `emergency` verb と自動着陸の中断用。
    ///
    /// @return true if transition succeeded
    bool requestEmergencyStop();

    /// Notify takeoff detected (ARMED_GROUND → TAKEOFF)
    /// 離陸検出通知（ARMED_GROUND → TAKEOFF）
    void notifyTakeoff();

    /// Tell the manager that a bench motor test has run and its estimator re-level is
    /// still owed (set by StateTask). While set, or while a motor test is active, ARM is
    /// rejected so the craft never flies on an estimate shaken off by the test.
    /// ベンチ用モータテストが走り、推定器の再水平化が未実施であることを通知する
    /// （StateTask が設定）。設定中、またはモータテスト実行中は ARM を拒否し、テストで
    /// 外れた推定のまま飛ばないようにする。
    ///
    /// @design detailed_design.md §3 注9 — ARM gated on bench-test re-level [OK]
    void setBenchRelevelPending(bool pending) { bench_relevel_pending_ = pending; }

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

    // =========================================================================
    // Flip maneuver — dedicated sequence state, same shape as TAKEOFF/LANDING
    // (FLYING → FLIP → FLYING). See flight_state.hpp::FlightState::FLIP.
    // 宙返りマニューバ — TAKEOFF/LANDING と同型の専用シーケンス状態
    // （FLYING → FLIP → FLYING）。flight_state.hpp::FlightState::FLIP 参照。
    //
    // @design docs/plans/flip-maneuver-plan.md §4.1 — FlightState::FLIP   [OK]
    // @design docs/plans/flip-maneuver-plan.md §4.3 — normative row       [OK]
    // =========================================================================

    /// Request Flip (FLYING → FLIP). Accepted ONLY while FLYING — this is the C1
    /// gate (detailed_design.md §3.1 FLIP row / plan §3.1); execution conditions
    /// C2-C8 (altitude, attitude/rate, velocity, battery, estimator health,
    /// cooldown, source conflict) are judged by the controller every cycle while
    /// FLYING (ControllerStatus.flip_ready/flip_block_reason) and are NOT
    /// re-checked here — the caller (state_task, on ApiCmd::Flip / the FLIP
    /// button) is expected to gate on flip_ready first. Rejected while already
    /// FLIP (busy) or in any other state (not flying) — a single guard covers
    /// both cells of the normative table.
    /// Flip リクエスト（FLYING → FLIP）。FLYING 中のみ受理 — これが実行条件 C1 の判定
    /// （detailed_design.md §3.1 FLIP 行 / plan §3.1）。実行条件 C2-C8（高度・姿勢/
    /// 角速度・速度・電池・推定器健全性・間隔・他ソースとの競合）は制御器が FLYING 中
    /// 毎周期判定し（ControllerStatus.flip_ready/flip_block_reason）、ここでは
    /// 再判定しない — 呼び出し側（state_task、ApiCmd::Flip / FLIP ボタン）が事前に
    /// flip_ready を見る想定。既に FLIP 中（busy）でもその他の状態（not flying）でも
    /// 拒否 — 単一の判定で規範表の両セルを兼ねる。
    ///
    /// @return true if the transition to FLIP succeeded
    ///
    /// @design detailed_design.md §3.1 — FLIP row, "Flip request" column  [OK]
    /// @design docs/plans/flip-maneuver-plan.md §4.3                     [OK]
    bool requestFlip(FlipDirection direction);

    /// Get the direction captured by the most recent accepted requestFlip() call.
    /// Valid once FLIP has been entered; read by the onEnter(FLIP) callback to
    /// publish ControllerCmd::Flip{direction} (state_task.cpp).
    /// 直近に受理された requestFlip() が捕捉した方向。FLIP 突入後に有効。onEnter(FLIP)
    /// コールバックが ControllerCmd::Flip{direction} を発行する際に読む（state_task.cpp）。
    FlipDirection getFlipDirection() const { return flip_direction_; }

    /// Notify flip complete (FLIP → FLYING): the controller finished the maneuver
    /// (success) or aborted mid-sequence and recovered to level flight — both are
    /// reported as controller_status.flip_done (architecture.md §4 INV-3:
    /// detection is the controller's fact, this method executes the judged
    /// transition). Re-evaluates any battery emergency that was deferred while
    /// FLIP was in progress (handleAlert, plan §3.5/§9 item 2).
    /// フリップ完了通知（FLIP → FLYING）: 制御器がマニューバを完了（成功）または
    /// シーケンス途中で打ち切って水平飛行へ回復した — どちらも controller_status.
    /// flip_done として報告される（architecture.md §4 INV-3: 検出は制御器の事実、
    /// 本メソッドが判断された遷移を実行）。FLIP 中に保留した電池緊急アラートを
    /// ここで再評価する（handleAlert、plan §3.5/§9 項目2）。
    ///
    /// @design docs/plans/flip-maneuver-plan.md §4.3 — FLIP → FLYING row  [OK]
    void notifyFlipComplete();

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

    /// Request entering Pairing (search). Valid in IDLE_GROUND or IDLE_HELD (on the
    /// ground or held in hand, 2026-09-12) — used for auto-enter when unpaired and for
    /// button re-pairing. Rejected while INIT / armed / airborne. Idempotent if already
    /// Pairing. Does not relax the ARM guard: requestArm() still accepts IDLE_GROUND
    /// only, and ARM is separately rejected while Pairing.
    /// Pairing（探索）への突入要求。IDLE_GROUND または IDLE_HELD（地上または手持ち、
    /// 2026-09-12）で有効 — 未ペア時の自動突入とボタン再ペアに使う。INIT/武装/空中では
    /// 拒否。既に Pairing なら冪等。ARM のガードは緩めない: requestArm() は引き続き
    /// IDLE_GROUND のみ受理し、Pairing 中は ARM を別途拒否する。
    ///
    /// @design requirements.md §2 — Pairing on IDLE_GROUND / IDLE_HELD  [OK]
    void requestPairing();

    /// Reflect that sf_comm has bound to a controller → Paired. Idempotent.
    /// sf_comm が相手局にバインドした事実を反映する → Paired。冪等。
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

    // A bench motor test has run and its estimator re-level is still owed
    // (see setBenchRelevelPending). Read by the pre-arm gate in requestArm().
    // ベンチ用モータテストが走り、推定器の再水平化が未実施（setBenchRelevelPending 参照）。
    // requestArm() の ARM 前判定が読む。
    bool bench_relevel_pending_ = false;

    // Comm-loss failsafe timer (requirements §9: hover hold 3 s → auto landing).
    // Armed by handleAlert(COMM_LOST) while FLYING; update() lands once the grace
    // period from comm_lost_time_us_ elapses. Cleared on landing or on leaving FLYING.
    // 通信断フェイルセーフのタイマ（要件§9: ホバー維持3秒→自動着陸）。FLYING 中の
    // handleAlert(COMM_LOST) で起動し、comm_lost_time_us_ から猶予経過で update() が
    // 着陸させる。着陸時または FLYING を外れた時にクリアする。
    bool     comm_lost_pending_ = false;
    uint32_t comm_lost_time_us_ = 0;

    // Direction captured by the most recent accepted requestFlip() — read by the
    // onEnter(FLYING, FLIP) callback (state_task.cpp) to publish
    // ControllerCmd::Flip{direction}. Only meaningful while state_ == FLIP.
    // 直近に受理された requestFlip() が捕捉した方向 — onEnter(FLYING, FLIP)
    // コールバック（state_task.cpp）が ControllerCmd::Flip{direction} を発行する際に
    // 読む。state_ == FLIP の間のみ意味を持つ。
    FlipDirection flip_direction_ = FlipDirection::Back;

    // Battery-emergency alert deferred while FLIP is in progress (plan §3.5/§9
    // item 2: "窓内は StateManager が無視... 衝撃判定は維持" extended to the
    // battery-emergency case, §4.3 FLIP row). handleAlert() sets this instead of
    // transitioning straight to LANDING from FLIP; notifyFlipComplete() re-applies
    // the existing rule once back in FLYING. Cleared on any FLIP exit that is NOT
    // FLYING (DISARM/impact/emergency stop already land on the ground, so the
    // deferred alert is moot — see transition()).
    // FLIP 進行中に保留した電池緊急アラート（plan §3.5/§9 項目2:「窓内は StateManager
    // が無視...衝撃判定は維持」を電池緊急にも拡張、§4.3 FLIP 行）。handleAlert() は
    // FLIP から直接 LANDING へ遷移する代わりにこれを立て、notifyFlipComplete() が
    // FLYING 復帰後に既存則を再適用する。FLIP を FLYING 以外へ抜けるとき（DISARM/
    // 衝撃/緊急停止は既に接地するので保留アラートは無意味）はクリアする（transition() 参照）。
    bool     battery_emergency_pending_ = false;

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
