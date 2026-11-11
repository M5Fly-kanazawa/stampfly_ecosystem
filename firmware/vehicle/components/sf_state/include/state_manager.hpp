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
#include "sensor_freshness.hpp"
#include "failsafe_status_guard.hpp"   // FailsafeStatusGuard: a stale failsafe_status reads as ToF silent
#include "tof_silence_policy.hpp"      // LandingClock: time since the state entered LANDING
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

    /// ARM for an API takeoff: switch to `takeoff_mode` (the auto-takeoff mode the API asks for),
    /// then ARM from IDLE_GROUND. When the ARM is refused (pre-arm gates: tilt, battery, pairing,
    /// calibration ...) the flight mode that was active before the call is put back, so a refused
    /// takeoff leaves no trace: without an RC packet nothing re-arbitrates the mode on the ground,
    /// and a later ARM (the on-board button) would otherwise start an automatic takeoff in the
    /// mode the refused request left behind. In ARMED_GROUND no ARM is requested (the craft is
    /// already armed) and the mode stays as switched.
    /// API 離陸のための ARM: `takeoff_mode`（API が求める自動離陸のモード）へ切り替え、IDLE_GROUND から ARM
    /// する。ARM が拒否されたとき（ARM 前判定: 傾き・電池・ペアリング・校正 …）は、呼び出し前の飛行モードに
    /// 戻す。拒否された離陸が痕跡を残さないようにするため: RC の電文が無いと地上でモードを調停し直す
    /// ものがなく、後の ARM（機体のボタン）が、拒否された要求の残したモードで自動離陸を始めてしまう。
    /// ARMED_GROUND では ARM を要求せず（既に ARM 済み）、切り替えたモードのままにする。
    ///
    /// @return true if the craft is armed after the call
    ///
    /// @design detailed_design.md §3.1 — API takeoff: mode change + ARM  [OK]
    bool requestArmForTakeoff(FlightMode takeoff_mode);

    /// Request DISARM. On the ground, in TAKEOFF (any mode) and in ACRO / STABILIZE it is an immediate motor cut (-> IDLE_GROUND). In ALT_HOLD / POS_HOLD while FLYING
    /// it starts an AUTO-LANDING (-> LANDING, gradual descent) instead of dropping the craft; touchdown then disarms. A DISARM pressed again while already LANDING
    /// is the emergency cut (see below). The ways into LANDING from TAKEOFF are the loss of the link, the battery EMERGENCY, the ToF rules and notifyTakeoffEndedByLanding() (a takeoff that does not lift).
    /// DISARM リクエスト。地上・TAKEOFF（どのモードでも）・ACRO / STABILIZE では即モータ停止（→IDLE_GROUND）。ALT_HOLD / POS_HOLD の FLYING 中は機体を落とさず
    /// 自動着陸を開始（→LANDING, 緩降下）し、接地で DISARM する。LANDING 中の再 DISARM は緊急カット（下記）。TAKEOFF から LANDING に入るのは
    /// 通信の途絶・電池の EMERGENCY・ToF の規則・notifyTakeoffEndedByLanding()（浮けない離陸）。
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

    /// Notify landing requested (FLYING → LANDING; ARMED_GROUND in an altitude mode → immediate DISARM, the motors have only begun to turn on the floor). Ignored in every other state, TAKEOFF included
    /// (the API `land` that arrives in TAKEOFF is held by ApiTask until the TAKEOFF has been left: a parked line during a `takeoff` wait, waited for by cmdLand outside one). From the API `land`.
    /// 着陸リクエスト通知（FLYING → LANDING。高度モードの ARMED_GROUND → 即時 DISARM: モータは床の上で回り始めただけ）。TAKEOFF を含むそれ以外の状態では無視する
    /// （TAKEOFF に届いた API の `land` は、TAKEOFF を抜けるまで ApiTask が保留する: `takeoff` の待機中なら後回しの行、待機の外なら cmdLand が待つ）。API の `land` から。
    void notifyLandingRequest();

    /// Notify that the TAKEOFF ends by a LANDING because it cannot lift (TakeoffAction::Land: the floor evidence is latched, not there now, and the abort time has passed). TAKEOFF → LANDING; ignored in every other state.
    /// TAKEOFF を、浮けないので LANDING で終えると通知する（TakeoffAction::Land: 床の証拠がラッチされ、今は無く、中止の時刻を過ぎた）。TAKEOFF → LANDING。それ以外の状態では無視する。
    ///
    /// @design detailed_design.md §3.1 — TAKEOFF -> LANDING (latch, no evidence now, 5 s) [OK]
    void notifyTakeoffEndedByLanding();

    /// Notify landing complete (LANDING → IDLE_GROUND)
    /// 着陸完了通知（LANDING → IDLE_GROUND）
    void notifyLandingComplete();

    /// How the last LANDING ended (set when it ends, cleared when the next LANDING begins). The two ToF-silent
    /// kinds are logged and kept here so that a touchdown without a ToF is not mistaken for a crash.
    /// 直近の LANDING の終わり方（終わるときに設定し、次の LANDING の開始でクリアする）。ToF 出力停止中の 2 種類は、
    /// ToF なしの接地が墜落と取り違えられないよう、ログに出し、ここに残す。
    enum class LandingEnd : uint8_t {
        None = 0,            ///< LANDING has not ended yet / まだ終わっていない
        Detected,            ///< touchdown detector (ToF alive: ground / stalled; silent: impact-then-still or the floor) / 接地検出器
        ToFSilentImpact,     ///< "ToF なし接地（衝撃）": the 3 G impact while the ToF was silent / ToF 出力停止中の 3 G の衝撃
        ToFSilentTimeout,    ///< time limit while the ToF was silent / ToF 出力停止中の時間切れ
    };
    LandingEnd lastLandingEnd() const { return last_landing_end_; }   ///< see LandingEnd / LandingEnd 参照

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
    /// Today this drives the comm-loss failsafe: a COMM_LOST while TAKEOFF / FLYING does not land
    /// immediately — it arms a timer (handleAlert) and this method commands the
    /// TAKEOFF / FLYING → LANDING transition only after the hover grace period elapses
    /// (requirements §9: "hover hold 3 s → auto landing"). Kept separate from
    /// handleAlert because the failsafe raises COMM_LOST only once (rising edge), so the
    /// elapsed-time check needs an independent periodic tick, not another alert.
    /// 時間遅延つき遷移の周期更新（StateTask の各サイクルで1回呼ぶ）。現状は通信断
    /// フェイルセーフを駆動する: TAKEOFF・FLYING 中の COMM_LOST は即着陸せず、タイマを起動
    /// （handleAlert）し、ホバー猶予が経過してから本メソッドが TAKEOFF・FLYING → LANDING を
    /// 指令する（要件§9「ホバー維持3秒→自動着陸」）。failsafe は COMM_LOST を立ち上がり
    /// エッジで1回だけ発報するので、経過判定にはアラートでなく独立した周期ティックが要る。
    ///
    /// Also runs the downward-ToF silence response every call (handleToFSilence): a ToF
    /// that stops while armed aborts the auto-takeoff / lands the craft, and ARM is refused
    /// on the ground (ArmBlock::TofSilent).
    /// 毎回、下向き ToF の出力停止への応答（handleToFSilence）も実行する: 武装中に ToF が止まれば
    /// 自動離陸を中止/着陸させ、地上では ARM を拒否する（ArmBlock::TofSilent）。
    ///
    /// @param now_us  current time [us] (esp_timer_get_time) supplied by the caller
    /// @design requirements.md §9 — comm loss: hover 3 s → LANDING     [OK]
    /// @design detailed_design.md §3.1 — ToF silent rows               [OK]
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

    /// True when the latest requestModeChange() call was refused IN FLIGHT because of a ToF-silence
    /// episode and its switch edge was therefore CONSUMED (ModeBlock::TofSwitchConsumed): StateTask
    /// must drop the edge instead of retrying it, so the mode never switches by itself when the ToF
    /// recovers; the pilot flips the switch again.
    /// 直近のモード変更要求が、ToF 出力停止のエピソードのため「飛行中」に拒否され、そのスイッチの
    /// エッジが「消費」されたとき true（ModeBlock::TofSwitchConsumed）: StateTask はエッジを再試行
    /// せずに捨てる。ToF が復帰してもモードがパイロットの操作なしに切り替わらず、パイロットがスイッチを入れ直す。
    bool isModeRequestConsumed() const { return last_mode_request_consumed_; }

    /// True while a downward-ToF silence EPISODE is in progress: no new sample for
    /// sf::limits::kTofSilenceUs (or never one), only invalid samples while flying, or the estimator
    /// has not re-acquired the ToF yet. The judgement is Failsafe's (R16); this reads the published
    /// level (failsafe_status.tof_silent) through this task's FailsafeStatusGuard, FRESH at every
    /// call, so a status nobody refreshes reads as silent.
    /// 下向き ToF の出力停止の「エピソード」が進行中なら true: sf::limits::kTofSilenceUs の間新サンプル
    /// 無し（または一度も無い）、飛行中に無効サンプルだけ、または推定器がまだ ToF を取り込み直して
    /// いない。判定は Failsafe のもの（R16）で、ここでは発行済みのレベル（failsafe_status.tof_silent）を
    /// このタスクの FailsafeStatusGuard を通して、呼ぶたびに「最新」を読む。誰も更新しない状態は
    /// 出力停止と読む。
    bool isToFSilent() const;

private:
    /// Execute state transition with callbacks
    /// コールバック付きで状態遷移を実行する
    void transition(FlightState new_state);
    void enterState(FlightState old_state, FlightState new_state);   // switch the state and apply what the entry implies / 状態を切り替え、入ることが意味する処理を行う

    /// Back on the ground: reset the flight mode to STABILIZE and fire the mode callbacks (called by transition() on entering IDLE_GROUND)
    /// 接地に戻った: 飛行モードを STABILIZE にリセットし、モードのコールバックを呼ぶ（transition() が IDLE_GROUND に入るときに呼ぶ）
    void resetModeOnGround();

    /// Publish current state to system.mode topic
    /// 現在の状態をsystem.modeトピックに発行する
    void publishMode();

    /// Evaluate the pre-arm gates; return the first failing one (ArmBlock::None = all pass).
    /// The single source of the ARM-refusal reason (requestArm acts on it, update publishes it).
    /// ARM 前判定を評価し、最初に落ちたものを返す（None=全て通る）。ARM 拒否理由の唯一の源
    /// （requestArm が従い、update が発行する）。
    ArmBlock evaluateArmBlock() const;

    /// Pre-arm gate 1: a valid voltage reading at or below the USB / unsafe threshold.
    /// ARM 前判定1: 有効な電圧読みが USB / 危険閾値以下。
    bool isBatteryUnsafeForArm() const;

    /// Pre-arm gate 4: attitude/gravity mismatch (TiltMismatch) or not yet verified (TiltPending).
    /// ARM 前判定4: 姿勢と重力の不一致（TiltMismatch）または未検証（TiltPending）。
    ArmBlock evaluateAttitudeGate() const;

    /// Log the refusal reason / 拒否理由をログ出力
    void logArmRejected(ArmBlock reason) const;


    /// Execute the state machine's response to an AlertType::TOF_SILENT alert (see
    /// tof_silence_policy.hpp): disarm on the ground / land from an altitude mode / warn only in
    /// ACRO and STABILIZE. Same form as the COMM_LOST handling: Failsafe detects, this decides.
    /// AlertType::TOF_SILENT アラートに対する状態機械の応答を実行する（tof_silence_policy.hpp 参照）:
    /// 地上なら DISARM・高度モードなら着陸・ACRO と STABILIZE では警告のみ。COMM_LOST の扱いと同じ形:
    /// Failsafe が検出し、ここが判断する。
    void handleToFSilenceAlert();

    /// True in the states that accept a flight-mode change (IDLE_GROUND, ARMED_GROUND, FLYING).
    /// フライトモード変更を受理する状態（IDLE_GROUND・ARMED_GROUND・FLYING）なら true。
    bool modeChangeAllowedInState() const;

    /// Log why an altitude mode lands on a ToF-silence episode / 高度モードが ToF 出力停止で着陸する理由をログ出力
    void logToFSilenceLanding() const;

    // Per-alert responses of handleAlert() (one function per alert, each short)
    // handleAlert() のアラート別の応答（アラートごとに 1 関数、いずれも短い）
    void handleImpactAlert(AlertType type);
    bool completeBlindLandingByImpact(AlertType type);   // IMPACT while LANDING with a silent ToF = touchdown, not an emergency / ToF 出力停止中の LANDING での IMPACT は接地（非常でない）
    void handleCommLostAlert(const SystemAlert& alert);
    void handleLowBatteryAlert(const SystemAlert& alert);
    void handleEskfDivergedAlert();

    /// End the LANDING by time while the ToF is silent (no ToF → no touchdown by height; the
    /// impact-based touchdown is detected by ImuTask). Called every update().
    /// ToF が出力停止している間の LANDING を時間で終える（ToF が無いと高さでは接地を検出できない。
    /// 衝撃による接地は ImuTask が検出する）。毎回の update() から呼ぶ。
    void handleBlindLandingTimeout(uint32_t now_us);

    /// Land (TAKEOFF / FLYING → LANDING) once the hover grace after a COMM_LOST alert has elapsed (decideCommLossLanding()).
    /// COMM_LOST アラートからのホバー猶予が経過したら着陸（TAKEOFF・FLYING → LANDING）させる（decideCommLossLanding()）。
    void handleCommLossGrace(uint32_t now_us);

    /// Refuse a switch into ALT_HOLD / POS_HOLD while the downward ToF is silent; true = refused.
    /// 下向き ToF が出力停止中の ALT_HOLD / POS_HOLD への切り替えを拒否する。true = 拒否。
    bool refuseModeForToFSilence(FlightMode new_mode);


    /// Clear SystemMode.mode_block (republish only if it was set).
    /// SystemMode.mode_block を下ろす（立っていたときだけ再発行）。
    void clearModeBlock();

    /// Re-evaluate evaluateArmBlock() and republish system_mode if it changed
    /// evaluateArmBlock() を再評価し、変化があれば system_mode を再発行
    void refreshArmBlock();

    /// Publish current PairingState to the pairing_state topic (comm/notify read it)
    /// 現在の PairingState を pairing_state トピックに発行する（comm/notify が読む）
    void publishPairingState();

    // Current state
    // 現在の状態
    FlightState state_ = FlightState::INIT;
    uint32_t state_since_us_ = 0;   // [us] when state_ was entered (published as SystemMode.state_since_us) / state_ に入った時刻（SystemMode.state_since_us として発行）
    FlightMode mode_ = FlightMode::STABILIZE;

    // Pairing state (parallel to FlightState). Owned here; comm reflects it.
    // ペアリング状態（FlightState と並行）。ここが所有し comm が反映する。
    PairingState pairing_state_ = PairingState::NotPaired;

    // A bench motor test has run and its estimator re-level is still owed
    // (see setBenchRelevelPending). Read by the pre-arm gate in requestArm().
    // ベンチ用モータテストが走り、推定器の再水平化が未実施（setBenchRelevelPending 参照）。
    // requestArm() の ARM 前判定が読む。
    bool bench_relevel_pending_ = false;

    // Last published ARM-block reason (SystemMode.arm_block); republished on change only.
    // 最後に発行した ARM 阻害理由（SystemMode.arm_block）。変化時のみ再発行。
    ArmBlock arm_block_ = ArmBlock::None;

    // Comm-loss failsafe timer (requirements §9: hover hold 3 s → auto landing).
    // Armed by handleAlert(COMM_LOST) while FLYING; update() lands once the grace
    // period from comm_lost_time_us_ elapses. Cleared on landing or on leaving FLYING.
    // 通信断フェイルセーフのタイマ（要件§9: ホバー維持3秒→自動着陸）。FLYING 中の
    // handleAlert(COMM_LOST) で起動し、comm_lost_time_us_ から猶予経過で update() が
    // 着陸させる。着陸時または FLYING を外れた時にクリアする。
    bool     comm_lost_pending_ = false;
    uint32_t comm_lost_time_us_ = 0;
    // Time since comm_lost_time_us_ (signed age, saturating); isSilent(grace) = grace elapsed.
    // comm_lost_time_us_ からの経過（符号付き、飽和）。isSilent(猶予) = 猶予経過。
    SilenceWatch comm_lost_watch_;

    // Why the latest flight-mode change request was refused (SystemMode.mode_block); republished
    // on change only. Set by requestModeChange() when it refuses an altitude mode while the ToF
    // is silent, cleared when a request is accepted or the ToF is alive again.
    // 直近のモード変更要求が拒否された理由（SystemMode.mode_block）。変化時のみ再発行。ToF が出力
    // 停止中に高度モードを拒否したとき requestModeChange() が立て、要求が受理されるか ToF が
    // 復帰すると下ろす。
    ModeBlock mode_block_ = ModeBlock::None;

    // The latest ToF-silence verdict, read through the staleness guard (this task's own instance).
    // Starts silent: before the first reading nothing is vouched for.
    // 最新の ToF 出力停止の判定。鮮度のガード（このタスク自身のインスタンス）を通して読む。
    // 出力停止から始まる: 最初の読みの前は何も保証されていない。
    // `mutable`: isToFSilent() is a const query that refreshes them (a cache of the latest reading).
    // `mutable`: isToFSilent() は const の問い合わせで、これらを更新する（最新の読みのキャッシュ）。
    // True only when the LATEST requestModeChange() call was refused and consumed (not a standing
    // block: a later request that the state merely defers, e.g. during FLIP, must keep its edge).
    // 「直近の」requestModeChange() 呼び出しが拒否され消費されたときだけ true（継続する block では
    // ない: 状態が単に保留する後の要求、例えば FLIP 中のものは、エッジを保たねばならない）。
    bool last_mode_request_consumed_ = false;

    mutable FailsafeStatusGuard tof_guard_;
    mutable FailsafeStatus      tof_status_ = {true, static_cast<uint8_t>(ToFSilenceReason::StatusStale), 0};

    // Counts TOF_SILENT alerts handled as WarnOnly, so the warning tone sounds on the first one and
    // then only every kToFWarnToneEveryN-th (the alert is re-raised about once a second).
    // WarnOnly として処理した TOF_SILENT アラートの数。警告音は最初の 1 回と、以後 kToFWarnToneEveryN
    // 回に 1 回だけ鳴らす（アラートは約 1 秒ごとに再発報される）。
    uint8_t tof_warn_count_ = 0;

    // LANDING timer: how long the state has been LANDING, from the moment it entered LANDING (no ToF → no touchdown
    // detection by height, so a landing that meets a silent ToF is ended by time, see
    // sf::limits::kBlindLandingTimeoutUs). Runs while LANDING, whether the ToF is alive or silent.
    // LANDING のタイマ: 状態が LANDING に入った時刻からの経過時間（ToF が無いと高さで接地を検出できないため、
    // 出力停止した ToF に出会った着陸は時間で終える。sf::limits::kBlindLandingTimeoutUs 参照）。
    // ToF が動作していても出力停止でも、LANDING の間は計時する。
    LandingClock landing_clock_;   // time since the state entered LANDING (tof_silence_policy.hpp) / 状態が LANDING に入ってからの時間
    LandingEnd last_landing_end_     = LandingEnd::None;

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
