/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_task.cpp
 * @brief State management task — event-driven (priority 22)
 *        状態管理タスク — イベント駆動（優先度22）
 *
 * Waits for events (alerts, commands) and delegates to StateManager.
 * This is the sole task that modifies flight state.
 *
 * イベント（アラート、コマンド）を待ち、StateManagerに委譲する。
 * フライト状態を変更する唯一のタスク。
 *
 * @design architecture.md §6 — StateTask: event-driven, priority 22   [OK]
 * @design architecture.md §2 — State Management: sole transition owner [OK]
 * @design detailed_design.md §8 — StateTask                          [OK]
 */

#include <cstdio>   // ROOT-CAUSE DIAG: raw printf markers (temporary)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "state_manager.hpp"
#include "config.hpp"

static const char* TAG = "StateTask";

/// State manager instance — file-local (static): the StateManager is owned by
/// StateTask alone and all transitions arrive via topics (R5/R6). Without static
/// the symbol could be extern'd from another translation unit, bypassing that wall.
/// 状態管理インスタンス — ファイルローカル（static）: StateManager は StateTask が単独
/// 所有し、遷移は全てトピック経由（R5/R6）。static が無いと他の翻訳単位から extern で
/// 触れてしまい、この防壁を迂回できてしまう。
static sf::StateManager g_state_manager;

// -----------------------------------------------------------------------------
// TEMPORARY DIAGNOSTICS (real-hardware INIT-stuck investigation, 2026-06-09).
// StateTask increments g_diag_state_loops at the top of each loop and sets
// g_diag_state_stage through the loop; ImuTask prints them (raw printf) so we can
// see whether StateTask's loop runs and, if blocked, WHERE. Remove once root-caused.
// 一時診断（実機 INIT 停止調査）。StateTask がループ先頭で g_diag_state_loops を増やし
// ステージを更新、ImuTask が raw printf で報告。StateTask が回っているか/どこで止まるかを見る。
// -----------------------------------------------------------------------------
extern "C" volatile uint32_t g_diag_state_loops = 0;   // loop iterations / ループ回数
extern "C" volatile uint8_t  g_diag_state_stage = 0;   // last reached stage / 到達ステージ

// =============================================================================
// Ground→flight covariance handoff at ARM — why ATTITUDE-only, not a full reset
//
// detailed_design §3 originally called for a full ESKF reset at ARM (rationale: the
// on-ground covariance convergence is not flight-representative). A SIL reset-timing
// sweep (8 strategies × the flight suite) showed the literal full reset — and any
// inflation of the POSITION/VELOCITY/BIAS covariance — destabilizes the takeoff
// transient: the re-inflated covariance over-trusts the thrust-contaminated
// accelerometer and POS_HOLD attitude diverges (pos_roll/pitch/flight crash). Only two
// strategies passed the whole suite: doing nothing, and inflating the ATTITUDE
// covariance alone. We adopt the latter: it honors the design intent (declare flight-
// uncertainty about attitude at ARM) yet stays stable, because attitude re-converges
// from gravity on the ground before takeoff. The position/velocity "ground is zero, not
// flight-representative" handoff is handled separately and precisely at the airborne
// edge by ImuTask's resetPositionVelocity (class-B, architecture §4); the bias is the
// same IMU in flight, so its ground estimate is kept. (CLAUDE.md: control changes are
// backed by simulation — the sweep is the evidence.)
//
// ARM での地上→飛行 共分散ハンドオフ — なぜ「姿勢のみ」で全リセットでないか。
// detailed_design §3 は当初 ARM時 ESKF 全リセットを求めた（地上の共分散収束は飛行を代表
// しない、という根拠）。SIL のリセットタイミング掃引（8方策×飛行スイート）で、字義通りの
// 全リセット — および位置/速度/バイアスの共分散の膨張 — は離陸過渡を不安定化すると判明:
// 再膨張した共分散がスラスト汚染された加速度計を過信し POS_HOLD 姿勢が発散
// （pos_roll/pitch/flight 墜落）。スイート全PASS は2方策のみ＝「何もしない」と「姿勢の共分散
// だけ膨張」。後者を採用: 設計意図（ARM で姿勢の自信をリセット）を満たしつつ安定 — 姿勢は
// 離陸前に地上で重力から再収束するため。位置/速度の「地上ゼロは飛行を代表しない」ハンドオフは
// 空中エッジで ImuTask の resetPositionVelocity が別途・正確に処理（クラスB, architecture §4）。
// バイアスは飛行中も同じ IMU ゆえ地上推定を保持。（CLAUDE.md: 制御変更はシミュレーションで裏付け。）
//
// @design detailed_design.md §3 — ARM: ESKF attitude-covariance inflation (was full reset) [OK]
// @design architecture.md §4 — ground→flight covariance handoff (SIL-validated)            [OK]

/// Register the StateManager transition callbacks (called once at task start, after
/// init()). Each callback publishes reset commands on the estimator/controller/notify
/// command topics per the detailed_design §3 transition table; the resource-owning
/// tasks (ImuTask=estimator, ControlTask=controller, NotifyTask=LED/buzzer) consume them.
/// This is the single place that encodes the transition table and makes the state
/// machine the sole owner of transition resets, replacing the ad-hoc edge detection that
/// used to live in ImuTask/ControlTask.
/// StateManager の遷移コールバックを登録する（タスク開始時に init() 後1回）。各コールバックは
/// detailed_design §3 遷移表に従い推定器/制御器/通知の指令トピックにリセット指令を publish し、
/// リソース所有タスク（ImuTask=推定器, ControlTask=制御器, NotifyTask=LED/ブザー）が消費する。
/// ここが遷移表を符号化する唯一の場所で、ImuTask/ControlTask に散らばっていたエッジ検出を
/// 置き換え、遷移リセットの唯一の所有者を状態機械にする。
///
/// @design architecture.md §4 — onExit/onEnter callbacks consolidate reset    [OK]
/// @design detailed_design.md §3 — state transition table                     [OK]
static void registerStateCallbacks(sf::StateManager& manager)
{
    using sf::FlightState;
    using sf::FlightMode;

    // onEnter — reset/init on entering the new state (detailed_design §3 onEnter column).
    // The (from, to) pair disambiguates transitions that share a destination.
    // onEnter — 新状態に入った時の reset/初期化（detailed_design §3 onEnter 列）。
    // (from, to) ペアで、行き先を共有する遷移を区別する。
    manager.onEnter([](FlightState from, FlightState to) {
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        switch (to) {
        case FlightState::IDLE_GROUND:
            if (from == FlightState::INIT || from == FlightState::IDLE_HELD) {
                // INIT → IDLE_GROUND: boot calibration. IDLE_HELD → IDLE_GROUND (placed
                // back down): re-calibration for re-fly readiness — the pre-arm check then
                // rejects ARM until calibration completes (requirements ②③). Both are
                // callback-driven (architecture §6).
                // INIT→IDLE_GROUND: 起動校正。IDLE_HELD→IDLE_GROUND（置き直し）: 再飛行
                // readiness のための再校正 — ARM前チェックが完了までARMを拒否（要件②③）。
                // どちらもコールバック駆動（architecture §6）。
                if (from == FlightState::IDLE_HELD) {
                    // Placed back down after handling a crashed craft: the estimator may carry
                    // a LATCHED gross attitude error from the crash — a tumble/inversion drives
                    // the attitude estimate far from truth, and the accel-attitude χ² gate then
                    // rejects the very correction that would fix it (the residual exceeds the
                    // gate forever), so it never self-rights. The craft is now KNOWN level and
                    // stationary on the ground, so a full ESKF reset re-seeds the attitude to
                    // level (identity) and re-inflates the covariance, clearing the latch — the
                    // missing half of re-fly readiness (SIL crash_refly surfaced this). INIT
                    // boots from a clean estimator (constructor reset), so it skips this.
                    // 墜落機を扱って置き直した直後: 推定器が墜落由来の大姿勢誤差を latch しうる ―
                    // タンブル/反転で姿勢推定が真値から大きく外れ、accel-attitude の χ² ゲートが
                    // それを直す補正自体を棄却し（残差がゲートを超え続ける）自己復帰しない。機体は
                    // 今 level・静止と既知ゆえ、ESKF 全リセットで姿勢を level（identity）へ再シード・
                    // 共分散を再膨張し latch を解除する ― 再飛行 readiness の欠けていた半分
                    // （SIL crash_refly が炙り出した）。INIT は構築時 reset のクリーンな推定器
                    // から起動するため不要。
                    sf::estimator_command.publish(
                        {static_cast<uint8_t>(sf::EstimatorCmd::Reset), now});
                }
                sf::estimator_command.publish(
                    {static_cast<uint8_t>(sf::EstimatorCmd::Recalibrate), now});
            } else {
                // Return to ground from an armed/airborne state (DISARM / crash / land).
                // detailed_design §3: FLYING→IDLE_GROUND and LANDING→IDLE_GROUND do an
                // ESKF full reset (clear in-flight/diverged state — re-fly readiness,
                // requirement ①); ARMED_GROUND→IDLE_GROUND (disarm without flying) does
                // NOT (the estimator was never disturbed). So gate the reset on
                // isAirborne(from). imu_task re-seeds the boot calibration after the reset
                // (reseedCalibration), and reset() un-freezes the accel bias.
                // 武装/空中状態からの接地復帰（DISARM/墜落/着陸）。detailed_design §3:
                // FLYING→IDLE_GROUND と LANDING→IDLE_GROUND は ESKF を全リセット（飛行中・
                // 発散状態を一掃 — 再飛行 readiness 要件①）、ARMED_GROUND→IDLE_GROUND
                // （飛ばずに DISARM）はしない（推定器は乱れていない）。よって reset は
                // isAirborne(from) でゲートする。reset 後に imu_task が起動校正を再注入
                // （reseedCalibration）し、reset() は加速度バイアスの凍結を解除する。
                if (sf::isAirborne(from)) {
                    sf::estimator_command.publish(
                        {static_cast<uint8_t>(sf::EstimatorCmd::Reset), now});
                    // A controlled landing (LANDING→IDLE_GROUND) ends at rest on the
                    // ground → freeze the bias to keep "ground = frozen" (reset un-froze
                    // it). The crash/emergency path (FLYING/TAKEOFF→IDLE_GROUND) leaves it
                    // un-frozen and re-freezes at the next ARM instead (detailed_design §3
                    // lists バイアスフリーズ only for LANDING→IDLE_GROUND).
                    // 制御着陸（LANDING→IDLE_GROUND）は地上静止で終わる → バイアスを凍結し
                    // 「地上=frozen」を保つ（reset が解除済み）。墜落/緊急経路（FLYING/
                    // TAKEOFF→IDLE_GROUND）は凍結せず次の ARM で再凍結する（detailed_design
                    // §3 はバイアスフリーズを LANDING→IDLE_GROUND にのみ記載）。
                    // [CANDIDATE] FreezeBias DROPPED (freeze mechanism broken — see ARM).
                    // if (from == FlightState::LANDING) {
                    //     sf::estimator_command.publish(
                    //         {static_cast<uint8_t>(sf::EstimatorCmd::FreezeBias), now});
                    // }
                }
                sf::notify_command.publish(
                    {static_cast<uint8_t>(sf::NotifyEvent::DisarmTone), now});
            }
            break;
        case FlightState::ARMED_GROUND:
            if (from == FlightState::IDLE_GROUND) {
                // ARM (detailed_design §3: 全PIDリセット、ESKF姿勢共分散の膨張、arm音):
                //  1. controller Reset — clear the PID integrators (no stale wind-up).
                //  2. estimator InflateCov(Attitude) — declare flight-uncertainty about the
                //     attitude (the ground convergence is re-asserted against gravity before
                //     takeoff). NOT a full reset / not pos-vel-bias — those destabilize the
                //     takeoff transient (see the header note: SIL sweep result).
                //  3. arm tone.
                // ARM（detailed_design §3: 全PIDリセット、ESKF姿勢共分散の膨張、arm音）:
                //  1. controller Reset — PID 積分器クリア。
                //  2. estimator InflateCov(Attitude) — 姿勢の飛行不確かさを宣言（地上収束は
                //     離陸前に重力で再主張される）。全リセットでも pos/vel/bias でもない — それらは
                //     離陸過渡を不安定化する（ヘッダ注記: SIL 掃引結果）。
                //  3. arm 音。
                sf::controller_command.publish(
                    {static_cast<uint8_t>(sf::ControllerCmd::Reset), 0, now});
                sf::estimator_command.publish(
                    {static_cast<uint8_t>(sf::EstimatorCmd::InflateCov), now,
                     static_cast<uint16_t>(sf::CovScope::Attitude)});
                sf::notify_command.publish(
                    {static_cast<uint8_t>(sf::NotifyEvent::ArmTone), now});
            }
            break;
        // TAKEOFF → FLYING needs no class-A reset here. The "ESKF position/velocity reset"
        // of detailed_design §3 is the timing-critical ToF-synced vertical handoff (class B,
        // ImuTask::applyVerticalGroundHandoff) — a ~20 ms-late reset via this callback
        // degrades POS_HOLD attitude (architecture §4). The bias unfreeze is dropped: the
        // ESKF freeze mechanism (active_mask) is for permanent sensor-absence isolation, not
        // ground↔flight toggling, so bias estimation simply stays active in flight.
        // TAKEOFF→FLYING はここでのクラスA reset 不要。detailed_design §3 の「ESKF 位置/速度
        // リセット」はタイミング命の ToF 同期鉛直ハンドオフ（クラスB, ImuTask）— 本コールバック
        // 経由の ~20ms 遅れ reset は POS_HOLD 姿勢を劣化させる（architecture §4）。bias 解除は
        // 見送り: ESKF 凍結機構（active_mask）は恒久センサ不在隔離用で地上↔飛行トグル用でない
        // ため、バイアス推定は飛行中も単にアクティブのまま。
        default:
            break;
        }
    });

    // onModeChange — FLYING sub-mode switch: reconfigure the controller (resets the
    // relevant loops, captures hold targets). detailed_design §3 FLYING row.
    // onModeChange — FLYING サブモード切替: 制御器を再構成（該当ループ reset、保持目標捕捉）。
    manager.onModeChange([](FlightMode /*old_mode*/, FlightMode new_mode) {
        const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
        sf::controller_command.publish(
            {static_cast<uint8_t>(sf::ControllerCmd::ModeChange),
             static_cast<uint8_t>(new_mode), now});
    });
}

void StateTask(void* pvParameters)
{
    ESP_LOGI(TAG, "StateTask started");

    // Transition from INIT → IDLE_GROUND
    // INIT → IDLE_GROUNDへ遷移
    //
    // @design requirements.md §2 — INIT → IDLE_GROUND on init complete [OK]
    g_state_manager.init();

    // Register the transition callbacks that consolidate all reset processing
    // (architecture §4). Must be after init() — init() zeroes the callback counts.
    // 全リセット処理を集約する遷移コールバックを登録する（architecture §4）。init() の後で
    // 行う — init() はコールバック数をゼロにするため。
    registerStateCallbacks(g_state_manager);

    // Previous ARM switch state, for rising/falling edge detection across iterations.
    // 前回の ARM スイッチ状態（立上り/立下りエッジ検出用、反復間で保持）。
    bool prev_arm = false;
    bool init_done = false;   // INIT → IDLE_GROUND done once / 初期化完了遷移を1回

    // Previous comm bind flag (pairing_complete.bound), for false→true edge detection.
    // We react to the EDGE (comm became bound) once, robustly to the SIL virtual clock
    // reading 0 (a timestamp-based sentinel would be unreliable there).
    // 前回の comm バインドフラグ（pairing_complete.bound）。false→true エッジ検出用。
    // SIL 仮想クロックが 0 を返しても頑健なよう、エッジ（comm がバインドした瞬間）に1回反応する。
    bool prev_bound = false;

    while (true) {
        // =====================================================================
        // Wake on a failsafe notification OR every 20 ms to poll the pilot's RC
        // stream (the controller sends at 50 Hz). Still event-driven for alerts;
        // the periodic tick lets us edge-detect the ARM/mode switches the pilot
        // holds without each packet having to notify this task across components.
        // フェイルセーフ通知、または 20ms 毎にウェイクしてパイロットの RC 列をポーリング
        // （コントローラは 50Hz 送信）。アラートはイベント駆動のまま、周期ティックで ARM/
        // モードのエッジ検出を行う（各パケットがコンポーネント跨ぎで通知しなくて済む）。
        //
        // @design detailed_design.md §8 — StateTask: event-driven + 50 Hz RC poll [OK]
        // =====================================================================

        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        g_diag_state_loops = g_diag_state_loops + 1;  // DIAG: loop is running / ループ稼働
        g_diag_state_stage = 1;      // DIAG: top of loop

        // =====================================================================
        // INIT → IDLE_GROUND once the estimator is producing valid output, i.e.
        // sensors are up and fused (our "initialization complete" signal). Until
        // then ARM is correctly rejected (not IDLE_GROUND).
        // 推定器が有効出力を出したら（センサ立上り＋融合済み＝「初期化完了」信号）
        // INIT → IDLE_GROUND。それまで ARM は正しく拒否される（IDLE_GROUND でない）。
        // =====================================================================
        if (!init_done && sf::sensor_imu.latest().timestamp != 0) {
            std::printf("  ROOT: ST pre-notifyInitComplete\n");
            g_state_manager.notifyInitComplete();
            std::printf("  ROOT: ST post-notifyInitComplete\n");
            init_done = true;
        }

        // =====================================================================
        // Process pilot requests (ARM edge + flight-mode selection). sf_comm
        // reports the switches as facts; StateManager makes the decisions (it
        // gates ARM to IDLE_GROUND and validates mode transitions internally).
        // パイロット要求（ARM エッジ＋飛行モード選択）を処理。sf_comm はスイッチを事実
        // として報告し、判断は StateManager が行う（ARM を IDLE_GROUND に限定し、モード
        // 遷移を内部で検証）。
        //
        // @design architecture.md §2 — detection reports, state management decides [OK]
        // =====================================================================

        g_diag_state_stage = 2;      // DIAG: about to read pilot_request (mutex)
        sf::PilotRequest req = sf::pilot_request.latest();
        if (req.timestamp != 0) {                          // skip until a packet arrives
            if (req.arm && !prev_arm) {
                g_state_manager.requestArm();              // rising edge: request ARM
            } else if (!req.arm && prev_arm) {
                g_state_manager.requestDisarm();           // falling edge: request DISARM
            }
            prev_arm = req.arm;

            // Derive the requested FlightMode from the mode switches (priority:
            // POS_HOLD > ALT_HOLD > ACRO > STABILIZE default).
            // モードスイッチから要求 FlightMode を導出（優先: POS>ALT>ACRO>STABILIZE）。
            sf::FlightMode want = sf::FlightMode::STABILIZE;
            if (req.pos_hold)      want = sf::FlightMode::POS_HOLD;
            else if (req.alt_hold) want = sf::FlightMode::ALT_HOLD;
            else if (req.acro)     want = sf::FlightMode::ACRO;
            if (want != g_state_manager.getMode()) {
                g_state_manager.requestModeChange(want);
            }
        }

        // =====================================================================
        // Button gestures (ButtonTask → button_event): a click toggles ARM/DISARM
        // on the GROUND only. The on-board button is an ARM action (requirements §2:
        // "ARM action (button or controller)"). We deliberately restrict the toggle to
        // IDLE_GROUND ↔ ARMED_GROUND: a single accidental short click must NOT cut the
        // motors in flight — an in-flight kill is a deliberate failsafe / controller
        // DISARM, not a button tap. requestArm()/requestDisarm() still validate
        // internally (pre-arm gates, armed-state check), so this is a request, not a
        // forced transition. The button reports the gesture as a fact; this task — the
        // sole transition authority — decides. Long-press gestures are reserved
        // (pairing / system reset) and ignored here.
        // ボタンジェスチャ（ButtonTask → button_event）: クリックは地上でのみ ARM/DISARM を
        // トグルする。機体ボタンは ARM 操作（要件§2「ARM 操作（ボタン or コントローラ）」）。
        // トグルを IDLE_GROUND↔ARMED_GROUND に意図的に限定する: 誤った単発の短押しが飛行中に
        // モータを切ってはならない — 飛行中のキルは意図的な failsafe/コントローラ DISARM で
        // あってボタンのタップではない。requestArm()/requestDisarm() は内部で検証する
        // （ARM前ゲート・armed 判定）ため、これは強制遷移でなく「要求」。ボタンはジェスチャを
        // 事実として報告し、唯一の遷移実行者である本タスクが判断する。長押しは予約
        // （ペアリング/システムリセット）でここでは無視する。
        //
        // @subscriber button_event
        // @design requirements.md §2 — ARM action (button or controller)   [OK]
        // @design architecture.md §2 — detection reports, state decides     [OK]
        // =====================================================================
        sf::ButtonEvent button;
        while (sf::button_event.read(button)) {
            const auto gesture = static_cast<sf::ButtonGesture>(button.gesture);
            if (gesture == sf::ButtonGesture::Click) {
                switch (g_state_manager.getState()) {
                case sf::FlightState::IDLE_GROUND:  g_state_manager.requestArm();    break;
                case sf::FlightState::ARMED_GROUND: g_state_manager.requestDisarm(); break;
                default: break;  // airborne / INIT / held: ignore a click (see note above)
                }
            } else if (gesture == sf::ButtonGesture::LongPress3s) {
                // Long-press 3 s = (re-)pair: discard the existing bind and re-enter
                // Pairing so a new transmitter can take over. requestPairing() is
                // internally gated to the ground and is idempotent, so this is safe
                // to call regardless of the current state.
                // 長押し3秒 = （再）ペア: 既存バインドを破棄し Pairing に再突入して新しい
                // 送信機が引き継げるようにする。requestPairing() は内部で地上に限定され
                // 冪等なので、現在状態に関わらず安全に呼べる。
                g_state_manager.requestPairing();
            }
        }

        // =====================================================================
        // Takeoff sequencing: ARMED_GROUND → TAKEOFF → FLYING.
        // 離陸シーケンス: ARMED_GROUND → TAKEOFF → FLYING。
        //
        // Requirements §2: ARMED_GROUND→TAKEOFF on "throttle input";
        // TAKEOFF→FLYING on "takeoff complete (altitude threshold reached)".
        // The altitude-threshold detector is the ToF-based TakeoffLandingMgr, owned by
        // ImuTask (it has the ToF and the vertical estimate); it publishes its airborne
        // state on system_status. With the vertical estimate now trustworthy (ALT/POS
        // hold, development_roadmap Phase B), this replaces the earlier estimator-
        // independent dwell. Reaching FLYING unlocks requestModeChange.
        // 要件§2: スロットル入力で離陸、高度閾値到達で離陸完了。高度検出は ImuTask 所有の
        // ToF ベース TakeoffLandingMgr（ToF と鉛直推定を持つ）で、airborne 状態を
        // system_status に発行する。鉛直推定が信頼できる今（ALT/POS 保持, ロードマップ
        // Phase B）、これが旧 estimator 非依存 dwell を置き換える。FLYING 到達で
        // requestModeChange が解放される。
        //
        // @design requirements.md §2 — ARMED_GROUND→TAKEOFF→FLYING        [OK]
        // @design development_roadmap.md §3 Layer 3 — ToF takeoff detect  [OK]
        // =====================================================================

        g_diag_state_stage = 4;      // DIAG: about to read command_setpoint (mutex)
        const sf::FlightState fs = g_state_manager.getState();
        const float throttle = sf::command_setpoint.latest().throttle;

        if (fs == sf::FlightState::ARMED_GROUND &&
            throttle > config::TAKEOFF_THROTTLE_THRESH) {
            g_state_manager.notifyTakeoff();              // → TAKEOFF
        } else if (fs == sf::FlightState::TAKEOFF &&
                   sf::system_status.latest().airborne) {
            g_state_manager.notifyTakeoffComplete();      // → FLYING (ToF: off the ground)
        }

        // =====================================================================
        // Ground-situation transitions from the ToF detector (system_status): the held
        // flag drives IDLE_GROUND ↔ IDLE_HELD (lifted / placed), and the landing flag
        // drives LANDING → IDLE_GROUND (landing complete). Detection lives in ImuTask
        // (it owns the ToF); the transition DECISION is here.
        // ToF 検出器（system_status）からの地上状況遷移: held フラグが IDLE_GROUND↔IDLE_HELD
        // （持上げ/設置）を、landing フラグが LANDING→IDLE_GROUND（着陸完了）を駆動する。検出は
        // ImuTask（ToF 所有）、遷移の判断はここ。
        //
        // @design requirements.md §2 — IDLE_GROUND ↔ IDLE_HELD, LANDING → IDLE_GROUND [OK]
        // @design architecture.md §2 — separate detection from decision               [OK]
        // =====================================================================
        const sf::SystemStatus status = sf::system_status.latest();
        if (fs == sf::FlightState::IDLE_GROUND || fs == sf::FlightState::IDLE_HELD) {
            g_state_manager.notifyIdleGroundHeld(status.held);
        } else if (fs == sf::FlightState::LANDING && status.landing) {
            g_state_manager.notifyLandingComplete();
        }

        // =====================================================================
        // Pairing: reflect sf_comm's bind status and auto-enter pairing when unpaired.
        // sf_comm publishes its bind status on pairing_complete (bound + learned MAC);
        // the StateManager owns the PairingState. A FRESH bound=true report → Paired.
        // While still NotPaired and on the ground we (auto-)enter Pairing so a
        // transmitter can discover us (mutual MAC learning). requestPairing() is gated
        // to ground states and idempotent, so calling it each cycle is safe. We process
        // the bind report BEFORE the auto-enter so a boot-restored bind wins.
        // ペアリング: sf_comm のバインド状態を反映し、未ペアなら自動で Pairing に入る。
        // sf_comm は pairing_complete にバインド状態（bound + 学習MAC）を発行し、PairingState は
        // StateManager が所有する。新しい bound=true 報告 → Paired。まだ NotPaired かつ地上なら
        // 自動で Pairing に入り、送信機が我々を発見できるようにする（相互 MAC 学習）。
        // requestPairing() は地上限定で冪等ゆえ毎周期呼んで安全。起動時の復元バインドが勝つよう
        // バインド報告を自動突入より先に処理する。
        //
        // @subscriber pairing_complete
        // @design requirements.md §2 — PairingState (auto-enter on unpaired) [OK]
        // @design architecture.md §4 — StateManager owns PairingState        [OK]
        // =====================================================================
        g_diag_state_stage = 6;      // DIAG: about to read pairing_complete (mutex)
        const sf::PairingComplete bind = sf::pairing_complete.latest();
        if (bind.timestamp != 0) {                         // comm has reported its bind status
            const bool bound = bind.bound;
            if (bound && !prev_bound) {
                g_state_manager.notifyPairingComplete();   // false→true edge → Paired
            }
            prev_bound = bound;
            const sf::FlightState fs_now = g_state_manager.getState();
            if (!bound &&
                g_state_manager.getPairingState() == sf::PairingState::NotPaired &&
                (fs_now == sf::FlightState::IDLE_GROUND ||
                 fs_now == sf::FlightState::IDLE_HELD)) {
                g_state_manager.requestPairing();          // unpaired on the ground → Pairing
            }
        }

        // =====================================================================
        // Process system alerts from failsafe
        // フェイルセーフからのシステムアラートを処理
        //
        // @design architecture.md §4 — FAILSAFE as event              [OK]
        // =====================================================================

        sf::SystemAlert alert;
        while (sf::system_alert.read(alert)) {
            g_state_manager.handleAlert(alert);

            // Mirror battery alerts onto notify_command so NotifyTask can sound the
            // buzzer. Notify cannot read system_alert (this task owns that shared
            // queue; a second reader would steal failsafe events), so we forward.
            // 電池アラートを notify_command に転送し NotifyTask がブザーを鳴らせるように
            // する。Notify は system_alert を読めない (本タスクが所有する共有キューで、
            // 二重消費は failsafe イベントを奪う) ため、ここで転送する。
            if (alert.type == static_cast<uint8_t>(sf::AlertType::LOW_BATTERY) ||
                alert.type == static_cast<uint8_t>(sf::AlertType::USB_POWER)) {
                sf::notify_command.publish(
                    {static_cast<uint8_t>(sf::NotifyEvent::LowBattery),
                     static_cast<uint32_t>(esp_timer_get_time())});
            }
        }

        // =====================================================================
        // Drive time-deferred transitions (the comm-loss hover→LANDING grace,
        // requirements §9). Called every cycle: the failsafe raises COMM_LOST only
        // once (rising edge), so the deferred landing needs this periodic tick, not
        // another alert. No-op unless a timer is pending.
        // 時間遅延つき遷移を駆動（通信断のホバー→LANDING 猶予、要件§9）。毎周期呼ぶ:
        // failsafe は COMM_LOST を1回（立ち上がり）しか出さないので、遅延着陸には
        // アラート再来でなくこの周期ティックが要る。保留タイマが無ければ no-op。
        //
        // @design requirements.md §9 — comm loss: hover 3 s → LANDING   [OK]
        // =====================================================================
        g_diag_state_stage = 9;      // DIAG: about to call update() (end of loop)
        g_state_manager.update(static_cast<uint32_t>(esp_timer_get_time()));
    }
}
