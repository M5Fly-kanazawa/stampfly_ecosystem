/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file state_manager.cpp
 * @brief StateManager implementation
 *        StateManager実装
 *
 * @design requirements.md §4 — Component #3: State Management         [OK]
 * @design architecture.md §2 — Sole transition owner                  [OK]
 * @design detailed_design.md §3 — State transition table              [OK]
 */

#include "state_manager.hpp"
#include "params.hpp"
#include "sensor_freshness.hpp"
#include "tof_silence_policy.hpp"
#include "takeoff_policy.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "StateManager";

namespace sf {

/// Below this voltage the sensor_power reading is "unknown" (power monitor absent or a
/// failed read publishes 0). A 0/unknown reading must NOT block ARM — power monitoring is
/// Optional (hardware_init.md §5) — so the pre-arm voltage gate ignores readings below it.
/// この電圧未満は sensor_power の読みが「不明」（電源モニタ不在 or 読み失敗で 0 が発行）。
/// 電源監視は Optional（hardware_init.md §5）ゆえ 0/不明は ARM を阻まない — ARM 前電圧判定は
/// これ未満の読みを無視する。
static constexpr float kVoltageValidMin = 0.1f;

/// Comm-loss hover grace period [us]: on link loss while FLYING the craft holds its
/// hover for this long before auto-landing (requirements §9: "hover hold 3 s → auto
/// landing"). Recovery within the window does NOT cancel the landing — the requirement
/// specifies an unconditional hover-then-land, and the failsafe re-raising on a fresh
/// loss must not reset the timer. Named constant (no magic number); a follow-up phase
/// may move it to params alongside the battery threshold (sf_state is config.hpp-free).
/// 通信断ホバー猶予 [us]: FLYING 中のリンク喪失でこの時間ホバーを維持してから自動着陸
/// （要件§9「ホバー維持3秒→自動着陸」）。窓内の復帰では着陸を取り消さない — 要件は無条件
/// のホバー→着陸を規定し、再喪失での再発報でタイマをリセットしない。名前付き定数（マジック
/// ナンバー回避）。電圧閾値と並べて params 化するのは後続フェーズ（sf_state は config.hpp 非依存）。
static constexpr uint32_t kCommLossHoverUs = 3000u * 1000u;   // 3 s / 3秒

// =============================================================================
// Initialization
// 初期化
// =============================================================================

void StateManager::init()
{
    state_ = FlightState::INIT;
    state_since_us_ = static_cast<uint32_t>(esp_timer_get_time());
    mode_ = FlightMode::STABILIZE;
    pairing_state_ = PairingState::NotPaired;
    exit_callback_count_ = 0;
    enter_callback_count_ = 0;
    mode_callback_count_ = 0;

    // Publish the initial PairingState so comm/notify start from a known value
    // (the topic default is also NotPaired; this stamps a timestamp).
    // 初期 PairingState を発行し comm/notify を既知値から開始させる（トピック既定も
    // NotPaired だが、これで timestamp を刻む）。
    publishPairingState();

    ESP_LOGI(TAG, "StateManager initialized (state=%s)",
             flightStateName(state_));
}

// =============================================================================
// Transition Requests
// 遷移リクエスト
// =============================================================================

/// @design requirements.md §2 — INIT → IDLE_GROUND on init complete   [OK]
void StateManager::notifyInitComplete()
{
    if (state_ != FlightState::INIT) {
        return;   // already past INIT — nothing to do / INIT を過ぎていれば無操作
    }
    ESP_LOGI(TAG, "Init complete → IDLE_GROUND");
    transition(FlightState::IDLE_GROUND);
}

/// Evaluate the pre-arm gates in priority order and return the FIRST failing one (None when
/// all pass). The single place the gates live (INV: one source of truth) — requestArm()
/// acts on it, update() publishes it (SystemMode.arm_block) for the LED / CLI. State-
/// independent: the IDLE_GROUND requirement is checked by requestArm() itself.
/// ARM 前判定を優先順に評価し、最初に落ちた理由を返す（全て通れば None）。判定の唯一の
/// 置き場（INV: 単一の情報源）— requestArm() がこれに従い、update() が LED/CLI 向けに
/// SystemMode.arm_block として発行する。状態非依存: IDLE_GROUND 要件は requestArm() 自身が見る。
///
/// @design requirements.md §9 — USB power / low-V: ARM prohibited         [OK]
/// @design detailed_design.md §3 注9/注10 — single pre-arm reason          [OK]
ArmBlock StateManager::evaluateArmBlock() const
{
    // Gate 0: not searching for a transmitter (the link is not bound yet).
    // 判定0: 送信機を探索中でないこと（リンクが未バインド）。
    if (pairing_state_ == PairingState::Pairing) {
        return ArmBlock::Pairing;
    }

    // Gate 1: battery / USB power (requirements §9). / 判定1: 電池/USB 電源（要件§9）。
    if (isBatteryUnsafeForArm()) {
        return ArmBlock::Battery;
    }

    // Gate 2: boot calibration complete (requirements §9 / design §3): no half-measured bias.
    // 判定2: 起動校正完了（要件§9 / 設計§3）。半端なバイアスで飛ばさない。
    if (!system_status.latest().calibrated) {
        return ArmBlock::Calibrating;
    }

    // Gate 3: no bench motor test active / re-level pending (else the owed re-level is dropped
    // and the craft flies on a tilted estimate).
    // 判定3: ベンチ用モータテストが実行中/再水平化待ちでないこと（でないと再水平化が破棄され、
    // 傾いた推定のまま飛ぶ）。
    // @design detailed_design.md §3 注9 — ARM rejected during bench test / re-level [OK]
    if (motor_test.latest().active || bench_relevel_pending_) {
        return ArmBlock::BenchRelevel;
    }

    // Gate 3b: no ToF-silence episode (the downward ToF produces samples AND the estimator has
    // re-acquired it), in EVERY mode incl. ACRO, see isToFSilent(). Before the attitude gate on
    // purpose: a silent ToF freezes the ground-contact flag (wrong TiltPending otherwise).
    // 判定3b: ToF 出力停止のエピソード中でないこと（サンプルが来て、推定器が取り込み直した）。ACRO を含む
    // 「全モード」。isToFSilent() 参照。姿勢判定より前に置くのは意図的: ToF が出力停止すると接地フラグが
    // 最後の値のまま固定され、そうしないと誤って TiltPending を報告する。
    // @design detailed_design.md §3.1 — ARM rejected while the downward ToF is silent [OK]
    if (toFSilenceRefusesArm(isToFSilent())) {
        return ArmBlock::TofSilent;
    }

    // Gate 4: attitude agrees with gravity. / 判定4: 姿勢推定が重力と一致していること。
    const ArmBlock attitude_block = evaluateAttitudeGate();
    if (attitude_block != ArmBlock::None) {
        return attitude_block;
    }

    // Gate 6: general sensor health — DEFERRED until sf_board::sensor_present() (M2b) is wired.
    // 判定6: センサ健全性（一般）— sf_board::sensor_present()（M2b）の配線まで繰延。
    return ArmBlock::None;
}

/// Pre-arm gate 1: reject ARM if a VALID voltage reading is at or below the unsafe threshold
/// — either running on the USB rail (no pack) or a critically low battery. A 0/unknown
/// reading (power monitor absent → Optional, hardware_init.md §5) does NOT block. Threshold
/// from the safety.battery.usb_v param.
/// ARM 前判定1: 有効な電圧読みが危険閾値以下なら ARM を拒否（USB レール=電池なし or 危険な
/// 低電圧）。0/不明（電源モニタ不在→Optional）は阻まない。閾値は safety.battery.usb_v param。
bool StateManager::isBatteryUnsafeForArm() const
{
    float usb_v = 3.3f;
    params::get_float("safety.battery.usb_v", usb_v);
    const float voltage = sensor_power.latest().voltage;
    return (voltage > kVoltageValidMin) && (voltage <= usb_v);
}

/// Pre-arm gate 4: ImuTask detects (TiltConsistencyMonitor) that, on the ground, the
/// estimated attitude disagrees with the gravity direction the accelerometer measures — the
/// estimate was shaken off (e.g. ground-spin vibration) and the accel-attitude chi2 gate may
/// have latched. Flying on that estimate tilts the takeoff. StateTask resets the estimator
/// once per IDLE_GROUND stay; until the check passes ARM is refused (a craft that stays
/// tilted needs a human to pick it up and place it down, which recalibrates).
/// ARM needs a POSITIVE verdict (attitude_verified), not just the absence of a mismatch:
/// after boot or an estimator Reset the monitor needs ~lpf_s to judge, and that window must
/// not let an unverified estimate through.
/// ARM 前判定4: 地上で推定姿勢が加速度計の測る重力方向と不一致（ImuTask の
/// TiltConsistencyMonitor が検出）なら、推定が外れ、accel-attitude の χ² 判定が latch して
/// いる可能性がある。その推定で飛ぶと離陸が傾く。IDLE_GROUND 中は StateTask が滞在ごとに
/// 1 回推定器を Reset し、判定が通るまで ARM を拒否する（傾いたままの機体は人が持ち上げて
/// 置き直す＝再校正が要る）。ARM には「不一致でない」だけでなく肯定の判定（attitude_verified）
/// が要る: 起動や Reset の後、モニタが判定できるまで約 lpf_s かかり、その窓で未検証の推定を
/// 通してはならない。
/// @design detailed_design.md §3 注10 — ARM rejected on attitude/gravity mismatch [OK]
ArmBlock StateManager::evaluateAttitudeGate() const
{
    const SystemStatus status = system_status.latest();
    if (status.attitude_mismatch) {
        return ArmBlock::TiltMismatch;
    }
    if (!status.attitude_verified) {
        return ArmBlock::TiltPending;
    }
    return ArmBlock::None;
}

/// Log why ARM was refused (serial console; the pilot-facing cues are the LED/tone).
/// ARM 拒否の理由をログ出力（シリアル。操縦者向けの合図は LED/音）。
void StateManager::logArmRejected(ArmBlock reason) const
{
    switch (reason) {
        case ArmBlock::Pairing:
            ESP_LOGW(TAG, "ARM rejected: pairing in progress");
            break;
        case ArmBlock::Battery:
            ESP_LOGW(TAG, "ARM rejected: battery %.2fV (USB/low — unsafe to fly)",
                     sensor_power.latest().voltage);
            break;
        case ArmBlock::Calibrating:
            ESP_LOGW(TAG, "ARM rejected: boot calibration not complete");
            break;
        case ArmBlock::BenchRelevel:
            ESP_LOGW(TAG, "ARM rejected: bench motor test active / re-level pending");
            break;
        case ArmBlock::TiltMismatch:
            ESP_LOGW(TAG, "ARM rejected: attitude estimate disagrees with gravity");
            break;
        case ArmBlock::TiltPending:
            ESP_LOGW(TAG, "ARM rejected: attitude check not passed yet");
            break;
        case ArmBlock::TofSilent:
            ESP_LOGW(TAG, "ARM rejected: downward ToF is silent (%s)",
                     toFSilenceReasonName(static_cast<ToFSilenceReason>(tof_status_.tof_reason)));
            break;
        case ArmBlock::None:
        default:
            break;
    }
}

/// @design requirements.md §2 — ARM from GROUND only                  [OK]
/// @design requirements.md §9 — USB power / low-V: ARM prohibited     [OK]
/// @design detailed_design.md §3 — ARM gated on calibration complete  [OK]
bool StateManager::requestArm()
{
    if (state_ != FlightState::IDLE_GROUND) {
        ESP_LOGD(TAG, "ARM rejected: not in IDLE_GROUND (state=%s)",
                 flightStateName(state_));
        return false;
    }

    // Pre-arm gates (single reason function, see evaluateArmBlock). A refusal is logged and
    // announced with an error tone (NotifyTask) — the LED shows the standing reason.
    // Every caller is edge-triggered (RC button press edge, on-board click, API verb), so
    // one request is one tone: no rate limit needed.
    // ARM 前判定（理由関数は evaluateArmBlock）。拒否はログ出力し、エラー音で知らせる
    // （NotifyTask）— 継続的な理由は LED が示す。呼び出し元は全て立ち上がりエッジ起因
    // （RC ボタン押下エッジ・機体クリック・API verb）なので、1要求=1音。レート制限は不要。
    // @design detailed_design.md §3 注9/注10 — ARM rejected with visible/audible cue [OK]
    const ArmBlock block = evaluateArmBlock();
    if (block != ArmBlock::None) {
        logArmRejected(block);
        notify_command.publish({static_cast<uint8_t>(NotifyEvent::ArmRejected),
                                static_cast<uint32_t>(esp_timer_get_time())});
        return false;
    }

    ESP_LOGI(TAG, "ARM accepted");
    transition(FlightState::ARMED_GROUND);
    return true;
}

/// @design detailed_design.md §3.1 — API takeoff: mode change + ARM        [OK]
bool StateManager::requestArmForTakeoff(FlightMode takeoff_mode)
{
    const FlightMode previous_mode = mode_;
    if (previous_mode != takeoff_mode) {
        requestModeChange(takeoff_mode);
    }
    if (state_ != FlightState::IDLE_GROUND) {
        return state_ == FlightState::ARMED_GROUND;
    }
    if (requestArm()) {
        return true;
    }
    // Refused: put the mode back (it is a request that failed, not a mode the pilot chose).
    // 拒否された: モードを戻す（パイロットが選んだモードではなく、失敗した要求のモード）。
    if (mode_ != previous_mode) {
        ESP_LOGI(TAG, "API takeoff refused: mode restored to %s", flightModeName(previous_mode));
        requestModeChange(previous_mode);
    }
    return false;
}

/// @design requirements.md §2 — ARMED_GROUND→IDLE_GROUND (DISARM),       [OK]
/// @design requirements.md §2 — FLYING→IDLE_GROUND (pilot DISARM)        [OK]
bool StateManager::requestDisarm()
{
    // DISARM is a pilot kill action. From ARMED_GROUND it stops the idle motors;
    // from any airborne state (TAKEOFF/FLYING/LANDING) it is an emergency cut
    // straight to IDLE_GROUND. Requirements §2 lists both "ARMED_GROUND →
    // IDLE_GROUND (DISARM)" and "FLYING → IDLE_GROUND (pilot DISARM)", so the
    // gate is "armed", not "ARMED_GROUND only". Motors are zeroed by ControlTask
    // once isArmed() goes false. Use the free function (the member isArmed()
    // hides it inside this scope).
    // DISARM はパイロットのキル操作。ARMED_GROUND ではアイドルのモータを止め、空中状態
    // (TAKEOFF/FLYING/LANDING)からは IDLE_GROUND への緊急カット。要件§2 は
    // 「ARMED_GROUND→IDLE_GROUND」と「FLYING→IDLE_GROUND(パイロット DISARM)」の両方を
    // 挙げるため、判定条件は「ARMED_GROUND 限定」でなく「armed」。モータは isArmed() が
    // false になり次第 ControlTask が 0 にする。
    if (!sf::isArmed(state_)) {
        ESP_LOGD(TAG, "DISARM rejected: not armed (state=%s)",
                 flightStateName(state_));
        return false;
    }

    // ALT_HOLD/POS_HOLD while FLYING: a pilot DISARM starts an AUTO-LANDING (gradual
    // descent) instead of an immediate motor cut — these altitude-controlled modes can
    // land themselves, and cutting motors mid-air would just drop the craft (user
    // request, 2026-06-14). Touchdown then disarms (LANDING → IDLE_GROUND via the ToF
    // landing detector). A DISARM AGAIN while already LANDING falls through to the
    // immediate cut below (state_ != FLYING) — pressing twice is the abort/emergency.
    // FLIP falls through the same way (state_ == FLIP, not FLYING): a DISARM mid-
    // maneuver is an immediate cut, same as ACRO/STABILIZE — plan §4.3 FLIP row.
    // ALT_HOLD/POS_HOLD の FLYING 中: パイロット DISARM は即モータ停止でなく自動着陸
    // （緩降下）を開始する — 高度制御モードは自力で着陸でき、空中でモータを切ると機体が
    // 落ちるだけ（ユーザー要望 2026-06-14）。接地で DISARM（LANDING→IDLE_GROUND, ToF 着陸
    // 検出）。LANDING 中の再 DISARM は下の即カットに落ちる（state_ != FLYING）— 2回押しが中断/緊急。
    // FLIP も同様に下へ落ちる（state_ == FLIP であり FLYING でない）: マニューバ中の
    // DISARM は ACRO/STABILIZE と同じ即カット — plan §4.3 FLIP 行。
    // TAKEOFF is not in this list: the DISARM there is the immediate cut below (a landing started on the floor can float the craft up; detailed_design.md §3 note 5).
    // TAKEOFF は対象外: そこでの DISARM は下の即時カット（床から始めた着陸は機体を浮かせうる。detailed_design.md §3 注5）。
    if (state_ == FlightState::FLYING && mode_ >= FlightMode::ALT_HOLD) {
        ESP_LOGI(TAG, "DISARM in %s → auto-land (LANDING)", flightModeName(mode_));
        transition(FlightState::LANDING);
        return true;
    }

    ESP_LOGI(TAG, "DISARM accepted (%s → IDLE_GROUND)", flightStateName(state_));
    transition(FlightState::IDLE_GROUND);
    return true;
}

bool StateManager::requestEmergencyStop()
{
    if (!sf::isArmed(state_)) {
        ESP_LOGD(TAG, "Emergency stop rejected: not armed (state=%s)",
                 flightStateName(state_));
        return false;
    }
    // Unconditional immediate motor cut — bypasses the ALT/POS auto-landing (API
    // `emergency`, or aborting an in-progress auto-land). Motors zero once isArmed() is false.
    // Also the abort path out of FLIP (plan §3.5: "API emergency は既存どおり即 DISARM").
    // 無条件の即モータ停止 — ALT/POS 自動着陸を迂回（API `emergency` / 自動着陸の中断）。
    // FLIP からの打ち切り経路も兼ねる（plan §3.5: 「API emergency は既存どおり即 DISARM」）。
    ESP_LOGW(TAG, "EMERGENCY STOP (%s → IDLE_GROUND)", flightStateName(state_));
    transition(FlightState::IDLE_GROUND);
    return true;
}

void StateManager::notifyTakeoff()
{
    if (state_ != FlightState::ARMED_GROUND) {
        return;
    }
    ESP_LOGI(TAG, "Takeoff detected");
    transition(FlightState::TAKEOFF);
}

void StateManager::notifyTakeoffComplete()
{
    if (state_ != FlightState::TAKEOFF) {
        return;
    }
    ESP_LOGI(TAG, "Takeoff complete → FLYING/%s", flightModeName(mode_));
    transition(FlightState::FLYING);
}

/// @design detailed_design.md §3.1 — FLYING -> LANDING (land request) / ARMED_GROUND -> IDLE_GROUND (land request) rows  [OK]
void StateManager::notifyLandingRequest()
{
    // ARMED_GROUND in an altitude mode (ALT_HOLD / POS_HOLD): the motors have only begun to spool on the floor and the auto-takeoff has not started, so the immediate cut is the safe answer (a landing from
    // the floor has nothing to land from). In a manual mode (ACRO / STABILIZE) the pilot holds the throttle and the craft can lift off from ARMED_GROUND (a throttle of 0.499 is enough), so it is not known to be on
    // the floor: the request is refused, as it was before.
    // 高度モード（ALT_HOLD / POS_HOLD）の ARMED_GROUND: モータは床の上でスプールを始めただけで、自動離陸も始まっていないので、即時のモータ停止が安全（床の上には、そこから降りる高さが無い）。
    // 手動モード（ACRO / STABILIZE）ではスロットルをパイロットが操作し、ARMED_GROUND からでも浮ける（スロットル 0.499 で足りる）ので、床の上とは限らない: 要求は従来どおり拒否する。
    if (state_ == FlightState::ARMED_GROUND) {
        if (!flightModeNeedsAltitude(mode_)) {
            return;
        }
        ESP_LOGI(TAG, "Landing requested on the floor (ARMED_GROUND) -> DISARM");
        requestDisarm();
        return;
    }
    // TAKEOFF is not accepted: a landing started on the floor can float the craft up (detailed_design.md §3 note 5). The API `land` that arrives in TAKEOFF is held by ApiTask until the TAKEOFF has been left (parked during a `takeoff` wait, waited for by cmdLand outside one).
    // TAKEOFF は受け付けない: 床から始めた着陸は機体を浮かせうる（detailed_design.md §3 注5）。TAKEOFF に届いた API の `land` は、TAKEOFF を抜けるまで ApiTask が保留する（`takeoff` の待機中なら後回しの行、待機の外なら cmdLand が待つ）。
    if (state_ != FlightState::FLYING) {
        return;
    }
    ESP_LOGI(TAG, "Landing requested");
    transition(FlightState::LANDING);
}

void StateManager::notifyTakeoffEndedByLanding()
{
    if (state_ != FlightState::TAKEOFF) {
        return;
    }
    ESP_LOGI(TAG, "Takeoff ended by a landing (the floor evidence is latched and not released)");
    transition(FlightState::LANDING);
}

void StateManager::notifyLandingComplete()
{
    if (state_ != FlightState::LANDING) {
        return;
    }
    ESP_LOGI(TAG, "Landing complete");
    if (last_landing_end_ == LandingEnd::None) {
        last_landing_end_ = LandingEnd::Detected;   // the two ToF-silent kinds are set by their callers / ToF 出力停止の 2 種類は呼び出し側が設定済み
    }
    transition(FlightState::IDLE_GROUND);
}

/// @design requirements.md §2 — Soft landing / touch-and-go           [OK]
void StateManager::notifySoftLanding()
{
    if (state_ != FlightState::FLYING) {
        return;
    }
    ESP_LOGI(TAG, "Soft landing → ARMED_GROUND");
    transition(FlightState::ARMED_GROUND);
}

/// @design detailed_design.md §3.1 — FLIP row, "Flip request" column  [OK]
/// @design docs/plans/flip-maneuver-plan.md §4.3                     [OK]
bool StateManager::requestFlip(FlipDirection direction)
{
    // C1 gate: FLYING only. This single check also covers the "busy" cell (state_
    // already FLIP, so state_ != FLYING) and every other non-flying state — the
    // normative table's three FLIP-request outcomes collapse to one guard because
    // they all reduce to "is the current state FLYING". Execution conditions
    // C2-C8 are judged by the controller, not here (see header comment).
    // 実行条件 C1 の判定: FLYING のみ。この単一チェックで「busy」（既に state_==FLIP
    // ゆえ state_ != FLYING）と他の全非飛行状態も兼ねる — 規範表の3通りの Flip 要求
    // 結果は「現在状態が FLYING か」に帰着するため単一の判定で足りる。実行条件 C2-C8
    // はここでなく制御器が判定する（ヘッダコメント参照）。
    if (state_ != FlightState::FLYING) {
        ESP_LOGD(TAG, "Flip rejected: not FLYING (state=%s)", flightStateName(state_));
        return false;
    }
    ESP_LOGI(TAG, "Flip requested (direction=%s)", flipDirectionName(direction));
    flip_direction_ = direction;
    transition(FlightState::FLIP);
    return true;
}

/// @design docs/plans/flip-maneuver-plan.md §4.3 — FLIP → FLYING row  [OK]
void StateManager::notifyFlipComplete()
{
    if (state_ != FlightState::FLIP) {
        return;
    }
    ESP_LOGI(TAG, "Flip complete → FLYING");
    transition(FlightState::FLYING);

    // Deferred battery emergency (handleAlert, LOW_BATTERY/EMERGENCY case below):
    // apply the existing rule (airborne + EMERGENCY → LANDING) now that FLIP has
    // ended, mirroring how comm-loss deferral resolves via update() once state_
    // becomes FLYING again (plan §3.5/§9 item 2).
    // 保留していた電池緊急（下の handleAlert LOW_BATTERY/EMERGENCY 分岐）: FLIP 終了後に
    // 既存則（空中+EMERGENCY→LANDING）を適用する。通信断の保留が state_ が FLYING に
    // 戻った時点で update() 経由で解消されるのと同じ考え方（plan §3.5/§9 項目2）。
    if (battery_emergency_pending_) {
        battery_emergency_pending_ = false;
        ESP_LOGE(TAG, "Deferred battery emergency → LANDING");
        transition(FlightState::LANDING);
    }

    // A ToF-silence episode that began (or is still going) during the maneuver: the TOF_SILENT
    // alerts of the FLIP window were ignored, so decide NOW from the standing level instead of
    // flying on ALT_HOLD/POS_HOLD on the estimate until the next re-raised alert (up to ~1 s).
    // handleToFSilenceAlert() reads state/mode/level and does nothing in LANDING (battery above).
    // マニューバ中に始まった（または続いている）ToF 出力停止のエピソード: FLIP 窓の TOF_SILENT アラートは
    // 無視されたので、次に再発報されるアラート（最大約 1 s）まで推定だけで ALT_HOLD/POS_HOLD を
    // 飛ばさず、継続状態（レベル）から「今」判断する。handleToFSilenceAlert() は状態・モード・レベルを
    // 読み、LANDING（上の電池）では何もしない。
    if (isToFSilent()) {
        ESP_LOGW(TAG, "ToF silence episode during the maneuver: decided now that FLIP is over");
        handleToFSilenceAlert();
    }
}

/// @design requirements.md §2 — IDLE_GROUND ↔ IDLE_HELD by ToF        [OK]
void StateManager::notifyIdleGroundHeld(bool is_held)
{
    if (is_held && state_ == FlightState::IDLE_GROUND) {
        ESP_LOGI(TAG, "Drone held in hand");
        transition(FlightState::IDLE_HELD);
    } else if (!is_held && state_ == FlightState::IDLE_HELD) {
        ESP_LOGI(TAG, "Drone placed on ground");
        transition(FlightState::IDLE_GROUND);
    }
}

/// While a downward-ToF silence episode is in progress, refuse a switch INTO ALT_HOLD / POS_HOLD
/// (they would fly on an altitude nobody can vouch for; ACRO / STABILIZE stay available). Returns
/// true when refused. The refusal is decided by sf::decideToFModeRefusal (tof_silence_policy.hpp):
///  - on the ground the pilot's switch edge PERSISTS (StateTask re-applies it every cycle) and the
///    mode switches by itself once the episode has ended (ModeBlock::TofSilent);
///  - IN FLIGHT the edge is CONSUMED (StateTask drops it, see isModeRequestConsumed): the mode must
///    not switch by itself at the instant the ToF recovers; the pilot flips the switch again
///    (ModeBlock::TofSwitchConsumed, shown on CLI `status`).
/// The reason is published on SystemMode.mode_block and logged once per refused request, not on
/// each retry. An accepted request ends the refusal.
/// 下向き ToF の出力停止エピソード中は、ALT_HOLD / POS_HOLD への切り替えを拒否する（保証できない高度で
/// 飛ぶことになる。ACRO / STABILIZE は使える）。拒否したら true。拒否は sf::decideToFModeRefusal
/// （tof_silence_policy.hpp）が決める:
///  - 地上ではパイロットのスイッチのエッジは「持続」し（StateTask が毎周期再適用）、エピソードが終わると
///    自動でモードが切り替わる（ModeBlock::TofSilent）。
///  - 飛行中はエッジを「消費」する（StateTask が捨てる。isModeRequestConsumed 参照）: ToF が復帰した
///    瞬間にモードがパイロットの操作なしに切り替わってはならない。パイロットがスイッチを入れ直す
///    （ModeBlock::TofSwitchConsumed。CLI `status` に出す）。
/// 理由は SystemMode.mode_block に出し、ログは再試行ごとでなく拒否した要求ごとに 1 回出す。
/// 受理されれば拒否は終わる。
/// @design detailed_design.md §3.1 — mode change refused while the ToF is silent [OK]
bool StateManager::refuseModeForToFSilence(FlightMode new_mode)
{
    const ToFModeRefusal refusal = decideToFModeRefusal(state_, new_mode, isToFSilent());
    if (refusal == ToFModeRefusal::None) {
        clearModeBlock();   // an accepted request ends the refusal / 受理で拒否は終わる
        return false;
    }
    const ModeBlock block = (refusal == ToFModeRefusal::Consume) ? ModeBlock::TofSwitchConsumed
                                                                 : ModeBlock::TofSilent;
    last_mode_request_consumed_ = (refusal == ToFModeRefusal::Consume);
    if (mode_block_ != block) {
        if (refusal == ToFModeRefusal::Consume) {
            ESP_LOGW(TAG, "Mode change to %s refused: downward ToF is silent (no altitude). IN FLIGHT "
                          "the switch request is CONSUMED: flip the switch again after the ToF has "
                          "recovered", flightModeName(new_mode));
        } else {
            ESP_LOGW(TAG, "Mode change to %s refused: downward ToF is silent (no altitude). The "
                          "request persists and applies when the silence episode ends",
                     flightModeName(new_mode));
        }
        mode_block_ = block;
        publishMode();
    }
    return true;
}

/// True in the states that accept a flight-mode change (see the rule below).
/// フライトモード変更を受理する状態なら true（下の規則を参照）。
bool StateManager::modeChangeAllowedInState() const
{
    // Mode changes are accepted on the GROUND (disarmed or armed) and in FLYING.
    // Changing the mode while parked is the SAFEST time to do it (user spec,
    // 2026-06-11) — the controller stays gated (zero thrust in ALT/POS until the
    // auto-takeoff verb), and the body LEDs show the selected mode. Rejected
    // only mid-transition (INIT/TAKEOFF/LANDING — a switch flip mid-sequence
    // is applied on reaching FLYING/IDLE by the state task's per-cycle check)
    // and while held in the hand (IDLE_HELD). FLIP is rejected the same way as
    // TAKEOFF/LANDING (not listed explicitly — it is simply absent from
    // the list below): a switch flip during the maneuver persists
    // and applies once FLIP → FLYING (detailed_design.md §3.1 FLIP row).
    // モード変更は「地上」（DISARM/ARM とも）と FLYING で受理する。設置時の変更が
    // 最も安全（ユーザー仕様, 2026-06-11）— 制御器は判定され（ALT/POS は自動離陸
    // verb まで推力ゼロ）、本体 LED が選択モードを表示する。拒否は遷移中のみ
    // （INIT/TAKEOFF/LANDING — シーケンス途中のスイッチは FLYING/IDLE 到達時に
    // state task の周期チェックが適用する）と手持ち中（IDLE_HELD）。FLIP も
    // TAKEOFF/LANDING と同じく拒否（下の一覧に単に含めないだけ）:
    // マニューバ中のスイッチ反転は持続し、FLIP → FLYING 到達時に適用される
    // （detailed_design.md §3.1 FLIP 行）。
    return state_ == FlightState::IDLE_GROUND ||
           state_ == FlightState::ARMED_GROUND ||
           state_ == FlightState::FLYING;
}

bool StateManager::requestModeChange(FlightMode new_mode)
{
    last_mode_request_consumed_ = false;   // set again only by a refusal that consumes the edge / エッジを消費する拒否だけが再び立てる
    if (!modeChangeAllowedInState()) {
        ESP_LOGD(TAG, "Mode change rejected: state %s", flightStateName(state_));
        return false;
    }

    if (new_mode == mode_) {
        clearModeBlock();   // the pilot asked for what we already fly: nothing is refused now / 既に飛んでいるモードの要求: 拒否中の要求は無い
        return true;  // Already in requested mode / 既にリクエストされたモード
    }

    if (refuseModeForToFSilence(new_mode)) {
        return false;
    }

    FlightMode old_mode = mode_;
    ESP_LOGI(TAG, "Mode change: %s → %s",
             flightModeName(old_mode), flightModeName(new_mode));

    mode_ = new_mode;

    // Fire mode change callbacks
    // モード変更コールバックを呼び出す
    for (int i = 0; i < mode_callback_count_; i++) {
        if (mode_callbacks_[i]) {
            mode_callbacks_[i](old_mode, new_mode);
        }
    }

    publishMode();
    return true;
}

// =============================================================================
// Alert Handling
// アラート処理
//
// @design architecture.md §4 — FAILSAFE as event                      [OK]
// @design requirements.md §9 — Safety requirements                    [OK]
// =============================================================================

void StateManager::handleAlert(const SystemAlert& alert)
{
    auto type = static_cast<AlertType>(alert.type);

    ESP_LOGW(TAG, "Alert received: type=%d severity=%d", alert.type, alert.severity);

    switch (type) {
        case AlertType::GYRO_ANOMALY:
        case AlertType::IMPACT:
            handleImpactAlert(type);
            break;

        case AlertType::COMM_LOST:
            handleCommLostAlert(alert);
            break;

        case AlertType::LOW_BATTERY:
            handleLowBatteryAlert(alert);
            break;

        case AlertType::TOF_SILENT:
            // The downward ToF stopped producing samples (Failsafe raised this while armed and
            // re-raises it about once a second). The response depends on state and mode, see
            // handleToFSilenceAlert / tof_silence_policy.hpp.
            // 下向き ToF がサンプルを出さなくなった（Failsafe が ARM 中に発報し、約 1 秒ごとに再発報する）。
            // 応答は状態とモードによる。handleToFSilenceAlert / tof_silence_policy.hpp 参照。
            handleToFSilenceAlert();
            break;

        case AlertType::USB_POWER:
            // USB power → ARM prohibited (checked in requestArm)
            // USB給電 → ARM禁止（requestArmで確認）
            break;

        case AlertType::ESKF_DIVERGED:
            handleEskfDivergedAlert();
            break;

        default:
            break;
    }
}

/// The 3 G impact while LANDING with a silent ToF IS the touchdown (a descent without a ToF meets the floor at 0.5-1 m/s, 6-9 g in SILS;
/// no other detector sees it first): the landing is complete, not an emergency. The crash threshold itself (Failsafe) is unchanged, and
/// GYRO_ANOMALY stays an emergency. Returns true when it handled the alert.
/// ToF 出力停止中の LANDING での 3 G の衝撃は接地そのもの（ToF なしの降下は 0.5〜1 m/s で床に当たり、SILS で 6〜9 g。他の検出器が先に
/// 見ることはない）: 非常でなく着陸の完了。墜落検出の閾値そのもの（Failsafe）は変えず、GYRO_ANOMALY は非常のまま。処理したら true。
/// @design detailed_design.md §3.1 — ToF silent, LANDING row: the impact is the touchdown [OK]
bool StateManager::completeBlindLandingByImpact(AlertType type)
{
    if (type != AlertType::IMPACT || state_ != FlightState::LANDING || !isToFSilent()) {
        return false;
    }
    ESP_LOGW(TAG, "Touchdown while the ToF is silent (impact) → landing complete, not an emergency");
    last_landing_end_ = LandingEnd::ToFSilentImpact;
    notifyLandingComplete();
    return true;
}

/// GYRO_ANOMALY and IMPACT: crash / abnormal rate → immediate DISARM.
/// GYRO_ANOMALY と IMPACT: 衝突 / 異常角速度 → 即時 DISARM。
void StateManager::handleImpactAlert(AlertType type)
{
    // FLIP window exception (plan §3.5/§3.7, detailed_design.md §3.1 FLIP
    // row): a commanded flip legitimately spins past the 800 deg/s crash
    // threshold, so the alert is IGNORED (logged only) while state_ ==
    // FLIP. The detector (ImuAnomalyDetector) is unchanged — only the
    // judgment here differs (INV-3: detection vs. judgment separation,
    // architecture.md §4). Falls through to the same handling as IMPACT
    // in every other state.
    // FLIP 窓の例外（plan §3.5/§3.7、detailed_design.md §3.1 FLIP 行）:
    // 指令された宙返りは正当に 800 deg/s の墜落しきい値を超えて回転するため、
    // state_ == FLIP の間はアラートを無視する（ログのみ）。検出器
    // （ImuAnomalyDetector）は変更せず、判断側のみが異なる（INV-3: 検出と
    // 判断の分離、architecture.md §4）。それ以外の状態では IMPACT と同じ扱いに
    // フォールスルーする。
    if (type == AlertType::GYRO_ANOMALY && state_ == FlightState::FLIP) {
        ESP_LOGW(TAG, "Gyro anomaly ignored during %s (commanded rotation)",
                 flightStateName(state_));
        return;
    }
    // Crash / abnormal rate → immediate DISARM, unconditional on armed state
    // (requirements §9: no airborne-only exception). Gate on isArmed, NOT
    // isAirborne: ARMED_GROUND has idling props, and the detection layer
    // (failsafe.cpp / imu_task.cpp) already feeds samples whenever isArmed,
    // so a craft knocked over right after ARM on the ground must also DISARM.
    // transition(IDLE_GROUND) from ARMED_GROUND zeroes the motors via the
    // same DISARM path as from the air. IMPACT itself is NOT exempted during
    // FLIP (plan §3.5: "既存どおり即 DISARM" — a real collision must still cut
    // the motors mid-maneuver).
    // 衝突 / 異常角速度 → 状態に依らず即時DISARM（要件§9: 空中限定の例外なし）。
    // 判定条件は isAirborne でなく isArmed: ARMED_GROUND はプロペラがアイドル回転
    // しており、検出層（failsafe.cpp / imu_task.cpp）も isArmed の間は常にサンプル
    // を供給するため、ARM 直後に地上で倒した機体も DISARM する必要がある。
    // ARMED_GROUND からの transition(IDLE_GROUND) は空中と同じ DISARM 経路で
    // モータをゼロにする。IMPACT 自体は FLIP 中も例外にしない（plan §3.5:
    // 「既存どおり即 DISARM」— マニューバ中の実衝突もモータを止める必要がある）。
    if (completeBlindLandingByImpact(type)) {
        return;
    }
    if (sf::isArmed(state_)) {
        ESP_LOGE(TAG, "Impact/anomaly → emergency DISARM");
        transition(FlightState::IDLE_GROUND);
    }
}

/// COMM_LOST: arm the hover-then-land timer. / COMM_LOST: ホバーしてから着陸するタイマを起動する。
void StateManager::handleCommLostAlert(const SystemAlert& alert)
{
    // Communication lost → keep hovering, then auto-land after the grace period
    // (requirements §9: hover hold 3 s → auto landing). We do NOT land here:
    // arm a timer (idempotent — only the first loss while airborne arms it) and
    // let update() command FLYING → LANDING once kCommLossHoverUs elapses.
    // Gate on isAirborne (TAKEOFF/FLYING/LANDING/FLIP), NOT on FLYING only: a
    // link lost during TAKEOFF used to be consumed here and never re-evaluated,
    // so the craft kept climbing on the stale setpoint with no failsafe at all.
    // The failsafe also re-raises COMM_LOST periodically while the link stays
    // lost (level, not edge), so a missed alert is not fatal either way. FLIP is
    // included in isAirborne (flight_state.hpp), so a loss during FLIP arms the
    // timer here but update() only fires the transition when state_ == FLYING —
    // this naturally defers the auto-land until FLIP → FLYING, matching plan
    // §3.5/§4.3 ("通信途絶は FLIP 完了後に既存則") with no extra state needed.
    // 通信途絶 → ホバーを維持し、猶予経過後に自動着陸（要件§9: ホバー維持3秒→
    // 自動着陸）。ここでは着陸しない: タイマを起動し（冪等 — 空中での最初の喪失
    // のみ起動）、kCommLossHoverUs 経過で update() が FLYING → LANDING を指令する。
    // 判定条件は FLYING 限定でなく isAirborne（TAKEOFF/FLYING/LANDING/FLIP）: 従来は
    // TAKEOFF 中の喪失がここで消費されて二度と再評価されず、機体は stale な
    // setpoint のままフェイルセーフなしで上昇し続けた。failsafe 側もリンク喪失中は
    // COMM_LOST を周期的に再発報する（エッジでなくレベル）ので、取りこぼしても致命傷
    // にならない。FLIP は isAirborne に含まれる（flight_state.hpp）ため、FLIP 中の
    // 喪失もここでタイマは起動するが、update() は state_ == FLYING のときだけ遷移を
    // 実行する — これにより FLIP → FLYING まで自動着陸が自然に保留される
    // （plan §3.5/§4.3「通信途絶は FLIP 完了後に既存則」を追加の状態なしで満たす）。
    if (isAirborne(state_) && !comm_lost_pending_) {
        ESP_LOGW(TAG, "Comm lost → hover %lu ms then LANDING",
                 static_cast<unsigned long>(kCommLossHoverUs / 1000));
        comm_lost_pending_ = true;
        comm_lost_time_us_ = alert.timestamp;
    }
}

/// LOW_BATTERY: a WARNING is a notification; an EMERGENCY is decided by decideBatteryEmergency() (takeoff_policy.hpp).
/// LOW_BATTERY: WARNING は通知のみ。EMERGENCY は decideBatteryEmergency()（takeoff_policy.hpp）が判断する。
/// @design detailed_design.md §3.1 — battery EMERGENCY rows (TAKEOFF: LANDING with evidence of the air, DISARM without; FLYING: LANDING; FLIP: deferred) [OK]
/// @design architecture.md INV-3 — detection in ImuTask (tof_health.floor_leaving), decision in takeoff_policy.hpp, execution here [OK]
void StateManager::handleLowBatteryAlert(const SystemAlert& alert)
{
    // WARNING (3.4 V): notification only (the buzzer is NotifyTask's).
    // EMERGENCY (3.0 V): the pack is about to sag below the cutoff. The EMERGENCY is a standing level: the failsafe raises it on
    // entry into the region below the critical voltage and again about every 5 s while the voltage stays there
    // (battery_alert_policy.hpp), so an alert consumed in a state that only logs it (the ground, LANDING) does not leave a later
    // flight without it. In FLIP the landing is deferred (a flag; notifyFlipComplete() lands once FLYING is back), because
    // FLIP -> LANDING directly would skip the FLIP -> FLYING onExit (position / altitude re-capture).
    // WARNING（3.4 V）: 通知のみ（ブザーは NotifyTask）。
    // EMERGENCY（3.0 V）: パックがカットオフを割る寸前。EMERGENCY は継続するレベル: failsafe は危険電圧より下の領域へ入ったときに
    // 発報し、電圧がそこに留まる間は約 5 s ごとに再発報する（battery_alert_policy.hpp）ので、ログ出力だけの状態（地上、LANDING）で
    // 消費されたアラートが、後の飛行から失われることはない。FLIP では着陸を保留する（フラグ。FLYING に戻ったら notifyFlipComplete() が
    // 着陸させる）: FLIP → LANDING に直接遷移すると、FLIP → FLYING の onExit（位置・高度目標の取り直し）を飛ばすため。
    if (static_cast<AlertSeverity>(alert.severity) != AlertSeverity::EMERGENCY) {
        ESP_LOGW(TAG, "Low battery warning");
        return;
    }
    const bool tof_silent = isToFSilent();
    BatteryEmergencyFacts facts;
    facts.state         = state_;
    facts.tof_silent    = tof_silent;
    facts.floor_leaving = tof_health.latest().floor_leaving && !tof_silent;   // detected by ImuTask (INV-3), only a healthy ToF counts / ImuTask が検出（INV-3）。健全な ToF だけが数える
    switch (decideBatteryEmergency(facts)) {
    case BatteryEmergencyAction::Defer:
        battery_emergency_pending_ = true;
        ESP_LOGW(TAG, "Battery emergency deferred (FLIP in progress)");
        break;
    case BatteryEmergencyAction::Land:
        ESP_LOGE(TAG, "Battery emergency → LANDING");
        transition(FlightState::LANDING);
        break;
    case BatteryEmergencyAction::CutMotors:
        ESP_LOGE(TAG, "Battery emergency in TAKEOFF without evidence of the air (the healthy ToF has not shown the craft off the floor) → DISARM");
        requestDisarm();   // in TAKEOFF a DISARM is the motor cut / TAKEOFF での DISARM はモータ停止
        break;
    case BatteryEmergencyAction::LogOnly:
        ESP_LOGW(TAG, "Battery emergency: no transition in %s", flightStateName(state_));
        break;
    }
}

/// ESKF_DIVERGED: command an estimator reset (not a state change).
/// ESKF_DIVERGED: 推定器リセットを指令する（状態変更なし）。
void StateManager::handleEskfDivergedAlert()
{
    // ESKF diverged → command an estimator reset. ImuTask consumes estimator_command and applies it
    // to the active IEstimator (architecture §4: ESKF divergence → ESKF reset).
    // ESKF発散 → 推定器リセットを指令。ImuTask が estimator_command を消費しアクティブな
    // IEstimator に適用する（architecture §4: ESKF発散 → ESKFリセット）。
    ESP_LOGW(TAG, "ESKF diverged → estimator reset commanded");
    estimator_command.publish({static_cast<uint8_t>(EstimatorCmd::Reset),
                               static_cast<uint32_t>(esp_timer_get_time()),
                               0});
}

// =============================================================================
// Pairing — parallel state machine (NotPaired / Pairing / Paired)
//
// @design requirements.md §2 — PairingState                            [OK]
// @design architecture.md §4 — Pairing positioning (StateManager owns) [OK]
// @design detailed_design.md §3 — Pairing state transitions            [OK]
// =============================================================================

void StateManager::requestPairing()
{
    // Pairing is a disarmed-vehicle-only activity: it advertises itself so a
    // transmitter can bind, but never spins the motors. Reject from INIT / armed /
    // airborne (we never re-pair in flight). Idempotent: if already searching, do
    // nothing (avoids re-publish churn).
    // IDLE_GROUND or IDLE_HELD (2026-09-12): pairing must also be reachable while the
    // vehicle is held in hand — pressing the button in hand is the common way users
    // start pairing (requirements.md §2 / detailed_design.md §3.1 updated to match).
    // This does NOT weaken the ARM guard: requestArm() still only accepts IDLE_GROUND,
    // and ARM is separately rejected while Pairing, so a held vehicle can never spin
    // its motors just because it can now start advertising.
    // ペアリングは disarmed の機体限定の活動: 自分自身を広告して送信機がバインドできるように
    // するだけで、モータは回さない。INIT/武装/空中からは拒否（飛行中に再ペアしない）。
    // 冪等: 既に探索中なら何もしない。
    // IDLE_GROUND または IDLE_HELD（2026-09-12）: 手に持った状態でもペアリングに入れる
    // 必要がある — ボタンを手持ちのまま押すのは利用者にとって一般的な操作（requirements.md
    // §2 / detailed_design.md §3.1 を合わせて更新済み）。ARM のガードはそのまま変えない:
    // requestArm() は引き続き IDLE_GROUND のみ受理し、Pairing 中は ARM を別途拒否するため、
    // 広告を開始できるようになっても手持ちの機体のモータが回ることはない。
    if (state_ != FlightState::IDLE_GROUND && state_ != FlightState::IDLE_HELD) {
        ESP_LOGD(TAG, "Pairing rejected: not in IDLE_GROUND/IDLE_HELD (state=%s)",
                 flightStateName(state_));
        return;
    }
    if (pairing_state_ == PairingState::Pairing) {
        return;   // already searching / 既に探索中
    }
    ESP_LOGI(TAG, "Pairing started (broadcasting; awaiting a transmitter)");
    pairing_state_ = PairingState::Pairing;
    publishPairingState();

    // One-shot pairing tone (NotifyTask plays it). The pairing LED (blue fast blink)
    // is driven continuously by NotifyTask reading pairing_state, so it needs no event.
    // ペアリング開始の単発音（NotifyTask が鳴らす）。ペアリング LED（青の高速点滅）は
    // NotifyTask が pairing_state を読んで連続駆動するためイベント不要。
    notify_command.publish({static_cast<uint8_t>(NotifyEvent::PairingMode),
                            static_cast<uint32_t>(esp_timer_get_time())});
}

void StateManager::notifyPairingComplete()
{
    // sf_comm reports it has bound to a controller (live pairing or NVS restore at
    // boot). Reflect Paired. Idempotent — repeated bind-status reports are common.
    // sf_comm が相手局にバインドした事実を報告（生のペアリング or 起動時の NVS 復元）。
    // Paired を反映する。冪等 — バインド状態の繰り返し報告は普通に起こる。
    if (pairing_state_ == PairingState::Paired) {
        return;
    }
    ESP_LOGI(TAG, "Pairing complete (bound to transmitter)");
    pairing_state_ = PairingState::Paired;
    publishPairingState();
}

bool StateManager::isToFSilent() const
{
    // The verdict is Failsafe's: a silence EPISODE (no new sample for kTofSilenceUs / only invalid
    // samples while flying / the estimator has not re-acquired the ToF yet), published every
    // 100 ms. It is read through this task's FailsafeStatusGuard: a status that nobody
    // refreshes reads as silent, so a stalled Failsafe can never leave the last "not silent" in
    // force; before the first Failsafe publish it reads silent too (the safe side). Read FRESH at
    // every call, not cached per update(): an alert handled earlier in the same StateTask cycle must
    // see the level that raised it, or the one-shot response is lost (found by SILS
    // tof_silent_armed_ground). The guard's watch is observed on every call (same task, same clock).
    // 判定は Failsafe のもの: ToF 出力停止の「エピソード」（kTofSilenceUs の間新サンプル無し／飛行中に
    // 無効サンプルだけ／推定器がまだ ToF を取り込み直していない）を 100 ms ごとに発行する。このタスクの
    // FailsafeStatusGuard を通して読む: 誰も更新しない状態は出力停止と読むので、Failsafe が停滞
    // しても最後の「出力停止でない」が効き続けることはない。Failsafe の最初の発行の前も出力停止と読む
    // （安全側）。update() ごとのキャッシュでなく、呼ぶたびに「最新」を読む: 同じ StateTask 周期の中で
    // 先に処理したアラートは、それを発報したレベルを見なければならず、さもないと 1 回限りの応答が
    // 失われる（SILS の tof_silent_armed_ground が発見）。ガードの監視器は呼ぶたびに観測する
    // （同じタスク、同じ時計）。
    tof_status_ = tof_guard_.guard(static_cast<uint32_t>(esp_timer_get_time()), failsafe_status.latest());
    return tof_status_.tof_silent;
}

/// Execute one decided ToF-silence action (the verbs of tof_silence_policy.hpp).
/// 判断済みの ToF 出力停止の動作 1 つを実行する（tof_silence_policy.hpp の動詞）。
/// @design detailed_design.md §3.1 — ToF silent rows (ARMED_GROUND / TAKEOFF / FLYING) [OK]
/// @design architecture.md INV-3 — decision in tof_silence_policy.hpp, execution here  [OK]
void StateManager::handleToFSilenceAlert()
{
    // Warning tone: the first WarnOnly alert and then every kToFWarnToneEveryN-th (the failsafe
    // re-raises about once a second; a tone each second over motor noise would be a nuisance).
    // 警告音: 最初の WarnOnly アラートと、以後 kToFWarnToneEveryN 回に 1 回（failsafe は約 1 秒ごとに
    // 再発報する。モータ騒音の中で毎秒の音は邪魔になる）。
    constexpr uint8_t kToFWarnToneEveryN = 5;

    switch (decideToFSilenceAction(state_, mode_, isToFSilent(), 0u, 0u)) {
        case ToFSilenceAction::Disarm:
            ESP_LOGE(TAG, "Downward ToF silent while armed on the ground → DISARM");
            transition(FlightState::IDLE_GROUND);
            break;
        case ToFSilenceAction::StartLanding:
            logToFSilenceLanding();
            transition(FlightState::LANDING);
            break;
        case ToFSilenceAction::WarnOnly:
            ESP_LOGW(TAG, "Downward ToF silent in %s/%s: NO forced landing, fly on "
                          "(altitude is unknown; ARM will be refused after landing)",
                     flightStateName(state_), flightModeName(mode_));
            if (tof_warn_count_ % kToFWarnToneEveryN == 0) {
                notify_command.publish({static_cast<uint8_t>(NotifyEvent::TofSilentWarn),
                                        static_cast<uint32_t>(esp_timer_get_time())});
            }
            ++tof_warn_count_;
            break;
        case ToFSilenceAction::CompleteLanding:   // timed by handleBlindLandingTimeout() / 時間は別関数
        case ToFSilenceAction::None:
        default:
            break;
    }
}

/// Log why an altitude mode lands on a ToF-silence episode (the reason decides the wording).
/// 高度モードが ToF 出力停止のエピソードで着陸する理由をログに出す（理由で文言が変わる）。
void StateManager::logToFSilenceLanding() const
{
    const auto reason = static_cast<ToFSilenceReason>(tof_status_.tof_reason);
    if (reason == ToFSilenceReason::NoSample) {
        ESP_LOGE(TAG, "Downward ToF silent (>= %lu ms without a sample) in %s/%s → LANDING",
                 static_cast<unsigned long>(limits::kTofSilenceUs / 1000),
                 flightStateName(state_), flightModeName(mode_));
        return;
    }
    ESP_LOGE(TAG, "Downward ToF unusable (%s) in %s/%s → LANDING",
             toFSilenceReasonName(reason), flightStateName(state_), flightModeName(mode_));
}

/// @design detailed_design.md §3.1 — ToF silent, LANDING row: ended by time           [OK]
void StateManager::handleBlindLandingTimeout(uint32_t now_us)
{
    const bool silent = isToFSilent();

    // LANDING clock: counted from the moment the state ENTERED LANDING, whether the ToF is alive or silent, and
    // cleared only when LANDING ends. A ToF that returns and stops again therefore does not extend it.
    // LANDING の時計: 状態が LANDING に「入った」時刻から、ToF が動作していても出力停止でも数え、LANDING が終わる
    // ときだけクリアする。ToF が戻ってまた止まっても延びない。
    landing_clock_.update(state_ == FlightState::LANDING, silent, now_us);
    const uint32_t blind_elapsed_us = landing_clock_.elapsedUs();

    if (decideToFSilenceAction(state_, mode_, silent, blind_elapsed_us, landing_clock_.silentElapsedUs()) ==
        ToFSilenceAction::CompleteLanding) {
        ESP_LOGW(TAG, "Downward ToF silent after %lu ms of LANDING → touchdown by time",
                 static_cast<unsigned long>(blind_elapsed_us / 1000));
        last_landing_end_ = LandingEnd::ToFSilentTimeout;
        notifyLandingComplete();
    }

    // The warning counter restarts with every silence (tone on the first alert of the next one).
    // 警告の回数は出力停止ごとに数え直す（次の出力停止の最初のアラートで音を鳴らす）。
    if (!silent) {
        tof_warn_count_ = 0;
    }
}

void StateManager::update(uint32_t now_us)
{
    // Observe the ToF-silence verdict through the staleness guard EVERY cycle (isToFSilent() also
    // does at every call; this keeps the watch observed when nothing else asks).
    // ToF 出力停止の判定を鮮度のガードを通して「毎周期」観測する（isToFSilent() も呼ぶたびに観測する。
    // 他に誰も尋ねなくても監視器が観測され続けるようにする）。
    (void)isToFSilent();

    // Keep the published ARM-block reason current (LED / CLI read it).
    // 発行済みの ARM 阻害理由を最新に保つ（LED/CLI が読む）。
    refreshArmBlock();

    // The mode-refusal reasons end: TofSilent (ground; the switch edge persists and is re-applied by
    // StateTask) when the episode ends; TofSwitchConsumed (in flight; the edge is gone) when the
    // craft is back on the ground, or when a mode request is accepted (requestModeChange).
    // モード拒否の理由が終わる: TofSilent（地上。スイッチのエッジは持続し StateTask が再適用する）は
    // エピソードが終わったとき。TofSwitchConsumed（飛行中。エッジはもう無い）は地上に戻ったとき、
    // またはモード要求が受理されたとき（requestModeChange）。
    if ((mode_block_ == ModeBlock::TofSilent && !isToFSilent()) ||
        (mode_block_ == ModeBlock::TofSwitchConsumed && !isAirborne(state_))) {
        clearModeBlock();
    }

    // Downward-ToF silence: the landing that the TOF_SILENT alert started ends by time when no
    // ToF can detect the touchdown (level-triggered, so it runs every cycle).
    // 下向き ToF の出力停止: TOF_SILENT アラートが始めた着陸は、ToF が接地を検出できないとき時間で
    // 終わる（レベル駆動なので毎周期実行する）。
    handleBlindLandingTimeout(now_us);

    // Comm-loss failsafe: deferred FLYING → LANDING (requirements §9).
    // 通信断フェイルセーフ: 遅延 FLYING → LANDING（要件§9）。
    handleCommLossGrace(now_us);
}

/// Comm-loss grace timer: lands once the hover grace after COMM_LOST has elapsed (see update()).
/// 通信断の猶予タイマ: COMM_LOST から猶予が経過したら着陸させる（update() 参照）。
void StateManager::handleCommLossGrace(uint32_t now_us)
{
    // The grace timer runs from the alert stamp, which the failsafe task wrote: it can be a few
    // us newer than now_us, and a pending loss can in principle outlast 35.8 min. comm_lost_watch_
    // takes the signed age and saturates, so neither case lands early or never. It is observed
    // EVERY cycle, pending or not, so the watch never sees a long gap between observations.
    // 猶予タイマは failsafe タスクが書いたアラートの時刻印から計る: now_us より数 us 新しいことが
    // あり、保留が 35.8 分を超えることも原理的にはある。comm_lost_watch_ は符号付きの経過で
    // 飽和するので、早すぎる着陸も、着陸しないままも起きない。保留中か否かに関わらず「毎周期」
    // 観測するので、観測の間に長い空きが生じない。
    comm_lost_watch_.observe(now_us, comm_lost_time_us_);

    // Nothing pending → nothing to do (the common case).
    // 保留なしなら何もしない（通常）。
    if (!comm_lost_pending_) {
        return;
    }

    // Cancel only once we are back on the ground: a pilot DISARM, impact-IDLE, or a
    // completed landing already ended the flight, so the auto-land is moot. While
    // still airborne the pending flag survives. (Recovery of the link does NOT cancel —
    // the requirement is an unconditional hover-then-land.)
    // 地上に戻ったらキャンセル: パイロット DISARM・衝撃 IDLE・着陸完了で飛行が終わって
    // いれば自動着陸は不要。空中にいる間は保留フラグを維持する。（リンク復帰では
    // キャンセルしない — 要件は無条件のホバー→着陸。）
    if (!isAirborne(state_)) {
        comm_lost_pending_ = false;
        return;
    }

    // Grace period elapsed in TAKEOFF or FLYING → land (decideCommLossLanding(), takeoff_policy.hpp). TAKEOFF
    // too: a TAKEOFF that does not complete (a craft the floor still holds, a ToF that shows the floor) used to
    // keep the craft on the stale setpoint with no failsafe at all. During LANDING the flag stays pending
    // (LANDING is already what we want); during FLIP it waits for FLYING.
    // 猶予経過が TAKEOFF または FLYING なら着陸（decideCommLossLanding()、takeoff_policy.hpp）。TAKEOFF も: 完了しない
    // TAKEOFF（床がまだ支えている機体、ToF が床を示している機体）では、機体が古い目標のままフェイルセーフなしで続いた。
    // LANDING 中はフラグを保留のまま維持（LANDING は既に目的の状態）、FLIP 中は FLYING まで待つ。
    if (decideCommLossLanding(state_, comm_lost_watch_.isSilent(kCommLossHoverUs))) {
        ESP_LOGW(TAG, "Comm lost → LANDING (hover grace elapsed)");
        comm_lost_pending_ = false;
        transition(FlightState::LANDING);
    }
}

void StateManager::forceIdle()
{
    ESP_LOGW(TAG, "Force IDLE_GROUND from %s", flightStateName(state_));
    transition(FlightState::IDLE_GROUND);
}

// =============================================================================
// Callback Registration
// コールバック登録
// =============================================================================

void StateManager::onExit(OnExitCallback callback)
{
    if (exit_callback_count_ < MAX_CALLBACKS) {
        exit_callbacks_[exit_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onExit callbacks (max=%d)", MAX_CALLBACKS);
    }
}

void StateManager::onEnter(OnEnterCallback callback)
{
    if (enter_callback_count_ < MAX_CALLBACKS) {
        enter_callbacks_[enter_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onEnter callbacks (max=%d)", MAX_CALLBACKS);
    }
}

void StateManager::onModeChange(OnModeChangeCallback callback)
{
    if (mode_callback_count_ < MAX_CALLBACKS) {
        mode_callbacks_[mode_callback_count_++] = callback;
    } else {
        ESP_LOGE(TAG, "Too many onModeChange callbacks (max=%d)", MAX_CALLBACKS);
    }
}

// =============================================================================
// Internal: Transition Execution
// 内部: 遷移実行
// =============================================================================

/// Back on the ground: reset the flight mode to STABILIZE (called by transition() on entering IDLE_GROUND).
/// 接地に戻った: 飛行モードを STABILIZE にリセットする（transition() が IDLE_GROUND に入るときに呼ぶ）。
void StateManager::resetModeOnGround()
{
    // On returning to the ground, reset the flight mode to STABILIZE so the NEXT
    // takeoff always starts in the direct-throttle mode that lifts off. Without this a
    // mode selected in flight (e.g. ALT_HOLD) PERSISTS across DISARM/emergency-IDLE, and
    // a re-takeoff in that mode never produces takeoff thrust from the ground — a real
    // crash→re-fly robustness bug found by crash_refly.scn. The controller learns its mode
    // ONLY via ControllerCmd::ModeChange (ControlTask does NOT read system_mode.sub_mode),
    // so we must FIRE the onModeChange callbacks here — not just set mode_. They publish the
    // mode change, which reconfigures the controller to STABILIZE and clears the stale
    // ALT/POS cascade + hover-thrust state; without the fire the controller keeps holding
    // hover on the re-takeoff and never climbs.
    // 接地に戻ったら飛行モードを STABILIZE にリセットし、次の離陸が必ず離陸スラストを出す直接
    // スロットルモードで始まるようにする。これが無いと飛行中に選んだモード（例 ALT_HOLD）が
    // DISARM/緊急IDLE を跨いで残り、そのモードでの再離陸は地上から離陸スラストを出せない
    // ——crash_refly.scn が見つけた実 crash→再飛行バグ。制御器はモードを ControllerCmd::
    // ModeChange 経由でのみ把握する（ControlTask は system_mode.sub_mode を読まない）ため、ここで
    // onModeChange コールバックを呼び出す必要がある（mode_ を変えるだけでは不十分）。呼び出しで
    // モード変更が発行され、制御器が STABILIZE へ再構成され古い ALT/POS カスケード＋ホバー推力
    // 状態がクリアされる。呼び出さないと制御器は再離陸でホバーを保持し続け上昇しない。
    if (mode_ != FlightMode::STABILIZE) {
        ESP_LOGI(TAG, "Flight mode reset to STABILIZE (on ground)");
        FlightMode old_mode = mode_;
        mode_ = FlightMode::STABILIZE;
        for (int i = 0; i < mode_callback_count_; i++) {
            if (mode_callbacks_[i]) {
                mode_callbacks_[i](old_mode, mode_);
            }
        }
    }
}

/// Switch to the new state and apply what the entry itself implies (between the onExit and the onEnter callbacks of transition()): the landing end is forgotten, a
/// battery emergency deferred during FLIP is cleared when FLIP is left WITHOUT returning to FLYING (DISARM / impact / emergency stop already land the craft on the ground
/// directly, so the deferred alert is moot; leaving it set would apply a stale emergency to some LATER, unrelated flip; the FLIP -> FLYING path re-applies it explicitly in
/// notifyFlipComplete()), and the flight mode is reset on entering IDLE_GROUND.
/// 新しい状態へ切り替え、入ること自体が意味する処理を行う（transition() の onExit と onEnter の間）: 着陸の終わり方を忘れ、FLIP を FLYING へ戻らずに抜けたときは FLIP 中に
/// 保留した電池緊急をクリアし（DISARM・衝撃・緊急停止は既に接地するので、保留したアラートは意味を失う。残すと無関係な「後の」flip に古い緊急事態を適用する。
/// FLIP → FLYING の経路は notifyFlipComplete() が明示的に再適用する）、IDLE_GROUND に入るときは飛行モードをリセットする。
void StateManager::enterState(FlightState old_state, FlightState new_state)
{
    state_ = new_state;
    state_since_us_ = static_cast<uint32_t>(esp_timer_get_time());   // the one clock of "time in this state" (SystemMode.state_since_us) / 「この状態にいる時間」の唯一の時計
    if (new_state == FlightState::LANDING) {
        last_landing_end_ = LandingEnd::None;   // the next landing's end is not known yet / 次の着陸の終わり方はまだ分からない
    }
    if (old_state == FlightState::FLIP && new_state != FlightState::FLYING) {
        battery_emergency_pending_ = false;
    }
    if (new_state == FlightState::IDLE_GROUND) {
        resetModeOnGround();
    }
}

void StateManager::transition(FlightState new_state)
{
    FlightState old_state = state_;

    if (old_state == new_state) {
        return;
    }

    ESP_LOGI(TAG, "Transition: %s → %s",
             flightStateName(old_state), flightStateName(new_state));

    // Fire onExit callbacks for old state
    // 旧状態のonExitコールバックを呼び出す
    for (int i = 0; i < exit_callback_count_; i++) {
        if (exit_callbacks_[i]) {
            exit_callbacks_[i](old_state, new_state);
        }
    }

    enterState(old_state, new_state);

    // Fire onEnter callbacks for new state
    // 新状態のonEnterコールバックを呼び出す
    for (int i = 0; i < enter_callback_count_; i++) {
        if (enter_callbacks_[i]) {
            enter_callbacks_[i](old_state, new_state);
        }
    }

    // Publish to system.mode topic
    // system.modeトピックに発行
    publishMode();
}

/// Re-evaluate the pre-arm gates and republish system_mode only when the reason changed
/// (the gates are cheap: topic reads, one param lookup; the publish is edge-only).
/// ARM 前判定を再評価し、理由が変わった時だけ system_mode を再発行する（判定は軽い:
/// トピック読み＋param 1回、発行はエッジのみ）。
void StateManager::refreshArmBlock()
{
    const ArmBlock block = evaluateArmBlock();
    if (block == arm_block_) {
        return;
    }
    arm_block_ = block;
    publishMode();
}

/// Clear the published mode-refusal reason (republish only when it was set).
/// 発行済みのモード拒否理由を下ろす（立っていたときだけ再発行）。
void StateManager::clearModeBlock()
{
    if (mode_block_ == ModeBlock::None) {
        return;
    }
    mode_block_ = ModeBlock::None;
    publishMode();
}

void StateManager::publishMode()
{
    SystemMode mode_msg = {};
    mode_msg.state = static_cast<uint8_t>(state_);
    mode_msg.sub_mode = static_cast<uint8_t>(mode_);
    mode_msg.armed = sf::isArmed(state_);
    mode_msg.arm_block = static_cast<uint8_t>(arm_block_);
    mode_msg.mode_block = static_cast<uint8_t>(mode_block_);
    mode_msg.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    mode_msg.state_since_us = state_since_us_;

    system_mode.publish(mode_msg);
}

void StateManager::publishPairingState()
{
    PairingStatus msg = {};
    msg.state = static_cast<uint8_t>(pairing_state_);
    msg.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    pairing_state.publish(msg);
}

}  // namespace sf
