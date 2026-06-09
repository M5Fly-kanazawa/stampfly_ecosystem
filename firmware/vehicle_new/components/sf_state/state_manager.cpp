/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
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
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>   // ROOT-CAUSE DIAG: raw printf markers (temporary)

static const char* TAG = "StateManager";

namespace sf {

/// Below this voltage the sensor_power reading is "unknown" (power monitor absent or a
/// failed read publishes 0). A 0/unknown reading must NOT block ARM — power monitoring is
/// Optional (hardware_init.md §5) — so the pre-arm voltage gate ignores readings below it.
/// この電圧未満は sensor_power の読みが「不明」（電源モニタ不在 or 読み失敗で 0 が発行）。
/// 電源監視は Optional（hardware_init.md §5）ゆえ 0/不明は ARM を阻まない — ARM 前電圧ゲートは
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
    std::printf("  ROOT: NIC-1 pre-esplog\n");
    ESP_LOGI(TAG, "Init complete → IDLE_GROUND");
    std::printf("  ROOT: NIC-2 post-esplog pre-transition\n");
    transition(FlightState::IDLE_GROUND);
    std::printf("  ROOT: NIC-3 post-transition\n");
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

    // --- Pre-arm gate 0: not actively pairing -----------------------------------------
    // While searching for a transmitter (PairingState::Pairing) the link is not yet
    // bound, so ARM is refused. A vehicle that boots unpaired auto-enters Pairing; it
    // must complete pairing (→ Paired) before it can be armed. Paired and (the transient)
    // NotPaired do not block — only the active search does.
    // ARM 前ゲート0: 探索中でないこと。送信機を探索中（PairingState::Pairing）はリンクが
    // 未バインドゆえ ARM を拒否する。未ペア起動は自動で Pairing に入り、ペア成立（→Paired）
    // するまで ARM できない。Paired と（過渡的な）NotPaired は阻まない — 探索中のみ阻む。
    if (pairing_state_ == PairingState::Pairing) {
        ESP_LOGW(TAG, "ARM rejected: pairing in progress");
        return false;
    }

    // --- Pre-arm gate 1: battery / USB power (requirements §9) ------------------------
    // Reject ARM if a VALID voltage reading is at or below the unsafe threshold — either
    // running on the USB rail (no pack) or a critically low battery. A 0/unknown reading
    // (power monitor absent → Optional, hardware_init.md §5) does NOT block. Threshold
    // from the safety.battery.usb_v param.
    // ARM 前ゲート1: 電池/USB 電源（要件§9）。有効な電圧読みが危険閾値以下なら ARM を拒否
    // （USB レール=電池なし or 危険な低電圧）。0/不明（電源モニタ不在→Optional）は阻まない。
    // 閾値は safety.battery.usb_v param から。
    float usb_v = 3.3f;
    params::get_float("safety.battery.usb_v", usb_v);
    const float voltage = sensor_power.latest().voltage;
    if (voltage > kVoltageValidMin && voltage <= usb_v) {
        ESP_LOGW(TAG, "ARM rejected: battery %.2fV <= %.2fV (USB/low — unsafe to fly)",
                 voltage, usb_v);
        return false;
    }

    // --- Pre-arm gate 2: boot calibration complete (requirements §9 / design §3) ------
    // The boot gyro/accel bias calibration must no longer be pending (ImuTask publishes
    // system_status.calibrated). Don't fly on a half-measured bias. Status via topic
    // (R16-style), not a cross-task object.
    // ARM 前ゲート2: 起動校正完了（要件§9 / 設計§3）。起動バイアス校正が保留中でないこと
    // （ImuTask が system_status.calibrated を発行）。半端なバイアスで飛ばさない。状態は
    // トピック経由（R16 流）。
    if (!system_status.latest().calibrated) {
        ESP_LOGW(TAG, "ARM rejected: boot calibration not complete");
        return false;
    }

    // --- Pre-arm gate 3: sensor health — DEFERRED ------------------------------------
    // A meaningful health gate needs sf_board::sensor_present() (the M2b per-sensor
    // presence infrastructure, which still returns false today), so it is wired with
    // that work, not here.
    // ARM 前ゲート3: センサ健全性 — 繰延。意味あるゲートには sf_board::sensor_present()
    // （M2b の per-sensor presence、現状 false）が要るため、その作業で配線する。

    ESP_LOGI(TAG, "ARM accepted");
    transition(FlightState::ARMED_GROUND);
    return true;
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
    // 挙げるため、ゲートは「ARMED_GROUND 限定」でなく「armed」。モータは isArmed() が
    // false になり次第 ControlTask が 0 にする。
    if (!sf::isArmed(state_)) {
        ESP_LOGD(TAG, "DISARM rejected: not armed (state=%s)",
                 flightStateName(state_));
        return false;
    }

    ESP_LOGI(TAG, "DISARM accepted (%s → IDLE_GROUND)", flightStateName(state_));
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

void StateManager::notifyLandingRequest()
{
    if (state_ != FlightState::FLYING) {
        return;
    }
    ESP_LOGI(TAG, "Landing requested");
    transition(FlightState::LANDING);
}

void StateManager::notifyLandingComplete()
{
    if (state_ != FlightState::LANDING) {
        return;
    }
    ESP_LOGI(TAG, "Landing complete");
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

bool StateManager::requestModeChange(FlightMode new_mode)
{
    if (state_ != FlightState::FLYING) {
        ESP_LOGD(TAG, "Mode change rejected: not FLYING");
        return false;
    }

    if (new_mode == mode_) {
        return true;  // Already in requested mode / 既にリクエストされたモード
    }

    FlightMode old_mode = mode_;
    ESP_LOGI(TAG, "Mode change: %s → %s",
             flightModeName(old_mode), flightModeName(new_mode));

    mode_ = new_mode;

    // Fire mode change callbacks
    // モード変更コールバックを発火
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
    auto severity = static_cast<AlertSeverity>(alert.severity);

    ESP_LOGW(TAG, "Alert received: type=%d severity=%d", alert.type, alert.severity);

    switch (type) {
        case AlertType::IMPACT:
        case AlertType::GYRO_ANOMALY:
            // Crash → immediate DISARM
            // 衝突 → 即時DISARM
            if (isAirborne(state_)) {
                ESP_LOGE(TAG, "Impact/anomaly → emergency IDLE");
                transition(FlightState::IDLE_GROUND);
            }
            break;

        case AlertType::COMM_LOST:
            // Communication lost → keep hovering, then auto-land after the grace period
            // (requirements §9: hover hold 3 s → auto landing). We do NOT land here:
            // arm a timer (idempotent — only the first loss while FLYING arms it) and let
            // update() command FLYING → LANDING once kCommLossHoverUs elapses. The
            // failsafe raises COMM_LOST only on the rising edge, so the deferred landing
            // cannot rely on a repeat alert. The alert timestamp is the loss instant.
            // 通信途絶 → ホバーを維持し、猶予経過後に自動着陸（要件§9: ホバー維持3秒→
            // 自動着陸）。ここでは着陸しない: タイマを起動し（冪等 — FLYING 中の最初の喪失
            // のみ起動）、kCommLossHoverUs 経過で update() が FLYING → LANDING を指令する。
            // failsafe は COMM_LOST を立ち上がりエッジで1回しか出さないので、遅延着陸を
            // アラート再来に頼れない。アラートの timestamp が喪失時刻。
            if (state_ == FlightState::FLYING && !comm_lost_pending_) {
                ESP_LOGW(TAG, "Comm lost → hover %lu ms then LANDING",
                         static_cast<unsigned long>(kCommLossHoverUs / 1000));
                comm_lost_pending_ = true;
                comm_lost_time_us_ = alert.timestamp;
            }
            break;

        case AlertType::LOW_BATTERY:
            // Low battery → warning only (buzzer handled by notification)
            // 低電圧 → 警告のみ（ブザーは通知コンポーネントが処理）
            ESP_LOGW(TAG, "Low battery warning");
            break;

        case AlertType::USB_POWER:
            // USB power → ARM prohibited (checked in requestArm)
            // USB給電 → ARM禁止（requestArmで確認）
            break;

        case AlertType::ESKF_DIVERGED:
            // ESKF diverged → reset estimator (not a state change)
            // ESKF発散 → 推定器リセット（状態変更なし）
            ESP_LOGW(TAG, "ESKF diverged → reset requested");
            break;

        default:
            break;
    }
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
    // Pairing is a ground-only activity: a disarmed vehicle advertising itself so a
    // transmitter can bind. Reject from INIT / armed / airborne (we never re-pair in
    // flight). Idempotent: if already searching, do nothing (avoids re-publish churn).
    // ペアリングは地上限定の活動: disarmed の機体が自分を広告して送信機がバインドできる
    // ようにする。INIT/武装/空中からは拒否（飛行中に再ペアしない）。冪等: 既に探索中なら
    // 何もしない（再発行のばたつき回避）。
    if (state_ != FlightState::IDLE_GROUND && state_ != FlightState::IDLE_HELD) {
        ESP_LOGD(TAG, "Pairing rejected: not on the ground (state=%s)",
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
    // sf_comm が相手にバインドした事実を報告（生のペアリング or 起動時の NVS 復元）。
    // Paired を反映する。冪等 — バインド状態の繰り返し報告は普通に起こる。
    if (pairing_state_ == PairingState::Paired) {
        return;
    }
    ESP_LOGI(TAG, "Pairing complete (bound to transmitter)");
    pairing_state_ = PairingState::Paired;
    publishPairingState();
}

void StateManager::update(uint32_t now_us)
{
    // Comm-loss failsafe: deferred FLYING → LANDING (requirements §9). Nothing pending
    // → nothing to do (the common case).
    // 通信断フェイルセーフ: 遅延 FLYING → LANDING（要件§9）。保留なしなら何もしない（通常）。
    if (!comm_lost_pending_) {
        return;
    }

    // Cancel if we are no longer FLYING: a pilot DISARM, impact-IDLE, or any other
    // transition already took us out of flight, so the auto-land is moot. (Recovery of
    // the link does NOT cancel — the requirement is an unconditional hover-then-land.)
    // FLYING を外れていたらキャンセル: パイロット DISARM・衝撃 IDLE 等で既に飛行を抜けて
    // いれば自動着陸は不要。（リンク復帰ではキャンセルしない — 要件は無条件のホバー→着陸。）
    if (state_ != FlightState::FLYING) {
        comm_lost_pending_ = false;
        return;
    }

    // Grace period elapsed → land. Modular uint32_t subtraction is correct for any age
    // far below the ~71 min wrap, which 3 s is.
    // 猶予経過 → 着陸。uint32_t の剰余差分は ~71分の巻き戻りより遥かに小さい経過（3秒）で正しい。
    if (now_us - comm_lost_time_us_ >= kCommLossHoverUs) {
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

void StateManager::transition(FlightState new_state)
{
    FlightState old_state = state_;

    if (old_state == new_state) {
        return;
    }

    std::printf("  ROOT: TR-1 enter (%d->%d) pre-esplog\n",
                (int)old_state, (int)new_state);
    ESP_LOGI(TAG, "Transition: %s → %s",
             flightStateName(old_state), flightStateName(new_state));
    std::printf("  ROOT: TR-2 post-esplog, exit_cb_cnt=%d\n", exit_callback_count_);

    // Fire onExit callbacks for old state
    // 旧状態のonExitコールバックを発火
    for (int i = 0; i < exit_callback_count_; i++) {
        std::printf("  ROOT: TR-3 exit_cb[%d] pre\n", i);
        if (exit_callbacks_[i]) {
            exit_callbacks_[i](old_state, new_state);
        }
        std::printf("  ROOT: TR-3 exit_cb[%d] post\n", i);
    }

    // Update state
    // 状態を更新
    state_ = new_state;
    std::printf("  ROOT: TR-4 state set, enter_cb_cnt=%d\n", enter_callback_count_);

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
    // ModeChange 経由でのみ知る（ControlTask は system_mode.sub_mode を読まない）ため、ここで
    // onModeChange コールバックを発火する必要がある（mode_ を変えるだけでは不十分）。発火で
    // モード変更が発行され、制御器が STABILIZE へ再構成され古い ALT/POS カスケード＋ホバー推力
    // 状態がクリアされる。発火しないと制御器は再離陸でホバーを保持し続け上昇しない。
    if (new_state == FlightState::IDLE_GROUND && mode_ != FlightMode::STABILIZE) {
        ESP_LOGI(TAG, "Flight mode reset to STABILIZE (on ground)");
        FlightMode old_mode = mode_;
        mode_ = FlightMode::STABILIZE;
        for (int i = 0; i < mode_callback_count_; i++) {
            if (mode_callbacks_[i]) {
                mode_callbacks_[i](old_mode, mode_);
            }
        }
    }

    // Fire onEnter callbacks for new state
    // 新状態のonEnterコールバックを発火
    for (int i = 0; i < enter_callback_count_; i++) {
        std::printf("  ROOT: TR-5 enter_cb[%d] pre\n", i);
        if (enter_callbacks_[i]) {
            enter_callbacks_[i](old_state, new_state);
        }
        std::printf("  ROOT: TR-5 enter_cb[%d] post\n", i);
    }

    // Publish to system.mode topic
    // system.modeトピックに発行
    std::printf("  ROOT: TR-6 pre-publishMode\n");
    publishMode();
    std::printf("  ROOT: TR-7 post-publishMode (transition done)\n");
}

void StateManager::publishMode()
{
    SystemMode mode_msg = {};
    mode_msg.state = static_cast<uint8_t>(state_);
    mode_msg.sub_mode = static_cast<uint8_t>(mode_);
    mode_msg.armed = sf::isArmed(state_);
    mode_msg.timestamp = static_cast<uint32_t>(esp_timer_get_time());

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
