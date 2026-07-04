/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file controller.hpp
 * @brief Control interface definition
 *        制御インターフェース定義
 *
 * Defines the contract that all controllers must satisfy.
 * PID, MPC, LQR, or any custom controller can be used
 * by implementing this interface.
 *
 * 全てのコントローラが満たすべき規約を定義する。
 * PID、MPC、LQR、その他のカスタムコントローラは
 * このインターフェースを実装すれば使用可能。
 *
 * The controller receives state estimates and command setpoints,
 * and outputs thrust/torque in body frame. It does not know about
 * motors, mixers, or FreeRTOS tasks.
 *
 * コントローラは状態推定値とコマンドセットポイントを受け取り、
 * 機体座標系での推力/トルクを出力する。モーター、ミキサー、
 * FreeRTOSタスクを知らない。
 *
 * @design requirements.md §4 — Component #6: replaceable control      [OK]
 * @design requirements.md §10 — Replaceable control                   [OK]
 * @design architecture.md §2 — Control: unified interface             [OK]
 * @design detailed_design.md §4 — IController definition              [OK]
 * @design coding_and_education.md §2 — Bilingual comments             [OK]
 */

#pragma once

#include "data_types.hpp"
#include "flight_state.hpp"

namespace sf {

/// Control interface
/// 制御インターフェース
///
/// All controller implementations (PID, MPC, LQR, etc.) inherit from
/// this class and implement the virtual methods. The control task calls
/// compute() every cycle; the controller does not manage its own timing.
///
/// 全コントローラ実装（PID、MPC、LQR等）はこのクラスを継承し、
/// 仮想メソッドを実装する。制御タスクが毎周期compute()を呼ぶ。
/// コントローラは自身のタイミングを管理しない。
///
/// @design detailed_design.md §4 — compute/reset/onModeChange         [OK]
class IController {
public:
    virtual ~IController() = default;

    // =========================================================================
    // Control Computation
    // 制御演算
    // =========================================================================

    /// Compute control output for one cycle
    /// 1周期分の制御出力を計算する
    ///
    /// Called at 400Hz, synchronized with IMU. The controller reads
    /// the current state and setpoint, and returns thrust/torque.
    ///
    /// 400Hzで呼ばれ、IMUに同期する。コントローラは現在の状態と
    /// セットポイントを読み、推力/トルクを返す。
    ///
    /// @param state    Current state estimate / 現在の推定値
    /// @param setpoint Target setpoint / 目標セットポイント
    /// @param dt       Time step [s] / タイムステップ
    /// @return         Thrust and torque in body frame / 機体座標系での推力とトルク
    virtual ControlOutput compute(
        const StateEstimate& state,
        const CommandSetpoint& setpoint,
        float dt
    ) = 0;

    // =========================================================================
    // Reset
    // リセット
    // =========================================================================

    /// Reset internal state (integrators, filters, history)
    /// 内部状態をリセットする（積分器、フィルタ、履歴）
    ///
    /// Called on ARM, DISARM, and state transitions via
    /// StateManager onExit/onEnter callbacks.
    ///
    /// ARM、DISARM、状態遷移時にStateManagerの
    /// onExit/onEnterコールバック経由で呼ばれる。
    virtual void reset() = 0;

    // =========================================================================
    // Mode Notification
    // モード通知
    // =========================================================================

    /// Notify that the flight mode has changed
    /// フライトモードの変更を通知する
    ///
    /// Called when switching between ACRO/STABILIZE/ALT_HOLD/POS_HOLD.
    /// The controller may reconfigure its internal structure
    /// (e.g., enable/disable cascade loops).
    ///
    /// ACRO/STABILIZE/ALT_HOLD/POS_HOLD間の切替時に呼ばれる。
    /// コントローラは内部構成を再構成できる
    /// （例: カスケードループの有効/無効切替）。
    ///
    /// @param new_mode The new flight mode / 新しいフライトモード
    virtual void onModeChange(FlightMode new_mode) = 0;

    // =========================================================================
    // Autonomous Landing
    // 自動着陸
    // =========================================================================

    /// Enter autonomous landing: level attitude + fixed descent rate, pilot
    /// sticks ignored. Issued via ControllerCmd::Landing when the StateManager
    /// enters LANDING (comm loss, battery emergency). The landing override is
    /// cleared by the next reset() (the ARM of the next flight). Default no-op
    /// so simple teaching controllers are not forced to implement it.
    /// 自動着陸開始: 水平姿勢＋固定降下率でパイロットスティックは無視。StateManager
    /// が LANDING に入ると ControllerCmd::Landing 経由で呼ばれる（通信断・電池緊急）。
    /// 着陸オーバーライドは次の reset()（次飛行の ARM）で解除される。教育用の単純な
    /// 制御器に実装を強制しないよう既定は no-op。
    virtual void onLanding() {}

    /// Auto-takeoff start (ControllerCmd::Takeoff, TAKEOFF entry in ALT/POS mode):
    /// climb at a fixed rate with level attitude until onTakeoffComplete().
    /// Default no-op for controllers without an altitude loop.
    /// 自動離陸開始（ControllerCmd::Takeoff, ALT/POS モードの TAKEOFF 突入）:
    /// onTakeoffComplete() まで固定上昇率＋水平姿勢で上昇する。
    /// 高度ループを持たない制御器では既定 no-op。
    virtual void onTakeoff() {}

    /// Takeoff finished (ControllerCmd::TakeoffComplete, FLYING entry): engage the
    /// normal mode law. Default no-op.
    /// 離陸完了（ControllerCmd::TakeoffComplete, FLYING 突入）: 通常のモード則を係合。
    /// 既定 no-op。
    virtual void onTakeoffComplete() {}

    /// Whether the ALT/POS auto-takeoff has captured its target altitude. The task
    /// layer publishes this on controller_status; the state machine uses it to drive
    /// TAKEOFF → FLYING for auto-takeoff (decoupled from the ToF 0.15m airborne edge,
    /// which is the ESKF vertical handoff only — 2026-06-14 redesign). Default false
    /// (controllers without an altitude loop fall back to ToF airborne detection).
    /// ALT/POS 自動離陸が目標高度を捕捉したか。タスク層が controller_status で publish し、
    /// 状態機械が自動離陸の TAKEOFF→FLYING を駆動する（ToF 0.15m 空中エッジ＝ESKF 鉛直
    /// ハンドオフ専用とは分離 — 2026-06-14 再設計）。既定 false（高度ループを持たない制御器は
    /// ToF 空中検知にフォールバック）。
    virtual bool isTakeoffComplete() const { return false; }

    /// Guidance/API position+yaw target (R11). Only meaningful in POS_HOLD —
    /// implementations may ignore it in other modes. Pilot stick MOVEMENT
    /// cancels guidance (the pilot always wins). Default no-op.
    /// Guidance/API の位置+yaw 目標（R11）。POS_HOLD でのみ意味を持ち、他モードでは
    /// 無視してよい。スティックを「動かす」と誘導は解除される（パイロット優先）。
    /// 既定 no-op。
    virtual void setGuidanceTarget(const GuidanceTarget& target,
                                   const CommandSetpoint& current_sticks) {}

    /// Whether a guidance target is currently engaged. Goes false when the
    /// controller self-cancels guidance (pilot stick movement or a mode change).
    /// The task layer publishes this on controller_status so the API source can
    /// release its target on the falling edge — keeping "the pilot always wins"
    /// observable to the API (M-3). Default false (controllers without guidance).
    /// 誘導目標が現在係合中か。制御器がスティック動作やモード変更で誘導を自発解除すると
    /// false。タスク層がこれを controller_status で publish し、API ソースが立下りで自分の
    /// 目標を解放できる — 「パイロット優先」を API から観測可能にする (M-3)。既定 false。
    virtual bool isGuidanceActive() const { return false; }

    /// Rate-loop sysid excitation (API `sysid`): add a chirp/doublet to one
    /// axis' rate setpoint while flying. Default no-op.
    /// レートループ同定励振（API `sysid`）: 飛行中に1軸のレート目標へチャープ/
    /// ダブレットを加算する。既定 no-op。
    virtual void startExcitation(const SysidCommand& cmd) {}

    /// One-shot fetch of a completed stepped-sine point (see startExcitation).
    /// The task layer publishes it. Default: none.
    /// 完了ステップドサイン点の一回限り取得。publish はタスク層。既定: なし。
    virtual bool fetchSysidResult(SysidFreqResult& out) { return false; }

    // =========================================================================
    // Live Parameter Reload
    // ライブパラメータ再読込
    // =========================================================================

    /// Re-read the controller's gains/limits from the parameter system, keeping
    /// the integrator state — live tuning. Issued via ControllerCmd::ReloadParams
    /// when a parameter changes (param set callback). Default no-op.
    /// 制御器のゲイン/リミットをパラメータシステムから読み直す。積分器状態は維持 —
    /// ライブチューニング。パラメータ変更時（param set コールバック）に
    /// ControllerCmd::ReloadParams 経由で実行。既定 no-op。
    virtual void reloadParams() {}
};

}  // namespace sf
