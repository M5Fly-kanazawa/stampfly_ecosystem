/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_controller.hpp
 * @brief Tier L1 template — an `IController` that delegates every method to
 *        `PidController`, except one insertion point (`adjust()`) where you
 *        write your own control law for a single axis.
 *        Tier L1 テンプレート — 全メソッドを `PidController` に委譲する
 *        `IController`。ただし1箇所（`adjust()`）だけ、1軸の独自制御則を
 *        書ける挿入点がある。
 *
 * This is the embedded (SF_APP_DIR) counterpart of examples/10_custom_controller's
 * LearnerController: same idea (wrap PidController, override one thing), but
 * this one compiles straight into the vehicle body (`sf app build` / `sf app
 * sils`) instead of running standalone against synthetic input — see the
 * README for the exact commands.
 *
 * examples/10_custom_controller の LearnerController の「組み込み版」
 * （SF_APP_DIR）。考え方は同じ（PidController を包んで1点だけ上書き）だが、
 * こちらは合成入力に対する単独実行ではなく vehicle 本体にそのまま組み込まれて
 * コンパイルされる（`sf app build` / `sf app sils`）— 正確なコマンドは
 * README を参照。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design docs/plans/sf-app-sils-plan.md §4 Phase 2 — 11_app_controller     [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"

namespace sf::app {

/// Thin `IController` wrapper around `PidController`. Forwards every method
/// unchanged except `compute()`, which routes its output through `adjust()`
/// — the one exercise hook.
/// `PidController` を包む薄い `IController` ラッパー。`compute()` を除く
/// 全メソッドをそのまま転送する。`compute()` の出力だけ `adjust()`
/// （唯一の挿入点）に通す。
class AppController : public sf::IController {
public:
    /// Initialize the wrapped PidController (loads gains, resets state).
    /// 内側の PidController を初期化する（ゲイン読込・状態リセット）。
    void init();

    /// Compute via PidController, then pass the result through adjust().
    /// PidController で計算し、その結果を adjust() に通す。
    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;
    void onLanding() override;
    void onTakeoff() override;
    void onTakeoffComplete() override;
    bool isTakeoffComplete() const override;
    void setGuidanceTarget(const sf::GuidanceTarget& target,
                           const sf::CommandSetpoint& current_sticks) override;
    bool isGuidanceActive() const override;
    void startExcitation(const sf::SysidCommand& cmd) override;
    bool fetchSysidResult(sf::SysidFreqResult& out) override;
    void reloadParams() override;

private:
    /// Your one insertion point: adjust the cascade's ControlOutput before it
    /// reaches the mixer. Default is the identity (kPitchTorqueScale = 1.0) —
    /// see app_controller.cpp for where to write your own control law.
    /// 唯一の挿入点: ミキサーへ渡す前にカスケードの ControlOutput を調整する。
    /// 既定は恒等（kPitchTorqueScale = 1.0）— 独自の制御則の書き場所は
    /// app_controller.cpp を参照。
    sf::ControlOutput adjust(
        sf::ControlOutput output,
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint);

    /// The controller being wrapped. Replace it with your own cascade, or
    /// keep it and only touch adjust().
    /// ラップ対象のコントローラ。自作のカスケードに置き換えてもよいし、
    /// そのまま使って adjust() だけ触ってもよい。
    sf::PidController pid_;
};

}  // namespace sf::app
