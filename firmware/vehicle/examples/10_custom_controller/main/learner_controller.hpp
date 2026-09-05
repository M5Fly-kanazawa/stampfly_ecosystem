/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file learner_controller.hpp
 * @brief Tier L1 exercise — a thin `IController` wrapper that shows exactly
 *        where a custom control law goes, by delegating everything else to
 *        the existing `PidController`.
 *        Tier L1 演習 — 既存の `PidController` に他を全て委譲することで、
 *        独自の制御則をどこに書けばよいかを示す薄い `IController` ラッパー。
 *
 * WHAT THIS CLASS IS FOR
 * このクラスの目的
 * ------------------------------------------------------------------
 * `IController` (controller.hpp) has 12 methods; `PidController`
 * (sf_controller_pid) implements all of them with real cascade PID control.
 * A learner who wants to try "what if compute() did one thing differently"
 * does NOT need to reimplement a cascade controller from scratch — they can
 * wrap the existing one and override just the one method (or one line inside
 * it) they are experimenting with. `LearnerController` forwards every method
 * unchanged EXCEPT `compute()`, where a single named constant
 * (`config::kYawTorqueScale`) is applied — see the "Try changing!" section in
 * compute()'s implementation for the exact insertion point.
 *
 * `IController`（controller.hpp）は12個のメソッドを持ち、`PidController`
 * （sf_controller_pid）が全てを実際のカスケードPID制御で実装している。
 * 「compute() の一部だけ変えたらどうなるか」を試したい学習者は、カスケード制御器を
 * ゼロから再実装する必要はない — 既存のものをラップし、実験したい1メソッド
 * （またはその中の1行）だけ上書きすればよい。`LearnerController` は `compute()`
 * を除く全メソッドをそのまま転送する — `compute()` では名前付き定数1つ
 * （`config::kYawTorqueScale`）を適用する。挿入箇所の正確な位置は compute() 実装内の
 * 「ここを変えてみよう！」節を参照。
 *
 * HOW TO ACTUALLY FLY WITH THIS (this example does NOT do it)
 * これで実際に飛ばす方法（本サンプルはそこまで行わない）
 * ------------------------------------------------------------------
 * `IController` has no runtime factory — the real firmware picks its
 * controller at COMPILE TIME with one static instance
 * (`tasks/control_task.cpp:63`: `static sf::PidController controller;`).
 * There is no component-level "plug in" point a standalone example project
 * can REQUIRE into; registering this class for real flight means editing
 * that one line in the real firmware and rebuilding `vehicle` itself. See
 * README.md, section "実機で飛ばすレシピ / Recipe to actually fly this" for
 * the exact 2-line diff — main.cpp here instead runs `LearnerController`
 * standalone against synthetic inputs, so you can see it compute real
 * `ControlOutput` values without touching the real firmware or hardware.
 *
 * `IController` に実行時ファクトリは無い — 実ファームは1個の static インスタンス
 * （`tasks/control_task.cpp:63`: `static sf::PidController controller;`）で
 * コンパイル時に制御器を選ぶ。単体サンプルプロジェクトから REQUIRES できる
 * コンポーネント単位の「差し込み口」は存在しない。実飛行に登録するには実ファーム
 * のその1行を編集して `vehicle` 自体を再ビルドする必要がある。正確な2行の差分は
 * README.md「実機で飛ばすレシピ / Recipe to actually fly this」を参照 — 代わりに
 * ここの main.cpp は `LearnerController` を合成入力に対してスタンドアロンで動かし、
 * 実ファームや実機に触れずに本物の `ControlOutput` 値が計算される様子を見せる。
 *
 * @design architecture.md §2.5 — L1: IEstimator/IController を実装して差替え [OK]
 * @design controller.hpp — IController interface (12 methods)              [OK]
 * @design coding_and_education.md §3 — 21_custom_controller 計画            [OK]
 */

#pragma once

#include "controller.hpp"
#include "pid_controller.hpp"

namespace sf {

/// Thin `IController` wrapper around `PidController`. Forwards every method
/// unchanged except `compute()`, which is the one exercise hook.
/// `PidController` を包む薄い `IController` ラッパー。演習用フックである
/// `compute()` を除き、全メソッドをそのまま転送する。
class LearnerController : public IController {
public:
    /// Initialize the wrapped PidController (loads gains, resets state).
    /// 内側の PidController を初期化する（ゲイン読込・状態リセット）。
    void init();

    /// The one exercise hook — see the .cpp file for the insertion point.
    /// 唯一の演習フック — 挿入箇所は .cpp ファイルを参照。
    ControlOutput compute(const StateEstimate& state, const CommandSetpoint& setpoint,
                           float dt) override;

    void reset() override;
    void onModeChange(FlightMode new_mode) override;
    void onLanding() override;
    void onTakeoff() override;
    void onTakeoffComplete() override;
    bool isTakeoffComplete() const override;
    void setGuidanceTarget(const GuidanceTarget& target,
                            const CommandSetpoint& current_sticks) override;
    bool isGuidanceActive() const override;
    void startExcitation(const SysidCommand& cmd) override;
    bool fetchSysidResult(SysidFreqResult& out) override;
    void reloadParams() override;

private:
    /// The controller being wrapped. A real learner exercise might replace
    /// this with their own cascade, or keep it and only touch compute().
    /// ラップ対象のコントローラ。実際の演習ではこれを自作のカスケードに
    /// 置き換えてもよいし、そのまま使って compute() だけ触ってもよい。
    PidController inner_controller_;
};

}  // namespace sf
