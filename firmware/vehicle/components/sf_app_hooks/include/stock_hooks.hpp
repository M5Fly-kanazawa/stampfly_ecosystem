/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file stock_hooks.hpp
 * @brief The vehicle's stock (built-in) sf::app::controller()/estimator()
 *        implementation, factored out of app_default.cpp so an application
 *        (SF_APP_DIR) can DELEGATE to it instead of re-implementing the whole
 *        stack — e.g. wrap PidController and override just one axis.
 *        vehicle 標準（組み込み）の sf::app::controller()/estimator() 実装。
 *        app_default.cpp から切り出し、アプリ（SF_APP_DIR）が丸ごと再実装せず
 *        「委譲」できるようにする — 例: PidController を包んで1軸だけ上書き。
 *
 * app_default.cpp (no SF_APP_DIR) forwards its own controller()/estimator()
 * straight to these functions, so the DEFAULT vehicle behavior is unchanged.
 * An application's own app.cpp may call sf::app::stock::controller() /
 * sf::app::stock::estimator() the same way — see examples/11_app_controller
 * and examples/12_app_task_hello.
 *
 * app_default.cpp（SF_APP_DIR 無し）は自身の controller()/estimator() を
 * そのままこれらの関数へ転送するので、vehicle の既定挙動は変わらない。
 * アプリ自身の app.cpp も同じように sf::app::stock::controller() /
 * sf::app::stock::estimator() を呼んでよい — examples/11_app_controller と
 * examples/12_app_task_hello を参照。
 *
 * @design app_hooks.hpp — sf::app::controller/estimator/start contract   [OK]
 * @design docs/plans/sf-app-sils-plan.md §4 Phase 2 — Stock implementation
 *         factored out for template reuse                               [OK]
 */

#pragma once

#include "controller.hpp"
#include "estimator.hpp"

namespace sf::app::stock {

/// The vehicle's standard controller, selected by param controller.type
/// (0 = PID [default], 1 = SNN — Stage 1 wiring skeleton, UNTRAINED),
/// initialized on first call. Same instance/lifetime as the former
/// app_default.cpp body — a function-local static constructed once, init()
/// called exactly once per selected implementation.
/// 標準コントローラ（controller.type で PID[既定]／SNN[Stage1配線骨格・
/// 未学習]を選択）。初回呼び出しで初期化。旧 app_default.cpp と同じ寿命 —
/// 選んだ実装ごとに関数内 static を1回だけ構築し、init() も1回だけ呼ぶ。
sf::IController& controller();

/// The vehicle's standard estimator, selected by param estimator.type
/// (0 = ESKF [default], 1 = complementary filter, 2 = SNN — Stage 1 wiring
/// skeleton, UNTRAINED), initialized on first call.
/// 標準推定器（estimator.type で ESKF[既定]／相補／SNN[Stage1配線骨格・
/// 未学習]を選択）。初回呼び出しで初期化。
sf::IEstimator& estimator();

}  // namespace sf::app::stock
