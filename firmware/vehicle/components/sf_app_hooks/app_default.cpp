/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_default.cpp
 * @brief Default sf::app::* implementation, compiled only when no application
 *        directory (SF_APP_DIR) is present. Reproduces vehicle's stock
 *        behavior exactly — this file is what CMakeLists.txt swaps OUT for
 *        the user's own *.cpp.
 *        既定の sf::app::* 実装。アプリディレクトリ（SF_APP_DIR）が無いときのみ
 *        コンパイルされる。vehicle の既定挙動をそのまま再現する — ユーザーの
 *        *.cpp に置き換えられるのがこのファイル（CMakeLists.txt 参照）。
 *
 * controller() and estimator() delegate to sf::app::stock:: (stock_hooks.hpp)
 * — the SAME functions an application's own app.cpp may call when it only
 * wants to override part of the stock behavior (examples/11_app_controller,
 * examples/12_app_task_hello). start() stays a plain no-op here — there is no
 * "stock start task" to delegate to; an application that wants one provides
 * its own in app.cpp.
 *
 * controller() と estimator() は sf::app::stock::（stock_hooks.hpp）へ委譲する
 * — アプリ自身の app.cpp が標準挙動の一部だけ上書きしたいときに呼ぶのと
 * 「同じ」関数（examples/11_app_controller, examples/12_app_task_hello）。
 * start() はここでは単なる no-op のまま — 委譲すべき「標準の開始タスク」は
 * 存在せず、必要なアプリは自分自身の app.cpp で用意する。
 *
 * @design app_hooks.hpp — sf::app::controller/estimator/start contract   [OK]
 * @design stock_hooks.hpp — sf::app::stock::controller/estimator         [OK]
 * @design docs/plans/sf-app-sils-plan.md §2 — Default app hooks           [OK]
 */

#include "app_hooks.hpp"
#include "stock_hooks.hpp"

namespace sf::app {

sf::IController& controller()
{
    return stock::controller();
}

sf::IEstimator& estimator()
{
    return stock::estimator();
}

void start()
{
    // No application present — nothing to start.
    // アプリ無し — 起動するものはない。
}

}  // namespace sf::app
