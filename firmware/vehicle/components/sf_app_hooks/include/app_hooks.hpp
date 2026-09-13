/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file app_hooks.hpp
 * @brief L1 application entry point (Topic API) — the seam where the user's own
 *        controller / estimator / tasks are wired into the vehicle body
 *        L1（Topic API）アプリの入口 — ユーザー自身のコントローラ／推定器／
 *        タスクを vehicle 本体に組み込む境界
 *
 * This is the ONE extension point vehicle offers at Tier L1 (architecture.md
 * §2.5): a user who has implemented their own IController / IEstimator, or who
 * wants to run an additional task that reads Topics (sf::api::*), provides
 * these three functions instead of the defaults. ControlTask and ImuTask call
 * controller()/estimator() once each at task start; main.cpp calls start() once
 * after every standard task is running (hardware_init.md §4 Phase 4).
 *
 * With no user application present, app_default.cpp (this component's own
 * SRCS) supplies the default implementation — PidController / the
 * estimator.type factory / a no-op start() — so vehicle's stock behavior is
 * unchanged. When the build sets SF_APP_DIR (main/CMakeLists.txt,
 * simulator/sils/CMakeLists.txt), the user's *.cpp replace app_default.cpp and
 * compile straight into the SAME binary (main component / emu_vehicle), so the
 * identical source runs on real hardware and in SILS (Code Identity,
 * development_roadmap.md §1).
 *
 * vehicle が Tier L1（architecture.md §2.5）で提供する唯一の拡張点。自作の
 * IController / IEstimator を実装した、あるいは Topic（sf::api::*）を読む
 * 追加タスクを実行したいユーザーは、既定の代わりにこの 3 関数を提供する。
 * ControlTask と ImuTask はタスク開始時に controller()/estimator() を 1 回ずつ
 * 呼び、main.cpp は全標準タスク起動後（hardware_init.md §4 Phase 4）に
 * start() を 1 回呼ぶ。
 *
 * ユーザーアプリが無ければ、本コンポーネント自身の SRCS である app_default.cpp
 * が既定実装（PidController／estimator.type ファクトリ／no-op start()）を
 * 供給し、vehicle の既定挙動は変わらない。ビルドが SF_APP_DIR を指定すると
 * （main/CMakeLists.txt、simulator/sils/CMakeLists.txt）ユーザーの *.cpp が
 * app_default.cpp を置き換えて同一バイナリ（main コンポーネント／emu_vehicle）
 * へ直接コンパイルされ、実機と SILS で同一ソースが動く（Code Identity,
 * development_roadmap.md §1）。
 *
 * @design architecture.md §2.5 — L1 entry: app hooks                      [OK]
 * @design coding_and_education.md §2 — Namespace 規約（L1 = sf::api::）    [OK]
 * @design docs/plans/sf-app-sils-plan.md §2 — App hooks / SF_APP_DIR       [OK]
 */

#pragma once

#include "controller.hpp"
#include "estimator.hpp"

namespace sf::app {

/// Return the controller ControlTask will use. Called once at ControlTask
/// start, after parameters are loaded (boot Phase 3). Must return an
/// initialized instance — the caller does not call init() again.
/// ControlTask が使うコントローラを返す。ControlTask 起動時（パラメータ読込後、
/// Phase 3 の後）に 1 回呼ばれる。初期化済みのインスタンスを返すこと —
/// 呼び出し側は init() を重ねて呼ばない。
sf::IController& controller();

/// Return the estimator ImuTask will use. Called once at ImuTask start. Must
/// return an initialized instance — the caller does not call init() again.
/// ImuTask が使う推定器を返す。ImuTask 起動時に 1 回呼ばれる。初期化済みの
/// インスタンスを返すこと — 呼び出し側は init() を重ねて呼ばない。
sf::IEstimator& estimator();

/// Called once at the end of boot, after every standard task is running
/// (hardware_init.md §4 Phase 4). Start any additional application tasks
/// here (e.g. one that reads sf::api::*_latest() on a timer). Default: does
/// nothing.
/// 起動完了時（全標準タスク起動後、hardware_init.md §4 Phase 4）に 1 回呼ばれる。
/// アプリ固有の追加タスク（例: sf::api::*_latest() を定周期で読むタスク）は
/// ここで起動する。既定は何もしない。
void start();

}  // namespace sf::app
