/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file stock_hooks.cpp
 * @brief sf::app::stock::controller()/estimator() — moved UNCHANGED from
 *        app_default.cpp (which itself moved them unchanged from
 *        control_task.cpp:63's file-scope `static sf::PidController` and
 *        imu_task.cpp's `createEstimator()`). Behavior, log TAG, and log
 *        text are all preserved across both moves.
 *        sf::app::stock::controller()/estimator() — app_default.cpp から
 *        挙動を変えずに移設（app_default.cpp 自身も control_task.cpp:63 の
 *        ファイルスコープ `static sf::PidController` と imu_task.cpp の
 *        `createEstimator()` から挙動を変えず移設したもの）。挙動・ログ
 *        TAG・ログ文言は両方の移設を通じて変わらない。
 *
 * @design stock_hooks.hpp — sf::app::stock::controller/estimator contract [OK]
 * @design docs/plans/sf-app-sils-plan.md §4 Phase 2 — Stock implementation
 *         factored out for template reuse                                [OK]
 */

#include "stock_hooks.hpp"

#include "esp_log.h"

#include "complementary_estimator.hpp"
#include "eskf_estimator.hpp"
#include "params.hpp"
#include "pid_controller.hpp"
#include "snn_controller.hpp"
#include "snn_estimator.hpp"

static const char* TAG = "AppDefault";

namespace sf::app::stock {

sf::IController& controller()
{
    // Function-local static: constructed once (first call), so its lifetime is
    // that of a file-scope `static sf::PidController controller;`. init() is
    // called exactly once, on that first call —
    // the hook contract says ControlTask calls this once, but the guard costs
    // nothing and protects against a future caller that does not honor it.
    // 関数内 static: 初回呼び出しで構築される — ファイルスコープの
    // `static sf::PidController controller;` と同じ寿命。init() は最初の呼び出しで一度だけ実行する — フック契約では
    // ControlTask が1回だけ呼ぶ約束だが、ガードは無害で、契約を守らない
    // 将来の呼び出し元からも保護する。
    //
    // controller.type (0 = PID, 1 = SNN) selects the implementation; the SNN
    // branch is the Stage 1 wiring skeleton (Stroobants et al. 2025 port,
    // arXiv:2411.13945). Each branch uses the same guarded-init pattern
    // (init() once) for its own static.
    // controller.type（0=PID, 1=SNN）で実装を選ぶ。SNN 分岐は Stage1 配線骨格
    // （Stroobants et al. 2025 移植, arXiv:2411.13945）。各分岐は自身の static に
    // 同じガード付き初期化パターン（init() を1回だけ）を使う。
    static sf::PidController pid_controller;
    static bool pid_initialized = false;
    static sf::SnnController snn_controller;
    static bool snn_initialized = false;

    int32_t controller_type = 0;
    sf::params::get_int("controller.type", controller_type);
    if (controller_type == 1) {
        if (!snn_initialized) {
            snn_controller.init();
            snn_initialized = true;
        }
        ESP_LOGI(TAG, "Controller: SNN (CUBA-LIF, UNTRAINED placeholder)");
        return snn_controller;
    }

    if (!pid_initialized) {
        pid_controller.init();
        pid_initialized = true;
    }
    return pid_controller;
}

sf::IEstimator& estimator()
{
    // Estimator factory: select by estimator.type (0 = ESKF, 1 = complementary,
    // 2 = SNN placeholder), construct statically (no heap), initialize, and
    // return via IEstimator (RESET_PLAN P2: algorithm-independence) — this is
    // the only place that names the concrete estimator types; everything else
    // uses IEstimator. The type==2 (SNN) branch is the Stroobants et al. 2025
    // port (arXiv:2411.13945).
    // 推定器ファクトリ: estimator.type（0=ESKF, 1=相補, 2=SNNプレースホルダー）
    // で選び、静的生成・初期化して IEstimator で返す（RESET_PLAN P2: アルゴリズム
    // 非依存）— 具象型の名を持つのはここだけで、他は全て IEstimator を
    // 使う。type==2（SNN）分岐は Stroobants et al. 2025 の移植
    // （arXiv:2411.13945）。
    static sf::EskfEstimator eskf;
    static sf::ComplementaryEstimator comp;
    static sf::SnnEstimator snn;
    int32_t type = 0;
    sf::params::get_int("estimator.type", type);
    if (type == 1) {
        comp.init();
        ESP_LOGI(TAG, "Estimator: complementary filter (attitude + rate)");
        return comp;
    }
    if (type == 2) {
        snn.init();
        ESP_LOGI(TAG, "Estimator: SNN (CUBA-LIF, UNTRAINED placeholder)");
        return snn;
    }
    eskf.init();
    ESP_LOGI(TAG, "Estimator: ESKF (15-state)");
    return eskf;
}

}  // namespace sf::app::stock
