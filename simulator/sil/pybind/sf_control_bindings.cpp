/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sf_control_bindings.cpp
 * @brief pybind11 module exposing the REAL firmware control-law structs to
 *        Python, so Python code drives the actual C++ implementation instead
 *        of a hand-ported re-implementation.
 *        firmware の制御則構造体を pybind11 で Python に公開する。Python 側が
 *        手作業移植の再実装ではなく、本物の C++ 実装を直接駆動できるようにする。
 *
 * Why this exists / 存在理由:
 * tools/log_analyzer/rate_sysid.py's replay_pid() is a hand-ported line-for-
 * line copy of firmware/vehicle/components/sf_controller_pid/include/pid.hpp,
 * kept in sync by hand (a structural sync-drift risk — nothing enforces the
 * two stay identical as pid.hpp evolves). This module compiles pid.hpp
 * UNMODIFIED and binds it directly, so a Python lockstep test
 * (simulator/tests/test_pid_lockstep.py) can drive the real firmware struct
 * step-by-step and numerically prove (or disprove) that replay_pid() still
 * matches it — instead of trusting a second hand-translation.
 * tools/log_analyzer/rate_sysid.py の replay_pid() は
 * firmware/vehicle/components/sf_controller_pid/include/pid.hpp を逐語手動
 * 移植したもので、「手で同期を保つ」運用（pid.hpp が変わっても両者が同一である
 * 保証が構造的にない = 同期ドリフトのリスク）になっている。本モジュールは
 * pid.hpp を無改変でコンパイル・直接バインドし、Python の lockstep テスト
 * （simulator/tests/test_pid_lockstep.py）が本物のファーム構造体を1ステップずつ
 * 駆動して、replay_pid() が今も一致するかを数値的に証明（または反証）できる
 * ようにする — 二重の手動移植を信頼する代わりに。
 *
 * Binding scope / バインド範囲:
 * `sf::PID` only (firmware/.../include/pid.hpp). This header is genuinely
 * self-contained (no #include at all — verified), so no other firmware
 * headers need to be pulled in or stubbed to compile it on the host.
 * `sf::PID` のみ（pid.hpp）。このヘッダは本当に自己完結（#include が一つも
 * 無いことを確認済み）なので、host でコンパイルするのに他の firmware ヘッダの
 * 取り込み・スタブ化は一切不要。
 */

#include <pybind11/pybind11.h>

// The real firmware header, compiled UNMODIFIED. It is header-only and has
// zero #includes of its own — pure arithmetic, trivial to bind, no ESP-IDF/
// FreeRTOS surface to stub out.
// 本物のファームヘッダを無改変でコンパイルする。ヘッダオンリーで #include が
// 一切無い（純粋な算術のみ）ため、ESP-IDF/FreeRTOS 依存のスタブ化が不要。
#include "pid.hpp"

namespace py = pybind11;

// Module name `stampfly_control`: a namespace for future firmware
// control-law bindings beyond PID (see the autotune.cpp mirror mentioned in
// the module docstring above — out of scope for this module, but the name is
// chosen so it does not need to be "pid"-specific if more structs join later.
// モジュール名 `stampfly_control`: PID 以外の将来の制御則バインディングも
// 収容できるよう、"pid" 専用ではない名前にしている（autotune.cpp の第三の
// 鏡写しは今回のスコープ外だが、将来ここに合流する余地を残す）。
PYBIND11_MODULE(stampfly_control, m) {
    m.doc() =
        "Real firmware control-law structs, compiled unmodified and exposed "
        "via pybind11 (see firmware/vehicle/components/sf_controller_pid/"
        "include/pid.hpp).\n"
        "無改変コンパイルした本物のファーム制御則構造体を pybind11 で公開する。";

    // sf::PID — single-axis PID with incomplete derivative (Tustin), D-on-M,
    // conditional-integration anti-windup. Every public field is exposed
    // read-write (including internal state: integral/deriv_filter/
    // prev_error/prev_measurement/first_run) so a Python test can both drive
    // compute() step-by-step AND inspect/seed internal state directly for
    // boundary-condition checks.
    // sf::PID — 不完全微分（Tustin）付き1軸PID、D-on-M、条件付き積分の
    // アンチワインドアップ。全公開フィールドを読み書き可能に公開する
    // （内部状態 integral/deriv_filter/prev_error/prev_measurement/first_run
    // を含む）。Python テストが compute() を1ステップずつ駆動しつつ、境界条件
    // 確認のために内部状態を直接観測・注入できるようにするため。
    py::class_<sf::PID>(m, "PID")
        .def(py::init<>(), "Default-construct with pid.hpp's in-class "
                            "defaults (kp=0, ti=1.0, td=0, eta=0.125, "
                            "output_limit=1.0).")
        // Gains / ゲイン
        .def_readwrite("kp", &sf::PID::kp, "Proportional gain / 比例ゲイン")
        .def_readwrite("ti", &sf::PID::ti, "Integral time [s] / 積分時間")
        .def_readwrite("td", &sf::PID::td, "Derivative time [s] / 微分時間")
        .def_readwrite("eta", &sf::PID::eta,
                        "Derivative filter coefficient / 微分フィルタ係数")
        // Output limit / 出力制限
        .def_readwrite("output_limit", &sf::PID::output_limit,
                        "Symmetric output clamp [+-] / 対称出力制限")
        // State (exposed for boundary-condition inspection / seeding)
        // 状態（境界条件の観測・注入用に公開）
        .def_readwrite("integral", &sf::PID::integral,
                        "Integral accumulator / 積分蓄積値")
        .def_readwrite("deriv_filter", &sf::PID::deriv_filter,
                        "Derivative filter state / 微分フィルタ状態")
        .def_readwrite("prev_error", &sf::PID::prev_error,
                        "Previous error (integral trapezoid) / "
                        "前回誤差（積分台形則用）")
        .def_readwrite("prev_measurement", &sf::PID::prev_measurement,
                        "Previous measurement (D-on-M) / "
                        "前回測定値（測定値微分用）")
        .def_readwrite("first_run", &sf::PID::first_run,
                        "Primes the D input after reset / "
                        "リセット後の微分入力初期化フラグ")
        // Methods / メソッド
        .def("compute", &sf::PID::compute, py::arg("setpoint"),
             py::arg("measurement"), py::arg("dt"),
             "Compute one PID step (mirrors pid.hpp compute() exactly, "
             "byte-for-byte the same code the firmware ships).\n"
             "PID を1ステップ計算する（pid.hpp の compute() そのもの、"
             "ファーム出荷コードと完全に同一）。")
        .def("reset", &sf::PID::reset,
             "Reset all internal state (integral/deriv_filter/prev_error/"
             "prev_measurement/first_run).\n"
             "全内部状態をリセットする。");
}
