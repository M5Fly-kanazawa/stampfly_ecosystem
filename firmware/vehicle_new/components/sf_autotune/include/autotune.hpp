/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file autotune.hpp
 * @brief Onboard rate-loop autotune — plant fit + phase-margin PID design
 *        オンボードのレートループ自動チューニング — プラントフィット＋位相余裕 PID 設計
 *
 * Pure math, no I/O, host-testable (mirrors tools/log_analyzer/rate_sysid.py,
 * which validated the algorithms against a synthetic known plant):
 *  - fitPlant(): fit G(s) = b·e^{-Ls} / (s(Ts+1)) to measured frequency points
 *    G(jωk) = Y(jωk)/U(jωk) (stepped-sine I/Q accumulation done by the
 *    controller) with a Nelder-Mead on a log-magnitude + wrapped-phase cost.
 *  - tunePid(): solve the firmware's exact PID form C(s) = Kp(1 + 1/(Ti s) +
 *    Td s/(η Td s + 1)), η = 0.125, for |C·G(jωc)| = 1 and the phase-margin
 *    condition; verify the resulting margins numerically.
 *
 * 純粋な数学のみ・I/O なし・ホストテスト可能（rate_sysid.py と同一アルゴリズム。
 * Python 側で既知プラント合成により検証済み）:
 *  - fitPlant(): 実測周波数点 G(jωk)=Y/U（ステップドサインの I/Q 蓄積は制御器が
 *    実施）に G(s)=b·e^{-Ls}/(s(Ts+1)) を Nelder-Mead でフィット。
 *  - tunePid(): ファーム PID 形そのものを |C·G(jωc)|=1 と位相余裕条件で解き、
 *    得られた余裕を数値検証する。
 *
 * @design requirements.md §6 — 制御の調整容易性                        [OK]
 * @design coding_and_education.md §2 — Bilingual comments              [OK]
 */

#pragma once

#include <cstdint>

namespace sf::autotune {

/// One measured open-loop frequency point (from the controller's stepped-sine
/// I/Q accumulation): G(jw) = (yr + j·yi) / (ur + j·ui).
/// 実測周波数点1つ（制御器のステップドサイン I/Q 蓄積から）。
struct FreqPoint {
    float w;     // [rad/s]
    float ur, ui;   // U(jw) I/Q sums  / U の I/Q 和
    float yr, yi;   // Y(jw) I/Q sums  / Y の I/Q 和
};

/// Fitted plant G(s) = b·e^{-Ls} / (s(Ts+1)) / フィット済みプラント
struct Plant {
    float b;         // [1/(kg m²)] effective inverse inertia / 有効慣性逆数
    float T;         // [s] motor/prop lag                    / モータ遅れ
    float L;         // [s] dead time                          / むだ時間
    float residual;  // mean fit cost per point (quality)      / 点あたり残差
};

/// Tuned PID + numerically verified margins / 設計 PID＋数値検証済み余裕
struct TuneResult {
    float kp, ti, td;
    float wc;        // achieved gain crossover [rad/s] / 達成ゲイン交差
    float pm_deg;    // achieved phase margin [deg]     / 達成位相余裕
    float gm_db;     // gain margin [dB] (valid only if gm_valid) / ゲイン余裕
    // True when a real −180° phase crossing was found and gm_db is a finite,
    // meaningful gain margin. False means the open loop never reaches −180° in
    // the sweep (gm_db is NOT 0 dB — the margin is effectively infinite/safe),
    // so a GM lower-bound gate must PASS this case rather than reject it (L-15).
    // −180° 位相交差が見つかり gm_db が有限で意味のあるゲイン余裕のとき true。
    // false は掃引中に開ループが −180° に達しないこと（gm_db は 0dB ではなく余裕は
    // 実質無限大＝安全）を意味し、GM 下限ゲートはこのケースを棄却でなく PASS させる (L-15)。
    bool  gm_valid;
};

/// Fit the plant to K measured points. b0 seeds the search (1/J from specs).
/// Returns false when the data is unusable (degenerate U, fit divergence).
/// K 個の実測点にプラントをフィット。b0 は探索の種（仕様の 1/J）。データ不良
/// （U が退化・発散）では false。
bool fitPlant(const FreqPoint* points, int count, float b0, Plant& out);

/// Solve Kp/Ti/Td for the spec (wc [rad/s], pm [deg]); Ti = ti_factor/wc.
/// Returns false when the required phase lead exceeds the PID's maximum.
/// 仕様（ωc, PM）に対する Kp/Ti/Td を解く。必要リードが PID の上限を超えるなら false。
bool tunePid(const Plant& plant, float wc, float pm_deg, float ti_factor,
             TuneResult& out);

/// Evaluate the open-loop margins of a GIVEN PID (kp,ti,td) against a plant by an
/// ω-sweep of L = kp·C(jω)·G(jω): fills out.{kp,ti,td,wc,pm_deg,gm_db,gm_valid}.
/// Returns true if a gain crossover (|L|=1) was found. Same routine tunePid uses
/// to verify its design — exposed so a caller can also score the CURRENT (e.g.
/// rejected/unchanged) gains against the freshly identified plant.
/// 与えられた PID(kp,ti,td)の開ループ余裕をプラントに対し ω 掃引で評価する。tunePid が
/// 設計検証に使うのと同じ計算 — 現（棄却/据置）ゲインを新同定プラントで採点するため公開。
bool evalMargins(const Plant& plant, float kp, float ti, float td, TuneResult& out);

}  // namespace sf::autotune
