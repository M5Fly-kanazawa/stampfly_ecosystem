/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file autotune.cpp
 * @brief Onboard rate-loop autotune implementation (see autotune.hpp)
 *        オンボード自動チューニング実装（autotune.hpp 参照）
 *
 * Mirrors tools/log_analyzer/rate_sysid.py — keep the two in sync.
 * rate_sysid.py と同一アルゴリズム — 双方を同期して保守すること。
 */

#include "autotune.hpp"
#include <cmath>

namespace sf::autotune {

namespace {

constexpr float kEta = 0.125f;     // firmware incomplete-derivative coefficient
constexpr float kPi  = 3.14159265f;

/// Complex value / 複素数（軽量、<complex> 依存を避ける）
struct Cplx {
    float re, im;
    float mag() const { return sqrtf(re * re + im * im); }
    float arg() const { return atan2f(im, re); }
};

Cplx cmul(Cplx a, Cplx b) { return {a.re * b.re - a.im * b.im,
                                    a.re * b.im + a.im * b.re}; }
Cplx cdiv(Cplx a, Cplx b)
{
    const float d = b.re * b.re + b.im * b.im;
    return {(a.re * b.re + a.im * b.im) / d, (a.im * b.re - a.re * b.im) / d};
}

/// G(jw) = b e^{-jwL} / (jw (jwT + 1)) / プラント周波数応答
Cplx plantResponse(float w, float b, float T, float L)
{
    const Cplx num = {b * cosf(-w * L), b * sinf(-w * L)};
    const Cplx den = cmul({0, w}, {1.0f, w * T});
    return cdiv(num, den);
}

/// C(jw)/Kp of the firmware PID / ファーム PID の単位ゲイン周波数応答
Cplx pidUnit(float w, float ti, float td)
{
    Cplx c = {1.0f, -1.0f / (w * ti)};
    if (td > 0.0f) {
        const Cplx lead = cdiv({0, w * td}, {1.0f, w * kEta * td});
        c.re += lead.re;
        c.im += lead.im;
    }
    return c;
}

/// Wrap an angle to (-π, π] / 角度を (-π, π] へ折返す
float wrapAngle(float a)
{
    while (a > kPi)  a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

/// Fit cost: Σ (Δlog|G|)² + (wrapped Δarg G)² over the points.
/// フィットコスト: 対数振幅差²＋折返し位相差² の総和。
float fitCost(const float p[3], const FreqPoint* pts, const Cplx* G_hat, int n)
{
    const float b = p[0], T = p[1], L = p[2];
    if (b <= 0.0f || T <= 1e-4f || L < 0.0f || L > 0.05f) {
        return 1e12f;
    }
    float cost = 0.0f;
    for (int i = 0; i < n; i++) {
        const Cplx gm = plantResponse(pts[i].w, b, T, L);
        const float dmag = logf(G_hat[i].mag() + 1e-12f) - logf(gm.mag() + 1e-12f);
        const float dph  = wrapAngle(G_hat[i].arg() - gm.arg());
        cost += dmag * dmag + dph * dph;
    }
    return cost;
}

}  // namespace

// =============================================================================
// fitPlant — Nelder-Mead over (b, T, L), seeded from the spec inertia.
// fitPlant — (b,T,L) の Nelder-Mead。仕様慣性を種にする。
// =============================================================================
bool fitPlant(const FreqPoint* points, int count, float b0, Plant& out)
{
    if (count < 4) {
        return false;
    }

    // Measured G(jw) per point; reject degenerate excitation (|U| ~ 0).
    // 点ごとの実測 G(jw)。励振が退化（|U|≈0）した点は不採用。
    Cplx G_hat[16];
    FreqPoint pts[16];
    int n = 0;
    for (int i = 0; i < count && n < 16; i++) {
        const Cplx U = {points[i].ur, points[i].ui};
        const Cplx Y = {points[i].yr, points[i].yi};
        if (U.mag() < 1e-9f) {
            continue;
        }
        G_hat[n] = cdiv(Y, U);
        pts[n] = points[i];
        n++;
    }
    if (n < 4) {
        return false;
    }

    // Nelder-Mead (3 params — same shape as the host tool's).
    // Nelder-Mead（3 パラメータ — ホストツールと同形）。
    float simplex[4][3] = {
        {b0,        0.020f, 0.005f},
        {b0 * 1.5f, 0.020f, 0.005f},
        {b0,        0.040f, 0.005f},
        {b0,        0.020f, 0.009f},
    };
    float vals[4];
    for (int i = 0; i < 4; i++) {
        vals[i] = fitCost(simplex[i], pts, G_hat, n);
    }

    for (int iter = 0; iter < 300; iter++) {
        // Order the simplex (insertion sort, 4 entries) / 単体を整列
        for (int i = 1; i < 4; i++) {
            for (int j = i; j > 0 && vals[j] < vals[j - 1]; j--) {
                float tv = vals[j]; vals[j] = vals[j - 1]; vals[j - 1] = tv;
                for (int k = 0; k < 3; k++) {
                    float tp = simplex[j][k];
                    simplex[j][k] = simplex[j - 1][k];
                    simplex[j - 1][k] = tp;
                }
            }
        }
        float centroid[3] = {0, 0, 0};
        for (int i = 0; i < 3; i++) {
            for (int k = 0; k < 3; k++) {
                centroid[k] += simplex[i][k] / 3.0f;
            }
        }
        float xr[3], xe[3], xc[3];
        for (int k = 0; k < 3; k++) {
            xr[k] = centroid[k] + (centroid[k] - simplex[3][k]);
        }
        const float fr = fitCost(xr, pts, G_hat, n);
        if (fr < vals[0]) {
            for (int k = 0; k < 3; k++) {
                xe[k] = centroid[k] + 2.0f * (centroid[k] - simplex[3][k]);
            }
            const float fe = fitCost(xe, pts, G_hat, n);
            const float* src = (fe < fr) ? xe : xr;
            for (int k = 0; k < 3; k++) simplex[3][k] = src[k];
            vals[3] = (fe < fr) ? fe : fr;
        } else if (fr < vals[2]) {
            for (int k = 0; k < 3; k++) simplex[3][k] = xr[k];
            vals[3] = fr;
        } else {
            for (int k = 0; k < 3; k++) {
                xc[k] = centroid[k] + 0.5f * (simplex[3][k] - centroid[k]);
            }
            const float fc = fitCost(xc, pts, G_hat, n);
            if (fc < vals[3]) {
                for (int k = 0; k < 3; k++) simplex[3][k] = xc[k];
                vals[3] = fc;
            } else {
                for (int i = 1; i < 4; i++) {       // shrink / 縮小
                    for (int k = 0; k < 3; k++) {
                        simplex[i][k] = simplex[0][k]
                                      + 0.5f * (simplex[i][k] - simplex[0][k]);
                    }
                    vals[i] = fitCost(simplex[i], pts, G_hat, n);
                }
            }
        }
    }

    int best = 0;
    for (int i = 1; i < 4; i++) {
        if (vals[i] < vals[best]) best = i;
    }
    out.b = simplex[best][0];
    out.T = simplex[best][1];
    out.L = simplex[best][2];
    out.residual = vals[best] / static_cast<float>(n);
    return out.b > 0.0f && out.residual < 1.0f;
}

// =============================================================================
// tunePid — phase condition by bisection on Td, magnitude condition for Kp,
// then numerical margin verification (log-spaced sweep).
// tunePid — Td は位相条件の二分法、Kp は振幅条件、余裕は対数掃引で数値検証。
// =============================================================================
bool tunePid(const Plant& plant, float wc, float pm_deg, float ti_factor,
             TuneResult& out)
{
    const float ti = ti_factor / wc;
    const Cplx g_c = plantResponse(wc, plant.b, plant.T, plant.L);
    const float phi_needed =
        (-180.0f + pm_deg) * kPi / 180.0f - g_c.arg();   // [rad]

    auto cPhase = [&](float td) { return pidUnit(wc, ti, td).arg(); };

    // C's phase is NOT monotonic in Td (the lead peak moves past wc): scan for
    // the peak, then bisect on the ASCENDING segment [0, Td_peak] only.
    // C の位相は Td に単調でない（リードのピークが wc を跨ぐ）: ピークを走査で
    // 探し、単調上昇区間 [0, Td_peak] でのみ二分する。
    const float td_max = 4.0f / (kEta * wc);
    float td_peak = 0.0f, phase_max = cPhase(0.0f);
    for (int i = 0; i <= 100; i++) {
        const float td_k = td_max * powf(10.0f, -4.0f + 4.0f * i / 100.0f);
        const float ph = cPhase(td_k);
        if (ph > phase_max) { phase_max = ph; td_peak = td_k; }
    }
    // Use phi_needed directly (do NOT wrap): the realizable PID lead is < 90°
    // (capped by eta), so phase_max gates the feasible range and an out-of-range
    // requirement returns false here — same as the host rate_sysid.py, which the
    // header pledges to mirror. Wrapping diverged from the host and could fold a
    // >180° requirement into a spurious td=0 "success" (code_review M-8).
    // phi_needed をそのまま使う（wrap しない）: 実現可能な PID リードは < 90°（eta で
    // 上限）ゆえ phase_max が可否を仕切り、範囲外は false を返す — ヘッダが「同一」と
    // 約束するホスト rate_sysid.py と一致。wrap はホストと乖離し、>180° の要求を誤って
    // td=0 の偽「成功」に畳み込みうる (M-8)。
    float td;
    const float needed = phi_needed;
    if (needed <= cPhase(0.0f)) {
        td = 0.0f;
    } else if (needed > phase_max) {
        return false;   // required lead beyond the PID maximum / 必要リードが上限超
    } else {
        float lo = 0.0f, hi = td_peak;
        for (int i = 0; i < 50; i++) {
            const float mid = 0.5f * (lo + hi);
            if (cPhase(mid) < needed) lo = mid; else hi = mid;
        }
        td = 0.5f * (lo + hi);
    }
    const float kp = 1.0f / (pidUnit(wc, ti, td).mag() * g_c.mag());

    // Numerical margin verification: sweep ω, find |L|=1 and arg L = −180°.
    // 数値検証: ω掃引で |L|=1 と位相 −180° を探す。
    out.kp = kp; out.ti = ti; out.td = td;
    out.wc = 0; out.pm_deg = 0; out.gm_db = 0; out.gm_valid = false;
    float prev_mag = 0, prev_ph = 0, prev_w = 0, prev_raw = 0;
    bool got_wc = false, got_gm = false;
    for (int i = 0; i <= 400; i++) {
        const float w = 0.5f * powf(2000.0f, i / 400.0f);   // 0.5 … 1000 rad/s
        Cplx Lw = cmul(pidUnit(w, ti, td), plantResponse(w, plant.b, plant.T, plant.L));
        Lw.re *= kp; Lw.im *= kp;
        const float mag = Lw.mag();
        // Continuous (unwrapped) phase: accumulate the WRAPPED increment of the
        // raw angle — adjacent sweep points differ by far less than π.
        // 連続（アンラップ）位相: 生角度の「折返し増分」を積算 — 掃引の隣接点の
        // 位相差は π より十分小さい。
        const float raw = Lw.arg();
        const float ph = (i == 0) ? raw : prev_ph + wrapAngle(raw - prev_raw);
        prev_raw = raw;
        if (i > 0) {
            if (!got_wc && (prev_mag - 1.0f) * (mag - 1.0f) <= 0.0f && prev_mag != mag) {
                const float f = (1.0f - prev_mag) / (mag - prev_mag);
                out.wc = prev_w + f * (w - prev_w);
                out.pm_deg = (prev_ph + f * (ph - prev_ph)) * 180.0f / kPi + 180.0f;
                got_wc = true;
            }
            if (!got_gm && (prev_ph + kPi) * (ph + kPi) <= 0.0f && prev_ph != ph) {
                const float f = (-kPi - prev_ph) / (ph - prev_ph);
                const float m180 = prev_mag + f * (mag - prev_mag);
                if (m180 > 1e-9f) {
                    out.gm_db = -20.0f * log10f(m180);
                    out.gm_valid = true;   // finite, meaningful GM (L-15)
                }
                // m180 ≈ 0 → |L| vanishes at −180°, GM effectively infinite:
                // leave gm_valid=false so a GM gate treats it as safe, not 0 dB.
                // m180≈0 → −180° で |L| が消失しGM実質無限大: gm_valid=false のままにし
                // GM ゲートが 0dB でなく安全側として扱えるようにする。
                got_gm = true;
            }
        }
        prev_mag = mag; prev_ph = ph; prev_w = w;
    }
    return got_wc;
}

}  // namespace sf::autotune
