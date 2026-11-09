// Float32 identity check of the F1 thrustToDuty (the exact C++ code from actuator.cpp)
// against the SILS plant steady state (Plant::routOhm / Plant::steadyStateOmega copied in float).
// F1 の thrustToDuty（actuator.cpp の C++ コードそのもの）を、SILS プラント定常特性
// （Plant::routOhm / steadyStateOmega を float で複写）に対して検査する。
//
// Build/run (from the worktree root):
//   python3 analysis/scripts/ct_cq_bench_20261015/f1_identity_f32_run.py
#include <cmath>
#include <cstdio>
#include "simulator/sils/plant/generated_params.hpp"

static constexpr float MIN_DUTY = 0.0f;
static constexpr float MAX_DUTY = 1.0f;
#include "f1_block.inc"   // namespace plant_inverse + static float thrustToDuty(), extracted from actuator.cpp

namespace P = sils_params::plant;

static float routOhm(float duty) {
    constexpr int kLast = P::ROUT_N - 1;
    constexpr float kStep = P::ROUT_D[1] - P::ROUT_D[0];
    if (duty <= P::ROUT_D[1]) return P::ROUT_OHM[1];
    if (duty >= 1.0f) return P::ROUT_OHM[kLast];
    const float pos = duty / kStep;
    int i = static_cast<int>(pos);
    if (i > kLast - 1) i = kLast - 1;
    const float frac = pos - static_cast<float>(i);
    return P::ROUT_OHM[i] + frac * (P::ROUT_OHM[i + 1] - P::ROUT_OHM[i]);
}

static float steadyOmega(float duty, float v) {
    if (duty < P::ROUT_D[1]) return 0.0f;
    const float r = routOhm(duty > 1.0f ? 1.0f : duty);
    const float a = P::CQ + P::KT * P::I0B;
    const float b = P::KT * P::KE / r;
    const float c = P::KT * P::I0A - P::KT * v / r;
    const float disc = b * b - 4.0f * a * c;
    if (disc <= 0.0f) return 0.0f;
    const float w = (-b + std::sqrt(disc)) / (2.0f * a);
    return w > 0.0f ? w : 0.0f;
}

int main() {
    double worst = 0, worst_rel_thrust = 0;
    float wd = 0, wv = 0;
    int n = 0;
    for (int di = 5; di <= 100; ++di) {
        for (int vi = 0; vi <= 90; ++vi) {
            const float d = di / 100.0f;
            const float v = 3.4f + vi * 0.01f;
            const float w = steadyOmega(d, v);
            const float t = P::THRUST_EFFICIENCY * P::CT * w * w;
            const float d2 = thrustToDuty(t, v);
            const double e = std::fabs(static_cast<double>(d2) - d);
            ++n;
            if (e > worst) { worst = e; wd = d; wv = v; }
            // thrust the plant makes at the recovered duty vs the requested thrust
            const float w2 = steadyOmega(d2, v);
            const float t2 = P::THRUST_EFFICIENCY * P::CT * w2 * w2;
            const double rel = std::fabs(static_cast<double>(t2) - t) / t;
            if (rel > worst_rel_thrust) worst_rel_thrust = rel;
        }
    }
    std::printf("float32 C++ path: %d points, max |duty error| = %.3e at D=%.2f V=%.2f; max relative thrust error = %.3e\n",
                n, worst, wd, wv, worst_rel_thrust);
    return 0;
}
