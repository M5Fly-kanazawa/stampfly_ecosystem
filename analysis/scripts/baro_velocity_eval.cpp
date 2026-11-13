// baro_velocity_eval.cpp — offline evaluation of "does a barometer velocity help the blind descent's vertical velocity?"
//
// Replays a flight log's raw accelerometer, attitude and ESKF vertical velocity through the SAME firmware class
// (sf::VerticalVelocityEstimator, firmware/vehicle/components/sf_core/include/vertical_velocity_estimator.hpp) and, at each start
// time t0 of the event file, declares the ToF silent (eskf_trusted = false from then on: rule 3, the accelerometer integral with
// the offset B frozen). Variant (i): nothing else. Variant (ii): every new barometer conversion pulls the integral toward a barometer
// velocity with fuseExternalVelocity(measured, gain). The barometer velocity is a first-order low-pass (time constant tau) of the
// difference quotient of the pressure altitude between two conversions. Output: the error against the ToF-derived reference velocity
// at 1, 2, 4, 6, 8, 10 s after t0.
// 飛行ログの加速度計・姿勢・ESKF の鉛直速度を、ファームと同じクラス（sf::VerticalVelocityEstimator）に通す。各 t0 で ToF が出力停止
// （以後 eskf_trusted = false: 規則 3、オフセット B を固定した加速度計の積分）とする。(i) は何もしない。(ii) は新しい気圧計の変換ごとに
// fuseExternalVelocity(測定, gain) で積分を気圧計の速度へ寄せる。気圧計の速度は、2 つの変換の間の気圧高度の差分商の 1 次ローパス
// （時定数 tau）。t0 の 1, 2, 4, 6, 8, 10 s 後の、ToF 由来の基準速度に対する誤差を出す。
//
// Event file (time t [s], sorted): I t ax ay az qw qx qy qz eskf_vz_up at_rest trusted | B t altitude_m | R t ref_vz_up | S t0
// Usage: baro_velocity_eval events.txt tau_s gain     (gain 0 = variant (i))
// build: g++ -std=c++17 -O2 -I firmware/vehicle/components/sf_core/include -I firmware/vehicle/components/sf_math/include baro_velocity_eval.cpp -o baro_velocity_eval
#include "vertical_velocity_estimator.hpp"
#include "blind_landing.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr double kHorizons[] = {1, 2, 4, 6, 8, 10};
struct Ev { char k; double t; double v[11]; };

std::vector<Ev> read(const char* path)
{
    std::vector<Ev> out; std::ifstream in(path); std::string line;
    while (std::getline(in, line)) {
        std::istringstream f(line); std::string k; Ev e{}; f >> k >> e.t; e.k = k[0];
        for (double& x : e.v) { if (!(f >> x)) break; }
        out.push_back(e);
    }
    return out;
}
double refAt(const std::vector<Ev>& ref, double t)
{
    auto n = std::lower_bound(ref.begin(), ref.end(), t, [](const Ev& e, double x) { return e.t < x; });
    if (n == ref.begin() || n == ref.end() || n->t - (n - 1)->t > 0.15) return std::numeric_limits<double>::quiet_NaN();
    return (n - 1)->v[0] + (t - (n - 1)->t) / (n->t - (n - 1)->t) * (n->v[0] - (n - 1)->v[0]);
}
}  // namespace

int main(int argc, char** argv)
{
    if (argc < 4) { std::fprintf(stderr, "usage: %s events tau gain\n", argv[0]); return 2; }
    const double tau = std::atof(argv[2]); const float gain = static_cast<float>(std::atof(argv[3]));
    const std::vector<Ev> ev = read(argv[1]);
    std::vector<Ev> ref; for (const Ev& e : ev) if (e.k == 'R') ref.push_back(e);
    for (const Ev& s : ev) {
        if (s.k != 'S') continue;
        sf::VerticalVelocityEstimator est; double last_t = -1, last_alt = 0, vb = 0; bool have_b = false;
        double next_h = 0; int hi = 0; const double t0 = s.t;
        for (const Ev& e : ev) {
            if (e.t > t0 + 10.001) break;
            if (e.k == 'B') {
                if (have_b) {
                    const double dt = e.t - last_t;
                    vb += std::min(1.0, dt / tau) * ((e.v[0] - last_alt) / dt - vb);
                    if (gain > 0 && e.t > t0) est.fuseExternalVelocity(static_cast<float>(vb), gain);
                }
                have_b = true; last_t = e.t; last_alt = e.v[0];
            }
            if (e.k != 'I') continue;
            static double prev_t = 0; const double dt = (prev_t > 0 && e.t > prev_t) ? e.t - prev_t : 0.0025; prev_t = e.t;
            const float q[4] = {(float)e.v[3], (float)e.v[4], (float)e.v[5], (float)e.v[6]};
            const float f[3] = {(float)e.v[0], (float)e.v[1], (float)e.v[2]};
            sf::VerticalVelocityInput in{};
            in.dt_s = (float)dt; in.at_rest_on_ground = e.v[8] > 0.5; in.eskf_trusted = (e.t <= t0) && e.v[9] > 0.5;
            in.eskf_velocity_up = (float)e.v[7]; in.upward_force_raw = sf::upwardSpecificForce(q, f);
            const float v = est.update(in);
            if (hi < 6 && e.t >= t0 + kHorizons[hi]) {
                std::printf("V %.3f %.0f %.4f %.4f %d\n", t0, kHorizons[hi], v, refAt(ref, e.t), (int)est.source());
                ++hi;
            }
        }
        (void)next_h;
    }
    return 0;
}
