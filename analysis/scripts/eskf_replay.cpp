// eskf_replay.cpp — offline ESKF parameter-sweep harness.
//
// Replays a flight log's RAW sensors through the SAME firmware ESKF (eskf_core.cpp,
// Code Identity) with a chosen parameter set, and reports innovation-consistency scores.
// Lets us find the best ESKF params from the DYNAMIC real flight (no stillness, no truth):
//   - accel-bias DRIFT: too-small accel_att_noise ⇒ the bias chases vibration ⇒ large drift.
//   - accel INNOVATION χ²-reject fraction (target ~5%): too-small R over-rejects (the bug).
//   - ToF innovation NIS (R-only proxy): tof_noise that makes it ~1 is consistent.
//   - optional accel pre-filter (LPF or notch) for the attitude update — tests whether a
//     filter on the gravity reference helps (e.g. the x-axis 11.9 Hz vibration).
//
// 飛行ログの生センサを、ファームと同一の ESKF にオフラインで通し、イノベーション整合性で
// スコアする。動的実データから最良 ESKF パラメータを探す（静止も真値も不要）。
//
// build: see analysis/scripts/eskf_sweep.py (g++ with the eskf include paths)
// args : accel_att_noise tof_noise accel_lpf_hz accel_notch_hz   (0 = filter off)
//        events on stdin (from eskf_replay_preprocess.py)

#include "eskf_core.hpp"
#include "sf_math.hpp"
#include <cstdio>
#include <cmath>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

using sf::math::Vec3;
using sf::math::Quat;
static const float G = sf::math::kGravity;

// One-pole low-pass (per axis) for the accel fed to the attitude update.
// 姿勢更新に渡す accel の1次ローパス（軸別）。
struct LPF1 {
    float a = 0.0f; Vec3 y{};
    void setHz(float hz, float dt) { a = (hz <= 0) ? 0.0f : 1.0f - std::exp(-2.0f * 3.14159265f * hz * dt); }
    Vec3 step(const Vec3& x) {
        if (a == 0.0f) return x;
        y.x += a * (x.x - y.x); y.y += a * (x.y - y.y); y.z += a * (x.z - y.z); return y;
    }
};
// 2nd-order biquad notch (per axis), Direct-Form-II transposed.
// 2次ノッチ（軸別）。
struct Notch {
    float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0;
    float wx1[3] = {}, wx2[3] = {};
    bool on = false;
    void set(float f0, float fs, float Q) {
        if (f0 <= 0) { on = false; return; }
        on = true;
        float w0 = 2.0f * 3.14159265f * f0 / fs, al = std::sin(w0) / (2.0f * Q), c = std::cos(w0);
        float a0 = 1 + al; b0 = 1 / a0; b1 = -2 * c / a0; b2 = 1 / a0; a1 = -2 * c / a0; a2 = (1 - al) / a0;
    }
    float step(int k, float x) {
        if (!on) return x;
        float y = b0 * x + wx1[k];
        wx1[k] = b1 * x - a1 * y + wx2[k];
        wx2[k] = b2 * x - a2 * y;
        return y;
    }
};

int main(int argc, char** argv) {
    float accel_att = argc > 1 ? std::atof(argv[1]) : 0.8f;
    float tof_noise = argc > 2 ? std::atof(argv[2]) : 0.03f;
    float lpf_hz    = argc > 3 ? std::atof(argv[3]) : 0.0f;
    float notch_hz  = argc > 4 ? std::atof(argv[4]) : 0.0f;

    sf::EskfConfig cfg;                     // struct defaults ≈ flight; override the sweep knobs
    cfg.accel_att_noise = accel_att;
    cfg.tof_noise = tof_noise;
    cfg.accel_att_lpf_hz = lpf_hz;          // use the CORE's LPF (Code Identity with firmware)
    cfg.accel_comp_enable = true;           // match the flown params.cpp
    cfg.use_flow = true; cfg.use_tof = true; cfg.use_mag = false; cfg.use_baro = false;

    sf::EskfCore core;
    core.init(cfg);

    // events
    std::string tok; std::getline(std::cin, tok);
    {   // INIT line
        std::istringstream ls(tok); std::string tag; float gb[3], ab[3];
        ls >> tag >> gb[0] >> gb[1] >> gb[2] >> ab[0] >> ab[1] >> ab[2];
        core.setGyroBias(Vec3(gb[0], gb[1], gb[2]));
        core.setAccelBias(Vec3(ab[0], ab[1], ab[2]));
    }

    LPF1 lpf; Notch notch;
    float last_t = -1.0f, last_flow_t = -1.0f;
    Vec3 ba0{}, ba_last{}; bool have_ba0 = false;
    // accumulators (flight window t>6s)
    double sum_nis_tof = 0; long n_tof = 0;
    long n_accel = 0, n_accel_rej = 0; double sum_innovmag = 0;
    double sum_roll = 0, sum_roll2 = 0, sum_pitch = 0, sum_pitch2 = 0; long n_att = 0;
    Vec3 lastg{}, lasta{}; float lastgx = 0, lastgy = 0;

    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream ls(line);
        char type; float t; ls >> type >> t;
        if (type == 'I') {
            Vec3 g, a; ls >> g.x >> g.y >> g.z >> a.x >> a.y >> a.z;
            float dt = (last_t < 0) ? 0.00267f : (t - last_t);
            if (dt <= 0 || dt > 0.05f) dt = 0.00267f;
            last_t = t; lastg = g; lasta = a; lastgx = g.x; lastgy = g.y;
            lpf.setHz(lpf_hz, dt);
            if (notch_hz > 0 && !notch.on) notch.set(notch_hz, 375.0f, 4.0f);
            core.predict(a, g, dt);

            // accel for the attitude update: optional harness-side notch (the core has no
            // notch); the LPF is applied INSIDE eskf_core (cfg.accel_att_lpf_hz) = Code Identity.
            Vec3 af = a;
            if (notch_hz > 0) { af.x = notch.step(0, a.x); af.y = notch.step(1, a.y); af.z = notch.step(2, a.z); }

            // external innovation (R-only NIS proxy; P unavailable offline) + χ² reject
            if (t > 6.0f) {
                Quat q = core.getAttitude();
                Vec3 ge = q.inv_rotate(Vec3(0, 0, -G));   // expected specific force at rest
                Vec3 innov(af.x - ge.x, af.y - ge.y, af.z - ge.z);
                float gdiff = af.norm() - G;
                float R = accel_att * accel_att * (1.0f + cfg.k_adaptive * gdiff * gdiff);
                float nis = (innov.x * innov.x + innov.y * innov.y + innov.z * innov.z) / R;
                n_accel++; if (nis > cfg.accel_chi2_gate) n_accel_rej++;
                sum_innovmag += std::sqrt(innov.x * innov.x + innov.y * innov.y + innov.z * innov.z);
                Vec3 e = q.to_euler();
                sum_roll += e.x; sum_roll2 += e.x * e.x; sum_pitch += e.y; sum_pitch2 += e.y * e.y; n_att++;
            }
            core.updateAccelAttitude(af);
            Vec3 ba = core.getAccelBias();
            if (!have_ba0 && t > 6.0f) { ba0 = ba; have_ba0 = true; }
            ba_last = ba;
        } else if (type == 'T') {
            float dist; int status; ls >> dist >> status;
            if (status == 0 && dist > 0) {
                if (t > 6.0f) {
                    Vec3 e = core.getAttitude().to_euler();
                    float height = dist * std::cos(e.x) * std::cos(e.y);
                    float innov = -height - core.getPosition().z;
                    sum_nis_tof += (innov * innov) / (tof_noise * tof_noise);
                    n_tof++;
                }
                core.updateToF(dist);
            }
        } else if (type == 'F') {
            int dx, dy, squal; ls >> dx >> dy >> squal;
            float dtf = (last_flow_t < 0) ? 0.0102f : (t - last_flow_t); last_flow_t = t;
            if (dtf <= 0 || dtf > 0.1f) dtf = 0.0102f;
            float height = -core.getPosition().z; if (height < 0.02f) height = 0.02f;
            core.updateFlowRaw((int16_t)dx, (int16_t)dy, height, dtf, lastgx, lastgy, (uint8_t)squal);
        } else if (type == 'B') {
            float alt; ls >> alt; (void)alt;   // baro disabled
        } else if (type == 'M') {
            float mx, my, mz; ls >> mx >> my >> mz; (void)mx;   // mag disabled
        }
    }

    double ba_drift = std::sqrt((ba_last.x - ba0.x) * (ba_last.x - ba0.x) +
                                (ba_last.y - ba0.y) * (ba_last.y - ba0.y) +
                                (ba_last.z - ba0.z) * (ba_last.z - ba0.z));
    double roll_std = std::sqrt(std::max(0.0, sum_roll2 / n_att - (sum_roll / n_att) * (sum_roll / n_att)));
    double pitch_std = std::sqrt(std::max(0.0, sum_pitch2 / n_att - (sum_pitch / n_att) * (sum_pitch / n_att)));
    // one CSV line: params then scores
    std::printf("%.4f %.4f %.1f %.1f  %.5f %.4f %.4f %.4f  %.4f %.4f %.4f  %.4f %.3f\n",
                accel_att, tof_noise, lpf_hz, notch_hz,
                ba_drift, ba_last.x, ba_last.y, ba_last.z,
                roll_std * 57.2958, pitch_std * 57.2958,
                (n_accel ? (double)n_accel_rej / n_accel : 0.0),
                (n_tof ? sum_nis_tof / n_tof : 0.0),
                (n_accel ? sum_innovmag / n_accel : 0.0));
    return 0;
}
