/**
 * @file test_params_sil.cpp
 * @brief Smoke test for the SIL params runtime — verifies that
 *        params.def is correctly expanded via the X-macro pattern
 *        and the firmware-style API returns the expected defaults.
 *        SIL params ランタイムのスモークテスト — params.def が
 *        X-macro パターンで正しく展開され、firmware と同形の API が
 *        期待値を返すことを検証する。
 *
 * Run with: ./test_params_sil
 * Exit: 0 on success, 1 on any failure.
 *
 * @design development_roadmap.md §2 — Principle 2: Parameter Identity   [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#include "params.hpp"

#include <cstdio>
#include <cmath>
#include <cstdlib>

namespace {

int failures = 0;

void check_float(const char* name, float expected, float tolerance = 1e-9f)
{
    float v = -999.0f;
    if (!sf::params::get_float(name, v)) {
        std::fprintf(stderr, "FAIL: get_float('%s') returned false\n", name);
        ++failures;
        return;
    }
    if (std::fabs(v - expected) > tolerance) {
        std::fprintf(stderr, "FAIL: '%s' = %g, expected %g\n",
            name, v, expected);
        ++failures;
        return;
    }
    std::fprintf(stdout, "PASS: %-32s = %g\n", name, v);
}

void check_bool(const char* name, bool expected)
{
    bool v = !expected;
    if (!sf::params::get_bool(name, v)) {
        std::fprintf(stderr, "FAIL: get_bool('%s') returned false\n", name);
        ++failures;
        return;
    }
    if (v != expected) {
        std::fprintf(stderr, "FAIL: '%s' = %s, expected %s\n",
            name, v ? "true" : "false", expected ? "true" : "false");
        ++failures;
        return;
    }
    std::fprintf(stdout, "PASS: %-32s = %s\n", name, v ? "true" : "false");
}

void check_set_float_clamps(const char* name, float bad_value)
{
    if (sf::params::set_float(name, bad_value)) {
        std::fprintf(stderr,
            "FAIL: set_float('%s', %g) accepted out-of-range value\n",
            name, bad_value);
        ++failures;
        return;
    }
    std::fprintf(stdout, "PASS: %-32s rejected out-of-range %g\n",
        name, bad_value);
}

void check_set_get_roundtrip(const char* name, float new_value)
{
    if (!sf::params::set_float(name, new_value)) {
        std::fprintf(stderr,
            "FAIL: set_float('%s', %g) returned false\n",
            name, new_value);
        ++failures;
        return;
    }
    float v = -999.0f;
    sf::params::get_float(name, v);
    if (std::fabs(v - new_value) > 1e-9f) {
        std::fprintf(stderr, "FAIL: roundtrip '%s' = %g, expected %g\n",
            name, v, new_value);
        ++failures;
        return;
    }
    std::fprintf(stdout, "PASS: %-32s set/get roundtrip %g\n",
        name, new_value);
}

}  // anonymous namespace

int main()
{
    std::fprintf(stdout, "=== SIL params runtime smoke test ===\n");

    sf::params::init();

    // Defaults match params.def
    // params.def の defaults と一致すること
    check_float("rate.roll.kp",                1.365e-3f);
    check_float("rate.roll.ti",                0.7f);
    check_float("attitude.roll.kp",            5.0f);
    check_float("attitude.roll.td",            0.04f);
    check_float("altitude.alt.kp",             0.6f);
    check_float("altitude.alt.ti",             7.0f);
    check_float("eskf.process.gyro_noise",     0.009655f);
    check_float("eskf.process.accel_noise",    0.3f);
    check_float("eskf.obs.accel_att_noise",    0.06f);
    check_float("safety.battery.low_v",        3.4f);

    check_bool("eskf.use_tof",   true);
    check_bool("eskf.use_flow",  true);
    check_bool("eskf.use_baro",  false);
    check_bool("eskf.use_mag",   false);

    // Range validation rejects out-of-range writes
    // 範囲検証が範囲外書き込みを拒否すること
    check_set_float_clamps("rate.roll.kp",     1.0f);   // > max 0.1
    check_set_float_clamps("attitude.roll.kp", -1.0f);  // < min 0
    check_set_float_clamps("safety.battery.low_v", 5.0f);  // > max 4.2

    // set/get roundtrip preserves the value
    // set/get ラウンドトリップが値を保持すること
    check_set_get_roundtrip("eskf.process.gyro_noise", 0.03f);
    check_set_get_roundtrip("attitude.roll.kp",        14.0f);

    // reset_all() restores defaults
    // reset_all() が defaults を復元すること
    sf::params::reset_all();
    check_float("eskf.process.gyro_noise", 0.009655f);
    check_float("attitude.roll.kp",        5.0f);

    // Total parameter count is non-zero
    // パラメータ総数が 0 ではないこと
    if (sf::params::count() < 30) {
        std::fprintf(stderr, "FAIL: count() = %d, expected >= 30\n",
            sf::params::count());
        ++failures;
    } else {
        std::fprintf(stdout, "PASS: count() = %d\n", sf::params::count());
    }

    std::fprintf(stdout, "=== %s (%d failure%s) ===\n",
        failures == 0 ? "ALL PASS" : "SOME FAILED",
        failures, failures == 1 ? "" : "s");

    return failures == 0 ? 0 : 1;
}
