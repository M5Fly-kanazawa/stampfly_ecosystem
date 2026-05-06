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
#include "sf_params_file.hpp"

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>

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

int main(int argc, char** argv)
{
    // Optional CLI demo mode:
    //   ./test_params_sil --demo <path>
    // Loads <path>, then prints the four most relevant tuning params.
    // Used to show end-to-end file → load_file → live values.
    //
    // CLI デモモード（任意）:
    //   ./test_params_sil --demo <path>
    // <path> をロードし、代表的なチューニングパラメータ4件を表示する。
    // ファイル → load_file → 実値の流れをエンドツーエンドで示すために使う。
    if (argc >= 3 && std::strcmp(argv[1], "--demo") == 0) {
        sf::params::init();
        int n = sf::params::load_file(argv[2]);
        std::fprintf(stdout, "load_file('%s') applied %d overrides\n",
            argv[2], n);
        float f = 0.0f; bool b = false;
        sf::params::get_float("eskf.process.gyro_noise", f);
        std::fprintf(stdout, "  eskf.process.gyro_noise = %g\n", f);
        sf::params::get_float("attitude.roll.kp", f);
        std::fprintf(stdout, "  attitude.roll.kp        = %g\n", f);
        sf::params::get_bool("eskf.use_baro", b);
        std::fprintf(stdout, "  eskf.use_baro           = %s\n",
            b ? "true" : "false");
        sf::params::get_float("rate.roll.kp", f);  // not overridden
        std::fprintf(stdout, "  rate.roll.kp (default)  = %g\n", f);
        return n < 0 ? 1 : 0;
    }

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

    // =========================================================================
    // load_file(): file-based parameter override (Phase 1.3)
    // load_file(): ファイル経由のパラメータオーバーライド（Phase 1.3）
    // =========================================================================
    sf::params::reset_all();

    const char* path = "/tmp/sf_params_test.txt";
    {
        std::FILE* fp = std::fopen(path, "w");
        if (fp == nullptr) {
            std::fprintf(stderr,
                "FAIL: could not create temp file %s\n", path);
            ++failures;
        } else {
            std::fprintf(fp,
                "# Phase 1.3 load_file smoke test\n"
                "\n"
                "# Float override (in range)\n"
                "eskf.process.gyro_noise = 0.03\n"
                "\n"
                "# Float with inline comment and odd spacing\n"
                "  attitude.roll.kp   =   14.0   # tuned\n"
                "\n"
                "# Bool overrides\n"
                "eskf.use_baro = true\n"
                "eskf.use_tof  = FALSE\n"
                "\n"
                "# Out-of-range (should be rejected, NOT counted)\n"
                "rate.roll.kp = 1.0\n"
                "\n"
                "# Unknown name (should be skipped, NOT counted)\n"
                "nonexistent.param = 42\n");
            std::fclose(fp);
        }
    }

    // We expect exactly 4 successful overrides:
    //   eskf.process.gyro_noise, attitude.roll.kp, eskf.use_baro, eskf.use_tof
    // 期待される成功オーバーライドは 4 件:
    //   eskf.process.gyro_noise, attitude.roll.kp, eskf.use_baro, eskf.use_tof
    int n_applied = sf::params::load_file(path);
    if (n_applied != 4) {
        std::fprintf(stderr,
            "FAIL: load_file returned %d, expected 4\n", n_applied);
        ++failures;
    } else {
        std::fprintf(stdout,
            "PASS: load_file applied %d overrides\n", n_applied);
    }

    // Overridden values match the file
    // オーバーライド値がファイルに一致すること
    check_float("eskf.process.gyro_noise", 0.03f);
    check_float("attitude.roll.kp",        14.0f);
    check_bool ("eskf.use_baro",           true);
    check_bool ("eskf.use_tof",            false);

    // Out-of-range line was rejected — params.def default is preserved
    // 範囲外の行は拒否され、params.def の default が維持されること
    check_float("rate.roll.kp", 1.365e-3f);

    // A parameter NOT mentioned in the file keeps its params.def default
    // ファイルに記述のないパラメータは params.def の default を保つこと
    check_float("altitude.alt.kp", 0.6f);
    check_bool ("eskf.use_flow",   true);

    // load_file on a non-existent path returns -1
    // 存在しないパスでは -1 を返すこと
    int rc = sf::params::load_file("/tmp/sf_params_does_not_exist_xyz.txt");
    if (rc != -1) {
        std::fprintf(stderr,
            "FAIL: load_file(missing) returned %d, expected -1\n", rc);
        ++failures;
    } else {
        std::fprintf(stdout, "PASS: load_file(missing) returned -1\n");
    }

    // Cleanup the temp file
    // 一時ファイルを削除
    std::remove(path);

    std::fprintf(stdout, "=== %s (%d failure%s) ===\n",
        failures == 0 ? "ALL PASS" : "SOME FAILED",
        failures, failures == 1 ? "" : "s");

    return failures == 0 ? 0 : 1;
}
