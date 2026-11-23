/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware — host unit tests).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file test_param_boot_notice.cpp
 * @brief Boot-time notices for saved parameters the defaults have left behind: a yaw-rate derivative time
 *        (rate.yaw.td != 0) and the altitude gains (2026-11-23: any of the three still at its old default)
 *        既定値に取り残された保存済みパラメータに対する起動時の注意の試験: ヨー角速度の微分時間
 *        （rate.yaw.td が 0 でない）と、高さのゲイン（2026-11-23: 3 つのうち 1 つでも旧既定値のまま）
 *
 * Part 1 tests the pure decision (param_boot_notice.hpp). Part 2 runs the real
 * params::init() on the SILS host shims (in-memory NVS, log capture): a default value logs
 * nothing, a saved 0.01 logs exactly one warning line and keeps the value.
 * 第 1 部は純粋関数の判定（param_boot_notice.hpp）。第 2 部は SILS のホスト用シム（インメモリ NVS・
 * ログ捕捉）上で実物の params::init() を動かす: 既定値なら何も出さず、保存済みの 0.01 なら
 * 警告を丁度 1 行出し、値は保つ。
 *
 * @design detailed_design.md §4 — Yaw rate PID: why it has no derivative  [OK]
 */

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <string>

#include <esp_log.h>   // the SILS shim; the quoted form would pick the local stub / SILS のシム。引用符形式だとローカルの代用を拾う
#include "param_boot_notice.hpp"
#include "params.hpp"
#include "topics.hpp"

static constexpr float kSavedLeftoverTdS = 0.01f;   // the old default / 旧既定値
static constexpr float kTdTolerance = 1.0e-6f;
static const char* const kYawRateTdName = "rate.yaw.td";
// Text that identifies the notice line in the log. / 注意の行をログから見分ける文字列。
static const char* const kNoticeKeyword = "param set rate.yaw.td 0";

static int g_failures = 0;
#define CHECK(cond) \
    do { \
        if (!(cond)) { \
            ++g_failures; \
            printf("      FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        } \
    } while (0)

static std::string g_captured;

/// Log output function that appends every line to g_captured. / 全ての行を g_captured に足す出力関数。
static int captureVprintf(const char* format, va_list args)
{
    char line[512];
    vsnprintf(line, sizeof(line), format, args);
    g_captured += line;
    return 0;
}

/// Number of non-overlapping occurrences of the notice keyword in the capture. / 捕捉した文字列内の注意の語の出現回数。
static int countNotices()
{
    int count = 0;
    for (size_t at = g_captured.find(kNoticeKeyword); at != std::string::npos;
         at = g_captured.find(kNoticeKeyword, at + 1)) {
        ++count;
    }
    return count;
}

/// Run params::init() once with the log captured and return the notice count. / ログを捕捉して init() を 1 回動かし、注意の回数を返す。
static int initAndCountNotices()
{
    g_captured.clear();
    vprintf_like_t previous = esp_log_set_vprintf(captureVprintf);
    sf::params::init();
    esp_log_set_vprintf(previous);
    return countNotices();
}

/// Store `td` in NVS as the saved value. / `td` を保存済みの値として NVS に書く。
static void saveTd(float td)
{
    sf::params::set_float(kYawRateTdName, td);
    sf::params::save();
}

static void testDecision()
{
    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    const float kInf = std::numeric_limits<float>::infinity();
    CHECK(!sf::params::needsYawRateDerivativeNotice(0.0f));
    CHECK(sf::params::needsYawRateDerivativeNotice(kSavedLeftoverTdS));
    CHECK(sf::params::needsYawRateDerivativeNotice(1.0f));
    CHECK(!sf::params::needsYawRateDerivativeNotice(kNaN));
    CHECK(!sf::params::needsYawRateDerivativeNotice(kInf));
}

static void testBootNotice()
{
    float td = -1.0f;
    // Fresh NVS: the default, no notice. / 空の NVS: 既定値で注意なし。
    CHECK(initAndCountNotices() == 0);
    sf::params::get_float(kYawRateTdName, td);
    CHECK(std::fabs(td) < kTdTolerance);

    // A saved 0.01: exactly one notice, and the value is kept (not rewritten to 0).
    // 保存済みの 0.01: 注意は丁度 1 行、値は保たれる（0 に書き換えない）。
    saveTd(kSavedLeftoverTdS);
    CHECK(initAndCountNotices() == 1);
    CHECK(g_captured.find("0.0100") != std::string::npos);
    sf::params::get_float(kYawRateTdName, td);
    CHECK(std::fabs(td - kSavedLeftoverTdS) < kTdTolerance);

    // After the fix (saved 0): no notice. / 直したあと（0 を保存）: 注意なし。
    saveTd(0.0f);
    CHECK(initAndCountNotices() == 0);
}

// --- Altitude-control gains (2026-11-23) / 高さの制御のゲイン（2026-11-23） ---

static const char* const kAltitudeKpName = "altitude.alt.kp";
static const char* const kVelocityKpName = "altitude.vel.kp";
static const char* const kVelocityTiHoverName = "altitude.vel.ti_hover";
// Text that identifies the altitude notice line. / 高さの注意の行を見分ける文字列。
static const char* const kAltitudeNoticeKeyword = "altitude gains alt.kp=";
// The new defaults and the superseded ones. / 新しい既定値と、置き換えられた旧既定値。
static constexpr float kNewAltitudeKp = 0.85f;
static constexpr float kNewVelocityKp = 0.12f;
static constexpr float kNewVelocityTiHoverS = 2.0f;
static constexpr float kOldAltitudeKp = 0.45f;
static constexpr float kOldVelocityKp = 0.1f;
static constexpr float kOldVelocityTiHoverS = 2.5f;
static constexpr float kGainTolerance = 1.0e-6f;

static int countAltitudeNotices()
{
    int count = 0;
    for (size_t at = g_captured.find(kAltitudeNoticeKeyword); at != std::string::npos;
         at = g_captured.find(kAltitudeNoticeKeyword, at + 1)) {
        ++count;
    }
    return count;
}

/// Run params::init() once with the log captured and return the altitude notice count. / ログを捕捉して init() を 1 回
/// 動かし、高さの注意の回数を返す。
static int initAndCountAltitudeNotices()
{
    g_captured.clear();
    vprintf_like_t previous = esp_log_set_vprintf(captureVprintf);
    sf::params::init();
    esp_log_set_vprintf(previous);
    return countAltitudeNotices();
}

/// Store the three altitude gains in NVS (save() writes every parameter). / 高さの 3 つのゲインを NVS に保存する。
static void saveAltitudeGains(float alt_kp, float vel_kp, float vel_ti_hover_s)
{
    sf::params::set_float(kAltitudeKpName, alt_kp);
    sf::params::set_float(kVelocityKpName, vel_kp);
    sf::params::set_float(kVelocityTiHoverName, vel_ti_hover_s);
    sf::params::save();
}

static void testAltitudeDecision()
{
    const float kNaN = std::numeric_limits<float>::quiet_NaN();
    // The new defaults: no notice. / 新しい既定値: 注意なし。
    CHECK(!sf::params::needsAltitudeGainNotice(kNewAltitudeKp, kNewVelocityKp, kNewVelocityTiHoverS));
    // The whole old set, and each old value alone (a half-updated set). / 旧い組の全部と、旧い値 1 つだけ（半端な更新）。
    CHECK(sf::params::needsAltitudeGainNotice(kOldAltitudeKp, kOldVelocityKp, kOldVelocityTiHoverS));
    CHECK(sf::params::needsAltitudeGainNotice(kOldAltitudeKp, kNewVelocityKp, kNewVelocityTiHoverS));
    CHECK(sf::params::needsAltitudeGainNotice(kNewAltitudeKp, kOldVelocityKp, kNewVelocityTiHoverS));
    CHECK(sf::params::needsAltitudeGainNotice(kNewAltitudeKp, kNewVelocityKp, kOldVelocityTiHoverS));
    // A deliberate other value is not an old default (the earlier opt-in hover Ti 1.5 included). / 意図した別の値（以前の
    // 任意の hover Ti 1.5 を含む）は旧既定値ではない。
    CHECK(!sf::params::needsAltitudeGainNotice(0.6f, 0.15f, 1.5f));
    CHECK(!sf::params::needsAltitudeGainNotice(kNaN, kNaN, kNaN));
}

static void testAltitudeBootNotice()
{
    float value = -1.0f;
    // The table defaults are the new ones. This is the check that the three defaults in params.cpp were changed
    // together (a stale default would be seen here). / table の既定値は新しい値。params.cpp の 3 つの既定値が
    // そろって変わったことの確認（古い既定値が残れば、ここで見える）。
    sf::params::reset_all();
    sf::params::get_float(kAltitudeKpName, value);
    CHECK(std::fabs(value - kNewAltitudeKp) < kGainTolerance);
    sf::params::get_float(kVelocityKpName, value);
    CHECK(std::fabs(value - kNewVelocityKp) < kGainTolerance);
    sf::params::get_float(kVelocityTiHoverName, value);
    CHECK(std::fabs(value - kNewVelocityTiHoverS) < kGainTolerance);

    // Saved new values: no notice. / 新しい値が保存されている: 注意なし。
    saveAltitudeGains(kNewAltitudeKp, kNewVelocityKp, kNewVelocityTiHoverS);
    CHECK(initAndCountAltitudeNotices() == 0);

    // Saved old set (what every landing wrote before the change): exactly one notice, values kept, with the values
    // in the line. / 保存済みの旧い組（変更前は着陸のたびにこれが書かれた）: 注意は丁度 1 行、値は保たれ、行に値が入る。
    saveAltitudeGains(kOldAltitudeKp, kOldVelocityKp, kOldVelocityTiHoverS);
    CHECK(initAndCountAltitudeNotices() == 1);
    CHECK(g_captured.find("alt.kp=0.450") != std::string::npos);
    sf::params::get_float(kAltitudeKpName, value);
    CHECK(std::fabs(value - kOldAltitudeKp) < kGainTolerance);
    sf::params::get_float(kVelocityTiHoverName, value);
    CHECK(std::fabs(value - kOldVelocityTiHoverS) < kGainTolerance);

    // A half-updated set still gets the notice. / 半端に更新された組にも注意が出る。
    saveAltitudeGains(kNewAltitudeKp, kNewVelocityKp, kOldVelocityTiHoverS);
    CHECK(initAndCountAltitudeNotices() == 1);

    // After the fix: no notice. / 直したあと: 注意なし。
    saveAltitudeGains(kNewAltitudeKp, kNewVelocityKp, kNewVelocityTiHoverS);
    CHECK(initAndCountAltitudeNotices() == 0);
    // The yaw notice is independent (td is 0 here). / ヨーの注意は独立（ここでは td は 0）。
    CHECK(countNotices() == 0);
}

int main()
{
    // set_float() publishes a reload command on a topic, so the topics must exist first.
    // set_float() はトピックへ再読込コマンドを発行するので、先にトピックが必要。
    sf::topics_init();
    testDecision();
    testBootNotice();
    testAltitudeDecision();
    testAltitudeBootNotice();
    printf("test_param_boot_notice: %s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? 0 : 1;
}
