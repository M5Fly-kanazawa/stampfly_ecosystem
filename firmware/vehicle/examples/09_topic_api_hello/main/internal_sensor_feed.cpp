/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file internal_sensor_feed.cpp
 * @brief See internal_sensor_feed.hpp for why this file exists.
 *        このファイルが存在する理由は internal_sensor_feed.hpp を参照。
 *
 * @design topic_reference.md §4.3 — Publisher パターン（HW学習者向け）    [OK]
 * @design architecture.md §3 — sensor_imu / estimate_state データ型      [OK]
 */

#include "internal_sensor_feed.hpp"

#include "bmi270_wrapper.hpp"
#include "complementary_estimator.hpp"
#include "topics.hpp"
#include "sf_math.hpp"
#include "esp_log.h"
#include "esp_timer.h"

/// IMU SPI driver debug breadcrumb, referenced `extern` by the BMI270 C
/// driver (components/sf_hal_bmi270/src/bmi270_spi.c) for crash-dump
/// checkpoint codes (40..44). The driver only DECLARES it — whoever owns the
/// BMI270 instance must DEFINE it, exactly as tasks/imu_task.cpp does for the
/// real firmware. C linkage matches the driver's `extern` declaration.
/// IMU SPI ドライバのデバッグ用ブレッドクラム。BMI270 の C ドライバ
/// （components/sf_hal_bmi270/src/bmi270_spi.c）がクラッシュダンプ用の
/// チェックポイント値（40..44）記録のため `extern` 参照する。ドライバ側は
/// 宣言のみ — BMI270 インスタンスを所有する側が定義する必要がある。実ファームの
/// tasks/imu_task.cpp と全く同じ扱い。C リンケージはドライバの `extern` 宣言に合わせる。
extern "C" volatile uint8_t g_imu_checkpoint = 0;

namespace internal_feed {

namespace {

const char* const kLogTag = "internal_feed";

/// BMI270 driver instance, file-scope static — mirrors tasks/imu_task.cpp's
/// "sensor wrapper lives in the owning file, no extern globals" pattern
/// (hardware_init.md §6).
/// BMI270 ドライバのインスタンス、ファイルスコープ static — tasks/imu_task.cpp の
/// 「センサ wrapper は所有ファイルに置き、extern グローバルにしない」パターンを
/// 踏襲する（hardware_init.md §6）。
stampfly::BMI270Wrapper imu_driver;

/// Attitude estimator instance. ComplementaryEstimator (not the 15-state
/// ESKF) is used here because it needs only ImuData to produce roll/pitch/
/// yaw; the ESKF also wants ToF/Baro/Mag for its position states, which this
/// "hello" example does not read.
/// 姿勢推定器インスタンス。ここでは（15状態のESKFでなく）ComplementaryEstimator
/// を使う — roll/pitch/yaw を出すのに ImuData だけで済むため（ESKF は位置状態の
/// ため ToF/Baro/Mag も要求するが、本 "hello" サンプルはそれらを読まない）。
sf::ComplementaryEstimator attitude_estimator;

/// Convert the driver's body-frame reading (accel in g, gyro already in
/// rad/s) into the SI-unit ImuData the estimator and Topic expect. Mirrors
/// tasks/imu_task.cpp::applyImuTransform() — see that function's comment for
/// why no axis remap is needed (the BMI270 driver already outputs body FRD
/// axes).
/// ドライバの機体座標読み（加速度は g、角速度は既に rad/s）を、推定器と Topic が
/// 期待する SI 単位の ImuData へ変換する。tasks/imu_task.cpp::applyImuTransform()
/// を踏襲（軸 remap が不要な理由は同関数のコメント参照 — BMI270 ドライバは既に
/// 機体 FRD 軸で出力する）。
sf::ImuData buildImuSample(const stampfly::AccelData& accel_g,
                            const stampfly::GyroData& gyro_rad_per_s,
                            uint32_t timestamp_microseconds)
{
    sf::ImuData sample = {};
    sample.accel[0] = accel_g.x * sf::math::kGravity;
    sample.accel[1] = accel_g.y * sf::math::kGravity;
    sample.accel[2] = accel_g.z * sf::math::kGravity;
    sample.gyro[0] = gyro_rad_per_s.x;
    sample.gyro[1] = gyro_rad_per_s.y;
    sample.gyro[2] = gyro_rad_per_s.z;
    sample.timestamp = timestamp_microseconds;
    return sample;
}

}  // namespace

esp_err_t init()
{
    // BMI270Wrapper::Config::defaultStampFly() sets skip_bus_init = false, so
    // this driver owns its own SPI bus (no sf_board dependency) — the same
    // standalone pattern as examples/04_read_imu.
    // BMI270Wrapper::Config::defaultStampFly() は skip_bus_init = false のため、
    // このドライバは自分の SPI バスを所有する（sf_board 不要）—
    // examples/04_read_imu と同じ単体パターン。
    auto imu_config = stampfly::BMI270Wrapper::Config::defaultStampFly();
    esp_err_t result = imu_driver.init(imu_config);
    if (result != ESP_OK) {
        ESP_LOGE(kLogTag, "BMI270 init failed: %s", esp_err_to_name(result));
        return result;
    }

    attitude_estimator.init();
    ESP_LOGI(kLogTag, "IMU + complementary filter ready");
    return ESP_OK;
}

void step(float dt_seconds)
{
    stampfly::AccelData accel_g = {};
    stampfly::GyroData gyro_rad_per_s = {};
    esp_err_t result = imu_driver.readSensorData(accel_g, gyro_rad_per_s);
    if (result != ESP_OK) {
        ESP_LOGW(kLogTag, "IMU read failed: %s", esp_err_to_name(result));
        return;
    }

    // Publish the raw sample first (sensor_imu), then the estimate derived
    // from it (estimate_state) — same order as tasks/imu_task.cpp Step 1/2,
    // so a subscriber reading both never sees an estimate newer than the
    // sample it came from.
    // まず生サンプル（sensor_imu）、次にそこから導いた推定値（estimate_state）を
    // 発行する — tasks/imu_task.cpp の Step 1/2 と同じ順序。両方を読む購読者が
    // 「元サンプルより新しい推定値」を見ることがないようにするため。
    uint32_t now_microseconds = static_cast<uint32_t>(esp_timer_get_time());
    sf::ImuData imu_sample = buildImuSample(accel_g, gyro_rad_per_s, now_microseconds);
    sf::sensor_imu.publish(imu_sample);

    attitude_estimator.predict(imu_sample, dt_seconds);
    sf::estimate_state.publish(attitude_estimator.getState());
}

}  // namespace internal_feed
