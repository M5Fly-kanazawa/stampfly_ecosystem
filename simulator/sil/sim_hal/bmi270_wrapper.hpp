/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file bmi270_wrapper.hpp
 * @brief SIL replacement for the BMI270 IMU wrapper (sensor seam).
 *        BMI270 IMU ラッパーの SIL 用差し替え（センサ境界）。
 *
 * Drop-in for sf_hal_bmi270/include/bmi270_wrapper.hpp: same namespace, class,
 * and the public surface imu_task.cpp uses. Instead of an SPI transaction it
 * returns a synthetic sample. The firmware (imu_task.cpp) is UNMODIFIED; the
 * SIL build simply puts this header on the include path instead of the HAL.
 *
 * sf_hal_bmi270/include/bmi270_wrapper.hpp のドロップイン: 同じ namespace・
 * クラス・imu_task.cpp が使う公開面。SPI トランザクションの代わりに合成
 * サンプルを返す。本体（imu_task.cpp）は無改変; SIL ビルドが HAL の代わりに
 * このヘッダを include パスに置くだけ。
 *
 * P1.1: returns a constant at-rest sample (the point of P1.1 is the scheduler,
 * not physics). P1.2 replaces the sample source with the MuJoCo physics state.
 *
 * P1.1: 静止の定数サンプルを返す（P1.1 の主眼はスケジューラで物理ではない）。
 * P1.2 でサンプル源を MuJoCo 物理状態に差し替える。
 */

#pragma once

#include <cstdint>
#include "esp_err.h"

namespace stampfly {

// Accelerometer sample [g] (same layout as the HAL).
// 加速度サンプル [g]（HAL と同じレイアウト）。
struct AccelData {
    float x;
    float y;
    float z;
};

// Gyroscope sample [rad/s].
// 角速度サンプル [rad/s]。
struct GyroData {
    float x;
    float y;
    float z;
};

// SIL stand-in for the BMI270 wrapper.
// BMI270 ラッパーの SIL 代役。
class BMI270Wrapper {
public:
    // Config is accepted for interface compatibility and ignored on the SIL.
    // Config はインターフェース互換のため受け取り、SIL では無視する。
    struct Config {
        static Config defaultStampFly() { return Config{}; }
    };

    esp_err_t init(const Config& config = Config::defaultStampFly());
    esp_err_t readSensorData(AccelData& accel, GyroData& gyro);
    esp_err_t readTemperature(float& temperature);
    bool isInitialized() const { return initialized_; }

private:
    bool initialized_ = false;
};

}  // namespace stampfly
