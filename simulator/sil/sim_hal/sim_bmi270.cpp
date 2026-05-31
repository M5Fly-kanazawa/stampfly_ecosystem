/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sim_bmi270.cpp
 * @brief SIL implementation of the BMI270 wrapper — synthetic IMU samples.
 *        BMI270 ラッパーの SIL 実装 — 合成 IMU サンプル。
 *
 * P1.1: a constant at-rest reading (accelerometer sees +1 g on sensor −Z,
 * gyro zero). P1.2 will draw the sample from the MuJoCo body state plus the
 * self-written noise/vibration model (noise_and_vibration_model.md).
 *
 * P1.1: 静止の定数（加速度計はセンサ −Z に +1 g、角速度ゼロ）。P1.2 で
 * MuJoCo 機体状態＋自作のノイズ/振動モデルからサンプルを引く。
 */

#include "bmi270_wrapper.hpp"

namespace stampfly {

esp_err_t BMI270Wrapper::init(const Config& /*config*/)
{
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BMI270Wrapper::readSensorData(AccelData& accel, GyroData& gyro)
{
    // At rest the accelerometer measures reaction to gravity: −1 g on sensor Z.
    // After imu_task's axis remap this becomes +9.8 m/s² on body Z (down).
    // 静止時、加速度計は重力への反作用を測る: センサ Z に −1 g。imu_task の
    // 軸変換後、機体 Z（下方）に +9.8 m/s² になる。
    accel.x = 0.0f;
    accel.y = 0.0f;
    accel.z = -1.0f;  // [g]

    gyro.x = 0.0f;
    gyro.y = 0.0f;
    gyro.z = 0.0f;    // [rad/s]

    return ESP_OK;
}

esp_err_t BMI270Wrapper::readTemperature(float& temperature)
{
    temperature = 25.0f;  // [°C]
    return ESP_OK;
}

}  // namespace stampfly
