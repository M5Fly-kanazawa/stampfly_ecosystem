/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file sim_bmi270.cpp
 * @brief SILS implementation of the BMI270 wrapper — synthetic IMU samples.
 *        BMI270 ラッパーの SILS 実装 — 合成 IMU サンプル。
 *
 * P1.1: a constant at-rest reading (accelerometer sees +1 g on sensor −Z,
 * gyro zero). P1.2 will draw the sample from the MuJoCo body state plus the
 * self-written noise/vibration model (noise_and_vibration_model.md).
 *
 * P1.1: 静止の定数（加速度計はセンサ −Z に +1 g、角速度ゼロ）。P1.2 で
 * MuJoCo 機体状態＋自作のノイズ/振動モデルからサンプルを引く。
 */

#include "bmi270_wrapper.hpp"

#include "plant_bridge.hpp"

namespace stampfly {

esp_err_t BMI270Wrapper::init(const Config& /*config*/)
{
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BMI270Wrapper::readSensorData(AccelData& accel, GyroData& gyro)
{
    // AccelData/GyroData are body-frame (FRD) [g]/[rad/s]: the real BMI270 driver
    // (bmi270_wrapper) now absorbs the mounting and returns body axes, so this stub
    // (which replaces that driver on the host) must do the same. The Plant's synthetic
    // IMU is already body-FRD m/s², so just rescale accel to [g] — no remap (the old
    // chip-frame round-trip is gone now that the remap lives in the driver).
    // AccelData/GyroData は機体(FRD) [g]/[rad/s]: 実 BMI270 ドライバ(bmi270_wrapper)が
    // 搭載向きを吸収し機体軸を返すようになったので、それを host で置換する本スタブも同様に
    // する。Plant の合成 IMU は既に機体 FRD m/s² ゆえ加速度を [g] に直すだけ（remap は
    // ドライバへ移ったので旧チップ系往復は不要）。
    if (sils::bridge::has_plant) {
        const sf::ImuData& imu = sils::bridge::current_imu;
        constexpr float G = 9.80665f;
        accel.x = imu.accel[0] / G;   // body forward (FRD X) [g]
        accel.y = imu.accel[1] / G;   // body right   (FRD Y) [g]
        accel.z = imu.accel[2] / G;   // body down    (FRD Z) [g]
        gyro.x = imu.gyro[0];         // body roll rate  (FRD X) [rad/s]
        gyro.y = imu.gyro[1];         // body pitch rate (FRD Y)
        gyro.z = imu.gyro[2];         // body yaw rate   (FRD Z)
        return ESP_OK;
    }

    // At level rest, body-frame accel is [0, 0, -1 g] (FRD Z is down; the sensor
    // measures -9.8 m/s² on body Z, the NED-consistent convention the ESKF expects —
    // gravity -9.8 down, so predict R·accel + g_ned balances to 0 at rest).
    // 水平静止で機体加速度は [0,0,-1g]（FRD Z は下、機体 Z に -9.8 m/s²＝ESKF が期待する
    // NED 整合の規約。重力 -9.8 が下、予測 R·accel + g_ned が静止で 0 に均衡）。
    accel.x = 0.0f;
    accel.y = 0.0f;
    accel.z = -1.0f;  // [g] body Z (down): -1 g at level rest

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
