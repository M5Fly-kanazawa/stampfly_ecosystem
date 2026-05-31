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

#include "plant_bridge.hpp"

namespace stampfly {

esp_err_t BMI270Wrapper::init(const Config& /*config*/)
{
    initialized_ = true;
    return ESP_OK;
}

esp_err_t BMI270Wrapper::readSensorData(AccelData& accel, GyroData& gyro)
{
    // Closed loop: take the Plant's synthetic IMU (body-FRD, m/s², −9.8 at rest)
    // and inverse-remap it to the chip frame, so imu_task's (unchanged) forward
    // remap restores body-FRD. Accel back to [g]. This round-trip exists only
    // while the 论点2 driver normalization is deferred.
    // 閉ループ: Plant の合成 IMU（機体 FRD・m/s²・静止 −9.8）をチップ系へ逆 remap し、
    // imu_task の(無改変の)前進 remap が機体 FRD を復元する。加速度は [g] に戻す。
    // この往復は論点2 ドライバ正規化を保留している間だけ存在する。
    if (sil::bridge::has_plant) {
        const sf::ImuData& imu = sil::bridge::current_imu;
        constexpr float G = 9.80665f;
        accel.x =  imu.accel[1] / G;   // chip X = body Y / G
        accel.y =  imu.accel[0] / G;   // chip Y = body X / G
        accel.z = -imu.accel[2] / G;   // chip Z = −body Z / G
        gyro.x =  imu.gyro[1];         // chip X = body Y
        gyro.y =  imu.gyro[0];         // chip Y = body X
        gyro.z = -imu.gyro[2];         // chip Z = −body Z
        return ESP_OK;
    }

    // At level rest the BMI270 reads +1 g on chip Z (this is what the real
    // hardware reports). imu_task's axis remap (body.z = −chip.z·G) then yields
    // −9.8 m/s² on body Z (down) — the NED-consistent convention the ESKF expects
    // (gravity −9.8 down, so predict R·accel + g_ned balances to 0 at rest).
    // The previous −1 g here was a sign bug: it produced +9.8 on body Z and made
    // the ESKF integrate gravity the wrong way (the drone "fell" in rtos_smoke).
    // See [[project_sil_coordinate_frames]] (论点2). NOTE: chip Z = +1 g at level
    // rest must be bench-confirmed before flight.
    //
    // 水平静止で BMI270 はチップ Z に +1 g を返す（実機の報告値）。imu_task の軸変換
    // （body.z = −chip.z·G）後、機体 Z（下方）に −9.8 m/s²＝ESKF が期待する NED 整合の
    // 規約（重力 −9.8 が下、予測 R·accel + g_ned が静止で 0 に均衡）。旧 −1 g は符号
    // バグで機体 Z に +9.8 を生み、ESKF が重力を逆積分していた（rtos_smoke で機体が
    // 「落下」）。飛行前にチップ Z=+1 g を実機確認すること。
    accel.x = 0.0f;
    accel.y = 0.0f;
    accel.z = 1.0f;   // [g] chip Z: +1 g at level rest (matches the real BMI270)

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
