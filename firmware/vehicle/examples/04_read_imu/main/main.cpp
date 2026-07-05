/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file main.cpp
 * @brief Example 04 — Read IMU (BMI270) data via SPI
 *        サンプル04 — SPI経由でIMU（BMI270）のデータを読む
 *
 * Initializes the BMI270 6-axis IMU over SPI and prints
 * accelerometer (g) and gyroscope (rad/s) data to the serial
 * console at 10 Hz.
 *
 * BMI270 6軸IMUをSPIで初期化し、加速度（g）と角速度（rad/s）を
 * 10Hzでシリアルコンソールに表示します。
 *
 * Hardware: StampFly — BMI270 on SPI2
 *   MOSI=GPIO14, MISO=GPIO43, SCK=GPIO44, CS=GPIO46
 * ハードウェア: StampFly — SPI2にBMI270
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmi270_wrapper.hpp"

static const char* TAG = "read_imu";

extern "C" void app_main(void)
{
    // Create BMI270 wrapper and initialize with default StampFly config
    // BMI270ラッパーを作成し、デフォルトのStampFly設定で初期化
    stampfly::BMI270Wrapper imu;
    auto config = stampfly::BMI270Wrapper::Config::defaultStampFly();

    ESP_LOGI(TAG, "Initializing BMI270...");
    esp_err_t ret = imu.init(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check SPI wiring: MOSI=14, MISO=43, SCK=44, CS=46");
        return;
    }
    ESP_LOGI(TAG, "BMI270 initialized successfully!");

    // Print header
    // ヘッダーを表示
    printf("\n");
    printf("%-8s  %-8s  %-8s  |  %-8s  %-8s  %-8s\n",
           "Ax(g)", "Ay(g)", "Az(g)", "Gx(r/s)", "Gy(r/s)", "Gz(r/s)");
    printf("-------------------------------------------------------\n");

    while (true) {
        // Read accelerometer and gyroscope data
        // 加速度と角速度のデータを読み取る
        stampfly::AccelData accel;
        stampfly::GyroData  gyro;

        ret = imu.readSensorData(accel, gyro);
        if (ret == ESP_OK) {
            printf("%+7.3f  %+7.3f  %+7.3f  |  %+7.3f  %+7.3f  %+7.3f\n",
                   accel.x, accel.y, accel.z,
                   gyro.x,  gyro.y,  gyro.z);
        } else {
            ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(ret));
        }

        // Read at 10 Hz (100 ms interval)
        // 10Hzで読み取り（100ms間隔）
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Increase the read rate to 100 Hz (delay = 10 ms)
//    読み取り速度を100Hzに上げてみよう（delay = 10ms）
//
// 2. Calculate the tilt angle from accelerometer:
//    加速度からチルト角を計算してみよう:
//    float roll  = atan2f(accel.y, accel.z) * 180.0f / M_PI;
//    float pitch = atan2f(-accel.x, accel.z) * 180.0f / M_PI;
//
// 3. Try reading accel and gyro separately with readAccel/readGyro
//    readAccel/readGyroで個別に読んでみよう
//
// 4. Read temperature with readTemperature()
//    readTemperature()でIMUの温度を読んでみよう
// ============================================================
