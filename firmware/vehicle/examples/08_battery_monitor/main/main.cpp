/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file main.cpp
 * @brief Example 08 — Monitor battery voltage and current
 *        サンプル08 — バッテリー電圧・電流を監視する
 *
 * Initializes the INA3221 3-channel power monitor via I2C and
 * displays battery voltage (V), current (mA), and power (mW)
 * at 2 Hz. Warns when battery voltage drops below 3.4V.
 *
 * INA3221 3チャネル電力モニタをI2C経由で初期化し、バッテリー電圧（V）、
 * 電流（mA）、電力（mW）を2Hzで表示します。
 * バッテリー電圧が3.4V以下になると警告します。
 *
 * Hardware: StampFly — INA3221 on I2C (SDA=GPIO3, SCL=GPIO4)
 * ハードウェア: StampFly — I2CにINA3221 (SDA=GPIO3, SCL=GPIO4)
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "power_monitor.hpp"

// I2C bus configuration
// I2Cバス設定
static constexpr gpio_num_t GPIO_I2C_SDA = GPIO_NUM_3;
static constexpr gpio_num_t GPIO_I2C_SCL = GPIO_NUM_4;

static const char* TAG = "battery";

extern "C" void app_main(void)
{
    // Initialize I2C master bus
    // I2Cマスターバスを初期化
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port   = I2C_NUM_0;
    bus_config.sda_io_num = GPIO_I2C_SDA;
    bus_config.scl_io_num = GPIO_I2C_SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    // Initialize INA3221 power monitor
    // INA3221電力モニタを初期化
    stampfly::PowerMonitor power;
    stampfly::PowerMonitor::Config config;
    config.i2c_bus = i2c_bus;
    config.i2c_addr = stampfly::INA3221_I2C_ADDR_GND;  // 0x40

    ESP_LOGI(TAG, "Initializing INA3221...");
    esp_err_t ret = power.init(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INA3221 init failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check I2C wiring: SDA=3, SCL=4, addr=0x40");
        return;
    }
    ESP_LOGI(TAG, "INA3221 initialized successfully!");

    // Print header
    // ヘッダーを表示
    printf("\n");
    printf("%-10s  %-10s  %-10s  %-8s\n",
           "Volt(V)", "Curr(mA)", "Power(mW)", "Status");
    printf("--------------------------------------------\n");

    while (true) {
        // Read battery power data
        // バッテリーの電力データを読み取る
        stampfly::PowerData data;
        ret = power.read(data);

        if (ret == ESP_OK) {
            // Determine battery status
            // バッテリー状態を判定
            const char* status;
            if (power.isUsbOnly()) {
                status = "USB";
            } else if (power.isLowBattery()) {
                status = "LOW!";
            } else {
                status = "OK";
            }

            printf("%8.3f    %8.1f    %8.1f    %s\n",
                   data.voltage_v, data.current_ma, data.power_mw, status);

            // Warn if battery is low
            // バッテリー低下警告
            if (power.isLowBattery()) {
                ESP_LOGW(TAG, "LOW BATTERY! %.2fV (threshold: %.1fV)",
                         data.voltage_v,
                         stampfly::PowerMonitor::LOW_BATTERY_THRESHOLD_V);
            }
        } else {
            ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(ret));
        }

        // Read at 2 Hz (500 ms interval)
        // 2Hzで読み取り（500ms間隔）
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ============================================================
// Try changing! / ここを変えてみよう！
// ============================================================
// 1. Display battery percentage using power.getBatteryPercent()
//    power.getBatteryPercent()でバッテリー残量(%)を表示してみよう
//
// 2. Read all 3 channels using power.readChannel(ch, data)
//    power.readChannel(ch, data)で3チャネル全てを読んでみよう
//
// 3. Add an LED indicator: green=OK, yellow=low, red=critical
//    LED表示を追加: 緑=OK、黄=低下、赤=危険
//
// 4. Log data to console in CSV format for later analysis
//    後で分析できるようCSV形式でログを出力してみよう
// ============================================================
