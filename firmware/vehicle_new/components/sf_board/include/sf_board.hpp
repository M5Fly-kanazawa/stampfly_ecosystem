/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/getting-started-with-stampfly-ecosystem
 */

/**
 * @file sf_board.hpp
 * @brief BSP (Board Support Package) — sole owner of shared HW resources
 *        BSP（ボードサポートパッケージ）— 共有 HW 資源の唯一の所有者
 *
 * sf_board は vehicle_new における HW 資源の所有を一箇所に集約する。
 * 各 HAL コンポーネントは sf_board の getter から bus handle を借用し、
 * extern グローバル変数を使わない構造を保証する。
 *
 * sf_board centralizes ownership of shared hardware resources in
 * vehicle_new. Each HAL component borrows bus handles via sf_board
 * getters, ensuring no extern global state is required.
 *
 * @design architecture.md §7 — ハードウェア初期化と所有権        [--]
 * @design hardware_init.md §3 — sf_board の責務                [--]
 * @design hardware_init.md §4 — 起動シーケンス                  [--]
 *
 * Cross-cutting rules / 横断ルール:
 *   R1: sf_board が共有 HW 資源の唯一の所有者である
 *   R2: HAL は Config 経由で bus handle を借用する
 *   R3: main.cpp は線形・宣言的 (Phase 1 で sf::internal::board::init() を 1 回呼ぶ)
 *   R4: 失敗 3 段階分類 (Critical = abort / Optional = present(id)=false / Recoverable = retry)
 *   R8: namespace 規約 (sf::internal は L3 専用、L0/L1/L2 から見えない)
 */

#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

namespace sf::internal::board {

/**
 * @brief Initialize all shared HW resources in correct order.
 *        共有 HW 資源を正しい順序で初期化する。
 *
 * Must be called exactly once from app_main() Phase 1, before any
 * task is created and before any HAL is initialized. Not idempotent.
 *
 * app_main() の Phase 1 から **必ず 1 度だけ** 呼ぶ。タスク生成・HAL
 * 初期化の前に呼ぶこと。冪等ではない。
 *
 * Internal init order:
 *   Level 0: default event loop (esp_event_loop_create_default)
 *   Level 1: I2C master bus (GPIO3=SDA, GPIO4=SCL, glitch filter)
 *   Level 2: (future) SPI host for IMU / Flow
 *   Level 3: (future) LEDC timer for motor / LED / buzzer
 *
 * @return ESP_OK on success.
 * @return ESP_FAIL or driver error code on Critical failure (caller
 *         should abort or display LED error pattern; see hardware_init.md §5).
 */
esp_err_t init();

/**
 * @brief Get the shared I2C master bus handle.
 *        共有 I2C マスターバスのハンドルを取得する。
 *
 * Used by sf_hal_bmp280, sf_hal_bmm150, sf_hal_vl53l3cx, sf_hal_power.
 * Returns nullptr if init() has not been called yet.
 *
 * sf_hal_bmp280 / sf_hal_bmm150 / sf_hal_vl53l3cx / sf_hal_power が借用する。
 * init() が未実行の場合は nullptr を返す。
 *
 * @return I2C bus handle, or nullptr if uninitialized.
 */
i2c_master_bus_handle_t i2c_bus();

/**
 * @brief Sensor presence query for Optional-class HAL.
 *        Optional 分類センサの存在問合せ。
 *
 * Failsafe / TelemetryTask が「ToF が存在しない場合」のような
 * 分岐を行うために使う。Critical センサ (IMU / Motor / NVS / I2C)
 * は init() で abort されるため、ここには登場しない。
 *
 * Used by Failsafe / TelemetryTask to branch on optional sensor
 * absence. Critical sensors (IMU / Motor / NVS / I2C bus) abort in
 * init() and never reach this query.
 */
enum class SensorId : uint8_t {
    Mag,        ///< BMM150 magnetometer
    FrontToF,   ///< VL53L3CX front-facing ToF
    Flow,       ///< PMW3901 optical flow
    Baro,       ///< BMP280 barometer
    Power,      ///< INA3221 power monitor
};

/**
 * @brief Returns true if the optional sensor is present and initialized.
 *        Optional センサが存在して初期化されているかを返す。
 *
 * Phase 1 (sf::internal::board::init()) の段階では未確定 (false 固定)。
 * 各 HAL タスクが起動して init() を成功したときにフラグが立つ。
 * 詳細な API は M2b 以降で拡張予定。
 *
 * Always returns false until M2b extends this API. Reserved for
 * future use; safe to call but not yet meaningful.
 */
bool sensor_present(SensorId id);

}  // namespace sf::internal::board
