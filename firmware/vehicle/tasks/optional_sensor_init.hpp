/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file optional_sensor_init.hpp
 * @brief Bounded retry of an Optional I2C sensor's initialisation
 *        Optional な I2C センサの初期化を有限回だけ再試行する共通処理
 *
 * MagTask (BMM150), BaroTask (BMP280) and PowerTask (INA3221) share this one
 * function. A transient I2C error at boot is retried a fixed number of times
 * (config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS, waiting
 * config::OPTIONAL_SENSOR_INIT_RETRY_DELAY_MS between tries) before the caller
 * treats the sensor as absent. Every failed try logs one WARN line (sensor,
 * try number, error name); a success after a retry logs one INFO line, so a
 * transient failure can be told from a missing sensor in the boot log.
 *
 * MagTask (BMM150)・BaroTask (BMP280)・PowerTask (INA3221) がこの 1 つの関数を共有する。
 * 起動時の一時的な I2C エラーは、呼び出し側がセンサ不在とみなす前に、決まった回数
 * （config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS、試行の間は
 * config::OPTIONAL_SENSOR_INIT_RETRY_DELAY_MS だけ待つ）再試行する。失敗した試行ごとに
 * WARN を 1 行（センサ名・試行番号・エラー名）、再試行で成功したら INFO を 1 行出すので、
 * 起動ログから一時的な失敗とセンサ不在を区別できる。
 *
 * It lives in tasks/ (not in sf_board) because the retry policy is a task-level
 * boot behaviour (it waits with vTaskDelay and reads config.hpp); the HAL
 * drivers only guarantee that init() can be called again after a failure.
 * 置き場所を sf_board でなく tasks/ にしたのは、再試行方針がタスク層の起動時動作
 * （vTaskDelay で待ち config.hpp を読む）だから。HAL ドライバは「失敗後に init() を
 * 再度呼んでも安全」であることだけを保証する。
 *
 * @design hardware_init.md §5 — Optional I2C sensors retry init before "absent"  [OK]
 * @design architecture.md §3 — R1: the HAL owns its handle; the task only calls  [OK]
 */

#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.hpp"

namespace sf {

/// Run `init_once` (a callable returning esp_err_t) until it succeeds or the
/// attempt limit is reached. Returns ESP_OK on success, otherwise the error of
/// the last attempt. Never retries after success.
/// `init_once`（esp_err_t を返す呼び出し可能オブジェクト）を、成功するか試行回数の
/// 上限に達するまで実行する。成功なら ESP_OK、失敗なら最後の試行のエラーを返す。
/// 成功した後は再試行しない。
template <typename InitFunction>
esp_err_t initOptionalI2cSensor(const char* log_tag, const char* sensor_name,
                                InitFunction init_once)
{
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS; ++attempt) {
        err = init_once();
        if (err == ESP_OK) {
            if (attempt > 1) {
                ESP_LOGI(log_tag, "%s init succeeded on attempt %d/%d (transient failure)",
                         sensor_name, attempt, config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS);
            }
            return ESP_OK;
        }
        ESP_LOGW(log_tag, "%s init failed (attempt %d/%d): %s", sensor_name, attempt,
                 config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS, esp_err_to_name(err));
        if (attempt < config::OPTIONAL_SENSOR_INIT_MAX_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(config::OPTIONAL_SENSOR_INIT_RETRY_DELAY_MS));
        }
    }
    return err;
}

}  // namespace sf
