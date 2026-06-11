/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file mag_task.cpp
 * @brief Magnetometer sensor task (25Hz) — BMM150 over shared I2C
 *        地磁気センサタスク (25Hz) — 共有 I2C 経由の BMM150
 *
 * Reads BMM150 magnetometer (3-axis micro-Tesla) via the shared I2C bus
 * owned by sf_board, and publishes to sensor_mag topic.
 *
 * sf_board が所有する共有 I2C 経由で BMM150 (3 軸 µT) を読み、
 * sensor_mag トピックに発行する。
 *
 * Owns the MagCalibrator (hard/soft-iron, sf_hal_bmm150): applies the NVS-loaded
 * correction to every sample, and executes the CLI `magcal` verbs (collect while
 * the user rotates the craft → sphere fit → NVS). The published MagData carries
 * a `calibrated` flag so downstream (ImuTask mag-reference capture) can refuse
 * to fuse an uncalibrated mag.
 * MagCalibrator（ハード/ソフトアイアン, sf_hal_bmm150）を所有: NVS から読んだ補正を
 * 全サンプルに適用し、CLI `magcal` の verb（ユーザーが機体を回す間収集 → 球面
 * フィット → NVS）を実行する。発行する MagData は `calibrated` フラグを持ち、下流
 * （ImuTask の磁気参照捕捉）が未校正の磁気を融合に入れない判断をできるようにする。
 *
 * @publisher  sensor_mag, mag_cal_status
 * @subscriber mag_command, system_mode
 *
 * @design architecture.md §6 — MagTask: Sensing(Mag)                  [OK]
 * @design detailed_design.md §8 — MagTask: 25Hz, priority 18         [OK]
 * @design hardware_init.md §3 — sf_board が i2c_bus を所有 (R1)      [OK]
 * @design topic_reference.md §3 — sensor_mag: Queue 2, 25Hz          [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - BMM150 init failure → Optional. ヨー推定がドリフトしやすくなる
 *     が ACRO/STAB/ALT/POS は ESKF + IMU で動く。abort せず deletion
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "topics.hpp"
#include "config.hpp"
#include "flight_state.hpp"     // isArmed (magcal refused while armed)
#include "sf_board.hpp"
#include "bmm150.hpp"
#include "mag_calibration.hpp"

static const char* TAG = "MagTask";

/// Cycle period [ticks]: 40ms = 25Hz
/// 周期 [tick]: 40ms = 25Hz
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(40);

/// Read-failure log throttle: at 25Hz, ~5s between warnings
/// 読み取り失敗ログ抑制: 25Hz で約 5 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 125;

/// BMM150 wrapper — task-local file-scope static (no extern global).
/// BMM150 ラッパー — タスクローカルなファイルスコープ static
static stampfly::BMM150 g_mag;

/// Hard/soft-iron calibrator — owned here (this task has the sample stream).
/// ハード/ソフトアイアン校正器 — 本タスクが所有（サンプル列を持つのはここ）。
static stampfly::MagCalibrator g_mag_cal;

/// Publish the calibration status for the CLI poller (R5: status by topic).
/// CLI ポーラー向けに校正状態を発行する（R5: 状態はトピックで）。
static void publishCalStatus(uint32_t now_us)
{
    const stampfly::MagCalibration& cal = g_mag_cal.getCalibration();
    sf::MagCalStatus st{};
    st.state        = static_cast<uint8_t>(g_mag_cal.getState());
    st.sample_count = static_cast<uint16_t>(g_mag_cal.getSampleCount());
    st.valid        = g_mag_cal.isCalibrated();
    st.offset[0] = cal.offset_x; st.offset[1] = cal.offset_y; st.offset[2] = cal.offset_z;
    st.scale[0]  = cal.scale_x;  st.scale[1]  = cal.scale_y;  st.scale[2]  = cal.scale_z;
    st.fitness   = cal.fitness;
    st.timestamp = now_us;
    sf::mag_cal_status.publish(st);
}

/// Execute one CLI `magcal` verb (see MagCalCmd). Runs in THIS task's context so
/// the calibrator object never crosses tasks. NVS writes (Save/Clear) stall the
/// flash cache — armed is refused for Start/Save (a stall mid-flight would zero
/// the motors for its duration; collection in flight is meaningless anyway).
/// CLI `magcal` の verb を1つ実行する（MagCalCmd 参照）。本タスクの文脈で実行し、
/// 校正器オブジェクトがタスクを跨がないようにする。NVS 書込み（Save/Clear）は
/// フラッシュキャッシュを止めるため、Start/Save は armed 中拒否（飛行中のストールは
/// その間モータをゼロにする。そもそも飛行中の収集は無意味）。
static void processCalCommand(const sf::MagCalCommand& cmd, uint32_t now_us)
{
    const bool armed = sf::isArmed(
        static_cast<sf::FlightState>(sf::system_mode.latest().state));

    switch (static_cast<sf::MagCalCmd>(cmd.command)) {
    case sf::MagCalCmd::Start:
        if (armed) {
            ESP_LOGW(TAG, "magcal start refused: armed");
            break;
        }
        g_mag_cal.startCalibration();
        ESP_LOGI(TAG, "Mag calibration started — rotate the craft in all "
                      "orientations (figure-8)");
        break;
    case sf::MagCalCmd::Stop:
        if (g_mag_cal.getState() == stampfly::MagCalibrator::State::COLLECTING) {
            g_mag_cal.computeCalibration();   // sets DONE or ERROR / DONE か ERROR に
        }
        break;
    case sf::MagCalCmd::Save:
        if (armed) {
            ESP_LOGW(TAG, "magcal save refused: armed (flash stall)");
            break;
        }
        g_mag_cal.saveToNVS();
        break;
    case sf::MagCalCmd::Clear:
        if (armed) {
            ESP_LOGW(TAG, "magcal clear refused: armed (flash stall)");
            break;
        }
        g_mag_cal.clearNVS();
        g_mag_cal.setCalibration(stampfly::MagCalibration{});  // revert to identity / 恒等へ
        break;
    case sf::MagCalCmd::None:
    default:
        break;
    }
    publishCalStatus(now_us);
}

/// Convert wrapper output to topic format.
/// ラッパー出力をトピック形式に変換。
static sf::MagData toTopic(const stampfly::MagData& src)
{
    sf::MagData out{};
    out.mag[0]    = src.x;
    out.mag[1]    = src.y;
    out.mag[2]    = src.z;
    out.timestamp = src.timestamp_us;
    return out;
}

void MagTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "MagTask started");

    // -------------------------------------------------------------------
    // Setup: borrow I2C bus from BSP and initialise BMM150
    // セットアップ: BSP から I2C バスを借用し BMM150 を初期化
    // -------------------------------------------------------------------
    stampfly::BMM150::Config cfg{};
    cfg.i2c_bus = sf::internal::board::i2c_bus();
    // ODR 25Hz to MATCH the 25Hz task period and config::MAG_DT (0.04s): the
    // driver default ODR_10HZ silently undersampled the 25Hz design rate
    // (surfaced by the Data Stream showing mag at ~10Hz on hardware).
    // ODR 25Hz — タスク周期 25Hz と config::MAG_DT(0.04s) に一致させる。ドライバ
    // 既定の ODR_10HZ は設計レート 25Hz を黙って下回っていた（Data Stream の実機
    // 計測で mag が ~10Hz と判明して発覚）。
    cfg.data_rate = stampfly::BMM150DataRate::ODR_25HZ;

    esp_err_t err = g_mag.init(cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BMM150 init failed: %s — Mag disabled (Optional)",
                 esp_err_to_name(err));
        sf::internal::board::set_sensor_present(
            sf::internal::board::SensorId::Mag, false);
        vTaskDelete(NULL);
        return;
    }
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::Mag, true);
    ESP_LOGI(TAG, "BMM150 ready (25Hz)");

    // Load the persisted hard/soft-iron calibration. Uncalibrated is a normal
    // first-boot state: samples publish raw with calibrated=false, and ImuTask
    // keeps the mag out of the fusion until `magcal` has been run.
    // 保存済みハード/ソフトアイアン校正を読む。未校正は初回起動の正常状態:
    // サンプルは calibrated=false の生値で発行され、`magcal` 実行までは ImuTask が
    // 磁気を融合に入れない。
    if (g_mag_cal.loadFromNVS() == ESP_OK && g_mag_cal.isCalibrated()) {
        const stampfly::MagCalibration& cal = g_mag_cal.getCalibration();
        ESP_LOGI(TAG, "Mag calibration loaded: offset=[%.1f %.1f %.1f] uT "
                      "scale=[%.2f %.2f %.2f] fitness=%.2f",
                 static_cast<double>(cal.offset_x), static_cast<double>(cal.offset_y),
                 static_cast<double>(cal.offset_z), static_cast<double>(cal.scale_x),
                 static_cast<double>(cal.scale_y),  static_cast<double>(cal.scale_z),
                 static_cast<double>(cal.fitness));
    } else {
        ESP_LOGW(TAG, "No mag calibration — yaw aiding disabled until 'magcal' is run");
    }
    publishCalStatus(static_cast<uint32_t>(esp_timer_get_time()));

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_fail_log_cycle = 0;

    // Outcome counters, reported once a minute (permanent health watermark,
    // same philosophy as the ImuTask load line): distinguishes "no new sample
    // yet" (DRDY low) from NaN overflow drops and I2C errors. This counter
    // found two real hardware bugs (ODR mismatch, missing DRDY gate).
    // 結果カウンタ（毎分1回の恒久健全性ログ。ImuTask の負荷ログと同じ思想）:
    // 「新サンプル未着」（DRDY low）と NaN（オーバーフロー）破棄・I2C エラーを
    // 区別する。このカウンタが実機バグを2件（ODR不整合・DRDYゲート欠落）発見した。
    uint32_t diag_ok = 0, diag_nodata = 0, diag_nan = 0, diag_err = 0;

    while (true) {
        ++cycle_count;

        stampfly::MagData reading{};
        err = g_mag.read(reading);
        if (err == ESP_OK && reading.data_ready) {
            ++diag_ok;
        } else if (err == ESP_OK) {
            ++diag_nodata;
        } else if (err == ESP_ERR_INVALID_RESPONSE) {
            ++diag_nan;
        } else {
            ++diag_err;
        }
        if ((cycle_count % 1500u) == 0u) {   // ~60 s at 25Hz
            ESP_LOGI(TAG, "mag 60s: ok=%lu nodata=%lu nan=%lu err=%lu",
                     static_cast<unsigned long>(diag_ok),
                     static_cast<unsigned long>(diag_nodata),
                     static_cast<unsigned long>(diag_nan),
                     static_cast<unsigned long>(diag_err));
            diag_ok = diag_nodata = diag_nan = diag_err = 0;
        }
        // CLI magcal verbs (drained every cycle, executed in this task's context).
        // CLI magcal verb（毎周期 drain、本タスク文脈で実行）。
        sf::MagCalCommand cal_cmd;
        while (sf::mag_command.read(cal_cmd)) {
            processCalCommand(cal_cmd, static_cast<uint32_t>(esp_timer_get_time()));
        }

        if (err == ESP_OK && reading.data_ready) {
            sf::MagData topic = toTopic(reading);

            const auto cal_state = g_mag_cal.getState();
            if (cal_state == stampfly::MagCalibrator::State::COLLECTING) {
                // Feed RAW samples to the collector (the fit must see the
                // uncorrected ellipsoid) and report progress for the CLI poller.
                // Armed mid-collection aborts — rotating by hand is a ground
                // operation and flight samples would corrupt the fit.
                // 収集器には「生」サンプルを入れ（フィットは未補正の楕円体を見る
                // 必要がある）、CLI ポーラー向けに進捗を発行する。収集中に armed に
                // なったら中止 — 手で回すのは地上作業で、飛行サンプルはフィットを壊す。
                if (sf::isArmed(static_cast<sf::FlightState>(
                        sf::system_mode.latest().state))) {
                    g_mag_cal.stopCalibration();
                    ESP_LOGW(TAG, "Mag calibration aborted: armed");
                } else {
                    g_mag_cal.addSample(topic.mag[0], topic.mag[1], topic.mag[2]);
                }
                publishCalStatus(topic.timestamp);
            }

            // Apply the hard/soft-iron correction to every published sample.
            // 発行する全サンプルにハード/ソフトアイアン補正を適用する。
            if (g_mag_cal.isCalibrated()) {
                g_mag_cal.applyCalibration(topic.mag[0], topic.mag[1], topic.mag[2],
                                           topic.mag[0], topic.mag[1], topic.mag[2]);
                topic.calibrated = true;
            }
            sf::sensor_mag.publish(topic);
            // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
            // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
            sf::internal::board::set_sensor_update(
                sf::internal::board::SensorId::Mag, topic.timestamp);
        } else if (err != ESP_OK &&
                   cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "BMM150 read failed: %s", esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }
        // data_ready=false は無声スキップ (sensor ODR < task rate の正常状況)。
        // Silently skip when data_ready=false (normal: ODR < task period).

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
