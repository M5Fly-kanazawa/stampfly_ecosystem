/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file tof_task.cpp
 * @brief ToF sensor reading task (30Hz)
 *        ToFセンサ読み取りタスク（30Hz）
 *
 * Reads VL53L3CX bottom ToF data via the shared I2C bus owned by
 * sf_board, and publishes distance measurements to sensor_tof topic.
 * The forward-facing ToF (optional, log only) is read in the same cycle,
 * AFTER the bottom sensor, and published to sensor_tof_front.
 *
 * sf_board が所有する共有 I2C バス経由で VL53L3CX 底面 ToF を
 * 読み取り、sensor_tof トピックに距離計測を発行する。
 * 前方 ToF（任意センサ・ログ専用）は同じ周期内で底面の「後」に読み、
 * sensor_tof_front トピックに発行する。
 *
 * @publisher  sensor_tof, sensor_tof_front
 * @subscriber system_mode, failsafe_status (facts for the re-initialisation decision)
 *
 * @design architecture.md §6 — TofTask: Sensing(ToF)                  [OK]
 * @design detailed_design.md §8 — TofTask: 30Hz, priority 14         [OK]
 * @design hardware_init.md §3 — sf_board が i2c_bus を所有 (R1)      [OK]
 * @design hardware_init.md §5 — bottom ToF: Recoverable while disarmed [OK]
 * @design detailed_design.md §3 note 12 — ToF re-initialisation        [OK]
 * @design architecture.md INV-3 — detect + execute here, decide in sf::mayReinitializeToF [OK]
 * @design topic_reference.md §3 — sensor_tof: Queue 2, 30Hz          [OK]
 *
 * Failure classification (hardware_init.md §5):
 *   - VL53L3CX **bottom** sensor: Recoverable while the craft is NOT armed
 *     (INIT / IDLE_GROUND / IDLE_HELD). If init or startRanging fails at boot, or the
 *     sensor has delivered no sample for kTofReinitSilenceUs, this task resets it
 *     through XSHUT (GPIO 7) and brings it up again, every kTofReinitRetryIntervalUs
 *     for as long as it stays silent. The decision "may it be re-initialised now" is
 *     the pure sf::mayReinitializeToF (tof_silence_policy.hpp); this task only
 *     observes the facts and executes. While ARMED (ARMED_GROUND / TAKEOFF / FLYING /
 *     LANDING / FLIP) the sensor is never touched. ARM stays refused during the
 *     whole silence by ArmBlock::TofSilent; the task never exits.
 *     底面センサ: ARM されていない間（INIT / IDLE_GROUND / IDLE_HELD）は復旧可能。
 *     起動時の init / startRanging の失敗、または kTofReinitSilenceUs の間サンプルが
 *     無いとき、本タスクが XSHUT（GPIO 7）でリセットして立ち上げ直し、出力停止が
 *     続く間 kTofReinitRetryIntervalUs ごとに繰り返す。「今再初期化してよいか」の判断は
 *     純関数 sf::mayReinitializeToF（tof_silence_policy.hpp）で、本タスクは事実の観測と
 *     実行だけを行う。ARM 中（ARMED_GROUND / TAKEOFF / FLYING / LANDING / FLIP）は
 *     センサに触れない。出力停止の間ずっと ArmBlock::TofSilent が ARM を拒否する。
 *     タスクは終了しない。
 *   - VL53L3CX init failure for **front** sensor → Optional. Logged as
 *     a warning, XSHUT is held low, bottom sensing continues unchanged.
 *     前方センサの初期化失敗は Optional 扱い: 警告のみ、XSHUT を Low に保ち
 *     底面の測距は継続する。
 *   - Front sensor fails kFrontDisableFailCycles reads in a row (seen with USB
 *     power only) → disabled at run time the same way (reset + presence cleared).
 *     前方が kFrontDisableFailCycles 回連続で読み出しに失敗（USB 給電のみで発生）
 *     → 実行中に同じ方法で停止する（リセット保持 + presence を落とす）。
 *
 * Takeoff/Landing detection logic (TakeoffLandingMgr) runs in a
 * different layer and is wired in a later milestone (M4+) — this
 * task focuses on raw sensing only.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"   // front-ToF XSHUT control / 前方ToF XSHUT制御

#include "topics.hpp"
#include "config.hpp"
#include "sf_board.hpp"
#include "flight_limits.hpp"
#include "tof_silence_policy.hpp"
#include "vl53l3cx_wrapper.hpp"

static const char* TAG = "TofTask";

/// Cycle period [ticks]: 33ms ≈ 30Hz, matching VL53L3CX timing budget.
/// 周期 [tick]: 33ms ≈ 30Hz、VL53L3CX の測距 1 回の時間設定（timing budget）と整合
static constexpr TickType_t kPeriodTicks = pdMS_TO_TICKS(33);

/// Throttle interval for read-failure warnings.
/// At 30Hz, log at most once every ~3 seconds during persistent failure.
/// 読み取り失敗ログの抑制間隔。30Hz で持続的失敗時は約 3 秒に 1 回まで警告
static constexpr uint32_t kReadFailLogIntervalCycles = 90;

/// Consecutive front-sensor I2C failures after which the front ToF is disabled.
/// 30 cycles ≈ 1 s at 30Hz. Seen on hardware with USB power only: the front part
/// re-addresses fine but every later read fails, flooding the console.
/// 前方 ToF を停止する連続 I2C 失敗回数。30Hz で 30 回 ≈ 1 秒。USB 給電のみの実機で、
/// 前方はアドレス付け替えまで成功するがその後の読み出しが全て失敗し、コンソールを埋めた。
static constexpr uint32_t kFrontDisableFailCycles = 30;

/// Front sensor I2C address after re-addressing (bottom is 0x30). Both parts boot at
/// 0x29, so the front one is moved only AFTER the bottom one has left 0x29.
/// 前方センサの付け替え後 I2C アドレス（底面は 0x30）。両センサとも 0x29 で起動するため、
/// 底面が 0x29 を離れた「後」に前方を付け替える。
static constexpr uint8_t kFrontI2cAddress = stampfly::VL53L3CXWrapper::FRONT_I2C_ADDR;

/// VL53L3CX bottom sensor wrapper. File-scope static — owned by this
/// task only, never shared (per @publisher annotation above).
/// VL53L3CX 底面センサのラッパー。ファイルスコープ static で本タスクが
/// 所有し、外部共有しない (上記 @publisher アノテーション参照)。
static stampfly::VL53L3CXWrapper g_tof_bottom;

/// VL53L3CX front sensor wrapper (optional; used only when g_front_present is true).
/// VL53L3CX 前方センサのラッパー（任意。g_front_present が true のときのみ使用）。
static stampfly::VL53L3CXWrapper g_tof_front;
static bool g_front_present = false;

/// Convert wrapper distance reading to sf::TofData topic format.
/// Conversion: mm → m, range_status==0 means valid measurement.
/// ラッパーの距離値を sf::TofData トピック形式に変換する。
/// 変換: mm → m、range_status==0 を有効計測とする。
static sf::TofData buildTofTopic(const stampfly::DistanceData& src,
                                  uint32_t now_us)
{
    sf::TofData out{};
    out.timestamp = now_us;
    out.distance  = static_cast<float>(src.distance_mm) * 1e-3f;
    out.status    = src.range_status;
    out.valid     = (src.range_status == 0);
    out.signal_rate  = src.signal_rate;    // [Mcps] diagnostics for the log / ログ用の診断値
    out.ambient_rate = src.ambient_rate;   // [Mcps]
    out.sigma_mm     = src.sigma_mm;       // [mm]
    return out;
}

/// Hold a ToF in reset (its XSHUT low) so it cannot answer on the bus.
/// ToF をリセット保持（XSHUT Low）し、バスに応答させない。
static void holdInReset(int xshut_gpio)
{
    gpio_config_t xshut = {};
    xshut.pin_bit_mask = 1ULL << xshut_gpio;
    xshut.mode         = GPIO_MODE_OUTPUT;
    gpio_config(&xshut);
    gpio_set_level(static_cast<gpio_num_t>(xshut_gpio), 0);
}

/// Hold the front ToF in reset (XSHUT low) so it cannot answer at 0x29.
/// 前方 ToF をリセット保持（XSHUT Low）し、0x29 に応答させない。
static void holdFrontInReset()
{
    holdInReset(config::GPIO_TOF_XSHUT_FRONT);
}

/// Bring up the optional front sensor. Call only after the bottom sensor has been
/// moved off 0x29. On failure XSHUT is returned to low and bottom sensing is untouched.
/// 任意の前方センサを起動する。底面が 0x29 を離れた後にのみ呼ぶこと。失敗時は
/// XSHUT を Low に戻し、底面の測距には影響しない。
static void initFrontTof()
{
    auto cfg = stampfly::VL53L3CXWrapper::Config::defaultFront(
        sf::internal::board::i2c_bus());
    cfg.xshut_pin = static_cast<gpio_num_t>(config::GPIO_TOF_XSHUT_FRONT);
    cfg.i2c_addr  = kFrontI2cAddress;

    bool ok = (g_tof_front.init(cfg) == ESP_OK) && (g_tof_front.startRanging() == ESP_OK);
    if (!ok) {
        ESP_LOGW(TAG, "VL53L3CX front not detected (optional sensor) — disabled");
        holdFrontInReset();
        return;
    }
    g_front_present = true;
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::FrontToF, true);
    ESP_LOGI(TAG, "VL53L3CX front ready (addr 0x%02X, ~30Hz)", kFrontI2cAddress);
}

/// Poll one sensor once: if a sample is ready, read it, publish it to `topic`, stamp the
/// BSP freshness for `sensor_id`, and rearm the sensor. Read failures are log-throttled
/// per sensor through `last_fail_log_cycle`. Returns false when an I2C transaction
/// failed; "no new sample yet" is NOT a failure.
/// センサ 1 個を 1 回ポーリングする。新サンプルがあれば読み、`topic` に発行し、
/// `sensor_id` の鮮度を BSP に記録し、センサを再アームする。読み取り失敗のログは
/// `last_fail_log_cycle` でセンサ毎に抑制する。I2C 通信が失敗したら false を返す
/// （「新サンプル未到達」は失敗ではない）。
static bool pollSensor(stampfly::VL53L3CXWrapper& sensor,
                       sf::Topic<sf::TofData, sf::Queue, 2>& topic,
                       sf::internal::board::SensorId sensor_id,
                       const char* label,
                       uint32_t cycle_count,
                       uint32_t& last_fail_log_cycle)
{
    // Step 1: poll for new data / ステップ 1: 新規データの有無をポーリング
    bool ready = false;
    esp_err_t err = sensor.isDataReady(ready);
    if (err != ESP_OK) {
        if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
            ESP_LOGW(TAG, "%s isDataReady failed: %s", label, esp_err_to_name(err));
            last_fail_log_cycle = cycle_count;
        }
        return false;
    }
    if (!ready) {
        // No new sample yet — retry on next cycle.
        // 新サンプル未到達 — 次サイクルに再試行
        return true;
    }

    // Step 2: read distance and publish / ステップ 2: 距離を読んで発行
    stampfly::DistanceData reading{};
    err = sensor.getDistance(reading);
    if (err == ESP_OK) {
        uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());
        topic.publish(buildTofTopic(reading, now_us));
        // Report freshness to the BSP for the 1 Hz sensor_health snapshot (R15).
        // 1Hz の sensor_health 用に鮮度を BSP へ報告する (R15)。
        sf::internal::board::set_sensor_update(sensor_id, now_us);
    } else if (cycle_count - last_fail_log_cycle >= kReadFailLogIntervalCycles) {
        ESP_LOGW(TAG, "%s getDistance failed: %s", label, esp_err_to_name(err));
        last_fail_log_cycle = cycle_count;
    }

    // Required after each read to rearm the sensor for the next
    // measurement (per VL53L3CX driver contract).
    // 各読み取り後、次の計測用にセンサを再アームする必要がある
    // (VL53L3CX ドライバの契約)。
    sensor.clearInterruptAndStartMeasurement();
    return err == ESP_OK;
}

/// Count consecutive front-sensor failures and disable the front ToF once the streak
/// reaches kFrontDisableFailCycles: hold it in reset and clear its presence so
/// sensor_health, the CLI and the log all show it as absent. Bottom is unaffected.
/// 前方センサの連続失敗を数え、kFrontDisableFailCycles に達したら前方 ToF を停止する:
/// リセット保持し presence を落とすので、sensor_health・CLI・ログとも「前方なし」になる。
/// 底面には影響しない。
static void trackFrontFailure(bool ok, uint32_t& fail_streak)
{
    if (ok) {
        fail_streak = 0;
        return;
    }
    if (++fail_streak < kFrontDisableFailCycles) {
        return;
    }
    ESP_LOGW(TAG, "VL53L3CX front failed %u reads in a row — disabled "
                  "(battery not connected?)", static_cast<unsigned>(fail_streak));
    holdFrontInReset();
    g_front_present = false;
    sf::internal::board::set_sensor_present(
        sf::internal::board::SensorId::FrontToF, false);
}

/// What TofTask keeps about the bottom sensor between cycles.
/// 底面センサについて TofTask が周期をまたいで保持するもの。
struct BottomState {
    bool     ranging = false;             ///< Initialised and ranging; polled only while true / 初期化済みで測距中。真の間だけポーリングする
    uint32_t last_fail_log_cycle = 0;     ///< Read-failure log throttle (pollSensor) / 読み取り失敗ログの抑制（pollSensor）
    uint32_t last_seen_update_us = 0;     ///< Last BSP freshness stamp seen: a change means a sample was published / 最後に見た BSP 鮮度の時刻印: 変化は発行を意味する
    uint32_t attempts = 0;                ///< Re-initialisation attempts since the sensor last delivered / センサが最後に出力してからの再初期化の試行回数
    sf::ToFReinitClock clock;             ///< Silence and attempt times for sf::mayReinitializeToF / sf::mayReinitializeToF 用の出力停止と試行の時間
};

static uint32_t nowMicroseconds()
{
    return static_cast<uint32_t>(esp_timer_get_time());
}

/// Reset the bottom sensor through XSHUT, initialise it and start ranging. Presence is reported to the BSP
/// (R15) according to the result; a failed sensor is released so the next call can try again. The sensor
/// is configured with the same code at boot and at every re-initialisation.
/// XSHUT で底面センサをリセットし、初期化して測距を開始する。結果に応じて presence を BSP へ報告する (R15)。
/// 失敗したセンサは解放し、次の呼び出しでやり直せるようにする。起動時も再初期化でも同じコードで設定する。
static esp_err_t bringUpBottom()
{
    g_tof_bottom.release();
    auto cfg = stampfly::VL53L3CXWrapper::Config::defaultBottom(sf::internal::board::i2c_bus());
    esp_err_t result = g_tof_bottom.init(cfg);
    if (result == ESP_OK) {
        result = g_tof_bottom.startRanging();
    }
    if (result != ESP_OK) {
        g_tof_bottom.release();
    }
    sf::internal::board::set_sensor_present(sf::internal::board::SensorId::BottomToF, result == ESP_OK);
    return result;
}

/// Note a sample of the bottom sensor, seen as a change of its BSP freshness stamp (pollSensor writes it
/// when it publishes), so the silence clock starts over.
/// 底面センサのサンプルを記録する。BSP の鮮度の時刻印の変化として見える（発行時に pollSensor が書く）ので、
/// 出力停止の時計を数え直す。
static void noteBottomSample(BottomState& bottom, uint32_t now_us)
{
    const uint32_t stamp = sf::internal::board::sensor_last_update_us(sf::internal::board::SensorId::BottomToF);
    if (stamp == bottom.last_seen_update_us) {
        return;
    }
    bottom.last_seen_update_us = stamp;
    bottom.clock.onSample(now_us);
    bottom.attempts = 0;
}

/// The facts mayReinitializeToF() decides on: what this task observed plus what Failsafe and the state machine
/// published (failsafe_status read raw: a stale or missing status never says "silent", so nothing is reset).
/// mayReinitializeToF() が判断に使う事実: 本タスクの観測と、Failsafe・状態機械が発行したもの
/// （failsafe_status は生で読む。古い・無い状態は「出力停止」と言わないので、何もリセットしない）。
static sf::ToFReinitFacts observeReinitFacts(const BottomState& bottom, uint32_t now_us)
{
    const sf::FailsafeStatus status = sf::failsafe_status.latest();
    sf::ToFReinitFacts facts{};
    facts.state             = static_cast<sf::FlightState>(sf::system_mode.latest().state);
    facts.no_sample_episode = status.tof_silent &&
                              status.tof_reason == static_cast<uint8_t>(sf::ToFSilenceReason::NoSample);
    facts.silent_us         = bottom.clock.silentUs(now_us);
    facts.since_attempt_us  = bottom.clock.sinceAttemptUs(now_us);
    return facts;
}

/// Judge the finished attempt: too slow counts as failed (the sensor goes back in reset), and report the result.
/// One line on success; a failure line on the first failure and then every kTofReinitFailLogEveryAttempts-th attempt.
/// 終わった試行を判定して結果を報告する。遅すぎれば失敗とする（センサはリセットに戻す）。
/// 成功は 1 行。失敗の行は最初の失敗と、その後 kTofReinitFailLogEveryAttempts 回目ごと。
static void reportReinitResult(BottomState& bottom, esp_err_t result, uint32_t elapsed_us)
{
    if (result == ESP_OK && elapsed_us > sf::limits::kTofReinitAttemptTimeoutUs) {
        result = ESP_ERR_TIMEOUT;
        g_tof_bottom.release();
        holdInReset(config::GPIO_TOF_XSHUT_BOTTOM);
        sf::internal::board::set_sensor_present(sf::internal::board::SensorId::BottomToF, false);
    }
    bottom.ranging = (result == ESP_OK);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "VL53L3CX bottom re-init succeeded (attempt %u, %u ms)",
                 static_cast<unsigned>(bottom.attempts), static_cast<unsigned>(elapsed_us / 1000u));
    } else if ((bottom.attempts - 1u) % sf::limits::kTofReinitFailLogEveryAttempts == 0u) {
        ESP_LOGW(TAG, "VL53L3CX bottom re-init failed (attempt %u, %u ms): %s",
                 static_cast<unsigned>(bottom.attempts), static_cast<unsigned>(elapsed_us / 1000u),
                 esp_err_to_name(result));
    }
}

/// One re-initialisation attempt of the bottom sensor. Called only after mayReinitializeToF() said yes. Nothing
/// else runs in this task meanwhile, so no sample is published and the silence episode (hence the ARM refusal)
/// lasts for the whole attempt.
/// 底面センサの再初期化の試行 1 回。mayReinitializeToF() が「可」と答えた後でだけ呼ぶ。その間、本タスクは他の
/// ことをせず、サンプルは発行されないので、出力停止のエピソード（つまり ARM の拒否）は試行の全期間続く。
static void reinitBottom(BottomState& bottom, uint32_t now_us)
{
    ++bottom.attempts;
    ESP_LOGI(TAG, "VL53L3CX bottom re-init: attempt %u (no sample for %u ms)",
             static_cast<unsigned>(bottom.attempts), static_cast<unsigned>(bottom.clock.silentUs(now_us) / 1000u));
    const esp_err_t result = bringUpBottom();
    const uint32_t end_us = nowMicroseconds();
    reportReinitResult(bottom, result, end_us - now_us);
    bottom.clock.onAttemptEnd(end_us);
}

/// Poll the bottom sensor while it is ranging, note its samples, and re-initialise it when the policy allows.
/// 底面センサが測距中ならポーリングし、サンプルを記録し、判断が許せば再初期化する。
static void serviceBottom(BottomState& bottom, uint32_t cycle_count)
{
    if (bottom.ranging) {
        pollSensor(g_tof_bottom, sf::sensor_tof, sf::internal::board::SensorId::BottomToF, "bottom",
                   cycle_count, bottom.last_fail_log_cycle);
    }
    const uint32_t now_us = nowMicroseconds();
    noteBottomSample(bottom, now_us);
    if (sf::mayReinitializeToF(observeReinitFacts(bottom, now_us))) {
        reinitBottom(bottom, now_us);
    }
}

void TofTask(void* /*pvParameters*/)
{
    ESP_LOGI(TAG, "TofTask started");

    // -------------------------------------------------------------------
    // Two-sensor bring-up. Both VL53L3CX parts power up at the same I2C address
    // 0x29, so they must never be awake together while one is being re-addressed:
    //   1. Hold FRONT in reset, then bring up BOTTOM (0x29 -> 0x30).
    //   2. Only then release FRONT (0x29 -> 0x31), as an optional sensor.
    // A bottom sensor that fails here is not the end: the loop below re-initialises it
    // while the craft is not armed (the front sensor stays in reset until the next boot).
    // 2 センサ起動手順。VL53L3CX は 2 個とも同じ I2C アドレス 0x29 で起動するため、
    // 片方の付け替え中にもう片方が起きていてはならない:
    //   1. 前方をリセット保持したまま底面を起動する（0x29 -> 0x30）。
    //   2. その後に前方を解除する（0x29 -> 0x31）。前方は任意センサ。
    // ここで底面が失敗しても終わりではない: 下のループが、ARM されていない間に再初期化する
    // （前方センサは次の起動までリセットのまま）。
    // -------------------------------------------------------------------
    holdFrontInReset();

    BottomState bottom;
    const esp_err_t bring_up = bringUpBottom();
    bottom.clock.onBringUp(nowMicroseconds());
    bottom.ranging = (bring_up == ESP_OK);
    if (bottom.ranging) {
        ESP_LOGI(TAG, "VL53L3CX bottom ready (continuous ranging at ~30Hz)");
        // Bottom has left 0x29, so the optional front sensor can now be released safely.
        // 底面が 0x29 を離れたので、任意の前方センサを安全に解除できる。
        initFrontTof();
    } else {
        ESP_LOGE(TAG, "VL53L3CX bottom bring-up failed: %s - retried while the craft is not armed",
                 esp_err_to_name(bring_up));
    }

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle_count = 0;
    uint32_t last_front_fail_log_cycle = 0;
    uint32_t front_fail_streak = 0;

    while (true) {
        ++cycle_count;

        // Bottom first, so its timing is unaffected by the front sensor's I2C traffic.
        // 底面を先に処理し、前方センサの I2C 通信が底面のタイミングに影響しないようにする。
        serviceBottom(bottom, cycle_count);
        if (g_front_present) {
            const bool front_ok = pollSensor(g_tof_front, sf::sensor_tof_front,
                                             sf::internal::board::SensorId::FrontToF,
                                             "front", cycle_count,
                                             last_front_fail_log_cycle);
            trackFrontFailure(front_ok, front_fail_streak);
        }

        vTaskDelayUntil(&last_wake, kPeriodTicks);
    }
}
