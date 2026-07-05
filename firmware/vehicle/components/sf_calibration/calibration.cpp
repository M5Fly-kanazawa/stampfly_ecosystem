/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file calibration.cpp
 * @brief Calibration manager implementation
 *        キャリブレーション管理の実装
 *
 * @design architecture.md §2 — Calibration (responsibility #11)        [OK]
 * @design detailed_design.md §3 — onEnter(IDLE_GROUND): boot calibration [OK]
 */

#include "calibration.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <cmath>
#include <cstring>

static const char* TAG = "calibration";

// NVS namespace and keys
// NVS名前空間とキー
static const char* NVS_NAMESPACE = "sf_cal";
static const char* NVS_KEY       = "cal_data";

// Gravity constant (matches the SSOT sf::math::kGravity; kept as a local literal here
// to avoid an sf_math dependency in this leaf component)
// 重力定数（SSOT sf::math::kGravity と同値。leaf コンポーネントの sf_math 依存を避けるため
// ここではローカルリテラルで保持）
static constexpr float G = 9.80665f;

namespace sf {

// -----------------------------------------------------------------------------
// init — load calibration from NVS
// 初期化 — NVSからキャリブレーションを読み込む
// -----------------------------------------------------------------------------
void CalibrationMgr::init()
{
    // NVS is touched ONCE per boot. init() is also reached from the 400Hz ImuTask
    // loop via the Recalibrate command (startBootCalibration), and an NVS read
    // there can block behind another task's flash commit (pairing save, param
    // save) for milliseconds — breaking the IMU period. On re-init just keep the
    // in-memory data; the fresh measurement overwrites it anyway.
    // NVS は起動につき1回だけ読む。init() は Recalibrate コマンド経由で 400Hz の
    // ImuTask ループからも呼ばれ（startBootCalibration）、そこで NVS を読むと他タスク
    // のフラッシュ commit（ペアリング保存・param 保存）の後ろで数msブロックし、IMU
    // 周期が崩れる。再 init ではメモリ上のデータを維持（どうせ新規測定が上書きする）。
    if (nvs_loaded_once_) {
        return;
    }
    nvs_loaded_once_ = true;

    if (loadFromNvs()) {
        ESP_LOGI(TAG, "Calibration loaded from NVS");
        ESP_LOGI(TAG, "  gyro_bias: [%.6f, %.6f, %.6f]",
                 data_.gyro_bias[0], data_.gyro_bias[1], data_.gyro_bias[2]);
    } else {
        // Normal on every boot today: saveToNvs() is deliberately not wired yet
        // (an NVS commit's flash erase stalls the 400Hz loop >10ms — persistence
        // waits for the flash auto-suspend investigation). The boot calibration
        // measures fresh values right after this and gates ARM until done, so
        // the zeros only cover the first ~4 s while disarmed.
        // 現状は毎起動で正常: saveToNvs() は意図的に未配線（NVS commit のフラッシュ
        // 消去が 400Hz ループを 10ms 超ストールさせるため、永続化は flash auto-suspend
        // の調査とセットで保留）。直後に起動校正がフレッシュに測定し、完了まで ARM を
        // ゲートするので、ゼロが使われるのは disarmed の最初の約4秒だけ。
        ESP_LOGI(TAG, "No saved calibration (normal: persistence not wired) — "
                      "boot calibration measures fresh");
        memset(&data_, 0, sizeof(data_));
    }
}

// -----------------------------------------------------------------------------
// startGyroCal — begin gyro bias calibration
// ジャイロキャリブレーション開始 — ジャイロバイアスキャリブレーションを開始
// -----------------------------------------------------------------------------
void CalibrationMgr::startGyroCal(uint32_t num_samples,
                                  const StillnessConfig& stillness)
{
    calibrating_    = true;
    target_samples_ = num_samples;
    sample_count_   = 0;
    memset(gyro_sum_,     0, sizeof(gyro_sum_));
    memset(accel_sum_,    0, sizeof(accel_sum_));
    memset(gyro_sq_sum_,  0, sizeof(gyro_sq_sum_));
    memset(accel_sq_sum_, 0, sizeof(accel_sq_sum_));

    still_          = stillness;
    ema_primed_     = false;
    restart_count_  = 0;

    ESP_LOGI(TAG, "Gyro calibration started (%lu still samples, gate: gyro<%.3f rad/s)",
             num_samples, static_cast<double>(still_.gyro_max));
}

// -----------------------------------------------------------------------------
// updateStillness — EMA the |gyro| and |accel| and judge rest (see StillnessConfig)
// 静止判定 — |gyro|・|accel| をEMAして静止を判定（StillnessConfig 参照）
// -----------------------------------------------------------------------------
bool CalibrationMgr::updateStillness(const float gyro[3], const float accel[3])
{
    const float gyro_mag = std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1]
                                     + gyro[2] * gyro[2]);
    const float accel_norm = std::sqrt(accel[0] * accel[0] + accel[1] * accel[1]
                                       + accel[2] * accel[2]);

    // Prime the EMAs with the first sample so the gate has no artificial
    // zero-start transient (a zero start would fake "still" at boot).
    // 初サンプルでEMAを初期化し、ゼロ始まりの偽過渡を作らない（ゼロ始まりは起動時に
    // 偽の「静止」を見せてしまう）。
    if (!ema_primed_) {
        gyro_mag_ema_   = gyro_mag;
        accel_norm_ema_ = accel_norm;
        ema_primed_     = true;
    }
    gyro_mag_ema_   += still_.ema_alpha * (gyro_mag - gyro_mag_ema_);
    accel_norm_ema_ += still_.ema_alpha * (accel_norm - accel_norm_ema_);

    return gyro_mag_ema_ < still_.gyro_max
        && accel_norm_ema_ > still_.accel_norm_min
        && accel_norm_ema_ < still_.accel_norm_max;
}

// -----------------------------------------------------------------------------
// restartAccumulation — motion detected: discard the partial window, start over
// やり直し — 動き検出: 部分蓄積を破棄して最初から
// -----------------------------------------------------------------------------
void CalibrationMgr::restartAccumulation()
{
    // Log only when real progress is lost (once per still→moving edge in
    // practice: after the discard the count stays 0 while motion continues).
    // 実際に進捗を失った時だけログする（事実上、静止→動きのエッジ毎に1回:
    // 破棄後は動きが続く間 count が 0 のまま）。
    if (sample_count_ > 0) {
        ++restart_count_;
        ESP_LOGW(TAG, "Motion during calibration — discarded %lu samples, "
                      "restarting (keep the craft still)",
                 static_cast<unsigned long>(sample_count_));
    }
    sample_count_ = 0;
    memset(gyro_sum_,     0, sizeof(gyro_sum_));
    memset(accel_sum_,    0, sizeof(accel_sum_));
    memset(gyro_sq_sum_,  0, sizeof(gyro_sq_sum_));
    memset(accel_sq_sum_, 0, sizeof(accel_sq_sum_));
}

// -----------------------------------------------------------------------------
// windowVariance — total variance of a 3-axis window (Σ per-axis variance).
// Variance is measured around the window's OWN mean, so it is insensitive to a
// constant sensor bias — exactly what is needed before the bias is known.
// 窓内分散 — 3軸窓の分散合計（軸ごとの分散の和）。窓「自身」の平均まわりの分散
// なので定数センサバイアスに不感 — バイアス既知前にまさに必要な性質。
// -----------------------------------------------------------------------------
float CalibrationMgr::windowVariance(const double sum[3], const double sq_sum[3],
                                     uint32_t count)
{
    if (count < 2) {
        return 0.0f;
    }
    const double n = static_cast<double>(count);
    double total = 0.0;
    for (int i = 0; i < 3; ++i) {
        const double mean = sum[i] / n;
        const double var  = sq_sum[i] / n - mean * mean;
        if (var > 0.0) {
            total += var;
        }
    }
    return static_cast<float>(total);
}

// -----------------------------------------------------------------------------
// feedSample — accumulate one verified-still IMU sample
// サンプル入力 — 静止確認済みIMUサンプルを1つ蓄積
// -----------------------------------------------------------------------------
bool CalibrationMgr::feedSample(const float gyro[3], const float accel[3])
{
    if (!calibrating_) {
        return false;
    }

    // Stillness gate (see StillnessConfig): motion discards the partial window.
    // The EMA keeps updating through motion so the gate re-opens only after the
    // craft has genuinely settled.
    // 静止ゲート（StillnessConfig 参照）: 動きは部分蓄積を破棄する。EMA は動いている間も
    // 更新し続けるため、本当に静定してからゲートが再び開く。
    if (!updateStillness(gyro, accel)) {
        restartAccumulation();
        return false;
    }

    // Accumulate gyro and accel values (and accel squares for the variance check)
    // ジャイロと加速度の値を蓄積（分散チェック用に加速度の二乗も）
    for (int i = 0; i < 3; ++i) {
        gyro_sum_[i]     += static_cast<double>(gyro[i]);
        accel_sum_[i]    += static_cast<double>(accel[i]);
        gyro_sq_sum_[i]  += static_cast<double>(gyro[i]) * gyro[i];
        accel_sq_sum_[i] += static_cast<double>(accel[i]) * accel[i];
    }

    sample_count_++;

    if (sample_count_ < target_samples_) {
        return false;   // still accumulating / まだ蓄積中
    }

    // Window complete — final validity check: the EMA gate can miss slow,
    // smooth motion (e.g. a gentle hand sway), but that motion leaves a
    // variance fingerprint over the whole window. Variance is bias-insensitive
    // (computed around the window's own mean), so it judges precisely even
    // though the sensor offsets are not yet known. Reject and start over.
    // 窓完了 — 最終妥当性チェック: EMA ゲートはゆっくり滑らかな動き（手のゆったり
    // した揺れ等）を見逃しうるが、その動きは窓全体の分散に痕跡を残す。分散は窓自身の
    // 平均まわりで計算されバイアス不感のため、センサオフセット未知でも精密に判定
    // できる。検出したら破棄してやり直す。
    const float gyro_var  = windowVariance(gyro_sum_,  gyro_sq_sum_,  sample_count_);
    const float accel_var = windowVariance(accel_sum_, accel_sq_sum_, sample_count_);
    if (gyro_var > still_.gyro_var_max || accel_var > still_.accel_var_max) {
        ESP_LOGW(TAG, "Calibration window rejected: gyro var %.5f (max %.5f), "
                      "accel var %.4f (max %.4f) — slow motion? restarting",
                 static_cast<double>(gyro_var),
                 static_cast<double>(still_.gyro_var_max),
                 static_cast<double>(accel_var),
                 static_cast<double>(still_.accel_var_max));
        restartAccumulation();
        return false;
    }

    computeAverages();
    computeLevelOffset();
    data_.valid  = true;
    calibrating_ = false;

    ESP_LOGI(TAG, "Calibration complete (still window verified, %lu restarts)",
             static_cast<unsigned long>(restart_count_));
    ESP_LOGI(TAG, "  gyro_bias: [%.6f, %.6f, %.6f]",
             data_.gyro_bias[0], data_.gyro_bias[1], data_.gyro_bias[2]);
    return true;
}

// -----------------------------------------------------------------------------
// computeAverages — calculate mean from accumulated samples
// 平均計算 — 蓄積サンプルから平均を計算
// -----------------------------------------------------------------------------
void CalibrationMgr::computeAverages()
{
    double n = static_cast<double>(sample_count_);
    for (int i = 0; i < 3; ++i) {
        data_.gyro_bias[i]  = static_cast<float>(gyro_sum_[i]  / n);
        data_.accel_bias[i] = static_cast<float>(accel_sum_[i] / n);
    }

    // Remove gravity from the Z-axis accel bias. At level rest the accelerometer
    // reads −G on body Z (gravity −9.8 on body-down, NED-consistent), so the true
    // bias is the mean PLUS G. (The old "-= G" yielded −2G — a latent sign bug:
    // the accelerometer is in the −g convention, not +g.)
    // Z軸加速度バイアスから重力を除去。水平静止で加速度計は機体 Z に −G を返す
    // （機体下方に重力 −9.8、NED 整合）ので、真のバイアスは平均＋G。
    // （旧 "-= G" は −2G を生む潜在符号バグ＝加速度計は −g 規約で +g ではない。）
    data_.accel_bias[2] += G;
}

// -----------------------------------------------------------------------------
// computeLevelOffset — derive roll/pitch offset from gravity vector
// レベルオフセット計算 — 重力ベクトルからロール/ピッチオフセットを導出
// -----------------------------------------------------------------------------
void CalibrationMgr::computeLevelOffset()
{
    // Rest SPECIFIC FORCE vector [ax, ay, az]: the raw rest mean, i.e. the bias
    // with the G that computeAverages added removed again (az ≈ −G·cosφ·cosθ).
    // The old "+ G" double-added gravity and produced az ≈ +G — the gravity
    // vector upside down, flipping both offset signs.
    // 静止時の「比力」ベクトル [ax, ay, az]: 生の静止平均、すなわち computeAverages が
    // 足した G を再び引いたもの（az ≈ −G·cosφ·cosθ）。旧 "+ G" は重力を二重加算して
    // az ≈ +G（重力ベクトルが上下逆）となり、両オフセットの符号が反転していた。
    float ax = data_.accel_bias[0];
    float ay = data_.accel_bias[1];
    float az = data_.accel_bias[2] - G;  // back to the raw rest mean / 生の静止平均へ戻す

    // Same convention as EskfCore::setAttitudeFromGravity: for true roll +φ the
    // rest specific force is [0, −g·sinφ, −g·cosφ], for true pitch +θ it is
    // [+g·sinθ, 0, −g·cosθ].
    // EskfCore::setAttitudeFromGravity と同一規約: 真のロール +φ で比力は
    // [0, −g·sinφ, −g·cosφ]、真のピッチ +θ で [+g·sinθ, 0, −g·cosθ]。
    data_.level_offset[0] = std::atan2(-ay, -az);                              // roll  = φ
    data_.level_offset[1] = std::atan2(ax, std::sqrt(ay * ay + az * az));      // pitch = θ

    ESP_LOGI(TAG, "  level_offset: [%.4f, %.4f] rad",
             data_.level_offset[0], data_.level_offset[1]);
}

// -----------------------------------------------------------------------------
// saveToNvs — persist calibration data to NVS
// NVS保存 — キャリブレーションデータをNVSに永続化
// -----------------------------------------------------------------------------
//
// Pre-condition: nvs_flash_init() has been called from app_main() (Phase 0).
// We open the dedicated namespace "sf_cal", store data_ as a blob under
// the key "cal_data", commit, and close.
//
// 前提条件: app_main() (Phase 0) で nvs_flash_init() 済み。
// 専用 namespace "sf_cal" を開いて、キー "cal_data" 下に data_ を blob
// として保存、commit、close する流れ。
// -----------------------------------------------------------------------------
void CalibrationMgr::saveToNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(handle, NVS_KEY, &data_, sizeof(data_));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_set_blob failed: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration saved to NVS (%u bytes)",
                 static_cast<unsigned>(sizeof(data_)));
    }
    nvs_close(handle);
}

// -----------------------------------------------------------------------------
// loadFromNvs — load calibration data from NVS
// NVS読み込み — NVSからキャリブレーションデータを読み込む
// -----------------------------------------------------------------------------
//
// Returns true only when:
//   1. nvs_open succeeds (namespace exists)
//   2. nvs_get_blob succeeds with the exact stored size
//   3. data_.valid flag is true (sentinel for genuine calibration)
//
// 以下の全てを満たすときのみ true を返す:
//   1. nvs_open 成功 (namespace 存在)
//   2. nvs_get_blob 成功、保存時と同サイズ
//   3. data_.valid フラグが true (キャリブ完了の sentinel)
// -----------------------------------------------------------------------------
bool CalibrationMgr::loadFromNvs()
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // First boot or never-saved is the common case — log at INFO not ERROR.
        // 初回起動 or 未保存は通常状況、INFO レベルでログ
        ESP_LOGI(TAG, "NVS namespace '%s' not found (%s)",
                 NVS_NAMESPACE, esp_err_to_name(err));
        return false;
    }

    size_t required = sizeof(data_);
    err = nvs_get_blob(handle, NVS_KEY, &data_, &required);
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGI(TAG, "NVS key '%s' not found (%s)",
                 NVS_KEY, esp_err_to_name(err));
        return false;
    }

    if (required != sizeof(data_)) {
        // Size mismatch means the struct layout changed since save. Treat
        // as invalid; user must recalibrate.
        // サイズ不一致は struct レイアウト変更を意味する。再キャリブ要求。
        ESP_LOGW(TAG, "NVS blob size mismatch (got %u, expected %u) — discarding",
                 static_cast<unsigned>(required),
                 static_cast<unsigned>(sizeof(data_)));
        std::memset(&data_, 0, sizeof(data_));
        return false;
    }

    if (!data_.valid) {
        ESP_LOGI(TAG, "NVS calibration blob present but valid=false");
        return false;
    }

    return true;
}

}  // namespace sf
