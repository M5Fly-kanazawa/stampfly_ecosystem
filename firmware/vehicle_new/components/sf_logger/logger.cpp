/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file logger.cpp
 * @brief Data logger implementation — Blackbox over SPIFFS
 *        データロガー実装 — SPIFFS を使った Blackbox
 *
 * Mounts the "storage" SPIFFS partition at "/spiffs", opens a per-session
 * binary log file, and appends fixed-size records on each update() call.
 *
 * @design architecture.md §7 — Logging subsystem                       [OK]
 * @design detailed_design.md §8 — partitions: storage 2MB SPIFFS       [OK]
 * @design topic_reference.md §3 — Logger consumes multiple topics      [OK]
 *
 * @subscriber sensor_imu, estimate_state, control_output, actuator_motor
 */

#include "logger.hpp"
#include "topics.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"

#include <cstdio>
#include <cstring>
#include <sys/stat.h>

static const char* TAG = "logger";

// SPIFFS mount configuration
// SPIFFS マウント設定
static constexpr const char* kSpiffsMountPoint = "/spiffs";
static constexpr const char* kSpiffsPartLabel  = "storage";
static constexpr size_t      kMaxFiles         = 5;

// Session file naming. With ESP32 having no real-time clock at boot,
// we use a wraparound counter; once a session ends, the next boot
// starts at 0 (the application can use NVS to persist this if desired).
//
// ESP32 は起動時に RTC を持たないため、ファイル名にはラップアラウンド
// カウンタを使う。NVS への持続化は将来の課題。
static constexpr const char* kSessionFmt = "/spiffs/log_%03u.bin";

// Maximum records before forcibly rotating to a new session file.
// At 50Hz × 76 bytes ≈ 3.8 KB/s, 8000 records ≈ 160 s ≈ 2.7 minutes.
// セッション当たり最大レコード数。50Hz で 8000 件 ≈ 約 2.7 分に相当。
static constexpr uint32_t kMaxRecordsPerSession = 8000;

// File-scope state shared by Logger methods (logger.hpp only declares
// log_fd_ for tracking open-state; the FILE* itself stays here so that
// the public header does not need to include <cstdio>).
//
// logger.hpp に FILE* を持たせると <cstdio> を transitive に exposure
// するため、ファイルスコープ static で隠蔽する。本コンポーネントは
// 単一インスタンスを LogTask が所有する設計なので static で問題ない。
static FILE* g_log_fp = nullptr;

namespace sf {

// File header written at the start of each session file.
// 各セッションファイルの先頭に書く識別ヘッダ。
struct BlackboxHeader {
    char     magic[4];        // "SFLG"
    uint16_t version;         // 1
    uint16_t record_size;     // sizeof(BlackboxRecord)
    uint64_t boot_timestamp;  // esp_timer_get_time() at session start
    uint8_t  reserved[16];
};
static_assert(sizeof(BlackboxHeader) == 32, "header must be 32 bytes");

// One record per Logger::update() call. Fixed-size for fast indexing
// and predictable storage usage. Total = 76 bytes.
//
// 1 レコード = Logger::update() 呼び出しごとに 1 件。固定サイズで
// インデックス計算と容量予測を容易にする。合計 76 バイト。
struct BlackboxRecord {
    uint32_t timestamp_us;     // 4
    float    accel[3];         // 12  (m/s^2)
    float    gyro[3];          // 12  (rad/s)
    float    quat[4];          // 16  (w, x, y, z)
    float    thrust;           // 4   (N)
    float    torque[3];        // 12  (N·m)
    float    motor_duty[4];    // 16  (0.0 - 1.0)
};
static_assert(sizeof(BlackboxRecord) == 76, "record size mismatch");

// -----------------------------------------------------------------------------
// init — mount SPIFFS, ready to start a session on demand
// 初期化 — SPIFFS マウント、必要時にセッション開始可能な状態に
// -----------------------------------------------------------------------------
void Logger::init()
{
    mountSpiffs();
    ESP_LOGI(TAG, "Logger initialized (spiffs=%s)", spiffs_ok_ ? "ok" : "fail");
}

// -----------------------------------------------------------------------------
// update — record one sample from all topics
// 更新 — 全トピックから 1 サンプルを記録
// -----------------------------------------------------------------------------
void Logger::update()
{
    if (mode_ == LogMode::DISABLED) {
        return;
    }

    if (mode_ == LogMode::BLACKBOX || mode_ == LogMode::BOTH) {
        writeBlackbox();
    }

    if (mode_ == LogMode::DATASTREAM || mode_ == LogMode::BOTH) {
        writeDataStream();
    }
}

// -----------------------------------------------------------------------------
// startSession — create new log file with header
// セッション開始 — ヘッダ付きで新ログファイルを作成
// -----------------------------------------------------------------------------
void Logger::startSession()
{
    if (!spiffs_ok_) {
        ESP_LOGW(TAG, "startSession: SPIFFS not mounted, skipping");
        return;
    }

    // Find an unused log_NNN.bin slot (rolling, simple modulo strategy).
    // 未使用の log_NNN.bin スロットを探す (modulo によるシンプル方式)。
    char path[64];
    static uint32_t s_session_counter = 0;
    std::snprintf(path, sizeof(path), kSessionFmt,
                  static_cast<unsigned>(s_session_counter % 1000));
    s_session_counter = (s_session_counter + 1) % 1000;

    FILE* fp = std::fopen(path, "wb");
    if (fp == nullptr) {
        ESP_LOGE(TAG, "fopen('%s') failed", path);
        return;
    }

    BlackboxHeader hdr{};
    std::memcpy(hdr.magic, "SFLG", 4);
    hdr.version        = 1;
    hdr.record_size    = sizeof(BlackboxRecord);
    hdr.boot_timestamp = static_cast<uint64_t>(esp_timer_get_time());

    if (std::fwrite(&hdr, sizeof(hdr), 1, fp) != 1) {
        ESP_LOGE(TAG, "header write failed");
        std::fclose(fp);
        return;
    }

    log_fd_       = fileno(fp);   // store FILE* via fileno for symmetry
    g_log_fp       = fp;
    record_count_ = 0;
    ESP_LOGI(TAG, "Logging session started: %s", path);
}

// -----------------------------------------------------------------------------
// stopSession — flush and close log file
// セッション停止 — フラッシュしてログファイルを閉じる
// -----------------------------------------------------------------------------
void Logger::stopSession()
{
    // Clear the failure latch so the next session re-attempts the open once.
    // 失敗ラッチを解除し、次セッションが open を 1 度だけ再試行できるようにする。
    blackbox_failed_ = false;

    if (g_log_fp == nullptr) {
        return;
    }

    std::fflush(g_log_fp);
    std::fclose(g_log_fp);
    g_log_fp = nullptr;
    log_fd_ = -1;

    ESP_LOGI(TAG, "Logging session stopped (%lu records)",
             static_cast<unsigned long>(record_count_));
}

// -----------------------------------------------------------------------------
// mountSpiffs — mount SPIFFS partition labelled "storage"
// SPIFSマウント — partitions.csv の "storage" ラベルをマウント
// -----------------------------------------------------------------------------
void Logger::mountSpiffs()
{
    esp_vfs_spiffs_conf_t conf{};
    conf.base_path              = kSpiffsMountPoint;
    conf.partition_label        = kSpiffsPartLabel;
    conf.max_files              = kMaxFiles;
    conf.format_if_mount_failed = true;  // First-boot format

    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_vfs_spiffs_register failed: %s",
                 esp_err_to_name(err));
        spiffs_ok_ = false;
        return;
    }

    size_t total = 0;
    size_t used  = 0;
    err = esp_spiffs_info(kSpiffsPartLabel, &total, &used);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SPIFFS mounted: %s (%u / %u bytes used)",
                 kSpiffsMountPoint,
                 static_cast<unsigned>(used),
                 static_cast<unsigned>(total));
    }
    spiffs_ok_ = true;
}

// -----------------------------------------------------------------------------
// writeBlackbox — write one record from current topic snapshots
// Blackbox 書き込み — 現在のトピックスナップショットから 1 レコードを書く
// -----------------------------------------------------------------------------
void Logger::writeBlackbox()
{
    // Lazy-open: if no session file is open yet, start one. If the open already
    // failed this session, do NOT retry every cycle — that would spam the log at
    // the logging rate when SPIFFS is unavailable (e.g. no partition, or host SIL).
    // 遅延 open: セッション未開始ならここで開く。今セッションで既に失敗していれば
    // 毎周期は再試行しない — SPIFFS が無い (パーティション無し / ホスト SIL) とき
    // ロギングレートでログが氾濫するため。
    if (g_log_fp == nullptr) {
        if (blackbox_failed_) {
            return;
        }
        startSession();
        if (g_log_fp == nullptr) {
            blackbox_failed_ = true;  // latch until stopSession()
            return;
        }
    }

    // Rotate session when reaching record cap to bound file size.
    // レコード数上限到達時はファイル分割。
    if (record_count_ >= kMaxRecordsPerSession) {
        stopSession();
        startSession();
        if (g_log_fp == nullptr) {
            return;
        }
    }

    // Pull latest values from topics (Latest semantics, safe from any task).
    // トピックから最新値を取得 (Latest 方式、任意タスクから安全)。
    ImuData       imu   = sensor_imu.latest();
    StateEstimate est   = estimate_state.latest();
    ControlOutput ctrl  = control_output.latest();
    MotorOutput   motor = actuator_motor.latest();

    BlackboxRecord rec{};
    rec.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());
    rec.accel[0] = imu.accel[0]; rec.accel[1] = imu.accel[1]; rec.accel[2] = imu.accel[2];
    rec.gyro[0]  = imu.gyro[0];  rec.gyro[1]  = imu.gyro[1];  rec.gyro[2]  = imu.gyro[2];
    rec.quat[0] = est.attitude[0]; rec.quat[1] = est.attitude[1];
    rec.quat[2] = est.attitude[2]; rec.quat[3] = est.attitude[3];
    rec.thrust  = ctrl.thrust;
    rec.torque[0] = ctrl.torque[0]; rec.torque[1] = ctrl.torque[1]; rec.torque[2] = ctrl.torque[2];
    rec.motor_duty[0] = motor.duty[0]; rec.motor_duty[1] = motor.duty[1];
    rec.motor_duty[2] = motor.duty[2]; rec.motor_duty[3] = motor.duty[3];

    if (std::fwrite(&rec, sizeof(rec), 1, g_log_fp) != 1) {
        // Treat as recoverable; rate-limited error log handled by caller.
        // Recoverable 扱い、レート制限ログは呼び出し側で対応。
        return;
    }
    ++record_count_;
}

// -----------------------------------------------------------------------------
// writeDataStream — placeholder for USB / UDP streaming
// DataStream 書き込み — USB / UDP ストリーミング用プレースホルダ
// -----------------------------------------------------------------------------
//
// sf_telemetry が UDP を担当するため、本関数は将来 USB シリアル経由の
// 高速ストリームを追加するときに実装する。現状は no-op。
// sf_telemetry handles UDP. This stub remains as a hook for high-speed
// USB-serial streaming added in a future milestone (no-op for now).
void Logger::writeDataStream()
{
    // Intentionally empty (no-op). See header comment.
}

}  // namespace sf
