/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (vehicle_new firmware).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file logger.hpp
 * @brief Data logger — Blackbox (SPIFFS) and DataStream (USB/UDP)
 *        データロガー — Blackbox（SPIFFS）およびDataStream（USB/UDP）
 *
 * Two logging backends:
 * - Blackbox: binary flight log to SPIFFS for post-flight analysis
 * - DataStream: real-time streaming via USB serial or UDP
 *
 * 2つのロギングバックエンド:
 * - Blackbox: フライト後解析用のSPIFFS上バイナリフライトログ
 * - DataStream: USBシリアルまたはUDP経由のリアルタイムストリーミング
 *
 * @design architecture.md §2 — Logging (responsibility #13)            [OK]
 * @design detailed_design.md §8 — Memory Layout (Blackbox/SPIFFS)      [OK]
 * @design coding_and_education.md §2 — Bilingual comments               [OK]
 */

#pragma once

#include <cstdint>

namespace sf {

/// Log output mode
/// ログ出力モード
enum class LogMode : uint8_t {
    DISABLED   = 0,   // Logging off          / ロギング無効
    BLACKBOX   = 1,   // SPIFFS binary log    / SPIFFSバイナリログ
    DATASTREAM = 2,   // Real-time streaming  / リアルタイムストリーミング
    BOTH       = 3,   // Blackbox + DataStream / 両方
};

/// Data logger: Blackbox + DataStream
/// データロガー: Blackbox + DataStream
class Logger {
public:
    /// Initialize logger subsystem (mount SPIFFS if needed)
    /// ロガーサブシステムを初期化する（必要に応じてSPIFSをマウント）
    void init();

    /// Record one sample from all topics (call at logging rate)
    /// 全トピックから1サンプルを記録する（ロギングレートで呼ぶ）
    void update();

    /// Start logging session (create new log file)
    /// ロギングセッションを開始する（新しいログファイルを作成）
    void startSession();

    /// Stop logging session (flush and close file)
    /// ロギングセッションを停止する（フラッシュしてファイルを閉じる）
    void stopSession();

    /// Set logging mode
    /// ロギングモードを設定する
    void setMode(LogMode mode) { mode_ = mode; }

    /// Get current logging mode
    /// 現在のロギングモードを取得する
    LogMode mode() const { return mode_; }

private:
    /// Mount SPIFFS partition
    /// SPIFSパーティションをマウントする
    void mountSpiffs();

    /// Write binary record to SPIFFS file
    /// SPIFSファイルにバイナリレコードを書き込む
    void writeBlackbox();

    /// Stream record via USB/UDP
    /// USB/UDP経由でレコードをストリーミングする
    void writeDataStream();

    LogMode  mode_        = LogMode::DISABLED;
    bool     spiffs_ok_   = false;    // SPIFFS mount status / SPIFSマウント状態
    bool     blackbox_failed_ = false;// Latched after a failed open; stops per-cycle
                                      // retry/log spam until the next stopSession().
                                      // open 失敗でラッチし、次の stopSession() まで
                                      // 毎周期の再試行/ログ氾濫を止める。
    int      log_fd_      = -1;       // Current log file fd / 現在のログファイルfd
    uint32_t record_count_ = 0;       // Records written     / 書き込みレコード数
};

}  // namespace sf
