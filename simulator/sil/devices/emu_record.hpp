/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_record.hpp
 * @brief Deterministic input/event recorder for the emulator (events.jsonl).
 *        エミュレータの決定論的な入力/事象レコーダ（events.jsonl）。
 *
 * Records every injected input (ESP-NOW frame, console bytes, ...) as one JSON
 * line stamped with the VIRTUAL clock (esp_timer_get_time, µs). Because the
 * timestamp and payload come from the deterministic virtual clock + the fixed
 * scenario — never wall-clock — two runs of the same scenario produce a
 * byte-identical events.jsonl. The recorder is PASSIVE: it writes to a separate
 * file and touches no scheduler primitive, so enabling it does not perturb the
 * firmware trace. When no path is opened, every record call is a no-op, so the
 * default (no-scenario) run is byte-identical to before this feature.
 *
 * 注入された入力（ESP-NOW フレーム・コンソールバイト等）を仮想時計（µs）でスタンプし
 * JSON 1行として記録。タイムスタンプもペイロードも決定論的（壁時計不使用）なので、同一
 * シナリオの2回実行は events.jsonl が byte-identical。レコーダは受動的でスケジューラを
 * 触らない。未オープン時は全 record 呼び出しが no-op ＝ シナリオ無し実行は本機能前と同一。
 *
 * @design simulator/sil/RESET_PLAN.md §13 P8 — scripted input / scenario seam
 */

#pragma once

#include <cstdint>

extern "C" {

// Open the recorder for writing JSONL to `path`. A null/empty path leaves the
// recorder closed (every record call becomes a no-op).
// JSONL 書き込み用にオープン。null/空なら閉じたまま（record は no-op）。
void sil_emu_record_open(const char* path);

// Flush and close the recorder.
// フラッシュして閉じる。
void sil_emu_record_close(void);

// True when an output file is open.
// 出力ファイルが開いていれば true。
bool sil_emu_record_active(void);

// Record a raw byte payload on a named channel at the current virtual time.
// Bytes are hex-encoded so the line is exact and diff-friendly.
// 名前付きチャネルに raw バイト列を現在の仮想時刻で記録（hex で厳密・diff フレンドリ）。
void sil_emu_record_bytes(const char* channel, const uint8_t* data, int len);

// Record a free-form marker/note line (no payload bytes).
// 自由形式のマーカー/注記行を記録（ペイロードなし）。
void sil_emu_record_note(const char* channel, const char* text);

}  // extern "C"
