/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_flightlog.hpp
 * @brief Firmware-agnostic "StampFly flight-log v1 bundle" CSV writer for emulator
 *        scenario runs. Replaces emu_trajectory.cpp (review video) and
 *        emu_rate_stream.cpp (model-match gate): both are superseded by the
 *        general-purpose bundle every consumer (viz / sysid / gate / video) now
 *        reads.
 *        エミュレータ実行を「StampFly フライトログ v1 一式」の CSV 群として書き出す、
 *        ファーム非依存のレコーダ。emu_trajectory.cpp（レビュー動画）と
 *        emu_rate_stream.cpp（モデル一致ゲート）を置き換える — どちらも、全ての
 *        消費側（可視化・同定・ゲート・動画）が今後読む汎用一式に統合される。
 *
 * Format: protocol/spec/flight_log.yaml (SSOT) / 決定文書:
 * docs/plans/flight-log-format-plan.md §3.3.
 *
 * Architecture: this file owns the container (open/close, per-stream lazy CSV
 * files) and samples the FIRMWARE-AGNOSTIC physics truth (truth.csv) from the
 * opaque sils::Plant*. It does NOT know about any firmware's pub-sub topics —
 * that is the job of a per-firmware glue (e.g. devices/emu_flightlog_vehicle.cpp)
 * that overrides the weak `sils_emu_flightlog_firmware_sample` hook, exactly the
 * same weak-hook pattern emu_trajectory.cpp used for `sils_emu_estimate`. A
 * firmware with no glue (none yet) still links and produces truth.csv only.
 * アーキテクチャ: 本体は容器（open/close、ストリーム毎の遅延オープン CSV）を持ち、
 * 不透明な sils::Plant* からファーム非依存の物理真値（truth.csv）を採取する。どの
 * ファームの Pub-Sub トピックも知らない — それは弱フック
 * `sils_emu_flightlog_firmware_sample` を上書きするファーム固有 glue
 * （例: devices/emu_flightlog_vehicle.cpp）の仕事（emu_trajectory.cpp が
 * `sils_emu_estimate` に使ったのと同じ弱フック方式）。glue の無いファームでも
 * リンクでき、truth.csv だけを出す。
 *
 * Default OFF: sils_emu_flightlog_open(nullptr/"") keeps every call below a
 * no-op, so a normal run is byte-identical to before this feature (same
 * discipline as emu_trajectory/emu_rate_stream).
 * 既定 OFF: 空/null パスなら以降の全呼び出しが no-op ＝ 通常実行は本機能前と
 * byte-identical（emu_trajectory/emu_rate_stream と同じ規律）。
 *
 * @design docs/plans/flight-log-format-plan.md §3.3 — SILS/vehicle unification [--]
 */

#pragma once

#include <cstdint>
#include <cstdio>

#ifdef __cplusplus
extern "C" {
#endif

// Open the flight-log bundle directory at `dir` (created if missing). A
// null/empty `dir` keeps the recorder closed — every call below then becomes
// a no-op, matching the emu_trajectory/emu_rate_stream default-OFF discipline.
// `dir` にフライトログ一式ディレクトリを開く（無ければ作成）。null/空なら
// 閉じたまま — 以降の全呼び出しが no-op（emu_trajectory/emu_rate_stream と
// 同じ既定 OFF の規律）。
void sils_emu_flightlog_open(const char* dir);

// Look up (or lazily open) the named CSV stream inside the open bundle
// directory: "<dir>/<name>.csv". On the FIRST call for a given `name`, opens
// the file, writes `header` + '\n', and remembers it in a small fixed table —
// so a stream that is never requested never creates a file (the bundle format
// simply omits streams with no packets). Returns nullptr when the recorder is
// not open (no-op) or the stream table is exhausted.
// 開いた一式ディレクトリ内の名前付き CSV ストリーム "<dir>/<name>.csv" を検索
// （無ければ遅延オープン）。ある `name` への最初の呼び出しでファイルを開き
// `header` を書き、固定サイズの小テーブルに憶える — 一度も要求されない
// ストリームはファイルを作らない（一式形式はパケットの無いストリームを
// 単に省く）。レコーダ未オープン（no-op）またはテーブル枯渇なら nullptr。
std::FILE* sils_emu_flightlog_stream(const char* name, const char* header);

// Advance hook: called once per virtual-clock step from emu_main's on_advance.
// Writes one truth.csv row at a fixed ~400Hz virtual cadence from the opaque
// sils::Plant* (nullptr skips truth sampling — used by targets with no Plant),
// then calls the weak per-firmware hook below. No-op if not open.
// advance フック: emu_main の on_advance から仮想クロック 1 ステップに 1 回呼ぶ。
// 不透明な sils::Plant* から約400Hz固定間隔で truth.csv を1行記録
// （nullptr なら truth 採取をスキップ — Plant を持たないターゲット用）、続けて
// 下の弱いファーム固有フックを呼ぶ。未オープンなら no-op。
void sils_emu_flightlog_sample(int64_t now_us, const void* plant);

// Flush and close every stream file opened since sils_emu_flightlog_open()
// (no-op if not open).
// sils_emu_flightlog_open() 以降に開いた全ストリームを flush して閉じる
// （未オープンなら no-op）。
void sils_emu_flightlog_close(void);

// Per-firmware hook, called once per sils_emu_flightlog_sample() call. WEAK
// no-op default defined in emu_flightlog.cpp (a weak DEFINITION, not just a
// declaration, so the reference always resolves on macOS/clang as well as ELF
// linkers — same trick emu_trajectory.cpp used for sils_emu_estimate). A
// firmware-specific glue (devices/emu_flightlog_vehicle.cpp) provides a STRONG
// override that reads that firmware's own published topics with `.latest()`
// (never `.read()`, which would steal samples from the real consumer) and
// writes the vehicle-topic streams (imu/attitude/posvel/rate_ref/motor/
// ctrl_output/ctrl_ref/pilot/status/baro/tof_bottom/flow/mag).
// ファーム固有フック。sils_emu_flightlog_sample() 呼び出し毎に1回呼ばれる。既定は
// emu_flightlog.cpp で定義する弱い no-op（弱「定義」— macOS/clang でも ELF でも
// 参照が必ず解決する。emu_trajectory.cpp が sils_emu_estimate に使ったのと同じ
// 手法）。ファーム固有 glue（devices/emu_flightlog_vehicle.cpp）が強い定義で
// そのファーム自身の発行トピックを `.latest()`（`.read()` は実消費者からサンプルを
// 横取りするため使わない）で読み、vehicle トピック系ストリームを書く。
void sils_emu_flightlog_firmware_sample(int64_t now_us);

// Per-firmware hook: dump the LIVE rate-loop PID gains to "<dir>/gains.json"
// (the former sils_emu_rate_write_gains()). WEAK no-op default here so
// emu_main_generic.cpp (vehicle_old, which has no rate-loop gains to dump)
// links without a strong override. Call AFTER app_main() has loaded params
// AND after any SILS_EMU_PARAMS_FILE override, so the sidecar reflects the
// gains the run actually flew.
// ファーム固有フック: 実行時の実ゲイン（rate-loop PID）を "<dir>/gains.json" へ
// 書く（旧 sils_emu_rate_write_gains() 相当）。ここでは弱い no-op 既定とし、
// emu_main_generic.cpp（ダンプすべきレートループゲインを持たない vehicle_old）は
// 強い上書き無しでリンクできる。app_main() の param ロード後、かつ
// SILS_EMU_PARAMS_FILE 上書き適用後に呼ぶこと — sidecar は実際に飛んだゲインを
// 反映する必要がある。
void sils_emu_flightlog_write_gains(void);

#ifdef __cplusplus
}  // extern "C"
#endif
