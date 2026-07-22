/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SIL host bench — StampFly emulator).
 */

/**
 * @file emu_rate_stream.hpp
 * @brief 400Hz rate-loop (rate_ref + gyro) recorder for the model-match gate.
 *        モデル一致ゲート用の400Hzレートループ（rate_ref+gyro）レコーダ。
 *
 * Code Identity lets the SAME identification code used on real-hardware logs
 * (tools/log_analyzer/rate_sysid.py) run on a SIL-generated log too. That code
 * expects a 400Hz CSV with columns rate_ref_roll/pitch/yaw + gyro_x/y/z (an
 * optional timestamp column is also read). This recorder writes exactly that
 * CSV from the REAL firmware's own topics — sf::control_output (rate_ref, the
 * inner-loop setpoint the controller is tracking) and sf::sensor_imu (gyro, the
 * measured value the firmware itself sees, noise included when noise is on).
 *
 * Code Identity により、実機ログの同定に使ったのと同じコード（rate_sysid.py）を SIL
 * 生成ログにもそのまま適用できる。そのコードは rate_ref_roll/pitch/yaw + gyro_x/y/z の
 * 400Hz CSV（timestamp 列は任意）を読む。本レコーダは実ファーム自身のトピック —
 * sf::control_output（レート内側ループの目標値）と sf::sensor_imu（ファーム自身が見て
 * いる測定ジャイロ。ノイズ有効時はノイズ込み）— からそのまま同じ CSV を書き出す。
 *
 * Both topics are read via `.latest()` (a non-destructive peek), NEVER `.read()`
 * (which pops the shared single-consumer ring and would race with the real
 * consumer — DataStream::update() already drains sf::log_stream at 50Hz, and the
 * estimator drains sf::sensor_imu; stealing entries from either would corrupt
 * the very system under test). New samples are found by an edge-detect on
 * ControlOutput::timestamp so the recorder never depends on how often the host
 * scheduler happens to call the sample hook.
 *
 * 両トピックとも `.latest()`（非破壊 peek）でのみ読み、`.read()`（単一消費者の共有
 * リングを消費 — 実消費者と競合する。DataStream::update() は既に log_stream を50Hzで
 * 排出しており、推定器は sensor_imu を排出している。横取りすれば被試験系そのものが
 * 壊れる）は使わない。新サンプルの検出は ControlOutput::timestamp のエッジ検出で行い、
 * ホストスケジューラがサンプルフックを呼ぶ頻度に依存しない。
 *
 * Default OFF: an empty/unset path makes every call a no-op, so a normal run is
 * byte-identical to before this feature (same discipline as emu_trajectory).
 * 既定 OFF: パス未指定なら全呼び出しが no-op ＝ 通常実行は本機能前と byte-identical
 * （emu_trajectory と同じ規律）。
 *
 * @design docs/architecture/simulation-policy.md §4 — model-match gate  [--]
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Open the rate-stream CSV at `path` and write its header. A null/empty path
// keeps the recorder closed (every call below then becomes a no-op).
// `path` のレート CSV を開きヘッダを書く。null/空なら閉じたまま（以降 no-op）。
void sil_emu_rate_open(const char* path);

// Read the LIVE rate-loop gains (SSOT params, post any GUI/env override) and
// write them to the `<path>.gains.json` sidecar. Call AFTER app_main() has
// loaded params AND after any SIL_EMU_PARAMS_FILE override has been applied,
// so the sidecar reflects the gains the run actually flew — the replay in
// `sf sil sysid-gate` needs the SAME gains to reconstruct the loop's torque
// output. No-op if the recorder is not open.
// 実行時の実ゲイン（SSOT params、GUI/env 上書き後）を読み `<path>.gains.json` に書く。
// app_main() の param ロード後、かつ SIL_EMU_PARAMS_FILE 上書き適用後に呼ぶこと —
// sidecar は実際に飛んだゲインを反映する必要がある（sf sil sysid-gate の再生に必須）。
// レコーダ未オープンなら no-op。
void sil_emu_rate_write_gains(void);

// Sample once per NEW control-loop cycle (edge-detected on control_output's
// timestamp), called from the emu_main advance hook. Safe to call more often
// than the control period — a repeat call before the next cycle is a no-op.
// 新しい制御周期ごとに1回サンプリング（control_output のタイムスタンプでエッジ検出）。
// emu_main の advance フックから呼ぶ。制御周期より高頻度に呼んでも、次周期までの
// 再呼び出しは no-op で安全。
void sil_emu_rate_sample(void);

// Flush and close the CSV (no-op if not open).
// CSV を flush して閉じる（未オープンなら no-op）。
void sil_emu_rate_close(void);

#ifdef __cplusplus
}  // extern "C"
#endif
