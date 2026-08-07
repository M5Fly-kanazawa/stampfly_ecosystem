/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 */

/**
 * @file emu_trajectory.hpp
 * @brief Review-video trajectory recorder for emulator scenario runs.
 *        エミュレータのシナリオ実行を録る、レビュー動画用の軌跡レコーダ。
 *
 * Writes a render_video.py-compatible trajectory.csv (the same 20-column layout
 * hover_smoke emits) so `sf sils` can render a 3D + state-graph review video of an
 * UNMODIFIED-firmware emulator run. Firmware-agnostic: the truth pose and motor
 * duty come from the Plant + virtual board (SILS infra). The estimate columns come
 * from a per-firmware weak hook (sils_emu_estimate) — without one they mirror the
 * truth. Default OFF: an empty/unset path makes every call a no-op, so a normal
 * run is byte-identical to before this feature.
 *
 * render_video.py 互換の trajectory.csv（hover_smoke と同じ20列）を書き出し、無改変
 * ファームのエミュレータ実行を 3D＋状態グラフのレビュー動画にできるようにする。
 * ファーム非依存: 真値姿勢とモータ duty は Plant＋仮想ボード（SILS基盤）から。estimate
 * 列はファーム固有の弱フック（sils_emu_estimate）から（無ければ真値を写す）。既定 OFF:
 * パス未指定なら全呼び出しが no-op ＝ 通常実行は本機能前と byte-identical。
 *
 * @design simulator/sils/RESET_PLAN.md §9 — reproducible review video  [--]
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Open the trajectory CSV at `path` and write the header. A null/empty path keeps
// the recorder closed (every sample/close is then a no-op).
// `path` の軌跡 CSV を開きヘッダを書く。null/空なら閉じたまま（以降 no-op）。
void sils_emu_traj_open(const char* path);

// Record one row at virtual time `t_s` [s] from the Plant (opaque sils::Plant*):
// truth pose + per-firmware estimate + motor duty. Internally decimated to ~50 fps.
// No-op if not open. 仮想時刻 t_s で Plant から1行記録（真値＋推定＋duty）。約50fpsに間引き。
void sils_emu_traj_sample(double t_s, void* plant);

// Flush and close the trajectory file (no-op if not open).
// 軌跡ファイルを flush して閉じる（未オープンなら no-op）。
void sils_emu_traj_close(void);

#ifdef __cplusplus
}  // extern "C"
#endif
