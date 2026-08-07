/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Kouhei Ito
 *
 * Part of StampFly Ecosystem (SILS host bench — StampFly emulator).
 * https://github.com/M5Fly-kanazawa/stampfly_ecosystem
 */

/**
 * @file emu_realtime.hpp
 * @brief Wall-clock pacing for the emulator's virtual clock (P6 stage 1 —
 *        keyboard-piloted SILS). SILS_EMU_REALTIME=1 slows the deterministic
 *        discrete-event scheduler down to real time so a human (or a live
 *        RC-over-stdin session) can fly it; unset, the scheduler runs exactly
 *        as before (as fast as the host CPU allows, byte-identical output).
 *        エミュレータの仮想時計を壁時計に同期させる（P6 stage 1 —
 *        キーボード操縦SILS）。SILS_EMU_REALTIME=1 で決定論的な離散事象
 *        スケジューラを実時間まで減速し、人間（またはRC-over-stdinの
 *        ライブセッション）が操縦できるようにする。未設定なら従来どおり
 *        （ホストCPUが許す限り高速・byte-identical）。
 *
 * Design: ONLY paces the scheduler's on_advance hook (called once per virtual-
 * time jump) with a plain usleep() when the virtual clock has gotten AHEAD of
 * the wall clock. When the virtual clock is BEHIND (a slow host, or a burst of
 * catch-up work), it never sleeps — it simply lets the loop run flat out until
 * the virtual clock catches back up, exactly like real hardware never "hurries"
 * but also never waits for itself. This is a pure ADDITIVE side effect (sleep
 * only), so the cooperative schedule (task order, notifications) — the thing
 * the determinism hash actually protects — is completely unchanged; only wall-
 * clock DURATION differs, which the trace/trajectory never encode.
 *
 * 設計: スケジューラの on_advance フック（仮想時刻が飛ぶたび1回呼ばれる）だけを
 * usleep() でペーシングする。仮想時計が壁時計より「進みすぎ」のときだけ待ち、
 * 「遅れ」のときは一切待たず全速で回して自然に追いつかせる（実機が自分から
 * 急いだり待ったりしないのと同じ）。これは純粋な追加の副作用（sleep のみ）
 * であり、決定論ハッシュが実際に保護している協調スケジュール（タスク順序・
 * 通知）には一切触れない。変わるのは壁時計上の所要時間だけで、
 * トレース/軌跡には記録されない。
 *
 * @design docs/architecture/simulation-policy.md — SILS fidelity / tooling  [--]
 */

#pragma once

#include <cstdint>

extern "C" {

// True iff SILS_EMU_REALTIME is set (checked once, cached). Every other
// function in this header is a no-op when this is false, so the whole
// feature compiles to a single cached env-var check on the default path.
// SILS_EMU_REALTIME が設定されていれば true（初回のみ判定・キャッシュ）。false の
// 間は他の全関数が no-op ＝既定経路はキャッシュ済み env 判定1回のみ。
bool sils_realtime_enabled(void);

// Call once per on_advance(now_us) — sleeps just enough so the virtual clock
// never outruns the wall clock. No-op unless sils_realtime_enabled().
// on_advance(now_us) 毎に呼ぶ — 仮想時計が壁時計を追い越さない分だけ眠る。
// sils_realtime_enabled() が false なら no-op。
void sils_realtime_pace(int64_t now_us);

}  // extern "C"
