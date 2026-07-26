#!/usr/bin/env python3
"""
Real-time keyboard-piloted SIL — pacing / RC-over-stdin / determinism (P6 stage 1)
リアルタイム・キーボード操縦SIL — ペーシング/RC-over-stdin/決定論性（P6 stage 1）

Verifies, WITHOUT any manual keyboard interaction, the three properties the P6
stage 1 emulator additions (simulator/sil/devices/emu_realtime.* and
rc_stdin.*, wired into simulator/sil/emu/emu_main.cpp) must hold:

  (a) SIL_EMU_REALTIME=1 paces the virtual clock to the wall clock, within
      tolerance — the whole point of a keyboard-piloted session.
  (b) SIL_EMU_RC_STDIN=1's scripted ARM -> throttle-up sequence produces a
      real altitude climb through the REAL, unmodified firmware state
      machine (StateManager ARM edge -> TAKEOFF -> FLYING/STABILIZE) — the
      same stick sequence scenarios/stab_flight.scn uses.
  (c) With NEITHER env var set, the emulator's output is BYTE-IDENTICAL to
      before this feature existed. This is the absolute non-negotiable:
      "既存の決定論性を絶対に壊さないこと — 通常モードはバイト一致維持が
      絶対条件" (CLAUDE.md). Every hook this feature adds to on_advance() is
      a cached env-var check that is a complete no-op when unset; this test
      is the numerical proof.

手動キーボード操作なしで、P6 stage 1 のemu追加（emu_realtime.*/rc_stdin.*、
emu_main.cppへの配線）が満たすべき3性質を検証する:
  (a) SIL_EMU_REALTIME=1 が仮想時計を壁時計にペーシングする（許容誤差内）—
      キーボード操縦セッションの本質そのもの。
  (b) SIL_EMU_RC_STDIN=1 の台本化ARM→スロットル上げ系列が、無改変の実ファーム
      状態機械（StateManager ARMエッジ→TAKEOFF→FLYING/STABILIZE）を通して
      実際の高度上昇を生む — scenarios/stab_flight.scn と同じスティック系列。
  (c) どちらのenv変数も未設定なら、本機能実装前とbyte-identical。これは絶対
      条件（CLAUDE.md）。on_advance() へ追加した全フックはキャッシュ済み
      env判定で未設定時は完全no-op — 本テストがその数値的証明。

Prerequisite / 事前条件:
    source setup_env.sh && sf sil build
    pytest simulator/tests/test_realtime_fly.py -v
"""

import hashlib
import os
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List

import pytest

from sfcli.utils.paths import paths


def _exe(name: str) -> Path:
    suffix = ".exe" if sys.platform.startswith("win") else ""
    return paths.sil_build() / f"{name}{suffix}"


EMU_VEHICLE = _exe("emu_vehicle")
MODEL = paths.root() / "simulator" / "sil" / "models" / "stampfly.xml"
ACRO_SCN = paths.root() / "simulator" / "sil" / "scenarios" / "acro_flight.scn"

# Captured BEFORE this feature existed (repo HEAD 3d1a35ed, "feat(sil):
# model-match gate", 2026-07-27), via:
#   sf sil scenario simulator/sil/scenarios/acro_flight.scn --target vehicle
#   shasum -a 256 simulator/sil/viz/out_scn_acro_flight/trajectory.csv
# This is the number test (c) below must reproduce exactly with NO new env
# vars set — see the module docstring's item (c).
# 本機能実装前（HEAD 3d1a35ed, 2026-07-27）に採取した基準値。下記(c)は新規
# env変数を一切設定せずにこの値を厳密再現しなければならない — docstring (c) 参照。
ACRO_FLIGHT_BASELINE_SHA256 = (
    "5c913abf585b07f6a7a59f027f6407cb84fca01e2dc2b870f015a0a7649a0b87"
)


def _require_built() -> None:
    if not EMU_VEHICLE.exists():
        pytest.fail(
            f"{EMU_VEHICLE} not built — run 'source setup_env.sh && sf sil build' first"
        )


def _parse_state(line: str) -> Dict[str, object]:
    """Parse one "STATE k=v k=v ..." HUD line into a {key: float|str} dict.

    "STATE k=v k=v ..." のHUD行を {key: float|str} 辞書に変換する。
    """
    fields: Dict[str, object] = {}
    for tok in line.split()[1:]:   # skip the leading "STATE" token
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        if k == "mode":
            fields[k] = v   # e.g. "FLYING:STABILIZE*" — not numeric
            continue
        try:
            fields[k] = float(v)
        except ValueError:
            pass
    return fields


# =============================================================================
# (c) Determinism — env vars unset -> byte-identical to before this feature.
# (c) 決定論性 — env変数未設定 -> 本機能追加前とbyte-identical。
# =============================================================================

def test_determinism_unchanged_without_env_vars(tmp_path):
    _require_built()
    traj = tmp_path / "trajectory.csv"
    env = dict(os.environ)
    env.pop("SIL_EMU_REALTIME", None)   # explicit: this run must NOT opt in
    env.pop("SIL_EMU_RC_STDIN", None)
    env["SIL_EMU_TRAJ"] = str(traj)

    with open(os.devnull) as devnull:
        r = subprocess.run(
            [str(EMU_VEHICLE), str(MODEL), "25000000", str(ACRO_SCN)],
            stdin=devnull, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", env=env, timeout=60,
        )
    assert r.returncode == 0, f"emu_vehicle exited {r.returncode}\n{r.stderr}"
    assert traj.exists(), "trajectory.csv was not written"

    digest = hashlib.sha256(traj.read_bytes()).hexdigest()
    assert digest == ACRO_FLIGHT_BASELINE_SHA256, (
        "acro_flight.scn trajectory.csv changed with NO new env vars set — "
        "the P6 stage 1 realtime/RC-stdin feature broke normal-path "
        f"determinism (got {digest}, expected {ACRO_FLIGHT_BASELINE_SHA256})"
    )


# =============================================================================
# (a) Pacing — SIL_EMU_REALTIME=1 keeps virtual time within wall-clock tolerance.
# (a) ペーシング — SIL_EMU_REALTIME=1 で仮想時間が壁時計の許容誤差内。
# =============================================================================

def test_realtime_pacing_matches_wall_clock():
    _require_built()
    env = dict(os.environ)
    env["SIL_EMU_REALTIME"] = "1"
    virtual_s = 4.0
    duration_us = int(virtual_s * 1e6)

    t0 = time.perf_counter()
    with open(os.devnull) as devnull:
        r = subprocess.run(
            [str(EMU_VEHICLE), str(MODEL), str(duration_us)],
            stdin=devnull, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", env=env, timeout=30,
        )
    wall_s = time.perf_counter() - t0
    assert r.returncode == 0, f"emu_vehicle exited {r.returncode}\n{r.stderr}"

    # Primary signal: whole-process wall time vs the requested virtual duration.
    # 主判定: プロセス全体の壁時計時間 対 要求仮想時間。
    rel_err = abs(wall_s - virtual_s) / virtual_s
    assert rel_err <= 0.20, (
        f"SIL_EMU_REALTIME pacing off by {rel_err:.1%}: wall={wall_s:.3f}s "
        f"vs virtual={virtual_s:.3f}s (want <=20%)"
    )

    # Cross-check against the HUD's OWN virtual-time column (the literal
    # quantity a pilot watches), emitted only in realtime mode at ~30 Hz.
    # 相互確認: HUD自身の仮想時刻列（パイロットが実際に見る量）、realtime限定
    # ~30Hzで出力。
    state_lines = [l for l in r.stdout.splitlines() if l.startswith("STATE ")]
    assert state_lines, "no STATE HUD lines emitted under SIL_EMU_REALTIME=1"
    last_t = _parse_state(state_lines[-1])["t"]
    rel_err_state = abs(float(last_t) - virtual_s) / virtual_s
    assert rel_err_state <= 0.20, (
        f"HUD virtual time off by {rel_err_state:.1%}: last STATE t={last_t}s "
        f"vs requested {virtual_s}s (want <=20%)"
    )


# =============================================================================
# (b) Scripted piloting — stdin ARM -> throttle-up produces a real climb.
# (b) 台本操縦 — stdin ARM→スロットル上げが実際の上昇を生む。
# =============================================================================

def _read_stdout_lines(proc: subprocess.Popen, sink: List[str]) -> None:
    """Background reader so the child's stdout pipe never backs up while the
    main thread is busy sleeping between scripted commands.
    メインスレッドが台本コマンド間でsleepしている間も子のstdout pipeが
    詰まらないようにするバックグラウンド読み取り。
    """
    for line in proc.stdout:   # closes cleanly when the child exits (EOF)
        sink.append(line.rstrip("\n"))


def test_rc_stdin_arm_and_throttle_climbs():
    _require_built()
    env = dict(os.environ)
    env["SIL_EMU_REALTIME"] = "1"
    env["SIL_EMU_RC_STDIN"] = "1"

    proc = subprocess.Popen(
        [str(EMU_VEHICLE), str(MODEL), "12000000"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1, env=env,
    )
    lines: List[str] = []
    reader = threading.Thread(target=_read_stdout_lines, args=(proc, lines), daemon=True)
    reader.start()

    def send(cmd: str) -> None:
        proc.stdin.write(cmd + "\n")
        proc.stdin.flush()

    try:
        # Same stick sequence as scenarios/stab_flight.scn's ARM+takeoff steps
        # (raw ADC, centre 2048; throttle 3473 matches its takeoff-trigger
        # step). The 5 s neutral lead matches the scenario's own comment: boot
        # calibration must reach IDLE_GROUND before an ARM press is accepted.
        # stab_flight.scn と同じスティック系列（raw ADC、中央2048。スロットル
        # 3473は同scnの離陸トリガ値と同じ）。5s の中立リードは同scnの注記どおり
        # — 起動校正がIDLE_GROUNDに達してからでないとARM押下が受理されない。
        send("rc 2048 2048 2048 2048")
        time.sleep(5.0)
        send("arm")                       # momentary ARM press (edge pulse)
        time.sleep(1.5)                   # ARM -> ARMED_GROUND settle
        send("rc 2048 2048 2048 3473")    # throttle up -> TAKEOFF -> FLYING
        time.sleep(3.0)
        send("quit")
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
        pytest.fail("emu_vehicle did not exit after 'quit' within 10s")
    finally:
        if proc.poll() is None:
            proc.kill()
        reader.join(timeout=5)

    assert proc.returncode == 0, f"emu_vehicle exited {proc.returncode}"

    log = "\n".join(lines)
    assert "ARM accepted" in log, "ARM was never accepted by StateManager"
    assert "Takeoff complete" in log, "firmware never reached FLYING"

    state_lines = [l for l in lines if l.startswith("STATE ")]
    assert state_lines, "no STATE HUD lines captured"
    alts = [float(_parse_state(l)["alt"]) for l in state_lines]
    assert max(alts) > 0.3, (
        f"throttle-up via RC-over-stdin never produced a real climb "
        f"(max alt={max(alts):.3f} m)"
    )
