#!/usr/bin/env python3
"""
Real-time keyboard-piloted SILS — pacing / RC-over-stdin / determinism (P6 stage 1)
リアルタイム・キーボード操縦SILS — ペーシング/RC-over-stdin/決定論性（P6 stage 1）

Verifies, WITHOUT any manual keyboard interaction, the three properties the P6
stage 1 emulator additions (simulator/sils/devices/emu_realtime.* and
rc_stdin.*, wired into simulator/sils/emu/emu_main.cpp) must hold:

  (a) SILS_EMU_REALTIME=1 paces the virtual clock to the wall clock, within
      tolerance — the whole point of a keyboard-piloted session.
  (b) SILS_EMU_RC_STDIN=1's scripted ARM -> throttle-up sequence produces a
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
  (a) SILS_EMU_REALTIME=1 が仮想時計を壁時計にペーシングする（許容誤差内）—
      キーボード操縦セッションの本質そのもの。
  (b) SILS_EMU_RC_STDIN=1 の台本化ARM→スロットル上げ系列が、無改変の実ファーム
      状態機械（StateManager ARMエッジ→TAKEOFF→FLYING/STABILIZE）を通して
      実際の高度上昇を生む — scenarios/stab_flight.scn と同じスティック系列。
  (c) どちらのenv変数も未設定なら、本機能実装前とbyte-identical。これは絶対
      条件（CLAUDE.md）。on_advance() へ追加した全フックはキャッシュ済み
      env判定で未設定時は完全no-op — 本テストがその数値的証明。

Prerequisite / 事前条件:
    source setup_env.sh && sf sils build
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
    return paths.sils_build() / f"{name}{suffix}"


EMU_VEHICLE = _exe("emu_vehicle")
MODEL = paths.root() / "simulator" / "sils" / "models" / "stampfly.xml"
ACRO_SCN = paths.root() / "simulator" / "sils" / "scenarios" / "acro_flight.scn"

# Re-captured 2026-09-11 for the StampFly flight-log v1 bundle format
# (docs/plans/flight-log-format-plan.md): the emulator now writes
# SILS_EMU_FLIGHTLOG=<dir> (imu.csv/attitude.csv/.../truth.csv) instead of the
# retired SILS_EMU_TRAJ=<path> single trajectory.csv, so the determinism
# baseline is the sha256 of truth.csv's bytes followed by imu.csv's bytes
# (concatenated in that order — see the hashing code below), via:
#   sf sils scenario simulator/sils/scenarios/acro_flight.scn --target vehicle
#   (finds the flight-log CSVs under the run's own bundle/flightlog/ dir
#    before it is zipped away by _finalize_flightlog())
# This is the number test (c) below must reproduce exactly with NO new env
# vars set — see the module docstring's item (c). This baseline protects the
# SAME property the old trajectory.csv hash protected: with SILS_EMU_REALTIME/
# SILS_EMU_RC_STDIN both unset, the emulator's numerical output must be
# byte-identical to before the P6 stage 1 feature existed.
# 2026-09-11、StampFly フライトログ v1 一式形式（flight-log-format-plan.md）
# 向けに再採取: エミュレータは廃止された SILS_EMU_TRAJ=<path>（単一の
# trajectory.csv）の代わりに SILS_EMU_FLIGHTLOG=<dir>（imu.csv/attitude.csv/
# .../truth.csv）を書くようになったため、決定論性の基準値は truth.csv の
# バイト列に続けて imu.csv のバイト列（この順で連結）の sha256 とする
# （下のハッシュ計算コード参照）。下記(c)は新規env変数を一切設定せずにこの値を
# 厳密再現しなければならない — docstring (c) 参照。この基準値は旧
# trajectory.csv ハッシュが守っていたのと同じ性質（SILS_EMU_REALTIME/
# SILS_EMU_RC_STDIN が両方未設定なら、P6 stage 1 機能追加前とbyte-identical）
# を保護する。
ACRO_FLIGHT_BASELINE_SHA256 = (
    "0c557610394d79f99359e0bd6463197aee002670f320c29ab7ec271b30a414eb"
)


def _require_built() -> None:
    if not EMU_VEHICLE.exists():
        pytest.fail(
            f"{EMU_VEHICLE} not built — run 'source setup_env.sh && sf sils build' first"
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
    flightlog_dir = tmp_path / "flightlog"
    env = dict(os.environ)
    env.pop("SILS_EMU_REALTIME", None)   # explicit: this run must NOT opt in
    env.pop("SILS_EMU_RC_STDIN", None)
    env["SILS_EMU_FLIGHTLOG"] = str(flightlog_dir)

    with open(os.devnull) as devnull:
        r = subprocess.run(
            [str(EMU_VEHICLE), str(MODEL), "25000000", str(ACRO_SCN)],
            stdin=devnull, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", errors="replace", env=env, timeout=60,
        )
    assert r.returncode == 0, f"emu_vehicle exited {r.returncode}\n{r.stderr}"
    truth_csv = flightlog_dir / "truth.csv"
    imu_csv = flightlog_dir / "imu.csv"
    assert truth_csv.exists(), "truth.csv was not written"
    assert imu_csv.exists(), "imu.csv was not written"

    # Hash truth.csv's bytes followed by imu.csv's bytes (that order) — the
    # two flight-log streams that replace the old single trajectory.csv this
    # baseline used to hash (see ACRO_FLIGHT_BASELINE_SHA256's comment).
    # truth.csv のバイト列に続けて imu.csv のバイト列（この順）をハッシュする
    # — 旧単一 trajectory.csv を置き換えた2つのフライトログストリーム
    # （ACRO_FLIGHT_BASELINE_SHA256 のコメント参照）。
    digest = hashlib.sha256(truth_csv.read_bytes() + imu_csv.read_bytes()).hexdigest()
    assert digest == ACRO_FLIGHT_BASELINE_SHA256, (
        "acro_flight.scn's truth.csv+imu.csv changed with NO new env vars set — "
        "the P6 stage 1 realtime/RC-stdin feature broke normal-path "
        f"determinism (got {digest}, expected {ACRO_FLIGHT_BASELINE_SHA256})"
    )


# =============================================================================
# (a) Pacing — SILS_EMU_REALTIME=1 keeps virtual time within wall-clock tolerance.
# (a) ペーシング — SILS_EMU_REALTIME=1 で仮想時間が壁時計の許容誤差内。
# =============================================================================

def test_realtime_pacing_matches_wall_clock():
    _require_built()
    env = dict(os.environ)
    env["SILS_EMU_REALTIME"] = "1"
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
        f"SILS_EMU_REALTIME pacing off by {rel_err:.1%}: wall={wall_s:.3f}s "
        f"vs virtual={virtual_s:.3f}s (want <=20%)"
    )

    # Cross-check against the HUD's OWN virtual-time column (the literal
    # quantity a pilot watches), emitted only in realtime mode at ~30 Hz.
    # 相互確認: HUD自身の仮想時刻列（パイロットが実際に見る量）、realtime限定
    # ~30Hzで出力。
    state_lines = [l for l in r.stdout.splitlines() if l.startswith("STATE ")]
    assert state_lines, "no STATE HUD lines emitted under SILS_EMU_REALTIME=1"
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
    env["SILS_EMU_REALTIME"] = "1"
    env["SILS_EMU_RC_STDIN"] = "1"

    proc = subprocess.Popen(
        [str(EMU_VEHICLE), str(MODEL), "12000000"],
        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, encoding="utf-8", errors="replace", bufsize=1, env=env,
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
