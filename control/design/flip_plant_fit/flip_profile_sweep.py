#!/usr/bin/env python3
"""Flip rate-profile sweep harness: run SILS flip scenarios and score each flip.
宙返りレートプロファイルの掃引プログラム: SILS の宙返りシナリオを走らせ、1 回ごとに採点する。

Evidence for docs/plans/flip-maneuver-plan.md section 5.7 (step 5, 2026-10-02). Analysis
helper only (not a user tool); the user-facing entry is `sf sils scenario`.
flip-maneuver-plan.md 5.7 節（ステップ 5、2026-10-02）の根拠。解析補助のみ。

A case = (axis, plant profile, firmware params, plant knobs, battery SoC, start height).
It copies simulator/sils/scenarios/api_flip_{roll,pitch}.scn under a unique name (so cases run in
parallel, each with its own bundle directory), runs `sf sils scenario`, and scores every flip
from the flight-log bundle (truth.csv, flight_phase.csv, ctrl_output.csv, rate_ref.csv, imu.csv).
ケース = (軸, プラントプロファイル, ファームのパラメータ, プラントのノブ, 電池 SoC, 開始高度)。
シナリオを固有名でコピーして並列実行し、フライトログから 1 回ごとに採点する。

Metrics per flip / 1 回ごとの指標:
  rot_deg      true rotation about the flip axis at the Brake -> Recover handoff (target 360 +- 15)
  peak_dps / overshoot_pct   true peak rate, and its excess over the commanded peak
  sat_frac     fraction of Spin+Brake samples with |torque| at the 7 mNm limit
  err_max_dps  max |rate_ref - gyro| in Spin+Brake
  alt_drop_m   start height minus the lowest height until 1 s after Done
  tilt_done_deg true tilt when the sequence reaches Done
  t_done_s     Spin start -> Done
  ok           result Ok, |rot - 360| <= 15, tilt_done <= 20 deg, no floor contact (height > 0.3 m)
"""
import concurrent.futures as cf
import glob
import os
import re
import shutil
import subprocess
import tempfile
import zipfile

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
SCN_DIR = os.path.join(REPO, "simulator", "sils", "scenarios")
VIZ_DIR = os.path.join(REPO, "simulator", "sils", "viz")
TORQUE_LIMIT = 7.0e-3
PHASE_SPIN, PHASE_BRAKE, PHASE_RECOVER, PHASE_DONE = 2, 3, 4, 5
AXIS_FILE = {"roll": "api_flip_roll", "pitch": "api_flip_pitch"}
DEFAULT_UP_CM = 60  # api_flip_*.scn climbs "up 60" from the 0.5 m takeoff hover


def _scenario_text(axis, up_cm):
    """Scenario text with the climb command replaced. / 上昇コマンドを置換したシナリオ。"""
    text = open(os.path.join(SCN_DIR, AXIS_FILE[axis] + ".scn"), encoding="utf-8").read()
    return text.replace('"up 60"', f'"up {up_cm}"')


def _duration_us(axis):
    run = re.search(r"--duration (\d+)", open(os.path.join(SCN_DIR, AXIS_FILE[axis] + ".scn")).read())
    return run.group(1)


def run_case(case):
    """Run one case; return (list of per-flip metric dicts, log) / 1 ケースを実行して採点する。

    case keys: name, axis ("roll"|"pitch"), profile ("default"|"flip_fit_2026_10"),
               params {name: value}, plant {"torque_authority","motor_delay","motor_slew"} (optional),
               soc (optional), up_cm (optional)
    """
    name, axis = case["name"], case["axis"]
    scn = os.path.join(SCN_DIR, f"{name}.scn")
    try:
        open(scn, "w", encoding="utf-8").write(_scenario_text(axis, case.get("up_cm", DEFAULT_UP_CM)))
        cmd = ["sf", "sils", "scenario", scn, "--target", "vehicle", "--duration", _duration_us(axis),
               "--param", "flip.enable=1"]
        for key, value in case.get("params", {}).items():
            cmd += ["--param", f"{key}={value}"]
        cmd += ["--plant-profile", case.get("profile", "default")]
        flags = {"torque_authority": "--torque-authority", "motor_delay": "--motor-delay",
                 "motor_slew": "--motor-slew"}
        for key, value in case.get("plant", {}).items():
            cmd += [flags[key], str(value)]
        env = dict(os.environ)
        if "soc" in case:
            env["SILS_EMU_BATT_INITIAL_FRAC"] = str(case["soc"])
        proc = subprocess.run(cmd, capture_output=True, text=True, env=env)
        bundle = os.path.join(VIZ_DIR, f"out_scn_{name}")
        zips = sorted(glob.glob(os.path.join(bundle, "*.sflog.zip")))
        if not zips:
            return [], proc.stdout[-2000:] + proc.stderr[-2000:]
        flips = score_bundle(zips[-1], axis)
        for f in flips:
            f["case"] = name
        return flips, ""
    finally:
        if os.path.exists(scn):
            os.remove(scn)
        shutil.rmtree(os.path.join(VIZ_DIR, f"out_scn_{name}"), ignore_errors=True)


def score_bundle(path, axis):
    """Score every flip in a SILS flight-log bundle. / バンドル内の全宙返りを採点する。"""
    d = tempfile.mkdtemp()
    try:
        zipfile.ZipFile(path).extractall(d)
        rd = lambda n: np.genfromtxt(os.path.join(d, n + ".csv"), delimiter=",", names=True)
        fp, tr, co, rr, im, st = (rd(n) for n in ("flight_phase", "truth", "ctrl_output", "rate_ref", "imu", "status"))
    finally:
        shutil.rmtree(d, ignore_errors=True)
    ax = 0 if axis == "roll" else 1
    t_fp = fp["timestamp_us"] / 1e6
    phase = fp["flip_phase"].astype(int)
    t_tr = tr["timestamp_us"] / 1e6
    rate_true = np.degrees(tr["rate_x" if ax == 0 else "rate_y"])
    height = -tr["pos_z"]  # truth is NED (z down) / 真値は NED（z 下向き）
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (tr["quat_x"] ** 2 + tr["quat_y"] ** 2), -1, 1)))
    t_co = co["timestamp_us"] / 1e6
    tq = co["torque_roll" if ax == 0 else "torque_pitch"]
    t_rr = rr["timestamp_us"] / 1e6
    ref = np.degrees(rr["rate_ref_roll" if ax == 0 else "rate_ref_pitch"])
    t_im = im["timestamp_us"] / 1e6
    gyro = np.degrees(im["gyro_x" if ax == 0 else "gyro_y"])

    out = []
    edges = np.where(np.diff(phase) != 0)[0] + 1
    starts = [i for i in edges if phase[i] == PHASE_SPIN]
    for i0 in starts:
        t_spin = t_fp[i0]
        t_brake = next((t_fp[i] for i in edges if i > i0 and phase[i] == PHASE_BRAKE), None)
        t_recover = next((t_fp[i] for i in edges if i > i0 and phase[i] == PHASE_RECOVER), None)
        j_done = next((i for i in edges if i > i0 and phase[i] == PHASE_DONE), None)
        if t_recover is None or j_done is None:
            out.append({"axis": axis, "ok": False, "note": "no recover/done"})
            continue
        t_done = t_fp[j_done]
        result = int(fp["flip_result"][j_done])
        w = (t_tr >= t_spin) & (t_tr <= t_recover)
        dt = np.median(np.diff(t_tr))
        sign = 1.0 if np.sum(rate_true[w]) > 0 else -1.0
        rot = float(np.sum(sign * rate_true[w]) * dt)
        peak = float(np.max(sign * rate_true[w]))
        w_ref = (t_rr >= t_spin) & (t_rr <= t_recover)
        peak_cmd = float(np.max(np.abs(ref[w_ref])))
        w_co = (t_co >= t_spin) & (t_co <= t_recover)
        sat = float(np.mean(np.abs(tq[w_co]) >= 0.99 * TORQUE_LIMIT)) if w_co.any() else float("nan")
        g_at = np.interp(t_rr[w_ref], t_im, gyro)
        err = float(np.max(np.abs(ref[w_ref] - g_at)))
        h0 = float(np.interp(t_fp[i0] - 0.15, t_tr, height))
        w_h = (t_tr >= t_spin) & (t_tr <= t_done + 1.0)
        drop = h0 - float(np.min(height[w_h]))
        tilt_done = float(np.interp(t_done, t_tr, tilt))
        floor = float(np.min(height[w_h])) < 0.3
        out.append({
            "axis": axis, "dir": "+" if sign > 0 else "-", "rot_deg": rot, "peak_dps": peak,
            "overshoot_pct": 100.0 * (peak - peak_cmd) / peak_cmd, "sat_frac": sat, "err_max_dps": err,
            "alt_drop_m": drop, "tilt_done_deg": tilt_done, "t_done_s": t_done - t_spin,
            "t_spin_s": t_spin, "result": result, "h0_m": h0,
            "volt_v": float(np.interp(t_spin, st["timestamp_us"] / 1e6, st["voltage"])),
            "ok": bool(result == 1 and abs(rot - 360.0) <= 15.0 and tilt_done <= 20.0 and not floor),
        })
    return out


def run_cases(cases, workers=8):
    """Run cases in parallel; return {name: (flips, log)}. / ケースを並列実行する。"""
    results = {}
    with cf.ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case, c): c["name"] for c in cases}
        for fut in cf.as_completed(futures):
            results[futures[fut]] = fut.result()
    return results


def fmt_row(label, flips):
    """One table row per case (worst flip of the two directions). / 2 方向の最悪値で 1 行。"""
    if not flips or "rot_deg" not in flips[0]:
        return f"{label:44s} NO DATA"
    f = flips
    return (f"{label:44s} ok={'/'.join('Y' if x['ok'] else 'N' for x in f)}  "
            f"rot={'/'.join(f'{x['rot_deg']:.0f}' for x in f)}  "
            f"peak={'/'.join(f'{x['peak_dps']:.0f}' for x in f)}  "
            f"os%={'/'.join(f'{x['overshoot_pct']:.0f}' for x in f)}  "
            f"sat={'/'.join(f'{x['sat_frac']:.2f}' for x in f)}  "
            f"err={max(x['err_max_dps'] for x in f):.0f}  "
            f"drop={max(x['alt_drop_m'] for x in f):.2f}  "
            f"tilt={max(x['tilt_done_deg'] for x in f):.1f}  "
            f"tdone={max(x['t_done_s'] for x in f):.2f}  res={'/'.join(str(x['result']) for x in f)}")


# Robustness conditions on the fitted plant (same as flip_profile_compare.py).
# ロバスト性の条件（フィット済みプラント、flip_profile_compare.py と同じ）。
ROBUST = {
    "ta0.5": {"plant": {"torque_authority": 0.5}}, "ta0.7": {"plant": {"torque_authority": 0.7}},
    "md0": {"plant": {"motor_delay": 0}}, "md12": {"plant": {"motor_delay": 12}},
    "ms12": {"plant": {"motor_slew": 12}}, "ms24": {"plant": {"motor_slew": 24}},
    "soc0.85": {"soc": 0.85}, "soc0.95": {"soc": 0.95},
    "up70": {"up_cm": 70}, "up80": {"up_cm": 80},
    "weak": {"plant": {"torque_authority": 0.5, "motor_delay": 12, "motor_slew": 12}},
    "strong": {"plant": {"torque_authority": 0.7, "motor_delay": 0, "motor_slew": 24}},
    "pitch_weak_ta0.3": {"plant": {"torque_authority": 0.3}},
}


def check_defaults(out_path, params=None, workers=12):
    """Score the firmware DEFAULT flip parameters (or `params` overrides): nominal on the fitted and
    the default plant, then the robustness sweep on the fitted plant. Writes a text table.
    ファームの既定パラメータ（または上書き `params`）を採点する: フィット済み・既定の両プラントで
    公称、フィット済みプラントでロバスト性掃引。表をテキストに書く。"""
    params = params or {}
    cases, n = [], 0
    for profile in ("flip_fit_2026_10", "default"):
        for axis in ("roll", "pitch"):
            n += 1
            cases.append(dict(name=f"chk_{n}", axis=axis, profile=profile, params=params,
                              label=(profile, "nominal", axis)))
    for rname, kw in ROBUST.items():
        for axis in ("roll", "pitch"):
            n += 1
            cases.append(dict(name=f"chk_{n}", axis=axis, profile="flip_fit_2026_10", params=params,
                              label=("flip_fit_2026_10", rname, axis), **kw))
    results = run_cases(cases, workers=workers)
    lines = [f"firmware params override: {params or 'none (defaults)'}",
             "ok = Ok result, rotation 360+-15 at the Brake->Recover handoff, tilt at Done <= 20 deg, "
             "no floor contact; values are per direction (+/-)", ""]
    for c in cases:
        lines.append(fmt_row("%s %s %s" % c["label"], results[c["name"]][0]))
    text = "\n".join(lines) + "\n"
    open(out_path, "w", encoding="utf-8").write(text)
    return text


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    print(check_defaults(os.path.join(here, "flip_profile_default_check.txt")))
