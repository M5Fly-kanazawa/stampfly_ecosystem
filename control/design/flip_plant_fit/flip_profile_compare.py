#!/usr/bin/env python3
"""Compare the flip rate-profile candidates on the SILS (nominal + robustness sweep).
宙返りレートプロファイルの候補を SILS で比較する（公称 + 頑健性掃引）。

Evidence for docs/plans/flip-maneuver-plan.md section 5.7. Needs the firmware of commit
"feat(flip): planned 360 deg rate profile with optional feedforward" (it still has
flip.profile_mode / flip.ff_gain / flip.motor_lag_ms / flip.brake_margin; the losing modes were
removed afterwards, the result is kept in flip_profile_comparison.txt).
Usage: python3 flip_profile_compare.py [--out flip_profile_comparison.txt]   (after `sf sils build`)
flip-maneuver-plan.md 5.7 節の根拠。負けた方式を後で削除したため、再実行にはコミット
"feat(flip): planned 360 deg rate profile with optional feedforward" のファームが要る。結果は
flip_profile_comparison.txt に残してある。

Candidates / 候補:
  B0  closed-loop trapezoid, main defaults before step 5 (ramp 300/500 rad/s^2, look-ahead 24 ms)
  C1  (1) closed-loop trapezoid retuned (ramp 200/250 rad/s^2, look-ahead 24 ms)
  C2  (2) planned 360 deg profile, ramps 145/145 rad/s^2, peak limit 1500 (roll) / 1400 (pitch)
  C3  (3) = C2 + feedforward I*d(rate_cmd)/dt (flip.ff_gain 1)
  C4  (4) = C1 + feedforward
  TRI factory-like planned triangle: ramps 145/145, peak = sqrt(2*pi*145) = 1729 deg/s
Robustness sweep on the fitted plant (flip_fit_2026_10): torque_authority {0.5, 0.7},
motor_delay {0, 12} ms, motor_slew {12, 24} duty/s, battery SoC {0.85, 0.95}, start height
{1.2, 1.3} m, a weak corner (0.5, 12 ms, 12) and a strong corner (0.7, 0, 24), roll and pitch.
The SILS ToF model reports no target above 1.4 m, so 1.5 m cannot be flown; the battery gate
(flip.min_voltage_v 3.6 V) blocks the flip below SoC ~0.8 (hover terminal voltage ~3.77 V at
full charge; the SILS battery R_int is provisional).
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import flip_profile_sweep as S  # noqa: E402

PLANNED = {"flip.profile_mode": 1, "flip.rate_ramp_rps2": 145, "flip.brake_ramp_rps2": 145,
           "flip.handoff_force_deg": 360}
TRAPEZOID = {"flip.profile_mode": 0, "flip.rate_ramp_rps2": 300, "flip.brake_ramp_rps2": 500,
             "flip.motor_lag_ms": 24, "flip.handoff_force_deg": 350}
CANDIDATES = {
    "B0 trapezoid (main before step 5)": dict(TRAPEZOID),
    "C1 (1) trapezoid retuned": {**TRAPEZOID, "flip.rate_ramp_rps2": 200, "flip.brake_ramp_rps2": 250},
    "C2 (2) planned 360 deg": dict(PLANNED),
    "C3 (3) planned + feedforward": {**PLANNED, "flip.ff_gain": 1.0},
    "C4 (4) trapezoid retuned + ff": {**TRAPEZOID, "flip.rate_ramp_rps2": 200,
                                      "flip.brake_ramp_rps2": 250, "flip.ff_gain": 1.0},
    "TRI factory-like triangle": {**PLANNED, "flip.rate_roll_dps": 1900, "flip.rate_pitch_dps": 1900},
}
ROBUST = {
    "ta0.5": {"plant": {"torque_authority": 0.5}}, "ta0.7": {"plant": {"torque_authority": 0.7}},
    "md0": {"plant": {"motor_delay": 0}}, "md12": {"plant": {"motor_delay": 12}},
    "ms12": {"plant": {"motor_slew": 12}}, "ms24": {"plant": {"motor_slew": 24}},
    "soc0.85": {"soc": 0.85}, "soc0.95": {"soc": 0.95},
    "up70": {"up_cm": 70}, "up80": {"up_cm": 80},
    "weak": {"plant": {"torque_authority": 0.5, "motor_delay": 12, "motor_slew": 12}},
    "strong": {"plant": {"torque_authority": 0.7, "motor_delay": 0, "motor_slew": 24}},
}


def worst(flips, key, fn=max):
    values = [f[key] for f in flips if key in f]
    return fn(values) if values else float("nan")


def success(flips):
    return len(flips) == 2 and all(f.get("ok") for f in flips)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                  "flip_profile_comparison.txt"))
    ap.add_argument("--workers", type=int, default=12)
    args = ap.parse_args()

    cases, n = [], 0
    for cname, params in CANDIDATES.items():
        for profile in ("flip_fit_2026_10", "default"):
            for axis in ("roll", "pitch"):
                n += 1
                cases.append(dict(name=f"cmp_{n}", axis=axis, profile=profile, params=params,
                                  label=(cname, profile, "nominal", axis)))
        for rname, kw in ROBUST.items():
            for axis in ("roll", "pitch"):
                n += 1
                cases.append(dict(name=f"cmp_{n}", axis=axis, profile="flip_fit_2026_10", params=params,
                                  label=(cname, "flip_fit_2026_10", rname, axis), **kw))
    results = S.run_cases(cases, workers=args.workers)
    by = {c["label"]: results[c["name"]][0] for c in cases}

    lines = []
    for profile, title in (("flip_fit_2026_10", "FITTED plant (flip_fit_2026_10), nominal"),
                           ("default", "DEFAULT plant (pre-fit), nominal")):
        lines.append(f"## {title}: worst of the two directions; success = Ok, rotation 360+-15, "
                     "tilt at Done <= 20 deg, no floor contact")
        lines.append(f"{'candidate':34s} {'axis':5s} ok  rot[deg] peak[dps] os%  sat   err[dps] drop[m] "
                     "tilt[deg] t_done[s]")
        for cname in CANDIDATES:
            for axis in ("roll", "pitch"):
                f = by[(cname, profile, "nominal", axis)]
                if not f or "rot_deg" not in f[0]:
                    lines.append(f"{cname:34s} {axis:5s} N   (no flip completed)")
                    continue
                lines.append(
                    f"{cname:34s} {axis:5s} {'Y' if success(f) else 'N'}   "
                    f"{min(x['rot_deg'] for x in f):4.0f}-{max(x['rot_deg'] for x in f):4.0f}  "
                    f"{worst(f, 'peak_dps'):7.0f}   {worst(f, 'overshoot_pct'):3.0f}  "
                    f"{worst(f, 'sat_frac'):4.2f}  {worst(f, 'err_max_dps'):7.0f}  "
                    f"{worst(f, 'alt_drop_m'):6.2f}  {worst(f, 'tilt_done_deg'):7.1f}   "
                    f"{worst(f, 't_done_s'):6.2f}")
        lines.append("")
    lines.append("## Robustness on the fitted plant (12 conditions x roll/pitch = 24 runs of 2 flips each)")
    lines.append(f"{'candidate':34s} pass/24  worst drop[m]  worst tilt[deg]  worst sat  max os%  failed conditions")
    for cname in CANDIDATES:
        runs = [(r, a, by[(cname, "flip_fit_2026_10", r, a)]) for r in ROBUST for a in ("roll", "pitch")]
        failed = [f"{r}/{a}" for r, a, f in runs if not success(f)]
        done = [f for _, _, f in runs if f and "rot_deg" in f[0]]
        lines.append(
            f"{cname:34s} {len(runs) - len(failed):3d}/24   "
            f"{max(worst(f, 'alt_drop_m') for f in done):10.2f}   "
            f"{max(worst(f, 'tilt_done_deg') for f in done):12.1f}   "
            f"{max(worst(f, 'sat_frac') for f in done):8.2f}   "
            f"{max(worst(f, 'overshoot_pct') for f in done):6.0f}   {', '.join(failed) or '-'}")
    text = "\n".join(lines) + "\n"
    open(args.out, "w", encoding="utf-8").write(text)
    json.dump({"|".join(map(str, k)): v for k, v in by.items()},
              open(os.path.splitext(args.out)[0] + ".json", "w"), default=float)
    print(text)


if __name__ == "__main__":
    main()
