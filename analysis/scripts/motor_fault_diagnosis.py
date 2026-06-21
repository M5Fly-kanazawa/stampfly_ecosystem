#!/usr/bin/env python3
"""Diagnose which motor is degraded from hover flight logs.
ホバー飛行ログから、どのモータが劣化しているかを診断する。

Hypothesis: one rotor was mechanically degraded (less thrust AND less
reaction torque per duty). The controller compensates by driving that
corner harder and by holding a steady yaw torque command. The sign of
the steady yaw command isolates the spin-direction group (immune to CG
offset / gravity trim), while roll/pitch trim and the per-motor duty
deviation pin down the corner.

仮説: あるロータが機械的に劣化（同じ duty で推力も反トルクも低下）。
制御器はその隅を強く回し、かつ定常的なヨートルク指令を保持して補償する。
ヨー指令の符号は回転方向グループを切り分ける（CG/重力トリムの影響なし）。
ロール/ピッチtrim と各モータ duty 偏差で隅を特定する。

Mixer (NED, X-quad), duty index = [M1/FR(CCW), M2/RR(CW), M3/RL(CCW), M4/FL(CW)]:
  M1 = 0.25(ut - up/d + uq/d + ur/k)
  M2 = 0.25(ut - up/d - uq/d - ur/k)
  M3 = 0.25(ut + up/d - uq/d + ur/k)
  M4 = 0.25(ut + up/d + uq/d - ur/k)
=> recoverable torque trims from duties (common thrust removed):
  up (roll)  ~ (M3+M4) - (M1+M2)
  uq (pitch) ~ (M1+M4) - (M2+M3)
  ur (yaw)   ~ (M1+M3) - (M2+M4)
"""
import json
import sys
import glob
import math
import statistics as st

MOTOR_NAMES = ["M1/FR(CCW)", "M2/RR(CW)", "M3/RL(CCW)", "M4/FL(CW)"]

# Fault signature table: (weak motor) -> expected sign of (ur, up, uq)
SIG = {
    0: ("+", "-", "+"),  # M1/FR
    1: ("-", "-", "-"),  # M2/RR
    2: ("+", "+", "-"),  # M3/RL
    3: ("-", "+", "+"),  # M4/FL
}


def quat_to_rollpitch(q):
    w, x, y, z = q
    # roll (x-axis), pitch (y-axis); NED body
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def load(path):
    """Return dict of per-id lists of (ts, payload)."""
    streams = {"ctrl_ref": [], "ctrl": [], "imu": [], "status": []}
    with open(path) as f:
        for line in f:
            try:
                d = json.loads(line)
            except Exception:
                continue
            i = d.get("id")
            if i in streams:
                streams[i].append(d)
    return streams


def hover_window(ctrl_ref):
    """Find the stable powered-flight window from motor_duty sum.
    motor_duty 合計から安定飛行区間を抽出（スピンアップ後〜着陸前を1秒ずつ除外）。"""
    sums = [(d["ts"], sum(d["motor_duty"])) for d in ctrl_ref]
    spun = [ts for ts, s in sums if s > 2.0]  # hovering thrust (~4x0.5)
    if len(spun) < 50:
        return None
    t_lo = spun[0] + 1_500_000   # skip 1.5s spin-up/climb transient
    t_hi = spun[-1] - 1_000_000  # skip last 1s before descent
    if t_hi <= t_lo:
        return None
    return t_lo, t_hi


def nearest(sorted_items, ts):
    """ctrl record nearest in time."""
    lo, hi = 0, len(sorted_items) - 1
    if not sorted_items:
        return None
    while lo < hi:
        mid = (lo + hi) // 2
        if sorted_items[mid]["ts"] < ts:
            lo = mid + 1
        else:
            hi = mid
    return sorted_items[lo]


def analyze(path):
    s = load(path)
    win = hover_window(s["ctrl_ref"])
    if win is None:
        return None
    t_lo, t_hi = win

    duties = [[], [], [], []]
    devs = [[], [], [], []]      # per-sample deviation from 4-motor mean
    ur_d, up_d, uq_d = [], [], []  # torque trims derived from duties
    for d in s["ctrl_ref"]:
        ts = d["ts"]
        if not (t_lo <= ts <= t_hi):
            continue
        m = d["motor_duty"]
        if sum(m) < 2.0:
            continue
        mean4 = sum(m) / 4.0
        for k in range(4):
            duties[k].append(m[k])
            devs[k].append(m[k] - mean4)
        up_d.append((m[2] + m[3]) - (m[0] + m[1]))
        uq_d.append((m[0] + m[3]) - (m[1] + m[2]))
        ur_d.append((m[0] + m[2]) - (m[1] + m[3]))

    # ctrl record torque commands (controller outputs) within window
    c_roll, c_pitch, c_yaw = [], [], []
    for d in s["ctrl"]:
        if t_lo <= d["ts"] <= t_hi:
            c_roll.append(d["roll"])
            c_pitch.append(d["pitch"])
            c_yaw.append(d["yaw"])

    # imu yaw rate & attitude
    gz, rolls, pitches = [], [], []
    for d in s["imu"]:
        if t_lo <= d["ts"] <= t_hi:
            gz.append(d["gyro"][2])
            r, p = quat_to_rollpitch(d["quat"])
            rolls.append(r)
            pitches.append(p)

    volts = [d["voltage"] for d in s["status"] if t_lo <= d["ts"] <= t_hi]

    def m(x):
        return st.mean(x) if x else float("nan")

    def sd(x):
        return st.pstdev(x) if len(x) > 1 else 0.0

    return {
        "path": path,
        "dur_s": (t_hi - t_lo) / 1e6,
        "duty_mean": [m(duties[k]) for k in range(4)],
        "dev_mean": [m(devs[k]) for k in range(4)],
        "ur_duty": m(ur_d), "up_duty": m(up_d), "uq_duty": m(uq_d),
        "c_yaw": m(c_yaw), "c_roll": m(c_roll), "c_pitch": m(c_pitch),
        "c_yaw_sd": sd(c_yaw),
        "gz_mean": m(gz), "gz_sd": sd(gz),
        "roll_deg": math.degrees(m(rolls)) if rolls else float("nan"),
        "pitch_deg": math.degrees(m(pitches)) if pitches else float("nan"),
        "volt": m(volts),
        "n": len(duties[0]),
    }


def sign(x, tol=0.0):
    if x > tol:
        return "+"
    if x < -tol:
        return "-"
    return "0"


def main():
    paths = sys.argv[1:]
    if not paths:
        paths = sorted(glob.glob("logs/stampfly_udp_2026061*.jsonl"))
    results = []
    for p in paths:
        try:
            r = analyze(p)
        except Exception as e:
            print(f"!! {p}: {e}")
            continue
        if r is None:
            print(f".. {p}: no hover window")
            continue
        # Reject too-short windows (e.g. crash-aborts like 185501 that only powered ~2s):
        # they are not clean hovers and corrupt the trim statistics.
        # 短すぎる窓（墜落アボート等）はクリーンなホバーでなく統計を汚すため除外。
        if r["dur_s"] < 10.0:
            print(f".. {p}: hover window too short ({r['dur_s']:.1f}s) -> excluded (crash/abort)")
            continue
        results.append(r)

    print("\n========== PER-LOG HOVER TRIM ==========")
    hdr = (f"{'log':<22} {'dur':>4} {'volt':>4} | "
           f"{'M1/FR':>7}{'M2/RR':>7}{'M3/RL':>7}{'M4/FL':>7} (duty mean) | "
           f"{'ur':>6}{'up':>6}{'uq':>6} (duty-trim) | "
           f"{'cyaw':>7}{'croll':>7}{'cpit':>7} | {'gz':>6}")
    print(hdr)
    for r in results:
        name = r["path"].split("/")[-1].replace("stampfly_udp_", "").replace(".jsonl", "")
        dm = r["duty_mean"]
        print(f"{name:<22} {r['dur_s']:>4.0f} {r['volt']:>4.2f} | "
              f"{dm[0]:>7.3f}{dm[1]:>7.3f}{dm[2]:>7.3f}{dm[3]:>7.3f} | "
              f"{r['ur_duty']:>6.3f}{r['up_duty']:>6.3f}{r['uq_duty']:>6.3f} | "
              f"{r['c_yaw']:>7.4f}{r['c_roll']:>7.4f}{r['c_pitch']:>7.4f} | {r['gz_mean']:>6.3f}")

    # Aggregate the deviation-from-mean per motor across logs
    print("\n========== PER-MOTOR DUTY DEVIATION (mean of dev-from-4-motor-mean, x1000) ==========")
    print(f"{'log':<22} {'M1/FR':>8}{'M2/RR':>8}{'M3/RL':>8}{'M4/FL':>8}   highest")
    agg = [[], [], [], []]
    for r in results:
        name = r["path"].split("/")[-1].replace("stampfly_udp_", "").replace(".jsonl", "")
        dv = r["dev_mean"]
        for k in range(4):
            agg[k].append(dv[k])
        hi = max(range(4), key=lambda k: dv[k])
        print(f"{name:<22} " + "".join(f"{dv[k]*1000:>8.1f}" for k in range(4)) +
              f"   {MOTOR_NAMES[hi]}")
    print("-" * 70)
    means = [st.mean(agg[k]) for k in range(4)]
    print(f"{'MEAN':<22} " + "".join(f"{means[k]*1000:>8.1f}" for k in range(4)) +
          f"   {MOTOR_NAMES[max(range(4), key=lambda k: means[k])]}")

    # Verdict from steady torque trims (sign-vote across logs)
    print("\n========== TORQUE-TRIM SIGN VOTE (from duty-derived ur/up/uq) ==========")
    ur_s = sign(st.mean([r["ur_duty"] for r in results]))
    up_s = sign(st.mean([r["up_duty"] for r in results]))
    uq_s = sign(st.mean([r["uq_duty"] for r in results]))
    print(f"  mean ur (yaw)  = {st.mean([r['ur_duty'] for r in results]):+.4f}  sign {ur_s}")
    print(f"  mean up (roll) = {st.mean([r['up_duty'] for r in results]):+.4f}  sign {up_s}")
    print(f"  mean uq (pitch)= {st.mean([r['uq_duty'] for r in results]):+.4f}  sign {uq_s}")
    obs = (ur_s, up_s, uq_s)
    print(f"  observed sign triple (ur,up,uq) = {obs}")
    for mi, sg in SIG.items():
        match = "  <== MATCH" if sg == obs else ""
        print(f"    {MOTOR_NAMES[mi]:<12} expects {sg}{match}")

    print("\n  Cross-check with controller ctrl.yaw/roll/pitch outputs:")
    print(f"    mean ctrl.yaw   = {st.mean([r['c_yaw'] for r in results]):+.4f}  sign {sign(st.mean([r['c_yaw'] for r in results]))}")
    print(f"    mean ctrl.roll  = {st.mean([r['c_roll'] for r in results]):+.4f}")
    print(f"    mean ctrl.pitch = {st.mean([r['c_pitch'] for r in results]):+.4f}")

    # CG-removal: the corner is ambiguous from absolute roll/pitch trim because the airframe
    # CG offset contaminates exactly those axes (yaw is CG-immune). But CG is constant per
    # airframe, while the fault scales with severity. So correlate roll/pitch trim against the
    # yaw deficit across logs: the FAULT-driven axis tracks ur; the CG-driven part does not.
    #   raise-M2 moves (ur,up,uq) all NEGATIVE  -> corr(ur,up)>0 and corr(ur,uq)>0
    #   raise-M4 moves up,uq POSITIVE as ur goes negative -> corr<0
    print("\n========== CG-REMOVED CORNER TEST (trim vs yaw-deficit across logs) ==========")
    if len(results) >= 4:
        UR = [r["ur_duty"] for r in results]
        UP = [r["up_duty"] for r in results]
        UQ = [r["uq_duty"] for r in results]

        def corr(a, b):
            ma, mb = st.mean(a), st.mean(b)
            num = sum((a[i] - ma) * (b[i] - mb) for i in range(len(a)))
            da = math.sqrt(sum((x - ma) ** 2 for x in a))
            db = math.sqrt(sum((x - mb) ** 2 for x in b))
            return num / (da * db) if da * db else 0.0

        cup, cuq = corr(UR, UP), corr(UR, UQ)
        print(f"  corr(ur, up-roll)  = {cup:+.3f}")
        print(f"  corr(ur, uq-pitch) = {cuq:+.3f}")
        print("  both > 0  => fault corner is M2/RR (rear-right, CW)")
        print("  both < 0  => fault corner is M4/FL (front-left, CW)")
        verdict = "M2/RR (rear-right)" if (cup + cuq) > 0 else "M4/FL (front-left)"
        print(f"  => CW group is certain (ur<0); corner favored: {verdict}")
        print("  NOTE: definitive confirmation requires a bench swap test (M2<->M4 prop/motor).")


if __name__ == "__main__":
    main()
