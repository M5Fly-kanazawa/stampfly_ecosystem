#!/usr/bin/env python3
"""Motor health report from hover flight logs.
ホバー飛行ログからのモータ健全性レポート。

Detects a mechanically degraded rotor (one motor producing less thrust AND
less reaction torque per duty) by reading the steady hover trim the controller
holds. Backend for `sf log analyze --health`.

劣化したロータ（同じ duty で推力も反トルクも低下したモータ）を、制御器が保持する
定常ホバートリムから検出する。`sf log analyze --health` のバックエンド。

Input / 入力:
  A StampFly flight-log v1 bundle (`.sflog.zip` or an extracted directory) --
  the output of `sf log wifi` (docs/plans/flight-log-format-plan.md, lib/sflog).
  Duty is read from `motor.csv` (400 Hz, per-cycle duty) when the bundle has
  it, otherwise from the duty columns embedded in `ctrl_ref.csv` (50 Hz).
  StampFly フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ）--
  `sf log wifi` の出力（計画書・lib/sflog 参照）。duty は一式に `motor.csv`
  （400Hz、周期ごとの duty）があればそれを使い、無ければ `ctrl_ref.csv`
  （50Hz）に埋め込まれた duty 列を使う。

Diagnostic principle / 診断原理:
  Mixer (NED X-quad), duty index = [M1/FR(CCW), M2/RR(CW), M3/RL(CCW), M4/FL(CW)].
  Torque trims recoverable from the 4 duties (common thrust removed):
    roll  up = (M3+M4) - (M1+M2)      # >0: controller raises LEFT  => body tends roll-RIGHT
    pitch uq = (M1+M4) - (M2+M3)      # <0: controller raises REAR  => body tends nose-UP
    yaw   ur = (M1+M3) - (M2+M4)      # = yaw_torque / kappa ; CG-IMMUNE
  - sign(ur) isolates the spin-direction group (immune to CG offset / gravity).
      ur < 0  => CW  group {M2/RR, M4/FL} is weak
      ur > 0  => CCW group {M1/FR, M3/RL} is weak
  - Within the group, the corner is confounded by the airframe CG offset in a
    single hover. With several logs of differing severity, CG is constant while
    the fault scales, so corr(ur, up) and corr(ur, uq) across logs localize it.

The firmware applies the SAME thrust->duty map to all 4 motors (shared
MotorParams, no per-motor gain), so any steady duty asymmetry is the controller
compensating a REAL physical asymmetry, not configuration.
"""
import json
import math
import statistics as st

import pandas as pd

import sflog

# Telemetry duty order = [M1/FR(CCW), M2/RR(CW), M3/RL(CCW), M4/FL(CW)]
MOTOR_NAMES = ["M1/FR(CCW)", "M2/RR(CW)", "M3/RL(CCW)", "M4/FL(CW)"]
# protocol/spec/flight_log.yaml column order for motor.csv / ctrl_ref.csv.
# protocol/spec/flight_log.yaml の motor.csv / ctrl_ref.csv の列順。
DUTY_COLUMNS = ["duty_FR", "duty_RR", "duty_RL", "duty_FL"]
CW_GROUP = (1, 3)    # M2/RR, M4/FL
CCW_GROUP = (0, 2)   # M1/FR, M3/RL

# Expected sign of (corr(ur,up), corr(ur,uq)) when motor i is the degraded corner.
# "raise-Mi" moves (ur,up,uq) by a fixed sign pattern; corr signs follow from it.
# モータ i が劣化隅のときの corr 符号（CG除去クロスログ判定の対応表）。
CORNER_CORR_SIGN = {
    0: ("-", "+"),  # M1/FR
    1: ("+", "+"),  # M2/RR
    2: ("+", "-"),  # M3/RL
    3: ("-", "-"),  # M4/FL
}

# Thresholds / 閾値
HOVER_DUTY_SUM = 2.0       # sum of 4 duties indicating hover thrust (~4 x 0.5)
MIN_HOVER_S = 10.0         # reject windows shorter than this (crash/abort)
UR_SIGNIFICANT = 0.03      # |mean ur| below this => no clear imbalance
SAT_DUTY = 0.98            # duty at/above this counts as saturated
MIN_SAMPLES = 50           # need at least this many in-window powered samples
SPINUP_SKIP_US = 1_500_000   # skip 1.5 s climb transient after spin-up
LANDING_SKIP_US = 1_000_000  # skip the last 1.0 s before descent


# --------------------------------------------------------------------------
# Loading / 読み込み
# --------------------------------------------------------------------------
def _load(path):
    """Load a flight-log v1 bundle (`.sflog.zip` or extracted directory).
    フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ）を読み込む。"""
    return sflog.load(path)


def _select_duty_stream(log):
    """Pick the duty source: 400 Hz `motor` when present, else the 50 Hz
    duty columns embedded in `ctrl_ref`. Returns (DataFrame, source_name)
    or (None, None) when neither stream is in the bundle.
    duty の出所を選ぶ: `motor`（400Hz）があればそれを、無ければ `ctrl_ref`
    （50Hz）埋め込みの duty 列を使う。どちらも無ければ (None, None)。"""
    motor_df = log.streams.get("motor")
    if motor_df is not None and len(motor_df) > 0:
        return motor_df, "motor"
    ctrl_ref_df = log.streams.get("ctrl_ref")
    if ctrl_ref_df is not None and len(ctrl_ref_df) > 0:
        return ctrl_ref_df, "ctrl_ref"
    return None, None


def _hover_window(duty_df):
    """Stable powered-flight window from the 4-duty sum (skip spin-up &
    landing). `duty_df` must have `timestamp_us` and DUTY_COLUMNS.
    4-duty合計から安定飛行区間を抽出（スピンアップ・着陸を除外）。"""
    duty_sum = duty_df[DUTY_COLUMNS].sum(axis=1)
    spun_ts = duty_df.loc[duty_sum > HOVER_DUTY_SUM, "timestamp_us"]
    if len(spun_ts) < MIN_SAMPLES:
        return None
    t_lo = spun_ts.iloc[0] + SPINUP_SKIP_US
    t_hi = spun_ts.iloc[-1] - LANDING_SKIP_US
    return (t_lo, t_hi) if t_hi > t_lo else None


def _quat_roll_pitch_deg(q):
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    return roll, pitch


def _window_series(df, t_lo, t_hi, column):
    """Values of `column` in `df` within [t_lo, t_hi] (empty Series if the
    stream is absent from the bundle).
    `df` の `column` を [t_lo, t_hi] 区間だけ取り出す（ストリームが一式に
    無ければ空の Series）。"""
    if df is None:
        return pd.Series(dtype=float)
    mask = (df["timestamp_us"] >= t_lo) & (df["timestamp_us"] <= t_hi)
    return df.loc[mask, column]


def per_log_stats(path):
    """Compute hover trim statistics for one log, or None if no clean hover.
    1ログのホバートリム統計を返す（クリーンなホバーが無ければ None）。"""
    log = _load(path)
    duty_df, duty_source = _select_duty_stream(log)
    if duty_df is None:
        return None

    window = _hover_window(duty_df)
    if window is None:
        return None
    t_lo, t_hi = window

    duty_sum = duty_df[DUTY_COLUMNS].sum(axis=1)
    in_window = (duty_df["timestamp_us"] >= t_lo) & (duty_df["timestamp_us"] <= t_hi)
    mask = in_window & (duty_sum > HOVER_DUTY_SUM)
    n = int(mask.sum())
    if n < MIN_SAMPLES:
        return None

    duties = duty_df.loc[mask, DUTY_COLUMNS]
    mean4 = duties.mean(axis=1)
    devs = duties.sub(mean4, axis=0)
    sat = duties.ge(SAT_DUTY)

    duty_mean = duties.mean(axis=0).reindex(DUTY_COLUMNS).tolist()
    dev_mean = devs.mean(axis=0).reindex(DUTY_COLUMNS).tolist()
    sat_frac = sat.mean(axis=0).reindex(DUTY_COLUMNS).tolist()

    up = float(((duties["duty_RL"] + duties["duty_FL"])
                - (duties["duty_FR"] + duties["duty_RR"])).mean())
    uq = float(((duties["duty_FR"] + duties["duty_FL"])
                - (duties["duty_RR"] + duties["duty_RL"])).mean())
    ur = float(((duties["duty_FR"] + duties["duty_RL"])
                - (duties["duty_RR"] + duties["duty_FL"])).mean())

    gz = _window_series(log.streams.get("imu"), t_lo, t_hi, "gyro_z")
    volts = _window_series(log.streams.get("status"), t_lo, t_hi, "voltage")

    return {
        "path": str(path),
        # float(): t_lo/t_hi come from pandas .iloc access (numpy int64),
        # so plain arithmetic would leave a numpy scalar here -- normalize
        # to a plain Python float like every other stat in this dict.
        # float(): t_lo/t_hi は pandas の .iloc 由来（numpy int64）なので、
        # そのまま演算すると numpy スカラーが残る -- この dict の他の統計量
        # と同様に素の Python float に揃える。
        "dur_s": float(t_hi - t_lo) / 1e6,
        "n": n,
        "duty_source": duty_source,
        "duty_mean": duty_mean,
        "dev_mean": dev_mean,
        "up": up, "uq": uq, "ur": ur,
        "sat_frac": sat_frac,
        "gz_sd": float(gz.std(ddof=0)) if len(gz) > 1 else 0.0,
        "gz_peak": float(gz.abs().max()) if len(gz) > 0 else 0.0,
        "volt": float(volts.mean()) if len(volts) > 0 else float("nan"),
    }


# --------------------------------------------------------------------------
# Verdict / 判定
# --------------------------------------------------------------------------
def _sign(x, tol=0.0):
    return "+" if x > tol else ("-" if x < -tol else "0")


def _corr(a, b):
    if len(a) < 3:
        return None
    ma, mb = st.mean(a), st.mean(b)
    num = sum((a[i] - ma) * (b[i] - mb) for i in range(len(a)))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((x - mb) ** 2 for x in b))
    return num / (da * db) if da * db else 0.0


def verdict(rows):
    """Build the spin-group + corner verdict from one or more per-log stats.
    1つ以上のログ統計から、回転グループ＋隅の判定を組み立てる。"""
    ur = st.mean([r["ur"] for r in rows])
    up = st.mean([r["up"] for r in rows])
    uq = st.mean([r["uq"] for r in rows])

    out = {"ur": ur, "up": up, "uq": uq, "n_logs": len(rows)}

    # No clear imbalance / 不均衡なし
    if abs(ur) < UR_SIGNIFICANT:
        out["group"] = None
        out["group_text"] = ("balanced: |yaw trim| below threshold "
                             f"(|ur|={abs(ur):.3f} < {UR_SIGNIFICANT}) - no clear motor fault")
        out["corner"] = None
        return out

    if ur < 0:
        group, group_name = CW_GROUP, "CW {M2/RR, M4/FL}"
    else:
        group, group_name = CCW_GROUP, "CCW {M1/FR, M3/RL}"
    out["group"] = group
    out["group_text"] = (f"yaw trim ur={ur:+.3f} (CG-immune) => {group_name} group is weak")

    # Corner: cross-log correlation (CG-removed) if enough logs, else single-log lean.
    if len(rows) >= 3:
        URs = [r["ur"] for r in rows]
        c_up = _corr(URs, [r["up"] for r in rows])
        c_uq = _corr(URs, [r["uq"] for r in rows])
        out["corr_up"], out["corr_uq"] = c_up, c_uq
        want = (_sign(c_up), _sign(c_uq))
        match = [i for i in group if CORNER_CORR_SIGN[i] == want]
        if len(match) == 1:
            out["corner"] = match[0]
            out["corner_method"] = "crosslog"
            out["corner_conf"] = "favored (CG-removed cross-log scaling)"
        else:
            # fall back to the hardest-driven motor in the group
            out["corner"] = max(group, key=lambda k: st.mean([r["dev_mean"][k] for r in rows]))
            out["corner_method"] = "crosslog-ambiguous"
            out["corner_conf"] = ("ambiguous (corr signs inconsistent); "
                                 "fell back to hardest-driven motor")
    else:
        out["corner"] = max(group, key=lambda k: st.mean([r["dev_mean"][k] for r in rows]))
        out["corner_method"] = "single-log"
        out["corner_conf"] = ("lean only - hardest-driven motor in group; "
                             "CG offset can mimic this. Use --batch or a bench swap to confirm")

    # Saturation of the suspect corner / 容疑モータの飽和
    if out.get("corner") is not None:
        ci = out["corner"]
        out["corner_sat"] = st.mean([r["sat_frac"][ci] for r in rows])
    return out


# --------------------------------------------------------------------------
# Reporting / レポート出力
# --------------------------------------------------------------------------
def _fmt_path(p):
    """Short display name for a bundle path: basename with `.sflog.zip` and
    a leading `flight_` stripped (a directory bundle shows its dir name).
    一式パスの短縮表示名: 拡張子 `.sflog.zip` と先頭の `flight_` を外した
    ベース名（ディレクトリの一式ならそのディレクトリ名）。"""
    base = str(p).rstrip("/").split("/")[-1]
    if base.endswith(".sflog.zip"):
        base = base[: -len(".sflog.zip")]
    if base.startswith("flight_"):
        base = base[len("flight_"):]
    return base


def analyze_health(paths, json_out=False):
    """Run the motor health report over one or more flight-log bundles.
    1つ以上のフライトログ一式でモータ健全性レポートを実行する。

    Returns the verdict dict (also printed unless json_out)."""
    rows = []
    skipped = []
    for p in paths:
        try:
            r = per_log_stats(p)
        except Exception as e:  # noqa: BLE001 - report and continue
            skipped.append((p, f"error: {e}"))
            continue
        if r is None:
            skipped.append((p, "no clean hover window"))
            continue
        if r["dur_s"] < MIN_HOVER_S:
            skipped.append((p, f"hover too short ({r['dur_s']:.1f}s) - crash/abort"))
            continue
        rows.append(r)

    if not rows:
        result = {"error": "no clean hover logs", "skipped": skipped}
        if not json_out:
            print("No clean hover windows found. Skipped:")
            for p, why in skipped:
                print(f"  - {_fmt_path(p)}: {why}")
        return result

    v = verdict(rows)
    result = {"verdict": v, "n_logs": len(rows), "skipped": skipped,
              "logs": [{"name": _fmt_path(r["path"]), **{k: r[k] for k in
                       ("dur_s", "duty_source", "duty_mean", "dev_mean", "up", "uq", "ur",
                        "sat_frac", "gz_sd", "volt")}} for r in rows]}
    if json_out:
        print(json.dumps(result, indent=2))
        return result

    _print_report(rows, v, skipped)
    return result


def _print_report(rows, v, skipped):
    line = "=" * 72
    print(line)
    print(" Motor Health Report  /  モータ健全性レポート")
    print(line)
    print(f"Clean hover logs: {len(rows)}")
    for p, why in skipped:
        print(f"  (skipped {_fmt_path(p)}: {why})")

    # Per-log table. "src" = duty source, motor.csv (400Hz) or ctrl_ref.csv (50Hz).
    # per-log 表。"src" = duty の出所、motor.csv (400Hz) か ctrl_ref.csv (50Hz)。
    print("\nPer-log hover trim (src: motor=motor.csv 400Hz, ctrl_ref=ctrl_ref.csv 50Hz):")
    print(f"  {'log':<16}{'src':>9}{'dur':>5}{'volt':>6} | "
          f"{'M1/FR':>7}{'M2/RR':>7}{'M3/RL':>7}{'M4/FL':>7} | "
          f"{'ur':>7}{'up':>7}{'uq':>7} | {'gz_sd':>6}")
    for r in rows:
        dm = r["duty_mean"]
        print(f"  {_fmt_path(r['path']):<16}{r['duty_source']:>9}{r['dur_s']:>5.0f}{r['volt']:>6.2f} | "
              f"{dm[0]:>7.3f}{dm[1]:>7.3f}{dm[2]:>7.3f}{dm[3]:>7.3f} | "
              f"{r['ur']:>7.3f}{r['up']:>7.3f}{r['uq']:>7.3f} | {r['gz_sd']:>6.3f}")

    # Aggregate duty deviation
    means = [st.mean([r["dev_mean"][k] for r in rows]) for k in range(4)]
    print("\nDuty deviation from 4-motor mean (x1000):")
    print("  " + "".join(f"{n:>11}" for n in ["M1/FR", "M2/RR", "M3/RL", "M4/FL"]))
    print("  " + "".join(f"{means[k]*1000:>11.1f}" for k in range(4)))

    # Saturation
    satmax = [max(r["sat_frac"][k] for r in rows) for k in range(4)]
    if max(satmax) > 0.01:
        print("\nSaturation (duty>={:.2f}, worst log):".format(SAT_DUTY))
        for k in range(4):
            if satmax[k] > 0.01:
                print(f"  {MOTOR_NAMES[k]}: {satmax[k]*100:.0f}% of samples"
                      f"  <- low control-authority margin")

    # Verdict
    print("\n" + "-" * 72)
    print(" Verdict / 判定")
    print("-" * 72)
    if v["group"] is None:
        print(f"  {v['group_text']}")
        return
    print(f"  Spin group : {v['group_text']}")
    if "corr_up" in v:
        print(f"  Cross-log  : corr(ur,up)={v['corr_up']:+.2f}  "
              f"corr(ur,uq)={v['corr_uq']:+.2f}")
    if v.get("corner") is not None:
        ci = v["corner"]
        print(f"  Corner     : {MOTOR_NAMES[ci]}  [{v['corner_conf']}]")
        if v.get("corner_sat", 0) > 0.01:
            print(f"               (saturates {v['corner_sat']*100:.0f}% of the time)")

    # Recommendation
    print("\n" + "-" * 72)
    print(" Recommendation / 推奨")
    print("-" * 72)
    if v.get("corner") is not None:
        name = MOTOR_NAMES[v["corner"]].split("(")[0]
        other = {0: "M3/RL", 2: "M1/FR", 1: "M4/FL", 3: "M2/RR"}[v["corner"]]
        print(f"  1. Inspect {name} PROPELLER first (chips/bend/play): the deficit is")
        print(f"     mostly reaction-torque (yaw), which points to prop drag.")
        print(f"  2. Compare {name} motor no-load current / spin-up / bearing vs others.")
        print(f"  3. Definitive: swap {name} <-> {other} prop, re-log, and re-run")
        print(f"     `sf log analyze --health --batch` - the yaw deficit should move")
        print(f"     with the swapped unit if the fault is in that hardware.")
        if v["corner_method"].startswith("single") or "ambiguous" in v["corner_method"]:
            print("  NOTE: corner is a lean only. Collect several hover logs and run")
            print("        `--batch` for the CG-removed cross-log confirmation.")
    print()


def _main():
    import sys
    import glob
    args = sys.argv[1:]
    json_out = "--json" in args
    args = [a for a in args if a != "--json"]
    if not args:
        args = sorted(glob.glob("logs/*.sflog.zip"))
    paths = []
    for a in args:
        paths.extend(sorted(glob.glob(a)) if any(c in a for c in "*?[") else [a])
    analyze_health(paths, json_out=json_out)


if __name__ == "__main__":
    _main()
