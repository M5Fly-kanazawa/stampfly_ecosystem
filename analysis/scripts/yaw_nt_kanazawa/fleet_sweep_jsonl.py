"""
fleet_sweep_jsonl.py
=====================
StampFly 全ログ横断: CW/CCW duty 非対称の集計スクリプト。

再利用（改変せず import）:
    torque_budget.py: find_quiet_windows, static_asymmetry, duty_to_thrust, KAPPA, load_log
    hover_duty_permotor.py: decompose (mixer-basis 分解), d_pred_of_vbat

目的: どのファイルでも同一の静穏窓検出・duty_diff・tau_static 計算を適用し、
モータ個別の Delta パターン（対称push-pull vs 単一モータ突出）も併せて出す。
判定（製品固有かどうか）は行わない。集計と例外列挙のみ。
"""
import sys
import json
import glob
import os
import numpy as np

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
SCRIPTS_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa"
sys.path.insert(0, SCRATCH)
sys.path.insert(0, SCRIPTS_DIR)
import yawlib  # noqa: E402
import torque_budget as tb  # noqa: E402
import hover_duty_permotor as hd  # noqa: E402

LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"

# Target: all logs from 2026-06-18 through 2026-06-29 (per task instruction)
DATE_PREFIXES = ["20260618", "20260619", "20260620", "20260621", "20260622",
                  "20260624", "20260627", "20260629"]


def target_files():
    files = sorted(glob.glob(f"{LOG_DIR}/stampfly_udp_*.jsonl"))
    out = []
    for f in files:
        base = os.path.basename(f)
        # stampfly_udp_YYYYMMDDTHHMMSS.jsonl
        datepart = base.split("_")[2][:8]
        if datepart in DATE_PREFIXES:
            out.append(base)
    return out


NOMINAL_CORR = 1.12


def analyze_file(fname):
    """Return dict: file-level summary + list of per-window records.
    Reuses tb.find_quiet_windows / tb.static_asymmetry verbatim; adds
    per-motor Delta decomposition via hd.decompose / hd.d_pred_of_vbat."""
    try:
        data = tb.load_log(fname)
    except Exception as e:
        return dict(file=fname, error=f"load_failed: {e}", windows=[])

    required = ["ctrl_ref", "status", "imu", "ctrl"]
    missing = [k for k in required if k not in data]
    if missing:
        return dict(file=fname, error=f"missing_topics: {missing}", windows=[])

    try:
        windows = tb.find_quiet_windows(fname)
    except Exception as e:
        return dict(file=fname, error=f"quiet_window_failed: {e}", windows=[])

    if len(windows) == 0:
        return dict(file=fname, error=None, windows=[])

    cr = data["ctrl_ref"]
    status = data["status"]
    recs = []
    for wi, (s, e, t0, t1) in enumerate(windows):
        duty = cr["motor_duty"][s:e + 1]           # (N,4) FR,RR,RL,FL
        t_seg = cr["ts"][s:e + 1]
        vbat_seg = np.interp(t_seg, status["ts"], status["voltage"])
        vbat_mean = float(np.mean(vbat_seg))

        duty_diff = float(np.mean(duty[:, 0] + duty[:, 2]) - np.mean(duty[:, 1] + duty[:, 3]))
        T = tb.duty_to_thrust(duty, vbat_seg[:, None])
        tau_static = float(np.mean(tb.KAPPA * ((T[:, 0] + T[:, 2]) - (T[:, 1] + T[:, 3]))))

        dbar = np.mean(duty, axis=0)
        dpred_seg = hd.d_pred_of_vbat(vbat_seg, NOMINAL_CORR)
        dpred_mean = float(np.mean(dpred_seg))
        delta = dbar - dpred_mean
        cT, croll, cpitch, cyaw = hd.decompose(delta)

        recs.append(dict(
            wi=wi, t0=float(t0), t1=float(t1), dur=float(t1 - t0),
            n=int(e - s + 1), vbat_mean=vbat_mean,
            duty_diff=duty_diff, tau_static_mNm=tau_static * 1e3,
            dbar=dbar.tolist(), delta=delta.tolist(),
            cT_delta=cT, croll_delta=croll, cpitch_delta=cpitch, cyaw_delta=cyaw,
        ))
    return dict(file=fname, error=None, windows=recs)


def main():
    files = target_files()
    print(f"Target files: {len(files)}")
    for f in files:
        print(f"  {f}")

    all_results = []
    for fname in files:
        print(f"\n[analyze] {fname}")
        res = analyze_file(fname)
        all_results.append(res)
        if res["error"]:
            print(f"  ERROR: {res['error']}")
            continue
        if len(res["windows"]) == 0:
            print("  0 quiet windows")
            continue
        for w in res["windows"]:
            print(f"  w{w['wi']} [{w['t0']:.1f},{w['t1']:.1f}]s dur={w['dur']:.1f}s "
                  f"vbat={w['vbat_mean']:.3f}V duty_diff={w['duty_diff']:+.4f} "
                  f"tau_static={w['tau_static_mNm']:+.4f}mNm "
                  f"delta=[{','.join(f'{d:+.4f}' for d in w['delta'])}] "
                  f"cyaw_delta={w['cyaw_delta']:+.4f} cT_delta={w['cT_delta']:+.4f}")

    with open(f"{SCRATCH}/fleet_sweep_jsonl.json", "w") as fp:
        json.dump(dict(nominal_corr=NOMINAL_CORR, results=all_results), fp, indent=2, default=str)
    print(f"\nSaved -> {SCRATCH}/fleet_sweep_jsonl.json")


if __name__ == "__main__":
    main()
