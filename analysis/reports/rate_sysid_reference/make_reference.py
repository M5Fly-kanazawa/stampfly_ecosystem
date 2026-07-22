#!/usr/bin/env python3
"""
make_reference.py — build the model-match gate's real-hardware reference
(analysis/reports/rate_sysid_reference/reference.json) from two real-flight
system-ID runs' metrics.json (sysid.<axis>.fit).

reference.json（モデル一致ゲートの実機基準値）を、2本の実飛行同定 run の metrics.json
（sysid.<axis>.fit）から機械的に生成する。手打ちしない — 値は全て metrics.json から
読む（simulation-policy.md §4）。

Why average two runs, and why L_total = T + L (not T, L separately):
T/L separation is degenerate in some fits (T collapses to ~1e-10 s and the delay
lands entirely in L) — see the per-run dump below where roll/yaw's T is ~0. The
COMBINED lag L_total = T + L is stable across the two runs (the physical dead
time + motor lag together, regardless of how the fit split them), so the gate
compares L_total, not T and L individually (docs/architecture/simulation-policy.md
§4, backlog #0 task notes).

なぜ2run平均で、なぜ L_total=T+L か:
T/L分離はフィットによって退化する（Tが~1e-10sに潰れ、遅れが全てLに寄る例がある —
下のダンプで roll/yaw の T がほぼ0）。合計遅れ L_total=T+L は2run間で安定
（実際の物理むだ時間＋モータ遅れの合計は、フィットの内訳配分に依らず安定）ため、
ゲートは T・L 個別ではなく L_total を比較する。

Usage: python3 analysis/reports/rate_sysid_reference/make_reference.py
       (regenerates reference.json in place, from the runs listed in RUNS below)
"""
import json
import os
import datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))

# The two real-flight system-ID runs this reference is built from (RESET_PLAN /
# simulation-policy.md §1 layer-1 identification results). Add more runs here and
# re-run this script to fold them into the average as more flights accumulate.
# この基準値の元になった2本の実飛行同定 run（layer-1同定の結果）。飛行が増えたら
# ここに追記し本スクリプトを再実行すれば平均に取り込まれる。
RUNS = ["altlog_20260614T201629", "altlog_20260614T214537"]
AXES = ["roll", "pitch", "yaw"]


def main():
    per_run = {}
    for run in RUNS:
        path = os.path.join(ROOT, "analysis", "reports", run, "metrics.json")
        with open(path) as f:
            metrics = json.load(f)
        per_run[run] = {}
        for axis in AXES:
            fit = metrics["sysid"][axis]["fit"]
            per_run[run][axis] = {
                "b": fit["b"], "T": fit["T"], "L": fit["L"],
                "L_total_s": fit["T"] + fit["L"],
                "coherence_mean": fit["coherence_mean"],
            }

    axes_out = {}
    for axis in AXES:
        b_vals = [per_run[r][axis]["b"] for r in RUNS]
        lt_vals = [per_run[r][axis]["L_total_s"] for r in RUNS]
        axes_out[axis] = {
            "b": sum(b_vals) / len(b_vals),
            "L_total_s": sum(lt_vals) / len(lt_vals),
            "runs": [
                {"run": r, "b": per_run[r][axis]["b"], "T_s": per_run[r][axis]["T"],
                 "L_s": per_run[r][axis]["L"], "L_total_s": per_run[r][axis]["L_total_s"],
                 "coherence_mean": per_run[r][axis]["coherence_mean"]}
                for r in RUNS
            ],
        }

    out = {
        "provenance": {
            "runs": RUNS,
            "source": "analysis/reports/<run>/metrics.json : sysid.<axis>.fit",
            "created": datetime.date.today().isoformat(),
            "generator": "analysis/reports/rate_sysid_reference/make_reference.py",
            "note": ("b and L_total=T+L averaged over the two runs above. T/L "
                     "individually are NOT used for the gate verdict (degenerate "
                     "split — see module docstring); kept per-run for reference "
                     "only. Re-run this script after adding more real-flight "
                     "system-ID runs to fold them into the average."),
        },
        "axes": axes_out,
    }

    out_path = os.path.join(HERE, "reference.json")
    with open(out_path, "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    print(f"wrote {out_path}")
    for axis in AXES:
        a = axes_out[axis]
        print(f"  {axis:6s} b={a['b']:.1f}  L_total={a['L_total_s']*1e3:.3f} ms")


if __name__ == "__main__":
    main()
