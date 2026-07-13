"""
hover_duty_permotor.py
=======================
StampFly 静穏ホバー duty のモータ個別レベル解析。

目的: CCW群(M1=FR,M3=RL)の duty が CW群(M2=RR,M4=FL)より高い既知の非対称
(torque_budget.py static_asymmetry) について、以下の対立仮説を反証形式で判別する。
    (a) CCW群モータの推力低下     -> 劣化モータの duty だけが健全予測値より上振れ
    (b) CW群プロペラの反トルク過大 -> CCW群+ε / CW群-ε の対称プッシュプル

再利用: torque_budget.py の duty_to_thrust/find_quiet_windows/load_log と物理定数
(KAPPA, MASS_KG, G, MOTOR_AM/BM/CM/CT, HOVER_THRUST_CORR) をそのまま使う。
出典は torque_budget.py 冒頭 docstring を参照(file:line 付き)。

ミキサー基底（actuator.cpp:226-231 mixerCompute の符号そのまま、motor_duty列順
FR,RR,RL,FL = M1,M2,M3,M4 は data_stream.cpp:281 コメントで確認済み）:
    T_FR = 0.25*(ut - up/d + uq/d + ur/kappa)
    T_RR = 0.25*(ut - up/d - uq/d - ur/kappa)
    T_RL = 0.25*(ut + up/d - uq/d + ur/kappa)
    T_FL = 0.25*(ut + up/d + uq/d - ur/kappa)
=> 基底ベクトル(FR,RR,RL,FL 順、符号のみ抽出):
    b_T     = [+1,+1,+1,+1]   (推力/collective)
    b_roll  = [-1,-1,+1,+1]   (up 項の符号)
    b_pitch = [+1,-1,-1,+1]   (uq 項の符号)
    b_yaw   = [+1,-1,+1,-1]   (ur 項の符号 = CCW群-CW群)
4本は互いに直交（内積0）・ノルム^2=4 の Hadamard 基底なので、
    v = cT*b_T + croll*b_roll + cpitch*b_pitch + cyaw*b_yaw,  c_X = (v . b_X)/4
で厳密に再構成できる（最小二乗射影と一致）。

d_pred(vbat, corr) は「健全モータが実推力 mg/4 を出すのに必要な duty」の解析解:
    thrust_model = corr * m*g/4   (corr=hover.thrust_corr; モータ曲線は実推力を
                                    約corr倍過大評価するので、真の目標推力を出すには
                                    モデル空間でcorr倍のthrustを要求する必要がある)
    omega = sqrt(thrust_model / Ct)
    volts = Am*omega^2 + Bm*omega + Cm      (モータ曲線、actuator.cpp:90-92 の直接式)
    d_pred = volts / vbat
"""
import sys
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
SCRIPTS_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa"
sys.path.insert(0, SCRATCH)
sys.path.insert(0, SCRIPTS_DIR)
import yawlib          # noqa: E402
import torque_budget as tb  # noqa: E402  (reuse vetted model + quiet-window detector)

LOG_DIR = tb.LOG_DIR

# --- Files under test (user-specified) ---
FILES_0627 = [
    "stampfly_udp_20260627T020050.jsonl",
    "stampfly_udp_20260627T020137.jsonl",
    "stampfly_udp_20260627T164611.jsonl",
    "stampfly_udp_20260627T165713.jsonl",
]
FILES_0629 = [
    "stampfly_udp_20260629T145357.jsonl",
    "stampfly_udp_20260629T145526.jsonl",
]
ALL_FILES = FILES_0627 + FILES_0629

MOTOR_LABELS = ["M1_FR(CCW)", "M2_RR(CW)", "M3_RL(CCW)", "M4_FL(CW)"]  # column order in motor_duty

# Mixer basis vectors, order FR,RR,RL,FL (actuator.cpp:226-229, verified above)
B_T = np.array([1.0, 1.0, 1.0, 1.0])
B_ROLL = np.array([-1.0, -1.0, 1.0, 1.0])
B_PITCH = np.array([1.0, -1.0, -1.0, 1.0])
B_YAW = np.array([1.0, -1.0, 1.0, -1.0])


def decompose(v):
    """Orthogonal projection of a length-4 duty(-like) vector [FR,RR,RL,FL]
    onto the (thrust, roll, pitch, yaw) mixer basis. Exact: v = cT*B_T + ..."""
    v = np.asarray(v, dtype=float)
    cT = float(np.dot(v, B_T) / 4.0)
    croll = float(np.dot(v, B_ROLL) / 4.0)
    cpitch = float(np.dot(v, B_PITCH) / 4.0)
    cyaw = float(np.dot(v, B_YAW) / 4.0)
    return cT, croll, cpitch, cyaw


def d_pred_of_vbat(vbat, corr):
    """Analytic duty required for a healthy motor to produce REAL thrust m*g/4
    at supply voltage vbat, given hover.thrust_corr=corr. Closed-form inverse
    of the motor curve (no iterative search needed since V(omega) is already
    given directly, unlike duty_to_thrust's inverse-quadratic omega(V))."""
    vbat = np.asarray(vbat, dtype=float)
    thrust_model_target = corr * tb.MASS_KG * tb.G / 4.0
    omega = np.sqrt(thrust_model_target / tb.MOTOR_CT)
    volts = tb.MOTOR_AM * omega**2 + tb.MOTOR_BM * omega + tb.MOTOR_CM
    return volts / vbat


def _selftest_dpred():
    """Cross-check the closed-form d_pred against tb.duty_to_thrust (numeric
    inverse) — must agree to float precision, else the analytic derivation
    above has a sign/algebra error."""
    for vb in (3.5, 3.7, 3.9):
        for corr in (1.0, 1.12, 1.3):
            dp = d_pred_of_vbat(vb, corr)
            t_real = tb.duty_to_thrust(dp, vb, corr=corr)
            target = tb.MASS_KG * tb.G / 4.0
            assert abs(t_real - target) < 1e-9, (vb, corr, t_real, target)
    print("[selftest] d_pred_of_vbat matches tb.duty_to_thrust inverse to 1e-9 N: OK")


# =============================================================================
# Step 1-3: per-window per-motor duty, prediction, deviation
# =============================================================================
def collect_windows(nominal_corr=1.12):
    """For every quiet window in every file: mean/std duty per motor, vbat,
    d_pred at nominal_corr, Delta_i = dbar_i - d_pred, and mixer-basis
    decomposition of both the raw duty vector and Delta."""
    rows = []
    for fname in ALL_FILES:
        data = tb.load_log(fname)
        cr = data["ctrl_ref"]
        status = data["status"]
        windows = tb.find_quiet_windows(fname)
        for wi, (s, e, t0, t1) in enumerate(windows):
            duty = cr["motor_duty"][s:e + 1]          # (N,4) FR,RR,RL,FL
            t_seg = cr["ts"][s:e + 1]
            vbat_seg = np.interp(t_seg, status["ts"], status["voltage"])

            dbar = np.mean(duty, axis=0)               # (4,)
            dstd = np.std(duty, axis=0)
            vbat_mean = float(np.mean(vbat_seg))

            dpred_seg = d_pred_of_vbat(vbat_seg, nominal_corr)
            dpred_mean = float(np.mean(dpred_seg))

            delta = dbar - dpred_mean                  # (4,) Delta_i

            cT_raw, croll_raw, cpitch_raw, cyaw_raw = decompose(dbar)
            cT_d, croll_d, cpitch_d, cyaw_d = decompose(delta)

            rows.append(dict(
                file=fname, date=fname.split("_")[2][:8], wi=wi,
                t0=float(t0), t1=float(t1), dur=float(t1 - t0),
                n=int(e - s + 1),
                vbat_mean=vbat_mean, dpred_mean=dpred_mean,
                dbar=dbar.tolist(), dstd=dstd.tolist(), delta=delta.tolist(),
                cT_raw=cT_raw, croll_raw=croll_raw, cpitch_raw=cpitch_raw, cyaw_raw=cyaw_raw,
                cT_delta=cT_d, croll_delta=croll_d, cpitch_delta=cpitch_d, cyaw_delta=cyaw_d,
            ))
    return rows


# =============================================================================
# Step 4: hypothesis residuals (least-squares projection onto each pattern)
# =============================================================================
def hypothesis_residuals(delta):
    """Given Delta=[FR,RR,RL,FL], compute best-fit residual sum-of-squares for
    each hypothesis pattern (1 window's worth). Returns dict."""
    delta = np.asarray(delta, dtype=float)
    total_ss = float(np.sum(delta**2))

    # Ha1: single motor i elevated by eps_i=delta[i], others predicted 0.
    ha1 = {}
    for i in range(4):
        resid = float(np.sum(delta[[j for j in range(4) if j != i]]**2))
        ha1[MOTOR_LABELS[i]] = resid
    best_motor = min(ha1, key=ha1.get)

    # Ha2: M1,M3 (idx 0,2) elevated equally by eps, M2,M4 (idx1,3) predicted 0.
    eps_ha2 = (delta[0] + delta[2]) / 2.0
    resid_ha2 = float((delta[0] - eps_ha2)**2 + (delta[2] - eps_ha2)**2 + delta[1]**2 + delta[3]**2)

    # Hb: pure antisymmetric push-pull = pure yaw-basis component.
    cT, croll, cpitch, cyaw = decompose(delta)
    resid_hb = 4.0 * (cT**2 + croll**2 + cpitch**2)   # = ||delta - cyaw*B_YAW||^2, verified below

    # Hab: overall-deficit + yaw component (drop roll/pitch trim as explained nuisance).
    resid_hab = 4.0 * (croll**2 + cpitch**2)

    return dict(total_ss=total_ss, ha1=ha1, best_motor=best_motor, ha1_best_resid=ha1[best_motor],
                ha2_eps=eps_ha2, ha2_resid=resid_ha2,
                hb_eps=cyaw, hb_resid=resid_hb,
                hab_resid=resid_hab,
                cT=cT, croll=croll, cpitch=cpitch, cyaw=cyaw)


def _selftest_hb_residual():
    """Verify resid_hb formula against direct vector residual."""
    rng = np.random.default_rng(0)
    for _ in range(20):
        d = rng.normal(size=4)
        cT, croll, cpitch, cyaw = decompose(d)
        direct = float(np.sum((d - cyaw * B_YAW)**2))
        formula = 4.0 * (cT**2 + croll**2 + cpitch**2)
        assert abs(direct - formula) < 1e-9, (direct, formula)
    print("[selftest] Hb residual formula matches direct vector residual: OK")


# =============================================================================
# Step 5: corr sensitivity
# =============================================================================
def corr_sensitivity(rows, corr_grid):
    """For each window, recompute Delta at each corr in corr_grid and confirm
    (a) croll/cpitch/cyaw of Delta are corr-invariant (they must be, since a
        constant shift of d_pred only projects onto B_T), and
    (b) cT_delta shifts monotonically with corr; find the corr at which
        cT_delta crosses 0 (pure-yaw / Hb boundary) and the corr at which
        cT_delta == cyaw_delta (Ha2 boundary: CW group exactly at prediction)."""
    out = []
    for r in rows:
        dbar = np.array(r["dbar"])
        vbat = r["vbat_mean"]
        cyaw_fixed = r["cyaw_raw"]     # invariant reference (== decompose(dbar) yaw term)
        croll_fixed = r["croll_raw"]
        cpitch_fixed = r["cpitch_raw"]

        cT_of_corr = []
        invariance_ok = True
        for corr in corr_grid:
            dpred = d_pred_of_vbat(vbat, corr)
            delta = dbar - dpred
            cT, croll, cpitch, cyaw = decompose(delta)
            cT_of_corr.append(cT)
            if not (abs(croll - croll_fixed) < 1e-12 and abs(cpitch - cpitch_fixed) < 1e-12
                    and abs(cyaw - cyaw_fixed) < 1e-12):
                invariance_ok = False

        cT_of_corr = np.array(cT_of_corr)

        # root: cT(corr) = 0  (Hb boundary)
        corr_zero = _find_root(corr_grid, cT_of_corr, 0.0)
        # root: cT(corr) = cyaw_fixed  (Ha2 boundary, CW group exactly at prediction)
        corr_ha2 = _find_root(corr_grid, cT_of_corr, cyaw_fixed)

        out.append(dict(file=r["file"], wi=r["wi"], invariance_ok=invariance_ok,
                         cT_at_corr=dict(zip([f"{c:.2f}" for c in corr_grid], cT_of_corr.tolist())),
                         corr_for_Hb_boundary=corr_zero, corr_for_Ha2_boundary=corr_ha2,
                         cyaw_fixed=cyaw_fixed))
    return out


def _find_root(x_grid, y_grid, target):
    """Linear-interpolation root find of y_grid(x_grid) == target; None if
    target not bracketed within the grid."""
    y = np.array(y_grid) - target
    sign_changes = np.where(np.diff(np.sign(y)) != 0)[0]
    if len(sign_changes) == 0:
        return None
    i = sign_changes[0]
    x0, x1 = x_grid[i], x_grid[i + 1]
    y0, y1 = y[i], y[i + 1]
    frac = -y0 / (y1 - y0)
    return float(x0 + frac * (x1 - x0))


# =============================================================================
# Reporting
# =============================================================================
def print_window_table(rows, nominal_corr):
    print("=" * 100)
    print(f"Per-window motor duty vs prediction (nominal corr={nominal_corr})")
    print("Column order: M1=FR(CCW) M2=RR(CW) M3=RL(CCW) M4=FL(CW)")
    print("=" * 100)
    hdr = (f"{'file':38s} {'w':2s} {'dur':>5s} {'vbat':>6s} {'dpred':>7s}  "
           + "  ".join([f"{m:>11s}" for m in MOTOR_LABELS]))
    print(hdr)
    for r in rows:
        label = f"{r['file'][12:27]}"
        deltas = "  ".join([f"{d:+.4f}" for d in r["delta"]])
        print(f"{label:38s} {r['wi']:2d} {r['dur']:5.1f} {r['vbat_mean']:6.3f} {r['dpred_mean']:7.4f}  "
              f"Delta: {deltas}")
        dbars = "  ".join([f"{d:.4f}" for d in r["dbar"]])
        print(f"{'':38s} {'':2s} {'':5s} {'':6s} {'':7s}  dbar:  {dbars}")


def print_decomposition_table(rows):
    print("\n" + "=" * 100)
    print("Mixer-basis decomposition of Delta (duty units). cT=collective offset from pred, "
          "croll/cpitch=trim, cyaw=(CCW-CW)/4")
    print("=" * 100)
    print(f"{'file/window':40s} {'cT_delta':>10s} {'croll_delta':>12s} {'cpitch_delta':>13s} {'cyaw_delta':>11s}")
    for r in rows:
        label = f"{r['file'][12:27]}_w{r['wi']}"
        print(f"{label:40s} {r['cT_delta']:+10.5f} {r['croll_delta']:+12.5f} "
              f"{r['cpitch_delta']:+13.5f} {r['cyaw_delta']:+11.5f}")


def print_hypothesis_table(rows):
    print("\n" + "=" * 100)
    print("Hypothesis residual comparison per window (duty^2 units; smaller = better fit)")
    print("=" * 100)
    print(f"{'file/window':32s} {'total_ss':>9s} {'Ha1_best':>9s} {'best_motor':>12s} "
          f"{'Ha2(M1M3)':>10s} {'Hb(push-pull)':>14s} {'Hab(mix)':>9s}")
    for r in rows:
        h = hypothesis_residuals(r["delta"])
        label = f"{r['file'][12:27]}_w{r['wi']}"
        print(f"{label:32s} {h['total_ss']:9.6f} {h['ha1_best_resid']:9.6f} {h['best_motor']:>12s} "
              f"{h['ha2_resid']:10.6f} {h['hb_resid']:14.6f} {h['hab_resid']:9.6f}")
    return [hypothesis_residuals(r["delta"]) for r in rows]


def aggregate_by_group(rows, label, files):
    sub = [r for r in rows if r["file"] in files]
    if not sub:
        print(f"\n{label}: no quiet windows found")
        return None
    delta_arr = np.array([r["delta"] for r in sub])   # (W,4)
    cT = np.array([r["cT_delta"] for r in sub])
    croll = np.array([r["croll_delta"] for r in sub])
    cpitch = np.array([r["cpitch_delta"] for r in sub])
    cyaw = np.array([r["cyaw_delta"] for r in sub])
    print(f"\n{label}: n_windows={len(sub)}")
    for i, m in enumerate(MOTOR_LABELS):
        print(f"  Delta_{m}: mean={np.mean(delta_arr[:, i]):+.4f} std={np.std(delta_arr[:, i]):.4f} "
              f"range=[{np.min(delta_arr[:, i]):+.4f},{np.max(delta_arr[:, i]):+.4f}]")
    print(f"  cT_delta:    mean={np.mean(cT):+.4f} std={np.std(cT):.4f}")
    print(f"  croll_delta: mean={np.mean(croll):+.4f} std={np.std(croll):.4f}")
    print(f"  cpitch_delta:mean={np.mean(cpitch):+.4f} std={np.std(cpitch):.4f}")
    print(f"  cyaw_delta:  mean={np.mean(cyaw):+.4f} std={np.std(cyaw):.4f}")
    return dict(delta_mean=np.mean(delta_arr, axis=0).tolist(),
                cT=float(np.mean(cT)), croll=float(np.mean(croll)),
                cpitch=float(np.mean(cpitch)), cyaw=float(np.mean(cyaw)),
                n=len(sub))


def plot_windows(rows, nominal_corr, outpath):
    n = len(rows)
    fig, ax = plt.subplots(figsize=(max(10, n * 1.3), 6))
    x = np.arange(n)
    width = 0.18
    colors = ["tab:blue", "tab:orange", "tab:blue", "tab:orange"]  # CCW,CW,CCW,CW
    hatches = [None, "//", None, "//"]
    for i, m in enumerate(MOTOR_LABELS):
        vals = [r["dbar"][i] for r in rows]
        ax.bar(x + (i - 1.5) * width, vals, width, label=m, color=colors[i],
               hatch=hatches[i], edgecolor="black", linewidth=0.4)
    dpred_vals = [r["dpred_mean"] for r in rows]
    ax.plot(x, dpred_vals, "k_", markersize=25, markeredgewidth=2.5,
            label=f"d_pred (corr={nominal_corr})", linestyle="none")
    labels = [f"{r['file'][12:27]}\nw{r['wi']} ({r['dur']:.0f}s)" for r in rows]
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=7, rotation=20, ha="right")
    ax.set_ylabel("duty [0-1]")
    ax.set_title("Per-motor mean hover duty vs healthy prediction, by quiet window")
    ax.legend(fontsize=8, ncol=3, loc="upper left")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(outpath, dpi=140)
    plt.close(fig)
    print(f"\nSaved plot -> {outpath}")


def main():
    NOMINAL_CORR = 1.12
    print("Physical model (reused from torque_budget.py):")
    print(f"  KAPPA={tb.KAPPA} mass={tb.MASS_KG}kg g={tb.G}")
    print(f"  motor curve Am={tb.MOTOR_AM} Bm={tb.MOTOR_BM} Cm={tb.MOTOR_CM} Ct={tb.MOTOR_CT}")
    print(f"  nominal hover.thrust_corr={NOMINAL_CORR}")

    _selftest_dpred()
    _selftest_hb_residual()

    rows = collect_windows(nominal_corr=NOMINAL_CORR)
    print(f"\nTotal quiet windows found across {len(ALL_FILES)} files: {len(rows)}")
    for fname in ALL_FILES:
        n = sum(1 for r in rows if r["file"] == fname)
        print(f"  {fname}: {n} window(s)")

    print_window_table(rows, NOMINAL_CORR)
    print_decomposition_table(rows)
    hyps = print_hypothesis_table(rows)

    print("\n" + "=" * 100)
    print("Pooled hypothesis tally (which single-motor Ha1 wins per window)")
    print("=" * 100)
    from collections import Counter
    tally = Counter(h["best_motor"] for h in hyps)
    for m, c in tally.most_common():
        print(f"  {m}: best-fit single-motor model in {c}/{len(hyps)} windows")

    print("\n" + "=" * 100)
    print("Step 6: date-group comparison (0627 vs 0629 — NOTE: same physical airframe NOT confirmed)")
    print("=" * 100)
    agg_0627 = aggregate_by_group(rows, "2026-06-27 (4 files)", FILES_0627)
    agg_0629 = aggregate_by_group(rows, "2026-06-29 (2 files)", FILES_0629)

    print("\n" + "=" * 100)
    print("Step 5: thrust_corr sensitivity sweep (1.00 - 1.30)")
    print("=" * 100)
    corr_grid = np.round(np.arange(1.00, 1.31, 0.05), 2)
    sens = corr_sensitivity(rows, corr_grid)
    n_invariant = sum(1 for s in sens if s["invariance_ok"])
    print(f"croll/cpitch/cyaw(Delta) invariance under corr sweep: {n_invariant}/{len(sens)} windows exactly invariant "
          "(<1e-12 duty units) -- CONFIRMS these components depend only on raw duty, not on corr")
    print(f"\n{'file/window':32s} " + "  ".join([f"cT@{c:.2f}" for c in corr_grid])
          + f"   {'corr@Hb=0':>10s} {'corr@Ha2':>9s}  cyaw_fixed")
    for s in sens:
        label = f"{s['file'][12:27]}_w{s['wi']}"
        cts = "  ".join([f"{s['cT_at_corr'][f'{c:.2f}']:+.4f}" for c in corr_grid])
        b = f"{s['corr_for_Hb_boundary']:.3f}" if s["corr_for_Hb_boundary"] is not None else "out-of-range"
        h2 = f"{s['corr_for_Ha2_boundary']:.3f}" if s["corr_for_Ha2_boundary"] is not None else "out-of-range"
        print(f"{label:32s} {cts}   {b:>10s} {h2:>9s}  {s['cyaw_fixed']:+.4f}")

    plot_windows(rows, NOMINAL_CORR, f"{SCRATCH}/hover_duty_permotor.png")

    # Save JSON summary for downstream reuse
    summary = dict(nominal_corr=NOMINAL_CORR, rows=rows,
                    agg_0627=agg_0627, agg_0629=agg_0629,
                    corr_sensitivity=sens)
    with open(f"{SCRATCH}/hover_duty_permotor_summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\nSaved summary -> {SCRATCH}/hover_duty_permotor_summary.json")


if __name__ == "__main__":
    main()
