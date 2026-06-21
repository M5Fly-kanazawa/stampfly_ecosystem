"""
rate_sysid.py — rate-loop identification + PID auto-tuning (sf sysid backend)
rate_sysid.py — レートループ同定＋PID 自動チューニング（sf sysid バックエンド）

Pipeline / 手順:
 1. Load the Data Stream CSV (400 Hz): r = rate_ref_<axis> (the excitation rides
    on it), y = gyro_<axis>.
 2. RECONSTRUCT the rate-loop output u (torque [Nm]) by replaying the firmware's
    exact discrete PID (D-on-M, Tustin, conditional anti-windup — a line-for-line
    port of pid.hpp) on (r, y) with the gains that flew. The 400 Hz stream has no
    torque channel; the controller is deterministic, so u is recoverable exactly.
 3. ETFE: G_hat(jω) = S_ur*(ω)... actually S_uy/S_uu via Welch cross-spectra over
    the excited band.
 4. Parametric fit of G(s) = b·e^{-Ls} / (s(Ts+1)) — b = 1/J effective inverse
    inertia, T = motor/prop lag, L = loop dead time — by Nelder-Mead on a
    log-magnitude + wrapped-phase cost.
 5. TUNE: given (b, T, L) and specs (gain-crossover ωc, phase margin PM), solve
    the firmware's exact PID form C(s) = Kp(1 + 1/(Ti s) + Td s/(η Td s + 1)),
    η = 0.125: Ti is placed below crossover (Ti = ti_factor/ωc), Td solves the
    phase condition by bisection, Kp the magnitude condition. Margins of the
    resulting loop are verified numerically and reported.

 1. Data Stream CSV（400Hz）を読む: r = rate_ref_<axis>（励振が乗っている）、
    y = gyro_<axis>。
 2. ファームの離散 PID（D-on-M・Tustin・条件付き AW — pid.hpp の逐語移植）を
    飛行時ゲインで (r,y) に再生し、レートループ出力 u（トルク [Nm]）を「再構成」
    する。400Hz ストリームにトルクは無いが、制御器は決定論的なので u は厳密に
    復元できる。
 3. ETFE: Welch クロススペクトルで G_hat = S_uy/S_uu（励振帯域のみ）。
 4. G(s) = b·e^{-Ls}/(s(Ts+1)) のパラメトリックフィット（b=有効慣性逆数、
    T=モータ/プロペラ遅れ、L=むだ時間）。対数振幅＋折返し位相コストの Nelder-Mead。
 5. チューニング: (b,T,L) と仕様（ゲイン交差 ωc・位相余裕 PM）から、ファームの
    PID 形 C(s)=Kp(1+1/(Ti s)+Td s/(η Td s+1)), η=0.125 を解く: Ti は交差より
    下に配置（ti_factor/ωc）、Td は位相条件の二分法、Kp は振幅条件。得られた
    ループの余裕は数値検証して報告する。

Mechanical-spec default plant / 機械仕様ベースの既定プラント:
    b = 1/J  (J: Ixx 9.16e-6 / Iyy 13.3e-6 / Izz 20.4e-6 kg m²,
              docs/architecture/stampfly-parameters.md)
    T = 0.02 s (motor+prop thrust lag — Model Identity with the SIL plant's
                motor_tau; plant.hpp)
    L = 0.005 s (1.5 control periods @400 Hz + BMI270 OSR4 group delay)
"""

import json
import math

import numpy as np

ETA = 0.125                  # firmware incomplete-derivative coefficient / 不完全微分係数
FS = 400.0                   # Data Stream rate [Hz]
DT = 1.0 / FS

# Mechanical-spec defaults / 機械仕様の既定値
SPEC_INERTIA = {"roll": 9.16e-6, "pitch": 13.3e-6, "yaw": 20.4e-6}   # [kg m^2]
SPEC_MOTOR_T = 0.02          # [s] motor/prop lag (SIL plant motor_tau — Model Identity)
SPEC_DELAY_L = 0.005         # [s] loop dead time (1.5 ctrl periods + IMU filter)

# Flight-proven firmware gains (params.cpp defaults) used for the u replay.
# u 再構成に使う飛行時ゲイン（params.cpp 既定 = 実績値）。
DEFAULT_GAINS = {
    "roll":  {"kp": 1.365e-3, "ti": 0.7, "td": 0.01, "limit": 5.2e-3},
    "pitch": {"kp": 1.995e-3, "ti": 0.7, "td": 0.01, "limit": 5.2e-3},
    "yaw":   {"kp": 5.31e-3,  "ti": 1.6, "td": 0.01, "limit": 2.2e-3},
}


# =============================================================================
# 1-2. Firmware PID replay (line-for-line port of pid.hpp compute())
#      ファーム PID の再生（pid.hpp compute() の逐語移植）
# =============================================================================

def replay_pid(r, y, kp, ti, td, limit, dt=DT):
    """Reconstruct u[k] from (r[k], y[k]) with the firmware's exact discrete PID.
    ファームの離散 PID で (r,y) から u を厳密再構成する。"""
    n = len(r)
    u = np.zeros(n)
    integral = 0.0
    deriv_filter = 0.0
    prev_error = 0.0
    prev_meas = 0.0
    first_run = True
    for k in range(n):
        error = r[k] - y[k]
        p_term = kp * error
        d_term = 0.0
        if td > 0.0:
            if first_run:
                prev_meas = y[k]
            else:
                alpha = 2.0 * ETA * td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * td / ((alpha + 1.0) * dt)
                deriv_filter = a * deriv_filter - b * (y[k] - prev_meas)
                d_term = kp * deriv_filter
        prev_meas = y[k]
        first_run = False
        if ti >= 0.01:
            i_next = integral + (kp / ti) * (error + prev_error) * (dt * 0.5)
            out_test = p_term + i_next + d_term
            push_high = (out_test > limit) and (error > 0)
            push_low = (out_test < -limit) and (error < 0)
            if not (push_high or push_low):
                integral = i_next
            integral = max(-limit, min(limit, integral))
        prev_error = error
        u[k] = max(-limit, min(limit, p_term + integral + d_term))
    return u


# =============================================================================
# 3. ETFE via Welch cross-spectra / Welch クロススペクトルの ETFE
# =============================================================================

def etfe(u, y, fs=FS, f_lo=0.8, f_hi=30.0, nperseg=1024):
    """Empirical transfer function S_uy/S_uu over [f_lo, f_hi].
    励振帯域の経験伝達関数。Returns (omega [rad/s], G complex, coherence)."""
    n = len(u)
    nperseg = min(nperseg, n)
    step = nperseg // 2
    win = np.hanning(nperseg)
    suu = syu = syy = None
    count = 0
    for start in range(0, n - nperseg + 1, step):
        uw = (u[start:start + nperseg] - np.mean(u[start:start + nperseg])) * win
        yw = (y[start:start + nperseg] - np.mean(y[start:start + nperseg])) * win
        fu = np.fft.rfft(uw)
        fy = np.fft.rfft(yw)
        if suu is None:
            suu = np.abs(fu) ** 2
            syu = fy * np.conj(fu)
            syy = np.abs(fy) ** 2
        else:
            suu += np.abs(fu) ** 2
            syu += fy * np.conj(fu)
            syy += np.abs(fy) ** 2
        count += 1
    if count == 0:
        raise ValueError("log too short for ETFE")
    freqs = np.fft.rfftfreq(nperseg, 1.0 / fs)
    mask = (freqs >= f_lo) & (freqs <= f_hi) & (suu > 0)
    G = syu[mask] / suu[mask]
    coh = np.abs(syu[mask]) ** 2 / (suu[mask] * syy[mask] + 1e-30)
    return 2.0 * np.pi * freqs[mask], G, coh


# =============================================================================
# 4. Parametric fit / パラメトリックフィット
# =============================================================================

def plant_response(omega, b, T, L):
    """G(jω) = b e^{-jωL} / (jω (jωT + 1))"""
    jw = 1j * omega
    return b * np.exp(-jw * L) / (jw * (jw * T + 1.0))


def _fit_cost(params, omega, G_hat, weights):
    b, T, L = params
    if b <= 0 or T <= 0 or L < 0:
        return 1e9
    Gm = plant_response(omega, b, T, L)
    dmag = np.log(np.abs(G_hat) + 1e-12) - np.log(np.abs(Gm) + 1e-12)
    dph = np.angle(G_hat / Gm)          # wrapped phase difference / 折返し位相差
    return float(np.sum(weights * (dmag ** 2 + dph ** 2)))


def _nelder_mead(f, x0, scale, iters=400):
    """Tiny dependency-free Nelder-Mead (3 params). / 依存なしの小型 Nelder-Mead。"""
    n = len(x0)
    simplex = [np.array(x0, dtype=float)]
    for i in range(n):
        p = np.array(x0, dtype=float)
        p[i] += scale[i]
        simplex.append(p)
    vals = [f(p) for p in simplex]
    for _ in range(iters):
        order = np.argsort(vals)
        simplex = [simplex[i] for i in order]
        vals = [vals[i] for i in order]
        centroid = np.mean(simplex[:-1], axis=0)
        xr = centroid + (centroid - simplex[-1])           # reflect
        fr = f(xr)
        if fr < vals[0]:
            xe = centroid + 2.0 * (centroid - simplex[-1])  # expand
            fe = f(xe)
            simplex[-1], vals[-1] = (xe, fe) if fe < fr else (xr, fr)
        elif fr < vals[-2]:
            simplex[-1], vals[-1] = xr, fr
        else:
            xc = centroid + 0.5 * (simplex[-1] - centroid)  # contract
            fc = f(xc)
            if fc < vals[-1]:
                simplex[-1], vals[-1] = xc, fc
            else:                                           # shrink
                for i in range(1, n + 1):
                    simplex[i] = simplex[0] + 0.5 * (simplex[i] - simplex[0])
                    vals[i] = f(simplex[i])
    best = int(np.argmin(vals))
    return simplex[best], vals[best]


def fit_plant(omega, G_hat, coherence, axis="roll"):
    """Fit (b, T, L), weighting by coherence. / コヒーレンス重み付きで (b,T,L) を推定。"""
    weights = np.clip(coherence, 0.0, 1.0) ** 2
    x0 = [1.0 / SPEC_INERTIA[axis], SPEC_MOTOR_T, SPEC_DELAY_L]
    best, cost = _nelder_mead(
        lambda p: _fit_cost(p, omega, G_hat, weights),
        x0, scale=[x0[0] * 0.5, 0.02, 0.004])
    b, T, L = (float(v) for v in best)
    return {"b": b, "T": T, "L": L, "cost": cost,
            "inertia_eff": 1.0 / b, "n_points": int(len(omega)),
            "coherence_mean": float(np.mean(coherence))}


# =============================================================================
# 5. PID auto-tuning for the firmware's exact form
#    ファーム PID 形そのものの自動チューニング
# =============================================================================

def pid_freq(omega, kp, ti, td, eta=ETA):
    """C(jω) of the firmware PID. / ファーム PID の周波数応答。"""
    jw = 1j * omega
    c = 1.0 + 1.0 / (jw * ti)
    if td > 0:
        c = c + jw * td / (1.0 + jw * eta * td)
    return kp * c


def tune_pid(b, T, L, wc, pm_deg, ti_factor=10.0, eta=ETA):
    """Solve Kp, Ti, Td so |C·G(jωc)| = 1 and arg = −180° + PM.
    |C·G(jωc)|=1 と位相条件を満たす Kp, Ti, Td を解く。"""
    ti = ti_factor / wc
    g_c = plant_response(np.array([wc]), b, T, L)[0]
    phi_g = math.degrees(math.atan2(g_c.imag, g_c.real))
    phi_needed = -180.0 + pm_deg - phi_g          # required controller phase [deg]

    def c_phase(td):
        c = pid_freq(np.array([wc]), 1.0, ti, td, eta)[0]
        return math.degrees(math.atan2(c.imag, c.real))

    # C's phase is NOT monotonic in Td (the lead peak moves past wc): find the
    # peak first, then bisect on the ASCENDING segment [0, Td_peak] only.
    # C の位相は Td に単調でない（リードのピークが wc を跨ぐ）: まずピークを探し、
    # 単調上昇区間 [0, Td_peak] でのみ二分する。
    td_grid = np.logspace(-4, math.log10(4.0 / (eta * wc)), 200)
    phases = [c_phase(td_k) for td_k in td_grid]
    i_peak = int(np.argmax(phases))
    td_peak, phase_max = td_grid[i_peak], phases[i_peak]
    if phi_needed <= c_phase(0.0):
        td = 0.0                                   # integral lag alone suffices
    elif phi_needed > phase_max:
        raise ValueError(
            f"required lead {phi_needed:.1f} deg exceeds the PID maximum "
            f"{phase_max:.1f} deg at wc={wc:.1f} rad/s — lower --wc or --pm "
            f"(plant phase {phi_g:.1f} deg there)")
    else:
        lo, hi = 0.0, td_peak
        for _ in range(60):                        # bisection / 二分法
            mid = 0.5 * (lo + hi)
            if c_phase(mid) < phi_needed:
                lo = mid
            else:
                hi = mid
        td = 0.5 * (lo + hi)

    c_unit = pid_freq(np.array([wc]), 1.0, ti, td, eta)[0]
    kp = 1.0 / (abs(c_unit) * abs(g_c))

    return {"kp": float(kp), "ti": float(ti), "td": float(td),
            "achieved": loop_margins(b, T, L, kp, ti, td, eta)}


def loop_margins(b, T, L, kp, ti, td, eta=ETA):
    """Numerically verify crossover/PM/GM of C·G. / 交差・余裕の数値検証。"""
    omega = np.logspace(-1, math.log10(FS * math.pi), 4000)
    Lw = pid_freq(omega, kp, ti, td, eta) * plant_response(omega, b, T, L)
    mag = np.abs(Lw)
    ph = np.unwrap(np.angle(Lw))
    # gain crossover: |L|=1 / ゲイン交差
    idx = np.where(np.diff(np.sign(mag - 1.0)))[0]
    out = {"wc": None, "pm_deg": None, "gm_db": None, "w180": None}
    if len(idx):
        i = idx[0]
        f = (1.0 - mag[i]) / (mag[i + 1] - mag[i])
        wc = omega[i] * (omega[i + 1] / omega[i]) ** f
        phc = ph[i] + f * (ph[i + 1] - ph[i])
        out["wc"] = float(wc)
        out["pm_deg"] = float(math.degrees(phc) + 180.0)
    # phase crossover: arg = −180° / 位相交差
    target = -math.pi
    idx = np.where(np.diff(np.sign(ph - target)))[0]
    if len(idx):
        i = idx[0]
        f = (target - ph[i]) / (ph[i + 1] - ph[i])
        w180 = omega[i] * (omega[i + 1] / omega[i]) ** f
        m180 = mag[i] * (mag[i + 1] / mag[i]) ** f
        out["w180"] = float(w180)
        out["gm_db"] = float(-20.0 * math.log10(m180))
    return out


# =============================================================================
# CSV front-end / CSV 入口
# =============================================================================

def load_csv(path, axis):
    """Pull (t, r, y) for one axis from a Data Stream CSV (sf log convert).
    Data Stream CSV から1軸分の (t, r, y) を取り出す。"""
    import csv as _csv
    col_r = f"rate_ref_{axis}"
    col_y = {"roll": "gyro_x", "pitch": "gyro_y", "yaw": "gyro_z"}[axis]
    t, r, y = [], [], []
    with open(path) as f:
        for row in _csv.DictReader(f):
            try:
                r.append(float(row[col_r]))
                y.append(float(row[col_y]))
                t.append(float(row.get("timestamp", len(t))))
            except (KeyError, ValueError):
                continue
    if len(r) < 1024:
        raise ValueError(f"too few samples ({len(r)}) — capture with the "
                         "excitation running (sf log wifi + api sysid)")
    return np.array(t), np.array(r), np.array(y)


def plot_fit(omega, G_hat, coh, b, T, L, axis, path):
    """Bode (measured ETFE vs fitted model) + coherence, saved to `path` (PNG).
    Bode（実測 ETFE vs フィット）＋コヒーレンスを path に保存。コヒーレンスが低い帯域＝外乱支配で
    フィットが信用できない箇所が一目で分かる（γ²=0.6 は CIFER 流の採否しきい線）。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    f = omega / (2.0 * np.pi)
    G_fit = plant_response(omega, b, T, L)
    fig, ax = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    ax[0].semilogx(f, 20 * np.log10(np.abs(G_hat) + 1e-12), ".", color="tab:blue",
                   label="measured (ETFE)")
    ax[0].semilogx(f, 20 * np.log10(np.abs(G_fit) + 1e-12), "-", color="tab:red",
                   label=f"fit  b={b:.0f}  T={T*1e3:.1f}ms  L={L*1e3:.2f}ms")
    ax[0].set_ylabel("|G|  [dB]"); ax[0].grid(True, which="both", alpha=0.3)
    ax[0].legend(fontsize=8); ax[0].set_title(f"{axis} rate-loop plant — Bode + coherence")
    ax[1].semilogx(f, np.unwrap(np.angle(G_hat)) * 180 / np.pi, ".", color="tab:blue")
    ax[1].semilogx(f, np.unwrap(np.angle(G_fit)) * 180 / np.pi, "-", color="tab:red")
    ax[1].set_ylabel("phase  [deg]"); ax[1].grid(True, which="both", alpha=0.3)
    ax[2].semilogx(f, np.clip(coh, 0, 1), "-", color="tab:green")
    ax[2].axhline(0.6, color="gray", ls="--", lw=1, label="γ²=0.6 (CIFER gate)")
    ax[2].set_ylabel("coherence γ²"); ax[2].set_xlabel("frequency  [Hz]")
    ax[2].set_ylim(0, 1.05); ax[2].grid(True, which="both", alpha=0.3); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def fit_from_csv(path, axis, gains=None, f_lo=0.8, f_hi=30.0, plot_path=None):
    """Full pipeline: CSV → replay u → ETFE → parametric (b, T, L).
    全手順: CSV → u 再生 → ETFE → (b,T,L)。plot_path 指定で Bode＋コヒーレンス図を保存。"""
    g = dict(DEFAULT_GAINS[axis])
    if gains:
        g.update(gains)
    _t, r, y = load_csv(path, axis)
    u = replay_pid(r, y, g["kp"], g["ti"], g["td"], g["limit"])
    omega, G_hat, coh = etfe(u, y, f_lo=f_lo, f_hi=f_hi)
    result = fit_plant(omega, G_hat, coh, axis)
    result["axis"] = axis
    result["gains_used"] = g
    if plot_path:
        plot_fit(omega, G_hat, coh, result["b"], result["T"], result["L"], axis, plot_path)
        result["plot_path"] = str(plot_path)
    return result


# =============================================================================
# Self-test: synthesize a flight from a KNOWN plant, recover it, tune it.
# 自己テスト: 既知プラントから飛行を合成し、復元・チューニングを検証。
# =============================================================================

def selftest(verbose=True):
    rng = np.random.default_rng(7)
    b_true, T_true, L_true = 1.0 / 9.16e-6, 0.025, 0.006
    g = DEFAULT_GAINS["roll"]
    n = 8000                                       # 20 s at 400 Hz
    t = np.arange(n) * DT
    # log chirp 1→25 Hz over 6 s, repeated — same waveform the firmware injects
    k = (25.0 / 1.0) ** (1.0 / 6.0)
    r = 0.4 * np.sin(2 * np.pi * 1.0 * ((k ** (t % 6.0)) - 1) / np.log(k))

    # discrete closed-loop sim: plant b/(s(Ts+1)) via two integrators + delay
    delay = int(round(L_true / DT))
    ubuf = np.zeros(delay + 1)
    y = np.zeros(n)
    u = np.zeros(n)
    rate = 0.0
    acc_state = 0.0                                # motor-lag state (alpha filt)
    integral = 0.0
    deriv_filter = 0.0
    prev_error = 0.0
    prev_meas = 0.0
    first = True
    for kk in range(n):
        yk = rate + rng.normal(0, 0.002)           # gyro noise / ジャイロノイズ
        y[kk] = yk
        # firmware PID (same as replay) / ファーム PID（replay と同一）
        error = r[kk] - yk
        p_term = g["kp"] * error
        d_term = 0.0
        if first:
            prev_meas = yk
        else:
            alpha = 2.0 * ETA * g["td"] / DT
            a = (alpha - 1.0) / (alpha + 1.0)
            bb = 2.0 * g["td"] / ((alpha + 1.0) * DT)
            deriv_filter = a * deriv_filter - bb * (yk - prev_meas)
            d_term = g["kp"] * deriv_filter
        prev_meas = yk
        first = False
        i_next = integral + (g["kp"] / g["ti"]) * (error + prev_error) * (DT * 0.5)
        out_test = p_term + i_next + d_term
        if not ((out_test > g["limit"] and error > 0) or
                (out_test < -g["limit"] and error < 0)):
            integral = i_next
        integral = max(-g["limit"], min(g["limit"], integral))
        prev_error = error
        uk = max(-g["limit"], min(g["limit"], p_term + integral + d_term))
        u[kk] = uk
        # plant: delay → motor lag → inertia integration
        ubuf = np.roll(ubuf, 1)
        ubuf[0] = uk
        ud = ubuf[-1]
        acc_state += DT / T_true * (ud * b_true - acc_state)
        rate += acc_state * DT

    u_rec = replay_pid(r, y, g["kp"], g["ti"], g["td"], g["limit"])
    err_u = float(np.max(np.abs(u_rec - u)))
    omega, G_hat, coh = etfe(u_rec, y)
    fit = fit_plant(omega, G_hat, coh, "roll")
    tune = tune_pid(fit["b"], fit["T"], fit["L"], wc=20.0, pm_deg=60.0)
    ach = tune["achieved"]
    ok = (err_u < 1e-9 and
          abs(fit["b"] / b_true - 1) < 0.15 and
          abs(fit["T"] / T_true - 1) < 0.30 and
          abs(fit["L"] - L_true) < 0.004 and
          abs(ach["wc"] / 20.0 - 1) < 0.05 and
          abs(ach["pm_deg"] - 60.0) < 3.0)
    if verbose:
        print(f"replay max|u_rec-u| = {err_u:.2e} (exactness of the PID port)")
        print(f"fit : b={fit['b']:.0f} (true {b_true:.0f})  "
              f"T={fit['T'] * 1000:.1f}ms (true {T_true * 1000:.1f})  "
              f"L={fit['L'] * 1000:.2f}ms (true {L_true * 1000:.2f})  "
              f"coh={fit['coherence_mean']:.2f}")
        print(f"tune: kp={tune['kp']:.3e} ti={tune['ti']:.3f} td={tune['td']:.4f}")
        print(f"      achieved wc={ach['wc']:.1f} rad/s pm={ach['pm_deg']:.1f} deg "
              f"gm={ach['gm_db']:.1f} dB")
        print("SELFTEST:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)
