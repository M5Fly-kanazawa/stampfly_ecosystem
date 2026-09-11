"""
rate_sysid.py — rate-loop identification + PID auto-tuning (sf sysid backend)
rate_sysid.py — レートループ同定＋PID 自動チューニング（sf sysid バックエンド）

Pipeline / 手順:
 1. Load the Data Stream CSV (400 Hz): r = rate_ref_<axis> (the excitation rides
    on it), y = gyro_<axis>.
 2. RECOVER the rate-loop output u (torque [Nm]). The PRIMARY path reads the
    400 Hz motor-duty entry (kPktDuty400) directly and inverts firmware/vehicle's
    real mixer (B^-1 allocation + nonlinear motor curve, sf_actuator/
    actuator.cpp) -- exact, and correct even if the firmware gains are unknown
    or were changed mid-flight (shared implementation with `sf sysid fit
    --mixer vehicle`, see sysid.plant_fit). Only when a log has no genuine
    400 Hz duty (older firmware/capture) does this fall back to REPLAYING the
    firmware's exact discrete PID (D-on-M, Tustin, conditional anti-windup —
    a line-for-line port of pid.hpp) on (r, y) with the gains that flew --
    this legacy path requires knowing those gains and assumes the firmware PID
    math exactly, so prefer the duty path whenever it's available.
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
 2. レートループ出力 u（トルク [Nm]）を「復元」する。**基本の経路**は400Hzの
    モータduty エントリ（kPktDuty400）を直接読み、firmware/vehicle の実際の
    ミキサー（B^-1配分＋非線形モータ曲線、sf_actuator/actuator.cpp）を逆算する
    ——厳密で、ファームのゲインが未知でも・飛行中に変更されていても正しい
    （`sf sysid fit --mixer vehicle` と実装を共有、sysid.plant_fit 参照）。
    本物の400Hz duty が無い（旧ファーム/旧キャプチャの）ログのときだけ、
    ファームの離散 PID（D-on-M・Tustin・条件付き AW — pid.hpp の逐語移植）を
    飛行時ゲインで (r,y) に再生する経路にフォールバックする——こちらはゲイン
    を知っている必要があり、ファーム PID の数式をそのまま仮定するため、
    duty 経路が使えるときは常にそちらを優先する。
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
    T = 0.02 s (motor+prop thrust lag — real-hardware structural default; as of
                2026-07-26 the SILS plant's own lag comes from its motor ODE's
                electromechanical time constant, not a hand-set T, see plant.hpp)
    L = 0.005 s (1.5 control periods @400 Hz + BMI270 OSR4 group delay)
"""

import json
import math
import sys as _sys
from pathlib import Path as _Path

import numpy as np

# Import the duty->torque physics from sysid.plant_fit (--mixer vehicle) so
# this module and `sf sysid fit` share ONE implementation of firmware/
# vehicle's actual mixer inversion instead of two copies that can silently
# drift apart (exactly the bug this whole effort started from -- see
# _duty_differential_vehicle()'s docstring in plant_fit.py).
# duty->トルクの物理計算は sysid.plant_fit（--mixer vehicle）から import し、
# このモジュールと `sf sysid fit` が firmware/vehicle 実ミキサー逆算の実装を
# 1つだけ共有する（2つのコピーが黙って食い違う——今回の一連の作業の発端と
# 同じバグ——のを防ぐ。plant_fit.py の _duty_differential_vehicle() の
# docstring参照）。
_TOOLS_DIR = str(_Path(__file__).resolve().parent.parent)
if _TOOLS_DIR not in _sys.path:
    _sys.path.insert(0, _TOOLS_DIR)
from sysid.plant_fit import (  # noqa: E402
    _duty_differential_vehicle, _classify_duty_source,
    _V_BATT_NOMINAL, _V_BATT_MIN,
    _ARM_D, _MOTOR_AM, _MOTOR_BM, _MOTOR_CM, _MOTOR_CT,
)

# rate_stream.csv (written by simulator/sils/devices/emu_rate_stream.cpp) is the
# OLD flat Data Stream CSV layout, NOT a flight-log-bundle. This is intentionally
# NOT migrated in this phase (docs/plans/flight-log-format-plan.md section 3.3
# schedules the SILS/emulator side for Phase 3). These three constants used to be
# imported from sysid.plant_fit, but that module's equivalent constants now
# describe the NEW bundle-derived aligned DataFrame's column names instead (e.g.
# 'duty_FR'/'voltage', not 'motor_duty_FR'/'vbat') -- kept here as local literals
# so this file's CSV path (load_csv()/fit_from_csv(), used only by
# lib/sfcli/commands/sils.py's `sysid-gate`) keeps working unchanged until
# Phase 3 replaces it.
# rate_stream.csv（simulator/sils/devices/emu_rate_stream.cpp が書く）は旧来の
# フラット Data Stream CSV 形式であり、フライトログ一式ではない。本フェーズでは
# 意図的に移行しない（docs/plans/flight-log-format-plan.md 3.3節で SILS/
# エミュレータ側は Phase 3 と規定）。この3定数は以前 sysid.plant_fit から
# import していたが、あちらの同等の定数は今や新しい一式由来の整列済み
# DataFrame の列名を指す（'motor_duty_FR'/'vbat' ではなく 'duty_FR'/
# 'voltage'）—— このファイルの CSV 経路（load_csv()/fit_from_csv()。
# lib/sfcli/commands/sils.py の `sysid-gate` のみが使う）が Phase 3 まで
# そのまま動くよう、ここにローカルなリテラルとして保持する。
_DUTY_COLS = ('motor_duty_FR', 'motor_duty_RR', 'motor_duty_RL', 'motor_duty_FL')
_DUTY_RATE_HZ_COL = 'duty_rate_hz'
_VBAT_COL = 'vbat'

ETA = 0.125                  # firmware incomplete-derivative coefficient / 不完全微分係数
FS = 400.0                   # Data Stream rate [Hz]
DT = 1.0 / FS

# Mechanical-spec defaults / 機械仕様の既定値
SPEC_INERTIA = {"roll": 9.16e-6, "pitch": 13.3e-6, "yaw": 20.4e-6}   # [kg m^2]
SPEC_MOTOR_T = 0.02          # [s] motor/prop lag (real-hardware structural default; see module docstring)
SPEC_DELAY_L = 0.005         # [s] loop dead time (1.5 ctrl periods + IMU filter)

# Flight-proven firmware gains (params.cpp defaults) used for the u replay.
# Synced with firmware/vehicle/components/sf_core/params.cpp table[] (checked
# 2026-09-06; roll/pitch/yaw kp/ti/td at that file's lines 712-720, roll/pitch
# limit = max_roll_pitch_torque_ in pid_controller.hpp, yaw limit =
# rate.yaw.max_torque). Previous values here (kp 1.365e-3/1.995e-3/5.31e-3,
# yaw ti 1.6, yaw limit 2.2e-3) were stale relative to the current firmware.
# u 再構成に使う飛行時ゲイン（params.cpp 既定 = 実績値）。
# firmware/vehicle/components/sf_core/params.cpp の table[]（2026-09-06 確認、
# 712〜720行目）と同期。roll/pitch の limit は pid_controller.hpp の
# max_roll_pitch_torque_、yaw の limit は rate.yaw.max_torque。旧値
# （kp 1.365e-3/1.995e-3/5.31e-3、yaw ti 1.6、yaw limit 2.2e-3）は
# 現行ファームより古い値だった。
DEFAULT_GAINS = {
    "roll":  {"kp": 1.0e-3,      "ti": 0.7, "td": 0.002, "limit": 5.2e-3},
    "pitch": {"kp": 1.426432e-3, "ti": 0.7, "td": 0.025, "limit": 5.2e-3},
    "yaw":   {"kp": 8.029796e-4, "ti": 0.8, "td": 0.01,  "limit": 1.226e-3},
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
#
# Phase 3 removes this (load_csv() and fit_from_csv()) once the SILS emulator
# writes a flight-log bundle instead of this flat CSV
# (docs/plans/flight-log-format-plan.md section 3.3). Until then this is the
# ONLY consumer of load_csv(), called exclusively by
# lib/sfcli/commands/sils.py's `sysid-gate` (SILS model-match gate) --
# real-vehicle-log tooling uses fit_from_df() instead (see below).
# Phase 3 でこれ（load_csv()・fit_from_csv()）は削除される。SILS エミュレータが
# このフラット CSV の代わりに一式を書くようになった時点で
# （docs/plans/flight-log-format-plan.md 3.3節）。それまでは load_csv() の
# 唯一の呼び出し元は lib/sfcli/commands/sils.py の `sysid-gate`（SILS
# モデル一致ゲート）だけ —— 実機ログ向けツールは代わりに fit_from_df() を使う
# （下記参照）。
# =============================================================================

def load_csv(path, axis):
    """Pull (t, r, y, duty_u, duty_quality, duty_reason) for one axis from a
    Data Stream CSV (sf log convert).
    Data Stream CSV から1軸分の (t, r, y, duty_u, duty_quality, duty_reason) を
    取り出す。

    duty_u is the differential TORQUE [Nm] recovered by inverting firmware/
    vehicle's real mixer on the 400Hz motor_duty_FR/RR/RL/FL columns (see
    module docstring) -- None if those columns aren't in the CSV. duty_quality
    is 'duty400' (genuine 400Hz, safe to use as u directly), 'duty50' (an
    older 50Hz-forward-filled staircase -- too coarse, caller must fall back
    to replay_pid()), or None when duty_u is None (duty_reason explains why).
    duty_u はfirmware/vehicle の実ミキサーを400Hzの motor_duty_FR/RR/RL/FL 列
    から逆算した差動トルク[Nm]（モジュール docstring 参照）——CSV にその列が
    無ければ None。duty_quality は 'duty400'（本物の400Hz、そのまま u として
    使える）、'duty50'（旧い50Hz前方補完の階段状データ——粗すぎるので呼び出し
    側は replay_pid() にフォールバックすること）、または duty_u が None の
    ときの None（理由は duty_reason）。
    """
    import csv as _csv
    col_r = f"rate_ref_{axis}"
    col_y = {"roll": "gyro_x", "pitch": "gyro_y", "yaw": "gyro_z"}[axis]
    t, r, y = [], [], []
    duty_fr, duty_rr, duty_rl, duty_fl = [], [], [], []
    duty_rate_hz, vbat = [], []
    with open(path) as f:
        reader = _csv.DictReader(f)
        fieldnames = set(reader.fieldnames or [])
        has_duty_cols = all(c in fieldnames for c in _DUTY_COLS)
        has_duty_rate_hz = _DUTY_RATE_HZ_COL in fieldnames
        has_vbat = _VBAT_COL in fieldnames
        for row in reader:
            try:
                r.append(float(row[col_r]))
                y.append(float(row[col_y]))
            except (KeyError, ValueError):
                continue
            t.append(len(t) * DT)
            if has_duty_cols:
                try:
                    duty_fr.append(float(row['motor_duty_FR']))
                    duty_rr.append(float(row['motor_duty_RR']))
                    duty_rl.append(float(row['motor_duty_RL']))
                    duty_fl.append(float(row['motor_duty_FL']))
                except (KeyError, ValueError):
                    duty_fr.append(0.0); duty_rr.append(0.0)
                    duty_rl.append(0.0); duty_fl.append(0.0)
            if has_duty_rate_hz:
                try:
                    duty_rate_hz.append(float(row[_DUTY_RATE_HZ_COL]))
                except (KeyError, ValueError):
                    duty_rate_hz.append(0.0)
            if has_vbat:
                try:
                    vbat.append(float(row[_VBAT_COL]))
                except (KeyError, ValueError):
                    vbat.append(0.0)
    if len(r) < 1024:
        raise ValueError(f"too few samples ({len(r)}) — capture with the "
                         "excitation running (sf log wifi + api sysid)")

    duty_u = duty_quality = None
    duty_reason = "no motor_duty_FR/RR/RL/FL columns in CSV"
    if has_duty_cols:
        duty_fr = np.array(duty_fr); duty_rr = np.array(duty_rr)
        duty_rl = np.array(duty_rl); duty_fl = np.array(duty_fl)
        duty_rate_hz_arr = np.array(duty_rate_hz) if has_duty_rate_hz else None
        duty_quality, duty_reason = _classify_duty_source(
            duty_fr, duty_rr, duty_rl, duty_fl, duty_rate_hz_arr)

        # vbat forward-filled from the 1Hz PKT_STATUS entry (udp_capture.py
        # save_stream_csv()); nominal 1S LiPo fallback when absent/implausible,
        # same policy as sysid.plant_fit._load_axis_data()'s --mixer vehicle path.
        # vbat は1Hz PKT_STATUS からの前方補完（udp_capture.py 参照）。無い/
        # 非現実的な場合は公称1S LiPo電圧にフォールバック
        # （sysid.plant_fit._load_axis_data() の --mixer vehicle 経路と同じ方針）。
        vbat_note = ''
        if has_vbat:
            vbat_arr = np.array(vbat)
            bad = vbat_arr < _V_BATT_MIN
            if np.any(bad):
                vbat_arr = np.where(bad, _V_BATT_NOMINAL, vbat_arr)
                vbat_note = (f" ({int(np.sum(bad))}/{len(vbat_arr)} rows had no/"
                             f"implausible vbat -- used the nominal {_V_BATT_NOMINAL}V there)")
        else:
            vbat_arr = np.full(len(duty_fr), _V_BATT_NOMINAL)
            vbat_note = (f" (no vbat column in CSV -- used the nominal "
                         f"{_V_BATT_NOMINAL}V throughout; re-capture with a current "
                         "udp_capture.py for the real battery-sag-corrected fit)")
        duty_reason += vbat_note
        duty_u = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat_arr, axis)

    return np.array(t), np.array(r), np.array(y), duty_u, duty_quality, duty_reason


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


def _fit_core(r, y, duty_u, duty_quality, duty_reason, axis, gains, f_lo, f_hi,
              plot_path, input_mode):
    """Shared tail of the pipeline, from "recover u" onward: input-mode
    resolution -> ETFE -> parametric (b, T, L) -> optional plot. Both
    fit_from_csv() (SILS rate_stream.csv) and fit_from_df() (real-vehicle
    flight-log bundle) build (r, y, duty_u, duty_quality, duty_reason) their
    own way, then call this ONE function so the two front-ends can never
    silently diverge in how they resolve/report the fit.
    パイプラインの後半（「u を復元」以降）の共有部分: 入力モード解決 -> ETFE ->
    パラメトリックフィット (b,T,L) -> 任意のプロット。fit_from_csv()（SILS の
    rate_stream.csv）と fit_from_df()（実機フライトログ一式）はそれぞれ自分の
    方法で (r, y, duty_u, duty_quality, duty_reason) を組み立てた後、この1つの
    関数を呼ぶ -- 2つの入口がフィットの解決・報告のしかたで黙って食い違うことが
    ないようにするため。
    """
    if input_mode not in ('auto', 'duty', 'kp'):
        raise ValueError(f"Unknown input_mode: {input_mode!r}. Choose from: auto, duty, kp")

    g = dict(DEFAULT_GAINS[axis])
    if gains:
        g.update(gains)

    resolved_mode = input_mode
    if resolved_mode == 'auto':
        resolved_mode = 'duty' if (duty_u is not None and duty_quality == 'duty400') else 'kp'

    if resolved_mode == 'duty':
        if duty_u is None:
            raise ValueError(
                "--input duty requested but this log has no genuine motor-duty "
                f"data ({duty_reason}). Capture with firmware sending the 400Hz "
                "duty entry (kPktDuty400), or pass --input kp.")
        if duty_quality != 'duty400':
            raise ValueError(
                f"--input duty requested but this log's duty is not genuine "
                f"400Hz data ({duty_reason}). Pass --input kp instead (needs "
                "--kp/--ti/--td for the firmware gains that flew).")
        u = duty_u
    else:
        u = replay_pid(r, y, g["kp"], g["ti"], g["td"], g["limit"])

    omega, G_hat, coh = etfe(u, y, f_lo=f_lo, f_hi=f_hi)
    result = fit_plant(omega, G_hat, coh, axis)
    result["axis"] = axis
    result["input_mode"] = resolved_mode
    result["duty_reason"] = duty_reason
    if resolved_mode == 'kp':
        result["gains_used"] = g
    if plot_path:
        plot_fit(omega, G_hat, coh, result["b"], result["T"], result["L"], axis, plot_path)
        result["plot_path"] = str(plot_path)
    return result


def fit_from_csv(path, axis, gains=None, f_lo=0.8, f_hi=30.0, plot_path=None,
                  input_mode='auto'):
    """Full pipeline: CSV → recover u → ETFE → parametric (b, T, L).
    全手順: CSV → u 復元 → ETFE → (b,T,L)。plot_path 指定で Bode＋コヒーレンス図を保存。

    Thin wrapper over _fit_core(): only load_csv()'s CSV-reading differs from
    fit_from_df(). Phase 3 removes this once the SILS emulator writes a
    flight-log bundle instead of this flat CSV
    (docs/plans/flight-log-format-plan.md section 3.3) -- until then this is
    the ONLY consumer of load_csv() and is called exclusively by
    lib/sfcli/commands/sils.py's `sysid-gate` (SILS model-match gate), never
    by real-vehicle-log tooling (which uses fit_from_df() instead).
    _fit_core() への薄いラッパー: fit_from_df() との違いは load_csv() の
    CSV読み込み部分だけ。Phase 3 で削除される（SILS エミュレータがこのフラット
    CSV の代わりに一式を書くようになった時点、
    docs/plans/flight-log-format-plan.md 3.3節）—— それまでは load_csv() の
    唯一の呼び出し元であり、lib/sfcli/commands/sils.py の `sysid-gate`
    （SILS モデル一致ゲート）専用。実機ログ向けツールは代わりに fit_from_df()
    を使う。

    Args:
        input_mode: 'auto' (default) -- prefer the 400Hz motor-duty
            reconstruction (exact, no firmware-gain assumption) and fall back
            to the legacy PID-replay reconstruction only when the CSV has no
            genuine 400Hz duty (older firmware/capture). 'duty' forces the
            duty path (error if unavailable). 'kp' forces the legacy replay
            path (error path is unchanged from before this input existed).
        input_mode: 'auto'（既定）—— 400Hzモータduty復元（厳密、ファーム
            ゲインの仮定不要）を優先し、本物の400Hz duty が無いログ（旧ファー
            ム/旧キャプチャ）のときだけ従来のPID再生経路にフォールバックする。
            'duty' は duty 経路を強制（使えなければエラー）。'kp' は従来の
            再生経路を強制（本入力追加前と同じ挙動）。
    """
    _t, r, y, duty_u, duty_quality, duty_reason = load_csv(path, axis)
    return _fit_core(r, y, duty_u, duty_quality, duty_reason, axis, gains,
                      f_lo, f_hi, plot_path, input_mode)


def fit_from_df(df, axis, gains=None, f_lo=0.8, f_hi=30.0, plot_path=None,
                 input_mode='auto'):
    """Full pipeline on an aligned flight-log-bundle DataFrame: df -> recover
    u -> ETFE -> parametric (b, T, L). This is the entry point `sf sysid
    rate-fit` uses for a REAL vehicle flight-log bundle (as opposed to
    fit_from_csv(), used only by the SILS `sysid-gate`).
    整列済みフライトログ一式 DataFrame での全手順: df -> u 復元 -> ETFE ->
    (b,T,L)。実機フライトログ一式に対して `sf sysid rate-fit` が使う入口
    （SILS の `sysid-gate` 専用の fit_from_csv() とは別）。

    Args:
        df: the aligned DataFrame from tools/sysid/loader.load_aligned()
            (base="imu", method="hold") -- must contain 'rate_ref_<axis>' and
            'gyro_x'/'gyro_y'/'gyro_z', and OPTIONALLY 'duty_FR'/'duty_RR'/
            'duty_RL'/'duty_FL' (genuine 400Hz duty when the bundle's `motor`
            stream is present, else the ctrl_ref stream's 50Hz-held values --
            distinguished via df.attrs["bundle_streams"], NOT by column
            presence alone) and 'voltage' (battery voltage, from the bundle's
            `status` stream, held/forward-filled onto every row). See
            tools/sysid/loader.py and
            docs/plans/flight-log-format-plan.md section 2.2/3.2 for the full
            column contract -- this is the SAME DataFrame contract
            tools/sysid/plant_fit.py's fit_plant() uses.
        input_mode: see fit_from_csv().
        df: tools/sysid/loader.load_aligned()（base="imu", method="hold"）が
            返す整列済み DataFrame -- 'rate_ref_<axis>' と
            'gyro_x'/'gyro_y'/'gyro_z' が必須、'duty_FR'/'duty_RR'/'duty_RL'/
            'duty_FL'（一式の `motor` ストリームがあれば本物の400Hz duty、
            無ければ ctrl_ref ストリームの50Hz保持値 -- 列の有無ではなく
            df.attrs["bundle_streams"] で判別する）と 'voltage'（バッテリ
            電圧、一式の `status` ストリーム由来、全行へ前方保持）は任意。
            列の全契約は tools/sysid/loader.py と
            docs/plans/flight-log-format-plan.md 2.2/3.2節を参照 --
            tools/sysid/plant_fit.py の fit_plant() と同じ DataFrame 契約。
        input_mode: fit_from_csv() 参照。
    """
    col_r = f"rate_ref_{axis}"
    col_y = {"roll": "gyro_x", "pitch": "gyro_y", "yaw": "gyro_z"}[axis]
    if col_r not in df.columns or col_y not in df.columns:
        raise ValueError(
            f"bundle DataFrame has no '{col_r}'/'{col_y}' column -- need a "
            "flight-log bundle with the rate_ref and imu streams "
            "(docs/plans/flight-log-format-plan.md section 2.2)")
    r = df[col_r].to_numpy(dtype=float)
    y = df[col_y].to_numpy(dtype=float)
    if len(r) < 1024:
        raise ValueError(f"too few samples ({len(r)}) — capture with the "
                          "excitation running (sf log wifi + api sysid)")

    # Real per-row timestamps are available (unlike load_csv()'s CSV, which
    # only assumes exact 400Hz spacing via `len(t) * DT`) but this function's
    # own math (etfe()/fit_plant()/loop_margins()) all key off the MODULE-level
    # FS/DT constants (Welch windowing, PID replay's dt, etc.), so we don't
    # build/return a separate high-fidelity time vector here -- there is no
    # `t` in this function's return value at all (load_csv()'s `_t` was
    # already discarded by fit_from_csv() above). If per-row timing ever
    # matters here, derive it the same way tools/sysid/plant_fit.py's
    # _load_axis_data() does: (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6.
    # 本物の行ごとのタイムスタンプは使えるが（load_csv() の CSV が厳密に等間隔
    # 400Hz を仮定する `len(t) * DT` とは違う）、この関数自身の計算
    # （etfe()/fit_plant()/loop_margins()）は全てモジュールレベルの FS/DT
    # 定数に基づく（Welch窓・PID再生の dt 等）ため、ここで別途高精度の時刻配列は
    # 作らない・返さない（fit_from_csv() 側でも load_csv() の `_t` は既に
    # 捨てている）。行ごとの時刻が必要になったら、tools/sysid/plant_fit.py の
    # _load_axis_data() と同じ要領で導出する:
    # (df['timestamp_us'] - df['timestamp_us'].iloc[0]) / 1e6。

    duty_u = duty_quality = None
    duty_reason = "no motor/ctrl_ref duty_FR/RR/RL/FL columns in bundle DataFrame"
    duty_cols = ('duty_FR', 'duty_RR', 'duty_RL', 'duty_FL')
    if all(c in df.columns for c in duty_cols):
        duty_fr = df['duty_FR'].to_numpy(dtype=float)
        duty_rr = df['duty_RR'].to_numpy(dtype=float)
        duty_rl = df['duty_RL'].to_numpy(dtype=float)
        duty_fl = df['duty_FL'].to_numpy(dtype=float)

        # "motor" in bundle_streams is the EXACT (non-heuristic) signal for
        # whether duty_FR/RR/RL/FL came from the bundle's motor.csv stream
        # (genuine 400Hz kPktDuty400 entries) or, when motor.csv is absent,
        # from ctrl_ref.csv's 50Hz-held values (aligned()'s column-collision
        # rule keeps ctrl_ref's bare 'duty_FR' name only when motor.csv did
        # NOT also supply one -- see lib/sflog/align.py). Synthesize a
        # duty_rate_hz_col array from this boolean so the UNMODIFIED
        # _classify_duty_source() (shared with tools/sysid/plant_fit.py, same
        # trick used there) reaches the correct verdict via its existing
        # hz >= 200.0 threshold, without needing a real CSV rate column.
        # "motor" が bundle_streams に含まれるかは、duty_FR/RR/RL/FL が一式の
        # motor.csv ストリーム（本物の400Hz kPktDuty400エントリ）由来か、
        # motor.csv が無いときの ctrl_ref.csv の50Hz保持値由来かを判別する
        # 厳密な（ヒューリスティックでない）シグナル（aligned() の列衝突改名
        # 規則により、motor.csv が同じ列を供給しない場合のみ ctrl_ref の
        # 素の 'duty_FR' が残る -- lib/sflog/align.py 参照）。この真偽値から
        # duty_rate_hz_col 配列を合成することで、変更していない
        # _classify_duty_source()（tools/sysid/plant_fit.py と共有、同じ
        # 手法をあちらでも使用）が既存の hz >= 200.0 しきい値判定で正しい結果
        # に達する -- 本物の CSV レート列は不要。
        motor_present = "motor" in df.attrs.get("bundle_streams", set())
        duty_rate_hz_arr = np.full(len(duty_fr), 400.0 if motor_present else 50.0)
        duty_quality, duty_reason = _classify_duty_source(
            duty_fr, duty_rr, duty_rl, duty_fl, duty_rate_hz_arr)

        # vbat: bundle's `voltage` column (status stream, held/forward-filled)
        # -- same nominal-fallback/implausible-value policy as load_csv().
        # vbat: 一式の `voltage` 列（status ストリーム、前方保持）——
        # load_csv() と同じノミナルフォールバック／非現実値の扱い。
        vbat_note = ''
        if 'voltage' in df.columns:
            vbat_arr = df['voltage'].to_numpy(dtype=float)
            bad = vbat_arr < _V_BATT_MIN
            if np.any(bad):
                vbat_arr = np.where(bad, _V_BATT_NOMINAL, vbat_arr)
                vbat_note = (f" ({int(np.sum(bad))}/{len(vbat_arr)} rows had no/"
                             f"implausible voltage -- used the nominal {_V_BATT_NOMINAL}V there)")
        else:
            vbat_arr = np.full(len(duty_fr), _V_BATT_NOMINAL)
            vbat_note = (f" (no voltage column in bundle -- used the nominal "
                         f"{_V_BATT_NOMINAL}V throughout; capture with the "
                         "bundle's status stream present for the real "
                         "battery-sag-corrected fit)")
        duty_reason += vbat_note
        duty_u = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat_arr, axis)

    return _fit_core(r, y, duty_u, duty_quality, duty_reason, axis, gains,
                      f_lo, f_hi, plot_path, input_mode)


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

    # --- duty-path regression (the NEW, preferred path): forward-map the
    # SAME true u[] (roll torque) through firmware/vehicle's real forward
    # mixer (B^-1 allocation, pitch=yaw=0, then thrustToDuty()) to synthesize
    # 4 motor duties, then invert with _duty_differential_vehicle() -- the
    # function fit_from_csv(input_mode='duty'/'auto') uses -- and confirm it
    # recovers u AND fits the same known plant. Proves the duty path reaches
    # the same result as the PID-replay path above WITHOUT knowing the PID
    # gains. vbat_true is deliberately off V_BATT_NOMINAL so a bug that
    # silently used the nominal fallback instead of the real voltage would
    # show up here as a scale error.
    # duty経路の回帰（新しい・優先すべき経路）: 同じ真の u[]（ロールトルク）を
    # firmware/vehicle の実順方向ミキサー（B^-1配分、pitch=yaw=0、その後
    # thrustToDuty()）で4モータduty に変換し、_duty_differential_vehicle()
    # （fit_from_csv(input_mode='duty'/'auto') が使う関数）で逆算してuと
    # 既知プラントの両方を復元できることを確認する。duty経路がPIDゲインを
    # 知らなくても上のPID再生経路と同じ結果に到達することの証明。vbat_true は
    # 意図的に V_BATT_NOMINAL からずらしてあり、実電圧を使わず黙ってノミナル
    # にフォールバックするバグがあればスケール誤差として現れる。
    vbat_true = 3.85    # [V] != V_BATT_NOMINAL (3.7)
    thrust_hover = 0.4  # [N] arbitrary in-flight value; keeps all 4 motor
                         # thrusts positive across the excitation amplitude
    t_fr = 0.25 * (thrust_hover - u / _ARM_D)
    t_rr = 0.25 * (thrust_hover - u / _ARM_D)
    t_rl = 0.25 * (thrust_hover + u / _ARM_D)
    t_fl = 0.25 * (thrust_hover + u / _ARM_D)

    def _thrust_to_duty(t_arr):
        omega_m = np.sqrt(np.maximum(t_arr, 0.0) / _MOTOR_CT)
        volts = _MOTOR_AM * omega_m ** 2 + _MOTOR_BM * omega_m + _MOTOR_CM
        return volts / vbat_true

    duty_fr, duty_rr = _thrust_to_duty(t_fr), _thrust_to_duty(t_rr)
    duty_rl, duty_fl = _thrust_to_duty(t_rl), _thrust_to_duty(t_fl)
    vbat_arr = np.full(n, vbat_true)

    u_duty = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat_arr, "roll")
    err_u_duty = float(np.max(np.abs(u_duty - u)))
    omega_d, G_hat_d, coh_d = etfe(u_duty, y)
    fit_d = fit_plant(omega_d, G_hat_d, coh_d, "roll")
    ok_duty = (err_u_duty < 1e-6 and
               abs(fit_d["b"] / b_true - 1) < 0.15 and
               abs(fit_d["T"] / T_true - 1) < 0.30 and
               abs(fit_d["L"] - L_true) < 0.004)
    ok = ok and ok_duty

    # --- fit_from_df() regression: package the SAME synthetic flight as a
    # minimal bundle DataFrame (reusing r/y/duty_fr../vbat_arr already built
    # above) and confirm the DataFrame front-end (used by `sf sysid rate-fit`
    # on a real vehicle flight-log bundle, as opposed to fit_from_csv()'s SILS
    # rate_stream.csv) reaches the same fit via input_mode='duty'.
    # fit_from_df() の回帰確認: 同じ合成フライト（上で作った r/y/duty_fr../
    # vbat_arr を再利用）を最小限の一式 DataFrame に詰め、DataFrame 側の入口
    # （fit_from_csv() の SILS rate_stream.csv とは別に、実機フライトログ一式に
    # `sf sysid rate-fit` が使う）が input_mode='duty' で同じフィットに
    # 到達することを確認する。
    import pandas as _pd
    df_selftest = _pd.DataFrame({
        "timestamp_us": (t * 1e6).astype("int64"),
        "rate_ref_roll": r,
        "gyro_x": y,
        "duty_FR": duty_fr, "duty_RR": duty_rr, "duty_RL": duty_rl, "duty_FL": duty_fl,
        "voltage": vbat_arr,
    })
    df_selftest.attrs["bundle_streams"] = {"imu", "rate_ref", "motor", "status"}
    fit_df = fit_from_df(df_selftest, "roll", input_mode='duty')
    ok_df = (abs(fit_df["b"] / b_true - 1) < 0.15 and
             abs(fit_df["T"] / T_true - 1) < 0.30 and
             abs(fit_df["L"] - L_true) < 0.004 and
             fit_df["input_mode"] == 'duty')
    ok = ok and ok_df

    if verbose:
        print(f"replay max|u_rec-u| = {err_u:.2e} (exactness of the PID port)")
        print(f"fit : b={fit['b']:.0f} (true {b_true:.0f})  "
              f"T={fit['T'] * 1000:.1f}ms (true {T_true * 1000:.1f})  "
              f"L={fit['L'] * 1000:.2f}ms (true {L_true * 1000:.2f})  "
              f"coh={fit['coherence_mean']:.2f}")
        print(f"tune: kp={tune['kp']:.3e} ti={tune['ti']:.3f} td={tune['td']:.4f}")
        print(f"      achieved wc={ach['wc']:.1f} rad/s pm={ach['pm_deg']:.1f} deg "
              f"gm={ach['gm_db']:.1f} dB")
        print(f"duty path: max|u_duty-u| = {err_u_duty:.2e}  "
              f"fit b={fit_d['b']:.0f} T={fit_d['T'] * 1000:.1f}ms "
              f"L={fit_d['L'] * 1000:.2f}ms coh={fit_d['coherence_mean']:.2f}  "
              f"{'PASS' if ok_duty else 'FAIL'}")
        print(f"fit_from_df (bundle DataFrame path): "
              f"fit b={fit_df['b']:.0f} T={fit_df['T'] * 1000:.1f}ms "
              f"L={fit_df['L'] * 1000:.2f}ms input_mode={fit_df['input_mode']}  "
              f"{'PASS' if ok_df else 'FAIL'}")
        print("SELFTEST:", "PASS" if ok else "FAIL")
    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)
