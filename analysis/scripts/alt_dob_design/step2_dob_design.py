#!/usr/bin/env python3
"""step2_dob_design.py — Accel-based disturbance observer (DOB) design for the
altitude velocity loop, built on the Step 1 actuation-path identification.
加速度ベース外乱オブザーバ（DOB）の設計シミュレーション（高度制御向け、Step 1同定を土台）

Structure implemented (mirrors the firmware target, see task brief):
  residual   = M*f_up_meas - T_applied_model(thrust_cmd)*cos(tilt)   [N]
  d_hat      = Washout(Q_LPF(residual)), clamped to +-0.10 N
  thrust_cmd = HOVER + PI_corr - d_hat
T_applied_model is the NOMINAL actuation model H_act(s)=g*exp(-Ls)/(Ts+1)
applied to the FINAL commanded thrust (DOB-compensation included) -- this is
the standard DOB architecture and creates an internal loop (see the linear
analysis section for its stability condition).
実装した構造（上記の通り）。T_applied_modelはノミナルモデルを「DOB補償込みの
最終推力コマンド」に適用したもの — これが正しいDOB構造で内部ループが生じる
（線形解析節でその安定条件を導出する）。

Plant model change from the old single-lag replay (alt_ti_venue_sim.py,
TAU=0.11s lumped): here the plant is PURE DELAY (L=62.4ms, rounded to an
integer number of grid samples) THEN a first-order lag (T=20ms) -- the
Step-1-identified actuation path. The velocity-loop feedback additionally
gets a first-order ESKF lag (~28ms = 110ms - 62.4ms - 20ms) that a
raw-accelerometer DOB bypasses entirely. See verification gates 1/2 in
main() for whether this reproduces measured altitude fluctuation.
旧再生（TAU=0.11s単一遅れ）からのプラント変更点: 純遅れ(62.4ms、グリッド丸め)
+ 1次遅れ(20ms)= Step1同定のアクチュエーション経路。速度FBにはさらにESKF遅れ
(~28ms)を1次遅れとして追加する（生加速度を使うDOBはこれをバイパスする）。

Usage / 使い方
--------------
    python3 analysis/scripts/alt_dob_design/step2_dob_design.py

No arguments. Reuses step1_actuation_id.py's log-loading / gap-handling /
f_up-construction / d_ext_acc-reconstruction code directly (imported as `s1`)
and alt_ti_venue_sim.py's PID class + smoothing helpers (imported as `avs`).
Outputs:
    analysis/scripts/alt_dob_design/step2_results.json
    analysis/scripts/alt_dob_design/figs/*.png (5 figures, see main() bottom)
Prints a human-readable summary to stdout. Does not commit anything.
"""

import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import butter, filtfilt, welch, lfilter_zi

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "analysis" / "scripts" / "yaw_nt_kanazawa"))
sys.path.insert(0, str(REPO / "analysis" / "scripts"))
import step1_actuation_id as s1          # noqa: E402  (log load / gap mask / f_up / d_ext_acc reuse)
import alt_ti_venue_sim as avs           # noqa: E402  (PID class, smooth/lpf helpers)

FIGS_DIR = HERE / "figs"
FIGS_DIR.mkdir(parents=True, exist_ok=True)
RESULTS_PATH = HERE / "step2_results.json"

# =============================================================================
# Physical / plant constants
# 物理・プラント定数
# =============================================================================
M = s1.M                          # 0.037 kg
G = s1.G                          # 9.80665 m/s^2
HOVER_CORR = s1.HOVER_CORR        # 1.12
HOVER = s1.HOVER                  # ~0.4064 N

# Step-1-identified actuation path H_act(s) = g*exp(-L s)/(T s + 1).
# g stays at 1.0 -- the DC-gain cross-check (step2b_dc_gain_check in
# step1_results.json) gave 0.952/0.945 for A/B, i.e. thrust command is a good
# Newton estimate "at face value" (task brief: "gは≈0.95で額面通り") -- treated
# as unit gain here, matching step1's own step4 model (g=1.0).
# Step1同定のアクチュエーション経路。gは1.0(額面通り、DCゲイン検算0.95で確認済み)。
L_ACT = 0.0624   # s, mean of takeoff transient delays (A=63.6ms, B=61.2ms)
T_ACT = 0.02     # s, TAU_M_REF (motor/ESC/prop time constant, independent ref)
G_ACT = 1.0
ACT_MODEL = dict(L=L_ACT, T=T_ACT, g=G_ACT)

TAU_COMBINED_REF = 0.11   # s, old lumped velocity-loop-inferred total lag
TAU_ESKF = TAU_COMBINED_REF - L_ACT - T_ACT   # ~0.0276s, ESKF vz estimation lag
assert TAU_ESKF > 0, "actuation lag exceeds the old combined-lag reference"

FS = 200.0        # replay/DOB grid [Hz] (task-recommended)
DT = 1.0 / FS
GAP_THR = s1.GAP_THR_ID    # 0.05s, same telemetry-gap threshold as step1
MIN_SEG = 2.0               # s, min contiguous valid segment (matches alt_ti_venue_sim.MIN_SEG)

N_DELAY_NOM = int(round(L_ACT * FS))   # pure-delay length in samples at FS=200Hz
DELAY_ROUND_ERR_MS = (N_DELAY_NOM / FS - L_ACT) * 1000.0
print(f"[grid] FS={FS:.0f}Hz dt={DT*1000:.2f}ms  L_ACT={L_ACT*1000:.1f}ms -> "
      f"{N_DELAY_NOM} samples = {N_DELAY_NOM/FS*1000:.1f}ms "
      f"(rounding error {DELAY_ROUND_ERR_MS:+.2f}ms, "
      f"{abs(DELAY_ROUND_ERR_MS)/(L_ACT*1000)*100:.1f}% of L; "
      f"phase error at 5Hz = {abs(2*np.pi*5*DELAY_ROUND_ERR_MS/1000)*180/np.pi:.1f} deg -- negligible below 5Hz)")

# ---- cascade gains (current firmware defaults, task brief) ----
AKP, ATI, ALIM = 0.45, 7.0, 0.5      # alt_pos PI [1/s, s, m/s]
VKP, VTI_HOVER, VLIM = 0.1, 1.5, 0.15  # alt_vel PI [N/(m/s), s, N] -- ti_hover=1.5 is the current adopted default

# ---- DOB design parameters ----
WASHOUT_FC = 0.03    # Hz, 1st-order highpass (DC belongs to the velocity-loop integrator + hover-thrust learning)
DOB_CLAMP = 0.10      # N
FC_LIST = [0.5, 1.0, 1.5, 2.0, 3.0]   # Hz, Q-filter cutoff sweep

LOGS_DIR = REPO / "logs"
LOG_A = s1.LOG_A
LOG_B = s1.LOG_B
LOG_C = s1.LOG_C
LOG_D = s1.LOG_D
WIN_C = s1.WIN_C
WIN_D = s1.WIN_D


# =============================================================================
# Generic causal IIR filter (Direct-Form-II-Transposed), stateful stepping.
# Used for the DOB's Q-filter and washout -- identical recursion to
# scipy.signal.lfilter (so scipy.signal.lfilter_zi() primes it correctly for
# a steady-state start, avoiding a startup transient on short/gappy segments).
# 汎用の状態保持IIRフィルタ（直接形II転置型）。DOBのQフィルタ/ウォッシュアウトに使用。
# scipy.signal.lfilterと同じ漸化式なので lfilter_zi() で定常初期化できる
# （短い/ギャップ区間で起動トランジェントを避けるため）。
# =============================================================================
class CausalIIR:
    def __init__(self, b, a):
        b = np.asarray(b, dtype=float)
        a = np.asarray(a, dtype=float)
        b = b / a[0]
        a = a / a[0]
        n = max(len(a), len(b))
        self.b = np.pad(b, (0, n - len(b)))
        self.a = np.pad(a, (0, n - len(a)))
        self.k = n - 1
        self.z = np.zeros(self.k)

    def set_steady_state(self, x0):
        if self.k == 0:
            return
        zi = lfilter_zi(self.b, self.a)
        self.z = zi * x0

    def step(self, x):
        if self.k == 0:
            return self.b[0] * x
        y = self.b[0] * x + self.z[0]
        for i in range(self.k - 1):
            self.z[i] = self.b[i + 1] * x - self.a[i + 1] * y + self.z[i + 1]
        self.z[self.k - 1] = self.b[self.k] * x - self.a[self.k] * y
        return y


def make_q_filter(fc, order):
    b, a = butter(order, fc, btype="low", fs=FS)
    return CausalIIR(b, a)


def make_washout_filter(fc=WASHOUT_FC):
    b, a = butter(1, fc, btype="high", fs=FS)
    return CausalIIR(b, a)


# =============================================================================
# Log loading: unified FS=200Hz grid with imu (f_up, cos_tilt, hf accel noise),
# ctrl_ref (thrust), posvel (alt, vz) channels, gap-masked across all three
# native timelines. Reuses step1's gap-mask/antialias/zoh utilities verbatim.
# ログ読み込み: imu/ctrl_ref/posvelの3系統をFS=200Hzグリッドへギャップ対応で統合。
# step1のギャップマスク/アンチエイリアス/ZOHユーティリティをそのまま再利用。
# =============================================================================
def load_full_log(path, win, fs=FS, gap_thr=GAP_THR, min_seg=MIN_SEG):
    d = s1.load_jsonl(str(path))
    imu = d["imu"]
    tim = imu["ts"]
    accel_corr = s1.bias_corrected_accel(imu)
    f_up_native, cos_native = s1.body_accel_to_f_up(accel_corr, imu["quat"])
    fs_imu_native = 1.0 / np.median(np.diff(tim))

    # anti-alias LPF before decimating f_up onto the fs-Hz grid (step1 finding:
    # without this, d_ext_acc inflates ~2x from aliased 20-200Hz vibration)
    f_up_lp = s1.antialias_lowpass(f_up_native, fs_imu_native, cutoff_hz=0.8 * (fs / 2.0))

    # high-frequency (>2Hz) accel content, used as the DOB's realistic
    # measurement-noise injection n_acc. AA-filtered again before decimating
    # (task brief: "400Hz→グリッドへAA間引き") to avoid a SECOND aliasing fold.
    f_up_2hz = s1.antialias_lowpass(f_up_native, fs_imu_native, cutoff_hz=2.0)
    n_acc_native = f_up_native - f_up_2hz
    n_acc_native_aa = s1.antialias_lowpass(n_acc_native, fs_imu_native, cutoff_hz=0.8 * (fs / 2.0))

    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]
    tpv = d["posvel"]["ts"]
    pos = d["posvel"]["pos"]
    vel = d["posvel"]["vel"]
    alt_native = -pos[:, 2]
    vz_native = -vel[:, 2]

    tg, invalid = s1.build_gap_mask(win, [tim, tcr, tpv], gap_thr, fs)
    segs = s1.find_valid_segments(tg, invalid, min_seg, fs)

    fupg = np.interp(tg, tim, f_up_lp)
    cosg = np.interp(tg, tim, cos_native)
    thrg = s1.zoh_interp(tg, tcr, thr)
    altg = np.interp(tg, tpv, alt_native)
    vzg = np.interp(tg, tpv, vz_native)
    n_accg = np.interp(tg, tim, n_acc_native_aa)

    valid_mask = ~invalid
    sp = float(np.median(altg[valid_mask])) if np.any(valid_mask) else float(np.median(altg))

    return dict(tg=tg, invalid=invalid, segs=segs, fupg=fupg, cosg=cosg, thrg=thrg,
                altg=altg, vzg=vzg, n_accg=n_accg, sp=sp, win=win)


# =============================================================================
# d_ext reconstruction: accel-based (Step 1 method, reused verbatim) and
# kinematic (existing alt_ti_venue_sim method, reused verbatim), both
# band-limited to <=2Hz (zero-phase, PER SEGMENT -- never across a gap) before
# use as the injected disturbance.
# d_ext再構成: 加速度ベース(Step1手法をそのまま再利用)と運動学ベース(既存手法を
# そのまま再利用)。両方とも2Hz以下に零位相帯域制限(区間ごと、ギャップをまたがない)。
# =============================================================================
def dext_acc_for_segment(L, s, e, model=ACT_MODEL):
    fup_seg = L["fupg"][s:e + 1]
    thr_seg = L["thrg"][s:e + 1]
    cos_seg = L["cosg"][s:e + 1]
    u_eff_seg = thr_seg * cos_seg
    u_lagged = s1.causal_delay_lag(u_eff_seg, DT, model["L"], model["T"], model["g"])
    y_seg = M * fup_seg
    return y_seg - u_lagged


def dext_kin_for_segment(L, s, e):
    """Existing (alt_ti_venue_sim) kinematic method, reused verbatim except
    fs-scaled smoothing windows (avs hardcodes FS=100; here FS=200, so the
    window sample-counts are doubled to preserve the same TIME constant)."""
    vzg_seg = L["vzg"][s:e + 1]
    thrg_seg = L["thrg"][s:e + 1]
    cosg_seg = L["cosg"][s:e + 1]
    n = len(vzg_seg)
    scale = FS / 100.0
    w = min(int(round(30 * scale)), max(int(round(5 * scale)), n // 8))
    w = max(w, 3)
    az = avs.smooth(np.gradient(avs.smooth(vzg_seg, w), DT), w)
    thr_lag = avs.lpf(thrg_seg, TAU_COMBINED_REF, FS)   # avs.TAU=0.11, the ORIGINAL kinematic model (unchanged)
    return M * az - (thr_lag * cosg_seg - M * G)


def bandlimit_2hz(x, fs=FS, cutoff=2.0, order=4):
    """Zero-phase Butterworth LPF, applied within ONE gap-free segment only."""
    if len(x) < 15:
        return x.copy()
    b, a = butter(order, cutoff / (fs / 2.0), btype="low")
    return filtfilt(b, a, x)


def compute_dext_all(L, source):
    """Returns list of (s, e, d_ext_bandlimited, n_acc_seg) per valid segment."""
    out = []
    for s, e in L["segs"]:
        d_raw = dext_acc_for_segment(L, s, e) if source == "acc" else dext_kin_for_segment(L, s, e)
        d_bl = bandlimit_2hz(d_raw)
        out.append((s, e, d_bl, L["n_accg"][s:e + 1]))
    return out


def vz_noise_for_segment(vzg_seg, fs=FS):
    """High-frequency fluctuation of the LOGGED vz around its own smoothed
    trend -- injected onto the SIMULATED true vz as a realistic velocity-
    estimate noise floor (same technique as alt_ti_venue_sim.replay_segment)."""
    nw = max(3, int(round(fs / 1.5)))
    return vzg_seg - avs.smooth(vzg_seg, nw)


# =============================================================================
# DOB filter-state initialization at a segment start (REVISED per coordinator
# review, 2026-07-18): falsification test target was that "sample0" (a SINGLE
# raw d_ext sample as the assumed steady-state residual) can seed the washout
# with the wrong DC level whenever that one sample is itself a bandlimit_2hz
# filtfilt edge artifact -- on a heavily gap-fragmented log (C/D, segments as
# short as 2.8s) the washout's own ~5.3s time constant (1/(2*pi*0.03Hz)) then
# needs most/all of a short segment to "walk down" from the wrong seed,
# manufacturing a REPLAY-ONLY transient with no firmware counterpart (the real
# control loop never resets). "avg025s" instead averages d_ext+M*n_acc over
# the first 0.25s (50 samples) -- exact because residual==d_ext+M*n_acc
# sample-for-sample in the matched-model case (verified numerically, see
# report), so this needs no forward simulation, just averaging the known
# input series before the loop starts.
# DOBフィルタ状態の区間頭初期化（2026-07-18レビュー対応の反証テスト対象）:
# 旧"sample0"は生のd_ext単一サンプルを定常残差とみなすため、そのサンプルが
# bandlimit_2hzのfiltfiltエッジアーティファクトだった場合に誤ったDCでウォッシュ
# アウトを初期化してしまう。区間が短いC/D(最短2.8s)ではウォッシュアウトの時定数
# (~5.3s)がその誤りを消化しきるのに区間のほとんど/全部を使ってしまい、実機には
# 存在しないリプレイ専用の過渡を作る。新"avg025s"は最初0.25秒(50サンプル)の
# d_ext+M*n_accの平均を使う（マッチモデル下でresidual=d_ext+M*n_accがサンプル
# 単位で厳密に成立するため、ループを回さず既知入力列の平均だけで求まる）。
N_INIT_S = 0.25
N_INIT_SAMPLES = int(round(N_INIT_S * FS))   # 50 samples @ FS=200Hz


def compute_dob_x0(d_ext_seg, n_acc_seg, mode):
    if mode == "sample0":
        return float(d_ext_seg[0])
    if mode == "avg025s":
        n = min(N_INIT_SAMPLES, len(d_ext_seg))
        return float(np.mean(d_ext_seg[:n] + M * n_acc_seg[:n]))
    raise ValueError(f"unknown dob_init_mode {mode!r}")


# =============================================================================
# Cascade + DOB + plant replay (one gap-free segment)
# カスケード+DOB+プラント再生（ギャップなし1区間）
# =============================================================================
def default_cfg(**overrides):
    cfg = dict(
        akp=AKP, ati=ATI, alim=ALIM,
        vkp=VKP, vti=VTI_HOVER, vlim=VLIM,
        dob_enabled=False, dob_fc=1.5, dob_order=2,
        washout_enabled=True, washout_fc=WASHOUT_FC, dob_clamp=DOB_CLAMP,
        residual_guard=None,   # None/0 = disabled; else clip |residual| to this [N] BEFORE the Q filter
                                # (2026-07-18 review: separate from dob_clamp, which limits the FINAL
                                # d_hat AFTER Q+washout. The guard limits the raw measurement feeding
                                # the filter, so a single large burst sample can't leave a large,
                                # slowly-decaying imprint inside the Q/washout filter states.)
        dob_init_mode="avg025s",   # "sample0" = original/legacy behavior, kept for the artifact A/B test
        plant_L=ACT_MODEL["L"], plant_T=ACT_MODEL["T"], plant_g=ACT_MODEL["g"],
        model_L=ACT_MODEL["L"], model_T=ACT_MODEL["T"], model_g=ACT_MODEL["g"],
    )
    cfg.update(overrides)
    return cfg


def replay_segment_dob(L, s, e, d_ext_seg, n_acc_seg, cfg):
    """Closed-loop replay of ONE gap-free segment: alt_pos PI -> alt_vel PI ->
    (optional DOB correction) -> plant (pure delay + 1st-order lag) -> ESKF-
    lagged + noisy vz feedback. All state (PID integrators, plant delay
    buffer/lag state, DOB filter state) is (re)initialized at the segment
    start -- never carried across a telemetry gap (same discipline as
    alt_ti_venue_sim.replay_segment)."""
    tg_seg = L["tg"][s:e + 1]
    n = len(tg_seg)
    cosg_seg = L["cosg"][s:e + 1]
    sp = L["sp"]

    a_pid = avs.PID(cfg["akp"], cfg["ati"], cfg["alim"])
    v_pid = avs.PID(cfg["vkp"], cfg["vti"], cfg["vlim"])

    alt = np.zeros(n)
    vz_true = np.zeros(n)
    vz_eskf = np.zeros(n)
    thr_applied = np.zeros(n)
    thrust_cmd = np.full(n, HOVER)
    corr_hist = np.zeros(n)
    dhat_hist = np.zeros(n)
    residual_hist = np.zeros(n)
    I_hist = np.zeros(n)
    dob_clamp_hits = np.zeros(n, dtype=bool)
    pi_clamp_hits = np.zeros(n, dtype=bool)
    residual_guard_hits = np.zeros(n, dtype=bool)

    alt[0] = L["altg"][s]
    vz0 = L["vzg"][s]
    vz_true[0] = vz0
    vz_eskf[0] = vz0
    thr_applied[0] = HOVER

    n_delay_p = max(1, int(round(cfg["plant_L"] * FS)))
    n_delay_m = max(1, int(round(cfg["model_L"] * FS)))
    ac_plant = np.exp(-DT / max(cfg["plant_T"], 1e-6))
    ac_model = np.exp(-DT / max(cfg["model_T"], 1e-6))
    ac_eskf = np.exp(-DT / TAU_ESKF)

    vz_noise_seg = vz_noise_for_segment(L["vzg"][s:e + 1])

    dob_on = cfg["dob_enabled"]
    qfilt = make_q_filter(cfg["dob_fc"], cfg["dob_order"]) if dob_on else None
    wfilt = make_washout_filter(cfg["washout_fc"]) if (dob_on and cfg["washout_enabled"]) else None
    if dob_on:
        x0 = compute_dob_x0(d_ext_seg, n_acc_seg, cfg.get("dob_init_mode", "avg025s"))
        rg0 = cfg.get("residual_guard")
        if rg0:
            x0 = float(np.clip(x0, -rg0, rg0))   # keep the seed consistent with the guarded per-sample input
        qfilt.set_steady_state(x0)
        if wfilt is not None:
            wfilt.set_steady_state(x0)
    thr_model_state = HOVER

    for i in range(1, n):
        # ---- plant physics (driven by PAST commands only, via the delay buffer) ----
        idx_dp = i - n_delay_p
        u_delayed_plant = thrust_cmd[idx_dp] if idx_dp >= 0 else HOVER
        thr_applied[i] = ac_plant * thr_applied[i - 1] + (1 - ac_plant) * (cfg["plant_g"] * u_delayed_plant)
        az_true = (thr_applied[i] * cosg_seg[i] - M * G + d_ext_seg[i]) / M
        vz_true[i] = vz_true[i - 1] + az_true * DT
        alt[i] = alt[i - 1] + vz_true[i] * DT
        f_up_true = az_true + G
        f_up_meas = f_up_true + n_acc_seg[i]

        vz_eskf[i] = ac_eskf * vz_eskf[i - 1] + (1 - ac_eskf) * vz_true[i]
        vz_meas = vz_eskf[i] + vz_noise_seg[i]

        # ---- cascade control ----
        vsp = a_pid.step(sp, alt[i - 1], DT)
        corr = v_pid.step(vsp, vz_meas, DT)
        corr_hist[i] = corr
        I_hist[i] = v_pid.I
        pi_clamp_hits[i] = abs(corr) >= 0.999 * cfg["vlim"]

        # ---- DOB ----
        if dob_on:
            idx_dm = i - n_delay_m
            u_delayed_model = thrust_cmd[idx_dm] if idx_dm >= 0 else HOVER
            thr_model_state = ac_model * thr_model_state + (1 - ac_model) * (cfg["model_g"] * u_delayed_model)
            residual = M * f_up_meas - thr_model_state * cosg_seg[i]
            residual_hist[i] = residual
            rg = cfg.get("residual_guard")
            if rg:
                residual_guard_hits[i] = abs(residual) > rg
                residual_q_in = float(np.clip(residual, -rg, rg))
            else:
                residual_q_in = residual
            q_val = qfilt.step(residual_q_in)
            dhat_raw = wfilt.step(q_val) if wfilt is not None else q_val
            dhat = float(np.clip(dhat_raw, -cfg["dob_clamp"], cfg["dob_clamp"]))
            dob_clamp_hits[i] = abs(dhat_raw) > cfg["dob_clamp"]
        else:
            dhat = 0.0
        dhat_hist[i] = dhat

        thrust_cmd[i] = float(np.clip(HOVER + corr - dhat, 0.0, 0.6))

    return dict(t=tg_seg, alt=alt, vz_true=vz_true, thrust_cmd=thrust_cmd, corr=corr_hist,
                dhat=dhat_hist, residual=residual_hist, I_hist=I_hist,
                dob_clamp_hits=dob_clamp_hits, pi_clamp_hits=pi_clamp_hits,
                residual_guard_hits=residual_guard_hits, thr_applied=thr_applied)


def replay_all(L, dext_per_seg, cfg):
    return [replay_segment_dob(L, s, e, d_ext_seg, n_acc_seg, cfg)
            for (s, e, d_ext_seg, n_acc_seg) in dext_per_seg]


# =============================================================================
# Falsification test 1: direct artifact detection (2026-07-18 coordinator review)
# 反証テスト1: 区間頭アーティファクトの直接検出
# =============================================================================
def artifact_split_metrics(traces, sp, split_s=5.0):
    """Split EACH segment's trace at split_s seconds from ITS OWN start, and
    aggregate |d_hat| and alt-vs-sp error separately for the early/late parts
    (duration-weighted via plain concatenation, since all segments share the
    same FS). If the coordinator's segment-reset-transient hypothesis is
    correct, |d_hat| and the alt error should be systematically larger in the
    early part than the late part, concentrated within the first ~5s -- a
    plain per-segment std (around each segment's own mean) would UNDERSTATE a
    persistent early bias, so this uses error against the GLOBAL setpoint,
    not each segment's own mean.
    各区間を「区間頭からsplit_s秒」で前後に分割し、|d_hat|とalt-sp誤差を集計。
    区間リセット過渡仮説が正しければ前半に偏って大きくなるはず。区間頭の持続的な
    バイアスを過小評価しないよう、区間自身の平均でなく共通のspに対する誤差を使う。"""
    early_dhat, late_dhat, early_err, late_err = [], [], [], []
    seg_summaries = []
    for t in traces:
        tt = t["t"] - t["t"][0]
        m_e, m_l = tt < split_s, tt >= split_s
        if m_e.sum() > 1:
            early_dhat.append(np.abs(t["dhat"][m_e]))
            early_err.append(t["alt"][m_e] - sp)
        if m_l.sum() > 1:
            late_dhat.append(np.abs(t["dhat"][m_l]))
            late_err.append(t["alt"][m_l] - sp)
        seg_summaries.append(dict(
            seg_len_s=float(tt[-1]),
            early_dhat_mean_mN=float(np.mean(np.abs(t["dhat"][m_e]))) * 1000 if m_e.sum() else float("nan"),
            late_dhat_mean_mN=float(np.mean(np.abs(t["dhat"][m_l]))) * 1000 if m_l.sum() else float("nan"),
            early_alt_rms_mm=float(np.sqrt(np.mean((t["alt"][m_e] - sp) ** 2))) * 1000 if m_e.sum() else float("nan"),
            late_alt_rms_mm=float(np.sqrt(np.mean((t["alt"][m_l] - sp) ** 2))) * 1000 if m_l.sum() else float("nan"),
        ))

    def cat_rms(parts):
        c = np.concatenate(parts) if parts else np.array([0.0])
        return float(np.sqrt(np.mean(c ** 2))) if len(c) else float("nan")

    def cat_mean(parts):
        c = np.concatenate(parts) if parts else np.array([0.0])
        return float(np.mean(c)) if len(c) else float("nan")

    return dict(
        split_s=split_s, n_segments=len(traces),
        early_dhat_mean_mN=cat_mean(early_dhat) * 1000,
        late_dhat_mean_mN=cat_mean(late_dhat) * 1000,
        early_alt_rms_mm=cat_rms(early_err) * 1000,
        late_alt_rms_mm=cat_rms(late_err) * 1000,
        per_segment=seg_summaries,
    )


# =============================================================================
# Falsification test 3: log-level DC removal from d_ext_acc before injection
# 反証テスト3: d_ext_accのログ全体DCを除去してから注入
# =============================================================================
def remove_log_dc(dext_per_seg):
    """Subtract ONE constant (duration-weighted mean over ALL valid segments
    of this log) from every segment's d_ext -- tests the interpretation that
    the persistent ~-40 to -80mN offset in d_ext_acc (thrust-calibration /
    HOVER_CORR model error, not a real external force) is something real
    hardware's PI integrator already silently absorbs, so it should not be
    presented to the DOB as if it were a genuine disturbance to fight.
    ログ全体で1つの定数(全有効区間の時間重み付き平均)をd_extから引く。d_ext_accの
    持続的な-40~-80mNオフセット(推力校正/HOVER_CORRのモデル誤差、真の外力ではない)
    は実機ではPI積分器が既に吸収している、という解釈の感度を見る。"""
    total_n = sum(e - s + 1 for s, e, _, _ in dext_per_seg)
    dc = sum(np.sum(d) for _, _, d, _ in dext_per_seg) / total_n
    return [(s, e, d - dc, n) for (s, e, d, n) in dext_per_seg], float(dc)


# =============================================================================
# Metrics
# =============================================================================
def band_rms_segments(per_seg_series, flo, fhi, fs=FS, min_n=8):
    """NOTE: nperseg targets a 50s window (0.02Hz resolution) so the
    0.02-0.1Hz LOW band is actually resolvable -- a fixed 10s window (as used
    elsewhere for coarser band splits) only reaches 0.1Hz resolution, i.e. at
    most ONE bin sits inside [0.02,0.1], and this function's own `>=2 bins`
    validity check then silently returns NaN for every case (found via the
    visual figure review: band_rms.png showed the low-band bars entirely
    missing). Segments shorter than 50s just use their own full length
    (whatever resolution that gives; very short C/D segments may still
    legitimately return NaN -- genuinely not resolvable)."""
    f_common, Pxx_sum, dur_sum = None, None, 0.0
    for tg_seg, x_seg in per_seg_series:
        n = len(x_seg)
        if n < min_n:
            continue
        nper = min(n, max(min_n, int(fs * 50)))
        f, Pxx = welch(x_seg, fs=fs, nperseg=nper)
        dur = n / fs
        if f_common is None:
            f_common = f
        Pxx_i = np.interp(f_common, f, Pxx)
        Pxx_sum = Pxx_i * dur if Pxx_sum is None else Pxx_sum + Pxx_i * dur
        dur_sum += dur
    if Pxx_sum is None or dur_sum == 0:
        return float("nan")
    Pxx_avg = Pxx_sum / dur_sum
    band = (f_common >= flo) & (f_common <= fhi)
    return float(np.sqrt(np.trapezoid(Pxx_avg[band], f_common[band]))) if band.sum() >= 2 else float("nan")


def welch_avg_psd(per_seg_series, fs=FS, min_n=8):
    """Duration-weighted-average PSD (for plotting), never across a gap."""
    f_common, Pxx_sum, dur_sum = None, None, 0.0
    for tg_seg, x_seg in per_seg_series:
        n = len(x_seg)
        if n < min_n:
            continue
        nper = min(n, max(min_n, int(fs * 10)))
        f, Pxx = welch(x_seg, fs=fs, nperseg=nper)
        dur = n / fs
        if f_common is None:
            f_common = f
        Pxx_i = np.interp(f_common, f, Pxx)
        Pxx_sum = Pxx_i * dur if Pxx_sum is None else Pxx_sum + Pxx_i * dur
        dur_sum += dur
    if Pxx_sum is None or dur_sum == 0:
        return None, None
    return f_common, Pxx_sum / dur_sum


def compute_metrics(traces, sp):
    alt_all = np.concatenate([t["alt"] for t in traces])
    thrust_all = np.concatenate([t["thrust_cmd"] for t in traces])
    dhat_all = np.concatenate([t["dhat"][1:] for t in traces]) if len(traces) else np.array([0.0])
    dob_clamp_all = np.concatenate([t["dob_clamp_hits"][1:] for t in traces]) if len(traces) else np.array([False])
    pi_clamp_all = np.concatenate([t["pi_clamp_hits"][1:] for t in traces]) if len(traces) else np.array([False])
    dthr_all = np.concatenate([np.diff(t["thrust_cmd"]) for t in traces])

    alt_std = float(np.std(alt_all))
    alt_rms = float(np.sqrt(np.mean((alt_all - sp) ** 2)))
    alt_p2p = float(np.max(alt_all) - np.min(alt_all))

    band_low = band_rms_segments([(t["t"], t["alt"] - sp) for t in traces], 0.02, 0.1)
    band_mid = band_rms_segments([(t["t"], t["alt"] - sp) for t in traces], 0.1, 0.5)
    band_high = band_rms_segments([(t["t"], t["alt"] - sp) for t in traces], 0.5, 2.0)

    return dict(
        alt_std_mm=alt_std * 1000, alt_rms_mm=alt_rms * 1000, alt_p2p_mm=alt_p2p * 1000,
        band_low_mm=band_low * 1000, band_mid_mm=band_mid * 1000, band_high_mm=band_high * 1000,
        thrust_std_mN=float(np.std(thrust_all)) * 1000,
        thrust_diff_std_mN=float(np.std(dthr_all)) * 1000,
        dhat_std_mN=float(np.std(dhat_all)) * 1000,
        dhat_mean_mN=float(np.mean(dhat_all)) * 1000,
        dob_clamp_sat_pct=float(np.mean(dob_clamp_all)) * 100,
        pi_clamp_sat_pct=float(np.mean(pi_clamp_all)) * 100,
        n_samples=int(len(alt_all)),
    )


def measured_metrics(L):
    alt = L["altg"][~L["invalid"]]
    sp = L["sp"]
    return dict(std_mm=float(np.std(alt)) * 1000, rms_mm=float(np.sqrt(np.mean((alt - sp) ** 2))) * 1000,
                p2p_mm=float(np.max(alt) - np.min(alt)) * 1000)


# =============================================================================
# Linear analysis: DOB internal-loop stability + outer-loop sensitivity
# 線形解析: DOB内部ループ安定性 + 外側ループ感度関数
# =============================================================================
def G_tf(w, L=L_ACT, T=T_ACT, g=G_ACT):
    s = 1j * w
    return g * np.exp(-s * L) / (1 + s * T)


def Q_tf(w, fc, order):
    wc = 2 * np.pi * fc
    s = 1j * w
    if order == 2:
        return wc ** 2 / (s ** 2 + np.sqrt(2) * wc * s + wc ** 2)
    return wc / (s + wc)


def W_tf(w, fc=WASHOUT_FC, enabled=True):
    if not enabled:
        return np.ones_like(w, dtype=complex)
    wc = 2 * np.pi * fc
    s = 1j * w
    return s / (s + wc)


PERTURBATIONS = [
    dict(name="delay +30ms", dL=0.030, gain_mult=1.0),
    dict(name="delay +50ms", dL=0.050, gain_mult=1.0),
    dict(name="delay -30ms", dL=-0.030, gain_mult=1.0),
    dict(name="thrust gain x0.7", dL=0.0, gain_mult=0.7),
    dict(name="thrust gain x1.3", dL=0.0, gain_mult=1.3),
    dict(name="mass +10%", dL=0.0, gain_mult=1.0 / 1.1),
    dict(name="mass -10%", dL=0.0, gain_mult=1.0 / 0.9),
]


def loop_L_DOB(w, fc, order, washout_enabled, pert):
    """L_DOB(s) = W(s) Q(s) [G'(s) - G(s)]  -- see derivation in the report:
    residual = [G'-G]u + d ; d_hat = WQ*residual ; u = u_c - d_hat
    => u[1+WQ(G'-G)] = u_c - WQ*d  => characteristic loop = WQ(G'-G)."""
    Gn = G_tf(w)
    Gp = G_tf(w, L=L_ACT + pert["dL"], g=G_ACT * pert["gain_mult"])
    Qf = Q_tf(w, fc, order)
    Wf = W_tf(w, enabled=washout_enabled)
    return Wf * Qf * (Gp - Gn)


def stability_margin(w, Lw):
    dist = np.abs(1.0 + Lw)
    idx = int(np.argmin(dist))
    sm = float(dist[idx])
    w_at_min = float(w[idx])
    mag = np.abs(Lw)
    phase_deg = np.degrees(np.angle(Lw))
    gm_db, pm_deg = np.nan, np.nan
    # phase crossover: where phase crosses -180 deg (loop most likely to be near -1)
    ph_wrapped = ((phase_deg + 180) % 360) - 180
    sign_cross = np.where(np.diff(np.sign(ph_wrapped + 180)) != 0)[0]
    if len(sign_cross) and mag[sign_cross[0]] > 0:
        gm_db = float(-20 * np.log10(mag[sign_cross[0]]))
    over = np.where(mag >= 1.0)[0]
    if len(over):
        i0 = over[0]
        pm_deg = float(180 + phase_deg[i0])
    return dict(min_dist_to_minus1=sm, freq_at_min_hz=w_at_min / (2 * np.pi),
                peak_gain=float(np.max(mag)), gain_margin_db=gm_db, phase_margin_deg=pm_deg)


def S_alt_tf(w, fc=None, order=2, washout_enabled=True, dob_on=True, vti=VTI_HOVER):
    """Outer-loop sensitivity alt(jw)/d_ext(jw), NOMINAL plant (G'=G) so the
    DOB provides EXACT feed-forward cancellation of the internal-loop term
    (residual == d_ext exactly with a matched model); see report derivation:
      S_alt_DOB(jw) = S_alt_noDOB(jw) * [1 - G(jw) W(jw) Q(jw)]
    with S_alt_noDOB(jw) = 1 / (M (jw)^2 D(jw)),
      D(jw) = 1 + P(jw) Cv(jw) (Ca(jw)/(jw) + E(jw)),  P(jw) = G(jw)/(M jw)."""
    s = 1j * w
    Gn = G_tf(w)
    E = 1.0 / (1 + s * TAU_ESKF)
    Ca = AKP * (1 + 1 / (s * ATI))
    Cv = VKP * (1 + 1 / (s * vti))
    P = Gn / (M * s)
    D = 1 + P * Cv * (Ca / s + E)
    S_noDOB = 1.0 / (M * s ** 2 * D)
    if not dob_on:
        return S_noDOB
    Qf = Q_tf(w, fc, order)
    Wf = W_tf(w, enabled=washout_enabled)
    T_dob = 1 - Gn * Wf * Qf
    return S_noDOB * T_dob


# =============================================================================
# Falsification test 5: velocity-loop-ONLY open-loop margins at vkp=0.1 vs 0.2
# 反証テスト5: 速度ループ単体の一巡伝達 vkp=0.1 vs 0.2
# =============================================================================
def vel_loop_open_loop(w, vkp, vti=VTI_HOVER):
    """L_v(s) = Cv(s) * G(s)/(M s) * E(s) -- velocity loop opened at vz_meas:
    Cv=PI controller, G=actuation model (delay+lag), 1/(Ms)=force->velocity
    integrator, E=ESKF measurement lag. This is the loop the firmware's own
    'raising vel.kp is counterproductive' note (params.cpp) is about -- it
    sees the SAME actuation lag as the outer alt.kp damping path."""
    s = 1j * w
    Cv = vkp * (1 + 1 / (s * vti))
    Gn = G_tf(w)
    E = 1.0 / (1 + s * TAU_ESKF)
    return Cv * (Gn / (M * s)) * E


def bode_margins(w_hz, Lw):
    """Classical GM/PM via the FIRST (lowest-frequency) 0dB and -180deg
    crossings of the unwrapped phase -- adequate for this single-hump PI +
    delay/lag loop (verified monotonic mag rolloff in the range examined)."""
    mag = np.abs(Lw)
    phase = np.degrees(np.unwrap(np.angle(Lw)))
    gm_db, pm_deg, f_gc, f_pc = np.nan, np.nan, np.nan, np.nan
    over = np.where(mag >= 1.0)[0]
    if len(over) and over[-1] < len(mag) - 1:
        # gain crossover = last index while mag>=1 before it drops below (mag is ~monotonic decreasing)
        i = over[-1]
        pm_deg = float(180 + phase[i])
        f_gc = float(w_hz[i])
    below_180 = np.where(np.diff(np.sign(phase + 180)) != 0)[0]
    if len(below_180):
        i = below_180[0]
        gm_db = float(-20 * np.log10(mag[i]))
        f_pc = float(w_hz[i])
    return dict(gain_margin_db=gm_db, phase_margin_deg=pm_deg,
                gain_crossover_hz=f_gc, phase_crossover_hz=f_pc)


# =============================================================================
# Plots
# =============================================================================
def plot_sensitivity_and_psd(f_hz, dext_psd_A, dext_psd_D):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    S0 = np.abs(S_alt_tf(2 * np.pi * f_hz, dob_on=False))
    ax1.loglog(f_hz, S0, "k-", lw=2, label="no DOB")
    for fc in FC_LIST:
        S1 = np.abs(S_alt_tf(2 * np.pi * f_hz, fc=fc, order=2, washout_enabled=True, dob_on=True))
        ax1.loglog(f_hz, S1, lw=1.3, label=f"DOB fc={fc}Hz")
    ax1.set_ylabel("|S_alt(jw)| = |alt/d_ext| [m/N]")
    ax1.set_title("Outer-loop disturbance sensitivity (nominal plant, no DOB vs DOB fc sweep)")
    ax1.legend(fontsize=7)
    ax1.grid(which="both", alpha=0.3)

    if dext_psd_A[0] is not None:
        ax2.loglog(dext_psd_A[0], np.sqrt(dext_psd_A[1]), label="d_ext_acc ASD, log A (hands-off)")
    if dext_psd_D[0] is not None:
        ax2.loglog(dext_psd_D[0], np.sqrt(dext_psd_D[1]), label="d_ext_acc ASD, log D (venue)")
    ax2.set_xlabel("frequency [Hz]")
    ax2.set_ylabel("d_ext amplitude spectral density [N/sqrt(Hz)]")
    ax2.set_xlim(0.02, 5)
    ax2.legend(fontsize=8)
    ax2.grid(which="both", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "sensitivity_dext_psd.png", dpi=150)
    plt.close(fig)


def plot_fc_sweep_bars(perf_table):
    logs = ["A", "B", "C", "D"]
    fcs = FC_LIST
    fig, ax = plt.subplots(figsize=(9, 5))
    width = 0.18
    x = np.arange(len(fcs))
    for i, name in enumerate(logs):
        vals = [perf_table[name][f"2_dob_fc{fc}"]["alt_rms_mm"] for fc in fcs]
        base = perf_table[name]["1_baseline"]["alt_rms_mm"]
        pct = [100 * (v / base - 1) for v in vals]
        ax.bar(x + i * width, pct, width, label=f"log {name}")
    ax.axhline(0, color="k", lw=0.8)
    ax.set_xticks(x + 1.5 * width)
    ax.set_xticklabels([f"fc={fc}Hz" for fc in fcs])
    ax.set_ylabel("alt RMS change vs baseline [%]")
    ax.set_title("DOB fc sweep: altitude RMS(vs sp) change relative to baseline (2nd-order Q, washout on)")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "fc_sweep_bars.png", dpi=150)
    plt.close(fig)


def plot_timeseries(traces_base, traces_dob, name, sp):
    t0_base = np.concatenate([t["t"] for t in traces_base])
    alt_base = np.concatenate([t["alt"] for t in traces_base])
    thr_base = np.concatenate([t["thrust_cmd"] for t in traces_base])
    t0_dob = np.concatenate([t["t"] for t in traces_dob])
    alt_dob = np.concatenate([t["alt"] for t in traces_dob])
    thr_dob = np.concatenate([t["thrust_cmd"] for t in traces_dob])
    dhat_dob = np.concatenate([t["dhat"] for t in traces_dob])

    # representative 40s window from the middle of the longest segment
    lens = [len(t["t"]) for t in traces_base]
    k = int(np.argmax(lens))
    tt = traces_base[k]["t"]
    c = tt[0] + (tt[-1] - tt[0]) / 2
    w = 20.0
    m0 = (t0_base >= c - w) & (t0_base <= c + w)
    m1 = (t0_dob >= c - w) & (t0_dob <= c + w)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    ax1.plot(t0_base[m0], (alt_base[m0] - sp) * 1000, label="baseline (no DOB)", lw=1)
    ax1.plot(t0_dob[m1], (alt_dob[m1] - sp) * 1000, label="DOB fc=1.5Hz", lw=1, alpha=0.85)
    ax1.set_ylabel("alt - sp [mm]")
    ax1.set_title(f"log {name}: baseline vs recommended DOB, representative {2*w:.0f}s window")
    ax1.legend(fontsize=8)
    ax1.grid(alpha=0.3)

    ax2.plot(t0_base[m0], thr_base[m0] * 1000, label="thrust_cmd, baseline", lw=1)
    ax2.plot(t0_dob[m1], thr_dob[m1] * 1000, label="thrust_cmd, DOB", lw=1, alpha=0.85)
    ax2.plot(t0_dob[m1], dhat_dob[m1] * 1000, label="d_hat, DOB", lw=1, ls="--")
    ax2.set_ylabel("force [mN]")
    ax2.set_xlabel("t [s]")
    ax2.legend(fontsize=8)
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / f"timeseries_{name}.png", dpi=150)
    plt.close(fig)


def plot_robustness_map(robust_table):
    names = [p["name"] for p in PERTURBATIONS]
    fig, ax = plt.subplots(figsize=(8, 5))
    mat = np.array([[robust_table[(pn, fc)]["min_dist_to_minus1"] for fc in FC_LIST] for pn in names])
    im = ax.imshow(mat, aspect="auto", cmap="RdYlGn", vmin=0, vmax=1.0)
    ax.set_xticks(range(len(FC_LIST)))
    ax.set_xticklabels([f"{fc}Hz" for fc in FC_LIST])
    ax.set_yticks(range(len(names)))
    ax.set_yticklabels(names, fontsize=8)
    for i in range(len(names)):
        for j in range(len(FC_LIST)):
            ax.text(j, i, f"{mat[i,j]:.3f}", ha="center", va="center", fontsize=7)
    ax.set_xlabel("Q-filter cutoff fc")
    ax.set_title("DOB inner-loop Nyquist distance to -1: min|1+L_DOB(jw)| (larger=more robust)")
    fig.colorbar(im, ax=ax, label="min distance to -1")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "robustness_map.png", dpi=150)
    plt.close(fig)


def plot_band_rms(perf_table):
    logs = ["A", "B", "C", "D"]
    cases = ["1_baseline", "2_dob_fc1.5", "5_dob_fc1.5_ti2.5", "6_novdob_vkp0.2"]
    bands = ["band_low_mm", "band_mid_mm", "band_high_mm"]
    fig, axes = plt.subplots(1, len(logs), figsize=(16, 4.5), sharey=False)
    x = np.arange(len(cases))
    width = 0.25
    for ax, name in zip(axes, logs):
        for bi, band in enumerate(bands):
            vals = [perf_table[name][c][band] for c in cases]
            ax.bar(x + bi * width, vals, width, label=band.replace("_mm", ""))
        ax.set_xticks(x + width)
        ax.set_xticklabels(["base", "DOB1.5", "DOB+ti2.5", "kp0.2"], rotation=20, fontsize=7)
        ax.set_title(f"log {name}")
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("alt band RMS [mm]")
    axes[0].legend(fontsize=7)
    fig.suptitle("Altitude band RMS: low(0.02-0.1Hz) / mid(0.1-0.5Hz) / high(0.5-2Hz)")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "band_rms.png", dpi=150)
    plt.close(fig)


def plot_artifact_detection(artifact_results):
    """artifact_results: dict name -> dict(sample0=..., avg025s=...) each an
    artifact_split_metrics() output. Grouped bars: early vs late |d_hat| and
    alt RMS-vs-sp, for BOTH init modes, per log -- the direct falsification
    figure for the segment-reset-transient hypothesis."""
    names = list(artifact_results.keys())
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    x = np.arange(len(names))
    width = 0.2
    labels = [("sample0", "early"), ("sample0", "late"), ("avg025s", "early"), ("avg025s", "late")]
    for ax, key, ylabel in ((axes[0], "dhat_mean_mN", "mean |d_hat| [mN]"),
                             (axes[1], "alt_rms_mm", "alt RMS vs sp [mm]")):
        for i, (mode, part) in enumerate(labels):
            vals = [artifact_results[n][mode][f"{part}_{key}"] for n in names]
            ax.bar(x + i * width, vals, width, label=f"{mode}/{part}")
        ax.set_xticks(x + 1.5 * width)
        ax.set_xticklabels(names)
        ax.set_ylabel(ylabel)
        ax.legend(fontsize=7)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Segment-reset artifact test: first 5s vs remainder, sample0 vs avg025s init (DOB fc=1.5Hz)")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "artifact_detection.png", dpi=150)
    plt.close(fig)


def plot_waterbed_prediction_vs_observed(waterbed_results):
    """waterbed_results: dict name -> dict(f_pred, ratio_pred, f_obs, ratio_obs)
    -- predicted |S_DOB/S_noDOB|^2 (pure linear model, no data) vs the OBSERVED
    ratio of replayed alt PSD (DOB fc1.5 / baseline), duration-weighted Welch,
    both on the SAME log's data. Falsification test for the 'DOB hurts C/D
    via 1-3Hz sensitivity amplification' mechanism claim."""
    names = list(waterbed_results.keys())
    fig, axes = plt.subplots(1, len(names), figsize=(6 * len(names), 4.5), squeeze=False)
    for i, name in enumerate(names):
        r = waterbed_results[name]
        ax = axes[0, i]
        ax.semilogx(r["f_pred"], r["ratio_pred"], "k-", lw=2, label="predicted |S_DOB/S_noDOB|^2 (linear model)")
        ax.semilogx(r["f_obs"], r["ratio_obs"], "o-", ms=3, lw=1, color="tab:red",
                     label="observed alt PSD ratio (DOB fc1.5 / baseline, replay)")
        ax.axhline(1.0, color="gray", ls=":", lw=1)
        ax.set_xlabel("frequency [Hz]")
        ax.set_ylabel("alt PSD ratio (DOB/no-DOB)")
        ax.set_title(f"log {name}")
        ax.legend(fontsize=7)
        ax.grid(which="both", alpha=0.3)
    fig.suptitle("Waterbed mechanism falsification: predicted vs observed sensitivity ratio")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "waterbed_pred_vs_obs.png", dpi=150)
    plt.close(fig)


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 100)
    print("Loading 4 logs onto FS=200Hz gap-aware grid (reusing step1 windows)")
    print("=" * 100)
    dA = s1.load_jsonl(str(LOG_A))
    dB = s1.load_jsonl(str(LOG_B))
    flightA_raw = s1.detect_flight_window(dA["ctrl_ref"]["ts"], dA["ctrl_ref"]["motor_duty"])
    flightB_raw = s1.detect_flight_window(dB["ctrl_ref"]["ts"], dB["ctrl_ref"]["motor_duty"])
    # EDGE_TRIM: A/B windows are auto-detected motor-duty thresholds that land
    # EXACTLY on the takeoff/landing thrust step (verified: flightA[0]==the
    # measured takeoff edge in step1_results.json to the ms). Replaying THAT
    # instant injects the highly nonlinear takeoff transient's model-mismatch
    # (d_ext_acc peaks at ~+380mN, comparable to HOVER itself, right at t=0 of
    # the segment) as if it were a steady external disturbance -- this alone
    # produces a spurious multi-metre runaway with no relation to the DOB
    # design question. Trimmed by the SAME 3s margin step1 itself uses for its
    # own noise-budget/DC-gain windows (dc_gain_check, step3_noise_budget).
    # C/D windows are the task-given fixed values (already clear of the
    # takeoff/landing edge, verified by inspection) -- left untouched.
    # EDGE_TRIM: A/Bの窓は自動検出のため離着陸のスラストステップの瞬間に一致する
    # （検証済み: flightA[0]はstep1の離陸イベント時刻とms単位で一致）。この瞬間を
    # そのまま再生すると、離陸過渡の強い非線形性によるモデル誤差(d_ext_accが
    # t=0でHOVER相当の+380mN)を定常外乱として注入してしまい、DOB設計とは無関係な
    # 数m級の暴走を生む。step1自身が使うのと同じ3sマージンでトリムする。
    # C/Dはタスク指定の固定窓（離着陸端から十分離れていることを確認済み）で変更しない。
    EDGE_TRIM_S = 3.0
    flightA = (flightA_raw[0] + EDGE_TRIM_S, flightA_raw[1] - EDGE_TRIM_S)
    flightB = (flightB_raw[0] + EDGE_TRIM_S, flightB_raw[1] - EDGE_TRIM_S)
    windows = {"A": (LOG_A, flightA), "B": (LOG_B, flightB), "C": (LOG_C, WIN_C), "D": (LOG_D, WIN_D)}

    logs = {}
    for name, (path, win) in windows.items():
        L = load_full_log(path, win)
        cov = sum(e - s for s, e in L["segs"]) / FS
        print(f"  log {name}: window={win[1]-win[0]:.1f}s  n_segments={len(L['segs'])}  "
              f"valid_coverage={cov:.1f}s ({100*cov/(win[1]-win[0]):.0f}%)  sp={L['sp']:.3f}m")
        logs[name] = L

    meas = {name: measured_metrics(L) for name, L in logs.items()}
    for name, m in meas.items():
        print(f"  measured alt: {name}: std={m['std_mm']:.1f}mm rms={m['rms_mm']:.1f}mm p2p={m['p2p_mm']:.1f}mm")

    print()
    print("=" * 100)
    print("Reconstructing d_ext (acc-based, primary; kin-based, secondary), band-limited <=2Hz per segment")
    print("=" * 100)
    dext_acc = {name: compute_dext_all(L, "acc") for name, L in logs.items()}
    dext_kin = {name: compute_dext_all(L, "kin") for name, L in logs.items()}
    for name in logs:
        d_acc_cat = np.concatenate([d for _, _, d, _ in dext_acc[name]])
        d_kin_cat = np.concatenate([d for _, _, d, _ in dext_kin[name]])
        print(f"  log {name}: d_ext_acc(<=2Hz) std={np.std(d_acc_cat)*1000:.1f}mN  "
              f"d_ext_kin(<=2Hz) std={np.std(d_kin_cat)*1000:.1f}mN  "
              f"ratio acc/kin={np.std(d_acc_cat)/np.std(d_kin_cat):.2f}")

    print()
    print("=" * 100)
    print("VERIFICATION GATE 1: new plant model (delay+lag split + ESKF lag) + d_ext_kin injection, "
          "baseline gains, DOB off -- reproduce measured alt std within +-30%?")
    print("=" * 100)
    gate1 = {}
    baseline_cfg = default_cfg()
    for name, L in logs.items():
        traces = replay_all(L, dext_kin[name], baseline_cfg)
        m = compute_metrics(traces, L["sp"])
        ratio = m["alt_std_mm"] / meas[name]["std_mm"]
        ok = 0.7 <= ratio <= 1.3
        gate1[name] = dict(replay_std_mm=m["alt_std_mm"], measured_std_mm=meas[name]["std_mm"],
                            ratio=ratio, pass_=ok)
        print(f"  {name}: replay={m['alt_std_mm']:.1f}mm  measured={meas[name]['std_mm']:.1f}mm  "
              f"ratio={ratio:.2f}  {'PASS' if ok else 'FAIL'}")

    print()
    print("=" * 100)
    print("VERIFICATION GATE 2: same plant/gains, DOB off, d_ext_acc injection (larger-amplitude estimate) --")
    print("does it also reproduce measured alt std within +-30%?")
    print("=" * 100)
    gate2 = {}
    for name, L in logs.items():
        traces = replay_all(L, dext_acc[name], baseline_cfg)
        m = compute_metrics(traces, L["sp"])
        ratio = m["alt_std_mm"] / meas[name]["std_mm"]
        ok = 0.7 <= ratio <= 1.3
        gate2[name] = dict(replay_std_mm=m["alt_std_mm"], measured_std_mm=meas[name]["std_mm"],
                            ratio=ratio, pass_=ok)
        print(f"  {name}: replay={m['alt_std_mm']:.1f}mm  measured={meas[name]['std_mm']:.1f}mm  "
              f"ratio={ratio:.2f}  {'PASS' if ok else 'FAIL'}")

    # ========================================================================
    # FALSIFICATION TEST 1: direct artifact detection (2026-07-18 coordinator
    # review). Hypothesis: on gap-fragmented C/D (segments 2.8-14s), the
    # washout's ~5.3s time constant needs most/all of a short segment to walk
    # down from a wrongly-seeded initial condition -- a REPLAY-ONLY artifact,
    # since the real firmware loop never resets. Falsifiable prediction: if
    # true, |d_hat| and alt error should be concentrated in the first 5s of
    # each segment under the OLD "sample0" init, and this concentration
    # should shrink substantially under the NEW "avg025s" init.
    # 反証テスト1: 区間頭アーティファクトの直接検出。仮説が正しければ旧初期化で
    # 区間頭5秒に|d_hat|・alt誤差が集中し、新初期化でその集中が縮小するはず。
    # ========================================================================
    print()
    print("=" * 100)
    print("FALSIFICATION TEST 1: segment-reset artifact -- first 5s vs remainder (DOB fc=1.5Hz)")
    print("=" * 100)
    artifact_results = {}
    for name, L in logs.items():
        cfg_old = default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                               dob_init_mode="sample0")
        cfg_new = default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                               dob_init_mode="avg025s")
        tr_old = replay_all(L, dext_acc[name], cfg_old)
        tr_new = replay_all(L, dext_acc[name], cfg_new)
        a_old = artifact_split_metrics(tr_old, L["sp"])
        a_new = artifact_split_metrics(tr_new, L["sp"])
        artifact_results[name] = dict(sample0=a_old, avg025s=a_new)
        print(f"  log {name} (n_segments={len(L['segs'])}):")
        print(f"    sample0 (OLD): early|d_hat|={a_old['early_dhat_mean_mN']:7.2f}mN "
              f"late|d_hat|={a_old['late_dhat_mean_mN']:7.2f}mN  |  "
              f"early_alt_rms={a_old['early_alt_rms_mm']:8.1f}mm late_alt_rms={a_old['late_alt_rms_mm']:8.1f}mm")
        print(f"    avg025s (NEW): early|d_hat|={a_new['early_dhat_mean_mN']:7.2f}mN "
              f"late|d_hat|={a_new['late_dhat_mean_mN']:7.2f}mN  |  "
              f"early_alt_rms={a_new['early_alt_rms_mm']:8.1f}mm late_alt_rms={a_new['late_alt_rms_mm']:8.1f}mm")
    print("  (early=first 5s of EACH segment, late=remainder; alt_rms is vs the log's own sp, not each")
    print("   segment's own mean, so a persistent early-segment bias is NOT hidden by local-mean removal)")

    # ========================================================================
    # MAIN CASE SWEEP (d_ext_acc injection, primary) -- NEW avg025s init is
    # now default_cfg()'s default. A SECOND full sweep with the OLD sample0
    # init is also run for direct old-vs-new comparison (falsification test 2).
    # メイン掃引: default_cfg()の既定は新avg025s初期化。旧sample0版も全ケース
    # 再実行し直接比較する（反証テスト2）。
    # ========================================================================
    print()
    print("=" * 100)
    print("MAIN CASE SWEEP (d_ext_acc injection, primary, REVISED avg025s DOB init)")
    print("=" * 100)

    def make_cases(init_mode):
        return {
            "1_baseline": default_cfg(dob_enabled=False),
            "2_dob_fc0.5": default_cfg(dob_enabled=True, dob_fc=0.5, dob_order=2, washout_enabled=True,
                                        dob_init_mode=init_mode),
            "2_dob_fc1.0": default_cfg(dob_enabled=True, dob_fc=1.0, dob_order=2, washout_enabled=True,
                                        dob_init_mode=init_mode),
            "2_dob_fc1.5": default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                                        dob_init_mode=init_mode),
            "2_dob_fc2.0": default_cfg(dob_enabled=True, dob_fc=2.0, dob_order=2, washout_enabled=True,
                                        dob_init_mode=init_mode),
            "2_dob_fc3.0": default_cfg(dob_enabled=True, dob_fc=3.0, dob_order=2, washout_enabled=True,
                                        dob_init_mode=init_mode),
            "3_dob_fc1.5_order1": default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=1, washout_enabled=True,
                                               dob_init_mode=init_mode),
            "4_dob_fc1.5_nowashout": default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=False,
                                                  dob_init_mode=init_mode),
            "5_dob_fc1.5_ti2.5": default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                                              vti=2.5, dob_init_mode=init_mode),
            "6_novdob_vkp0.2": default_cfg(dob_enabled=False, vkp=0.2, vti=VTI_HOVER),
        }

    cases = make_cases("avg025s")
    cases_old = make_cases("sample0")

    # NOTE on metrics (2026-07-18 review, item 7): "alt_std" (std around the
    # run's OWN mean) is used as the PRIMARY comparison metric everywhere
    # below, matching the Gate 1/2 methodology exactly (gates compare std to
    # std). "alt_rms" (rms vs the log's sp, so it also carries any mean-vs-sp
    # bias) is reported alongside as a SECONDARY column -- large std/rms gaps
    # flag a persistent offset (e.g. from segment-reset transients or a
    # DC-biased d_ext), not just "wobble". The previous report used alt_rms
    # for the main table and alt_std for the gates, which for C/D (620 vs
    # 820mm) looked like an unexplained inconsistency -- it wasn't a bug, just
    # two different metrics; this revision uses one metric consistently.
    # 指標の統一（2026-07-18レビュー項目7）: 以下すべてでalt_std（自身の平均まわりの
    # 標準偏差）を主指標とし、Gate1/2と同じ定義に統一する。alt_rms（sp基準、平均の
    # オフセットも含む）は副次列として併記する。前回はメイン表にalt_rms・ゲートに
    # alt_stdを使っており、C/Dで620対820mmのような食い違いに見えたが、バグでは
    # なく指標の違いだった。今回は一貫してalt_stdを基準に統一する。
    perf_table = {}
    perf_table_old = {}
    traces_cache = {}   # (log, case) -> traces, kept for timeseries/I_hist/waterbed plots
    for name, L in logs.items():
        perf_table[name] = {}
        perf_table_old[name] = {}
        print(f"\n  --- log {name} ---")
        print(f"  {'case':24s} {'alt_std':>9s} {'alt_rms':>9s} {'p2p':>8s} {'vs base(std)':>13s} "
              f"{'thr_std':>9s} {'dthr_std':>9s} {'dhat_std':>9s} {'DOBsat%':>8s} {'PIsat%':>8s}")
        base_std = None
        for cname, cfg in cases.items():
            traces = replay_all(L, dext_acc[name], cfg)
            m = compute_metrics(traces, L["sp"])
            if base_std is None:
                base_std = m["alt_std_mm"]
            m["vs_base_pct"] = 100 * (m["alt_std_mm"] / base_std - 1)
            perf_table[name][cname] = m
            if cname in ("1_baseline", "2_dob_fc1.5", "4_dob_fc1.5_nowashout"):
                traces_cache[(name, cname)] = traces
            print(f"  {cname:24s} {m['alt_std_mm']:9.1f} {m['alt_rms_mm']:9.1f} {m['alt_p2p_mm']:8.1f} "
                  f"{m['vs_base_pct']:12.1f}% {m['thrust_std_mN']:9.2f} {m['thrust_diff_std_mN']:9.3f} "
                  f"{m['dhat_std_mN']:9.2f} {m['dob_clamp_sat_pct']:8.2f} {m['pi_clamp_sat_pct']:8.2f}")
        # OLD-init parallel sweep (falsification test 2, no separate printout -- summarized in the
        # comparison table right after this loop)
        base_std_old = None
        for cname, cfg in cases_old.items():
            traces = replay_all(L, dext_acc[name], cfg)
            m = compute_metrics(traces, L["sp"])
            if base_std_old is None:
                base_std_old = m["alt_std_mm"]
            m["vs_base_pct"] = 100 * (m["alt_std_mm"] / base_std_old - 1)
            perf_table_old[name][cname] = m

    print()
    print("=" * 100)
    print("FALSIFICATION TEST 2: OLD (sample0) vs NEW (avg025s) init, full case sweep, alt_std [mm]")
    print("=" * 100)
    print(f"  {'log':4s} {'case':24s} {'old(sample0)':>13s} {'new(avg025s)':>13s} {'change':>9s}")
    for name in logs:
        for cname in cases:
            if cname == "6_novdob_vkp0.2":
                continue   # no DOB in this case -> init mode irrelevant, identical by construction
            o = perf_table_old[name][cname]["alt_std_mm"]
            n = perf_table[name][cname]["alt_std_mm"]
            print(f"  {name:4s} {cname:24s} {o:13.1f} {n:13.1f} {100*(n/o-1):8.1f}%")

    print()
    print("=" * 100)
    print("d_ext_kin re-run (baseline + recommended DOB) -- report acc/kin range")
    print("=" * 100)
    kin_check = {}
    for name, L in logs.items():
        kin_check[name] = {}
        for cname in ("1_baseline", "2_dob_fc1.5"):
            traces = replay_all(L, dext_kin[name], cases[cname])
            m = compute_metrics(traces, L["sp"])
            kin_check[name][cname] = m
        acc_b, kin_b = perf_table[name]["1_baseline"]["alt_std_mm"], kin_check[name]["1_baseline"]["alt_std_mm"]
        acc_d, kin_d = perf_table[name]["2_dob_fc1.5"]["alt_std_mm"], kin_check[name]["2_dob_fc1.5"]["alt_std_mm"]
        print(f"  {name}: baseline alt_std  acc={acc_b:.1f}mm kin={kin_b:.1f}mm  |  "
              f"DOB1.5 alt_std  acc={acc_d:.1f}mm kin={kin_d:.1f}mm  |  "
              f"DOB benefit  acc={100*(acc_d/acc_b-1):.1f}%  kin={100*(kin_d/kin_b-1):.1f}%")

    # ========================================================================
    # FALSIFICATION TEST 3: log-level DC removal from d_ext_acc, C/D only
    # 反証テスト3: C/Dでd_ext_accのログ全体DCを除去して注入
    # ========================================================================
    print()
    print("=" * 100)
    print("FALSIFICATION TEST 3: d_ext_acc with log-level DC removed (C/D only)")
    print("=" * 100)
    dc_removal_results = {}
    for name in ("C", "D"):
        dext_dc0, dc_val = remove_log_dc(dext_acc[name])
        dc_removal_results[name] = dict(dc_removed_mN=dc_val * 1000)
        print(f"  log {name}: removed DC = {dc_val*1000:.1f}mN")
        for cname in ("1_baseline", "2_dob_fc1.5"):
            traces = replay_all(logs[name], dext_dc0, cases[cname])
            m = compute_metrics(traces, logs[name]["sp"])
            dc_removal_results[name][cname] = m
            orig = perf_table[name][cname]["alt_std_mm"]
            print(f"    {cname:24s} alt_std(DC removed)={m['alt_std_mm']:8.1f}mm  "
                  f"(orig with DC={orig:8.1f}mm)")

    print()
    print("=" * 100)
    print("Washout-off DC interaction check (log A, longest continuous run): "
          "vel-loop integrator I state, washout ON vs OFF")
    print("=" * 100)
    tr_wash = traces_cache[("A", "2_dob_fc1.5")]
    tr_nowash = traces_cache[("A", "4_dob_fc1.5_nowashout")]
    for label, trs in (("washout ON ", tr_wash), ("washout OFF", tr_nowash)):
        I_cat = np.concatenate([t["I_hist"][1:] for t in trs])
        dhat_cat = np.concatenate([t["dhat"][1:] for t in trs])
        print(f"  {label}: vel-PID I  mean={np.mean(I_cat)*1000:7.2f}mN std={np.std(I_cat)*1000:7.2f}mN  "
              f"|  d_hat mean={np.mean(dhat_cat)*1000:7.2f}mN std={np.std(dhat_cat)*1000:7.2f}mN")

    # -------------------------------------------------------------- robustness (perturbed replay)
    print()
    print("=" * 100)
    print("PERTURBED REPLAY (logs A, D): +50ms delay / gain x0.7 / both, DOB off vs DOB fc=1.5")
    print("=" * 100)
    pert_replay = {}
    pert_cases = {
        "nominal": dict(dL=0.0, gm=1.0),
        "delay+50ms": dict(dL=0.05, gm=1.0),
        "gain_x0.7": dict(dL=0.0, gm=0.7),
        "both": dict(dL=0.05, gm=0.7),
    }
    for name in ("A", "D"):
        pert_replay[name] = {}
        L = logs[name]
        print(f"\n  --- log {name} ---")
        for pname, p in pert_cases.items():
            for dob_state, base_cname in (("DOB_off", "1_baseline"), ("DOB_fc1.5", "2_dob_fc1.5")):
                cfg = dict(cases[base_cname])
                cfg["plant_L"] = ACT_MODEL["L"] + p["dL"]
                cfg["plant_g"] = ACT_MODEL["g"] * p["gm"]
                traces = replay_all(L, dext_acc[name], cfg)
                m = compute_metrics(traces, L["sp"])
                key = f"{pname}_{dob_state}"
                pert_replay[name][key] = m
                print(f"    {pname:12s} {dob_state:10s} alt_std={m['alt_std_mm']:8.1f}mm "
                      f"thr_std={m['thrust_std_mN']:7.2f}mN dthr_std={m['thrust_diff_std_mN']:7.3f}mN "
                      f"DOBsat%={m['dob_clamp_sat_pct']:5.2f}")

    # -------------------------------------------------------------- linear robustness
    print()
    print("=" * 100)
    print("LINEAR ROBUSTNESS: DOB inner-loop L_DOB(jw)=W(jw)Q(jw)[G'(jw)-G(jw)], stability margins")
    print("=" * 100)
    w = 2 * np.pi * np.logspace(-2, np.log10(20), 4000)
    robust_table = {}
    for pert in PERTURBATIONS:
        for fc in FC_LIST:
            Lw = loop_L_DOB(w, fc, 2, True, pert)
            sm = stability_margin(w, Lw)
            robust_table[(pert["name"], fc)] = sm
    print(f"  {'perturbation':18s}" + "".join(f"{'fc='+str(fc)+'Hz':>12s}" for fc in FC_LIST))
    for pert in PERTURBATIONS:
        row = "".join(f"{robust_table[(pert['name'], fc)]['min_dist_to_minus1']:12.4f}" for fc in FC_LIST)
        print(f"  {pert['name']:18s}{row}")
    print("  (min|1+L_DOB(jw)| over 0.01-20Hz; 1.0=no perturbation effect, smaller=less robust; "
          "loop never has |L|>=1 in this perturbation range for any fc tested here -> classical "
          "GM/PM undefined/infinite, min-distance-to-(-1) used as the robust single metric instead)")

    # ========================================================================
    # FALSIFICATION TEST 5: velocity-loop-ONLY open-loop margins, vkp=0.1 vs 0.2
    # (honest lag model: G(s) delay+lag + ESKF lag E(s) INSIDE the loop, since
    # vz_meas passes through the ESKF before reaching the PI). Cross-check
    # against the firmware's own real-flight note (params.cpp): "raising
    # alt.vel.kp is counterproductive because the damping path sees the same
    # actuation lag" -- and poshold_alt_loop_sim.py's own caveat that ITS
    # sim (single 0.11s lag, no explicit delay) is optimistic here.
    # 反証テスト5: 速度ループ単体の一巡伝達（ESKF遅れも内包）でkp=0.1と0.2の
    # PM/GMを比較。ファーム実機注記「vel.kpを上げるのは逆効果」と突き合わせる。
    # ========================================================================
    print()
    print("=" * 100)
    print("FALSIFICATION TEST 5: velocity-loop-only open-loop margins, vkp=0.1 vs 0.2 (honest lag model)")
    print("=" * 100)
    w_hz_vel = np.logspace(-2, np.log10(20), 8000)
    w_vel = 2 * np.pi * w_hz_vel
    vel_margins = {}
    for vkp in (0.1, 0.15, 0.2):
        Lv = vel_loop_open_loop(w_vel, vkp)
        vm = bode_margins(w_hz_vel, Lv)
        vel_margins[vkp] = vm
        print(f"  vkp={vkp:.2f}: GM={vm['gain_margin_db']:6.2f}dB @ {vm['phase_crossover_hz']:.2f}Hz   "
              f"PM={vm['phase_margin_deg']:6.2f}deg @ {vm['gain_crossover_hz']:.2f}Hz")
    print("  (loop: Cv(s)*G(s)/(M s)*E(s), G=delay60ms+lag20ms actuation, E=ESKF lag 27.6ms, vti=1.5s fixed)")
    print("  Cross-check: firmware params.cpp real-flight note says raising alt.vel.kp is counterproductive")
    print("  because the damping path sees the actuation lag -- see whether PM/GM degrade from 0.1->0.2 here.")

    # ========================================================================
    # FALSIFICATION TEST 6: combined DOB fc1.5 + vel.kp=0.15 (A, D)
    # 反証テスト6: DOB fc1.5 + vel.kp=0.15の組合せ（A, D）
    # ========================================================================
    print()
    print("=" * 100)
    print("FALSIFICATION TEST 6: combined DOB fc1.5 + vel.kp=0.15 (logs A, D)")
    print("=" * 100)
    combo_cfg = default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True, vkp=0.15)
    combo_results = {}
    for name in ("A", "D"):
        traces = replay_all(logs[name], dext_acc[name], combo_cfg)
        m = compute_metrics(traces, logs[name]["sp"])
        combo_results[name] = m
        base = perf_table[name]["1_baseline"]["alt_std_mm"]
        dob_only = perf_table[name]["2_dob_fc1.5"]["alt_std_mm"]
        kp02_only = perf_table[name]["6_novdob_vkp0.2"]["alt_std_mm"]
        print(f"  {name}: combo alt_std={m['alt_std_mm']:7.1f}mm  vs baseline({base:.1f}mm)="
              f"{100*(m['alt_std_mm']/base-1):6.1f}%  vs DOB-only({dob_only:.1f}mm)="
              f"{100*(m['alt_std_mm']/dob_only-1):6.1f}%  vs kp0.2-only({kp02_only:.1f}mm)="
              f"{100*(m['alt_std_mm']/kp02_only-1):6.1f}%")

    # ========================================================================
    # RESIDUAL GUARD TEST (2026-07-18 review follow-up): reviewer flagged that
    # the recommendation included NO validated guard against a single large
    # residual sample leaving a large, slowly-decaying imprint in the Q/
    # washout filter state. Per the review's own follow-up finding, the log C
    # burst near raw ts~57.86s is not pure fabrication -- it is a genuine
    # short transient (-84 to -166mN, <=0.2s, accel/posvel force-balance
    # consistent, occurring at 0.10m altitude i.e. near the floor) that the
    # d_ext_acc pipeline amplifies ~3x in both amplitude and duration. A
    # residual guard should pass the genuine part (<=~166mN) but clip the
    # amplified excess -- tested at +-0.2N (permissive) and +-0.10N (tight,
    # matches the existing dob_clamp) against a clean log (A) to confirm no
    # clean-condition performance cost.
    # 残差ガード検証（2026-07-18レビュー追随）: 単一の大きな残差サンプルが
    # Q/washoutフィルタ状態に大きく緩慢な痕跡を残すことへの対策が未検証との
    # 指摘。レビューの追加発見: logCのt~57.86s付近のバーストは完全な偽ではなく、
    # 実在の短時間トランジェント(-84~-166mN, <=0.2s, 加速度とposvelの力収支が
    # 一致、発生高度0.10m=床すれすれ)をd_ext_accパイプラインが振幅・時間とも
    # 約3倍に膨張させた混成イベント。ガードは実在部分(<=166mN)を通しつつ
    # 膨張分を頭打ちにするのが目的。+-0.2N(緩め)と+-0.10N(既存dob_clampと
    # 同水準)を、清浄ログAでの性能劣化がないことも合わせて検証する。
    # ========================================================================
    print()
    print("=" * 100)
    print("RESIDUAL GUARD TEST: clip |residual| before Q filter, avg025s init, DOB fc=1.5, washout ON")
    print("=" * 100)
    BURST_WIN = (55.86, 59.86)   # raw ts, 57.86s +-2s (log C burst window per coordinator)
    guard_settings = {"none": None, "guard_0.20N": 0.20, "guard_0.10N": 0.10}
    residual_guard_results = {}
    for name in ("A", "C", "D"):
        residual_guard_results[name] = {}
        print(f"\n  --- log {name} ---")
        for gname, gval in guard_settings.items():
            cfg = default_cfg(dob_enabled=True, dob_fc=1.5, dob_order=2, washout_enabled=True,
                               dob_init_mode="avg025s", residual_guard=gval)
            traces = replay_all(logs[name], dext_acc[name], cfg)
            m = compute_metrics(traces, logs[name]["sp"])
            guard_hit_pct = float(np.mean(np.concatenate(
                [t["residual_guard_hits"][1:] for t in traces]))) * 100 if gval else 0.0
            # burst-window max |d_hat| (log C only, but computed for all logs for completeness)
            dhat_in_win = np.concatenate([
                t["dhat"][(t["t"] >= BURST_WIN[0]) & (t["t"] <= BURST_WIN[1])] for t in traces
            ]) if any(np.any((t["t"] >= BURST_WIN[0]) & (t["t"] <= BURST_WIN[1])) for t in traces) else np.array([0.0])
            max_dhat_burst_mN = float(np.max(np.abs(dhat_in_win))) * 1000 if len(dhat_in_win) else float("nan")
            residual_guard_results[name][gname] = dict(
                alt_std_mm=m["alt_std_mm"], alt_rms_mm=m["alt_rms_mm"],
                guard_hit_pct=guard_hit_pct, max_dhat_burst_window_mN=max_dhat_burst_mN,
                dob_clamp_sat_pct=m["dob_clamp_sat_pct"], thrust_std_mN=m["thrust_std_mN"])
            print(f"    {gname:12s} alt_std={m['alt_std_mm']:9.1f}mm  guard_hit%={guard_hit_pct:5.2f}  "
                  f"max|d_hat| in burst window[{BURST_WIN[0]:.2f},{BURST_WIN[1]:.2f}]s = {max_dhat_burst_mN:7.1f}mN  "
                  f"DOBsat%={m['dob_clamp_sat_pct']:5.2f}")
    print(f"\n  (burst window = raw ts {BURST_WIN}; A/D included as clean/comparison logs -- burst is C-specific)")

    # ========================================================================
    # FALSIFICATION TEST 4: waterbed mechanism, predicted vs observed alt PSD
    # ratio (DOB fc1.5 / baseline), C and D. Predicted = pure linear model
    # |S_DOB/S_noDOB|^2 (no data). Observed = Welch PSD ratio of the ACTUAL
    # avg025s-init replay traces already computed above (perf sweep).
    # 反証テスト4: ウォーターベッド機構の予測(線形モデルのみ)vs観測(実際の再生)。
    # ========================================================================
    print()
    print("=" * 100)
    print("FALSIFICATION TEST 4: waterbed mechanism -- predicted vs observed alt PSD ratio (DOB fc1.5/baseline)")
    print("=" * 100)
    waterbed_results = {}
    for name in ("C", "D"):
        base_traces = traces_cache[(name, "1_baseline")]
        dob_traces = traces_cache[(name, "2_dob_fc1.5")]
        sp = logs[name]["sp"]
        f_base, Pxx_base = welch_avg_psd([(t["t"], t["alt"] - sp) for t in base_traces])
        f_dob, Pxx_dob = welch_avg_psd([(t["t"], t["alt"] - sp) for t in dob_traces])
        if f_base is None or f_dob is None:
            print(f"  {name}: insufficient data for PSD (skipped)")
            continue
        fmask = (f_base >= 0.05) & (f_base <= 2.0)
        f_use = f_base[fmask]
        Pxx_dob_i = np.interp(f_use, f_dob, Pxx_dob)
        ratio_obs = Pxx_dob_i / Pxx_base[fmask]
        ratio_pred = (np.abs(S_alt_tf(2 * np.pi * f_use, fc=1.5, order=2, washout_enabled=True, dob_on=True)) ** 2 /
                      np.abs(S_alt_tf(2 * np.pi * f_use, dob_on=False)) ** 2)
        waterbed_results[name] = dict(f_pred=f_use.tolist(), ratio_pred=ratio_pred.tolist(),
                                       f_obs=f_use.tolist(), ratio_obs=ratio_obs.tolist())
        # log-log correlation as a simple "do they move together" check
        valid = np.isfinite(ratio_obs) & (ratio_obs > 0)
        corr = float(np.corrcoef(np.log(ratio_pred[valid]), np.log(ratio_obs[valid]))[0, 1]) if valid.sum() > 3 else float("nan")
        print(f"  {name}: log-log correlation(predicted, observed) over {f_use[0]:.2f}-{f_use[-1]:.2f}Hz = {corr:.2f}")
        for f0 in (0.1, 0.3, 0.5, 1.0):
            i = int(np.argmin(np.abs(f_use - f0)))
            print(f"    f~{f_use[i]:.2f}Hz: predicted ratio={ratio_pred[i]:.2f}  observed ratio={ratio_obs[i]:.2f}")

    # -------------------------------------------------------------- d_ext PSD for sensitivity plot
    f_psd_A, Pxx_A = welch_avg_psd([(logs["A"]["tg"][s:e + 1], dext_acc_for_segment(logs["A"], s, e))
                                     for s, e in logs["A"]["segs"]])
    f_psd_D, Pxx_D = welch_avg_psd([(logs["D"]["tg"][s:e + 1], dext_acc_for_segment(logs["D"], s, e))
                                     for s, e in logs["D"]["segs"]])

    # -------------------------------------------------------------- plots
    print()
    print("Generating figures...")
    f_sens = np.logspace(np.log10(0.02), np.log10(5), 500)
    plot_sensitivity_and_psd(f_sens, (f_psd_A, Pxx_A), (f_psd_D, Pxx_D))
    plot_fc_sweep_bars(perf_table)
    plot_timeseries(traces_cache[("A", "1_baseline")], traces_cache[("A", "2_dob_fc1.5")], "A", logs["A"]["sp"])
    plot_timeseries(traces_cache[("D", "1_baseline")], traces_cache[("D", "2_dob_fc1.5")], "D", logs["D"]["sp"])
    plot_robustness_map(robust_table)
    plot_band_rms(perf_table)
    plot_artifact_detection(artifact_results)
    if waterbed_results:
        plot_waterbed_prediction_vs_observed(waterbed_results)
    print(f"  figures written to {FIGS_DIR}")

    # -------------------------------------------------------------- JSON dump
    results = dict(
        constants=dict(M=M, G=G, HOVER=HOVER, ACT_MODEL=ACT_MODEL, TAU_ESKF=TAU_ESKF, FS=FS,
                       N_DELAY_NOM=N_DELAY_NOM, DELAY_ROUND_ERR_MS=DELAY_ROUND_ERR_MS,
                       cascade=dict(akp=AKP, ati=ATI, alim=ALIM, vkp=VKP, vti_hover=VTI_HOVER, vlim=VLIM),
                       dob=dict(washout_fc=WASHOUT_FC, clamp_N=DOB_CLAMP, fc_list=FC_LIST)),
        flight_windows=dict(A=list(flightA), B=list(flightB), C=list(WIN_C), D=list(WIN_D)),
        measured=meas,
        dext_summary={name: dict(
            acc_std_mN=float(np.std(np.concatenate([d for _, _, d, _ in dext_acc[name]]))) * 1000,
            kin_std_mN=float(np.std(np.concatenate([d for _, _, d, _ in dext_kin[name]]))) * 1000,
        ) for name in logs},
        gate1_kin_baseline=gate1,
        gate2_acc_baseline=gate2,
        artifact_detection=artifact_results,
        perf_table=perf_table,
        perf_table_sample0_init=perf_table_old,
        kin_check=kin_check,
        dc_removal_test=dc_removal_results,
        combo_dob_kp015=combo_results,
        residual_guard_test=residual_guard_results,
        vel_loop_margins={str(k): v for k, v in vel_margins.items()},
        waterbed_pred_vs_obs=waterbed_results,
        pert_replay=pert_replay,
        robust_table={f"{k[0]}|{k[1]}": v for k, v in robust_table.items()},
        washout_dc_check=dict(
            washout_on=dict(I_mean_mN=float(np.mean(np.concatenate([t["I_hist"][1:] for t in tr_wash]))) * 1000,
                             I_std_mN=float(np.std(np.concatenate([t["I_hist"][1:] for t in tr_wash]))) * 1000,
                             dhat_mean_mN=float(np.mean(np.concatenate([t["dhat"][1:] for t in tr_wash]))) * 1000,
                             dhat_std_mN=float(np.std(np.concatenate([t["dhat"][1:] for t in tr_wash]))) * 1000),
            washout_off=dict(I_mean_mN=float(np.mean(np.concatenate([t["I_hist"][1:] for t in tr_nowash]))) * 1000,
                              I_std_mN=float(np.std(np.concatenate([t["I_hist"][1:] for t in tr_nowash]))) * 1000,
                              dhat_mean_mN=float(np.mean(np.concatenate([t["dhat"][1:] for t in tr_nowash]))) * 1000,
                              dhat_std_mN=float(np.std(np.concatenate([t["dhat"][1:] for t in tr_nowash]))) * 1000),
        ),
    )
    with open(RESULTS_PATH, "w") as fh:
        json.dump(results, fh, indent=2, default=lambda o: float(o) if isinstance(o, np.floating) else
                   (bool(o) if isinstance(o, np.bool_) else str(o)))
    print(f"\nResults written to {RESULTS_PATH}")


if __name__ == "__main__":
    main()
