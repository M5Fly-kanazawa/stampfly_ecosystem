"""
yaw_cm_sim.py
=============
実測ヨー外乱トルク τ_d(t) を入力とする閉ループ・ヨー軸シミュレーション。
レートPID（sf_controller_pid/pid.hpp を忠実に写経）+ ヘディングホールド
（pid_controller.hpp のP則）で ψ(t) を再現し、対策シナリオ S0〜S6 を比較する。

【誠実性の方針】
- 結果を仮説に合わせるチューニングは行わない。再現が悪ければ悪いまま報告する。
- 全ての固定数値に出典コメントを付す。
- 測定した τ_d 以外の外乱は注入しない。
- S0 の再現は「τ_d の定義上ある程度自動的に合う」構造的事実であることを明記する
  （τ_d = I_z dr/dt − τ_realized なので、真値の dr/dt を丸ごと外乱扱いすれば
  再構成できて当然。真の検証は (a) 開ループのPIDモデル検証と (b) 反事実
  シナリオの相対比較。§ baseline_reproduction 参照）。

出典（file:line で追跡）:
    PID実装（Tustin不完全微分、条件付き積分アンチワインドアップ、D-on-M）
        firmware/vehicle/components/sf_controller_pid/include/pid.hpp:71-143
    rate.yaw ゲイン既定値（現行 / 旧値）
        firmware/vehicle/components/sf_core/params.cpp:539-541 (kp=1.901691e-3, ti=0.8, td=0.01)
        firmware/vehicle/components/sf_core/params.cpp:157 comment ("was legacy 5.31e-3")
        firmware/vehicle_old/main/config.hpp:426 (YAW_RATE_KP = 5.31e-3)
    ヘディングホールド式（P制御、レートクランプ、wrap）
        firmware/vehicle/components/sf_controller_pid/pid_controller.cpp:296-322
        firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp:312-315
        （既定値 yaw_hold_kp_=3.0 [1/s], yaw_hold_rate_max_=2.0 [rad/s]。
          このセッションのログにこれらのパラメータの上書きは見つかっていない
          — 未確認。params.cpp に attitude.yawhold.kp/.rate_max のデフォルト値の
          明示的な行が見当たらないため、pid_controller.hpp のメンバ初期化値を採用）
    レート内ループのフィードバック = state.angular_rate（ESKFバイアス補正済み生ジャイロ、
    追加フィルタなし）
        firmware/vehicle/components/sf_controller_pid/pid_controller.cpp:660-671
        firmware/vehicle/components/sf_estimator_eskf/eskf_estimator.cpp:296-297
        → ログの imu.gyro − imu.gyro_bias と等価（yawlib/torque_budget と同じ量）
    rate_yaw_.output_limit = max_yaw_torque_（ソフトウェア・ヨートルクキャップ、NOMINAL空間）
        firmware/vehicle/components/sf_controller_pid/pid_controller.hpp:410
        firmware/vehicle/components/sf_controller_pid/pid_controller.cpp:134
    hover.thrust_corr（NOMINAL→REAL 物理トルクへの換算係数）
        firmware/vehicle/components/sf_core/params.cpp:608, torque_budget.py 参照
    I_z, KAPPA, モータ曲線, ミキサー配分 — torque_budget.py の出典コメントをそのまま流用
    （このスクリプトは torque_budget.analyze_event() を直接呼び出して τ_d / τ_nominal /
     物理包絡（duty飽和）を再利用する — 数値の二重実装を避けるため）
"""
import sys
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
sys.path.insert(0, SCRATCH)
import yawlib
import torque_budget as tb

R2D = 180.0 / np.pi
D2R = np.pi / 180.0

# =============================================================================
# Constants (see module docstring for file:line provenance)
# =============================================================================
IZZ = tb.IZZ                                   # [kg*m^2]
HOVER_THRUST_CORR = tb.HOVER_THRUST_CORR        # NOMINAL -> REAL torque factor
MAX_YAW_TORQUE_NOMINAL_S0 = tb.MAX_YAW_TORQUE   # 2.2e-3 Nm (current software cap, NOMINAL)

RATE_YAW_KP = 1.901691e-3   # params.cpp:539
RATE_YAW_TI = 0.8           # params.cpp:540
RATE_YAW_TD = 0.01          # params.cpp:541
RATE_YAW_KP_OLD = 5.31e-3   # params.cpp:157 comment / vehicle_old/main/config.hpp:426
PID_ETA = 0.125             # pid.hpp:54 default; no rate.yaw.eta param exists (checked params.cpp)

YAW_HOLD_KP = 3.0           # pid_controller.hpp:314 default (attitude.yawhold.kp not seen
                             # overridden in these logs' param dumps -- UNVERIFIED against
                             # the actual flashed NVS values; flagged in caveats)
YAW_HOLD_RATE_MAX = 2.0     # pid_controller.hpp:315 default

DT_SIM = 0.0025             # [s] 400 Hz control loop period (imu/rate_ref log interval confirms this)

RECOVERY_THRESH_DEG = 10.0  # recovery = |psi - psi_tgt| drops back below this


# =============================================================================
# PID (verbatim port of sf_controller_pid/include/pid.hpp compute())
# =============================================================================
class PID:
    def __init__(self, kp, ti, td, eta=PID_ETA, output_limit=1.0):
        self.kp, self.ti, self.td, self.eta = kp, ti, td, eta
        self.output_limit = output_limit
        self.integral = 0.0
        self.deriv_filter = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.first_run = True

    def compute(self, setpoint, measurement, dt):
        if dt <= 0:
            return 0.0
        error = setpoint - measurement

        p_term = self.kp * error

        d_term = 0.0
        if self.td > 0:
            if self.first_run:
                self.prev_measurement = measurement
            else:
                alpha = 2.0 * self.eta * self.td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * self.td / ((alpha + 1.0) * dt)
                self.deriv_filter = a * self.deriv_filter - b * (measurement - self.prev_measurement)
                d_term = self.kp * self.deriv_filter
        self.prev_measurement = measurement
        self.first_run = False

        if self.ti >= 0.01:
            i_next = self.integral + (self.kp / self.ti) * (error + self.prev_error) * (dt * 0.5)
            out_test = p_term + i_next + d_term
            push_high = (out_test > self.output_limit) and (error > 0)
            push_low = (out_test < -self.output_limit) and (error < 0)
            if not push_high and not push_low:
                self.integral = i_next
            if self.integral > self.output_limit:
                self.integral = self.output_limit
            if self.integral < -self.output_limit:
                self.integral = -self.output_limit

        self.prev_error = error

        output = p_term + self.integral + d_term
        if output > self.output_limit:
            output = self.output_limit
        if output < -self.output_limit:
            output = -self.output_limit
        return output

    def snapshot(self):
        return dict(integral=self.integral, deriv_filter=self.deriv_filter,
                    prev_error=self.prev_error, prev_measurement=self.prev_measurement,
                    first_run=self.first_run)

    def restore(self, snap, reset_integral=False):
        if reset_integral:
            self.integral = 0.0
            self.deriv_filter = 0.0
            self.prev_error = 0.0
            self.prev_measurement = snap["prev_measurement"]
            self.first_run = False
        else:
            self.integral = snap["integral"]
            self.deriv_filter = snap["deriv_filter"]
            self.prev_error = snap["prev_error"]
            self.prev_measurement = snap["prev_measurement"]
            self.first_run = snap["first_run"]


def heading_hold_cmd(psi_tgt, psi, kp, rate_max):
    """pid_controller.cpp:312-318 の写経（wrap を modulo で実装。while ループと数学的に等価）。"""
    err = psi_tgt - psi
    err = (err + np.pi) % (2.0 * np.pi) - np.pi
    cmd = kp * err
    if cmd > rate_max:
        cmd = rate_max
    if cmd < -rate_max:
        cmd = -rate_max
    return cmd


# =============================================================================
# Gap-aware helpers
# =============================================================================
def find_safe_preroll_start(ts, t0, preroll_s=10.0, max_gap_s=3.0):
    """t0 から遡って利用可能な実データの開始点を返す。t0 直前に max_gap_s を
    超える欠測があれば、そこで打ち切る（無関係な別フライト区間を跨がないため）。"""
    candidate = t0 - preroll_s
    data_min = float(ts.min())
    start = max(candidate, data_min)
    m = (ts >= start) & (ts <= t0)
    seg = ts[m]
    if len(seg) < 2:
        return max(start, t0)  # no usable preroll
    gaps = np.diff(seg)
    big = np.where(gaps > max_gap_s)[0]
    if len(big) > 0:
        # keep only the data AFTER the last big gap
        start = seg[big[-1] + 1]
    return start


def interp_with_gap_mask(t_query, t_src, v_src, gap_thresh_s):
    """t_src(非等間隔, 実測)上のv_srcをt_query(等間隔)へ線形補間。
    t_queryの各点が、隣接する実測サンプル間隔がgap_thresh_sを超える区間に
    落ちる場合は「補間区間」としてマスクする。"""
    v_q = np.interp(t_query, t_src, v_src)
    # For each query point, find bracketing source-sample gap
    idx = np.searchsorted(t_src, t_query, side="left")
    idx = np.clip(idx, 1, len(t_src) - 1)
    left_t = t_src[idx - 1]
    right_t = t_src[idx]
    local_gap = right_t - left_t
    interpolated_mask = local_gap > gap_thresh_s
    return v_q, interpolated_mask


# =============================================================================
# Step 1+2: open-loop PID model validation
# =============================================================================
def openloop_validate(ev, preroll_s=10.0):
    data = tb.load_log(ev["file"])
    imu = data["imu"]
    rr = data["rate_ref"]
    t0, t1 = ev["t0"], ev["t1"]

    ts_imu = imu["ts"]
    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]      # rad/s, matches state.angular_rate[2]
    ts_rr = rr["ts"]
    rr_full = rr["rate_ref"][:, 2]                            # rad/s, onboard rate_sp_yaw (incl. heading hold)

    grid_start = find_safe_preroll_start(ts_imu, t0, preroll_s=preroll_s)
    grid_end = t1
    n = int(round((grid_end - grid_start) / DT_SIM)) + 1
    t_grid = grid_start + np.arange(n) * DT_SIM

    r_grid, r_gap_mask = interp_with_gap_mask(t_grid, ts_imu, r_full, gap_thresh_s=0.02)
    rr_grid, rr_gap_mask = interp_with_gap_mask(t_grid, ts_rr, rr_full, gap_thresh_s=0.02)

    pid = PID(RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, PID_ETA,
              output_limit=MAX_YAW_TORQUE_NOMINAL_S0)
    tau_model_nominal = np.zeros(n)
    seed_state_at_t0 = None
    seed_idx = int(round((t0 - grid_start) / DT_SIM))
    seed_idx = min(max(seed_idx, 0), n - 1)

    for i in range(n):
        dt = DT_SIM if i > 0 else DT_SIM  # first call just primes state (pid.hpp semantics)
        tau_model_nominal[i] = pid.compute(rr_grid[i], r_grid[i], dt)
        if i == seed_idx:
            seed_state_at_t0 = pid.snapshot()

    # --- score against torque_budget's duty-reconstructed tau_nominal (independent source) ---
    res = tb.analyze_event(ev)
    tau_model_at_cr = np.interp(res["t"], t_grid, tau_model_nominal)
    valid = (res["t"] >= t0) & (res["t"] <= t1) & ~np.isnan(res["tau_nominal"])
    err = tau_model_at_cr[valid] - res["tau_nominal"][valid]
    rms = float(np.sqrt(np.mean(err ** 2)))
    if np.std(tau_model_at_cr[valid]) > 0 and np.std(res["tau_nominal"][valid]) > 0:
        corr = float(np.corrcoef(tau_model_at_cr[valid], res["tau_nominal"][valid])[0, 1])
    else:
        corr = float("nan")
    ref_rms = float(np.sqrt(np.mean(res["tau_nominal"][valid] ** 2)))

    gap_frac_r = float(np.mean(r_gap_mask[(t_grid >= t0) & (t_grid <= t1)]))
    gap_frac_rr = float(np.mean(rr_gap_mask[(t_grid >= t0) & (t_grid <= t1)]))

    return dict(
        name=ev["name"], grid_start=float(grid_start), t0=t0, t1=t1,
        preroll_actual_s=float(t0 - grid_start),
        rms_error_Nm=rms, ref_rms_Nm=ref_rms,
        rms_error_frac_of_ref=(rms / ref_rms if ref_rms > 0 else float("nan")),
        correlation=corr, n_scored=int(np.sum(valid)),
        gap_fraction_r=gap_frac_r, gap_fraction_rate_ref=gap_frac_rr,
        seed_state_at_t0=seed_state_at_t0,
        t_grid=t_grid, tau_model_nominal=tau_model_nominal,
        r_grid=r_grid, rr_grid=rr_grid,
    )


# =============================================================================
# Quiet-window static-trim bias (for S2 "hardware repair")
# =============================================================================
_quiet_bias_cache = {}


def get_quiet_bias(fname):
    """その飛行ファイルの静穏区間 tau_static_Nm の平均（=定常τ_dバイアスの符号反転値の元）。
    見つからなければ全飛行横断の平均にフォールバックする（E0のファイルはこのケース）。"""
    if fname in _quiet_bias_cache:
        return _quiet_bias_cache[fname]
    res = tb.static_asymmetry(fname)
    if len(res) == 0:
        all_vals = []
        for f in tb.QUIET_FILES:
            for r in tb.static_asymmetry(f):
                all_vals.append(r["tau_static_Nm"])
        bias = float(np.mean(all_vals))
        is_fallback = True
    else:
        bias = float(np.mean([r["tau_static_Nm"] for r in res]))
        is_fallback = False
    _quiet_bias_cache[fname] = (bias, is_fallback)
    return bias, is_fallback


# =============================================================================
# Closed-loop scenario simulation
# =============================================================================
def simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                       kp, ti, td, cap_nominal, envelope_real,
                       hh_kp, hh_rate_max, reset_integral=False):
    n = len(t_sim)
    pid = PID(kp, ti, td, PID_ETA, output_limit=cap_nominal)
    pid.restore(seed_state, reset_integral=reset_integral)

    psi = np.zeros(n)
    r = np.zeros(n)
    tau_applied_hist = np.zeros(n)
    psi[0] = psi0
    r[0] = r0
    psi_tgt = psi0

    for i in range(1, n):
        dt = t_sim[i] - t_sim[i - 1]
        rate_sp = heading_hold_cmd(psi_tgt, psi[i - 1], hh_kp, hh_rate_max)
        tau_pid_nominal = pid.compute(rate_sp, r[i - 1], dt)
        tau_pid_real = tau_pid_nominal / HOVER_THRUST_CORR
        # explicit cap clip (redundant with the PID's own output_limit, kept for
        # literal fidelity to the task's stated formula; see caveats)
        cap_real = cap_nominal / HOVER_THRUST_CORR
        tau_pid_real = np.clip(tau_pid_real, -cap_real, cap_real)
        tau_applied = np.clip(tau_pid_real, -envelope_real, envelope_real)
        tau_applied_hist[i] = tau_applied

        drdt = (tau_applied + tau_d_sim[i - 1]) / IZZ
        r[i] = r[i - 1] + drdt * dt
        psi[i] = psi[i - 1] + 0.5 * (r[i - 1] + r[i]) * dt

    dev_deg = (psi - psi_tgt) * R2D
    max_dev_idx = int(np.argmax(np.abs(dev_deg)))
    max_dev_deg = float(np.abs(dev_deg[max_dev_idx]))
    peak_t = float(t_sim[max_dev_idx])

    recovery_time_s = None
    for i in range(max_dev_idx, n):
        if abs(dev_deg[i]) < RECOVERY_THRESH_DEG:
            recovery_time_s = float(t_sim[i] - t_sim[0])
            break

    return dict(psi=psi, r=r, dev_deg=dev_deg, tau_applied=tau_applied_hist,
                max_dev_deg=max_dev_deg, peak_t_rel_s=peak_t - t_sim[0],
                recovery_time_s=recovery_time_s)


# =============================================================================
# Main per-event pipeline
# =============================================================================
def run_event(ev, ol):
    data = tb.load_log(ev["file"])
    imu = data["imu"]
    t0, t1 = ev["t0"], ev["t1"]
    ts_imu = imu["ts"]
    data_min = float(ts_imu.min())
    eff_t0 = max(t0, data_min)
    t0_clip_note = None
    if eff_t0 > t0 + 1e-6:
        t0_clip_note = f"log data starts at {data_min:.3f}s, {eff_t0 - t0:.3f}s after nominal t0={t0}s"

    res = tb.analyze_event(ev)   # tau_nominal/tau_d/envelope at 50Hz t_cr, window [t0,t1] fixed

    # --- envelope: median half-width, real Nm, treated as a constant per event ---
    half_width = (res["ur_max_possible"] - res["ur_min_possible"]) / 2.0
    envelope_real = float(np.nanmedian(half_width))

    # --- tau_d: fill NaN gaps by linear interpolation over t_cr, record fraction ---
    t_cr = res["t"]
    tau_d_raw = res["tau_d"]
    nan_mask = np.isnan(tau_d_raw)
    interpolated_fraction = float(np.mean(nan_mask))
    if np.any(~nan_mask):
        tau_d_filled = np.interp(t_cr, t_cr[~nan_mask], tau_d_raw[~nan_mask])
    else:
        tau_d_filled = np.nan_to_num(tau_d_raw)

    # --- simulation grid: fixed 2.5ms, from eff_t0 to t1 ---
    n = int(round((t1 - eff_t0) / DT_SIM)) + 1
    t_sim = eff_t0 + np.arange(n) * DT_SIM
    tau_d_sim = np.interp(t_sim, t_cr, tau_d_filled)

    # --- initial conditions from real telemetry at eff_t0 ---
    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
    r0 = float(np.interp(eff_t0, ts_imu, r_full))
    quat = imu["quat"]
    psi_full = yawlib.quat_to_yaw(quat)
    psi_unwrapped = np.unwrap(psi_full)  # rad, native imu ts order (gaps not re-ordered)
    psi0 = float(np.interp(eff_t0, ts_imu, psi_unwrapped))

    # real psi(t) over the window, for overlay plots
    m_plot = (ts_imu >= eff_t0) & (ts_imu <= t1)
    t_real_plot = ts_imu[m_plot]
    psi_real_plot_deg = np.degrees(psi_unwrapped[m_plot] - psi0)

    # --- seed PID state: replay the open-loop model from its grid start up to eff_t0
    # (handles eff_t0 != t0, e.g. E0 where the log starts partway into the window) ---
    seed_state = _seed_state_from_replay(ol, eff_t0)

    # --- static-trim bias for S2/S3 ---
    bias, bias_is_fallback = get_quiet_bias(ev["file"])
    tau_d_repaired = tau_d_sim + bias   # see docstring derivation: repaired = measured + mean(tau_static)

    cap_S0 = MAX_YAW_TORQUE_NOMINAL_S0          # 2.2e-3 Nm nominal
    cap_S1 = 2.9e-3                              # Nm nominal, relaxed cap
    cap_S4 = 1.0                                 # effectively unbounded (cap removed)

    scenarios = {}
    common = dict(psi0=psi0, r0=r0, t_sim=t_sim, res=res)

    scenarios["S0"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S0, envelope_real,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
    scenarios["S1"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S1, envelope_real,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
    scenarios["S2"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_repaired, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S0, envelope_real,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
    scenarios["S3"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_repaired, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S1, envelope_real,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
    scenarios["S4"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S4, 3.2e-3,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX)
    scenarios["S5"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                                         RATE_YAW_KP_OLD, RATE_YAW_TI, RATE_YAW_TD, cap_S0, envelope_real,
                                         YAW_HOLD_KP, YAW_HOLD_RATE_MAX, reset_integral=True)
    scenarios["S6"] = simulate_scenario(ev, res, seed_state, psi0, r0, tau_d_sim, t_sim,
                                         RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, cap_S0, envelope_real,
                                         YAW_HOLD_KP, 4.0)

    # --- S0 vs real reproduction check ---
    sim_psi_deg = np.degrees(scenarios["S0"]["psi"] - psi0)
    sim_at_real_t = np.interp(t_real_plot, t_sim, sim_psi_deg)
    repro_err = sim_at_real_t - psi_real_plot_deg
    repro_rms = float(np.sqrt(np.mean(repro_err ** 2)))
    real_max_dev = float(np.max(np.abs(psi_real_plot_deg)))
    sim_max_dev = scenarios["S0"]["max_dev_deg"]

    return dict(
        name=ev["name"], file=ev["file"], t0=t0, t1=t1, eff_t0=eff_t0,
        t0_clip_note=t0_clip_note,
        envelope_real_mNm=envelope_real * 1e3,
        tau_d_interpolated_fraction=interpolated_fraction,
        quiet_bias_mNm=bias * 1e3, quiet_bias_is_fallback=bias_is_fallback,
        real_max_dev_deg=real_max_dev, s0_sim_max_dev_deg=sim_max_dev,
        repro_rms_deg=repro_rms,
        t_sim=t_sim, t_real_plot=t_real_plot, psi_real_plot_deg=psi_real_plot_deg,
        scenarios=scenarios,
    )


def _seed_state_from_replay(ol, t_query):
    """openloop_validate() の t_grid 上を再走査し、t_query 直前までの状態を厳密に再現する
    （t_query が ol['t0'] と異なる場合 — E0 のように eff_t0 != t0 — に対応するため、
    毎回 PID を先頭から t_query まで走らせ直す。preroll が短い場合でも安全）。"""
    pid = PID(RATE_YAW_KP, RATE_YAW_TI, RATE_YAW_TD, PID_ETA,
              output_limit=MAX_YAW_TORQUE_NOMINAL_S0)
    t_grid = ol["t_grid"]
    rr_grid = ol["rr_grid"]
    r_grid = ol["r_grid"]
    idx = int(round((t_query - t_grid[0]) / DT_SIM))
    idx = min(max(idx, 0), len(t_grid) - 1)
    for i in range(idx + 1):
        pid.compute(rr_grid[i], r_grid[i], DT_SIM)
    return pid.snapshot()


# =============================================================================
# Plot
# =============================================================================
def plot_event(ev_result):
    name = ev_result["name"]
    t_sim = ev_result["t_sim"]
    t_rel = t_sim - ev_result["eff_t0"]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(ev_result["t_real_plot"] - ev_result["eff_t0"], ev_result["psi_real_plot_deg"],
            color="black", lw=1.6, label="real psi (quat, unwrapped)")
    colors = dict(S0="tab:orange", S1="tab:blue", S2="tab:green", S3="tab:purple",
                  S4="tab:red", S5="tab:brown", S6="tab:cyan")
    for sname in ["S0", "S1", "S2", "S4", "S5"]:
        sc = ev_result["scenarios"][sname]
        ax.plot(t_rel, sc["dev_deg"], color=colors[sname], lw=1.1, alpha=0.9, label=sname)
    ax.axhline(0, color="gray", lw=0.6)
    ax.axhline(RECOVERY_THRESH_DEG, color="gray", lw=0.5, ls=":")
    ax.axhline(-RECOVERY_THRESH_DEG, color="gray", lw=0.5, ls=":")
    ax.set_xlabel(f"t - {ev_result['eff_t0']:.2f} [s]")
    ax.set_ylabel("psi - psi_tgt [deg]")
    ax.set_title(f"{name}  {ev_result['file']}  window[{ev_result['t0']},{ev_result['t1']}]s\n"
                 f"real max|dev|={ev_result['real_max_dev_deg']:.1f}deg  "
                 f"S0 sim max|dev|={ev_result['s0_sim_max_dev_deg']:.1f}deg")
    ax.legend(fontsize=8, ncol=3, loc="best")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    outpath = f"{SCRATCH}/cm_sim_{name}.png"
    fig.savefig(outpath, dpi=140)
    plt.close(fig)
    print(f"  saved {outpath}")


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 78)
    print("yaw_cm_sim.py — closed-loop yaw simulation with measured tau_d(t)")
    print(f"RATE_YAW: kp={RATE_YAW_KP} ti={RATE_YAW_TI} td={RATE_YAW_TD} eta={PID_ETA}")
    print(f"YAW_HOLD: kp={YAW_HOLD_KP} rate_max={YAW_HOLD_RATE_MAX}")
    print(f"IZZ={IZZ} HOVER_THRUST_CORR={HOVER_THRUST_CORR} cap_S0_nominal={MAX_YAW_TORQUE_NOMINAL_S0}")

    ol_results = {}
    print("\n" + "=" * 78)
    print("Step 2: open-loop PID model validation")
    for ev in tb.EVENTS:
        ol = openloop_validate(ev)
        ol_results[ev["name"]] = ol
        print(f"\n--- {ev['name']} ---")
        print(f"  preroll actual = {ol['preroll_actual_s']:.2f}s (requested 10s)")
        print(f"  n_scored={ol['n_scored']}  RMS error={ol['rms_error_Nm']*1e3:.4f} mNm "
              f"({ol['rms_error_frac_of_ref']*100:.1f}% of ref RMS {ol['ref_rms_Nm']*1e3:.4f} mNm)")
        print(f"  correlation(model, duty-reconstructed) = {ol['correlation']:.4f}")
        print(f"  gap-interpolated fraction: r={ol['gap_fraction_r']*100:.1f}%  "
              f"rate_ref={ol['gap_fraction_rate_ref']*100:.1f}%")

    print("\n" + "=" * 78)
    print("Step 3-5: closed-loop scenario simulation")
    all_results = {}
    for ev in tb.EVENTS:
        print(f"\n--- {ev['name']} ---")
        er = run_event(ev, ol_results[ev["name"]])
        all_results[ev["name"]] = er
        if er["t0_clip_note"]:
            print(f"  NOTE: {er['t0_clip_note']}")
        print(f"  envelope_real (median half-width) = {er['envelope_real_mNm']:.3f} mNm")
        print(f"  tau_d NaN-interpolated fraction = {er['tau_d_interpolated_fraction']*100:.1f}%")
        print(f"  quiet-window bias (S2) = {er['quiet_bias_mNm']:.3f} mNm "
              f"{'(cross-flight fallback, no quiet window in this file)' if er['quiet_bias_is_fallback'] else '(same-file)'}")
        print(f"  real max|dev| = {er['real_max_dev_deg']:.1f} deg   "
              f"S0 sim max|dev| = {er['s0_sim_max_dev_deg']:.1f} deg   "
              f"S0 vs real RMS = {er['repro_rms_deg']:.1f} deg")
        for sname, sc in er["scenarios"].items():
            rt = sc["recovery_time_s"]
            rt_str = f"{rt:.2f}s" if rt is not None else "NOT RECOVERED in window"
            print(f"  {sname}: max|dev|={sc['max_dev_deg']:6.1f} deg  recovery={rt_str}")
        plot_event(er)

    # --- scenario table (text) ---
    print("\n" + "=" * 78)
    print("Scenario table (max|psi-psi_tgt| [deg] / recovery [s])")
    header = f"{'event':6s}" + "".join(f"{s:>18s}" for s in ["S0", "S1", "S2", "S3", "S4", "S5", "S6"])
    print(header)
    for ev in tb.EVENTS:
        er = all_results[ev["name"]]
        row = f"{ev['name']:6s}"
        for sname in ["S0", "S1", "S2", "S3", "S4", "S5", "S6"]:
            sc = er["scenarios"][sname]
            rt = sc["recovery_time_s"]
            rt_s = f"{rt:.1f}s" if rt is not None else "N/R"
            row += f"{sc['max_dev_deg']:7.1f}/{rt_s:>9s}"
        print(row)

    # --- save JSON ---
    def strip_arrays(d):
        out = {}
        for k, v in d.items():
            if isinstance(v, np.ndarray):
                continue
            elif isinstance(v, dict):
                out[k] = strip_arrays(v)
            elif isinstance(v, (np.floating,)):
                out[k] = float(v)
            elif isinstance(v, (np.integer,)):
                out[k] = int(v)
            else:
                out[k] = v
        return out

    json_out = dict(
        params=dict(RATE_YAW_KP=RATE_YAW_KP, RATE_YAW_TI=RATE_YAW_TI, RATE_YAW_TD=RATE_YAW_TD,
                    RATE_YAW_KP_OLD=RATE_YAW_KP_OLD, PID_ETA=PID_ETA,
                    YAW_HOLD_KP=YAW_HOLD_KP, YAW_HOLD_RATE_MAX=YAW_HOLD_RATE_MAX,
                    IZZ=IZZ, HOVER_THRUST_CORR=HOVER_THRUST_CORR,
                    MAX_YAW_TORQUE_NOMINAL_S0=MAX_YAW_TORQUE_NOMINAL_S0,
                    cap_S1_nominal=2.9e-3, S4_envelope_real_Nm=3.2e-3, DT_SIM=DT_SIM),
        openloop_validation={k: strip_arrays(v) for k, v in ol_results.items()},
        events={k: strip_arrays(v) for k, v in all_results.items()},
    )
    with open(f"{SCRATCH}/cm_sim_results.json", "w") as f:
        json.dump(json_out, f, indent=2, default=str)
    print(f"\nSaved -> {SCRATCH}/cm_sim_results.json")


if __name__ == "__main__":
    main()
