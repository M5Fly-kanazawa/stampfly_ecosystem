"""
legacy_yaw_replay.py
=====================
旧 vehicle_old CSV ログのヨートルク指令(u_yaw)を PID 再生で復元し、静穏ホバー中の
定常ヨートリムの符号・大きさを全ファイルで集計する。

【手順】
  Step 0: 手法検証（必須） — 6月の統一JSONLログ3本に対し、dutyを使わず
          rate_ref[2] と gyro[2]-gyro_bias[2] から現行vehicleのヨーレートPID
          (条件付きAW, yaw_cm_sim.PID) で u_yaw(t) を再生し、静穏窓平均を
          duty由来のtau_static(torque_budget.static_asymmetry, 既知値
          +0.75/+0.63/+0.76mNm)と比較。3本とも符号一致・大きさ±50%以内なら合格。
  Step 1: 合格した場合のみ、考古学調査で確定した列意味・制御則・ゲインに従い、
          旧CSVで u_yaw(t) を再生（vehicle_old の PID = 台形積分+バックカルキュレー
          ションAW+D-on-M、pid.cpp の写経。conditional-AWの現行PIDとは別実装）。

出典（考古学調査JSON、ユーザー提供）:
  - ctrl_throttle/roll/pitch/yaw = スティック生値（正規化ADC）:
        firmware/vehicle_old/components/sf_svc_state/stampfly_state.cpp:432-443
  - gyro_corrected_z = 制御ループが実際に使うバイアス補正済みジャイロ:
        firmware/vehicle_old/main/tasks/telemetry_task.cpp:194-196
        firmware/vehicle_old/main/tasks/control_task.cpp:676,731-733,774-779
  - ヨー = 全モード共通で直接レート制御（ヘディング保持なし）:
        control_task.cpp:200-202(cascade), 659-664(ACRO)
        rate_ref = clamp(ctrl_yaw + g_trim_yaw, -1, 1) * YAW_RATE_MAX(5.0 rad/s)
        g_trim_yaw はランタイム値でログから復元不可、既定 0.0 と仮定（cmd_control.cpp:25）
        → trim=0 の仮定下では ACRO/cascade どちらでも rate_ref = ctrl_yaw*5.0 rad/s に一致
  - レートPID: 不完全微分(D-on-M)+台形積分+バックカルキュレーションAW(Tt=Ti):
        firmware/vehicle_old/components/sf_algo_pid/pid.cpp:48-153
        Kp=1.77e-3 Nm/(rad/s), Ti=0.8s, Td=0.01s, eta=0.125, output_limit=±2.2e-3Nm
        （2026-01-14T23:20〜2026-04-05T15:02 の安定期間。対象CSV全てこの期間内）
  - ミキサー符号規約（motor_driver.cpp:156-159）は vehicle_new の actuator.cpp:226-229
    と同一パターン（M1=FR,M3=RLに+yaw、M2=RR,M4=FLに-yaw）→ u_yaw>0 は vehicle_new の
    tau_realized>0 と同じ「CCW群(FR+RL)優位」を意味する。符号規約は共通。
"""
import sys
import json
import numpy as np
import pandas as pd

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
SCRIPTS_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa"
LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"
sys.path.insert(0, SCRATCH)
sys.path.insert(0, SCRIPTS_DIR)
import yawlib               # noqa: E402
import torque_budget as tb  # noqa: E402  (validated duty->tau model, quiet-window detector)
import yaw_cm_sim as cm     # noqa: E402  (validated current-vehicle PID, gap-mask interp helpers)

R2D = 180.0 / np.pi
D2R = np.pi / 180.0

# =============================================================================
# Step 0: method validation against 6-gatsu JSONL (duty-free replay vs duty-based tau_static)
# =============================================================================
VALIDATION_FILES = [
    "stampfly_udp_20260622T161016.jsonl",
    "stampfly_udp_20260627T164611.jsonl",
    "stampfly_udp_20260627T165713.jsonl",
]

# current-vehicle rate.yaw gains (task-specified for validation only)
VAL_KP = 1.901691e-3
VAL_TI = 0.8
VAL_TD = 0.01
VAL_ETA = cm.PID_ETA
VAL_OUTPUT_LIMIT_NOMINAL = 2.2e-3  # Nm, nominal space (pid_controller.hpp:410)


def validate_method():
    """rate_ref[2]/gyro[2]-gyro_bias[2] だけを使い、現行vehicleの条件付きAW PIDで
    u_yaw(t)を全ファイル通しで再生。tb.find_quiet_windows() と同じ静穏窓内平均を
    tau_static(duty由来、実機空間)と比較する。PID出力はnominal空間なので
    HOVER_THRUST_CORRで実機空間に変換してから比較する（torque_budget.pyの規約通り）。
    """
    rows = []
    for fname in VALIDATION_FILES:
        data = tb.load_log(fname)
        imu = data["imu"]
        rr = data["rate_ref"]
        ts_imu = imu["ts"]
        r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
        ts_rr = rr["ts"]
        rr_full = rr["rate_ref"][:, 2]

        # uniform 400Hz grid over the whole file (DT_SIM = cm.DT_SIM = 0.0025s)
        t0 = max(ts_imu.min(), ts_rr.min())
        t1 = min(ts_imu.max(), ts_rr.max())
        n = int(round((t1 - t0) / cm.DT_SIM)) + 1
        t_grid = t0 + np.arange(n) * cm.DT_SIM

        r_grid, r_gap = cm.interp_with_gap_mask(t_grid, ts_imu, r_full, gap_thresh_s=0.02)
        rr_grid, rr_gap = cm.interp_with_gap_mask(t_grid, ts_rr, rr_full, gap_thresh_s=0.02)

        pid = cm.PID(VAL_KP, VAL_TI, VAL_TD, VAL_ETA, output_limit=VAL_OUTPUT_LIMIT_NOMINAL)
        u_nominal = np.zeros(n)
        for i in range(n):
            u_nominal[i] = pid.compute(rr_grid[i], r_grid[i], cm.DT_SIM)
        u_real = u_nominal / tb.HOVER_THRUST_CORR  # nominal -> real, same convention as torque_budget.py

        windows = tb.find_quiet_windows(fname)
        static = tb.static_asymmetry(fname)
        assert len(windows) == len(static)

        window_means_replay = []
        window_means_duty = []
        for (s, e, wt0, wt1), sres in zip(windows, static):
            m = (t_grid >= wt0) & (t_grid <= wt1)
            u_win = u_real[m]
            mean_replay_mNm = float(np.mean(u_win)) * 1e3
            mean_duty_mNm = sres["tau_static_Nm"] * 1e3
            window_means_replay.append(mean_replay_mNm)
            window_means_duty.append(mean_duty_mNm)
            rows.append(dict(file=fname, t0=float(wt0), t1=float(wt1),
                              replay_mNm=mean_replay_mNm, duty_tau_static_mNm=mean_duty_mNm))

        file_replay_mean = float(np.mean(window_means_replay)) if window_means_replay else float("nan")
        file_duty_mean = float(np.mean(window_means_duty)) if window_means_duty else float("nan")
        sign_match = (np.sign(file_replay_mean) == np.sign(file_duty_mean)) if window_means_replay else False
        if file_duty_mean != 0 and window_means_replay:
            ratio = file_replay_mean / file_duty_mean
        else:
            ratio = float("nan")
        rows.append(dict(file=fname, summary=True,
                          n_windows=len(windows),
                          file_replay_mean_mNm=file_replay_mean,
                          file_duty_mean_mNm=file_duty_mean,
                          sign_match=bool(sign_match),
                          ratio=ratio,
                          within_50pct=bool(ratio == ratio and 0.5 <= ratio <= 1.5)))
    return rows


# =============================================================================
# Step 1: legacy vehicle_old PID replica (trapezoidal I + back-calculation AW + D-on-M)
#   pid.cpp:48-153 の写経（考古学調査による）
# =============================================================================
class LegacyPID:
    def __init__(self, kp, ti, td, eta, output_limit, tt=None):
        self.kp, self.ti, self.td, self.eta = kp, ti, td, eta
        self.output_limit = output_limit
        self.tt = tt if tt is not None else ti
        self.integral = 0.0          # dimension: same as error (rad/s); output = kp*integral
        self.deriv_filter = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.first_run = True

    def compute(self, setpoint, measurement, dt):
        if dt <= 0:
            return self.kp * self.integral + 0.0  # hold (should not normally hit)
        error = setpoint - measurement
        p_term = self.kp * error

        # trapezoidal integral (unconditional; corrected below via back-calculation)
        i_state_pre = self.integral + (dt / (2.0 * self.ti)) * (error + self.prev_error)
        i_term = self.kp * i_state_pre

        d_term = 0.0
        if self.td > 0:
            if self.first_run:
                self.prev_measurement = measurement
            else:
                alpha = 2.0 * self.eta * self.td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * self.td / ((alpha + 1.0) * dt)
                # x = -measurement (D-on-M): b*(x[k]-x[k-1]) = -b*(measurement-prev_measurement)
                self.deriv_filter = a * self.deriv_filter - b * (measurement - self.prev_measurement)
                d_term = self.kp * self.deriv_filter
        self.prev_measurement = measurement
        self.first_run = False

        unlimited = p_term + i_term + d_term
        limited = unlimited
        if limited > self.output_limit:
            limited = self.output_limit
        if limited < -self.output_limit:
            limited = -self.output_limit

        # back-calculation anti-windup: correct integral STATE (not i_term) so the
        # next cycle's i_state_pre reflects the actually-applied (clamped) output.
        self.integral = i_state_pre + (limited - unlimited) * (dt / self.tt) / self.kp

        self.prev_error = error
        return limited


LEGACY_YAW_KP = 1.77e-3     # Nm/(rad/s)   config.hpp (stable era 2026-01-14T23:20..2026-04-05T15:02)
LEGACY_YAW_TI = 0.8         # s
LEGACY_YAW_TD = 0.01        # s
LEGACY_YAW_ETA = 0.125      # PID_ETA (roll/pitch/yaw common)
LEGACY_YAW_LIMIT = 2.2e-3   # Nm  (YAW_OUTPUT_LIMIT)
LEGACY_YAW_RATE_MAX = 5.0   # rad/s

DT_CAP_S = 0.5              # sanity cap on per-sample dt fed to the PID (documented caveat)

CSV_FILES_ALL = [
    "flight.csv",
    "stampfly_wifi_20260303T014044.csv",
    "stampfly_wifi_20260303T025543.csv",
    "stampfly_wifi_20260303T134820.csv",
    "hover_test_static.csv",
    "hover_test_static02.csv",
    "hover_test_static03.csv",
] + [f"log_wifi_test{('' if i == 1 else str(i).zfill(2))}.csv" for i in range(1, 51)] + [
    "hover01.csv", "hover02.csv", "hover03.csv", "hover04.csv",
    "hover06.csv", "hover07.csv", "hover08.csv", "hover09.csv",
    "hover10.csv", "hover11.csv", "hover12.csv", "hover13.csv",
    "stampfly_udp_20260401T155904.csv",
    "stampfly_udp_20260401T161253.csv",
    "stampfly_udp_20260401T162900.csv",
]


def load_csv(fname):
    path = f"{LOG_DIR}/{fname}"
    df = pd.read_csv(path)
    return df


def replay_legacy_file(df):
    """Full-file legacy PID replay. Reset at the first sample where the craft is
    judged 'flying' (tof_bottom>0.15m), as a proxy for ARM (unknown from CSV)."""
    n = len(df)
    ts_s = df["timestamp_us"].to_numpy(dtype=float) / 1e6
    dt_arr = np.diff(ts_s, prepend=ts_s[0] - 0.0025)
    dt_arr[0] = 0.0025
    dt_arr = np.clip(dt_arr, 0.0, DT_CAP_S)

    gyro_z = df["gyro_corrected_z"].to_numpy(dtype=float)
    ctrl_yaw = df["ctrl_yaw"].to_numpy(dtype=float)
    rate_ref = ctrl_yaw * LEGACY_YAW_RATE_MAX  # trim=0 assumption (see module docstring)

    tof = df["tof_bottom"].to_numpy(dtype=float) if "tof_bottom" in df.columns else np.full(n, np.nan)
    flying = tof > 0.15
    flying_idx = np.where(flying)[0]

    u_yaw = np.full(n, np.nan)
    if len(flying_idx) == 0:
        return u_yaw, flying, dt_arr, rate_ref

    start = int(flying_idx[0])
    pid = LegacyPID(LEGACY_YAW_KP, LEGACY_YAW_TI, LEGACY_YAW_TD, LEGACY_YAW_ETA, LEGACY_YAW_LIMIT)
    for i in range(start, n):
        dt = dt_arr[i] if dt_arr[i] > 0 else 0.0025
        u_yaw[i] = pid.compute(rate_ref[i], gyro_z[i], dt)
    return u_yaw, flying, dt_arr, rate_ref


def find_quiet_windows_legacy(df, flying, gyro_thresh_dps=20.0, stick_thresh=0.05, min_dur_s=5.0):
    """静穏窓の条件はタスク指示に厳密に従う: |gyro_corrected_z|<20deg/s,
    スティック中立は ctrl_yaw のみ(|ctrl_yaw|<0.05) — roll/pitch は問わない
    (6月JSONLのtb.find_quiet_windowsはroll/pitchも要求するが、旧CSVの実飛行は
    並進操作を伴い roll/pitch が常時非中立なため、タスク指示通りyawのみで判定する)。"""
    ts_s = df["timestamp_us"].to_numpy(dtype=float) / 1e6
    gyro_z = df["gyro_corrected_z"].to_numpy(dtype=float)
    ctrl_yaw = df["ctrl_yaw"].to_numpy(dtype=float)

    gyro_quiet = np.abs(gyro_z) * R2D < gyro_thresh_dps
    stick_neutral = np.abs(ctrl_yaw) < stick_thresh
    quiet = flying & gyro_quiet & stick_neutral

    idx = np.where(quiet)[0]
    windows = []
    if len(idx) == 0:
        return windows
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    for s, e in zip(starts, ends):
        dur = ts_s[e] - ts_s[s]
        if dur >= min_dur_s:
            windows.append((int(s), int(e), float(ts_s[s]), float(ts_s[e])))
    return windows


def analyze_legacy_file(fname):
    df = load_csv(fname)
    n = len(df)
    thr = df["ctrl_throttle"].to_numpy(dtype=float) if "ctrl_throttle" in df.columns else np.zeros(n)
    thr_max = float(np.nanmax(thr)) if n else 0.0

    if thr_max < 0.02:
        return dict(file=fname, excluded=True, reason="ground_static (throttle max < 0.02)",
                     n=n, thr_max=thr_max)

    u_yaw, flying, dt_arr, rate_ref = replay_legacy_file(df)
    if not np.any(flying):
        return dict(file=fname, excluded=True, reason="no tof_bottom>0.15m sample (no airborne detection)",
                     n=n, thr_max=thr_max)

    ctrl_yaw = df["ctrl_yaw"].to_numpy(dtype=float)
    nonneutral_frac = float(np.mean(np.abs(ctrl_yaw[flying]) >= 0.05)) if np.any(flying) else float("nan")

    def _windows_to_recs(windows):
        recs = []
        for s, e, wt0, wt1 in windows:
            seg = u_yaw[s:e + 1]
            seg = seg[~np.isnan(seg)]
            mean_mNm = float(np.mean(seg)) * 1e3 if len(seg) else float("nan")
            recs.append(dict(t0=wt0, t1=wt1, dur=wt1 - wt0, n=int(e - s + 1),
                              mean_u_yaw_mNm=mean_mNm))
        return recs

    # Primary: task spec, min_dur=5.0s
    windows5 = find_quiet_windows_legacy(df, flying, min_dur_s=5.0)
    win_recs5 = _windows_to_recs(windows5)
    file_mean5 = (float(np.mean([w["mean_u_yaw_mNm"] for w in win_recs5]))
                  if win_recs5 else float("nan"))

    # Diagnostic (non-primary): min_dur=2.0s -- WiFi frame loss (13-20%, per
    # archaeology) fragments most real-flight quiet stretches below 5s; this
    # relaxed pass recovers sign information for those files. Labeled explicitly
    # as diagnostic in the output; never substituted for the primary result.
    windows2 = find_quiet_windows_legacy(df, flying, min_dur_s=2.0)
    win_recs2 = _windows_to_recs(windows2)
    file_mean2 = (float(np.mean([w["mean_u_yaw_mNm"] for w in win_recs2]))
                  if win_recs2 else float("nan"))

    return dict(file=fname, excluded=False, n=n, thr_max=thr_max,
                n_windows=len(win_recs5), windows=win_recs5,
                file_mean_u_yaw_mNm=file_mean5,
                n_windows_diag2s=len(win_recs2), windows_diag2s=win_recs2,
                file_mean_u_yaw_mNm_diag2s=file_mean2,
                ctrl_yaw_nonneutral_frac=nonneutral_frac)


def main():
    print("=" * 70)
    print("Step 0: method validation (6-gatsu JSONL, duty-free replay)")
    print("=" * 70)
    val_rows = validate_method()
    summaries = [r for r in val_rows if r.get("summary")]
    for r in summaries:
        print(f"  {r['file']}: n_windows={r['n_windows']} "
              f"replay={r['file_replay_mean_mNm']:+.3f}mNm duty={r['file_duty_mean_mNm']:+.3f}mNm "
              f"sign_match={r['sign_match']} ratio={r['ratio']:.3f} within_50pct={r['within_50pct']}")

    all_pass = all(r["sign_match"] and r["within_50pct"] for r in summaries)
    print(f"\nVALIDATION {'PASS' if all_pass else 'FAIL'}")

    result = dict(validation_rows=val_rows, validation_pass=all_pass)

    if not all_pass:
        print("Validation failed -- aborting legacy CSV replay per task instructions.")
        with open(f"{SCRATCH}/legacy_yaw_replay.json", "w") as fp:
            json.dump(result, fp, indent=2, default=str)
        return

    print("\n" + "=" * 70)
    print("Step 1: legacy CSV replay")
    print("=" * 70)
    legacy_results = []
    for fname in CSV_FILES_ALL:
        try:
            res = analyze_legacy_file(fname)
        except Exception as e:
            res = dict(file=fname, excluded=True, reason=f"error: {e}")
        legacy_results.append(res)
        if res.get("excluded"):
            print(f"  {fname}: EXCLUDED ({res.get('reason')})")
        else:
            n5 = res.get("n_windows", 0)
            n2 = res.get("n_windows_diag2s", 0)
            msg = f"  {fname}: n_windows(5s)={n5}"
            if n5 > 0:
                msg += f" mean={res['file_mean_u_yaw_mNm']:+.3f}mNm"
            msg += f"  | diag(2s): n={n2}"
            if n2 > 0:
                msg += f" mean={res['file_mean_u_yaw_mNm_diag2s']:+.3f}mNm"
            msg += f"  nonneutral_frac={res['ctrl_yaw_nonneutral_frac']:.3f}"
            print(msg)

    result["legacy_results"] = legacy_results
    with open(f"{SCRATCH}/legacy_yaw_replay.json", "w") as fp:
        json.dump(result, fp, indent=2, default=str)
    print(f"\nSaved -> {SCRATCH}/legacy_yaw_replay.json")


if __name__ == "__main__":
    main()
