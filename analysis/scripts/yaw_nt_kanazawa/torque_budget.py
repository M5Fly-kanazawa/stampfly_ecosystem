"""
torque_budget.py
=================
StampFly 実飛行ログ（2026-06-27 NT金沢会場）のヨートルク収支再構成。

適用モデル出典（すべて file:line で追跡）:
    KAPPA, ARM_D, MOTOR_AM/BM/CM, MOTOR_CT
        firmware/vehicle/components/sf_actuator/actuator.cpp:88-93
    duty <-> thrust の相互変換（thrustToDuty の逆関数）
        firmware/vehicle/components/sf_actuator/actuator.cpp:184-190
    hover.thrust_corr（モータ曲線が実機推力を約12%過大に見積もる補正係数）
        firmware/vehicle/components/sf_core/params.cpp:608
        docs/architecture/stampfly-parameters.md:343,786
    I_z（ヨー慣性）
        simulator/sil/models/stampfly.xml:86 (diaginertia 第3成分 = Z軸)
        docs/architecture/stampfly-parameters.md:525 (Izz = 20.4e-6 kg·m^2)
    質量 m = 0.037 kg
        simulator/sil/plant/plant.hpp:140 (0.037), docs/architecture/stampfly-parameters.md:522
    ソフトウェア・ヨートルクリミット max_yaw_torque_ = 2.2e-3 Nm
        firmware/vehicle/components/sf_controller_pid/include/pid_controller.hpp:410
    ctrl_ref.motor_duty = 最終適用 duty（actuator_motor トピックのミラー、post-clamp）
        firmware/vehicle/components/sf_telemetry/data_stream.cpp:281
        firmware/vehicle/components/sf_telemetry/telemetry.cpp:197,244
    ミキサー配分（B^-1、CCW=M1(FR)/M3(RL), CW=M2(RR)/M4(FL)）
        firmware/vehicle/components/sf_actuator/actuator.cpp:214-231

ヨートルクの符号規約: mixerCompute (actuator.cpp:226-229) より
    T_FR = 0.25*(ut - up/d + uq/d + ur/kappa)
    T_RR = 0.25*(ut - up/d - uq/d - ur/kappa)
    T_RL = 0.25*(ut + up/d - uq/d + ur/kappa)
    T_FL = 0.25*(ut + up/d + uq/d - ur/kappa)
逆算すると ur = kappa*(T_FR - T_RR + T_RL - T_FL) = kappa*((T_FR+T_RL)-(T_RR+T_FL))
  = kappa*(CCW群推力和 - CW群推力和)   … 実現ヨートルク τ_yaw_realized の定義（+が右回り/CW正、FRD +z=下向き軸まわり）
ヘディングホールドは hold_cmd = yawhold.kp*wrap(target-psi) をレートコマンドとして加算し
（pid_controller.cpp:296-322）、外乱で機体が右に回されれば負のレート指令（左回り抵抗）を出す
= 符号は自己無矛盾（背景記述の「抵抗方向」と整合）。

外乱トルク: tau_d(t) = I_z * dr/dt - tau_yaw_realized(t)
  r = gyro[2] - gyro_bias[2] （imu, 400Hz, 生値）
  dr/dt は連続区間内（UDPギャップをまたがない）で Savitzky-Golay 平滑微分。
"""
import sys
import json
import numpy as np
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
sys.path.insert(0, SCRATCH)
import yawlib

LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"

R2D = 180.0 / np.pi
D2R = np.pi / 180.0

# =============================================================================
# Physical parameters (see module docstring for file:line provenance)
# =============================================================================
KAPPA = 0.00971          # actuator.cpp:89  torque/thrust ratio Cq/Ct [m]
ARM_D = 0.023             # actuator.cpp:88  moment arm [m] (unused directly for yaw)
MOTOR_AM = 5.39e-8        # actuator.cpp:90  V = Am*w^2 + Bm*w + Cm
MOTOR_BM = 6.33e-4        # actuator.cpp:91
MOTOR_CM = 1.53e-2        # actuator.cpp:92
MOTOR_CT = 1.00e-8        # actuator.cpp:93  T = Ct*w^2 [N]
HOVER_THRUST_CORR = 1.12  # params.cpp:608 default (onboard-learned value may differ; not logged)
MASS_KG = 0.037           # plant.hpp:140 / stampfly-parameters.md:522
IZZ = 20.4e-6             # stampfly.xml:86 / stampfly-parameters.md:525  [kg*m^2]
MAX_YAW_TORQUE = 2.2e-3   # pid_controller.hpp:410  [Nm]
G = 9.81


def duty_to_thrust(duty, vbat, corr=HOVER_THRUST_CORR):
    """Per-motor thrust [N] from applied duty [0,1] and supply voltage [V].
    Exact inverse of actuator.cpp:184-190 thrustToDuty, divided by the
    hover.thrust_corr flight-measured over-prediction factor (params.cpp:608)."""
    duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
    vbat = np.asarray(vbat, dtype=float)
    volts = duty * vbat
    disc = MOTOR_BM**2 - 4.0 * MOTOR_AM * (MOTOR_CM - volts)
    disc = np.maximum(disc, 0.0)
    omega = (-MOTOR_BM + np.sqrt(disc)) / (2.0 * MOTOR_AM)
    omega = np.maximum(omega, 0.0)
    thrust_model = MOTOR_CT * omega**2
    return thrust_model / corr


def max_thrust_at_duty1(vbat, corr=HOVER_THRUST_CORR):
    """Max achievable per-motor thrust [N] at duty=1 for given vbat (physical ceiling)."""
    return duty_to_thrust(np.ones_like(np.asarray(vbat, dtype=float)), vbat, corr)


def continuous_runs(ts, gap_thresh):
    """Indices (s,e inclusive) of runs where consecutive dt <= gap_thresh."""
    ts = np.asarray(ts)
    if len(ts) < 2:
        return []
    gaps = np.where(np.diff(ts) > gap_thresh)[0]
    starts = np.concatenate(([0], gaps + 1))
    ends = np.concatenate((gaps, [len(ts) - 1]))
    return list(zip(starts.tolist(), ends.tolist()))


def smoothed_derivative_runs(ts, r, gap_thresh=0.02, win_s=0.05, polyorder=3, min_pts=20):
    """
    Per continuous run (no gap > gap_thresh crossed): resample r(ts) onto a
    uniform grid at the run's native rate, Savitzky-Golay smooth + differentiate.
    Returns list of (t_uniform, r_smooth, drdt) tuples, one per usable run.
    UDPギャップ（最大0.5s）をまたがないよう、run 内でのみ微分する。
    """
    runs = continuous_runs(ts, gap_thresh)
    out = []
    for s, e in runs:
        if e - s < min_pts:
            continue
        t_seg = ts[s:e + 1]
        r_seg = r[s:e + 1]
        dt = float(np.median(np.diff(t_seg)))
        if dt <= 0:
            continue
        t_uniform = np.arange(t_seg[0], t_seg[-1], dt)
        if len(t_uniform) < min_pts:
            continue
        r_uniform = np.interp(t_uniform, t_seg, r_seg)
        win_len = int(round(win_s / dt))
        if win_len % 2 == 0:
            win_len += 1
        win_len = max(win_len, polyorder + 2 if (polyorder + 2) % 2 else polyorder + 3)
        if win_len > len(t_uniform):
            win_len = len(t_uniform) if len(t_uniform) % 2 else len(t_uniform) - 1
        if win_len <= polyorder or win_len < 5:
            continue
        r_smooth = savgol_filter(r_uniform, win_len, polyorder)
        drdt = savgol_filter(r_uniform, win_len, polyorder, deriv=1, delta=dt)
        out.append((t_uniform, r_smooth, drdt))
    return out


def interp_from_runs(t_query, runs):
    """Interpolate a per-run (t,val) series onto t_query; NaN outside all runs."""
    out = np.full_like(np.asarray(t_query, dtype=float), np.nan)
    for t_arr, _, val_arr in runs:
        inside = (t_query >= t_arr[0]) & (t_query <= t_arr[-1])
        if np.any(inside):
            out[inside] = np.interp(t_query[inside], t_arr, val_arr)
    return out


_log_cache = {}


def load_log(fname):
    if fname not in _log_cache:
        path = f"{LOG_DIR}/{fname}"
        print(f"[load] {fname}")
        _log_cache[fname] = yawlib.load_jsonl(path)
    return _log_cache[fname]


# =============================================================================
# Event windows (from background: E0/E1/E2/E4 of the 6 fast-yaw events)
# =============================================================================
EVENTS = [
    dict(name="E0", file="stampfly_udp_20260627T020050.jsonl", t0=30.0, t1=43.0),
    dict(name="E1", file="stampfly_udp_20260627T165713.jsonl", t0=170.0, t1=188.0),
    dict(name="E2", file="stampfly_udp_20260627T164611.jsonl", t0=110.0, t1=127.0),
    dict(name="E4", file="stampfly_udp_20260627T165713.jsonl", t0=89.0, t1=106.0),
]


def analyze_event(ev):
    data = load_log(ev["file"])
    imu = data["imu"]
    cr = data["ctrl_ref"]
    status = data["status"]

    t0, t1 = ev["t0"], ev["t1"]

    # --- gyro_z derivative (own continuous-run segmentation over full file, then mask) ---
    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
    runs = smoothed_derivative_runs(imu["ts"], r_full)

    # --- ctrl_ref window ---
    m = (cr["ts"] >= t0) & (cr["ts"] <= t1)
    t_cr = cr["ts"][m]
    duty = cr["motor_duty"][m]          # (N,4) = FR,RR,RL,FL (data_stream.cpp:281)
    mode = cr["mode"][m]

    vbat = np.interp(t_cr, status["ts"], status["voltage"])

    # --- NOMINAL (uncorrected, corr=1.0) reconstruction ---
    # This is the model space actuator.cpp/pid_controller.cpp actually compute
    # in (thrustToDuty has NO hover.thrust_corr division inside it — that
    # correction only inflates the total-thrust SETPOINT chosen upstream in
    # pid_controller.cpp; the mixer + duty conversion themselves are nominal).
    # So max_yaw_torque_=2.2e-3 Nm must be compared in THIS nominal space.
    T_nom = duty_to_thrust(duty, vbat[:, None], corr=1.0)
    T_FR_n, T_RR_n, T_RL_n, T_FL_n = T_nom[:, 0], T_nom[:, 1], T_nom[:, 2], T_nom[:, 3]
    tau_nominal = KAPPA * ((T_FR_n + T_RL_n) - (T_RR_n + T_FL_n))

    T_max_nom = max_thrust_at_duty1(vbat, corr=1.0)
    base_FR = T_FR_n - tau_nominal / (4.0 * KAPPA)
    base_RL = T_RL_n - tau_nominal / (4.0 * KAPPA)
    base_RR = T_RR_n + tau_nominal / (4.0 * KAPPA)
    base_FL = T_FL_n + tau_nominal / (4.0 * KAPPA)

    ur_max_candidates = np.stack([
        4 * KAPPA * (T_max_nom - base_FR),
        4 * KAPPA * (T_max_nom - base_RL),
        4 * KAPPA * (base_RR - 0.0),
        4 * KAPPA * (base_FL - 0.0),
    ])
    ur_min_candidates = np.stack([
        -4 * KAPPA * (base_FR - 0.0),
        -4 * KAPPA * (base_RL - 0.0),
        -4 * KAPPA * (T_max_nom - base_RR),
        -4 * KAPPA * (T_max_nom - base_FL),
    ])
    ur_max_possible_nom = np.min(ur_max_candidates, axis=0)
    ur_min_possible_nom = np.max(ur_min_candidates, axis=0)

    # --- REAL (physical, corr=1.12) reconstruction — used for the EOM balance
    # against I_z*dr/dt, which is a real physical quantity from real IMU data.
    # Linear in corr, so real = nominal / HOVER_THRUST_CORR throughout.
    tau_realized = tau_nominal / HOVER_THRUST_CORR
    ur_max_possible = ur_max_possible_nom / HOVER_THRUST_CORR
    ur_min_possible = ur_min_possible_nom / HOVER_THRUST_CORR

    drdt_at_cr = interp_from_runs(t_cr, runs)
    r_at_cr = interp_from_runs(t_cr, [(t_arr, rs, rs) for (t_arr, rs, dr) in runs])

    tau_d = IZZ * drdt_at_cr - tau_realized

    valid = ~np.isnan(tau_d)
    peak_idx = np.nanargmax(np.abs(tau_d)) if np.any(valid) else None

    # Software-cap check MUST be done in nominal (uncorrected) space — that is
    # the space max_yaw_torque_ (pid_controller.hpp:410) actually clips.
    frac_cap = float(np.mean(np.abs(tau_nominal) >= 0.95 * MAX_YAW_TORQUE))

    result = dict(
        name=ev["name"], file=ev["file"], t0=t0, t1=t1,
        t=t_cr, tau_realized=tau_realized, tau_nominal=tau_nominal, tau_d=tau_d,
        ur_max_possible=ur_max_possible, ur_min_possible=ur_min_possible,
        r_deg_s=r_at_cr * R2D, mode=mode, vbat=vbat,
        frac_time_at_cap=frac_cap,
    )
    if peak_idx is not None:
        result["peak_tau_d"] = float(tau_d[peak_idx])
        result["peak_tau_d_t"] = float(t_cr[peak_idx])
        result["peak_tau_d_sign"] = "+" if tau_d[peak_idx] > 0 else "-"
    # duration where |tau_d| exceeds 1e-3 Nm (arbitrary but reported threshold)
    thresh = 1.0e-3
    above = np.abs(np.nan_to_num(tau_d, nan=0.0)) > thresh
    result["duration_above_1e-3Nm_s"] = float(np.sum(above) * np.median(np.diff(t_cr))) if len(t_cr) > 1 else 0.0

    return result


def plot_event(res):
    t = res["t"] - res["t0"]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

    axes[0].plot(t, res["r_deg_s"], color="tab:blue", lw=1.2)
    axes[0].axhline(0, color="gray", lw=0.6)
    axes[0].set_ylabel("gyro_z r [deg/s]")
    axes[0].set_title(f"{res['name']}  {res['file']}  window [{res['t0']:.1f},{res['t1']:.1f}]s")
    axes[0].grid(alpha=0.3)

    axes[1].fill_between(t, res["ur_min_possible"] * 1e3, res["ur_max_possible"] * 1e3,
                          color="tab:gray", alpha=0.25, label="physical duty-saturation envelope (real Nm)")
    axes[1].plot(t, res["tau_realized"] * 1e3, color="tab:orange", lw=1.3, label="tau_yaw_realized (real, /1.12)")
    cap_real = MAX_YAW_TORQUE / HOVER_THRUST_CORR * 1e3
    axes[1].axhline(cap_real, color="red", ls="--", lw=0.8,
                     label="software cap 2.2mNm (nominal) = %.2f mNm (real)" % cap_real)
    axes[1].axhline(-cap_real, color="red", ls="--", lw=0.8)
    axes[1].set_ylabel("tau_yaw [mNm] (real/physical units)")
    axes[1].legend(fontsize=7, loc="upper right")
    axes[1].grid(alpha=0.3)

    axes[2].plot(t, res["tau_d"] * 1e3, color="tab:red", lw=1.3)
    axes[2].axhline(0, color="gray", lw=0.6)
    axes[2].set_ylabel("tau_d [mNm]")
    axes[2].set_xlabel(f"t - {res['t0']:.1f} [s]")
    axes[2].grid(alpha=0.3)

    fig.tight_layout()
    outpath = f"{SCRATCH}/torque_{res['name']}.png"
    fig.savefig(outpath, dpi=140)
    plt.close(fig)
    print(f"  saved {outpath}")


# =============================================================================
# Multi-axis discriminator: roll/pitch gyro activity during each event
# =============================================================================
def multiaxis_check(ev):
    data = load_log(ev["file"])
    imu = data["imu"]
    m = (imu["ts"] >= ev["t0"]) & (imu["ts"] <= ev["t1"])
    gx = imu["gyro"][m, 0] - imu["gyro_bias"][m, 0]
    gy = imu["gyro"][m, 1] - imu["gyro_bias"][m, 1]
    gz = imu["gyro"][m, 2] - imu["gyro_bias"][m, 2]
    peak_roll = float(np.nanmax(np.abs(gx))) * R2D
    peak_pitch = float(np.nanmax(np.abs(gy))) * R2D
    peak_yaw = float(np.nanmax(np.abs(gz))) * R2D
    return dict(name=ev["name"], peak_roll_dps=peak_roll, peak_pitch_dps=peak_pitch,
                peak_yaw_dps=peak_yaw,
                ratio_rollpitch_to_yaw=(max(peak_roll, peak_pitch) / peak_yaw) if peak_yaw > 0 else float("nan"))


# =============================================================================
# Static hover CW/CCW asymmetry (quiet hover windows)
# =============================================================================
QUIET_FILES = [
    "stampfly_udp_20260627T020050.jsonl",
    "stampfly_udp_20260627T164611.jsonl",
    "stampfly_udp_20260627T165713.jsonl",
]


def find_quiet_windows(fname, gyro_thresh_dps=20.0, stick_thresh=0.05, min_dur_s=5.0):
    data = load_log(fname)
    imu = data["imu"]
    ctrl = data["ctrl"]
    cr = data["ctrl_ref"]

    t_cr = cr["ts"]
    duty_mean = np.nanmean(cr["motor_duty"], axis=1)
    duty_min = np.nanmin(cr["motor_duty"], axis=1)
    # duty_min > 0.15 excludes the arm/spin-up transient (individual motors
    # pulsing near 0 while the craft is still on the ground) seen at the start
    # of some flight segments — a genuine hover has all 4 motors well above 0.
    flying = (duty_mean > 0.3) & (duty_min > 0.15)

    r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
    r_at_cr = np.interp(t_cr, imu["ts"], r_full, left=np.nan, right=np.nan)
    gyro_quiet = np.abs(r_at_cr) * R2D < gyro_thresh_dps

    yaw_c = np.interp(t_cr, ctrl["ts"], ctrl["yaw"], left=np.nan, right=np.nan)
    roll_c = np.interp(t_cr, ctrl["ts"], ctrl["roll"], left=np.nan, right=np.nan)
    pitch_c = np.interp(t_cr, ctrl["ts"], ctrl["pitch"], left=np.nan, right=np.nan)
    stick_neutral = ((np.abs(yaw_c) < stick_thresh) & (np.abs(roll_c) < stick_thresh) &
                      (np.abs(pitch_c) < stick_thresh))

    quiet = flying & gyro_quiet & stick_neutral & ~np.isnan(r_at_cr)

    idx = np.where(quiet)[0]
    windows = []
    if len(idx) == 0:
        return windows
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    for s, e in zip(starts, ends):
        dur = t_cr[e] - t_cr[s]
        if dur >= min_dur_s:
            windows.append((s, e, t_cr[s], t_cr[e]))
    return windows


def static_asymmetry(fname):
    data = load_log(fname)
    cr = data["ctrl_ref"]
    status = data["status"]
    windows = find_quiet_windows(fname)
    results = []
    for s, e, t0, t1 in windows:
        duty = cr["motor_duty"][s:e + 1]
        t_seg = cr["ts"][s:e + 1]
        vbat = np.interp(t_seg, status["ts"], status["voltage"])
        duty_diff = np.mean(duty[:, 0] + duty[:, 2]) - np.mean(duty[:, 1] + duty[:, 3])
        T = duty_to_thrust(duty, vbat[:, None])
        tau_static = np.mean(KAPPA * ((T[:, 0] + T[:, 2]) - (T[:, 1] + T[:, 3])))
        vbat_mean = float(np.mean(vbat))
        results.append(dict(t0=float(t0), t1=float(t1), dur=float(t1 - t0),
                             duty_diff=float(duty_diff), tau_static_Nm=float(tau_static),
                             vbat_mean=vbat_mean))
    return results


# =============================================================================
# Microwind physical plausibility estimate
# =============================================================================
def microwind_estimate():
    """
    Order-of-magnitude yaw moment from a 1-2 m/s ambient gust acting on an
    asymmetric feature of the frame (antenna/canopy/battery bump offset from CG).
    A perfectly symmetric X-quad frame in uniform steady wind produces ~zero net
    yaw moment (drag force acts through the frontal-area centroid, which is
    close to CG for a symmetric shape); a real yaw moment requires either (a) an
    asymmetric drag-center offset, or (b) rotor-inflow asymmetry (differential
    induced-velocity effect on advancing/retreating blade sides across the 4
    rotors under a crosswind — a real but typically SMALL secondary effect for
    a hovering multirotor, since each rotor's own thrust >> the wind dynamic
    pressure it experiences).
    """
    rho = 1.225      # air density [kg/m^3]
    v_list = [1.0, 2.0]
    # Asymmetric-drag-center model: frame frontal area ~ collision box
    # 0.0408*2 x 0.0103*2 (stampfly.xml:89) plus a CG-offset assumption.
    frontal_area = (0.0408 * 2) * (0.0103 * 2)  # [m^2], conservative (box side)
    cd = 1.2
    cg_offset_arm = 0.015  # [m] assumed drag-center offset from CG (uncertain, order-of-magnitude)
    out = []
    for v in v_list:
        q = 0.5 * rho * v**2
        f_drag = cd * q * frontal_area
        tau_wind = f_drag * cg_offset_arm
        out.append(dict(v=v, q_Pa=q, f_drag_N=f_drag, tau_wind_Nm=tau_wind))
    return out


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 78)
    print("Physical parameters")
    print(f"  KAPPA={KAPPA} ARM_D={ARM_D} I_z={IZZ} mass={MASS_KG}")
    print(f"  motor curve: Am={MOTOR_AM} Bm={MOTOR_BM} Cm={MOTOR_CM} Ct={MOTOR_CT}")
    print(f"  hover.thrust_corr={HOVER_THRUST_CORR} (default; onboard-learned value not in telemetry)")
    print(f"  max_yaw_torque_={MAX_YAW_TORQUE} Nm (software cap)")

    # --- sanity: hover duty/voltage check ---
    T_hover = MASS_KG * G / 4.0
    vb = 3.7
    d_hover = None
    for d in np.linspace(0, 1, 4001):
        t = duty_to_thrust(d, vb, corr=1.0)
        if t >= T_hover:
            d_hover = d
            break
    print(f"  sanity: hover per-motor thrust={T_hover*1e3:.2f} mN -> duty~{d_hover:.3f} @ {vb}V (corr=1.0, uninflated)")

    event_results = []
    print("\n" + "=" * 78)
    print("Event torque reconstruction")
    for ev in EVENTS:
        print(f"\n--- {ev['name']}: {ev['file']} [{ev['t0']},{ev['t1']}]s ---")
        res = analyze_event(ev)
        event_results.append(res)
        print(f"  n_samples={len(res['t'])}")
        if "peak_tau_d" in res:
            print(f"  peak |tau_d| = {res['peak_tau_d']*1e3:.3f} mNm (sign {res['peak_tau_d_sign']}) at t-t0={res['peak_tau_d_t']-res['t0']:.2f}s")
        print(f"  duration |tau_d|>1mNm = {res['duration_above_1e-3Nm_s']:.2f} s")
        print(f"  fraction of time |tau_NOMINAL|>=95% of software cap (2.2mNm) = {res['frac_time_at_cap']*100:.1f}%")
        print(f"  tau_nominal range (model-space, compared to 2.2mNm cap) = "
              f"[{np.nanmin(res['tau_nominal'])*1e3:.3f}, {np.nanmax(res['tau_nominal'])*1e3:.3f}] mNm")
        print(f"  tau_realized range (REAL physical, /1.12 corr, compared to tau_d) = "
              f"[{np.nanmin(res['tau_realized'])*1e3:.3f}, {np.nanmax(res['tau_realized'])*1e3:.3f}] mNm")
        mx = multiaxis_check(ev)
        print(f"  multiaxis: peak roll={mx['peak_roll_dps']:.1f} deg/s pitch={mx['peak_pitch_dps']:.1f} deg/s yaw={mx['peak_yaw_dps']:.1f} deg/s ratio(roll/pitch : yaw)={mx['ratio_rollpitch_to_yaw']:.3f}")
        # baseline tau_realized just before the fast-yaw sub-event (first 2s of window)
        # vs at the peak, to see whether the event is a transient ON TOP OF the static bias.
        base_mask = (res["t"] - res["t0"]) < 2.0
        if np.any(base_mask):
            base_tau = float(np.nanmean(res["tau_realized"][base_mask]))
            print(f"  baseline tau_realized (first 2s of window) = {base_tau*1e3:.3f} mNm "
                  f"vs peak-|tau_d| tau_realized = {res['tau_realized'][np.nanargmax(np.abs(res['tau_d']))]*1e3:.3f} mNm")
        plot_event(res)

    print("\n" + "=" * 78)
    print("Static hover CW/CCW asymmetry (quiet windows: |gyro_z|<20deg/s, stick neutral,")
    print("all 4 motor duty>0.15, mean duty>0.3, >=5s continuous)")
    static_results = {}
    all_vbat, all_tau, all_duty_diff = [], [], []
    for fname in QUIET_FILES:
        res = static_asymmetry(fname)
        static_results[fname] = res
        print(f"\n{fname}: {len(res)} quiet window(s) >=5s")
        for r in res:
            print(f"  [{r['t0']:.1f},{r['t1']:.1f}]s dur={r['dur']:.1f}s vbat={r['vbat_mean']:.3f}V "
                  f"duty_diff(CCW-CW)={r['duty_diff']:.4f} tau_static={r['tau_static_Nm']*1e3:.4f} mNm")
            all_vbat.append(r["vbat_mean"])
            all_tau.append(r["tau_static_Nm"])
            all_duty_diff.append(r["duty_diff"])

    all_vbat = np.array(all_vbat)
    all_tau = np.array(all_tau)
    all_duty_diff = np.array(all_duty_diff)
    n_pos = int(np.sum(all_tau > 0))
    print(f"\nAcross all {len(all_tau)} quiet windows (both venue flights):")
    print(f"  sign: {n_pos}/{len(all_tau)} windows have tau_static > 0 (CCW-group-dominant)")
    print(f"  tau_static: mean={np.mean(all_tau)*1e3:.4f} mNm, std={np.std(all_tau)*1e3:.4f} mNm, "
          f"range=[{np.min(all_tau)*1e3:.4f},{np.max(all_tau)*1e3:.4f}] mNm")
    if len(all_tau) >= 3:
        corr = np.corrcoef(all_vbat, all_tau)[0, 1]
        print(f"  corr(vbat, tau_static) = {corr:.3f}  (near 0 => no clear voltage-sag trend; "
              f"negative => bias GROWS as battery drains)")

    print("\n" + "=" * 78)
    print("Microwind plausibility estimate")
    for r in microwind_estimate():
        print(f"  v={r['v']} m/s: q={r['q_Pa']:.3f} Pa, F_drag={r['f_drag_N']*1e3:.3f} mN, "
              f"tau_wind(asym-drag-center model)={r['tau_wind_Nm']*1e3:.4f} mNm")

    # Save a compact JSON summary
    summary = {
        "params": dict(KAPPA=KAPPA, ARM_D=ARM_D, IZZ=IZZ, MASS_KG=MASS_KG,
                        MOTOR_AM=MOTOR_AM, MOTOR_BM=MOTOR_BM, MOTOR_CM=MOTOR_CM, MOTOR_CT=MOTOR_CT,
                        HOVER_THRUST_CORR=HOVER_THRUST_CORR, MAX_YAW_TORQUE=MAX_YAW_TORQUE),
        "events": [
            {k: v for k, v in r.items() if k not in
             ("t", "tau_realized", "tau_d", "ur_max_possible", "ur_min_possible", "r_deg_s", "mode", "vbat")}
            for r in event_results
        ],
        "static": {f: rs for f, rs in static_results.items()},
        "microwind": microwind_estimate(),
    }
    with open(f"{SCRATCH}/torque_budget_summary.json", "w") as f:
        json.dump(summary, f, indent=2, default=str)
    print(f"\nSaved summary -> {SCRATCH}/torque_budget_summary.json")


if __name__ == "__main__":
    main()
