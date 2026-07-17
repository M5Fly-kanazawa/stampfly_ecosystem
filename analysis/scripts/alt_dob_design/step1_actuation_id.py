#!/usr/bin/env python3
"""step1_actuation_id.py — Vertical actuation-path identification for altitude DOB design.
鉛直アクチュエーション経路の同定（高度DOB設計の前段）

Goal / 目的
-----------
The vertical disturbance-observer (DOB) feasibility is set by the lag between the
commanded total_thrust and the ACTUAL vertical force the airframe feels. This script
splits the velocity loop's known total lag (TAU=0.11s) into:
  - the motor/ESC/prop actuation lag (what the DOB has to fight), and
  - the ESKF estimation lag (what a DOB bypasses by using raw accel instead of the
    velocity-loop's filtered vz).
高度DOBの実現可能性は「推力コマンド→機体が実際に感じる鉛直力」の遅れで決まる。
速度ループの合計遅れ(TAU=0.11s)を、モータ/ESC/プロペラのアクチュエーション遅れ
（DOBが受ける遅れ）とESKF推定側の遅れ（DOBがバイパスできる遅れ）に分解する。

Four sub-tasks (see module functions):
  1. step1_sanity_check_f_up()   — verify the accel(body,FRD)->NED->f_up rotation sign
  2. step2_actuation_id()        — H(jw)=Suy/Suu + coherence (freq domain) + L/T/g fit
                                    (model) + takeoff/landing transient delay (time domain)
  3. step3_noise_budget()        — f_up PSD during hover + DOB Q-filter noise propagation
  4. step4_dext_cross_check()    — accel-based d_ext vs. the existing kinematic d_ext
                                    (alt_ti_venue_sim.dext_for_segment), all 4 logs

IMPORTANT FINDING baked into this script (see report / §5 anomalies): the JSONL "accel"
field is documented (task brief) as "bias-corrected", but firmware source
(firmware/vehicle/tasks/control_task.cpp:86 `sample.accel[i] = imu.accel[i]`, and
firmware/vehicle/components/sf_estimator_eskf/eskf_core.cpp:174
`Vec3 accel = accel_raw - ba_;`) shows the wire "accel" field is the RAW (bias
calibration NOT subtracted) sensor value scaled to m/s²; "accel_raw" is bit-identical
to "accel" on the wire (data_stream_wire.hpp:274-276 assigns both from the same
source). The physically-correct bias-corrected specific force is accel - accel_bias
(both logged per-sample). This script always uses accel - accel_bias.
重要な発見（詳細は報告書§5）: タスク指示書はJSONLの"accel"を「バイアス補正済み」と
説明しているが、ファームウェアのソース（control_task.cpp:86, eskf_core.cpp:174）を見ると
"accel"はバイアス未補正の生値（m/s²にスケール変換のみ）で、"accel_raw"は電文上
"accel"と完全に同一（data_stream_wire.hpp）。物理的に正しい比力は accel - accel_bias
（両方とも毎サンプル記録されている）。本スクリプトは常にこの補正を適用する。

Usage / 使い方
--------------
    python3 analysis/scripts/alt_dob_design/step1_actuation_id.py

No arguments; log paths and flight windows are the 4 fixed logs from the task brief
(A/B auto-detected via total_thrust/altitude threshold, C/D use the given raw-ts
windows). Outputs:
    analysis/scripts/alt_dob_design/step1_results.json   (machine-readable)
    analysis/scripts/alt_dob_design/figs/*.png           (bode/coherence, transients,
                                                            noise PSD, d_ext comparison)
Prints a human-readable summary to stdout.

引数なし。ログパスと飛行窓はタスク指示書の4本（A/Bは閾値で自動検出、C/Dは指定の
raw-ts窓）に固定。出力は上記の通り。
"""

import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import welch, csd, coherence as scipy_coherence, butter, filtfilt
from scipy.interpolate import interp1d

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO / "analysis" / "scripts" / "yaw_nt_kanazawa"))
sys.path.insert(0, str(REPO / "analysis" / "scripts"))
from yawlib import load_jsonl, detect_flight_segments  # noqa: E402
import alt_ti_venue_sim as avs  # noqa: E402  (reused for the kinematic d_ext + gap-aware 100Hz grid)

FIGS_DIR = HERE / "figs"
FIGS_DIR.mkdir(parents=True, exist_ok=True)
RESULTS_PATH = HERE / "step1_results.json"

# =============================================================================
# Physical / plant constants (matches poshold_alt_loop_sim.py / alt_ti_venue_sim.py
# conventions so all three scripts stay comparable)
# 物理・プラント定数（既存2スクリプトの慣例と揃える）
# =============================================================================
M = 0.037                       # airframe mass [kg]
G = 9.80665                     # standard gravity [m/s^2]
HOVER_CORR = 1.12                # hover-thrust correction factor (existing convention)
HOVER = M * G * HOVER_CORR       # nominal hover thrust [N] (~0.4066 N)
TAU_COMBINED_REF = 0.11          # existing velocity-loop-inferred TOTAL lag [s] (reference only)
TAU_M_REF = 0.02                 # motor time constant estimate, tools/sysid/defaults.py [s] (reference only)

LOGS_DIR = REPO / "logs"
LOG_A = LOGS_DIR / "stampfly_udp_20260717T231940.jsonl"   # 137s hands-off POS_HOLD hover + landing, low gap
LOG_B = LOGS_DIR / "stampfly_udp_20260717T224906.jsonl"   # POS_HOLD, big altitude excursions (rich excitation)
LOG_C = LOGS_DIR / "stampfly_udp_20260627T164611.jsonl"   # venue, up to 25% telemetry loss
LOG_D = LOGS_DIR / "stampfly_udp_20260627T165713.jsonl"   # venue, up to 25% telemetry loss

WIN_C = (33.1, 199.6)   # raw ts, given by task brief (verified against detect_flight_segments)
WIN_D = (56.3, 196.5)   # raw ts, given by task brief


# =============================================================================
# Core physics: body(FRD) accel -> NED -> upward specific force f_up
# 物理コア: body(FRD)加速度 -> NED -> 上向き比力 f_up
# =============================================================================
def quat_to_dcm_row3(quat):
    """Third row of the body->NED DCM (q=[w,x,y,z], q_nb convention), i.e. the
    row that projects a body-frame vector onto the NED z (down) axis. Matches
    sf_math.hpp Quat::to_dcm() R[2][*] exactly (verified against firmware source).
    q_nb（[w,x,y,z]、body->NED）のDCM第3行（NED下向きz軸への射影）。
    firmware sf_math.hpp の Quat::to_dcm() R[2][*] と一致（ソース照合済み）。

    Returns (R31, R32, R33) arrays.
    """
    quat = np.asarray(quat, dtype=float)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    r31 = 2.0 * (x * z - w * y)
    r32 = 2.0 * (y * z + w * x)
    r33 = 1.0 - 2.0 * (x * x + y * y)
    return r31, r32, r33


def body_accel_to_f_up(accel_body, quat):
    """f_up = -(R_nb @ accel_body)_z  — upward specific force [m/s^2].
    accel_body: (N,3) body FRD [ax,ay,az], BIAS-CORRECTED (caller's job, see
    bias_corrected_accel()). quat: (N,4) [w,x,y,z] q_nb.
    Also returns cos_tilt = R33 (=1 at level attitude), used elsewhere as the
    thrust vertical-projection factor.
    """
    r31, r32, r33 = quat_to_dcm_row3(quat)
    az_ned = r31 * accel_body[:, 0] + r32 * accel_body[:, 1] + r33 * accel_body[:, 2]
    f_up = -az_ned
    return f_up, r33


def bias_corrected_accel(imu):
    """accel - accel_bias, see module docstring for why this is required
    (wire 'accel' field is raw, not bias-corrected)."""
    return imu["accel"] - imu["accel_bias"]


# =============================================================================
# Gap-aware uniform-grid utilities (local copy; same algorithm as
# alt_ti_venue_sim.build_gap_mask/find_valid_segments but fs-parameterized so we
# can run it at 200Hz for the frequency-domain ID here, while step4 calls the
# ORIGINAL alt_ti_venue_sim functions directly at their native FS=100 for exact
# reuse of the kinematic method).
# ギャップ対応の等間隔グリッドユーティリティ（fsを引数化したローカル版。
# 周波数領域同定はここで200Hzに使う。step4は既存の運動学的手法を一切改変せず
# そのままimportして使う — FS=100固定）。
# =============================================================================
def build_gap_mask(win, time_arrays, gap_thr, fs):
    tg = np.arange(win[0], win[1], 1.0 / fs)
    invalid = np.zeros(len(tg), dtype=bool)
    for t_native in time_arrays:
        tw = np.sort(t_native)
        m = (tw >= win[0] - 5) & (tw <= win[1] + 5)
        tw = tw[m]
        if len(tw) < 2:
            invalid[:] = True
            continue
        dt = np.diff(tw)
        idx = np.where(dt > gap_thr)[0]
        for i in idx:
            invalid |= (tg > tw[i]) & (tg < tw[i + 1])
        invalid |= (tg < tw[0]) | (tg > tw[-1])
    return tg, invalid


def find_valid_segments(tg, invalid, min_len, fs):
    valid = ~invalid
    if not np.any(valid):
        return []
    idx = np.where(valid)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    segs = []
    min_n = int(min_len * fs)
    for s, e in zip(starts, ends):
        if e - s + 1 >= min_n:
            segs.append((s, e))
    return segs


def zoh_interp(tg, t_native, x_native):
    """Zero-order-hold ('previous value held') resample — models the firmware
    holding a 50Hz total_thrust command until the next update.
    ZOH再サンプル（直前値保持）— 50Hzコマンドを次回更新まで保持するファーム挙動を模擬。"""
    f = interp1d(t_native, x_native, kind="previous", bounds_error=False,
                 fill_value=(x_native[0], x_native[-1]))
    return f(tg)


def antialias_lowpass(x, fs_native, cutoff_hz, order=4):
    """Zero-phase (filtfilt) Butterworth low-pass, used BEFORE decimating the
    400Hz-native f_up down to a coarser analysis grid (200Hz for step2, 100Hz
    for step4). Without this, naive np.interp decimation aliases the large
    20-200Hz accelerometer/vibration content (RMS ~2.3 m/s^2, see step3) into
    the decimated series: measured ablation shows the FULL-BAND std of
    d_ext_acc inflates ~1.8x on log A (1.2x on log C) without this filter.
    NOTE (2026-07-18 audit): the 0.02-1Hz band RMS ratio acc/kin (~1.7-2.3x)
    is NOT explained by aliasing -- the ablation leaves that ratio essentially
    unchanged (+9% / -1%). The residual acc/kin gap is attributed elsewhere
    (d_ext_kin's double smoothing attenuates in-band; cause not confirmed).
    filtfilt runs on the whole native array in one pass (index-based, not
    gap-aware) -- for the C/D venue logs (up to 25% loss) this can leave a
    few-sample transient right after a native gap resumes, but that artifact
    is confined to ~1/cutoff_hz seconds (a few ms) out of >=2s valid segments
    and is broadband (does not preferentially land in the 0.02-1Hz band we
    integrate), so it is accepted rather than gap-splitting the filter.
    400Hzネイティブのf_upを粗いグリッドに間引く前に必ずかける零位相LPF。これが
    ないと20-200Hz帯の大きな加速度計/振動ノイズ(RMS~2.3m/s^2、step3参照)が
    単純np.interpの間引きでエイリアシングし、d_ext_accの全帯域stdが約1.8倍
    （ログA実測、ログCは1.2倍）に膨張する。注（2026-07-18監査）: 0.02-1Hz帯の
    acc/kin比（約1.7-2.3倍）はエイリアシングでは説明できない（アブレーションで
    比はほぼ不変）。残存差は運動学側の二重平滑化による帯域内減衰等が候補（真因は
    未確定）。C/Dのギャップ直後に数サンプルの過渡が乗り得るが、続く長さ>=2sの
    有効区間に対しては無視できる。"""
    nyq = fs_native / 2.0
    wn = min(cutoff_hz / nyq, 0.99)
    b, a = butter(order, wn, btype="low")
    return filtfilt(b, a, x)


def detect_window_by_threshold(t_ctrl, thrust, t_pv, alt, thrust_thr=0.2, alt_thr=0.15):
    """Auto-detect the flight window: largest contiguous run (on the ctrl_ref
    50Hz timebase) where total_thrust>thrust_thr AND altitude>alt_thr.
    飛行窓の自動検出: total_thrust>thrust_thr かつ 高度>alt_thr の最大連続区間。

    NOTE: kept for reference/diagnostics only -- during a POS_HOLD altitude bob
    the altitude can briefly dip below alt_thr (e.g. log A touches 0.155m /
    0.160m mid-flight), fragmenting this "largest single run" into a much
    shorter piece than the true flight. detect_flight_window() below (motor
    duty based, with gap-bridging) is what main() actually uses.
    注記: 参考用に残すのみ。POS_HOLDの高度ボブで一時的にalt_thrを割ることがあり
    （ログAは飛行中に0.155m/0.160mまで低下）、「最大単一連続区間」だと真の飛行区間
    より大幅に短く分断される。main()が実際に使うのは下のdetect_flight_window()
    （motor duty基準、ギャップ橋渡し付き）。"""
    alt_on_tcr = np.interp(t_ctrl, t_pv, alt)
    mask = (thrust > thrust_thr) & (alt_on_tcr > alt_thr)
    if not np.any(mask):
        raise RuntimeError("no samples pass the flight-window threshold")
    idx = np.where(mask)[0]
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]]))
    lengths = ends - starts
    k = np.argmax(lengths)
    return float(t_ctrl[starts[k]]), float(t_ctrl[ends[k]])


def detect_flight_window(t_ctrl, motor_duty, min_duration_s=10.0, merge_gap_s=2.0):
    """Robust flight-window detector used by main(): motor_duty-based (via
    yawlib.detect_flight_segments), which bridges the brief low-altitude dips
    of a POS_HOLD altitude bob instead of fragmenting on them. Returns the
    LONGEST merged segment (t0, t1) in raw ts seconds.
    実際にmain()が使う頑健な飛行窓検出: motor_duty基準（yawlib.detect_flight_segments）。
    POS_HOLDの高度ボブによる一時的な低高度をギャップ橋渡しで吸収する。最長の
    結合区間 (t0, t1) を返す（raw ts秒）。"""
    segs = detect_flight_segments(t_ctrl, motor_duty, min_duration_s=min_duration_s,
                                   merge_gap_s=merge_gap_s)
    if not segs:
        raise RuntimeError("no flight segment detected")
    segs.sort(key=lambda s: s[1] - s[0])
    return segs[-1]


def load_log_channels(path):
    """Thin wrapper: load a log and pull out the 3 channels used throughout
    (imu accel/accel_bias/quat, posvel pos/vel, ctrl_ref total_thrust/motor_duty)."""
    d = load_jsonl(str(path))
    return d


# =============================================================================
# Step 1 — f_up sign/convention sanity check
# ステップ1 — f_up の符号・変換規約の検証
# =============================================================================
def step1_sanity_check_f_up(d):
    """Verify mean(f_up) ~= +G on (a) a genuine pre-arm ground-static window and
    (b) the best-available low-net-drift in-flight window (exercises the FULL
    DCM row, not just R33, since attitude is not level there)."""
    imu = d["imu"]
    t = imu["ts"]
    quat = imu["quat"]
    accel_raw = imu["accel"]
    accel_corr = bias_corrected_accel(imu)

    # --- (a) ground-static window: before the flight segment starts, altitude==0 ---
    tcr = d["ctrl_ref"]["ts"]
    tpv = d["posvel"]["ts"]
    t0_flight, t1_flight = detect_flight_window(tcr, d["ctrl_ref"]["motor_duty"])
    # Logs only start recording a few seconds before ARM (no long pre-boot
    # ground period), so take whatever pre-flight sliver is available -- prefer
    # up to 30s before flight start but never before the log's own first sample.
    # ログはARM直前の数秒しか記録が無いことが多い（起動直後からの長い地上静止
    # 区間は無い）。飛行開始30秒前まで遡ろうとするが、ログの先頭より前には行かない。
    ground_hi = t0_flight - 0.5
    ground_lo = max(t[0] + 0.2, t0_flight - 30)
    if ground_lo >= ground_hi:
        ground_lo, ground_hi = t[0] + 0.1, t0_flight - 0.1
    ground_mask = (t >= ground_lo) & (t <= ground_hi)

    fup_ground_raw, _ = body_accel_to_f_up(accel_raw[ground_mask], quat[ground_mask])
    fup_ground_corr, _ = body_accel_to_f_up(accel_corr[ground_mask], quat[ground_mask])

    # --- (b) best low-net-drift in-flight window: minimize |vz(t1)-vz(t0)|/duration
    #     over sliding 25s windows inside the flight span (exercises full attitude) ---
    vel = d["posvel"]["vel"]
    vz = -vel[:, 2]
    win_len = 25.0
    starts = np.arange(t0_flight + 2, t1_flight - win_len - 2, 2.0)
    best = None
    for ws in starts:
        we = ws + win_len
        vz0 = np.interp(ws, tpv, vz)
        vz1 = np.interp(we, tpv, vz)
        drift = abs(vz1 - vz0) / win_len
        if best is None or drift < best[0]:
            best = (drift, ws, we)
    _, fws, fwe = best
    fmask = (t >= fws) & (t <= fwe)
    fup_flight_raw, cos_flight = body_accel_to_f_up(accel_raw[fmask], quat[fmask])
    fup_flight_corr, _ = body_accel_to_f_up(accel_corr[fmask], quat[fmask])

    result = dict(
        ground_window_s=[float(t[ground_mask][0]), float(t[ground_mask][-1])],
        ground_n=int(ground_mask.sum()),
        f_up_ground_mean_raw=float(np.mean(fup_ground_raw)),
        f_up_ground_std_raw=float(np.std(fup_ground_raw)),
        f_up_ground_mean_biascorr=float(np.mean(fup_ground_corr)),
        f_up_ground_std_biascorr=float(np.std(fup_ground_corr)),
        flight_lowdrift_window_s=[float(fws), float(fwe)],
        flight_lowdrift_vz_drift_mps2=float(best[0]),
        f_up_flight_mean_raw=float(np.mean(fup_flight_raw)),
        f_up_flight_mean_biascorr=float(np.mean(fup_flight_corr)),
        f_up_flight_mean_cos_tilt=float(np.mean(cos_flight)),
        g_reference=G,
        note=("wire 'accel' is RAW (not bias-corrected) despite the task-brief schema "
              "description; correct specific force = accel - accel_bias. See module "
              "docstring / firmware source citations."),
    )
    return result, (t0_flight, t1_flight)


# =============================================================================
# Step 2a — frequency-domain H(jw) = Suy/Suu + coherence, logs A & B
# ステップ2a — 周波数領域 H(jw)=Suy/Suu + コヒーレンス（ログA,B）
# =============================================================================
FS_ID = 200.0
GAP_THR_ID = 0.05
MIN_SEG_ID = 20.0


def build_id_grid(d, win, fs=FS_ID, gap_thr=GAP_THR_ID, min_seg=MIN_SEG_ID):
    """Resample u_eff=total_thrust*cos(tilt) and y=M*f_up onto a common fs-Hz
    grid over `win`, gap-masked against the imu and ctrl_ref native timelines.
    Thrust uses ZOH (firmware holds the 50Hz command); f_up is anti-alias
    low-pass filtered (see antialias_lowpass()) before linear-interpolation
    decimation from the native 400Hz IMU down to the fs-Hz target grid."""
    imu = d["imu"]
    tim = imu["ts"]
    accel_corr = bias_corrected_accel(imu)
    f_up_native, cos_native = body_accel_to_f_up(accel_corr, imu["quat"])
    fs_imu_native = 1.0 / np.median(np.diff(tim))
    f_up_native = antialias_lowpass(f_up_native, fs_imu_native, cutoff_hz=0.8 * (fs / 2.0))

    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]

    tg, invalid = build_gap_mask(win, [tim, tcr], gap_thr, fs)
    segs = find_valid_segments(tg, invalid, min_seg, fs)

    fupg = np.interp(tg, tim, f_up_native)
    cosg = np.interp(tg, tim, cos_native)
    thrg = zoh_interp(tg, tcr, thr)

    u_eff = thrg * cosg
    y = M * fupg
    return dict(tg=tg, invalid=invalid, segs=segs, u_eff=u_eff, y=y,
                thrg=thrg, cosg=cosg, fupg=fupg, win=win)


def segment_cross_spectrum(u_seg, y_seg, fs):
    """Welch cross-spectrum + coherence for ONE contiguous (gap-free) segment.
    nperseg targets ~20s, adapted down for shorter segments (>=3 windows)."""
    n = len(u_seg)
    target = int(20.0 * fs)
    # Cap nperseg so we get at least ~3 (non-overlapping; more with 50% Welch
    # overlap) windows for a minimally-reliable coherence estimate, but never
    # exceed the ~20s target (frequency-resolution ceiling).
    # 少なくとも約3窓（Welchの50%オーバーラップ込みではそれ以上）確保しつつ、
    # 目標の20s(周波数分解能の上限)は超えないようにnpersegを決める。
    nperseg = min(target, max(16, n // 3))
    nperseg = min(nperseg, n)
    if nperseg < 16:
        return None
    f, Pxy = csd(u_seg, y_seg, fs=fs, nperseg=nperseg)
    _, Pxx = welch(u_seg, fs=fs, nperseg=nperseg)
    _, Pyy = welch(y_seg, fs=fs, nperseg=nperseg)
    return dict(f=f, Pxy=Pxy, Pxx=Pxx, Pyy=Pyy, dur=n / fs)


def combine_cross_spectra(specs, f_common=None, df=0.02, f_max=20.0):
    """Duration-weighted average of Pxy/Pxx/Pyy across segments (and, if called
    again, across logs) onto a common frequency grid. Never averages RAW time
    series across a gap boundary — only already-independent per-segment Welch
    outputs are combined here (same principle as alt_ti_venue_sim.band_power_welch).
    """
    if f_common is None:
        f_common = np.arange(0.0, f_max + df, df)
    Pxy_sum = np.zeros(len(f_common), dtype=complex)
    Pxx_sum = np.zeros(len(f_common))
    Pyy_sum = np.zeros(len(f_common))
    dur_sum = 0.0
    for sp in specs:
        if sp is None:
            continue
        dur = sp["dur"]
        Pxy_i = np.interp(f_common, sp["f"], sp["Pxy"].real) + \
            1j * np.interp(f_common, sp["f"], sp["Pxy"].imag)
        Pxx_i = np.interp(f_common, sp["f"], sp["Pxx"])
        Pyy_i = np.interp(f_common, sp["f"], sp["Pyy"])
        Pxy_sum += Pxy_i * dur
        Pxx_sum += Pxx_i * dur
        Pyy_sum += Pyy_i * dur
        dur_sum += dur
    if dur_sum == 0:
        return None
    Pxy_avg = Pxy_sum / dur_sum
    Pxx_avg = Pxx_sum / dur_sum
    Pyy_avg = Pyy_sum / dur_sum
    H = Pxy_avg / Pxx_avg
    coh = (np.abs(Pxy_avg) ** 2) / (Pxx_avg * Pyy_avg + 1e-30)
    return dict(f=f_common, H=H, coherence=coh, dur_s=dur_sum)


def step2a_frequency_id(logs_grids):
    """logs_grids: dict name -> build_id_grid() output. Returns per-log combined
    H/coherence plus the overall (all logs, all segments) combined H/coherence
    used for the model fit."""
    per_log = {}
    all_specs = []
    for name, G_ in logs_grids.items():
        specs = []
        for s, e in G_["segs"]:
            sp = segment_cross_spectrum(G_["u_eff"][s:e + 1], G_["y"][s:e + 1], FS_ID)
            specs.append(sp)
            if sp is not None:
                all_specs.append(sp)
        comb = combine_cross_spectra(specs)
        per_log[name] = comb
    combined = combine_cross_spectra(all_specs)
    return per_log, combined


# =============================================================================
# Step 2b — model fit: H(s) = g * exp(-L s) / (T s + 1)
# ステップ2b — モデルフィット
# =============================================================================
def model_shape(f, L, T):
    """Unit-gain model shape phi(f) = exp(-j*2*pi*f*L) / (1 + j*2*pi*f*T)."""
    w = 2.0 * np.pi * f
    num = np.exp(-1j * w * L)
    den = 1.0 + 1j * w * T
    return num / den


def fit_gain_and_residual(H_meas, f, L, T, weights):
    phi = model_shape(f, L, T)
    num = np.sum(weights * np.conj(phi) * H_meas).real
    den = np.sum(weights * np.abs(phi) ** 2)
    if den <= 0:
        return np.nan, np.inf
    g_hat = num / den
    resid = np.sum(weights * np.abs(H_meas - g_hat * phi) ** 2)
    return float(g_hat), float(resid)


def grid_search_model(H_meas, f, weights, L_grid, T_grid):
    best = None
    for L in L_grid:
        for T in T_grid:
            g_hat, resid = fit_gain_and_residual(H_meas, f, L, T, weights)
            if best is None or resid < best["resid"]:
                best = dict(L=float(L), T=float(T), g=g_hat, resid=resid)
    return best


def step2b_model_fit(combined):
    """Restrict to the coherence>0.5 band (report the actual band found, do not
    assume 0.1-5Hz), grid-search 3 model variants, report residuals + gains."""
    f = combined["f"]
    H = combined["H"]
    coh = combined["coherence"]

    band_mask = (coh > 0.5) & (f >= 0.02) & (f <= 15.0)
    if band_mask.sum() < 5:
        band_mask = (coh > 0.3) & (f >= 0.02) & (f <= 15.0)
    f_band = f[band_mask]
    H_band = H[band_mask]
    weights = coh[band_mask]

    L_grid = np.arange(0.0, 0.1 + 1e-9, 0.0025)
    T_grid = np.arange(0.005, 0.15 + 1e-9, 0.005)

    full = grid_search_model(H_band, f_band, weights, L_grid, T_grid)
    delay_only = grid_search_model(H_band, f_band, weights, L_grid, np.array([0.0]))
    lag_only = grid_search_model(H_band, f_band, weights, np.array([0.0]), T_grid)

    norm = float(np.sum(weights * np.abs(H_band) ** 2))
    for m in (full, delay_only, lag_only):
        m["resid_frac"] = m["resid"] / norm if norm > 0 else np.nan

    trusted_band = (float(f_band.min()), float(f_band.max())) if len(f_band) else (np.nan, np.nan)
    return dict(
        trusted_band_hz=trusted_band,
        n_freq_bins_used=int(band_mask.sum()),
        full=full, delay_only=delay_only, lag_only=lag_only,
        band_mask_threshold=0.5 if band_mask.sum() >= 5 else 0.3,
    )


def dc_gain_check(d, flight_window):
    """Simple, robust DC-gain cross-check: mean(M*f_up)/mean(u_eff) over the
    flight window (minus 3s edges). Independent of the noisy frequency-domain
    weighted-LS fit (no delay/phase sensitivity at all) -- used as a sanity
    bound on g, since step2b's grid-search g varies wildly (0.78-9.0 across
    model variants / per-log subsets, see report) and is not trusted.
    周波数領域の重み付き最小二乗フィットに依存しない単純なDCゲイン検算
    （位相・遅れに一切依存しない）。2bのグリッドサーチgが変動大（0.78～9.0）で
    信用できないため、gの妥当性チェックに使う。"""
    imu = d["imu"]
    t = imu["ts"]
    accel_corr = bias_corrected_accel(imu)
    f_up_native, cos_native = body_accel_to_f_up(accel_corr, imu["quat"])
    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]
    t0, t1 = flight_window
    t0, t1 = t0 + 3.0, t1 - 3.0
    mi = (t >= t0) & (t <= t1)
    mc = (tcr >= t0) & (tcr <= t1)
    cos_on_tcr = np.interp(tcr[mc], t, cos_native)
    u_mean = float(np.mean(thr[mc] * cos_on_tcr))
    y_mean = float(np.mean(M * f_up_native[mi]))
    return dict(u_eff_mean_N=u_mean, y_mean_N=y_mean,
                dc_gain=(y_mean / u_mean) if u_mean else float("nan"))


# =============================================================================
# Step 2c — time-domain transient delay (takeoff / landing), logs A & B
# ステップ2c — 時間領域トランジェント遅れ（離陸・着陸）
# =============================================================================
def find_step_edge(t_native, x_native, t_approx, search_half, rising, min_jump):
    """Locate the single largest step (biggest 1-sample jump, correct sign) in
    `x_native` within +-search_half of t_approx. Returns (t_before, t_after,
    jump) straddling the two 50Hz ctrl_ref samples of the edge, or None if no
    jump exceeds min_jump (e.g. log B's recording simply ENDS mid-descent with
    no captured landing cutoff -- verified by inspection, see report).
    t_approx近傍で最大の単発ジャンプ（符号一致）を探す。B着陸のように録画が着陸前に
    切れていてカットオフが存在しない場合はNoneを返す（実データで確認済み）。"""
    mask = (t_native >= t_approx - search_half) & (t_native <= t_approx + search_half)
    idx_all = np.where(mask)[0]
    if len(idx_all) < 3:
        return None
    tt, xx = t_native[idx_all], x_native[idx_all]
    d = np.diff(xx)
    i = int(np.argmax(d)) if rising else int(np.argmin(d))
    if abs(d[i]) < min_jump:
        return None
    return float(tt[i]), float(tt[i + 1]), float(d[i])


def plateau_crossing(t, x, t_edge, pre_win, post_win, rising):
    """50%-amplitude crossing time using LOCAL before/after plateau levels
    (median over [t_edge-pre_win, t_edge) and (t_edge, t_edge+post_win]),
    restricted to a TIGHT window around the located edge. This avoids the
    failure mode of a single global min/max threshold over a wide window
    picking up an unrelated earlier oscillation (POS_HOLD altitude bob) as
    the "crossing" -- which is what a naive whole-window threshold does.
    局所的な前後プラトー中央値から閾値を作り、局所化した窓内でのみ交差時刻を
    探す。広い窓で全体min/maxから閾値を作ると、無関係な早い振動（高度ボブ）を
    誤って「交差」と検出してしまう失敗モードを避ける。"""
    before = (t >= t_edge - pre_win) & (t < t_edge - 0.02)
    after = (t > t_edge + 0.02) & (t <= t_edge + post_win)
    if before.sum() < 2 or after.sum() < 2:
        return None
    lo_level = np.median(x[before]) if rising else np.median(x[after])
    hi_level = np.median(x[after]) if rising else np.median(x[before])
    thr = 0.5 * (lo_level + hi_level)
    win = (t >= t_edge - pre_win) & (t <= t_edge + post_win)
    tt, xx = t[win], x[win]
    if rising:
        idx = np.where((xx[:-1] < thr) & (xx[1:] >= thr))[0]
    else:
        idx = np.where((xx[:-1] > thr) & (xx[1:] <= thr))[0]
    if len(idx) == 0:
        return None
    i = idx[0]
    t0, t1, x0, x1 = tt[i], tt[i + 1], xx[i], xx[i + 1]
    frac_i = 0.0 if x1 == x0 else (thr - x0) / (x1 - x0)
    return float(t0 + frac_i * (t1 - t0))


def moving_average(x, w):
    w = max(1, int(w))
    if w <= 1:
        return x.copy()
    kernel = np.ones(w) / w
    return np.convolve(x, kernel, mode="same")


def transient_event(d, t_event_approx, rising, label=""):
    """Estimate the u->y delay around one step event (takeoff or landing) via
    (i) a locally-thresholded 50%-crossing-time difference and (ii) a
    cross-correlation peak, both restricted to a TIGHT window around the
    actual located edge (see find_step_edge/plateau_crossing docstrings for
    why a wide window fails). Both operate on u_eff=thrust*cos(tilt) [N] and
    y=M*f_up [N] so the overlay plot is directly interpretable in force units.

    Landing (rising=False) uses a SHORTER post-edge window than takeoff:
    after full thrust cutoff the airframe free-falls then hits the ground
    (see the raw f_up trace ~150ms after an observed cutoff: violent
    +-2 m/s^2 impact/bounce spikes), so a long post-window captures impact
    noise, not the actuation lag. This was confirmed empirically (log A
    landing): post_win=0.6s -> delay=-337ms (non-causal, contaminated by the
    ~160ms-later impact); post_win=0.25s -> delay=+17ms (physically sane).
    着陸(rising=False)は離陸より短い事後窓を使う: 全推力カットオフ後は自由落下→
    着地衝撃（カットオフ後約150msでf_upに±2m/s^2級の激しい衝撃/バウンドスパイク、
    実データで確認）に入るため、長い事後窓は衝撃ノイズを遅れとして誤検出する
    （ログA着陸で実証: post_win=0.6s→delay=-337ms(非因果、衝撃混入)、
    post_win=0.25s→delay=+17ms(物理的に妥当)）。
    """
    imu = d["imu"]
    tim = imu["ts"]
    accel_corr = bias_corrected_accel(imu)
    f_up_native, cos_native = body_accel_to_f_up(accel_corr, imu["quat"])
    y_native = M * f_up_native
    y_native_smooth = moving_average(y_native, 8)   # ~20ms smoothing @400Hz

    tcr = d["ctrl_ref"]["ts"]
    thr = d["ctrl_ref"]["total_thrust"]
    cos_on_tcr = np.interp(tcr, tim, cos_native)
    u_native_on_tcr = thr * cos_on_tcr

    edge = find_step_edge(tcr, thr, t_event_approx, search_half=3.0, rising=rising,
                           min_jump=0.1 * HOVER)
    if edge is None:
        return None
    t_before, t_after, jump = edge
    t_edge = 0.5 * (t_before + t_after)

    pre_win = 0.4
    post_win = 0.6 if rising else 0.25
    t_u_cross = plateau_crossing(tcr, u_native_on_tcr, t_edge, pre_win, post_win, rising)
    t_y_cross = plateau_crossing(tim, y_native_smooth, t_edge, pre_win, post_win, rising)
    delay_crossing = (t_y_cross - t_u_cross) if (t_u_cross is not None and t_y_cross is not None) else None

    # Cross-correlation over positive lag in [0, min(0.3s, post_win)]: u ZOH'd
    # onto the imu grid, restricted to the SAME tight [t_edge-pre_win, t_edge+post_win]
    # window (avoids the landing impact contamination described above).
    win_lo, win_hi = t_edge - pre_win, t_edge + post_win
    m_im = (tim >= win_lo) & (tim <= win_hi)
    m_cr = (tcr >= win_lo) & (tcr <= win_hi)
    u_on_imu = zoh_interp(tim[m_im], tcr, thr) * cos_native[m_im]
    y_on_imu = y_native[m_im]
    dt = 1.0 / 400.0
    max_lag = min(int(round(0.3 / dt)), int(round(post_win / dt)))
    u_c = u_on_imu - np.mean(u_on_imu)
    y_c = y_on_imu - np.mean(y_on_imu)
    best_lag, best_corr = 0, -np.inf
    for lag in range(0, max_lag):
        if lag == 0:
            a, b = u_c, y_c
        else:
            a, b = u_c[:-lag], y_c[lag:]
        if len(a) < 20:
            break
        denom = (np.linalg.norm(a) * np.linalg.norm(b))
        corr = float(np.dot(a, b) / denom) if denom > 0 else 0.0
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    delay_xcorr = best_lag * dt

    # Wider plot window (for visual context beyond the tight fit window).
    plot_lo, plot_hi = t_edge - 0.5, t_edge + max(1.0, post_win + 0.5)
    m_im_plot = (tim >= plot_lo) & (tim <= plot_hi)
    m_cr_plot = (tcr >= plot_lo) & (tcr <= plot_hi)

    return dict(
        label=label, t_event_approx=float(t_event_approx), t_edge=float(t_edge),
        edge_jump_N=float(jump), fit_window_s=[float(win_lo), float(win_hi)],
        t_u_cross=t_u_cross, t_y_cross=t_y_cross,
        delay_crossing_s=delay_crossing, delay_xcorr_s=float(delay_xcorr),
        xcorr_peak=float(best_corr),
        plot_data=dict(t_win=(plot_lo, plot_hi),
                        t_u=tcr[m_cr_plot].tolist(), u=u_native_on_tcr[m_cr_plot].tolist(),
                        t_y=tim[m_im_plot].tolist(), y=y_native[m_im_plot].tolist()),
    )


# =============================================================================
# Step 3 — noise PSD of f_up during hover + DOB Q-filter noise budget
# ステップ3 — ホバー中f_upのPSD + DOB Qフィルタのノイズ見積り
# =============================================================================
def step3_noise_budget(d, flight_window):
    imu = d["imu"]
    t = imu["ts"]
    accel_corr = bias_corrected_accel(imu)
    f_up_native, _ = body_accel_to_f_up(accel_corr, imu["quat"])

    t0, t1 = flight_window
    t0, t1 = t0 + 3.0, t1 - 3.0   # trim takeoff/landing transients
    m = (t >= t0) & (t <= t1)
    fs_native = 1.0 / np.median(np.diff(t[m]))
    gap_frac = float(np.mean(np.diff(t[m]) > 0.01))

    nperseg = min(int(round(20.0 * fs_native)), int(m.sum()))
    f, Pxx = welch(f_up_native[m], fs=fs_native, nperseg=nperseg)

    def band_rms(flo, fhi):
        band = (f >= flo) & (f <= fhi)
        if band.sum() < 2:
            return float("nan")
        return float(np.sqrt(np.trapezoid(Pxx[band], f[band])))

    bands = dict(
        lt2Hz=band_rms(0.05, 2.0),
        f2_20Hz=band_rms(2.0, 20.0),
        f20_200Hz=band_rms(20.0, 200.0),
    )

    # Pure sensor-noise floor, extrapolated from the 20-100Hz region where real
    # airframe/bob dynamics should be negligible (BMI270 accel noise + prop/motor
    # vibration coupling dominate there). This gives a LOWER-BOUND, noise-only Q
    # estimate to complement the UPPER-BOUND below (which integrates the FULL
    # measured PSD -- including the genuine low-frequency POS_HOLD bob motion
    # that is present because log A's "hover" is a real hands-off flight, not a
    # bench-clamped motors-off test -- through Q, so it conflates true low-freq
    # disturbance signal with sensor noise).
    # 20-100Hz帯（実機の姿勢/ボブ運動は無視できるはずの帯域=BMI270ノイズ+振動結合が
    # 支配的）から外挿した「純粋なセンサノイズ床」。以下の測定PSDそのまま版（実際の
    # POS_HOLDボブという「真の低周波外乱」まで込みで積分する上限値）を補う下限値。
    hf_band = (f >= 20.0) & (f <= 100.0)
    Pxx_floor = float(np.median(Pxx[hf_band])) if hf_band.sum() >= 2 else float(np.median(Pxx))

    fcs = [0.5, 1.0, 1.5, 2.0, 3.0, 5.0]
    noise_table = []
    for fc in fcs:
        wc = 2.0 * np.pi * fc
        w = 2.0 * np.pi * f
        Q2 = 1.0 / (1.0 + (w / wc) ** 4)     # ideal 2nd-order Butterworth |Q(jw)|^2
        var_thrust_full = np.trapezoid((M ** 2) * Pxx * Q2, f)
        var_thrust_floor = np.trapezoid((M ** 2) * Pxx_floor * Q2, f)
        std_full_N = float(np.sqrt(max(var_thrust_full, 0.0)))
        std_floor_N = float(np.sqrt(max(var_thrust_floor, 0.0)))
        noise_table.append(dict(
            fc_hz=fc,
            std_thrust_mN_upper_fullPSD=std_full_N * 1000.0,
            pct_of_hover_upper=std_full_N / HOVER * 100.0,
            std_thrust_mN_lower_noisefloor=std_floor_N * 1000.0,
            pct_of_hover_lower=std_floor_N / HOVER * 100.0,
        ))

    return dict(window_s=[float(t0), float(t1)], fs_native_hz=float(fs_native),
                gap_frac_over_10ms=gap_frac, nperseg=int(nperseg),
                f_hz=f.tolist(), Pxx=Pxx.tolist(), Pxx_noise_floor_mps2_per_Hz=Pxx_floor,
                band_rms_mps2=bands, noise_table=noise_table)


# =============================================================================
# Step 4 — accel-based d_ext vs. existing kinematic d_ext (all 4 logs)
# ステップ4 — 加速度ベースd_ext vs 既存の運動学ベースd_ext（全4ログ）
# =============================================================================
def causal_delay_lag(x, dt, L, T, g):
    """Apply pure delay L (integer-sample shift, edge-padded) then a causal
    first-order lag of time constant T (state reset at index 0 of the ARRAY
    passed in -- caller must pass one gap-free segment), gain g."""
    shift = int(round(L / dt))
    if shift > 0:
        xd = np.concatenate([np.full(shift, x[0]), x[:-shift]])
    else:
        xd = x.copy()
    if T <= 1e-6:
        return g * xd
    a = np.exp(-dt / T)
    y = np.zeros_like(xd)
    y[0] = xd[0]
    for i in range(1, len(xd)):
        y[i] = a * y[i - 1] + (1 - a) * xd[i]
    return g * y


def extend_log_with_fup(path, win, gap_thr=avs.GAP_THR, min_seg=avs.MIN_SEG):
    """Load a log via alt_ti_venue_sim.load_log() (FS=100 gap-aware grid, EXACT
    reuse of the existing kinematic-method infrastructure), then attach fupg =
    f_up interpolated onto the SAME tg grid (bias-corrected, full-DCM,
    anti-alias filtered before decimation -- see antialias_lowpass())."""
    L = avs.load_log(str(path), win)
    d = load_jsonl(str(path))
    imu = d["imu"]
    accel_corr = bias_corrected_accel(imu)
    f_up_native, _ = body_accel_to_f_up(accel_corr, imu["quat"])
    fs_imu_native = 1.0 / np.median(np.diff(imu["ts"]))
    f_up_native = antialias_lowpass(f_up_native, fs_imu_native, cutoff_hz=0.8 * (avs.FS / 2.0))
    L["fupg"] = np.interp(L["tg"], imu["ts"], f_up_native)
    return L


def dext_acc_for_segment(L, s, e, model):
    """d_ext_acc = M*f_up - u_eff_lagged, u_eff_lagged from the IDENTIFIED
    (L,T,g) actuation model applied causally within this one gap-free segment."""
    fup_seg = L["fupg"][s:e + 1]
    thr_seg = L["thrg"][s:e + 1]
    cos_seg = L["cosg"][s:e + 1]
    u_eff_seg = thr_seg * cos_seg
    dt = 1.0 / avs.FS
    u_lagged = causal_delay_lag(u_eff_seg, dt, model["L"], model["T"], model["g"])
    y_seg = M * fup_seg
    return y_seg - u_lagged


def band_rms_from_series_segments(per_seg_series, fs, flo, fhi, min_n=8):
    """Duration-weighted band RMS from a list of (tg_seg, x_seg) — each Welch'd
    SEPARATELY (never concatenated across a gap), matching
    alt_ti_venue_sim.band_power_welch's anti-leakage discipline."""
    f_common = None
    Pxx_sum = None
    dur_sum = 0.0
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
        return float("nan"), None, None
    Pxx_avg = Pxx_sum / dur_sum
    band = (f_common >= flo) & (f_common <= fhi)
    rms = float(np.sqrt(np.trapezoid(Pxx_avg[band], f_common[band]))) if band.sum() >= 2 else float("nan")
    return rms, f_common, Pxx_avg


def coherence_band_from_segments(per_seg_pairs, fs, flo, fhi, min_n=8):
    specs = []
    for tg_seg, a_seg, b_seg in per_seg_pairs:
        n = len(a_seg)
        if n < min_n:
            continue
        nper = min(n, max(min_n, int(fs * 10)))
        f, coh = scipy_coherence(a_seg, b_seg, fs=fs, nperseg=nper)
        specs.append((f, coh, n / fs))
    if not specs:
        return float("nan")
    f_common = specs[0][0]
    coh_sum = np.zeros(len(f_common))
    dur_sum = 0.0
    for f, coh, dur in specs:
        coh_i = np.interp(f_common, f, coh)
        coh_sum += coh_i * dur
        dur_sum += dur
    coh_avg = coh_sum / dur_sum
    band = (f_common >= flo) & (f_common <= fhi)
    return float(np.mean(coh_avg[band])) if band.sum() else float("nan")


def step4_dext_cross_check(logs_win, model):
    results = {}
    for name, (path, win) in logs_win.items():
        L = extend_log_with_fup(path, win)
        d_kin_all, d_acc_all, t_all = [], [], []
        per_seg_kin, per_seg_acc, per_seg_pairs = [], [], []
        for s, e in L["segs"]:
            tg_seg, d_kin_seg, _noise = avs.dext_for_segment(L, s, e)
            d_acc_seg = dext_acc_for_segment(L, s, e, model)
            d_kin_all.append(d_kin_seg)
            d_acc_all.append(d_acc_seg)
            t_all.append(tg_seg)
            per_seg_kin.append((tg_seg, d_kin_seg))
            per_seg_acc.append((tg_seg, d_acc_seg))
            per_seg_pairs.append((tg_seg, d_kin_seg, d_acc_seg))
        if not d_kin_all:
            results[name] = dict(error="no valid segments")
            continue
        d_kin = np.concatenate(d_kin_all)
        d_acc = np.concatenate(d_acc_all)
        t_cat = np.concatenate(t_all)
        coverage_s = float(sum((e - s) / avs.FS for s, e in L["segs"]))

        rms_kin_band, _, _ = band_rms_from_series_segments(per_seg_kin, avs.FS, 0.02, 1.0)
        rms_acc_band, _, _ = band_rms_from_series_segments(per_seg_acc, avs.FS, 0.02, 1.0)
        coh_band = coherence_band_from_segments(per_seg_pairs, avs.FS, 0.02, 1.0)

        _f, _Pxx, (blo, bhi) = avs.band_power_welch([(tg, d, None) for tg, d in per_seg_acc])

        # NOTE: report BOTH std (fluctuation, mean-removed -- what the task asks
        # for) and rms=sqrt(mean(x^2)) (includes any DC bias). d_ext often has a
        # non-negligible DC component (thrust-calibration/trim error, battery
        # sag -- see poshold_alt_loop_sim.py docstring), so the two differ
        # materially (verified: log C d_ext_kin has mean=-38.0mN, so
        # rms=51.8mN != std=35.2mN). The historical "night 8.4/home 19.7/venue
        # 52-57mN" reference values in project memory are RMS (confirmed by
        # reproducing alt_ti_venue_sim.py's own printed numbers bit-for-bit
        # for logs C/D: 51.8/57.3mN RMS here matches "52-57mN" exactly, while
        # std would read as only 35.2/41.6mN and look like a mismatch).
        # std（変動分、平均除去 — タスクの要求指標）とrms=sqrt(mean(x^2))
        # （DCバイアス込み）の両方を報告する。d_extは無視できないDC成分を持つこと
        # が多く（推力校正/トリム誤差、バッテリ電圧降下）、両者は大きく異なる
        # （実測: ログCのd_ext_kinはmean=-38.0mNのためrms=51.8mN≠std=35.2mN）。
        # メモリの「夜間8.4/自宅19.7/会場52-57mN」はRMSであることを確認済み
        # （alt_ti_venue_sim.py自身の出力51.8/57.3mNとビット一致で再現）。
        rms_kin = float(np.sqrt(np.mean(d_kin ** 2)) * 1000.0)
        rms_acc = float(np.sqrt(np.mean(d_acc ** 2)) * 1000.0)
        results[name] = dict(
            coverage_s=coverage_s, window=list(win),
            n_segments=len(L["segs"]),
            d_ext_kin_std_mN=float(np.std(d_kin) * 1000.0),
            d_ext_acc_std_mN=float(np.std(d_acc) * 1000.0),
            d_ext_kin_rms_mN=rms_kin, d_ext_acc_rms_mN=rms_acc,
            d_ext_kin_mean_mN=float(np.mean(d_kin) * 1000.0),
            d_ext_acc_mean_mN=float(np.mean(d_acc) * 1000.0),
            band_0p02_1Hz_rms_kin_mN=rms_kin_band * 1000.0,
            band_0p02_1Hz_rms_acc_mN=rms_acc_band * 1000.0,
            band_ratio_acc_over_kin=(rms_acc_band / rms_kin_band) if rms_kin_band else float("nan"),
            coherence_0p02_1Hz=coh_band,
            d_ext_acc_80pct_band_hz=[float(blo), float(bhi)],
            t=t_cat, d_kin=d_kin, d_acc=d_acc,   # kept for plotting, stripped before JSON dump
        )
    return results


# =============================================================================
# Plots
# 図の生成
# =============================================================================
def plot_bode_coherence(per_log, combined, model_fit):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    colors = dict(A="tab:blue", B="tab:orange")
    for name, comb in per_log.items():
        if comb is None:
            continue
        f = comb["f"]
        m = f > 0.02
        ax1.semilogx(f[m], 20 * np.log10(np.abs(comb["H"][m]) + 1e-12),
                      color=colors.get(name, None), label=f"log {name}", alpha=0.8)
        ax2.semilogx(f[m], np.degrees(np.angle(comb["H"][m])),
                      color=colors.get(name, None), alpha=0.8)
        ax3.semilogx(f[m], comb["coherence"][m], color=colors.get(name, None), alpha=0.8)
    f = combined["f"]
    m = f > 0.02
    ax1.semilogx(f[m], 20 * np.log10(np.abs(combined["H"][m]) + 1e-12), "k-", lw=2, label="combined A+B")
    ax2.semilogx(f[m], np.degrees(np.angle(combined["H"][m])), "k-", lw=2)
    ax3.semilogx(f[m], combined["coherence"][m], "k-", lw=2)

    for key, style, label in [("full", "r--", "full fit"), ("delay_only", "g:", "delay-only"),
                               ("lag_only", "m-.", "lag-only")]:
        mdl = model_fit[key]
        Hm = mdl["g"] * model_shape(f, mdl["L"], mdl["T"])
        ax1.semilogx(f[m], 20 * np.log10(np.abs(Hm[m]) + 1e-12), style,
                      label=f"{label} (L={mdl['L']*1000:.1f}ms,T={mdl['T']*1000:.1f}ms,g={mdl['g']:.2f})")
        ax2.semilogx(f[m], np.degrees(np.angle(Hm[m])), style)

    lo, hi = model_fit["trusted_band_hz"]
    for ax in (ax1, ax2, ax3):
        if np.isfinite(lo):
            ax.axvspan(lo, hi, color="yellow", alpha=0.15)
    ax3.axhline(0.5, color="gray", ls=":", lw=1)

    ax1.set_ylabel("|H| [dB]")
    ax1.legend(fontsize=7, loc="lower left")
    ax1.set_title("Actuation path H(jw) = Suy/Suu : total_thrust*cos(tilt) -> M*f_up\n"
                   "(yellow = coherence>0.5; fit UNRELIABLE -- closed-loop bias, NOT used for step4)")
    ax2.set_ylabel("phase [deg]")
    ax3.set_ylabel("coherence $\\gamma^2$")
    ax3.set_xlabel("frequency [Hz]")
    ax3.set_ylim(0, 1.05)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "bode_coherence.png", dpi=150)
    plt.close(fig)


def plot_transients(events):
    n = len(events)
    fig, axes = plt.subplots(n, 1, figsize=(8, 2.6 * n), squeeze=False)
    for i, ev in enumerate(events):
        ax = axes[i, 0]
        pd_ = ev["plot_data"]
        t0 = ev["t_edge"]
        ax.axvspan(ev["fit_window_s"][0] - t0, ev["fit_window_s"][1] - t0,
                   color="gray", alpha=0.12, label="fit window")
        ax.plot(np.array(pd_["t_u"]) - t0, pd_["u"], "o-", ms=3, label="u_eff = thrust*cos(tilt) [N]", color="tab:red")
        ax.plot(np.array(pd_["t_y"]) - t0, pd_["y"], "-", lw=0.8, label="y = M*f_up [N]", color="tab:blue", alpha=0.8)
        if ev["t_u_cross"] is not None:
            ax.axvline(ev["t_u_cross"] - t0, color="tab:red", ls=":", lw=1)
        if ev["t_y_cross"] is not None:
            ax.axvline(ev["t_y_cross"] - t0, color="tab:blue", ls=":", lw=1)
        dc = ev["delay_crossing_s"]
        dx = ev["delay_xcorr_s"]
        dc_str = f"{dc*1000:.1f}ms" if dc is not None else "n/a"
        ax.set_title(f"{ev['label']}  delay(crossing)={dc_str}  delay(xcorr)={dx*1000:.1f}ms "
                      f"(xcorr peak={ev['xcorr_peak']:.2f})")
        ax.set_xlabel("t - t_event [s]")
        ax.set_ylabel("force [N]")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "transients.png", dpi=150)
    plt.close(fig)


def plot_noise_psd(noise_result):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    f = np.array(noise_result["f_hz"])
    Pxx = np.array(noise_result["Pxx"])
    ax.loglog(f, Pxx)
    ax.set_xlim(0.05, 200)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("PSD of f_up [$(m/s^2)^2$/Hz]")
    ax.set_title("f_up PSD during hover (log A, 400Hz native)")
    ax.grid(which="both", alpha=0.3)
    txt = "\n".join([f"{k}: {v*1000:.1f} mm/s^2 RMS" for k, v in noise_result["band_rms_mps2"].items()])
    ax.text(0.02, 0.05, txt, transform=ax.transAxes, fontsize=8,
            bbox=dict(boxstyle="round", fc="white", alpha=0.8))
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "noise_psd.png", dpi=150)
    plt.close(fig)


def plot_dext_comparison(step4_results):
    names = [n for n, r in step4_results.items() if "t" in r]
    fig, axes = plt.subplots(len(names), 1, figsize=(9, 2.8 * len(names)), squeeze=False)
    for i, name in enumerate(names):
        r = step4_results[name]
        t, dk, da = r["t"], r["d_kin"], r["d_acc"]
        # representative 60s window: middle of the longest contiguous run
        dt = np.diff(t)
        breaks = np.where(dt > 1.5 / avs.FS)[0]
        idx_groups = np.split(np.arange(len(t)), breaks + 1) if len(breaks) else [np.arange(len(t))]
        longest = max(idx_groups, key=len)
        seg_t, seg_k, seg_a = t[longest], dk[longest], da[longest]
        c = seg_t[0] + (seg_t[-1] - seg_t[0]) / 2
        m = (seg_t >= c - 30) & (seg_t <= c + 30)
        ax = axes[i, 0]
        ax.plot(seg_t[m], seg_k[m] * 1000, label="d_ext_kin (existing, velocity-based)", lw=0.9)
        ax.plot(seg_t[m], seg_a[m] * 1000, label="d_ext_acc (new, accel-based)", lw=0.9, alpha=0.8)
        # Actual plotted span -- for the gappy venue logs (C/D) the longest
        # single gap-free segment can be well under 60s, so the title reports
        # the REAL duration shown rather than a hardcoded "60s" claim.
        # 実際に描画した区間長を表示する（会場ログC/Dはギャップだらけで最長の
        # ギャップなし区間が60s未満のことがあるため、固定の「60s」表記は書かない）。
        span_s = seg_t[m][-1] - seg_t[m][0] if m.sum() > 1 else 0.0
        ax.set_title(f"{name}: d_ext comparison, longest gap-free segment (showing {span_s:.0f}s here) "
                      f"(std kin={np.std(dk)*1000:.1f}mN acc={np.std(da)*1000:.1f}mN)")
        ax.set_ylabel("d_ext [mN]")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    axes[-1, 0].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(FIGS_DIR / "dext_comparison.png", dpi=150)
    plt.close(fig)


# =============================================================================
# Main
# =============================================================================
def main():
    print("=" * 100)
    print("STEP 1: f_up sign / rotation convention sanity check (log A)")
    print("=" * 100)
    dA = load_jsonl(str(LOG_A))
    sanity, flightA = step1_sanity_check_f_up(dA)
    for k, v in sanity.items():
        print(f"  {k}: {v}")

    print()
    print("=" * 100)
    print("STEP 2a: frequency-domain H(jw)=Suy/Suu + coherence (logs A, B)")
    print("=" * 100)
    dB = load_jsonl(str(LOG_B))
    flightB = detect_flight_window(dB["ctrl_ref"]["ts"], dB["ctrl_ref"]["motor_duty"])
    print(f"  flight window A = {flightA}  B = {flightB}")

    gridA = build_id_grid(dA, flightA)
    gridB = build_id_grid(dB, flightB)
    print(f"  grid A: n_segments={len(gridA['segs'])} "
          f"coverage={sum(e - s for s, e in gridA['segs']) / FS_ID:.1f}s")
    print(f"  grid B: n_segments={len(gridB['segs'])} "
          f"coverage={sum(e - s for s, e in gridB['segs']) / FS_ID:.1f}s")

    per_log, combined = step2a_frequency_id({"A": gridA, "B": gridB})

    print()
    print("=" * 100)
    print("STEP 2b: model fit H(s) = g*exp(-L s)/(T s + 1)  -- combined A+B, and A-only/B-only")
    print("=" * 100)
    model_fit = step2b_model_fit(combined)
    fit_A_only = step2b_model_fit(per_log["A"])
    fit_B_only = step2b_model_fit(per_log["B"])
    for tag, fit in [("combined A+B", model_fit), ("A-only", fit_A_only), ("B-only", fit_B_only)]:
        print(f"  [{tag}] trusted band (coherence>{fit['band_mask_threshold']}): "
              f"{fit['trusted_band_hz']}  ({fit['n_freq_bins_used']} bins)")
        for key in ("full", "delay_only", "lag_only"):
            mm = fit[key]
            print(f"    {key:12s}: L={mm['L']*1000:6.2f}ms  T={mm['T']*1000:6.2f}ms  "
                  f"g={mm['g']:.3f}  resid_frac={mm['resid_frac']:.4f}")
    print("  RELIABILITY: g and the trusted band disagree strongly between A-only/B-only/combined\n"
          "  (g ranges ~0.8-9x; |H| RISES with frequency in places instead of rolling off), and\n"
          "  resid_frac stays >0.7 even in the 'trusted' (coherence>0.5) band for every variant.\n"
          "  This is the closed-loop identification bias the task brief warned about for <0.5Hz --\n"
          "  empirically it contaminates the WHOLE examined band (0.02-15Hz) for this hover data,\n"
          "  not just the low-frequency d_ext-dominated region. The Welch H1 frequency-domain fit\n"
          "  is NOT used for step 4; see the DC-gain check and step 2c time-domain delay below.")

    dcA = dc_gain_check(dA, flightA)
    dcB = dc_gain_check(dB, flightB)
    print(f"  DC-gain cross-check (mean M*f_up / mean u_eff, delay-insensitive): "
          f"A={dcA['dc_gain']:.3f}  B={dcB['dc_gain']:.3f}  (both close to 1 -> thrust command is a"
          f" good absolute Newton estimate; grid-search g above is the unreliable one, not the DC level)")

    plot_bode_coherence(per_log, combined, model_fit)

    print()
    print("=" * 100)
    print("STEP 2c: time-domain transient delay (takeoff / landing, logs A & B)")
    print("=" * 100)
    events = []
    for name, d_, (t0, t1) in [("A", dA, flightA), ("B", dB, flightB)]:
        ev_up = transient_event(d_, t0, rising=True, label=f"{name} takeoff")
        ev_dn = transient_event(d_, t1, rising=False, label=f"{name} landing")
        for ev in (ev_up, ev_dn):
            if ev is not None:
                events.append(ev)
                dc = ev["delay_crossing_s"]
                dc_str = f"{dc*1000:.1f}ms" if dc is not None else "n/a"
                print(f"  {ev['label']:14s} delay(crossing)={dc_str:>8s}  "
                      f"delay(xcorr)={ev['delay_xcorr_s']*1000:5.1f}ms  xcorr_peak={ev['xcorr_peak']:.2f}")
    plot_transients(events)

    print()
    print("=" * 100)
    print("STEP 3: f_up noise PSD during hover (log A) + DOB Q-filter noise budget")
    print("=" * 100)
    noise_result = step3_noise_budget(dA, flightA)
    print(f"  window={noise_result['window_s']}  fs_native={noise_result['fs_native_hz']:.2f}Hz "
          f"gap_frac(>10ms)={noise_result['gap_frac_over_10ms']*100:.3f}%")
    for k, v in noise_result["band_rms_mps2"].items():
        print(f"  band {k}: RMS = {v*1000:.2f} mm/s^2")
    print(f"  noise floor (median PSD, 20-100Hz) = {noise_result['Pxx_noise_floor_mps2_per_Hz']:.4e} (m/s^2)^2/Hz")
    print("  NOTE: log A's 'hover' is a real hands-off POS_HOLD flight (altitude bob included), not a\n"
          "  bench/motors-off test, so the FULL-PSD column below conflates true low-frequency\n"
          "  disturbance signal with sensor+vibration noise (upper bound). The NOISE-FLOOR column\n"
          "  extrapolates the 20-100Hz (dynamics-free) level flat across all f (lower bound, closer\n"
          "  to 'pure accelerometer/vibration noise').")
    print(f"  {'fc[Hz]':>8s} {'upper(fullPSD)[mN]':>20s} {'%hover':>8s} {'lower(floor)[mN]':>20s} {'%hover':>8s}")
    for row in noise_result["noise_table"]:
        print(f"  {row['fc_hz']:8.1f} {row['std_thrust_mN_upper_fullPSD']:20.2f} "
              f"{row['pct_of_hover_upper']:8.2f} {row['std_thrust_mN_lower_noisefloor']:20.2f} "
              f"{row['pct_of_hover_lower']:8.2f}")
    plot_noise_psd(noise_result)

    print()
    print("=" * 100)
    print("STEP 4: accel-based d_ext vs. existing kinematic d_ext (logs A, B, C, D)")
    print("=" * 100)
    # Model used for u_eff_lagged: per the task brief's own caveat ("distrust
    # closed-loop H below ~0.5Hz; time-domain events are the primary reference
    # for pure delay L"), and given step 2b's frequency-domain fit is flagged
    # UNRELIABLE above (weak/inconsistent coherence across the whole examined
    # band, not just <0.5Hz), the model actually used here is built from:
    #   L = mean of the two clean, mutually-consistent TAKEOFF delay estimates
    #       (A=63.6ms, B=61.2ms -- large step, high SNR, no free-fall/impact
    #       contamination, unlike the landing events)
    #   T = TAU_M_REF (0.02s, tools/sysid/defaults.py motor-time-constant
    #       reference) -- NOT empirically fit here, just the independently
    #       measured motor/ESC/prop time constant, used as a physically-motivated
    #       (not data-fit) lag on top of the time-domain L
    #   g = 1.0 -- matches d_ext_kin's own convention (existing
    #       alt_ti_venue_sim.dext_for_segment takes thrust at face value, no
    #       gain correction), so the two d_ext definitions stay directly
    #       comparable in step 4 (a different g here would just show up as a
    #       spurious DC offset between the two methods, not a real disagreement)
    # 遅れLは離陸イベント（大振幅・高SNR・自由落下や着地衝撃の混入なし）の平均、
    # 遅れTはtools/sysid/defaults.pyのモータ時定数実測値（データフィットでなく
    # 独立測定値）、g=1は既存d_ext_kinの流儀（推力を額面通り使う）に合わせた
    # ——2b の周波数領域フィットは上で「信用できない」と結論しているため使わない。
    L_takeoff = [ev["delay_crossing_s"] for ev in events
                 if "takeoff" in ev["label"] and ev["delay_crossing_s"] is not None]
    L_time_domain = float(np.mean(L_takeoff)) if L_takeoff else TAU_M_REF
    model = dict(L=L_time_domain, T=TAU_M_REF, g=1.0,
                 source="time-domain takeoff delay (mean of A,B) + TAU_M_REF lag + g=1 "
                         "(freq-domain fit not used, see step 2b reliability note)")
    print(f"  model for u_eff_lagged (NOT the step-2b freq-domain fit -- see reliability note above): "
          f"L={model['L']*1000:.1f}ms (mean of takeoff events) "
          f"T={model['T']*1000:.1f}ms (TAU_M_REF) g={model['g']:.3f}")
    logs_win = {
        "A": (LOG_A, flightA),
        "B": (LOG_B, flightB),
        "C": (LOG_C, WIN_C),
        "D": (LOG_D, WIN_D),
    }
    step4_results = step4_dext_cross_check(logs_win, model)
    print(f"  {'log':4s} {'kin std':>8s} {'acc std':>8s} {'kin rms':>8s} {'acc rms':>8s} "
          f"{'band ratio':>10s} {'coh 0.02-1Hz':>12s} {'acc 80% band[Hz]':>18s}  (mN unless noted)")
    for name, r in step4_results.items():
        if "error" in r:
            print(f"  {name:4s} ERROR: {r['error']}")
            continue
        print(f"  {name:4s} {r['d_ext_kin_std_mN']:8.1f} {r['d_ext_acc_std_mN']:8.1f} "
              f"{r['d_ext_kin_rms_mN']:8.1f} {r['d_ext_acc_rms_mN']:8.1f} "
              f"{r['band_ratio_acc_over_kin']:10.2f} {r['coherence_0p02_1Hz']:12.2f} "
              f"[{r['d_ext_acc_80pct_band_hz'][0]:.2f},{r['d_ext_acc_80pct_band_hz'][1]:.2f}]")
    print("  (rms=sqrt(mean(x^2)) includes DC bias; std removes it. Historical 'night 8.4/home 19.7/\n"
          "  venue 52-57mN' reference values are RMS -- see code comment in step4_dext_cross_check.)")
    plot_dext_comparison(step4_results)

    # ------------------------------------------------------------------- dump
    def strip_for_json(r):
        return {k: v for k, v in r.items() if k not in ("t", "d_kin", "d_acc")}

    results_json = dict(
        constants=dict(M=M, G=G, HOVER=HOVER, HOVER_CORR=HOVER_CORR,
                       TAU_COMBINED_REF=TAU_COMBINED_REF, TAU_M_REF=TAU_M_REF),
        step1_sanity=sanity,
        step2a_flight_windows=dict(A=list(flightA), B=list(flightB)),
        step2b_model_fit_combined=model_fit,
        step2b_model_fit_A_only=fit_A_only,
        step2b_model_fit_B_only=fit_B_only,
        step2b_reliability="UNRELIABLE -- see stdout reliability note; not used for step 4",
        step2b_dc_gain_check=dict(A=dcA, B=dcB),
        step2c_transients=[{k: v for k, v in ev.items() if k != "plot_data"} for ev in events],
        step2c_L_time_domain_mean_takeoff_s=L_time_domain,
        step3_noise_budget={k: v for k, v in noise_result.items() if k not in ("f_hz", "Pxx")},
        step3_noise_budget_psd_summary="full f_hz/Pxx arrays omitted from JSON for size; see figs/noise_psd.png",
        step4_model_used=model,
        step4_dext_cross_check={name: strip_for_json(r) for name, r in step4_results.items()},
    )
    with open(RESULTS_PATH, "w") as f:
        json.dump(results_json, f, indent=2, default=lambda o: float(o) if isinstance(o, np.floating) else str(o))
    print()
    print(f"Results written to {RESULTS_PATH}")
    print(f"Figures written to {FIGS_DIR}")


if __name__ == "__main__":
    main()
