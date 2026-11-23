"""
Vertical thrust-excitation identification (backend of `sf sysid thrust-fit`)
鉛直の推力励振の同定（`sf sysid thrust-fit` のバックエンド）

What this measures / 測るもの:
  1. G(s) = K exp(-L s) / (tau s + 1): total thrust command [N] -> upward
     acceleration [m/s^2] (K ~ 1/mass, tau: motor+propeller lag, L: dead time).
     総推力の指令 [N] -> 上向きの加速度 [m/s^2]（K は質量の逆数に近い値、tau はモータと
     プロペラの遅れ、L はむだ時間）。
  2. The delay of the downward ToF (distance sensor) RELATIVE TO the accelerometer.
     下向き ToF（距離センサ）の、加速度計に対する相対的な遅れ。
  3. The in-flight accelerometer noise (power spectral density, band RMS, floor).
     飛行中の加速度計の雑音（電力スペクトル密度・帯域ごとの実効値・床の値）。

Why the excitation r is the reference / 励振 r を基準にする理由:
  The altitude loop is closed, so noise on the acceleration comes back into the thrust u.
  The plain ratio S_ua / S_uu is then biased. The excitation r is known and does not depend
  on that noise, so G = S_ra / S_ru (instrumental-variable ratio) is not biased.
  Because the eight tones are multiples of 0.1 Hz, S_ra / S_ru is evaluated tone by tone:
  fit cos, sin (and a polynomial trend) of every tone to each signal by least squares
  and divide the complex amplitudes. Least squares also accepts irregular sample times
  (the ToF samples), which is why no zero-order-hold resampling is used.
  高さのループが閉じているので、加速度の雑音がループを回って推力 u に戻り、単純な比
  S_ua / S_uu は偏る。励振 r は既知でその雑音に依存しないので、G = S_ra / S_ru（操作変数の比）
  は偏らない。8 本の音は 0.1 Hz の整数倍なので、各信号に全ての音の cos・sin（と定数・1 次の項）を
  最小二乗で当てはめ、複素振幅の比を取る。最小二乗は不規則な時刻の標本（ToF）にも使えるので、
  零次保持の補間は使わない（28 Hz の零次保持は約 17 ms の遅れを足す）。

The excitation signal itself is not logged: it is regenerated from the shared table
(tools/sysid/thrust_excitation_table.py, generated from protocol/spec/thrust_excitation.yaml).
励振の信号そのものは記録していない: 基準の表（protocol/spec/thrust_excitation.yaml から生成した
tools/sysid/thrust_excitation_table.py）から再生成する。
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from . import thrust_excitation_table as table
from .loader import load_aligned

# --- Named constants / 名前を付けた定数 -----------------------------------------------------
CYCLE_RATE_HZ = float(table.SAMPLE_RATE_HZ)          # control-cycle rate / 制御周期の周波数
CYCLE_PERIOD_US = 1.0e6 / CYCLE_RATE_HZ              # one control cycle [us] / 制御 1 周期 [us]
GRAVITY_MPS2 = 9.80665                               # [m/s^2] (firmware math::kGravity)
STATE_MASK = 0b111                                   # low 3 bits: state / 下位 3 ビット = 状態
STATE_RAMP_UP = 2                                    # state 2: ramp-up / 立ち上げ
WAVEFORM_SHIFT = 3                                   # bits 3-4: waveform / ビット 3〜4 = 波形
WAVEFORM_MASK = 0b11
WAVEFORM_MULTISINE = 1
WAVEFORM_STEPS = 2
TONE_FREQUENCIES_HZ = tuple(0.1 * index for index in table.TONE_FREQUENCY_INDEX)
START_SEARCH_RADIUS_CYCLES = 12                      # +-12 cycles around the 50 Hz edge / 50 Hz の変わり目の前後
HIGH_PASS_WINDOW_CYCLES = 400                        # 1 s moving average removed / 引く移動平均の長さ（1 s）
DURATION_QUANTUM_CYCLES = 40                         # run lengths are rounded to 0.1 s / 実行の長さは 0.1 s に丸める
DURATION_ROUND_TOLERANCE_CYCLES = 12                 # edge uncertainty (50 Hz) / 変わり目の不確かさ
ABORTED_RAMP_DOWN_CYCLES_MAX = 150                   # state-4 cycles below this = aborted (normal 200, abort 100) / 状態 4 の周期数がこれ未満なら打ち切り
STEADY_MARGIN_CYCLES = 20                            # trimmed beside the ramps (0.05 s) / 立ち上げ・立ち下げの外側の余白
TREND_ORDER = 5                                      # polynomial trend removed with the tones / 音と一緒に引く多項式の次数
NEIGHBOR_OFFSETS_HZ = (-0.3, -0.2, -0.1, 0.1, 0.2, 0.3)
NEIGHBOR_MIN_DISTANCE_HZ = 0.07                      # a neighbor must stay this far from every tone / 音からこれ以上離す
COHERENCE_MIN = 0.8                                  # tones below this are not fitted / これ未満の音は当てはめない
TOF_SNR_MIN = 10.0                                   # tone power / neighbor power, ToF / 音の電力 ÷ 近傍の電力（ToF）
FIT_TAU_STARTS_S = (0.01, 0.03, 0.06)
FIT_DEAD_TIME_START_S = 0.005
FIT_PARAMETER_COUNT = 3
FINITE_DIFFERENCE_RATIO = 1.0e-4
HOVER_WINDOW_LONG_S = 10.0
HOVER_WINDOW_SHORT_S = 5.0
HOVER_GUARD_S = 1.0                                  # stop this long before the standby / 待機の手前で止める
HOVER_HEIGHT_STD_MAX_M = 0.05
NOISE_BANDS_HZ = ((0.4, 1.0), (1.0, 3.0), (3.0, 6.0), (6.0, 9.5))
NOISE_FLOOR_BAND_HZ = (50.0, 150.0)
NOISE_WELCH_SEGMENT_CYCLES = 4000                    # 10 s -> 0.1 Hz resolution / 0.1 Hz の分解能
NOISE_TONE_MASK_HALF_WIDTH_HZ = 0.15
STEP_LEVEL_WINDOW = (0.6, 0.95)                      # of the first half period / 最初の半周期の内側


@dataclass
class ThrustRecord:
    """Arrays of one flight in control-cycle order / 1 回の飛行を制御周期の順に並べた配列"""
    cycle: np.ndarray            # seq (control-cycle number) / 制御周期の通し番号
    thrust: np.ndarray           # ctrl_output.thrust [N] (NaN = missing row) / 欠けた行は NaN
    upward_accel: np.ndarray     # upward acceleration minus gravity [m/s^2] / 重力を引いた上向きの加速度
    tilt_cos: np.ndarray         # cos of the tilt of the body z axis / 機体 z 軸の傾きの余弦
    state_code: np.ndarray       # ctrl_ref thrust_excitation_state (held) / 励振の状態コード（保持）
    tof_cycle: np.ndarray        # ToF sample time on the cycle axis / ToF 標本の時刻（周期の軸）
    tof_height: np.ndarray       # ToF distance x cos(tilt) [m] / ToF の距離 × cos(傾き)
    gap_count: int               # ctrl_output rows missing inside the flight / 飛行中に欠けた行数
    source: str = ""


# =============================================================================
# 1. Loading / 読み込み
# =============================================================================

def upward_acceleration(quaternion: np.ndarray, specific_force: np.ndarray):
    """Upward acceleration minus gravity and cos(tilt), as firmware upwardSpecificForce().
    ファームの upwardSpecificForce() と同じ式で、重力を引いた上向きの加速度と傾きの余弦。

    quaternion: (n, 4) w, x, y, z. specific_force: (n, 3) body-frame accelerometer [m/s^2].
    Returns (upward_acceleration [m/s^2], tilt_cos).
    """
    qw, qx, qy, qz = (quaternion[:, i] for i in range(4))
    r31 = 2.0 * (qx * qz - qw * qy)
    r32 = 2.0 * (qy * qz + qw * qx)
    r33 = 1.0 - 2.0 * (qx * qx + qy * qy)
    upward = -(r31 * specific_force[:, 0] + r32 * specific_force[:, 1] + r33 * specific_force[:, 2])
    return upward - GRAVITY_MPS2, r33


def _extract_tof(df, cycle: np.ndarray, tilt_cos: np.ndarray):
    """ToF samples at their own times (valid status only), tilt-corrected.
    ToF の標本を標本そのものの時刻で取り出す（有効な status のみ）。傾きを補正する。"""
    offset_us = float(np.median(df["timestamp_us"].to_numpy() - cycle * CYCLE_PERIOD_US))
    samples = df.drop_duplicates("tof_bottom_timestamp_us")
    samples = samples[samples["tof_bottom_timestamp_us"].notna() & (samples["status"] == 0)]
    tof_cycle = (samples["tof_bottom_timestamp_us"].to_numpy() - offset_us) / CYCLE_PERIOD_US
    cos_tilt = np.interp(tof_cycle, cycle, tilt_cos)
    return tof_cycle, samples["distance"].to_numpy() * cos_tilt


def load_thrust_record(path) -> ThrustRecord:
    """Read a flight-log bundle into a ThrustRecord. / 記録の一式を ThrustRecord に読む。"""
    df = load_aligned(path)
    cycle = df["seq"].to_numpy(dtype=float)
    quaternion = df[["quat_w", "quat_x", "quat_y", "quat_z"]].to_numpy(dtype=float)
    force = df[["accel_x", "accel_y", "accel_z"]].to_numpy(dtype=float)
    upward, tilt_cos = upward_acceleration(quaternion, force)
    thrust = df["thrust"].to_numpy(dtype=float)
    state = df["thrust_excitation_state"].fillna(0).to_numpy(dtype=int)
    tof_cycle, tof_height = _extract_tof(df, cycle, tilt_cos)
    armed = np.flatnonzero(np.isfinite(thrust))
    gap_count = int(np.isnan(thrust[armed[0]:armed[-1] + 1]).sum()) if armed.size else 0
    return ThrustRecord(cycle, thrust, upward, tilt_cos, state, tof_cycle, tof_height,
                        gap_count, str(path))


# =============================================================================
# 2. Segments and regenerated excitation / 区間と励振の再生成
# =============================================================================

def find_excitation_segments(record: ThrustRecord) -> list:
    """Runs of rows with (state & 7) >= 2: each a dict with the 50 Hz edge rows.
    (状態 & 7) >= 2 の連続した行の区間。50 Hz の変わり目の行を持つ辞書のリスト。"""
    running = (record.state_code & STATE_MASK) >= STATE_RAMP_UP
    edges = np.flatnonzero(np.diff(running.astype(int)))
    starts = list(edges[running[edges + 1]] + 1) if edges.size else []
    if running.size and running[0]:
        starts.insert(0, 0)
    segments = []
    for start in starts:
        stop = start
        while stop < running.size and running[stop]:
            stop += 1
        waveform = int(record.state_code[start] >> WAVEFORM_SHIFT) & WAVEFORM_MASK
        ramp_down_cycles = int(((record.state_code[start:stop] & STATE_MASK) == 4).sum())
        segments.append({"edge_row": int(start), "end_row": int(stop), "waveform": waveform,
                         "length_cycles_edge": int(stop - start),
                         "ramp_down_cycles": ramp_down_cycles})
    return segments


def _half_cosine_rise(step: np.ndarray, steps: int) -> np.ndarray:
    return 0.5 * (1.0 - np.cos(math.pi * step / steps))


def multisine_envelope(total_cycles: int) -> np.ndarray:
    """Envelope of the multisine run (firmware ThrustExcitation::envelope).
    多重正弦波の包絡線（ファームの envelope と同じ）。"""
    cycle = np.arange(total_cycles, dtype=float)
    ramp_up, ramp_down = table.MULTISINE_RAMP_UP_CYCLES, table.MULTISINE_RAMP_DOWN_CYCLES
    gain = np.ones(total_cycles)
    rising = cycle < ramp_up
    gain[rising] = _half_cosine_rise(cycle[rising], ramp_up)
    fall_start = total_cycles - ramp_down
    falling = cycle >= fall_start
    gain[falling] = 1.0 - _half_cosine_rise(cycle[falling] - fall_start + 1.0, ramp_down)
    return gain


def multisine_waveform(total_cycles: int) -> np.ndarray:
    """Unit-peak sum of the eight tones from the shared table. / 基準の表の 8 本の音の和（ピーク 1）。"""
    cycle = np.arange(total_cycles) % table.PERIOD_CYCLES
    total = np.zeros(total_cycles)
    for frequency_index, phase_index in zip(table.TONE_FREQUENCY_INDEX, table.TONE_PHASE_INDEX):
        turns = (frequency_index * cycle + phase_index) % table.PERIOD_CYCLES
        total += np.sin(2.0 * math.pi * turns / table.PERIOD_CYCLES)
    return total * table.MULTISINE_SCALE


def regenerate_multisine(total_cycles: int) -> np.ndarray:
    """Excitation r (unit amplitude, envelope included), index = cycles since the run began.
    励振 r（振幅 1、包絡線込み）。添字は実行の開始からの周期数。"""
    return multisine_waveform(total_cycles) * multisine_envelope(total_cycles)


def estimate_total_cycles(length_cycles_edge: int) -> int:
    """Run length in cycles: the 50 Hz estimate rounded to 0.1 s when it is close to a multiple.
    実行の長さ（周期数）: 50 Hz からの推定値。0.1 s の倍数に近ければ丸める。"""
    rounded = int(round(length_cycles_edge / DURATION_QUANTUM_CYCLES)) * DURATION_QUANTUM_CYCLES
    if abs(rounded - length_cycles_edge) <= DURATION_ROUND_TOLERANCE_CYCLES:
        return rounded
    return length_cycles_edge


def _high_pass(values: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Deviation from a 1 s moving average over valid rows (0 on invalid rows).
    有効な行の 1 s 移動平均からの偏差（無効な行は 0）。"""
    kernel = np.ones(HIGH_PASS_WINDOW_CYCLES)
    filled = np.where(valid, values, 0.0)
    count = np.convolve(valid.astype(float), kernel, mode="same")
    mean = np.convolve(filled, kernel, mode="same") / np.maximum(count, 1.0)
    return np.where(valid, values - mean, 0.0)


def _correlation_at(start_cycle: int, record: ThrustRecord, deviation, valid, reference) -> float:
    index = record.cycle.astype(np.int64) - start_cycle
    inside = valid & (index >= 0) & (index < reference.size)
    r = np.where(inside, reference[np.clip(index, 0, reference.size - 1)], 0.0)
    energy = math.sqrt(float(np.sum(deviation[inside] ** 2) * np.sum(r[inside] ** 2)))
    return float(np.sum(deviation * r) / energy) if energy > 0.0 else 0.0


def search_start_cycle(record: ThrustRecord, edge_row: int, total_cycles: int) -> dict:
    """Pick the run's first cycle among edge +-12 by correlating r with the high-passed thrust.
    変わり目の前後 ±12 周期から、再生成した r と高域通過した推力の相関が最大の位置を選ぶ。"""
    valid = np.isfinite(record.thrust)
    deviation = _high_pass(record.thrust, valid)
    reference = regenerate_multisine(total_cycles)
    edge_cycle = int(record.cycle[edge_row])
    offsets = range(-START_SEARCH_RADIUS_CYCLES, START_SEARCH_RADIUS_CYCLES + 1)
    scores = np.array([_correlation_at(edge_cycle + o, record, deviation, valid, reference)
                       for o in offsets])
    order = np.argsort(scores)[::-1]
    best = int(order[0])
    return {"start_cycle": edge_cycle + best - START_SEARCH_RADIUS_CYCLES,
            "offset_from_edge_cycles": best - START_SEARCH_RADIUS_CYCLES,
            "correlation": float(scores[best]), "correlation_runner_up": float(scores[order[1]]),
            "searched_cycles": int(scores.size)}


# =============================================================================
# 3. Tone amplitudes and coherence / 音ごとの複素振幅とコヒーレンス
# =============================================================================

def neighbor_frequencies(tones_hz) -> np.ndarray:
    """Non-tone frequencies within +-0.3 Hz of the tones (noise estimate). / 音の ±0.3 Hz の音でない周波数。"""
    tones = np.asarray(tones_hz, dtype=float)
    found = set()
    for tone in tones:
        for offset in NEIGHBOR_OFFSETS_HZ:
            candidate = round(tone + offset, 6)
            if candidate > 0.0 and np.min(np.abs(tones - candidate)) >= NEIGHBOR_MIN_DISTANCE_HZ:
                found.add(candidate)
    return np.array(sorted(found))


def _design_matrix(times_s: np.ndarray, frequencies_hz: np.ndarray) -> np.ndarray:
    phase = 2.0 * math.pi * np.outer(times_s, frequencies_hz)
    span = max(float(times_s.max() - times_s.min()), 1e-9)
    scaled = 2.0 * (times_s - times_s.mean()) / span                 # -1 .. 1
    trend = np.polynomial.legendre.legvander(scaled, TREND_ORDER)    # constant, slope, curvature, ... / 定数・1 次・2 次…
    return np.column_stack([np.cos(phase), np.sin(phase), trend])


def fit_complex_amplitudes(times_s, values, frequencies_hz):
    """Least squares of cos, sin (all frequencies), a polynomial trend.
    Returns complex amplitudes A with value ~ Re(A exp(j 2 pi f t)), and the coefficient vector.
    全周波数の cos・sin と定数・1 次の項の最小二乗。value ~ Re(A exp(j 2 pi f t)) となる複素振幅 A を返す。"""
    frequencies = np.asarray(frequencies_hz, dtype=float)
    matrix = _design_matrix(np.asarray(times_s, dtype=float), frequencies)
    coefficients, *_ = np.linalg.lstsq(matrix, np.asarray(values, dtype=float), rcond=None)
    count = frequencies.size
    return coefficients[:count] - 1j * coefficients[count:2 * count], coefficients


def tone_amplitudes_with_noise(times_s, values, tones_hz) -> dict:
    """Complex amplitude of each tone and the mean power of its non-tone neighbors.
    各音の複素振幅と、その近傍（音でない周波数）の平均電力。"""
    tones = np.asarray(tones_hz, dtype=float)
    neighbors = neighbor_frequencies(tones)
    amplitudes, _ = fit_complex_amplitudes(times_s, values, np.concatenate([tones, neighbors]))
    tone_amp = amplitudes[:tones.size]
    neighbor_power = np.abs(amplitudes[tones.size:]) ** 2
    noise_power = np.empty(tones.size)
    for i, tone in enumerate(tones):
        near = np.abs(neighbors - tone) <= max(abs(o) for o in NEIGHBOR_OFFSETS_HZ) + 1e-6
        noise_power[i] = float(np.mean(neighbor_power[near])) if near.any() else float(np.mean(neighbor_power))
    power = np.abs(tone_amp) ** 2
    return {"amplitude": tone_amp, "noise_power": noise_power,
            "power_ratio": power / np.maximum(noise_power, 1e-300),
            "coherence": power / np.maximum(power + noise_power, 1e-300)}


# =============================================================================
# 4. G(jw) and the (K, tau, L) fit / 周波数応答と (K, tau, L) の当てはめ
# =============================================================================

def model_response(parameters, omega: np.ndarray) -> np.ndarray:
    """K exp(-L jw) / (tau jw + 1). tau and L enter as absolute values. / tau と L は絶対値で入れる。"""
    gain, tau, dead_time = parameters[0], abs(parameters[1]), abs(parameters[2])
    return gain * np.exp(-1j * omega * dead_time) / (1.0 + 1j * omega * tau)


def _log_residual(parameters, omega, measured) -> np.ndarray:
    """Real vector [ln-magnitude error, phase error] of measured / model (phase wrapped).
    測定 ÷ モデルの [対数振幅の誤差, 位相の誤差]（位相は巻き戻す）。"""
    ratio = np.log(measured / model_response(parameters, omega))
    return np.concatenate([ratio.real, ratio.imag])


def _weighted_cost(parameters, omega, measured, weights) -> float:
    residual = _log_residual(parameters, omega, measured)
    return float(np.sum(np.tile(weights, 2) * residual ** 2))


def _jacobian(parameters, omega, measured) -> np.ndarray:
    columns = []
    for i in range(FIT_PARAMETER_COUNT):
        step = FINITE_DIFFERENCE_RATIO * max(abs(parameters[i]), 1e-3)
        plus, minus = np.array(parameters, float), np.array(parameters, float)
        plus[i] += step
        minus[i] -= step
        columns.append((_log_residual(plus, omega, measured) - _log_residual(minus, omega, measured)) / (2 * step))
    return np.column_stack(columns)


def _standard_errors(parameters, omega, measured, weights):
    """sqrt(diag(s^2 (J' W J)^-1)) with s^2 the weighted residual variance. / 重み付き残差分散で縮尺した標準誤差。"""
    jac = _jacobian(parameters, omega, measured)
    w = np.tile(weights, 2)
    residual = _log_residual(parameters, omega, measured)
    dof = max(residual.size - FIT_PARAMETER_COUNT, 1)
    variance = float(np.sum(w * residual ** 2)) / dof
    try:
        covariance = np.linalg.inv(jac.T @ (w[:, None] * jac)) * variance
        return np.sqrt(np.maximum(np.diag(covariance), 0.0)), math.sqrt(variance)
    except np.linalg.LinAlgError:
        return np.full(FIT_PARAMETER_COUNT, float("nan")), math.sqrt(variance)


def _nelder_mead():
    """The small dependency-free optimizer of rate_sysid.py. / rate_sysid.py の小型の最適化。"""
    backend = str(Path(__file__).resolve().parent.parent / "log_analyzer")
    if backend not in sys.path:
        sys.path.insert(0, backend)
    import rate_sysid
    return rate_sysid._nelder_mead


def fit_thrust_plant(frequencies_hz, response, weights) -> dict:
    """Weighted least squares of K exp(-L s)/(tau s + 1) to the complex response (Nelder-Mead).
    複素応答への K exp(-L s)/(tau s + 1) の重み付き最小二乗（Nelder-Mead）。"""
    omega = 2.0 * math.pi * np.asarray(frequencies_hz, dtype=float)
    response = np.asarray(response, dtype=complex)
    weights = np.asarray(weights, dtype=float)
    gain_start = float(np.median(np.abs(response[:2])))
    best, best_cost = None, float("inf")
    for tau_start in FIT_TAU_STARTS_S:
        start = [gain_start, tau_start, FIT_DEAD_TIME_START_S]
        scale = [0.1 * gain_start, 0.5 * tau_start, 0.5 * FIT_DEAD_TIME_START_S]
        candidate, cost = _nelder_mead()(lambda p: _weighted_cost(p, omega, response, weights),
                                         start, scale, iters=600)
        if cost < best_cost:
            best, best_cost = candidate, cost
    best = np.array([best[0], abs(best[1]), abs(best[2])])
    errors, residual_rms = _standard_errors(best, omega, response, weights)
    return {"K": float(best[0]), "tau_s": float(best[1]), "L_s": float(best[2]),
            "K_std_error": float(errors[0]), "tau_std_error_s": float(errors[1]),
            "L_std_error_s": float(errors[2]), "residual_rms": residual_rms,
            "weighted_cost": float(best_cost)}


def frequency_response(thrust_tones: dict, accel_tones: dict, tones_hz) -> dict:
    """G(jw) = A_a / A_u per tone, i.e. S_ra / S_ru (r is common and cancels), with coherence.
    音ごとの G(jw) = A_a / A_u（= S_ra / S_ru。r は共通で消える）とコヒーレンス。"""
    response = accel_tones["amplitude"] / thrust_tones["amplitude"]
    coherence = np.minimum(thrust_tones["coherence"], accel_tones["coherence"])
    return {"frequency_hz": np.asarray(tones_hz, dtype=float), "response": response,
            "coherence": coherence, "used": coherence >= COHERENCE_MIN}


# =============================================================================
# 5. ToF delay relative to the accelerometer / ToF の遅れ（加速度計に対する相対値）
# =============================================================================

def estimate_tof_delay(tones_hz, accel_amplitude, tof_tones: dict) -> dict:
    """Delay of the ToF relative to the accelerometer from the phase slope over the tones.
    a = h'' gives H_acc = -A_a / w^2, so arg(H_tof / H_acc) = -w (delay_tof - delay_accel).
    各音の位相の傾きから、ToF の加速度計に対する相対的な遅れを求める。"""
    omega = 2.0 * math.pi * np.asarray(tones_hz, dtype=float)
    ratio = tof_tones["amplitude"] / (-accel_amplitude / omega ** 2)
    phase = np.angle(ratio)
    snr = tof_tones["power_ratio"]
    usable = snr >= TOF_SNR_MIN
    result = {"tone_hz": [float(f) for f in tones_hz], "tof_snr": [float(v) for v in snr],
              "phase_rad": [float(v) for v in phase], "magnitude_ratio": [float(v) for v in np.abs(ratio)],
              "used_tones_hz": [float(f) for f in np.asarray(tones_hz)[usable]]}
    if usable.sum() < 2:
        return {**result, "tof_delay_relative_to_accelerometer_s": None, "std_error_s": None}
    weight = 2.0 * snr[usable]
    w_omega = omega[usable]
    slope_denominator = float(np.sum(weight * w_omega ** 2))
    delay = -float(np.sum(weight * w_omega * phase[usable])) / slope_denominator
    chi_square = float(np.sum(weight * (phase[usable] + w_omega * delay) ** 2)) / max(usable.sum() - 1, 1)
    std_error = math.sqrt(max(chi_square, 1.0) / slope_denominator)
    return {**result, "tof_delay_relative_to_accelerometer_s": delay, "std_error_s": std_error}


# =============================================================================
# 6. Accelerometer noise / 加速度計の雑音
# =============================================================================

def welch_psd(values: np.ndarray):
    """Welch PSD of a 400 Hz series [(m/s^2)^2/Hz]; Hann window, 50 % overlap.
    400 Hz の系列の Welch 法の電力スペクトル密度。"""
    from scipy.signal import welch
    segment = int(min(NOISE_WELCH_SEGMENT_CYCLES, values.size))
    return welch(values - values.mean(), fs=CYCLE_RATE_HZ, nperseg=segment, detrend="linear")


def noise_summary(values: np.ndarray, masked_tones_hz=()) -> dict:
    """Band RMS [m/s^2] over NOISE_BANDS_HZ, floor [(m/s^2)/sqrt(Hz)] and total RMS.
    帯域ごとの実効値、床の値、全体の実効値。"""
    frequency, psd = welch_psd(values)
    keep = np.ones(frequency.size, dtype=bool)
    for tone in masked_tones_hz:
        keep &= np.abs(frequency - tone) > NOISE_TONE_MASK_HALF_WIDTH_HZ
    bin_width = float(frequency[1] - frequency[0])
    bands = {}
    for low, high in NOISE_BANDS_HZ:
        inside = keep & (frequency >= low) & (frequency < high)
        bands[f"{low}-{high}Hz"] = (math.sqrt(float(np.sum(psd[inside])) * bin_width)
                                    if inside.any() else None)
    floor_band = (frequency >= NOISE_FLOOR_BAND_HZ[0]) & (frequency <= NOISE_FLOOR_BAND_HZ[1])
    return {"band_rms_mps2": bands, "total_rms_mps2": float(np.std(values)),
            "floor_mps2_per_sqrt_hz": math.sqrt(float(np.median(psd[floor_band]))),
            "samples": int(values.size)}


def _hover_window_rows(record: ThrustRecord, first_active_row: int) -> Optional[tuple]:
    """Rows of the last stable hover window before the standby (10 s, else 5 s).
    待機の前の、高さが安定したホバー区間（10 s、だめなら 5 s）の行範囲。"""
    stop = first_active_row - int(HOVER_GUARD_S * CYCLE_RATE_HZ)
    for length_s in (HOVER_WINDOW_LONG_S, HOVER_WINDOW_SHORT_S):
        start = stop - int(length_s * CYCLE_RATE_HZ)
        if start < 0:
            continue
        low, high = record.cycle[start], record.cycle[stop - 1]
        inside = (record.tof_cycle >= low) & (record.tof_cycle <= high)
        valid = np.isfinite(record.thrust[start:stop]).all()
        if valid and inside.sum() > 3 and np.std(record.tof_height[inside]) <= HOVER_HEIGHT_STD_MAX_M:
            return start, stop
    return None


def accelerometer_noise(record: ThrustRecord, excited: Optional[dict]) -> dict:
    """Noise before the excitation (state 0, stable hover) and during it (tones subtracted).
    励振の前（状態 0・安定したホバー）と励振の最中（音を引いた後）の雑音。"""
    result = {}
    active = np.flatnonzero((record.state_code & STATE_MASK) >= 1)
    if active.size:
        window = _hover_window_rows(record, int(active[0]))
        if window is not None:
            start, stop = window
            result["hover_before_excitation"] = noise_summary(record.upward_accel[start:stop])
            result["hover_before_excitation"]["duration_s"] = (stop - start) / CYCLE_RATE_HZ
    if excited is not None:
        result["during_excitation"] = excited
    return result


# =============================================================================
# 7. One multisine run / 多重正弦波の 1 回の実行
# =============================================================================

def _steady_window(record: ThrustRecord, start_cycle: int, total_cycles: int):
    """Rows (valid thrust and acceleration) inside the steady part of the run, and their times [s].
    実行の定常部（立ち上げ・立ち下げの外側）の有効な行と、その時刻 [s]。"""
    first = table.MULTISINE_RAMP_UP_CYCLES + STEADY_MARGIN_CYCLES
    last = total_cycles - table.MULTISINE_RAMP_DOWN_CYCLES - STEADY_MARGIN_CYCLES
    k = record.cycle - start_cycle
    rows = (k >= first) & (k < last) & np.isfinite(record.thrust) & np.isfinite(record.upward_accel)
    tof = (record.tof_cycle - start_cycle >= first) & (record.tof_cycle - start_cycle < last)
    return rows, tof, first, last


def _tone_table(frequency: dict, thrust_tones: dict, accel_tones: dict, fitted) -> list:
    rows = []
    for i, tone in enumerate(frequency["frequency_hz"]):
        response = frequency["response"][i]
        rows.append({"frequency_hz": float(tone), "gain_mps2_per_n": float(abs(response)),
                     "phase_rad": float(np.angle(response)),
                     "coherence": float(frequency["coherence"][i]), "used_in_fit": bool(frequency["used"][i]),
                     "thrust_amplitude_n": float(abs(thrust_tones["amplitude"][i])),
                     "accel_amplitude_mps2": float(abs(accel_tones["amplitude"][i])),
                     "fitted_gain": float(abs(fitted[i])) if fitted is not None else None,
                     "fitted_phase_rad": float(np.angle(fitted[i])) if fitted is not None else None})
    return rows


def analyze_multisine_segment(record: ThrustRecord, segment: dict) -> dict:
    """Frequency response, (K, tau, L), ToF delay and noise of one multisine run.
    1 回の多重正弦波の実行の、周波数応答・(K, tau, L)・ToF の遅れ・雑音。"""
    total = estimate_total_cycles(segment["length_cycles_edge"])
    start = search_start_cycle(record, segment["edge_row"], total)
    result = {"waveform": "multisine", "length_cycles": total, "start": start,
              "aborted_suspected": segment["ramp_down_cycles"] < ABORTED_RAMP_DOWN_CYCLES_MAX}
    if result["aborted_suspected"]:
        return {**result, "status": "skipped: the run was aborted (ramp-down shorter than normal)"}
    rows, tof_rows, first, last = _steady_window(record, start["start_cycle"], total)
    times = (record.cycle[rows] - start["start_cycle"]) / CYCLE_RATE_HZ
    tof_times = (record.tof_cycle[tof_rows] - start["start_cycle"]) / CYCLE_RATE_HZ
    tones = np.array(TONE_FREQUENCIES_HZ)
    thrust_tones = tone_amplitudes_with_noise(times, record.thrust[rows], tones)
    accel_tones = tone_amplitudes_with_noise(times, record.upward_accel[rows], tones)
    tof_tones = tone_amplitudes_with_noise(tof_times, record.tof_height[tof_rows], tones)
    response = frequency_response(thrust_tones, accel_tones, tones)
    used = response["used"]
    fit = None
    if used.sum() >= FIT_PARAMETER_COUNT:
        fit = fit_thrust_plant(tones[used], response["response"][used], response["coherence"][used])
    fitted = (model_response([fit["K"], fit["tau_s"], fit["L_s"]], 2 * math.pi * tones)
              if fit else None)
    residual = record.upward_accel[rows] - _tone_fit(times, record.upward_accel[rows], tones)
    result.update({
        "status": "ok", "fit": fit, "tones": _tone_table(response, thrust_tones, accel_tones, fitted),
        "tof": estimate_tof_delay(tones, accel_tones["amplitude"], tof_tones),
        "noise_during_excitation": noise_summary(residual, tones),
        "data": {"steady_window_s": (last - first) / CYCLE_RATE_HZ, "rows_used": int(rows.sum()),
                 "tof_samples_used": int(tof_rows.sum()), "tones_used_in_fit": int(used.sum())},
        "_plot": {"tones": tones, "response": response["response"], "used": used,
                  "fitted": fitted, "tof": None}})
    return result


def _tone_fit(times, values, tones) -> np.ndarray:
    """The part of `values` explained by the tones, a polynomial trend. / 音・定数・1 次の項で説明できる部分。"""
    _, coefficients = fit_complex_amplitudes(times, values, tones)
    return _design_matrix(np.asarray(times, float), np.asarray(tones, float)) @ coefficients


# =============================================================================
# 8. One step run (cut-out and rise delay only) / 階段の 1 回の実行（切り出しと立ち上がりの遅れだけ）
# =============================================================================

def _first_crossing(oriented: np.ndarray, threshold: float) -> Optional[int]:
    hit = np.flatnonzero(oriented >= threshold)
    return int(hit[0]) if hit.size else None


def analyze_steps_segment(record: ThrustRecord, segment: dict) -> dict:
    """Cut the step run out and estimate how late the acceleration rises after the thrust does.
    The step length is unknown to the PC, so the first half period is taken from the first sign
    change of the thrust deviation. The 20 % crossing times of the first step are compared.
    階段の区間を切り出し、推力に対する加速度の立ち上がりの遅れを簡易に推定する。階段の長さは PC が
    知らないので、最初の半周期は推力の偏差の最初の符号の反転から取る。最初の段の 20 % 通過時刻を比べる。"""
    edge = segment["edge_row"]
    base = slice(max(edge - HIGH_PASS_WINDOW_CYCLES // 2, 0), edge)
    thrust = record.thrust[edge:segment["end_row"]] - np.nanmedian(record.thrust[base])
    accel = record.upward_accel[edge:segment["end_row"]] - np.nanmedian(record.upward_accel[base])
    ramp = table.STEPS_RAMP_UP_CYCLES
    plateau = np.nanmedian(thrust[ramp:ramp + ramp])
    orient = 1.0 if plateau >= 0.0 else -1.0
    flips = np.flatnonzero(orient * thrust[ramp:] < 0.0)
    half = int(flips[0]) + ramp if flips.size else thrust.size
    peak_u, peak_a = np.nanmax(orient * thrust[:half]), np.nanmax(orient * accel[:half])
    rise_u = _first_crossing(np.nan_to_num(orient * thrust[:half]), 0.2 * peak_u)
    rise_a = _first_crossing(np.nan_to_num(orient * accel[:half]), 0.2 * peak_a)
    delay = (rise_a - rise_u) / CYCLE_RATE_HZ if rise_u is not None and rise_a is not None else None
    return {"waveform": "steps", "status": "ok", "edge_cycle": int(record.cycle[edge]),
            "length_cycles": segment["length_cycles_edge"],
            "first_half_period_s_estimate": half / CYCLE_RATE_HZ,
            "thrust_step_height_n": float(peak_u), "accel_peak_mps2": float(peak_a),
            "accel_rise_delay_20pct_s": delay,
            "note": "20% crossing delay of a relative to u; includes both L and part of tau. "
                    "20 % 通過の遅れ。L と tau の一部の両方を含む。"}


# =============================================================================
# 9. Whole bundle, output, plot / 一式の全体・出力・図
# =============================================================================

def analyze_record(record: ThrustRecord) -> dict:
    """All analyses of one flight record. / 1 回の飛行の記録の全解析。"""
    segments = find_excitation_segments(record)
    results = []
    for segment in segments:
        if segment["waveform"] == WAVEFORM_MULTISINE:
            results.append(analyze_multisine_segment(record, segment))
        elif segment["waveform"] == WAVEFORM_STEPS:
            results.append(analyze_steps_segment(record, segment))
    output = {"source": record.source, "segments": results,
              "data": {"cycles": int(record.cycle.size), "thrust_rows": int(np.isfinite(record.thrust).sum()),
                       "ctrl_output_gap_rows": record.gap_count, "tof_samples": int(record.tof_cycle.size),
                       "excitation_segments": len(segments)}}
    multisine = next((r for r in results if r.get("status") == "ok" and r["waveform"] == "multisine"), None)
    noise = accelerometer_noise(record, multisine["noise_during_excitation"] if multisine else None)
    output["accelerometer_noise"] = noise
    if multisine:
        output.update({"fit": multisine["fit"], "tof": multisine["tof"], "start": multisine["start"],
                       "tones": multisine["tones"], "_plot": multisine["_plot"]})
    return output


def analyze_bundle(path) -> dict:
    """Load a flight-log bundle and analyze it. / 記録の一式を読み込んで解析する。"""
    return analyze_record(load_thrust_record(path))


def to_jsonable(value):
    """Plain Python for json.dumps; keys starting with '_' (plot arrays) are dropped.
    json.dumps 用に素の Python へ。'_' で始まるキー（図用の配列）は落とす。"""
    if isinstance(value, dict):
        return {k: to_jsonable(v) for k, v in value.items() if not str(k).startswith("_")}
    if isinstance(value, (list, tuple)):
        return [to_jsonable(v) for v in value]
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def summary_lines(result: dict) -> list:
    """Short console summary. / コンソール用の短い要約。"""
    lines = []
    fit = result.get("fit")
    if fit:
        lines.append(f"K = {fit['K']:.2f} +- {fit['K_std_error']:.2f} (m/s^2)/N   "
                     f"tau = {fit['tau_s'] * 1e3:.1f} +- {fit['tau_std_error_s'] * 1e3:.1f} ms   "
                     f"L = {fit['L_s'] * 1e3:.1f} +- {fit['L_std_error_s'] * 1e3:.1f} ms (dead time incl. accelerometer path)")
    tof = result.get("tof") or {}
    delay = tof.get("tof_delay_relative_to_accelerometer_s")
    if delay is not None:
        lines.append(f"ToF delay RELATIVE to the accelerometer = {delay * 1e3:.1f} +- {tof['std_error_s'] * 1e3:.1f} ms "
                     f"(tones used: {tof['used_tones_hz']})")
    start = result.get("start")
    if start:
        lines.append(f"excitation start: edge{start['offset_from_edge_cycles']:+d} cycles, "
                     f"correlation {start['correlation']:.3f} (runner-up {start['correlation_runner_up']:.3f})")
    for name, noise in result.get("accelerometer_noise", {}).items():
        lines.append(f"accelerometer noise [{name}]: floor {noise['floor_mps2_per_sqrt_hz']:.2e} (m/s^2)/sqrt(Hz), "
                     f"bands {noise['band_rms_mps2']}")
    return lines


def plot_result(result: dict, path) -> Optional[str]:
    """Magnitude, phase, coherence and the ToF phase against frequency (PNG). / 振幅・位相・コヒーレンス・ToF の位相の図。"""
    arrays = result.get("_plot")
    if not arrays:
        return None
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    tones, response, used, fitted = arrays["tones"], arrays["response"], arrays["used"], arrays["fitted"]
    coherence = [t["coherence"] for t in result["tones"]]
    figure, axes = plt.subplots(4, 1, figsize=(7, 10), sharex=True)
    axes[0].semilogy(tones, np.abs(response), "o", label="measured")
    axes[1].plot(tones, np.degrees(np.angle(response)), "o")
    if fitted is not None:
        dense = np.linspace(tones[0], tones[-1], 200)
        fit = result["fit"]
        curve = model_response([fit["K"], fit["tau_s"], fit["L_s"]], 2 * math.pi * dense)
        axes[0].semilogy(dense, np.abs(curve), "-", label="fit")
        axes[1].plot(dense, np.degrees(np.angle(curve)), "-")
    axes[0].set_ylabel("|G| [(m/s^2)/N]")
    axes[1].set_ylabel("phase [deg]")
    axes[2].bar(tones, coherence, width=0.15)
    axes[2].axhline(COHERENCE_MIN, color="r", linestyle="--")
    axes[2].set_ylabel("coherence")
    tof = result["tof"]
    axes[3].plot(tof["tone_hz"], np.degrees(tof["phase_rad"]), "s")
    axes[3].set_ylabel("ToF vs accel phase [deg]")
    axes[3].set_xlabel("frequency [Hz]")
    axes[0].legend()
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)
    return str(path)
