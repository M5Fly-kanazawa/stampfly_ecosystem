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
TREND_ORDER = 5                                      # polynomial trend removed with the tones (absorbs the ToF height wander) / 音と一緒に引く多項式の次数（ToF の高さの漂いを吸収）
NOISE_PROBE_HALF_WIDTH_HZ = 0.6                      # noise probed at Fourier bins k/T within +-0.6 Hz of a tone / 雑音は音の ±0.6 Hz の中の k/T で見積もる
PROBE_GAIN_MAX = 2.0                                 # a probe bin the trend absorbs more than this is dropped / 傾向に吸収される度合いがこれを超える点は捨てる
NOISE_PROBE_MAX_COUNT = 6                            # use the nearest 6 noise-only bins (12 degrees of freedom) / 最も近い 6 点（自由度 12）を使う
NOISE_PROBE_TONE_EXCLUSION_BINS = 1.0                # probes stay this many 1/T away from every tone / 見積もり点を音から離す距離（1/T の倍数）
CONDITION_NUMBER_WARN = 100.0                        # design-matrix condition number above this -> warning / 設計行列の条件数がこれを超えたら警告
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

def _design_matrix(times_s: np.ndarray, frequencies_hz: np.ndarray) -> np.ndarray:
    phase = 2.0 * math.pi * np.outer(times_s, frequencies_hz)
    span = max(float(times_s.max() - times_s.min()), 1e-9)
    scaled = 2.0 * (times_s - times_s.mean()) / span                 # -1 .. 1
    trend = np.polynomial.legendre.legvander(scaled, TREND_ORDER)    # constant, slope, curvature, ... / 定数・1 次・2 次…
    return np.column_stack([np.cos(phase), np.sin(phase), trend])


def fit_complex_amplitudes(times_s, values, frequencies_hz):
    """Least squares of cos, sin (all frequencies), a polynomial trend.
    Returns complex amplitudes A with value ~ Re(A exp(j 2 pi f t)), and the coefficient vector.
    全周波数の cos・sin と多項式の傾向の最小二乗。value ~ Re(A exp(j 2 pi f t)) となる複素振幅 A を返す。"""
    frequencies = np.asarray(frequencies_hz, dtype=float)
    matrix = _design_matrix(np.asarray(times_s, dtype=float), frequencies)
    coefficients, *_ = np.linalg.lstsq(matrix, np.asarray(values, dtype=float), rcond=None)
    count = frequencies.size
    return coefficients[:count] - 1j * coefficients[count:2 * count], coefficients


def window_resolution_hz(times_s) -> float:
    """Fourier resolution 1/T of the window. / 窓のフーリエ分解能 1/T。"""
    times = np.asarray(times_s, dtype=float)
    return 1.0 / max(float(times.max() - times.min()), 1e-9)


def probe_frequencies(tones_hz, resolution_hz: float, exclude_tones: bool = True) -> list:
    """For each tone, the Fourier bins k/T (k >= 1) within +-0.6 Hz that are not within 1/T of any tone.
    Bins spaced by 1/T are orthogonal on a uniform window, so the probes never make a design ill-conditioned,
    whatever the window length. Listed nearest first; the nearest 6 usable ones are used (12 degrees of freedom: stable, and
    local, because the noise of the ToF height and of the slow wander is red, larger at low frequency).
    各音について、±0.6 Hz の中の窓のフーリエの格子 k/T（k >= 1）のうち、どの音からも 1/T 以上離れたもの。
    1/T の間隔の格子は一様な窓で直交するので、窓の長さによらず悪条件にならない。近い順に並べ、使える点のうち最も近い
    6 点を使う（自由度 12 で安定し、かつ局所的。ToF の高さや遅い変動の雑音は低周波ほど大きい赤色のため）。"""
    tones = np.asarray(tones_hz, dtype=float)
    bins = resolution_hz * np.arange(1, int(100.0 / resolution_hz))
    probes = []
    for tone in tones:
        near = np.abs(bins - tone) <= NOISE_PROBE_HALF_WIDTH_HZ
        if exclude_tones:
            near &= np.min(np.abs(bins[:, None] - tones[None, :]), axis=1) >= NOISE_PROBE_TONE_EXCLUSION_BINS * resolution_hz
        candidates = bins[near]
        probes.append([float(f) for f in candidates[np.argsort(np.abs(candidates - tone))]])    # nearest first / 近い順
    return probes


def _probe_power(times_s, values, frequency_hz: float) -> Optional[float]:
    """|A|^2 of one least-squares fit of cos, sin at `frequency_hz` plus the same polynomial trend as the
    tone fit: the power a noise-only bin shows in the tone estimator. Returns None when the trend absorbs the
    bin (the noise gain of the fit is more than PROBE_GAIN_MAX times that of an ideal orthogonal fit), because
    such a bin cannot show the noise: its amplitude is blown up by the near-collinearity instead.
    `frequency_hz` の cos・sin と音の当てはめと同じ多項式の傾向を最小二乗で当てはめたときの |A|^2。雑音だけの格子点が
    音の推定器に現れる電力。傾向が格子点を吸収してしまう（当てはめの雑音の倍率が理想の直交の場合の
    PROBE_GAIN_MAX 倍を超える）ときは None。そのような点は雑音を映さず、ほぼ一次従属による増幅だけが出るため。"""
    times = np.asarray(times_s, dtype=float)
    matrix = _design_matrix(times, np.array([frequency_hz]))
    inverse = np.linalg.pinv(matrix.T @ matrix)
    if (inverse[0, 0] + inverse[1, 1]) * times.size / 4.0 > PROBE_GAIN_MAX:
        return None
    coefficients, *_ = np.linalg.lstsq(matrix, np.asarray(values, dtype=float), rcond=None)
    return float(coefficients[0] ** 2 + coefficients[1] ** 2)


def local_noise_power(times_s, residual, tones_hz, resolution_hz: float, exclude_tones: bool = True):
    """Per tone: (mean |A|^2 of the nearest NOISE_PROBE_MAX_COUNT usable noise-only bins of `residual`,
    number of bins used). The mean is NaN when no bin is usable. It is the noise power of that tone's
    amplitude estimate. The bins are those of `resolution_hz` (the EXCITATION window's 1/T).
    各音について、(残差の使える雑音だけの格子点のうち最も近い NOISE_PROBE_MAX_COUNT 点の |A|^2 の平均, 使った点数)。
    使える点が無ければ平均は NaN。その音の振幅の推定に乗る雑音の電力に等しい。格子は励振の窓の 1/T。"""
    power = np.full(len(tones_hz), np.nan)
    counts = np.zeros(len(tones_hz), dtype=int)
    for i, found in enumerate(probe_frequencies(tones_hz, resolution_hz, exclude_tones)):
        values = []
        for frequency in found:
            value = _probe_power(times_s, residual, frequency)
            if value is not None:
                values.append(value)
            if len(values) >= NOISE_PROBE_MAX_COUNT:
                break
        counts[i] = len(values)
        if values:
            power[i] = float(np.mean(values))
    return power, counts


def _design_condition_number(times_s, tones_hz) -> float:
    """Condition number of the tones + trend design matrix. / 音と傾向の設計行列の条件数。"""
    matrix = _design_matrix(np.asarray(times_s, dtype=float), np.asarray(tones_hz, dtype=float))
    return float(np.linalg.cond(matrix))


def _trend_residual(times_s, values) -> np.ndarray:
    """`values` minus its low-order polynomial trend (no tones). / 低次の多項式の傾向を引いた値（音は引かない）。"""
    times = np.asarray(times_s, dtype=float)
    matrix = _design_matrix(times, np.empty(0))
    coefficients, *_ = np.linalg.lstsq(matrix, np.asarray(values, dtype=float), rcond=None)
    return np.asarray(values, dtype=float) - matrix @ coefficients


def tone_amplitudes_with_noise(times_s, values, tones_hz, hover=None) -> dict:
    """Complex amplitude of each tone (tones + low-order trend only, so the design stays well
    conditioned for any window length) and its noise power from the RESIDUAL spectrum: the mean power
    of the noise-only Fourier bins k/T around the tone, seen through the same estimator.
    `hover` = (times, values) of the hover before the excitation gives the optimistic comparison noise
    (its power is scaled to the sample count of the excitation window).
    各音の複素振幅（音と低次の傾向だけを当てはめるので、窓が短くても設計行列は悪条件にならない）と、残差の
    スペクトルから見積もった雑音の電力（同じ推定器で見た、音の周りの雑音だけの格子点 k/T の電力の平均）。
    `hover` = 励振の前のホバーの (時刻, 値)。楽観的になる比較用の雑音も返す（電力は励振の窓の標本数に換算）。"""
    tones = np.asarray(tones_hz, dtype=float)
    times = np.asarray(times_s, dtype=float)
    amplitudes, coefficients = fit_complex_amplitudes(times, values, tones)
    residual = np.asarray(values, dtype=float) - _design_matrix(times, tones) @ coefficients
    condition = _design_condition_number(times, tones)
    resolution = window_resolution_hz(times)
    noise_power, probe_counts = local_noise_power(times, residual, tones, resolution)
    power = np.abs(amplitudes) ** 2
    result = {"amplitude": amplitudes, "noise_power": noise_power,
              "power_ratio": power / np.maximum(noise_power, 1e-300),
              "coherence": power / np.maximum(power + noise_power, 1e-300),
              "condition_number": condition, "resolution_hz": resolution, "residual": residual,
              "probe_counts": probe_counts,
              "hover_noise_power": None, "hover_power_ratio": None}
    if hover is not None:
        hover_times = np.asarray(hover[0], dtype=float)
        hover_residual = _trend_residual(hover_times, hover[1])
        # The hover has no tones: no bin needs to be excluded. / ホバーには音がないので、除く点はない。
        hover_power, _ = local_noise_power(hover_times, hover_residual, tones, window_resolution_hz(hover_times), False)
        hover_power = hover_power * hover_times.size / times.size
        result.update({"hover_noise_power": hover_power,
                       "hover_power_ratio": power / np.maximum(hover_power, 1e-300)})
    return result


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
    coherence = np.fmin(thrust_tones["coherence"], accel_tones["coherence"])      # NaN = no noise estimate / 雑音の見積もりなし
    return {"frequency_hz": np.asarray(tones_hz, dtype=float), "response": response,
            "coherence": coherence, "used": np.nan_to_num(coherence, nan=0.0) >= COHERENCE_MIN}


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
        reason = (f"ToF delay skipped: {int(usable.sum())} of {len(snr)} tones have ToF SNR >= {TOF_SNR_MIN:g} "
                  f"(at least 2 are needed); best SNR {np.nanmax(snr) if np.isfinite(snr).any() else float('nan'):.2f}")
        return {**result, "tof_delay_relative_to_accelerometer_s": None, "std_error_s": None, "status": reason}
    weight = 2.0 * snr[usable]
    w_omega = omega[usable]
    slope_denominator = float(np.sum(weight * w_omega ** 2))
    delay = -float(np.sum(weight * w_omega * phase[usable])) / slope_denominator
    chi_square = float(np.sum(weight * (phase[usable] + w_omega * delay) ** 2)) / max(usable.sum() - 1, 1)
    std_error = math.sqrt(max(chi_square, 1.0) / slope_denominator)
    return {**result, "tof_delay_relative_to_accelerometer_s": delay, "std_error_s": std_error,
            "status": f"ok: {int(usable.sum())} of {len(snr)} tones with ToF SNR >= {TOF_SNR_MIN:g}"}


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


def _unusable_reason(coherence: float, snr_accel: float, snr_thrust: float) -> Optional[str]:
    """Why a tone is left out of the fit (None = usable). / 当てはめに使えない理由（None = 使える）。"""
    if not math.isfinite(coherence):
        return "no noise estimate (window too short for the noise probes)"
    if coherence >= COHERENCE_MIN:
        return None
    weaker = "acceleration" if snr_accel <= snr_thrust else "thrust"
    return f"coherence {coherence:.2f} < {COHERENCE_MIN} (weakest: {weaker})"


def _tone_table(frequency: dict, thrust_tones: dict, accel_tones: dict, fitted) -> list:
    """One row per tone, always (also when the fit is skipped). / 当てはめを省いたときも含め、音ごとの行を必ず作る。"""
    rows = []
    for i, tone in enumerate(frequency["frequency_hz"]):
        response = frequency["response"][i]
        snr_a, snr_u = float(accel_tones["power_ratio"][i]), float(thrust_tones["power_ratio"][i])
        hover_a = accel_tones["hover_power_ratio"]
        hover_u = thrust_tones["hover_power_ratio"]
        rows.append({"frequency_hz": float(tone), "gain_mps2_per_n": float(abs(response)),
                     "phase_rad": float(np.angle(response)),
                     "coherence": float(frequency["coherence"][i]), "used_in_fit": bool(frequency["used"][i]),
                     "unused_reason": _unusable_reason(float(frequency["coherence"][i]), snr_a, snr_u),
                     "snr_accel_in_excitation": snr_a, "snr_thrust_in_excitation": snr_u,
                     "snr_accel_hover": float(hover_a[i]) if hover_a is not None else None,
                     "snr_thrust_hover": float(hover_u[i]) if hover_u is not None else None,
                     "thrust_amplitude_n": float(abs(thrust_tones["amplitude"][i])),
                     "accel_amplitude_mps2": float(abs(accel_tones["amplitude"][i])),
                     "fitted_gain": float(abs(fitted[i])) if fitted is not None else None,
                     "fitted_phase_rad": float(np.angle(fitted[i])) if fitted is not None else None})
    return rows


def _fit_status(used_count: int, total: int, fit) -> str:
    """One line saying whether the (K, tau, L) fit ran and why not. / 当てはめを行ったか、省いた理由の 1 行。"""
    if fit is not None:
        return f"ok: fitted with {used_count} of {total} tones (coherence >= {COHERENCE_MIN})"
    return (f"(K, tau, L) fit skipped: {used_count} of {total} tones are usable "
            f"(coherence >= {COHERENCE_MIN}, i.e. tone power / noise power >= "
            f"{COHERENCE_MIN / (1.0 - COHERENCE_MIN):g}); at least {FIT_PARAMETER_COUNT} are needed")


def _hover_inputs(record: ThrustRecord):
    """(accel hover, thrust hover) as (times, values), or (None, None) when no stable hover precedes the run.
    ホバー区間の（加速度, 推力）を (時刻, 値) で返す。安定したホバーが無ければ (None, None)。"""
    active = np.flatnonzero((record.state_code & STATE_MASK) >= 1)
    window = _hover_window_rows(record, int(active[0])) if active.size else None
    if window is None:
        return None, None
    rows = slice(*window)
    times = (record.cycle[rows] - record.cycle[window[0]]) / CYCLE_RATE_HZ
    return (times, record.upward_accel[rows]), (times, record.thrust[rows])


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
    hover_accel, hover_thrust = _hover_inputs(record)
    thrust_tones = tone_amplitudes_with_noise(times, record.thrust[rows], tones, hover_thrust)
    accel_tones = tone_amplitudes_with_noise(times, record.upward_accel[rows], tones, hover_accel)
    tof_tones = tone_amplitudes_with_noise(tof_times, record.tof_height[tof_rows], tones)
    response = frequency_response(thrust_tones, accel_tones, tones)
    used = response["used"]
    fit = None
    if used.sum() >= FIT_PARAMETER_COUNT:
        fit = fit_thrust_plant(tones[used], response["response"][used], response["coherence"][used])
    fitted = (model_response([fit["K"], fit["tau_s"], fit["L_s"]], 2 * math.pi * tones)
              if fit else None)
    residual = accel_tones["residual"]
    conditions = {"thrust_and_accel": thrust_tones["condition_number"], "tof": tof_tones["condition_number"]}
    warnings = [f"design-matrix condition number {value:.3g} ({name}) exceeds {CONDITION_NUMBER_WARN:g}: "
                f"the tone amplitudes may be inaccurate" for name, value in conditions.items()
                if value > CONDITION_NUMBER_WARN]
    result.update({
        "status": "ok", "fit": fit, "fit_status": _fit_status(int(used.sum()), tones.size, fit),
        "tones": _tone_table(response, thrust_tones, accel_tones, fitted), "warnings": warnings,
        "tof": estimate_tof_delay(tones, accel_tones["amplitude"], tof_tones),
        "noise_during_excitation": noise_summary(residual, tones),
        "data": {"steady_window_s": (last - first) / CYCLE_RATE_HZ, "rows_used": int(rows.sum()),
                 "tof_samples_used": int(tof_rows.sum()), "tones_used_in_fit": int(used.sum()),
                 "fourier_resolution_hz": thrust_tones["resolution_hz"],
                 "condition_number": conditions,
                 "noise_probes_per_tone": {"thrust": thrust_tones["probe_counts"], "accel": accel_tones["probe_counts"],
                                           "tof": tof_tones["probe_counts"]}},
        "_plot": {"tones": tones, "response": response["response"], "used": used,
                  "fitted": fitted, "tof": None}})
    return result


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
        output.update({"fit": multisine["fit"], "fit_status": multisine["fit_status"], "tof": multisine["tof"],
                       "start": multisine["start"], "tones": multisine["tones"], "warnings": multisine["warnings"],
                       "run_data": multisine["data"], "_plot": multisine["_plot"]})
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
    if isinstance(value, np.ndarray):
        return to_jsonable(value.tolist())
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return to_jsonable(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _format_snr(value) -> str:
    return "   n/a" if value is None or not math.isfinite(value) else f"{value:6.2f}"


def tone_table_lines(tones: list) -> list:
    """The per-tone table: always printed, whether or not the fit ran. SNR = tone power / noise power;
    `run` = noise from the residual during the excitation (the reference), `hover` = noise before it (optimistic).
    音ごとの表。当てはめを行っても省いても必ず出す。SNR = 音の電力 ÷ 雑音の電力。run = 励振の間の残差の雑音（基準）、
    hover = 励振の前のホバーの雑音（楽観的）。"""
    lines = ["per-tone response (SNR = tone power / noise power; run = residual during the excitation [reference], "
             "hover = hover before it [optimistic])",
             "  f[Hz]   |G|[(m/s^2)/N]  phase[deg]  SNR_a(run) SNR_a(hover) SNR_u(run) coherence  usable"]
    for row in tones:
        lines.append(f"  {row['frequency_hz']:5.2f}  {row['gain_mps2_per_n']:13.2f}  "
                     f"{math.degrees(row['phase_rad']):9.1f}  {_format_snr(row['snr_accel_in_excitation'])}    "
                     f"{_format_snr(row['snr_accel_hover'])}     {_format_snr(row['snr_thrust_in_excitation'])}   "
                     f"{row['coherence']:8.2f}   {'yes' if row['used_in_fit'] else 'no'}"
                     + ("" if row["used_in_fit"] else f"  ({row['unused_reason']})"))
    return lines


def summary_lines(result: dict) -> list:
    """Console summary: fit (or why it was skipped), the tone table, ToF, start, noise, warnings.
    コンソール用の要約: 当てはめ（省いたなら理由）・音の表・ToF・開始位置・雑音・警告。"""
    lines = []
    fit = result.get("fit")
    if fit:
        lines.append(f"K = {fit['K']:.2f} +- {fit['K_std_error']:.2f} (m/s^2)/N   "
                     f"tau = {fit['tau_s'] * 1e3:.1f} +- {fit['tau_std_error_s'] * 1e3:.1f} ms   "
                     f"L = {fit['L_s'] * 1e3:.1f} +- {fit['L_std_error_s'] * 1e3:.1f} ms (dead time incl. accelerometer path)")
    if result.get("fit_status"):
        lines.append(f"fit: {result['fit_status']}")
    if result.get("tones"):
        lines.extend(tone_table_lines(result["tones"]))
    tof = result.get("tof") or {}
    delay = tof.get("tof_delay_relative_to_accelerometer_s")
    if delay is not None:
        lines.append(f"ToF delay RELATIVE to the accelerometer = {delay * 1e3:.1f} +- {tof['std_error_s'] * 1e3:.1f} ms "
                     f"(tones used: {tof['used_tones_hz']})")
    if tof.get("status"):
        lines.append(f"ToF: {tof['status']}")
    start = result.get("start")
    if start:
        lines.append(f"excitation start: edge{start['offset_from_edge_cycles']:+d} cycles, "
                     f"correlation {start['correlation']:.3f} (runner-up {start['correlation_runner_up']:.3f})")
    run_data = result.get("run_data")
    if run_data:
        conditions = run_data["condition_number"]
        lines.append(f"window {run_data['steady_window_s']:.1f} s (Fourier resolution 1/T = "
                     f"{run_data['fourier_resolution_hz']:.3f} Hz); design condition number "
                     f"thrust/accel {conditions['thrust_and_accel']:.3g}, ToF {conditions['tof']:.3g}")
    for text in result.get("warnings", []):
        lines.append(f"WARNING: {text}")
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
