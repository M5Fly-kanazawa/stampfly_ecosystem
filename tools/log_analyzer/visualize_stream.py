#!/usr/bin/env python3
"""
visualize_stream.py - StampFly flight-log v1 bundle renderer (backend of `sf log viz`)
visualize_stream.py - StampFly フライトログ v1 一式の描画処理（`sf log viz` の実体）

Renders a "StampFly flight-log v1" bundle (a `.sflog.zip` file, or an
extracted directory with the same flat layout -- see
docs/plans/flight-log-format-plan.md section 2 and protocol/spec/
flight_log.yaml, the format's Single Source of Truth). This is the ONE
renderer for the format: every panel is plotted at the stream's OWN native
sample timestamps -- there is no cross-stream alignment, forward-fill, or
resampling here, because the whole point of the v1 format is that a
primary-record CSV never holds a held/interpolated value (an aligned table
is a read-time derived product built by `lib/sflog/align.py`, not something
this module produces).
「StampFly フライトログ v1」一式（`.sflog.zip` ファイル、または同じ平坦
レイアウトの展開済みフォルダ -- 形式の正本は計画書 2 節と
protocol/spec/flight_log.yaml を参照）を描画する。この形式の描画処理は
本モジュール1つに統合されている: 各パネルはそのストリーム自身の原時刻で
描く -- ストリーム間の整列・前方補完・再標本化は一切行わない。v1 形式の
核心は「一次記録の CSV は保持値・補間値を持たない」ことであり、整列表が
欲しければ読み込み時の派生物（`lib/sflog/align.py`）を使う（本モジュールは
作らない）。

Usage:
    python3 visualize_stream.py <bundle_path> [options]

Options:
    --mode {all,attitude,sensors,position,eskf}  Panel group (default: all)
    --save FILE                    Save figure to file
    --no-show                      Don't display window (use with --save)
    --time-range START END         Plot only this time range (seconds)

Examples:
    python3 visualize_stream.py logs/flight_20260911T120000.sflog.zip
    python3 visualize_stream.py flight.sflog.zip --mode attitude --save attitude.png
    python3 visualize_stream.py flight.sflog.zip --time-range 5 15
"""

import argparse
import functools
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

import sflog

# --- Constants (no magic numbers) / 定数（マジックナンバー禁止） ---

# Conversion factor: radians -> degrees. Displayed rates/angles use degrees
# (the units operators normally discuss firmware behavior in); the v1
# bundle itself stores everything in SI (rad, rad/s) per
# protocol/spec/flight_log.yaml `units`.
# rad -> deg 変換係数。表示上のレート・角度は度を使う（現場でファームの
# 挙動を議論する際に普段使う単位）。一式自体は
# protocol/spec/flight_log.yaml の `units` どおり SI（rad, rad/s）で持つ。
RAD_TO_DEG = 180.0 / np.pi

# N*m -> mN*m, used only for the ctrl_output torque panel's display units.
# N*m -> mN*m。ctrl_output のトルクパネルの表示単位にのみ使う。
NM_TO_MNM = 1000.0

# Grid layout: panels are laid out in `cols` columns (default 3) so the
# whole overview fits one screen without squashing the panels vertically
# (a single 15-panel column made the y-axis tick labels overlap once the
# window was maximised).
# 格子配置: パネルを `cols` 列（既定 3）に並べ、縦に潰さずに一画面へ収める
# （15 段の 1 列配置は最大化すると縦軸の目盛りラベルが重なっていた）。
DEFAULT_COLUMNS = 3
MAX_COLUMNS = 4
FIGURE_WIDTH_PER_COLUMN_IN = 5.6
FIGURE_HEIGHT_PER_ROW_IN = 2.3
FIGURE_MAX_HEIGHT_IN = 12.0
# When shown in a window, the figure is resized to this fraction of the
# screen (room for the window title bar and the toolbar) so the overview
# fits the display as opened, whatever the monitor size.
# ウィンドウ表示時は図を画面のこの割合に合わせる（タイトルバーとツールバーの
# 分を残す）。開いた時点でモニタの大きさに関係なく一画面に収まる。
SCREEN_FIT_FRACTION_W = 0.92
SCREEN_FIT_FRACTION_H = 0.86
# Legends sit at the upper right; give every panel head-room above the data
# so the legend rarely covers a trace, and keep the frame translucent.
# 凡例は右上固定。凡例がデータに被りにくいよう各パネルの上側に余白を取り、
# 枠は半透明にする。
LEGEND_HEADROOM_FRACTION = 0.30
LEGEND_FRAME_ALPHA = 0.8
LINE_WIDTH_MEASURED = 1.0
LINE_WIDTH_REFERENCE = 1.0
LINE_WIDTH_THRUST = 1.8
MODE_LINE_WIDTH = 1.2
LEGEND_FONT_SIZE = 8
PANEL_TITLE_FONT_SIZE = 10
FIGURE_TITLE_FONT_SIZE = 14

# Marker sizes for the slower (30-100 Hz) streams, where a marker per
# sample stays legible instead of looking like a solid line.
# 低速（30〜100Hz）ストリーム向けのマーカーサイズ。1標本1マーカーでも
# 実線に潰れず見えるようにする。
HEIGHT_MARKER_SIZE = 3
STATUS_MARKER_SIZE = 4

_MOTOR_NAMES = ('FR', 'RR', 'RL', 'FL')
_MOTOR_COLORS = ('C0', 'C1', 'C2', 'C3')

# (stream name, column, legend label, line style) for panel_height, in the
# order they are drawn -- whichever of these streams exist in the bundle.
# panel_height 用の (ストリーム名, 列名, 凡例ラベル, 線種)。バンドルに
# 存在するものだけが描かれる。
_HEIGHT_SOURCES = (
    ('baro', 'altitude', 'baro.altitude', 'C0-'),
    ('tof_bottom', 'distance', 'tof_bottom', 'C1-'),
    ('tof_front', 'distance', 'tof_front', 'C2-'),
)


# =============================================================================
# Stream-presence helpers and the `requires`/`requires_any` decorators
# ストリーム存在判定と `requires`/`requires_any` デコレータ
# =============================================================================


def _has(log, *stream_names) -> bool:
    """True if EVERY named stream is present and non-empty in `log`.
    列挙した全ストリームが `log` に存在し、かつ空でなければ True。
    """
    return all(name in log.streams and not log.streams[name].empty for name in stream_names)


def _has_any(log, *stream_names) -> bool:
    """True if AT LEAST ONE named stream is present and non-empty in `log`.
    列挙したストリームのうち少なくとも1つが存在し、かつ空でなければ True。
    """
    return any(name in log.streams and not log.streams[name].empty for name in stream_names)


def requires(*stream_names):
    """Decorator: a panel needs every one of `stream_names`. The wrapped
    panel becomes a no-op (returns False without touching `ax`) when any is
    absent, and gets a `.presence(log)` predicate that `render()` uses to
    decide which panels get an axis at all (see the module docstring for
    why the v1 format never fabricates a row for a missing stream).
    デコレータ: パネルが `stream_names` の全てを必要とする場合に使う。
    いずれか欠落時は `ax` に触れず False を返すだけの無処理になり、
    `render()` がどのパネルに軸を割り当てるか決めるための
    `.presence(log)` 述語を持つ。
    """
    def decorator(fn):
        @functools.wraps(fn)
        def wrapper(ax, log, t0_us):
            if not _has(log, *stream_names):
                return False
            return fn(ax, log, t0_us)
        wrapper.presence = lambda log: _has(log, *stream_names)
        return wrapper
    return decorator


def requires_any(*stream_names):
    """Like `requires`, but the panel draws when AT LEAST ONE of
    `stream_names` is present (e.g. panel_height: baro OR tof_bottom OR
    tof_front; panel_motor_duty: motor OR ctrl_ref).
    `requires` と同様だが、`stream_names` の少なくとも1つが存在すれば
    描画する（例: panel_height は baro/tof_bottom/tof_front のいずれか、
    panel_motor_duty は motor か ctrl_ref）。
    """
    def decorator(fn):
        @functools.wraps(fn)
        def wrapper(ax, log, t0_us):
            if not _has_any(log, *stream_names):
                return False
            return fn(ax, log, t0_us)
        wrapper.presence = lambda log: _has_any(log, *stream_names)
        return wrapper
    return decorator


def duty_source(log):
    """Which stream panel_motor_duty reads duty from: 'motor' (400 Hz,
    preferred) if present, else 'ctrl_ref' (50 Hz) if present, else None.
    panel_motor_duty が duty を読む先: 'motor'（400Hz、優先）があればそれ、
    無ければ 'ctrl_ref'（50Hz）、どちらも無ければ None。
    """
    if _has(log, 'motor'):
        return 'motor'
    if _has(log, 'ctrl_ref'):
        return 'ctrl_ref'
    return None


# =============================================================================
# Time-axis helpers
# 時間軸ヘルパー
# =============================================================================


def _t(df, t0_us):
    """Relative time [s] for `df`'s OWN `timestamp_us`, referenced to
    `t0_us` -- never aligned to another stream's clock.
    `df` 自身の `timestamp_us` を `t0_us` 基準の相対時間[秒]に変換する
    -- 他ストリームの時刻には一切揃えない。
    """
    return (df['timestamp_us'].to_numpy() - t0_us) / 1e6


def _bundle_t0_us(log) -> int:
    """The bundle's t=0 reference: the minimum first `timestamp_us` across
    every stream present in `log` (not just the ones a given panel group
    draws), so the time axis lines up the same way regardless of `mode`.
    バンドルの t=0 基準: `log` に存在する全ストリーム（描画対象のパネル群に
    限らない）の最初の `timestamp_us` の最小値。`mode` によらず時間軸が
    揃うようにする。
    """
    firsts = [
        int(df['timestamp_us'].iloc[0])
        for df in log.streams.values()
        if not df.empty and 'timestamp_us' in df.columns
    ]
    return min(firsts) if firsts else 0


def _filter_time_range(log, t0_us, time_range):
    """Return a FlightLog restricted to `time_range` (seconds, relative to
    `t0_us`), filtering EACH stream independently on its own
    `timestamp_us` -- this is where "time_range filters each stream
    individually" happens, so panel functions never need to know about
    `time_range` themselves.
    `time_range`（`t0_us` 基準の秒）に制限した FlightLog を返す。各
    ストリームを自身の `timestamp_us` で個別に絞る -- 「time_range は
    各ストリームを個別に絞る」はここで行うため、パネル関数側は
    `time_range` を意識しなくてよい。
    """
    if time_range is None:
        return log
    start, end = time_range
    filtered = {}
    for name, df in log.streams.items():
        if df.empty or 'timestamp_us' not in df.columns:
            filtered[name] = df
            continue
        t = _t(df, t0_us)
        filtered[name] = df[(t >= start) & (t <= end)]
    return sflog.FlightLog(meta=log.meta, schema=log.schema, streams=filtered)


# =============================================================================
# Quaternion -> Euler
# =============================================================================


def quat_to_euler_rad(qw, qx, qy, qz):
    """Convert a unit quaternion (w,x,y,z) to roll/pitch/yaw in RADIANS
    (numpy arrays), using StampFly's body-to-world convention (matches the
    firmware's ESKF attitude output, protocol/spec/flight_log.yaml
    `attitude.quat_*`).
    単位クォータニオン(w,x,y,z)をroll/pitch/yaw[rad]（numpy配列）に変換する。
    StampFly の body-to-world 規約（ファーム ESKF の姿勢出力、
    protocol/spec/flight_log.yaml の `attitude.quat_*` と一致）を使う。
    """
    qw, qx, qy, qz = (np.asarray(v, dtype=float) for v in (qw, qx, qy, qz))

    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# =============================================================================
# Panels (each: panel_xxx(ax, log, t0_us) -> bool)
# パネル（各関数の型: panel_xxx(ax, log, t0_us) -> bool）
# =============================================================================


@requires('imu')
def panel_rate_roll(ax, log, t0_us) -> bool:
    """Roll rate: imu.gyro_x [deg/s] solid, rate_ref.rate_ref_roll [deg/s]
    dashed if the 'rate_ref' stream is present.
    ロールレート: imu.gyro_x [deg/s]（実線）に、'rate_ref' があれば
    rate_ref.rate_ref_roll [deg/s]（破線）を重ねる。
    """
    imu = log.streams['imu']
    t = _t(imu, t0_us)
    ax.plot(t, imu['gyro_x'].to_numpy() * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro_x')
    if _has(log, 'rate_ref'):
        rr = log.streams['rate_ref']
        ax.plot(_t(rr, t0_us), rr['rate_ref_roll'].to_numpy() * RAD_TO_DEG, 'C1--',
                linewidth=LINE_WIDTH_REFERENCE, label='rate_ref_roll')
    ax.set_ylabel('Roll rate [deg/s]')
    ax.set_title('Roll Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA)
    ax.grid(True, alpha=0.3)
    return True


@requires('imu')
def panel_rate_pitch(ax, log, t0_us) -> bool:
    """Pitch rate: imu.gyro_y vs rate_ref.rate_ref_pitch, both [deg/s].
    ピッチレート: imu.gyro_y と rate_ref.rate_ref_pitch を [deg/s] で重ねる。
    """
    imu = log.streams['imu']
    t = _t(imu, t0_us)
    ax.plot(t, imu['gyro_y'].to_numpy() * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro_y')
    if _has(log, 'rate_ref'):
        rr = log.streams['rate_ref']
        ax.plot(_t(rr, t0_us), rr['rate_ref_pitch'].to_numpy() * RAD_TO_DEG, 'C1--',
                linewidth=LINE_WIDTH_REFERENCE, label='rate_ref_pitch')
    ax.set_ylabel('Pitch rate [deg/s]')
    ax.set_title('Pitch Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA)
    ax.grid(True, alpha=0.3)
    return True


@requires('imu')
def panel_rate_yaw(ax, log, t0_us) -> bool:
    """Yaw rate: imu.gyro_z vs rate_ref.rate_ref_yaw, both [deg/s].
    ヨーレート: imu.gyro_z と rate_ref.rate_ref_yaw を [deg/s] で重ねる。
    """
    imu = log.streams['imu']
    t = _t(imu, t0_us)
    ax.plot(t, imu['gyro_z'].to_numpy() * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro_z')
    if _has(log, 'rate_ref'):
        rr = log.streams['rate_ref']
        ax.plot(_t(rr, t0_us), rr['rate_ref_yaw'].to_numpy() * RAD_TO_DEG, 'C1--',
                linewidth=LINE_WIDTH_REFERENCE, label='rate_ref_yaw')
    ax.set_ylabel('Yaw rate [deg/s]')
    ax.set_title('Yaw Rate', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA)
    ax.grid(True, alpha=0.3)
    return True


@requires('attitude')
def panel_attitude(ax, log, t0_us) -> bool:
    """Attitude from the ESKF quaternion (roll/pitch/yaw [deg]), plus
    ctrl_ref's outer-loop angle_ref_roll/pitch [deg] as a dashed step, if
    the 'ctrl_ref' stream is present.
    ESKF クォータニオンから得た姿勢(roll/pitch/yaw[deg])に、'ctrl_ref' が
    あれば外側ループの angle_ref_roll/pitch[deg]（破線ステップ）を重ねる。
    """
    att = log.streams['attitude']
    t = _t(att, t0_us)
    roll, pitch, yaw = quat_to_euler_rad(
        att['quat_w'].to_numpy(), att['quat_x'].to_numpy(),
        att['quat_y'].to_numpy(), att['quat_z'].to_numpy(),
    )
    ax.plot(t, roll * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='roll')
    ax.plot(t, pitch * RAD_TO_DEG, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='pitch')
    ax.plot(t, yaw * RAD_TO_DEG, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='yaw')

    if _has(log, 'ctrl_ref'):
        cr = log.streams['ctrl_ref']
        t_cr = _t(cr, t0_us)
        ax.step(t_cr, cr['angle_ref_roll'].to_numpy() * RAD_TO_DEG, 'C0--',
                where='post', linewidth=LINE_WIDTH_REFERENCE, label='roll_ref')
        ax.step(t_cr, cr['angle_ref_pitch'].to_numpy() * RAD_TO_DEG, 'C3--',
                where='post', linewidth=LINE_WIDTH_REFERENCE, label='pitch_ref')

    ax.set_ylabel('Attitude [deg]')
    ax.set_title('Attitude (from quaternion)', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('imu')
def panel_accel(ax, log, t0_us) -> bool:
    """Body-frame acceleration x/y/z [m/s^2] (post-filter).
    機体座標系の加速度 x/y/z [m/s^2]（フィルタ後）。
    """
    imu = log.streams['imu']
    t = _t(imu, t0_us)
    ax.plot(t, imu['accel_x'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='accel_x')
    ax.plot(t, imu['accel_y'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='accel_y')
    ax.plot(t, imu['accel_z'].to_numpy(), 'C2-', linewidth=LINE_WIDTH_MEASURED, label='accel_z')
    ax.set_ylabel('Accel [m/s^2]')
    ax.set_title('Acceleration', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('imu')
def panel_gyro_raw(ax, log, t0_us) -> bool:
    """Pre-filter raw gyro rates x/y/z [deg/s] (no reference overlay -- for
    the 'sensors' group where only the raw signal matters).
    フィルタ前の生ジャイロレート x/y/z [deg/s]（指令値は重ねない -- 生信号
    だけを見たい 'sensors' グループ向け）。
    """
    imu = log.streams['imu']
    t = _t(imu, t0_us)
    ax.plot(t, imu['gyro_raw_x'].to_numpy() * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='gyro_raw_x')
    ax.plot(t, imu['gyro_raw_y'].to_numpy() * RAD_TO_DEG, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='gyro_raw_y')
    ax.plot(t, imu['gyro_raw_z'].to_numpy() * RAD_TO_DEG, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='gyro_raw_z')
    ax.set_ylabel('Gyro (raw) [deg/s]')
    ax.set_title('Raw Gyro', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('posvel')
def panel_position(ax, log, t0_us) -> bool:
    """ESKF-estimated position x/y/z [m], NED frame (z is positive down).
    ESKF が推定した位置 x/y/z [m]（NED 座標系、z は下向き正）。
    """
    pv = log.streams['posvel']
    t = _t(pv, t0_us)
    ax.plot(t, pv['pos_x'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='pos_x')
    ax.plot(t, pv['pos_y'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='pos_y')
    ax.plot(t, pv['pos_z'].to_numpy(), 'C2-', linewidth=LINE_WIDTH_MEASURED, label='pos_z (down)')
    ax.set_ylabel('Position [m]')
    ax.set_title('Position', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('posvel')
def panel_velocity(ax, log, t0_us) -> bool:
    """ESKF-estimated velocity x/y/z [m/s], NED frame.
    ESKF が推定した速度 x/y/z [m/s]（NED 座標系）。
    """
    pv = log.streams['posvel']
    t = _t(pv, t0_us)
    ax.plot(t, pv['vel_x'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='vel_x')
    ax.plot(t, pv['vel_y'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='vel_y')
    ax.plot(t, pv['vel_z'].to_numpy(), 'C2-', linewidth=LINE_WIDTH_MEASURED, label='vel_z')
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('Velocity', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires_any('motor', 'ctrl_ref')
def panel_motor_duty(ax, log, t0_us) -> bool:
    """Motor duty [0,1]: motor.csv (400 Hz, solid) if present, else
    ctrl_ref.csv's (50 Hz) duty columns as a step plot -- see
    `duty_source()`. ctrl_ref.total_thrust [N] (thick black) is overlaid
    whenever 'ctrl_ref' is present, regardless of which stream supplied
    the duty curves.
    モータ duty [0,1]: motor.csv（400Hz、実線）があればそれ、無ければ
    ctrl_ref.csv（50Hz）の duty 列をステップ線で描く（`duty_source()`
    参照）。ctrl_ref.total_thrust [N]（太い黒線）は duty の出所によらず
    'ctrl_ref' があれば重ねる。
    """
    source = duty_source(log)
    df = log.streams[source]
    t = _t(df, t0_us)
    if source == 'motor':
        for motor, color in zip(_MOTOR_NAMES, _MOTOR_COLORS):
            ax.plot(t, df[f'duty_{motor}'].to_numpy(), color + '-', linewidth=LINE_WIDTH_MEASURED, label=motor)
        title = 'Motor duty (motor.csv, 400 Hz)'
    else:
        for motor, color in zip(_MOTOR_NAMES, _MOTOR_COLORS):
            ax.step(t, df[f'duty_{motor}'].to_numpy(), color + '-', where='post',
                    linewidth=LINE_WIDTH_MEASURED, label=motor)
        title = 'Motor duty (ctrl_ref.csv, 50 Hz)'

    if _has(log, 'ctrl_ref'):
        cr = log.streams['ctrl_ref']
        ax.plot(_t(cr, t0_us), cr['total_thrust'].to_numpy(), 'k-',
                linewidth=LINE_WIDTH_THRUST, label='total_thrust [N]')

    ax.set_ylabel('duty [0-1] / thrust [N]')
    ax.set_title(title, fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=5)
    ax.grid(True, alpha=0.3)
    return True


@requires('ctrl_output')
def panel_ctrl_output(ax, log, t0_us) -> bool:
    """PRE-MIXER commanded thrust [N] (left axis) and torque [mN*m] (twin
    right axis), merged into a single legend.
    ミキサー手前の指令推力[N]（左軸）とトルク[mN·m]（右の双子軸）を、
    1つの凡例にまとめて描く。
    """
    co = log.streams['ctrl_output']
    t = _t(co, t0_us)
    ax.plot(t, co['thrust'].to_numpy(), 'k-', linewidth=LINE_WIDTH_THRUST, label='thrust [N]')
    ax.set_ylabel('Thrust [N]')

    ax_torque = ax.twinx()
    ax_torque.plot(t, co['torque_roll'].to_numpy() * NM_TO_MNM, 'C0-',
                   linewidth=LINE_WIDTH_MEASURED, label='torque_roll')
    ax_torque.plot(t, co['torque_pitch'].to_numpy() * NM_TO_MNM, 'C3-',
                   linewidth=LINE_WIDTH_MEASURED, label='torque_pitch')
    ax_torque.plot(t, co['torque_yaw'].to_numpy() * NM_TO_MNM, 'C2-',
                   linewidth=LINE_WIDTH_MEASURED, label='torque_yaw')
    ax_torque.set_ylabel('Torque [mN*m]')

    lines_l, labels_l = ax.get_legend_handles_labels()
    lines_r, labels_r = ax_torque.get_legend_handles_labels()
    ax.legend(lines_l + lines_r, labels_l + labels_r, loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=4)
    ax.set_title('Control Output (pre-mixer)', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    return True


@requires('pilot')
def panel_pilot(ax, log, t0_us) -> bool:
    """Pilot stick input throttle/roll/pitch/yaw [-1..1] (throttle is
    physically [0,1] but shares this axis), each as a step line (50 Hz,
    one value per RC frame).
    操縦スティック throttle/roll/pitch/yaw [-1..1]（throttle は物理的には
    [0,1] だが同じ軸で表示）をステップ線で描く（50Hz、無線1フレームに
    1値）。
    """
    df = log.streams['pilot']
    t = _t(df, t0_us)
    for col, color in zip(('throttle', 'roll', 'pitch', 'yaw'), ('k', 'C0', 'C3', 'C2')):
        ax.step(t, df[col].to_numpy(), color + '-', where='post', linewidth=LINE_WIDTH_MEASURED, label=col)
    ax.set_ylabel('Pilot input [-1..1]')
    ax.set_title('Pilot Input', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=4)
    ax.grid(True, alpha=0.3)
    return True


@requires_any('baro', 'tof_bottom', 'tof_front')
def panel_height(ax, log, t0_us) -> bool:
    """Altitude/distance from whichever of baro/tof_bottom/tof_front is
    present, each plotted at its OWN native timestamps with small markers
    (their 30-50 Hz rates stay legible at marker-per-sample density).
    baro/tof_bottom/tof_front のうち存在するものを、それぞれ自身の原時刻で
    小さいマーカー付きで描く（30〜50Hz は1標本1マーカーでも見やすい密度）。
    """
    drew_any = False
    for name, col, label, style in _HEIGHT_SOURCES:
        if _has(log, name):
            df = log.streams[name]
            ax.plot(_t(df, t0_us), df[col].to_numpy(), style, marker='.',
                    markersize=HEIGHT_MARKER_SIZE, linewidth=LINE_WIDTH_MEASURED, label=label)
            drew_any = True
    ax.set_ylabel('Distance [m]')
    ax.set_title('Height / Distance', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return drew_any


@requires('flow')
def panel_flow(ax, log, t0_us) -> bool:
    """Optical flow dx/dy [counts] (left axis) and quality (twin right
    axis).
    オプティカルフロー dx/dy [counts]（左軸）と quality（右の双子軸）。
    """
    df = log.streams['flow']
    t = _t(df, t0_us)
    ax.plot(t, df['dx'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='dx')
    ax.plot(t, df['dy'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='dy')
    ax.set_ylabel('Flow [counts]')

    ax_q = ax.twinx()
    ax_q.plot(t, df['quality'].to_numpy(), 'k-', linewidth=LINE_WIDTH_MEASURED, label='quality')
    ax_q.set_ylabel('Quality')

    lines_l, labels_l = ax.get_legend_handles_labels()
    lines_r, labels_r = ax_q.get_legend_handles_labels()
    ax.legend(lines_l + lines_r, labels_l + labels_r, loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.set_title('Optical Flow', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    return True


@requires('mag')
def panel_mag(ax, log, t0_us) -> bool:
    """Magnetometer x/y/z [uT], body frame.
    地磁気 x/y/z [uT]（機体座標系）。
    """
    df = log.streams['mag']
    t = _t(df, t0_us)
    ax.plot(t, df['x'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='mag_x')
    ax.plot(t, df['y'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='mag_y')
    ax.plot(t, df['z'].to_numpy(), 'C2-', linewidth=LINE_WIDTH_MEASURED, label='mag_z')
    ax.set_ylabel('Magnetic field [uT]')
    ax.set_title('Magnetometer', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('attitude')
def panel_gyro_bias_mode(ax, log, t0_us) -> bool:
    """Gyro bias x/y/z [deg/s] (left axis) plus ctrl_ref.flight_mode as a
    step line on a twin right axis, if 'ctrl_ref' is present. Merges both
    axes' legends into one box (like the pre-v1 plot_bias_and_mode()).
    ジャイロバイアス x/y/z [deg/s]（左軸）に、'ctrl_ref' があれば
    flight_mode をステップ線で右の双子軸に重ねる。両軸の凡例は1つの箱に
    まとめる（v1 移行前の plot_bias_and_mode() と同様）。
    """
    att = log.streams['attitude']
    t = _t(att, t0_us)
    ax.plot(t, att['gyro_bias_x'].to_numpy() * RAD_TO_DEG, 'C0-', linewidth=LINE_WIDTH_MEASURED, label='bias_x')
    ax.plot(t, att['gyro_bias_y'].to_numpy() * RAD_TO_DEG, 'C3-', linewidth=LINE_WIDTH_MEASURED, label='bias_y')
    ax.plot(t, att['gyro_bias_z'].to_numpy() * RAD_TO_DEG, 'C2-', linewidth=LINE_WIDTH_MEASURED, label='bias_z')
    ax.set_ylabel('Gyro bias [deg/s]')
    ax.set_title('Gyro Bias / Flight Mode', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.grid(True, alpha=0.3)

    lines, labels = ax.get_legend_handles_labels()
    if _has(log, 'ctrl_ref'):
        cr = log.streams['ctrl_ref']
        ax_mode = ax.twinx()
        ax_mode.step(_t(cr, t0_us), cr['flight_mode'].to_numpy(), 'k-',
                     where='post', linewidth=MODE_LINE_WIDTH, label='flight_mode')
        ax_mode.set_ylabel('flight_mode')
        lines_r, labels_r = ax_mode.get_legend_handles_labels()
        lines, labels = lines + lines_r, labels + labels_r
    ax.legend(lines, labels, loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=4)
    return True


@requires('attitude')
def panel_accel_bias(ax, log, t0_us) -> bool:
    """Accelerometer bias x/y/z [m/s^2], body frame (ESKF estimate).
    加速度バイアス x/y/z [m/s^2]（機体座標系、ESKF 推定値）。
    """
    att = log.streams['attitude']
    t = _t(att, t0_us)
    ax.plot(t, att['accel_bias_x'].to_numpy(), 'C0-', linewidth=LINE_WIDTH_MEASURED, label='bias_x')
    ax.plot(t, att['accel_bias_y'].to_numpy(), 'C3-', linewidth=LINE_WIDTH_MEASURED, label='bias_y')
    ax.plot(t, att['accel_bias_z'].to_numpy(), 'C2-', linewidth=LINE_WIDTH_MEASURED, label='bias_z')
    ax.set_ylabel('Accel bias [m/s^2]')
    ax.set_title('Accelerometer Bias', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.legend(loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=3)
    ax.grid(True, alpha=0.3)
    return True


@requires('status')
def panel_status(ax, log, t0_us) -> bool:
    """Battery voltage [V] (line+marker, 1 Hz, left axis) and current [mA]
    (twin right axis).
    バッテリ電圧[V]（線+マーカー、1Hz、左軸）と電流[mA]（右の双子軸）。
    """
    df = log.streams['status']
    t = _t(df, t0_us)
    ax.plot(t, df['voltage'].to_numpy(), 'C0-o', linewidth=LINE_WIDTH_MEASURED,
            markersize=STATUS_MARKER_SIZE, label='voltage')
    ax.set_ylabel('Voltage [V]')

    ax_i = ax.twinx()
    ax_i.plot(t, df['current_ma'].to_numpy(), 'C3-o', linewidth=LINE_WIDTH_MEASURED,
              markersize=STATUS_MARKER_SIZE, label='current_ma')
    ax_i.set_ylabel('Current [mA]')

    lines_l, labels_l = ax.get_legend_handles_labels()
    lines_r, labels_r = ax_i.get_legend_handles_labels()
    ax.legend(lines_l + lines_r, labels_l + labels_r, loc='upper right', fontsize=LEGEND_FONT_SIZE, framealpha=LEGEND_FRAME_ALPHA, ncol=2)
    ax.set_title('Battery Status', fontsize=PANEL_TITLE_FONT_SIZE)
    ax.grid(True, alpha=0.3)
    return True


# =============================================================================
# Panel groups selectable via --mode / mode 引数で選べるパネル群
# =============================================================================

PANEL_GROUPS = {
    'all': [
        panel_rate_roll, panel_rate_pitch, panel_rate_yaw,
        panel_attitude, panel_accel, panel_position, panel_velocity,
        panel_motor_duty, panel_ctrl_output, panel_pilot, panel_height,
        panel_flow, panel_mag, panel_gyro_bias_mode, panel_status,
    ],
    'attitude': [panel_rate_roll, panel_rate_pitch, panel_rate_yaw, panel_attitude],
    'sensors': [panel_accel, panel_gyro_raw, panel_height, panel_flow, panel_mag],
    'position': [panel_position, panel_velocity, panel_height, panel_pilot],
    'eskf': [panel_attitude, panel_position, panel_velocity, panel_gyro_bias_mode, panel_accel_bias],
}


# =============================================================================
# load_bundle() / render() / main()
# =============================================================================


def load_bundle(path) -> sflog.FlightLog:
    """Load a v1 bundle and print a short human-readable summary (bundle
    name, imu row count, duration, measured imu rate).
    v1 一式を読み込み、短い要約（一式名、imu 行数、飛行時間、実測 imu
    レート）を表示する。
    """
    log = sflog.load(path)
    print(f"Loaded bundle: {Path(path).name}")
    if _has(log, 'imu'):
        imu = log.streams['imu']
        rows = len(imu)
        duration_s = (int(imu['timestamp_us'].iloc[-1]) - int(imu['timestamp_us'].iloc[0])) / 1e6
        measured_hz = (rows - 1) / duration_s if rows > 1 and duration_s > 0 else float('nan')
        print(f"  imu: {rows} rows, {duration_s:.1f} s")
        print(f"  measured imu rate: {measured_hz:.1f} Hz")
    else:
        print("  Warning: no 'imu' stream in this bundle")
    return log



def _screen_size_px():
    """Best-effort logical screen size (width, height) in pixels for the
    active matplotlib backend; None if it cannot be determined (headless).
    現在の matplotlib バックエンドでの画面の論理サイズ (幅, 高さ) [px]。
    判定できなければ None（ヘッドレス等）。
    """
    import matplotlib
    backend = matplotlib.get_backend().lower()
    if "qt" in backend:
        try:
            from matplotlib.backends.qt_compat import QtWidgets
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
            geometry = app.primaryScreen().availableGeometry()
            return geometry.width(), geometry.height()
        except Exception:  # noqa: BLE001 -- fall through to Tk / None
            pass
    if "agg" == backend:
        return None
    try:
        import tkinter as tk
        root = tk.Tk()
        root.withdraw()
        size = (root.winfo_screenwidth(), root.winfo_screenheight())
        root.destroy()
        return size
    except Exception:  # noqa: BLE001 -- no Tk (or no display)
        return None


def _fit_figure_to_screen(fig) -> None:
    """Resize `fig` so its window fits the screen as opened (a fraction of
    the screen, leaving room for the title bar and toolbar). No-op when the
    screen size is unknown.
    開いた時点でウィンドウが画面に収まるよう `fig` の大きさを合わせる
    （タイトルバー・ツールバー分を残した画面の一定割合）。画面サイズが
    分からなければ何もしない。
    """
    size = _screen_size_px()
    if not size:
        return
    width_px, height_px = size
    dpi = fig.get_dpi()
    fig.set_size_inches(width_px * SCREEN_FIT_FRACTION_W / dpi,
                        height_px * SCREEN_FIT_FRACTION_H / dpi, forward=True)
    manager = getattr(fig.canvas, "manager", None)
    window = getattr(manager, "window", None)
    # Tk: place the window at the top-left so the whole figure is on screen.
    # Tk: 図全体が画面内に入るようウィンドウを左上に寄せる。
    if window is not None and hasattr(window, "geometry"):
        try:
            window.geometry("+0+0")
        except Exception:  # noqa: BLE001 -- not a Tk window
            pass


def render(log, title: str, save_path=None, show=True, time_range=None, mode='all',
           cols: int = DEFAULT_COLUMNS) -> int:
    """Render the panel group `mode` for `log` and return the number of
    panels actually drawn (0 if none of the group's required streams are
    present).
    `log` について `mode` のパネル群を描画し、実際に描いたパネル数を返す
    （そのグループが必要とするストリームが1つも無ければ0）。

    save_path: if given, saves the figure (dpi=150, bbox_inches='tight')
        and prints "Saved figure to ...".
    show: plt.show() if True, else plt.close(fig).
    time_range: (start, end) in seconds; see `_filter_time_range()`.
    cols: number of panel columns (1..MAX_COLUMNS); panels fill row by row,
        all share the time axis.
    cols: パネルの列数（1..MAX_COLUMNS）。行優先で並べ、時間軸は全パネルで共有。
    """
    if mode not in PANEL_GROUPS:
        print(f"Info: render() has no '{mode}' panel group; showing 'all' instead.")
        mode = 'all'

    t0_us = _bundle_t0_us(log)
    log = _filter_time_range(log, t0_us, time_range)

    panel_fns = [fn for fn in PANEL_GROUPS[mode] if fn.presence(log)]
    n_panels = len(panel_fns)
    if n_panels == 0:
        print(f"Warning: no panels to draw for mode '{mode}' -- required streams are all absent.")
        return 0

    cols = max(1, min(int(cols), MAX_COLUMNS, n_panels))
    rows = (n_panels + cols - 1) // cols
    fig_height = min(FIGURE_HEIGHT_PER_ROW_IN * rows, FIGURE_MAX_HEIGHT_IN)
    fig = plt.figure(figsize=(FIGURE_WIDTH_PER_COLUMN_IN * cols, fig_height),
                     constrained_layout=True)
    fig.suptitle(
        f"{title} - flight-log bundle (source={log.meta.get('source', '?')})",
        fontsize=FIGURE_TITLE_FONT_SIZE,
    )
    gs = GridSpec(rows, cols, figure=fig)

    # Row-major fill; every axis shares the time axis with the first one so
    # zooming/panning one panel moves them all.
    # 行優先で埋める。時間軸は先頭パネルと共有し、1 つを拡大・移動すると
    # 全パネルが追従する。
    axes = []
    for i, panel_fn in enumerate(panel_fns):
        ax = fig.add_subplot(gs[i // cols, i % cols], sharex=axes[0] if axes else None)
        panel_fn(ax, log, t0_us)
        axes.append(ax)

    # The bottom panel of each column gets the shared "Time [s]" label --
    # which panels those are depends on `mode`, stream presence and `cols`,
    # so panels never set it themselves.
    # 各列の最下段のパネルにだけ共有の "Time [s]" ラベルを付ける -- どれが
    # 最下段かは mode・ストリームの有無・列数次第なので、各パネル側では設定しない。
    for i, ax in enumerate(axes):
        is_bottom = (i + cols >= n_panels)
        if is_bottom:
            ax.set_xlabel('Time [s]')

    # Head-room above the data on every axis (twin axes included) so the
    # upper-right legends do not sit on the traces.
    # 全軸（右軸も含む）の上側に余白を取り、右上の凡例が線に被らないようにする。
    for ax in fig.axes:
        lo, hi = ax.get_ylim()
        if hi > lo:
            ax.set_ylim(lo, hi + LEGEND_HEADROOM_FRACTION * (hi - lo))

    if show:
        _fit_figure_to_screen(fig)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved figure to {save_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return n_panels


def main():
    parser = argparse.ArgumentParser(
        description='Visualize a StampFly flight-log v1 bundle (.sflog.zip or an extracted directory)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('bundle', help='Path to a .sflog.zip file or an extracted bundle directory')
    parser.add_argument('--mode', choices=list(PANEL_GROUPS.keys()), default='all',
                        help='Panel group to show (default: all)')
    parser.add_argument('--save', metavar='FILE', help='Save figure to file')
    parser.add_argument('--no-show', action='store_true', help="Don't display window")
    parser.add_argument('--time-range', nargs=2, type=float, metavar=('START', 'END'),
                        help='Time range to plot (seconds)')

    args = parser.parse_args()

    bundle_path = Path(args.bundle)
    if not bundle_path.exists():
        print(f"Error: bundle not found: {bundle_path}")
        sys.exit(1)

    log = load_bundle(bundle_path)
    time_range = tuple(args.time_range) if args.time_range else None
    n_panels = render(log, bundle_path.name, save_path=args.save, show=not args.no_show,
                      time_range=time_range, mode=args.mode)
    if n_panels == 0:
        sys.exit(1)


if __name__ == '__main__':
    main()
