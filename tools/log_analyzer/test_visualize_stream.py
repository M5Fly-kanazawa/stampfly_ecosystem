#!/usr/bin/env python3
"""
test_visualize_stream.py - Tests for the Data Stream CSV visualizer
Data Stream CSV 可視化ツールのテスト

Builds a synthetic StampFly flight-log v1 bundle (lib/sflog; docs/plans/
flight-log-format-plan.md) in memory and writes it out as an ALIGNED CSV
via sflog.aligned_to_csv() -- the "sf log convert --aligned" derived-file
path -- since `sf log wifi` no longer writes a merged Data Stream CSV
directly (that CSV is now always a derived product of a bundle). The one
column-naming difference visualize_stream.py itself still expects
(`motor_duty_*`, not v1's `duty_*`) is patched into the CSV by this helper
-- see the rename below for why. visualize_stream.py itself is NOT changed.
lib/sflog を使い、合成の StampFly フライトログ v1 一式（計画書参照）を
メモリ上に組み立て、sflog.aligned_to_csv() で整列 CSV として書き出す --
`sf log convert --aligned` が作る派生ファイルと同じ経路（`sf log wifi` は
もはやマージ済み Data Stream CSV を直接書かず、CSV は常に一式からの
派生物になったため）。visualize_stream.py が今も期待する唯一の列名の
違い（v1の `duty_*` ではなく `motor_duty_*`）は、このヘルパーが CSV に
後から適用する（理由は下のリネーム箇所を参照）。visualize_stream.py
自体は変更しない。

Usage:
    python3 test_visualize_stream.py
    pytest test_visualize_stream.py
"""

import csv
import math
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import visualize_extended  # noqa: E402
import visualize_stream  # noqa: E402

import sflog  # noqa: E402

# Minimum PNG size used as a smoke check that matplotlib actually rendered
# panels (an empty/failed figure saves far smaller than this).
# matplotlib が実際にパネルを描画したことを確認する下限サイズ（空/失敗図は
# これより大幅に小さく保存される）。
MIN_PNG_BYTES = 10_000

SAMPLE_RATE_HZ = 400
DURATION_S = 5
N_SAMPLES = SAMPLE_RATE_HZ * DURATION_S
DT_US = 1_000_000 // SAMPLE_RATE_HZ
CTRL_REF_STRIDE = 8  # 50Hz CtrlRef among 400Hz IMU+ESKF samples / 400Hz中50Hz

# aligned() merges v1's LOCKSTEP_STREAMS in schema.STREAMS order, and
# "motor" is declared before "ctrl_ref" there -- so motor.duty_FR keeps the
# bare name `duty_FR` in the aligned table (see align.py's docstring for the
# full collision rule) while ctrl_ref's own (50Hz, forward-filled) duty
# columns get renamed `ctrl_ref_duty_FR` etc. visualize_stream.py's plotting
# code still expects the OLD save_stream_csv() name `motor_duty_FR` for
# this (400Hz, motor-sourced) series -- rename it after the fact rather
# than touch visualize_stream.py.
# aligned() は v1 の LOCKSTEP_STREAMS を schema.STREAMS の並び順で結合し、
# "motor" は "ctrl_ref" より先に定義されている -- そのため整列表では
# motor.duty_FR がそのまま `duty_FR` の名前を保つ（衝突規則の詳細は
# align.py の docstring 参照）一方、ctrl_ref 自身の（50Hz前方補完の）
# duty 列は `ctrl_ref_duty_FR` 等に改名される。visualize_stream.py の
# 描画コードはこの（400Hz、motor由来の）系列に今も旧 save_stream_csv() の
# 名前 `motor_duty_FR` を期待するため、visualize_stream.py には触れず
# 事後にリネームする。
_DUTY_MOTORS = ('FR', 'RR', 'RL', 'FL')


def _build_stream_csv(tmp_path) -> Path:
    """Build a synthetic 5s/400Hz flight-log v1 bundle (roll rate_ref step)
    in memory and write its aligned table via sflog.aligned_to_csv();
    return the CSV path.
    5秒/400Hzの合成フライトログ v1 一式（ロールrate_refのステップ）を
    メモリ上に組み立て、sflog.aligned_to_csv() で整列表を書き出し、
    CSVパスを返す。"""
    ts = [1_000_000 + i * DT_US for i in range(N_SAMPLES)]
    seq = list(range(N_SAMPLES))
    gyro_x = [0.1 * math.sin(i / 50) for i in range(N_SAMPLES)]
    # Roll rate_ref step at t=1s: 0 -> 0.1 rad/s (already in v1's physical
    # units -- unlike the old wire-quantized save_stream_csv() input, v1
    # streams hold the converted value directly).
    # t=1sでロールrate_refをステップ: 0 -> 0.1 rad/s（v1の物理単位そのもの
    # -- 旧・電文量子化されていた save_stream_csv() の入力と異なり、v1
    # ストリームは変換済みの値を直接持つ）。
    rate_ref_roll = [0.1 if i >= N_SAMPLES // 2 else 0.0 for i in range(N_SAMPLES)]

    imu_df = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'gyro_x': gyro_x, 'gyro_y': 0.0, 'gyro_z': 0.0,
        'accel_x': 0.0, 'accel_y': 0.0, 'accel_z': -9.81,
        'gyro_raw_x': 0.0, 'gyro_raw_y': 0.0, 'gyro_raw_z': 0.0,
        'accel_raw_x': 0.0, 'accel_raw_y': 0.0, 'accel_raw_z': -9.81,
    })
    attitude_df = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'quat_w': 1.0, 'quat_x': 0.0, 'quat_y': 0.0, 'quat_z': 0.0,
        'gyro_bias_x': 0.0, 'gyro_bias_y': 0.0, 'gyro_bias_z': 0.0,
        'accel_bias_x': 0.0, 'accel_bias_y': 0.0, 'accel_bias_z': 0.0,
    })
    rate_ref_df = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'rate_ref_roll': rate_ref_roll, 'rate_ref_pitch': 0.0, 'rate_ref_yaw': 0.0,
    })
    motor_df = pd.DataFrame({
        'timestamp_us': ts, 'seq': seq,
        'duty_FR': 0.5, 'duty_RR': 0.5, 'duty_RL': 0.5, 'duty_FL': 0.5,
    })
    ctrl_ref_ts = ts[::CTRL_REF_STRIDE]
    ctrl_ref_df = pd.DataFrame({
        'timestamp_us': ctrl_ref_ts,
        'flight_mode': 1,
        'angle_ref_roll': 0.0, 'angle_ref_pitch': 0.0,
        'total_thrust': 0.5,
        'duty_FR': 0.5, 'duty_RR': 0.5, 'duty_RL': 0.5, 'duty_FL': 0.5,
        'alt_setpoint': 0.0, 'alt_vel_target': 0.0, 'climb_rate_cmd': 0.0,
        'pos_setpoint_x': 0.0, 'pos_setpoint_y': 0.0,
    })

    streams = {
        'imu': imu_df, 'attitude': attitude_df, 'rate_ref': rate_ref_df,
        'motor': motor_df, 'ctrl_ref': ctrl_ref_df,
    }
    meta = sflog.make_meta(
        source='sim', tool_name='test_visualize_stream', tool_version='0.0.0',
        streams=streams,
    )
    log = sflog.FlightLog(
        meta=meta, schema=sflog.schema.schema_for(streams.keys()), streams=streams,
    )

    csv_path = tmp_path / "stream.csv"
    sflog.aligned_to_csv(log, csv_path, base='imu')

    df = pd.read_csv(csv_path)
    df = df.rename(columns={f'duty_{m}': f'motor_duty_{m}' for m in _DUTY_MOTORS})
    df.to_csv(csv_path, index=False)

    return csv_path


def _read_header(csv_path):
    with open(csv_path, 'r') as f:
        return next(csv.reader(f))


def test_is_stream_csv_true_and_extended_branch_would_have_been_wrong(tmp_path):
    """The stream CSV header both (a) satisfies is_stream_csv(), and (b)
    contains timestamp_us + quat_w -- i.e. it would ALSO match the legacy
    "Extended telemetry" test in visualize_extended.load_csv(). Reproduces
    the ORIGINAL bug (KeyError 'gyro_corrected_x' is wrong -- the real
    failure is on the first column the extended branch reads that a stream
    CSV lacks) to document why run_viz() must check is_stream_csv() first.
    stream CSV のヘッダは (a) is_stream_csv() を満たし、かつ (b) timestamp_us +
    quat_w を含む -- つまり visualize_extended.load_csv() の旧
    「Extended telemetry」判定にも一致してしまう。元のバグを再現し、
    run_viz() が is_stream_csv() を先に判定すべき理由を示す。"""
    csv_path = _build_stream_csv(tmp_path)
    header = _read_header(csv_path)

    assert visualize_stream.is_stream_csv(header) is True
    assert 'timestamp_us' in header and 'quat_w' in header

    data, fmt = visualize_extended.load_csv(str(csv_path))
    assert fmt == 'extended'  # matches the legacy branch's own detection
    try:
        visualize_extended.plot_extended(data, output_file=None)
        raise AssertionError(
            "plot_extended() unexpectedly succeeded on a stream CSV -- the "
            "legacy branch should not be able to render this format")
    except KeyError:
        pass  # expected: stream CSV lacks the extended-format columns
    finally:
        import matplotlib.pyplot as plt
        plt.close('all')


def test_visualize_all_default_mode(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_all.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png), show=False)

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_visualize_all_attitude_mode(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_attitude.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png),
                                   show=False, mode='attitude')

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_visualize_all_time_range(tmp_path):
    df = visualize_stream.load_stream_csv(str(_build_stream_csv(tmp_path)))
    out_png = tmp_path / "viz_tr.png"
    visualize_stream.visualize_all(df, "stream.csv", save_path=str(out_png),
                                   show=False, time_range=(0.5, 1.5))

    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_legacy_extended_header_not_detected_as_stream():
    """A 45-column legacy "extended" header (gyro_raw_x, pos_x, ctrl_throttle,
    no rate_ref_roll) must NOT be detected as a Data Stream CSV.
    45列の旧「extended」ヘッダ（gyro_raw_x, pos_x, ctrl_throttle を含み
    rate_ref_roll を含まない）は Data Stream CSV として検出されてはならない。"""
    legacy_header = [
        'timestamp_us', 'timestamp_ms',
        'gyro_x', 'gyro_y', 'gyro_z',
        'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
        'accel_x', 'accel_y', 'accel_z',
        'accel_raw_x', 'accel_raw_y', 'accel_raw_z',
        'quat_w', 'quat_x', 'quat_y', 'quat_z',
        'pos_x', 'pos_y', 'pos_z',
        'vel_x', 'vel_y', 'vel_z',
        'baro_alt', 'tof_bottom', 'tof_front',
        'flow_x', 'flow_y',
        'mag_x', 'mag_y', 'mag_z',
        'ctrl_throttle', 'ctrl_roll', 'ctrl_pitch', 'ctrl_yaw',
        'battery_voltage', 'battery_current',
        'motor_1', 'motor_2', 'motor_3', 'motor_4',
        'flight_state', 'eskf_status', 'loop_time_us',
    ]
    assert len(legacy_header) == 45
    assert 'rate_ref_roll' not in legacy_header
    assert visualize_stream.is_stream_csv(legacy_header) is False


def _run_all():
    """Standalone runner using a throwaway tmp dir (mirrors pytest's tmp_path
    fixture for `python3 test_visualize_stream.py` execution).
    使い捨ての一時ディレクトリで実行するスタンドアロンランナー
    （`python3 test_visualize_stream.py` 実行時、pytest の tmp_path を模す）。"""
    import shutil
    import tempfile

    tmp_dir = Path(tempfile.mkdtemp(prefix="test_visualize_stream_"))
    tests = [
        (test_is_stream_csv_true_and_extended_branch_would_have_been_wrong, (tmp_dir,)),
        (test_visualize_all_default_mode, (tmp_dir,)),
        (test_visualize_all_attitude_mode, (tmp_dir,)),
        (test_visualize_all_time_range, (tmp_dir,)),
        (test_legacy_extended_header_not_detected_as_stream, ()),
    ]
    failures = 0
    try:
        for fn, fn_args in tests:
            try:
                fn(*fn_args)
                print(f"  [TEST] {fn.__name__:<55} PASS")
            except AssertionError as e:
                failures += 1
                print(f"  [TEST] {fn.__name__:<55} FAIL: {e}")
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    total = len(tests)
    print(f"\n=== Results: {total - failures}/{total} passed, {failures} failed ===")
    return failures


if __name__ == '__main__':
    sys.exit(1 if _run_all() > 0 else 0)
