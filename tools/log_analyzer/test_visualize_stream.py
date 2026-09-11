#!/usr/bin/env python3
"""
test_visualize_stream.py - Tests for the flight-log v1 bundle renderer
test_visualize_stream.py - フライトログ v1 一式描画処理のテスト

Exercises visualize_stream.py (the backend of `sf log viz`, see
docs/plans/flight-log-format-plan.md section 3.2) against two bundles
provided by tools/log_analyzer/conftest.py:
  * `synthetic_bundle`: every stream a renderer might read, at native
    multi-rate cadence (fixture built from `build_synthetic_log()`).
  * `reference_bundle`: a real 30 s hover capture that is MISSING
    motor/ctrl_output/tof_front -- this is what proves panel_motor_duty's
    motor/ctrl_ref fallback and the group-filtering-by-presence logic
    actually work on real data, not just a bundle built to have everything.
visualize_stream.py（`sf log viz` の実体、計画書 3.2節参照）を、
tools/log_analyzer/conftest.py が提供する2つの一式に対して検証する:
  * `synthetic_bundle`: 描画処理が読み得る全ストリームを原レートで持つ
    一式（`build_synthetic_log()` から組み立てたフィクスチャ）。
  * `reference_bundle`: motor/ctrl_output/tof_front を欠く実機30秒
    ホバーの一式 -- panel_motor_duty の motor/ctrl_ref フォールバックと
    「存在するストリームでグループを絞る」ロジックが、都合よく全部
    揃った一式だけでなく実データでも機能することを裏付ける。

Usage:
    pytest test_visualize_stream.py
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import conftest  # noqa: E402
import visualize_stream  # noqa: E402

import sflog  # noqa: E402

# Minimum PNG size used as a smoke check that matplotlib actually rendered
# panels (an empty/failed figure saves far smaller than this).
# matplotlib が実際にパネルを描画したことを確認する下限サイズ（空/失敗図は
# これより大幅に小さく保存される）。
MIN_PNG_BYTES = 10_000


def test_render_all_mode_synthetic_bundle(synthetic_bundle, tmp_path):
    """mode='all' on the synthetic bundle (every stream present) draws
    every panel PANEL_GROUPS['all'] defines.
    合成一式（全ストリーム有り）での mode='all' は、PANEL_GROUPS['all'] が
    定義する全パネルを描く。"""
    log = visualize_stream.load_bundle(synthetic_bundle)
    out_png = tmp_path / "viz_all.png"

    n_panels = visualize_stream.render(log, "synthetic", save_path=str(out_png), show=False, mode='all')

    assert n_panels == len(visualize_stream.PANEL_GROUPS['all'])
    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_render_other_modes_synthetic_bundle(synthetic_bundle, tmp_path):
    """Every non-'all' panel group also draws its full panel list on the
    synthetic bundle, since every stream those groups need is present.
    'all' 以外の各パネル群も、必要とするストリームが全て存在するため、
    合成一式では自分の全パネルを描く。"""
    log = visualize_stream.load_bundle(synthetic_bundle)

    for mode in ('attitude', 'sensors', 'position', 'eskf'):
        out_png = tmp_path / f"viz_{mode}.png"
        n_panels = visualize_stream.render(log, "synthetic", save_path=str(out_png), show=False, mode=mode)

        assert n_panels == len(visualize_stream.PANEL_GROUPS[mode]), mode
        assert out_png.exists()
        assert out_png.stat().st_size > MIN_PNG_BYTES


def test_render_time_range(synthetic_bundle, tmp_path):
    """A time_range still draws every 'all' panel (the synthetic bundle's
    streams all span the requested window) and does not raise.
    time_range を指定しても（合成一式の全ストリームが指定区間を覆うため）
    'all' の全パネルが描け、例外も起きない。"""
    log = visualize_stream.load_bundle(synthetic_bundle)
    out_png = tmp_path / "viz_time_range.png"

    n_panels = visualize_stream.render(log, "synthetic", save_path=str(out_png), show=False,
                                       time_range=(0.5, 1.5), mode='all')

    assert n_panels == len(visualize_stream.PANEL_GROUPS['all'])
    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_render_imu_only_bundle(tmp_path):
    """A bundle with only the 'imu' stream renders mode='all' with exactly
    the panels that need nothing else: rate x3 + accel.
    'imu' ストリームしか持たない一式で mode='all' を描くと、他に何も
    要らないパネル（レートx3 + 加速度）だけが描かれる。"""
    log = conftest.build_synthetic_log()
    log.streams = {'imu': log.streams['imu']}
    log.meta = sflog.make_meta(
        source='sim', tool_name='test_visualize_stream', tool_version='0.0.0', streams=log.streams,
    )
    log.schema = sflog.schema.schema_for(log.streams.keys())
    bundle_path = tmp_path / "imu_only.sflog.zip"
    log.save(bundle_path)

    loaded = visualize_stream.load_bundle(bundle_path)
    out_png = tmp_path / "imu_only.png"
    n_panels = visualize_stream.render(loaded, "imu_only", save_path=str(out_png), show=False, mode='all')

    # rate_roll, rate_pitch, rate_yaw, accel -- every other 'all' panel
    # requires a stream this bundle does not have.
    # rate_roll, rate_pitch, rate_yaw, accel -- 'all' の他のパネルは全て
    # この一式に無いストリームを必要とする。
    assert n_panels == 4
    assert out_png.exists()


def test_reference_bundle_renders_and_duty_source(reference_bundle, synthetic_bundle, tmp_path):
    """The real-vehicle reference bundle (no motor/ctrl_output/tof_front)
    renders mode='all' to a real PNG, and panel_motor_duty's fallback is
    verified directly via duty_source() -- 'ctrl_ref' here (no 'motor'
    stream) vs 'motor' on the synthetic bundle (which has both).
    実機由来の基準一式（motor/ctrl_output/tof_front 無し）で mode='all' が
    実際に PNG を描き、panel_motor_duty のフォールバックを duty_source()
    で直接検証する -- こちらは 'ctrl_ref'（'motor' ストリームが無い）、
    合成一式（両方ある）は 'motor'。"""
    ref_log = visualize_stream.load_bundle(reference_bundle)
    syn_log = visualize_stream.load_bundle(synthetic_bundle)

    assert visualize_stream.duty_source(ref_log) == 'ctrl_ref'
    assert visualize_stream.duty_source(syn_log) == 'motor'

    out_png = tmp_path / "reference_all.png"
    n_panels = visualize_stream.render(ref_log, "reference", save_path=str(out_png), show=False, mode='all')

    assert n_panels > 0
    assert out_png.exists()
    assert out_png.stat().st_size > MIN_PNG_BYTES


def test_quat_to_euler_rad_identity_and_yaw90():
    """Identity quaternion -> zero roll/pitch/yaw; a 90 degree yaw
    quaternion -> yaw = pi/2, roll = pitch = 0.
    単位クォータニオン -> roll/pitch/yaw は全て0。90度ヨー回転の
    クォータニオン -> yaw = pi/2、roll = pitch = 0。"""
    roll, pitch, yaw = visualize_stream.quat_to_euler_rad(
        np.array([1.0]), np.array([0.0]), np.array([0.0]), np.array([0.0]),
    )
    assert np.allclose(roll, 0.0)
    assert np.allclose(pitch, 0.0)
    assert np.allclose(yaw, 0.0)

    half_angle = math.pi / 4  # 90 deg yaw: qw=cos(45deg), qz=sin(45deg)
    qw, qz = math.cos(half_angle), math.sin(half_angle)
    roll, pitch, yaw = visualize_stream.quat_to_euler_rad(
        np.array([qw]), np.array([0.0]), np.array([0.0]), np.array([qz]),
    )
    assert np.allclose(roll, 0.0, atol=1e-9)
    assert np.allclose(pitch, 0.0, atol=1e-9)
    assert np.allclose(yaw, math.pi / 2, atol=1e-9)
