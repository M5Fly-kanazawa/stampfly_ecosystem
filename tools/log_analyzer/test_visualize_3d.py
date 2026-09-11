#!/usr/bin/env python3
"""
test_visualize_3d.py - Tests for the 3D attitude/pose animation viewers
3D姿勢/ポーズ・アニメーション表示ツールのテスト

Covers the bundle-reading side of visualize_attitude_3d.py and
visualize_pose_3d.py (load_bundle_attitude/load_bundle_pose,
quat_to_euler_rad, build_animation) against the synthetic and real-vehicle
reference bundles from conftest.py. Drawing correctness (pixel content) is
not tested here -- build_animation is exercised headlessly (Agg backend,
few frames) only to prove it does not raise.
visualize_attitude_3d.py・visualize_pose_3d.py の一式読み込み側
（load_bundle_attitude/load_bundle_pose、quat_to_euler_rad、
build_animation）を、conftest.py の合成/実機基準一式で検証する。描画内容
（ピクセル）は対象外 -- build_animation はヘッドレス（Aggバックエンド、
少フレーム数）で例外を出さないことだけを確認する。

Usage:
    pytest test_visualize_3d.py
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行
import matplotlib.pyplot as plt  # noqa: E402

import conftest  # noqa: E402
import sflog  # noqa: E402
import visualize_attitude_3d  # noqa: E402
import visualize_pose_3d  # noqa: E402

# Frame count for headless build_animation smoke tests -- small enough to
# build fast, large enough to exercise the subsampling path.
# ヘッドレス build_animation スモークテスト用フレーム数 -- 高速に組み立て
# つつ、間引き処理を通す程度の大きさ。
SMOKE_TEST_FRAMES = 5

# Expected sample count for conftest's default 5s/400Hz synthetic bundle.
# conftest の既定 5秒/400Hz 合成一式が持つはずのサンプル数。
EXPECTED_SYNTHETIC_SAMPLES = conftest.N_SAMPLES

# Tolerance for "all-zero" Euler angles decoded from an identity quaternion.
# 単位クォータニオンから得る「ほぼゼロ」オイラー角の許容誤差。
EULER_ZERO_ATOL = 1e-9


def test_load_bundle_attitude_synthetic(synthetic_bundle):
    """Identity-quaternion synthetic bundle -> 2000 samples, all-zero Euler angles.
    単位クォータニオンの合成一式 -> 2000サンプル、オイラー角は全てほぼゼロ。
    """
    time_s, roll, pitch, yaw = visualize_attitude_3d.load_bundle_attitude(synthetic_bundle)

    assert len(time_s) == EXPECTED_SYNTHETIC_SAMPLES
    assert len(roll) == EXPECTED_SYNTHETIC_SAMPLES
    assert len(pitch) == EXPECTED_SYNTHETIC_SAMPLES
    assert len(yaw) == EXPECTED_SYNTHETIC_SAMPLES
    assert np.allclose(roll, 0.0, atol=EULER_ZERO_ATOL)
    assert np.allclose(pitch, 0.0, atol=EULER_ZERO_ATOL)
    assert np.allclose(yaw, 0.0, atol=EULER_ZERO_ATOL)


def test_load_bundle_pose_synthetic(synthetic_bundle):
    """Synthetic bundle's hover altitude -> pos_z == -0.5 m (NED) everywhere.
    合成一式のホバー高度 -> pos_z は NED で全て -0.5 m。
    """
    time_s, pos_xyz, roll, pitch, yaw = visualize_pose_3d.load_bundle_pose(synthetic_bundle)

    assert len(time_s) == EXPECTED_SYNTHETIC_SAMPLES
    assert pos_xyz.shape == (EXPECTED_SYNTHETIC_SAMPLES, 3)
    assert np.allclose(pos_xyz[:, 2], -conftest.HOVER_ALT_M)


def test_build_animation_attitude_reference(reference_bundle):
    """build_animation() on the real hover log builds a figure without raising.
    実機ホバーログで build_animation() が例外なく図を組み立てられる。
    """
    time_s, roll, pitch, yaw = visualize_attitude_3d.load_bundle_attitude(reference_bundle)
    fig, anim = visualize_attitude_3d.build_animation(
        time_s, roll, pitch, yaw, frames=SMOKE_TEST_FRAMES,
    )
    try:
        assert fig is not None
        assert anim is not None
    finally:
        plt.close(fig)


def test_build_animation_pose_reference(reference_bundle):
    """build_animation() on the real hover log builds a figure without raising.
    実機ホバーログで build_animation() が例外なく図を組み立てられる。
    """
    time_s, pos_xyz, roll, pitch, yaw = visualize_pose_3d.load_bundle_pose(reference_bundle)
    fig, anim = visualize_pose_3d.build_animation(
        time_s, pos_xyz, roll, pitch, yaw, frames=SMOKE_TEST_FRAMES,
    )
    try:
        assert fig is not None
        assert anim is not None
    finally:
        plt.close(fig)


def test_load_bundle_pose_missing_posvel_raises(tmp_path):
    """No `posvel` stream -> load_bundle_pose() raises ValueError (clear error).
    `posvel` ストリームが無ければ load_bundle_pose() は ValueError を送出する
    （明確なエラーで失敗する）。
    """
    log = conftest.build_synthetic_log()
    del log.streams['posvel']
    log.meta = sflog.make_meta(
        source='sim', tool_name='tools/log_analyzer/test_visualize_3d', tool_version='0.0.0',
        streams=log.streams,
    )
    bundle_path = tmp_path / "flight_no_posvel.sflog.zip"
    log.save(bundle_path)

    with pytest.raises(ValueError):
        visualize_pose_3d.load_bundle_pose(bundle_path)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
