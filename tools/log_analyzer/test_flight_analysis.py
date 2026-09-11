#!/usr/bin/env python3
"""
test_flight_analysis.py - Tests for the flight-log v1 bundle analyzer
test_flight_analysis.py - フライトログ v1 一式の解析処理のテスト

Exercises flight_analysis.py (the backend of `sf log analyze`, see
docs/plans/flight-log-format-plan.md section 3.2) against the two bundles
provided by tools/log_analyzer/conftest.py:
  * `synthetic_bundle`: every stream at native multi-rate cadence, with a
    known 400 Hz sample rate and a known gyro_x sine so the measured rate
    and dominant frequency can be checked against ground truth.
  * `reference_bundle`: a real 30 s hover capture with the firmware's
    documented ~13% repeated-IMU-timestamp behaviour (plan section 7),
    used to pin the exact row/repeat counts this module must reproduce.
An imu-only bundle and a bundle missing `imu` altogether cover the
required-vs-optional stream contract.

flight_analysis.py（`sf log analyze` のバックエンド、計画書3.2節参照）を、
tools/log_analyzer/conftest.py が提供する2つの一式に対して検証する:
  * `synthetic_bundle`: 全ストリームを原レートで持ち、400Hz という既知の
    標本化レートと既知の gyro_x 正弦波を持つため、実測レート・卓越周波数を
    真値と照合できる。
  * `reference_bundle`: 実機30秒ホバーの一式。ファームの既知の挙動である
    IMU タイムスタンプ再利用が約13%発生する（計画書7節）ため、本モジュールが
    再現すべき正確な行数・重複数を固定する。
imuのみの一式と imu が全く無い一式で、必須／任意ストリームの契約を確認する。

Usage:
    MPLBACKEND=Agg .venv/bin/python3 -m pytest tools/log_analyzer/test_flight_analysis.py -q
"""
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless / ヘッドレス実行

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import conftest  # noqa: E402
import flight_analysis  # noqa: E402

import sflog  # noqa: E402


def test_synthetic_bundle(synthetic_bundle, tmp_path):
    """Synthetic-bundle metrics: measured rate close to the true 400 Hz,
    zero repeated timestamps (synthetic imu never reuses a sample), the
    known gyro_x sine's frequency as the top roll-axis PSD peak, a real PNG,
    and the hover-window boundary case.
    合成一式の指標: 真の400Hzに近い実測レート、重複タイムスタンプ0件
    （合成 imu は標本を再利用しない）、既知の gyro_x 正弦波の周波数が
    roll軸 PSD の最上位ピークになること、実体のある PNG、ホバー区間の
    境界条件を確認する。
    """
    log = sflog.load(synthetic_bundle)
    png_path = tmp_path / "synthetic_analysis.png"
    result = flight_analysis.analyze_flight(log, "synthetic", png_path=str(png_path))

    assert abs(result['sample_rate_hz'] - conftest.SAMPLE_RATE_HZ) / conftest.SAMPLE_RATE_HZ < 0.01
    assert result['repeated_timestamps'] == 0

    # gyro_x = AMPLITUDE * sin(i / GYRO_SINE_PERIOD_SAMPLES): the argument's
    # angular frequency is 1/GYRO_SINE_PERIOD_SAMPLES rad per SAMPLE (not
    # per second), so converting to Hz needs the sample rate:
    #   omega_rad_s = (1 / PERIOD_SAMPLES) * SAMPLE_RATE_HZ
    #   f_hz = omega_rad_s / (2 * pi) = SAMPLE_RATE_HZ / (2 * pi * PERIOD_SAMPLES)
    # gyro_x = AMPLITUDE * sin(i / GYRO_SINE_PERIOD_SAMPLES): 引数の角周波数は
    # 1標本あたり 1/GYRO_SINE_PERIOD_SAMPLES ラジアン（1秒あたりではない）
    # なので、Hz へは標本化レートを介して変換する。
    expected_hz = conftest.SAMPLE_RATE_HZ / (2 * math.pi * conftest.GYRO_SINE_PERIOD_SAMPLES)
    top_roll_hz = result['dominant_hz']['x'][0][0]
    assert abs(top_roll_hz - expected_hz) < 0.5

    assert png_path.exists()
    assert png_path.stat().st_size > 10_000

    # All 4 synthetic duties equal conftest.HOVER_DUTY (0.5) exactly, so
    # their sum equals flight_analysis.HOVER_DUTY_SUM (2.0) exactly.
    # _find_hover_window() requires duty sum > HOVER_DUTY_SUM (strictly
    # greater, matching motor_health.py's _hover_window and this repo's
    # existing test_motor_health.py, which documents this same synthetic
    # bundle as sitting "exactly ON the HOVER_DUTY_SUM boundary (not above
    # it), which must NOT be classified as hover") -- a sum that only ever
    # equals the threshold never crosses it, so this bundle can never
    # produce a hover window.
    # 合成データの4つの duty は全て conftest.HOVER_DUTY(0.5)ちょうどなので
    # 合計は flight_analysis.HOVER_DUTY_SUM(2.0)に厳密に一致する。
    # _find_hover_window() は「duty合計 > HOVER_DUTY_SUM」(厳密に超える。
    # motor_health.py の _hover_window、および本リポジトリの既存
    # test_motor_health.py が同じ合成一式を「HOVER_DUTY_SUM の境界上ちょうど
    # （その上ではない）にあり、ホバーと判定してはならない」と明記している
    # のと同じ規約)を要求するため、閾値と等しいだけの合計は条件を超えられず、
    # この一式は原理的にホバー区間を検出できない。
    assert result['hover_window_s'] is None


def test_reference_bundle(reference_bundle, tmp_path):
    """Real-vehicle 30 s hover bundle: exact row/repeat counts (plan
    section 7's documented ~13% repeated-IMU-sample behaviour), a plausible
    measured rate, the optional streams that ARE present, and a real PNG.
    実機の30秒ホバー一式: 行数・重複数の厳密一致（計画書7節が記す約13%の
    IMU標本再利用）、妥当な実測レート、実在するオプションストリーム、
    実体のある PNG を確認する。
    """
    log = sflog.load(reference_bundle)
    png_path = tmp_path / "reference_analysis.png"
    result = flight_analysis.analyze_flight(log, "reference", png_path=str(png_path))

    assert result['rows'] == 11736
    assert result['repeated_timestamps'] == 1561
    assert 380.0 < result['sample_rate_hz'] < 400.0
    assert result['attitude'] is not None
    assert result['position'] is not None
    assert result['pilot'] is not None

    assert png_path.exists()
    assert png_path.stat().st_size > 10_000


def test_imu_only_bundle():
    """A bundle with only the required `imu` stream must not raise, and
    every optional-stream result must be None (no attitude/posvel/pilot/
    ctrl_ref/motor/status to compute them from).
    必須の imu ストリームだけの一式でも例外を起こさず、任意ストリーム由来の
    結果（attitude/posvel/pilot/ctrl_ref/motor/status が無い）は全て None に
    なることを確認する。
    """
    log = conftest.build_synthetic_log()
    log.streams = {'imu': log.streams['imu']}
    log.meta = sflog.make_meta(source='sim', tool_name='test_flight_analysis',
                               tool_version='0.0.0', streams=log.streams)
    log.schema = sflog.schema.schema_for(log.streams.keys())

    result = flight_analysis.analyze_flight(log, "imu_only", png_path=None)

    assert result['attitude'] is None
    assert result['position'] is None
    assert result['pilot'] is None
    assert result['hover_window_s'] is None
    assert result['png'] is None


def test_missing_imu_raises():
    """A bundle without an `imu` stream is not analyzable -- analyze_flight()
    must raise ValueError up front rather than fail deep in the pipeline on
    a missing column.
    imu ストリームの無い一式は解析できない -- analyze_flight() は
    パイプライン奥で列不在により失敗するのではなく、最初に ValueError を
    送出しなければならない。
    """
    log = conftest.build_synthetic_log()
    log.streams = {k: v for k, v in log.streams.items() if k != 'imu'}

    with pytest.raises(ValueError):
        flight_analysis.analyze_flight(log, "no_imu", png_path=None)
