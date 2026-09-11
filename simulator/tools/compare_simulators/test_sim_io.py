"""
test_sim_io.py - Tests for sim_io.py's StampFly flight-log v1 bundle
read/write (save_output_bundle / load_output_bundle), added 2026-09-11
when the simulator comparison tool's plain `#`-comment CSV format was
replaced by the flight-log bundle (docs/plans/flight-log-format-plan.md
section 3.1 "sf sim headless"; simulator/tools/compare_simulators/sim_io.py).

sim_io.py の StampFly フライトログ v1 一式読み書き
（save_output_bundle / load_output_bundle）のテスト。シミュレータ比較ツールの
`#` コメント付き素の CSV 形式をフライトログ一式へ置き換えた際に追加
（2026-09-11、計画書 3.1節参照）。
"""

import math
import sys
from pathlib import Path

import pytest

_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from sim_io import (  # noqa: E402
    ControlInput,
    StateLog,
    _genesis_euler_to_ned,
    _genesis_vec_to_ned,
    load_output_bundle,
    save_output_bundle,
)

import sflog  # noqa: E402 (sim_io.py inserts lib/ onto sys.path on import)

TOLERANCE = 1e-6


def _make_synthetic_logs(n=6, dt=0.01):
    """A short, non-degenerate synthetic trajectory: distinct, non-zero
    values on every field (including velocity) and Euler angles far from
    the pitch = +-90 deg gimbal-lock singularity, so quat_to_euler() must
    recover them exactly (within floating-point error) for the round trip
    to prove anything.
    短い非退化の合成軌道: 全フィールド（速度含む）に個別の非ゼロ値を持ち、
    オイラー角は pitch = ±90度のジンバルロック特異点から十分離す
    （quat_to_euler() が厳密に復元できないと往復テストの意味がないため）。
    """
    logs = []
    for i in range(n):
        t = i * dt
        logs.append(StateLog(
            time=t,
            x=1.0 + 0.1 * i, y=-2.0 + 0.2 * i, z=-0.5 - 0.05 * i,
            roll=0.05 * i, pitch=0.03 * i - 0.1, yaw=0.02 * i + 0.2,
            p=0.01 * i, q=-0.02 * i, r=0.005 * i,
            vx=0.5 + 0.01 * i, vy=-0.3 + 0.02 * i, vz=0.1 - 0.01 * i,
        ))
    return logs


def _make_synthetic_inputs(n=4, dt=0.02):
    return [
        ControlInput(time=i * dt, throttle=0.1 * i, roll=0.2 * i, pitch=-0.1 * i, yaw=0.05 * i)
        for i in range(n)
    ]


# =============================================================================
# Round trip: save_output_bundle -> load_output_bundle (vpython backend --
# see sim_io.py's frame-conversion comment: vpython's native frame is
# already NED, so this backend's conversion is the identity and a genuine
# round trip is expected on every field, not just up to a documented
# coordinate remap. Genesis's remap is a DIFFERENT transform tested
# separately below (test_genesis_frame_conversion*) rather than folded
# into this round trip, since load_output_bundle() always returns NED and
# the genesis-native input values are therefore NOT expected to reappear
# unchanged.
# 往復テスト: save_output_bundle -> load_output_bundle（vpython バックエンド
# -- sim_io.py の座標変換コメントの通り、vpython のネイティブ座標系は
# 既に NED のため恒等変換であり、全フィールドで真の往復が成立する。
# Genesis の座標変換は別物であり、下記の test_genesis_frame_conversion*
# で独立に検証する（load_output_bundle() は常に NED を返すため、
# Genesis ネイティブ値をそのまま入力してもこの往復では一致しない）。
# =============================================================================


def test_round_trip_positions_and_velocities(tmp_path):
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {
        "simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
        "input_file": "test.csv", "duration_s": 0.05,
    }
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    loaded_logs, _meta = load_output_bundle(bundle_path)
    assert len(loaded_logs) == len(logs)

    for original, loaded in zip(logs, loaded_logs):
        assert loaded.time == pytest.approx(original.time, abs=TOLERANCE)
        assert loaded.x == pytest.approx(original.x, abs=TOLERANCE)
        assert loaded.y == pytest.approx(original.y, abs=TOLERANCE)
        assert loaded.z == pytest.approx(original.z, abs=TOLERANCE)
        assert loaded.vx == pytest.approx(original.vx, abs=TOLERANCE)
        assert loaded.vy == pytest.approx(original.vy, abs=TOLERANCE)
        assert loaded.vz == pytest.approx(original.vz, abs=TOLERANCE)


def test_round_trip_euler_and_rates(tmp_path):
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
                "input_file": None, "duration_s": 0.05}
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    loaded_logs, _meta = load_output_bundle(bundle_path)

    for original, loaded in zip(logs, loaded_logs):
        # Euler angles round-trip through euler_to_quat()/quat_to_euler() --
        # not stored directly, so this also exercises the quaternion math.
        # オイラー角は euler_to_quat()/quat_to_euler() を経由して往復する
        # （直接保存しないため、クォータニオン変換自体も検証する）。
        assert loaded.roll == pytest.approx(original.roll, abs=TOLERANCE)
        assert loaded.pitch == pytest.approx(original.pitch, abs=TOLERANCE)
        assert loaded.yaw == pytest.approx(original.yaw, abs=TOLERANCE)
        assert loaded.p == pytest.approx(original.p, abs=TOLERANCE)
        assert loaded.q == pytest.approx(original.q, abs=TOLERANCE)
        assert loaded.r == pytest.approx(original.r, abs=TOLERANCE)


def test_round_trip_timestamps_exact(tmp_path):
    """timestamp_us is an EXACT integer round trip (int64 CSV column, no
    float rounding on the way back) -- unlike the other fields, this must
    match bit-for-bit, not just within a tolerance.
    timestamp_us は整数として厳密に往復する（int64 の CSV 列。読み戻しに
    浮動小数点の丸めが入らない）-- 他のフィールドと異なり、許容誤差ではなく
    完全一致でなければならない。
    """
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
                "input_file": None, "duration_s": 0.05}
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    log_obj = sflog.FlightLog.load(bundle_path)
    truth = log_obj.streams["truth"]
    expected_ts = [int(round(log.time * 1e6)) for log in logs]
    assert list(truth["timestamp_us"]) == expected_ts


def test_pilot_csv_matches_inputs(tmp_path):
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
                "input_file": None, "duration_s": 0.05}
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    log_obj = sflog.FlightLog.load(bundle_path)
    pilot = log_obj.streams["pilot"]
    assert len(pilot) == len(inputs)

    for original, row in zip(inputs, pilot.itertuples(index=False)):
        assert int(row.timestamp_us) == int(round(original.time * 1e6))
        assert row.throttle == pytest.approx(original.throttle, abs=TOLERANCE)
        assert row.roll == pytest.approx(original.roll, abs=TOLERANCE)
        assert row.pitch == pytest.approx(original.pitch, abs=TOLERANCE)
        assert row.yaw == pytest.approx(original.yaw, abs=TOLERANCE)


def test_meta_source_is_sim(tmp_path):
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
                "input_file": "hover", "duration_s": 0.05}
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    log_obj = sflog.FlightLog.load(bundle_path)
    assert log_obj.meta["source"] == "sim"
    assert log_obj.meta["format"] == sflog.schema.FORMAT
    assert "vpython" in log_obj.meta["notes"]
    assert "hover" in log_obj.meta["notes"]


def test_check_bundle_reports_no_errors(tmp_path):
    """`sflog.check_bundle()` must not flag anything this module writes
    (units, timestamp monotonicity, schema.json presence, required
    streams -- a `sim` bundle is required to carry only truth.csv, see
    `required_streams` in protocol/spec/flight_log.yaml).
    `sflog.check_bundle()` は本モジュールが書くもの（単位、timestamp の
    単調性、schema.json の有無、必須ストリーム -- `sim` 一式に必須なのは
    truth.csv だけ。protocol/spec/flight_log.yaml の `required_streams`
    参照）について何も指摘してはならない。
    """
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "vpython", "physics_hz": 2000, "control_hz": 400,
                "input_file": None, "duration_s": 0.05}
    bundle_path = tmp_path / "vpython_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    findings = sflog.check_bundle(bundle_path)
    errors = [f for f in findings if f.level == "error"]
    assert errors == [], [str(f) for f in errors]


# =============================================================================
# Genesis native-frame -> NED conversion (unit-level, not a round trip --
# load_output_bundle() always returns NED, so testing the mapping itself
# needs the raw functions, not a save/load cycle; see the round-trip
# section's docstring above)
# Genesis ネイティブ座標系 -> NED 変換（往復ではなく単体テスト --
# load_output_bundle() は常に NED を返すため、変換そのものの検証には
# 生の変換関数が要る。上の往復テストのdocstring参照）
# =============================================================================


def test_genesis_vec_to_ned_matches_documented_mapping():
    # Genesis (X-right, Y-forward, Z-up) -> NED (X-forward, Y-right, Z-down):
    # a unit vector along each Genesis axis lands where the mapping table
    # in docs/architecture/coordinate-systems.md says it should.
    assert _genesis_vec_to_ned((1.0, 0.0, 0.0)) == (0.0, 1.0, -0.0)  # Genesis +X -> NED +Y
    assert _genesis_vec_to_ned((0.0, 1.0, 0.0)) == (1.0, 0.0, -0.0)  # Genesis +Y -> NED +X
    assert _genesis_vec_to_ned((0.0, 0.0, 1.0)) == (0.0, 0.0, -1.0)  # Genesis +Z -> NED -Z


def test_genesis_vec_to_ned_is_involutory():
    """Applying the swap-and-negate transform twice must return the
    original vector (it is its own inverse) -- a structural sanity check
    independent of the specific values above.
    入れ替え＋符号反転の変換を2回適用すると元のベクトルに戻る（自己逆変換）
    -- 上記の具体値とは独立した構造的な健全性チェック。
    """
    vec = (1.23, -4.56, 7.89)
    twice = _genesis_vec_to_ned(_genesis_vec_to_ned(vec))
    for a, b in zip(vec, twice):
        assert a == pytest.approx(b, abs=TOLERANCE)


def test_genesis_euler_to_ned_matches_documented_mapping():
    roll, pitch, yaw = 0.1, 0.2, 0.3
    ned_roll, ned_pitch, ned_yaw = _genesis_euler_to_ned(roll, pitch, yaw)
    assert ned_roll == pytest.approx(pitch, abs=TOLERANCE)
    assert ned_pitch == pytest.approx(roll, abs=TOLERANCE)
    assert ned_yaw == pytest.approx(-yaw, abs=TOLERANCE)


def test_save_output_bundle_genesis_backend_does_not_crash(tmp_path):
    """A minimal end-to-end smoke test for the 'genesis' backend path
    through save_output_bundle()/load_output_bundle() -- exercises
    _state_log_to_ned()'s genesis branch (not covered by the vpython-
    backend round trip above), without asserting frame-mapped values
    (those are covered unit-level above).
    'genesis' バックエンド経路の最小限のスモークテスト --
    save_output_bundle()/load_output_bundle() 経由で _state_log_to_ned() の
    genesis 分岐を通す（上の vpython 往復テストではカバーされない）。
    座標変換後の値そのものは上の単体テストで検証済みのためここでは
    確認しない。
    """
    logs = _make_synthetic_logs()
    inputs = _make_synthetic_inputs()
    metadata = {"simulator": "genesis", "physics_hz": 2000, "control_hz": 400,
                "input_file": None, "duration_s": 0.05}
    bundle_path = tmp_path / "genesis_test.sflog.zip"
    save_output_bundle(bundle_path, logs, inputs, metadata)

    loaded_logs, meta = load_output_bundle(bundle_path)
    assert len(loaded_logs) == len(logs)
    assert "genesis" in meta["notes"]
    # Position X after the mapping is the ORIGINAL y (see
    # _genesis_vec_to_ned): a spot check that the genesis branch actually
    # ran (not the vpython identity path).
    # 変換後の位置Xは元のy（_genesis_vec_to_ned 参照）-- genesis 分岐が
    # 実際に通った（vpython の恒等変換ではない）ことの抜き取り確認。
    assert loaded_logs[0].x == pytest.approx(logs[0].y, abs=TOLERANCE)
    assert not math.isnan(loaded_logs[0].vx)
