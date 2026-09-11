"""
test_convert.py - JSONL <-> bundle round trip (convert.jsonl_to_bundle,
convert.bundle_to_jsonl), plus an opportunistic integration test against a
real capture if one exists locally under logs/.
test_convert.py - JSONL <-> 一式の往復（convert.jsonl_to_bundle,
convert.bundle_to_jsonl）。加えて logs/ 配下に実キャプチャがあれば、
それを使った統合テストも行う（無ければスキップ）。
"""

from __future__ import annotations

import glob
import json
import os
from pathlib import Path

import pytest

from sflog.check import check_bundle, is_ok
from sflog.convert import bundle_to_jsonl, jsonl_to_bundle

REPO_ROOT = Path(__file__).resolve().parents[3]

# One synthetic record per id `udp_capture.py`'s save_jsonl() ever writes
# (see JSONL_FORMAT in tools/log_analyzer/udp_capture.py). Numeric values
# for fields that jsonl_to_bundle()/bundle_to_jsonl() round (ctrl_ref's
# total_thrust/motor_duty/alt_*/climb_cmd/pos_sp, duty400's duty, status's
# voltage/current_ma) are pre-rounded to the same precision so the round
# trip is exact -- rounding an already-rounded value is a no-op.
# `udp_capture.py` の save_jsonl() が書く全ての id（同ファイルの
# JSONL_FORMAT 参照）につき合成レコードを1件ずつ用意する。丸めが入る
# フィールド（ctrl_ref の total_thrust/motor_duty/alt_*/climb_cmd/pos_sp、
# duty400 の duty、status の voltage/current_ma）は、往復が正確に一致する
# よう事前に同じ桁数へ丸めてある（丸め済みの値を丸め直しても値は変わらない）。
SYNTHETIC_RECORDS = [
    {
        "id": "imu", "ts": 1_000_000,
        "gyro": [0.01, 0.02, -0.03],
        "accel": [0.1, -0.2, -9.81],
        "gyro_raw": [0.011, 0.021, -0.031],
        "accel_raw": [0.101, -0.201, -9.809],
        "quat": [0.999, 0.01, 0.02, -0.03],
        "gyro_bias": [-0.001, 0.002, -0.0015],
        "accel_bias": [0.01, -0.02, 0.03],
    },
    {
        "id": "posvel", "ts": 1_000_000,
        "pos": [1.0, 2.0, -3.0],
        "vel": [0.1, -0.2, 0.05],
    },
    {"id": "ctrl", "ts": 999_800, "throttle": 0.55, "roll": -0.1, "pitch": 0.2, "yaw": 0.0},
    {"id": "flow", "ts": 1_001_000, "dx": 5, "dy": -3, "quality": 120},
    {"id": "tof_b", "ts": 1_002_000, "distance": 0.523, "status": 0},
    {"id": "tof_f", "ts": 1_002_500, "distance": 1.204, "status": 0},
    {"id": "baro", "ts": 1_000_500, "altitude": 12.34, "pressure": 923.45},
    {"id": "mag", "ts": 1_000_700, "x": 10.5, "y": -20.25, "z": 30.0},
    {
        "id": "ctrl_ref", "ts": 999_800,
        "mode": 2,
        "angle_ref": [0.0123, -0.0456],
        "total_thrust": 1.2345,
        "motor_duty": [0.1, 0.2, 0.3, 0.4],
        "alt_sp": 0.5,
        "alt_vel_target": 0.05,
        "climb_cmd": -0.02,
        "pos_sp": [0.11, -0.22],
    },
    {
        "id": "p_diag", "ts": 1_000_000,
        "pos": [0.001, 0.002, 0.003],
        "vel": [0.01, 0.02, 0.03],
        "att": [0.0001, 0.0002, 0.0003],
        "bg": [1e-6, 2e-6, 3e-6],
        "ba": [1e-5, 2e-5, 3e-5],
    },
    {"id": "rate_ref", "ts": 1_000_000, "rate_ref": [0.1, -0.2, 0.05]},
    {"id": "duty400", "ts": 1_000_000, "duty": [0.1, 0.2, 0.3, 0.4]},
    {
        "id": "status", "ts": 5_000_000,
        "uptime_ms": 5000,
        "voltage": 3.85,
        "flight_state": 1,
        "eskf_status": 1,
        "pid_roll": [0.001, 0.7, 0.002],
        "pid_pitch": [0.0014, 0.7, 0.025],
        "pid_yaw": [0.0008, 0.8, 0.01],
        "current_ma": 125.4,
    },
]


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        for rec in records:
            f.write(json.dumps(rec, separators=(",", ":")) + "\n")


def test_roundtrip_covers_every_supported_id(tmp_path):
    jsonl_path = tmp_path / "synthetic.jsonl"
    _write_jsonl(jsonl_path, SYNTHETIC_RECORDS)

    bundle_path = tmp_path / "synthetic.sflog.zip"
    log = jsonl_to_bundle(jsonl_path, bundle_path, source="vehicle", notes="unit test")

    # Every id maps to at least one stream -- nothing should have been
    # silently skipped as "unknown".
    # 全ての id が少なくとも1ストリームへ対応する -- 「未知」として無言で
    # 捨てられるものが無いこと。
    expected_streams = {
        "imu", "attitude", "posvel", "pilot", "flow", "tof_bottom", "tof_front",
        "baro", "mag", "ctrl_ref", "eskf_cov", "rate_ref", "motor", "status",
    }
    assert set(log.streams.keys()) == expected_streams
    for name, df in log.streams.items():
        assert len(df) == 1, name  # one input record each -> one row each

    out_jsonl = tmp_path / "roundtrip.jsonl"
    n_written = bundle_to_jsonl(log, out_jsonl)
    assert n_written == len(SYNTHETIC_RECORDS)

    with open(out_jsonl, encoding="utf-8") as f:
        roundtrip_records = [json.loads(line) for line in f]

    original_by_key = {(r["id"], r["ts"]): r for r in SYNTHETIC_RECORDS}
    roundtrip_by_key = {(r["id"], r["ts"]): r for r in roundtrip_records}
    assert set(original_by_key.keys()) == set(roundtrip_by_key.keys())

    for key, original in original_by_key.items():
        roundtrip = roundtrip_by_key[key]
        assert roundtrip.keys() == original.keys(), key
        for field, value in original.items():
            got = roundtrip[field]
            if isinstance(value, list):
                assert len(got) == len(value), (key, field)
                for a, b in zip(value, got):
                    assert a == pytest.approx(b, abs=1e-9), (key, field)
            elif isinstance(value, float):
                assert got == pytest.approx(value, abs=1e-9), (key, field)
            else:
                assert got == value, (key, field)


def test_repeated_imu_timestamp_all_rows_kept_with_distinct_seq(tmp_path):
    """A control cycle that reused a stale IMU sample repeats imu's
    timestamp_us -- plan section 2.2/7 requires BOTH rows to survive (no
    dedup: the primary record never drops an observation), each getting
    its own `seq` (0-based capture-order row index), and the co-timed
    rate_ref rows -- freshly computed by the control law every cycle even
    when timestamp_us repeats -- to be matched to the correct imu row by
    `seq`, never by the shared timestamp_us.
    制御周期が古い IMU 標本を再利用すると imu の timestamp_us を繰り返す
    -- 計画書 2.2/7節により両方の行が生き残らねばならない（重複除去禁止:
    一次記録は観測を捨てない）。それぞれが自分の `seq`（0始まりの捕捉順
    行番号）を得て、同時刻の rate_ref 行（timestamp_us が重複していても
    制御則がその周期ごとに新しく計算する）は、共有された timestamp_us
    ではなく `seq` で正しい imu 行に対応付けられる。
    """
    records = [
        {"id": "imu", "ts": 1000, "gyro": [0.0, 0.0, 0.0], "accel": [0.0, 0.0, -9.8],
         "gyro_raw": [0.0, 0.0, 0.0], "accel_raw": [0.0, 0.0, -9.8],
         "quat": [1.0, 0.0, 0.0, 0.0], "gyro_bias": [0.0, 0.0, 0.0], "accel_bias": [0.0, 0.0, 0.0]},
        {"id": "rate_ref", "ts": 1000, "rate_ref": [0.1, 0.0, 0.0]},  # pairs with the FIRST imu@1000
        # Control cycle reused the same (stale) IMU sample -- timestamp_us
        # repeats, but this is a distinct, later control cycle with its
        # own freshly computed rate_ref value.
        # 制御周期が同じ（古い）IMU 標本を再利用 -- timestamp_us は重複
        # するが、これは別個の後続の制御周期であり、rate_ref は独自に
        # 新しく計算された値を持つ。
        {"id": "imu", "ts": 1000, "gyro": [9.9, 9.9, 9.9], "accel": [0.0, 0.0, -9.8],
         "gyro_raw": [9.9, 9.9, 9.9], "accel_raw": [0.0, 0.0, -9.8],
         "quat": [1.0, 0.0, 0.0, 0.0], "gyro_bias": [0.0, 0.0, 0.0], "accel_bias": [0.0, 0.0, 0.0]},
        {"id": "rate_ref", "ts": 1000, "rate_ref": [0.2, 0.0, 0.0]},  # pairs with the SECOND imu@1000
        {"id": "imu", "ts": 2000, "gyro": [0.1, 0.1, 0.1], "accel": [0.0, 0.0, -9.8],
         "gyro_raw": [0.1, 0.1, 0.1], "accel_raw": [0.0, 0.0, -9.8],
         "quat": [1.0, 0.0, 0.0, 0.0], "gyro_bias": [0.0, 0.0, 0.0], "accel_bias": [0.0, 0.0, 0.0]},
        {"id": "rate_ref", "ts": 2000, "rate_ref": [0.3, 0.0, 0.0]},
    ]
    jsonl_path = tmp_path / "repeat.jsonl"
    _write_jsonl(jsonl_path, records)

    log = jsonl_to_bundle(jsonl_path, tmp_path / "repeat.sflog.zip")

    # No dedup -- all 3 imu rows survive, in capture order.
    # 重複除去なし -- 3件の imu 行全てが捕捉順のまま残る。
    imu = log.streams["imu"]
    assert len(imu) == 3
    assert imu["gyro_x"].tolist() == [0.0, 9.9, 0.1]
    assert imu["seq"].tolist() == [0, 1, 2]  # 0-based capture-order row index

    # rate_ref rows are matched to the correct imu row by seq, not by the
    # shared timestamp_us -- each keeps its own, distinct value.
    # rate_ref の行は共有された timestamp_us ではなく seq で正しい imu 行に
    # 対応付けられる -- それぞれ別個の値を保つ。
    rate_ref = log.streams["rate_ref"]
    assert len(rate_ref) == 3
    assert rate_ref["seq"].tolist() == [0, 1, 2]
    assert rate_ref["rate_ref_roll"].tolist() == [0.1, 0.2, 0.3]

    # meta.json notes the repeated timestamp (1 repeat: the 2nd imu@1000).
    # meta.json に重複時刻の件数を記録する（1件: 2番目の imu@1000）。
    assert "repeated timestamps: 1 (imu)" in log.meta["notes"]

    findings = check_bundle(log)
    assert is_ok(findings), [str(f) for f in findings]  # a repeat is a warning, never an error
    assert any(
        f.level == "warning" and "repeated timestamp" in f.message and f.stream == "imu"
        for f in findings
    ), [str(f) for f in findings]


def test_unsupported_id_is_skipped_not_fatal(tmp_path, capsys):
    records = list(SYNTHETIC_RECORDS) + [{"id": "some_future_sensor", "ts": 123, "value": 1}]
    jsonl_path = tmp_path / "with_unknown.jsonl"
    _write_jsonl(jsonl_path, records)

    log = jsonl_to_bundle(jsonl_path, tmp_path / "with_unknown.sflog.zip")
    assert "imu" in log.streams  # conversion still succeeds
    captured = capsys.readouterr()
    assert "some_future_sensor" in captured.out


def test_aligned_to_csv_marks_derived(tmp_path):
    from sflog.convert import aligned_to_csv

    jsonl_path = tmp_path / "synthetic.jsonl"
    _write_jsonl(jsonl_path, SYNTHETIC_RECORDS)
    log = jsonl_to_bundle(jsonl_path, tmp_path / "synthetic.sflog.zip")

    out_csv = tmp_path / "synthetic_aligned400.csv"
    aligned_to_csv(log, out_csv, rate_note="400Hz", base="imu")

    assert out_csv.exists()
    sidecar_path = out_csv.with_name(out_csv.name + ".meta.json")
    assert sidecar_path.exists()
    sidecar = json.loads(sidecar_path.read_text(encoding="utf-8"))
    assert sidecar["derived"] is True
    assert sidecar["base"] == "imu"
    assert sidecar["method"] == "hold"
    assert sidecar["rate_note"] == "400Hz"
    assert "source_meta" in sidecar


# =============================================================================
# Opportunistic integration test against a real local capture
# ローカルの実キャプチャがあれば行う統合テスト
# =============================================================================


def _newest_local_jsonl() -> "Path | None":
    candidates = sorted(glob.glob(str(REPO_ROOT / "logs" / "stampfly_udp_*.jsonl")))
    if not candidates:
        return None
    return Path(max(candidates, key=os.path.getmtime))


def test_convert_real_capture_passes_check_if_available(tmp_path):
    jsonl_path = _newest_local_jsonl()
    if jsonl_path is None:
        pytest.skip("no logs/stampfly_udp_*.jsonl available locally")

    bundle_path = tmp_path / "real_capture.sflog.zip"
    log = jsonl_to_bundle(jsonl_path, bundle_path, source="vehicle", notes="test_convert.py integration")

    findings = check_bundle(log)
    assert is_ok(findings), [str(f) for f in findings]

    jsonl_size = jsonl_path.stat().st_size
    zip_size = bundle_path.stat().st_size
    print(
        f"\n  integration: {jsonl_path.name}: {jsonl_size:,} bytes (JSONL) "
        f"-> {zip_size:,} bytes (.sflog.zip), ratio {zip_size / jsonl_size:.2f}"
    )
