#!/usr/bin/env python3
"""
test_motor_health.py - Tests for the motor health report (sf log analyze --health)
モータ健全性レポート（sf log analyze --health）のテスト

Uses the shared tools/log_analyzer/conftest.py fixtures:
  * `reference_bundle` -- real 30 s indoor hover. It has no `motor` stream,
    so it exercises the 50 Hz `ctrl_ref` duty fallback.
  * `synthetic_bundle` -- every stream at native rate, every duty exactly
    0.5 so the 4-duty sum sits exactly ON the HOVER_DUTY_SUM boundary (not
    above it), which must NOT be classified as hover.

`conftest.build_synthetic_log()` (a plain function, not a fixture) is used
directly to build a scaled-duty variant in memory for the group-detection
tests below.

Usage:
    .venv/bin/python3 -m pytest tools/log_analyzer/test_motor_health.py -q
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import conftest  # noqa: E402
import motor_health  # noqa: E402
import sflog  # noqa: E402

# CCW motors (FR/RL) driven at this duty, CW motors (RR/FL) at CW_DUTY below
# -- ur = (FR+RL)-(RR+FL) = 2*(CCW_DUTY-CW_DUTY) = -0.20, well past
# -UR_SIGNIFICANT, so the CW group reads as the weak one.
# CCW モータ(FR/RL)をこの duty、CW モータ(RR/FL)を CW_DUTY で駆動する --
# ur = -0.20 となり -UR_SIGNIFICANT を大きく下回るため、CW 群が弱いと判定される。
CCW_DUTY = 0.55
CW_DUTY = 0.65


def _build_weak_cw_bundle(tmp_path, filename, drop_motor_stream):
    """Build a synthetic bundle with a CW-weak duty asymmetry, optionally
    dropping the `motor` stream to exercise the `ctrl_ref` fallback.
    CW 群が弱い duty 不均衡を持つ合成一式を作る（`drop_motor_stream` で
    `motor` ストリームを外し `ctrl_ref` フォールバックを検証できる）。"""
    log = conftest.build_synthetic_log()
    for stream_name in ("motor", "ctrl_ref"):
        df = log.streams[stream_name]
        df["duty_FR"] = CCW_DUTY
        df["duty_RL"] = CCW_DUTY
        df["duty_RR"] = CW_DUTY
        df["duty_FL"] = CW_DUTY
    if drop_motor_stream:
        del log.streams["motor"]
    log.meta = sflog.make_meta(
        source="sim", tool_name="test_motor_health", tool_version="0.0.0",
        streams=log.streams,
    )
    path = tmp_path / filename
    log.save(path)
    return path


# --------------------------------------------------------------------------
# Reference bundle (real hover, ctrl_ref-only) / 基準一式（実機、ctrl_refのみ）
# --------------------------------------------------------------------------
def test_reference_bundle_stats(reference_bundle):
    stats = motor_health.per_log_stats(str(reference_bundle))
    assert stats is not None
    assert stats["dur_s"] >= 10
    assert stats["n"] >= 50
    assert stats["duty_source"] == "ctrl_ref"
    for duty_mean in stats["duty_mean"]:
        assert 0.3 <= duty_mean <= 0.9
    assert 3.0 <= stats["volt"] <= 4.5


def test_reference_bundle_json_report(reference_bundle, capsys):
    # Only json_out's stdout needs to be valid, parseable JSON -- the
    # in-memory `result` dict itself legitimately differs after a JSON
    # round-trip (e.g. `group` is a Python tuple, JSON has no tuple type
    # and reads it back as a list), so this checks the printed JSON's
    # shape directly rather than comparing it back to `result`.
    # 検証するのは json_out の標準出力が妥当な JSON であることのみ --
    # メモリ上の `result` 自体は JSON 往復で正当に変わり得る（例: `group`
    # は Python のタプルだが JSON にタプル型は無くリストとして読み戻る）
    # ため、`result` と突き合わせず出力 JSON の形を直接検証する。
    motor_health.analyze_health([str(reference_bundle)], json_out=True)
    captured = capsys.readouterr()
    parsed = json.loads(captured.out)
    assert "verdict" in parsed
    assert parsed["n_logs"] == 1


# --------------------------------------------------------------------------
# Synthetic bundle (exact hover-boundary duty, must not count as hover)
# 合成一式（duty がホバー境界ちょうど、ホバーと判定されてはいけない）
# --------------------------------------------------------------------------
def test_synthetic_bundle_exact_hover_boundary_is_not_hover(synthetic_bundle):
    # Every duty is exactly 0.5, so the 4-duty sum equals
    # motor_health.HOVER_DUTY_SUM exactly -- not strictly greater than it --
    # so no sample counts as "spun" and there is no hover window.
    assert motor_health.per_log_stats(str(synthetic_bundle)) is None
    result = motor_health.analyze_health([str(synthetic_bundle)])
    assert "error" in result


# --------------------------------------------------------------------------
# Duty source selection: motor.csv (400Hz) vs ctrl_ref.csv (50Hz) fallback
# duty の出所選択: motor.csv (400Hz) と ctrl_ref.csv (50Hz) フォールバック
# --------------------------------------------------------------------------
def test_motor_stream_duty_source_detects_weak_cw_group(tmp_path):
    path = _build_weak_cw_bundle(tmp_path, "weak_cw_motor.sflog.zip",
                                  drop_motor_stream=False)
    stats = motor_health.per_log_stats(str(path))
    assert stats is not None
    assert stats["duty_source"] == "motor"
    v = motor_health.verdict([stats])
    assert v["group"] == motor_health.CW_GROUP


def test_ctrl_ref_fallback_when_motor_stream_absent(tmp_path):
    path = _build_weak_cw_bundle(tmp_path, "weak_cw_ctrl_ref.sflog.zip",
                                  drop_motor_stream=True)
    stats = motor_health.per_log_stats(str(path))
    assert stats is not None
    assert stats["duty_source"] == "ctrl_ref"
    v = motor_health.verdict([stats])
    assert v["group"] == motor_health.CW_GROUP


# --------------------------------------------------------------------------
# _fmt_path
# --------------------------------------------------------------------------
def test_fmt_path_strips_sflog_extension_and_flight_prefix():
    assert motor_health._fmt_path(
        "logs/flight_20260908T121243.sflog.zip"
    ) == "20260908T121243"


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
