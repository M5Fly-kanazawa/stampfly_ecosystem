#!/usr/bin/env python3
"""
gen_thrust_excitation.py - Generate the thrust-excitation waveform tables from the SSOT YAML.
gen_thrust_excitation.py - 基準 YAML から推力励振の波形表を生成する。

Reads `protocol/spec/thrust_excitation.yaml` and writes two generated artifacts:

    firmware/vehicle/components/sf_core/include/thrust_excitation_table.hpp   firmware tables
    tools/sysid/thrust_excitation_table.py                                    PC-side constants

The same numbers reach the firmware and the PC analysis, so the excitation that the PC regenerates
is the one the craft flew. Do not edit the generated files; edit the YAML and run this script.
`protocol/spec/thrust_excitation.yaml` を読み、生成物を 2 つ書き出す。ファームと PC の解析に同じ
数値が届くので、PC が再生成する励振は機体が飛んだ励振と同じになる。生成物は手で編集せず、
YAML を直してこのスクリプトを実行する。

Usage / 使い方:
    python3 protocol/tools/gen_thrust_excitation.py            # same as --write
    python3 protocol/tools/gen_thrust_excitation.py --write    # regenerate on disk
    python3 protocol/tools/gen_thrust_excitation.py --check    # CI: fail if stale
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SPEC_PATH = REPO_ROOT / "protocol" / "spec" / "thrust_excitation.yaml"
HEADER_PATH = (REPO_ROOT / "firmware" / "vehicle" / "components" / "sf_core" / "include"
               / "thrust_excitation_table.hpp")
PYTHON_PATH = REPO_ROOT / "tools" / "sysid" / "thrust_excitation_table.py"

GENERATED_NOTICE = (
    "GENERATED FILE - do not edit; run protocol/tools/gen_thrust_excitation.py\n"
    "生成ファイル - 手で編集しないこと。protocol/tools/gen_thrust_excitation.py を実行して再生成する。"
)

# Number of entries of the quarter-wave sine table: one per quarter of the period, plus the end point.
# 4 分の 1 波の正弦表の項数: 周期の 4 分の 1 ごとに 1 つ、終点を足す。
QUARTER_DIVISOR = 4


def load_spec() -> dict[str, Any]:
    """Load and validate the YAML. / YAML を読み込んで検査する。"""
    spec = yaml.safe_load(SPEC_PATH.read_text(encoding="utf-8"))
    period = spec["period_cycles"]
    if period % QUARTER_DIVISOR != 0:
        raise ValueError("period_cycles must be a multiple of 4 (quarter-wave table)")
    for tone in spec["multisine"]["tones"]:
        if not (0 < tone["frequency_index"] < period // 2):
            raise ValueError("frequency_index must lie below the Nyquist index")
        if not (0 <= tone["phase_index"] < period):
            raise ValueError("phase_index must lie in [0, period_cycles)")
    return spec


def sine_at(index: int, period: int) -> float:
    """sin(2*pi*index/period) in float64. / float64 での sin(2π·index/period)。"""
    return math.sin(2.0 * math.pi * (index % period) / period)


def multisine_peak(spec: dict[str, Any]) -> float:
    """Peak of the unit-amplitude sum of all tones over one period. / 全音の合計（単位振幅）の 1 周期のピーク。"""
    period = spec["period_cycles"]
    tones = spec["multisine"]["tones"]
    peak = 0.0
    for cycle in range(period):
        total = 0.0
        for tone in tones:
            total += sine_at(tone["frequency_index"] * cycle + tone["phase_index"], period)
        peak = max(peak, abs(total))
    return peak


def multisine_rms(spec: dict[str, Any]) -> float:
    """RMS of the same sum. / 同じ合計の実効値。"""
    period = spec["period_cycles"]
    tones = spec["multisine"]["tones"]
    acc = 0.0
    for cycle in range(period):
        total = 0.0
        for tone in tones:
            total += sine_at(tone["frequency_index"] * cycle + tone["phase_index"], period)
        acc += total * total
    return math.sqrt(acc / period)


def seconds_to_cycles(seconds: float, rate_hz: int) -> int:
    """Whole control cycles of a duration. / 時間を制御周期の整数に変換する。"""
    return int(round(seconds * rate_hz))


def derived_values(spec: dict[str, Any]) -> dict[str, Any]:
    """Every number the two artifacts share. / 2 つの生成物が共有する全ての数値。"""
    rate = spec["sample_rate_hz"]
    period = spec["period_cycles"]
    peak = multisine_peak(spec)
    rms = multisine_rms(spec)
    quarter = [sine_at(i, period) for i in range(period // QUARTER_DIVISOR + 1)]
    return {
        "rate": rate,
        "period": period,
        "tones": spec["multisine"]["tones"],
        "peak": peak,
        "scale": 1.0 / peak,
        "crest": peak / rms,
        "multisine_ramp_up_cycles": seconds_to_cycles(spec["multisine"]["ramp_up_s"], rate),
        "multisine_ramp_down_cycles": seconds_to_cycles(spec["multisine"]["ramp_down_s"], rate),
        "steps_ramp_up_cycles": seconds_to_cycles(spec["steps"]["ramp_up_s"], rate),
        "steps_ramp_down_cycles": seconds_to_cycles(spec["steps"]["ramp_down_s"], rate),
        "abort_ramp_cycles": seconds_to_cycles(spec["abort_ramp_s"], rate),
        "quarter": quarter,
    }


def float_literal(value: float) -> str:
    """A C++ float literal that always has a decimal point. / 必ず小数点を持つ C++ の float リテラル。"""
    text = f"{value:.9g}"
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def render_header(values: dict[str, Any]) -> str:
    """Text of thrust_excitation_table.hpp. / thrust_excitation_table.hpp の中身。"""
    tones = values["tones"]
    freq = ", ".join(str(t["frequency_index"]) for t in tones)
    phase = ", ".join(str(t["phase_index"]) for t in tones)
    quarter = values["quarter"]
    rows = []
    for start in range(0, len(quarter), 6):
        rows.append("    " + ", ".join(float_literal(v) for v in quarter[start:start + 6]) + ",")
    table = "\n".join(rows)
    lines = [
        "/*",
        " * SPDX-License-Identifier: MIT",
        " * Copyright (c) 2026 Kouhei Ito",
        " *",
        " * Part of StampFly Ecosystem (vehicle firmware).",
        " */",
        "",
        "// " + GENERATED_NOTICE.split("\n")[0],
        "// " + GENERATED_NOTICE.split("\n")[1],
        "//",
        "// Source: protocol/spec/thrust_excitation.yaml",
        "// 基準: protocol/spec/thrust_excitation.yaml",
        "",
        "/**",
        " * @file thrust_excitation_table.hpp",
        " * @brief Waveform tables of the vertical thrust excitation (generated)",
        " *        鉛直の推力励振の波形表（生成物）",
        " *",
        " * @design detailed_design.md §3 — thrust excitation  [OK]",
        " */",
        "",
        "#pragma once",
        "",
        "#include <cstdint>",
        "",
        "namespace sf {",
        "namespace limits {",
        "",
        "/// Control cycles per second the waveform is defined at. / 波形を定義する 1 秒あたりの制御周期数。",
        f"inline constexpr uint32_t kThrustExciteSampleRateHz = {values['rate']}u;",
        "/// Cycles after which the multisine repeats (every tone is a whole number of turns). / 多重正弦波が繰り返す周期数（どの音も整数回転）。",
        f"inline constexpr uint32_t kThrustExcitePeriodCycles = {values['period']}u;",
        "/// Number of multisine tones. / 多重正弦波の音の数。",
        f"inline constexpr uint32_t kThrustExciteToneCount = {len(tones)}u;",
        "/// Tone frequency in units of 0.1 Hz. / 音の周波数（0.1 Hz 単位）。",
        f"inline constexpr uint16_t kThrustExciteToneFrequencyIndex[kThrustExciteToneCount] = {{{freq}}};",
        "/// Tone phase in units of 1/kThrustExcitePeriodCycles turn. / 音の位相（1/kThrustExcitePeriodCycles 回転単位）。",
        f"inline constexpr uint16_t kThrustExciteTonePhaseIndex[kThrustExciteToneCount] = {{{phase}}};",
        "/// Scale that makes the peak of the unit-amplitude sum of all tones equal to 1 (1 / peak; the crest factor is "
        f"{values['crest']:.3f}). / 全音の合計（単位振幅）のピークを 1 にする倍率（1 / ピーク。クレストファクタは {values['crest']:.3f}）。",
        f"inline constexpr float kThrustExciteMultisineScale = {float_literal(values['scale'])};",
        "/// Ramp lengths in control cycles. / 立ち上げ・立ち下げの長さ（制御周期数）。",
        f"inline constexpr uint32_t kThrustExciteMultisineRampUpCycles   = {values['multisine_ramp_up_cycles']}u;",
        f"inline constexpr uint32_t kThrustExciteMultisineRampDownCycles = {values['multisine_ramp_down_cycles']}u;",
        f"inline constexpr uint32_t kThrustExciteStepsRampUpCycles       = {values['steps_ramp_up_cycles']}u;",
        f"inline constexpr uint32_t kThrustExciteStepsRampDownCycles     = {values['steps_ramp_down_cycles']}u;",
        f"inline constexpr uint32_t kThrustExciteAbortRampCycles         = {values['abort_ramp_cycles']}u;",
        "",
        "/// Quarter-wave sine table: sin(2*pi*i/kThrustExcitePeriodCycles) for i = 0 .. period/4. "
        "/ 4 分の 1 波の正弦表。",
        f"inline constexpr uint32_t kThrustExciteQuarterTableSize = {len(quarter)}u;",
        "inline constexpr float kThrustExciteQuarterSine[kThrustExciteQuarterTableSize] = {",
        table,
        "};",
        "",
        "}  // namespace limits",
        "}  // namespace sf",
        "",
    ]
    return "\n".join(lines)


def render_python(values: dict[str, Any]) -> str:
    """Text of thrust_excitation_table.py. / thrust_excitation_table.py の中身。"""
    tones = values["tones"]
    lines = [
        '"""Waveform constants of the vertical thrust excitation (generated).',
        "鉛直の推力励振の波形定数（生成物）。",
        "",
        GENERATED_NOTICE,
        "",
        "Source: protocol/spec/thrust_excitation.yaml",
        '"""',
        "",
        f"SAMPLE_RATE_HZ = {values['rate']}",
        f"PERIOD_CYCLES = {values['period']}",
        f"TONE_FREQUENCY_INDEX = {tuple(t['frequency_index'] for t in tones)!r}  # units of 0.1 Hz / 0.1 Hz 単位",
        f"TONE_PHASE_INDEX = {tuple(t['phase_index'] for t in tones)!r}  # 1/PERIOD_CYCLES turn / 1/PERIOD_CYCLES 回転",
        f"MULTISINE_SCALE = {values['scale']!r}  # 1 / peak of the unit-amplitude sum / 単位振幅の合計のピークの逆数",
        f"MULTISINE_CREST_FACTOR = {values['crest']!r}",
        f"MULTISINE_RAMP_UP_CYCLES = {values['multisine_ramp_up_cycles']}",
        f"MULTISINE_RAMP_DOWN_CYCLES = {values['multisine_ramp_down_cycles']}",
        f"STEPS_RAMP_UP_CYCLES = {values['steps_ramp_up_cycles']}",
        f"STEPS_RAMP_DOWN_CYCLES = {values['steps_ramp_down_cycles']}",
        f"ABORT_RAMP_CYCLES = {values['abort_ramp_cycles']}",
        "",
    ]
    return "\n".join(lines)


def expected_outputs() -> dict[Path, str]:
    """Path -> expected file text. / パス → 期待されるファイルの中身。"""
    values = derived_values(load_spec())
    return {HEADER_PATH: render_header(values), PYTHON_PATH: render_python(values)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--write", action="store_true", help="regenerate on disk (default)")
    group.add_argument("--check", action="store_true", help="fail if the generated files are stale")
    args = parser.parse_args()
    outputs = expected_outputs()
    if args.check:
        stale = [p for p, text in outputs.items()
                 if not p.exists() or p.read_text(encoding="utf-8") != text]
        for path in stale:
            print(f"STALE: {path.relative_to(REPO_ROOT)} (run protocol/tools/gen_thrust_excitation.py)")
        if stale:
            return 1
        print("thrust excitation tables are up to date")
        return 0
    for path, text in outputs.items():
        path.write_text(text, encoding="utf-8")
        print(f"wrote {path.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
