"""Tests of the generated thrust-excitation table and its generator.
推力励振の生成表と、その生成スクリプトの試験。

The firmware reads the multisine from a quarter-wave table by integer phase indices
(sf::thrustExciteMultisine, firmware/vehicle/components/sf_core/include/thrust_excitation.hpp); the PC
regenerates it from the same constants. The golden values below were printed by the firmware function (a host
build of the header, 2026-11-23), so a drift of the generated constants, of the formula or of the firmware
header fails here.
ファームは多重正弦波を、整数の位相の添字で 4 分の 1 波の表から読む（sf::thrustExciteMultisine）。PC は同じ定数から
再生成する。下の基準値は、ファームの関数（ヘッダのホストビルド）が出力したもの。生成定数・式・ファームのヘッダの
どれかがずれればここで落ちる。
"""

import math
import subprocess
import sys
from pathlib import Path

import pytest

from tools.sysid import thrust_excitation_table as table

REPO_ROOT = Path(__file__).resolve().parents[2]
GENERATOR = REPO_ROOT / "protocol" / "tools" / "gen_thrust_excitation.py"

# (control cycle, value printed by the firmware as float32). / （制御周期、ファームが float32 で出力した値）
FIRMWARE_GOLDEN = (
    (0, -0.305209219), (1, -0.313943952), (7, -0.428849548), (100, -0.150615826),
    (999, 0.0370924622), (1000, 0.0187446997), (1234, -0.132082626), (2500, 0.292815059),
    (3999, -0.300224692), (4000, -0.305209219), (4001, -0.313943952), (12345, 0.365184605),
)
FLOAT32_TOLERANCE = 2.0e-6


def multisine(cycle: int) -> float:
    """The multisine at a control cycle, from the generated constants (peak 1). / 生成定数からの多重正弦波（ピーク 1）。"""
    period = table.PERIOD_CYCLES
    total = 0.0
    for frequency_index, phase_index in zip(table.TONE_FREQUENCY_INDEX, table.TONE_PHASE_INDEX):
        total += math.sin(2.0 * math.pi * ((frequency_index * (cycle % period) + phase_index) % period) / period)
    return total * table.MULTISINE_SCALE


@pytest.mark.parametrize("cycle, expected", FIRMWARE_GOLDEN)
def test_regenerated_multisine_matches_the_firmware(cycle, expected):
    assert multisine(cycle) == pytest.approx(expected, abs=FLOAT32_TOLERANCE)


def test_peak_is_one_and_the_pattern_repeats():
    values = [multisine(cycle) for cycle in range(table.PERIOD_CYCLES)]
    assert max(abs(value) for value in values) == pytest.approx(1.0, abs=1e-9)
    assert multisine(table.PERIOD_CYCLES + 17) == pytest.approx(multisine(17), abs=1e-12)


def test_crest_factor_is_the_designed_one():
    values = [multisine(cycle) for cycle in range(table.PERIOD_CYCLES)]
    rms = math.sqrt(sum(value * value for value in values) / len(values))
    assert 1.0 / rms == pytest.approx(table.MULTISINE_CREST_FACTOR, rel=1e-9)
    assert table.MULTISINE_CREST_FACTOR < 2.5   # random phases give about 3.2 / 乱数位相では約 3.2


def test_generated_files_are_up_to_date():
    result = subprocess.run([sys.executable, str(GENERATOR), "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
