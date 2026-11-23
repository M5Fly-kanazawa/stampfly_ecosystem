"""Waveform constants of the vertical thrust excitation (generated).
鉛直の推力励振の波形定数（生成物）。

GENERATED FILE - do not edit; run protocol/tools/gen_thrust_excitation.py
生成ファイル - 手で編集しないこと。protocol/tools/gen_thrust_excitation.py を実行して再生成する。

Source: protocol/spec/thrust_excitation.yaml
"""

SAMPLE_RATE_HZ = 400
PERIOD_CYCLES = 4000
TONE_FREQUENCY_INDEX = (4, 7, 11, 17, 27, 41, 59, 89)  # units of 0.1 Hz / 0.1 Hz 単位
TONE_PHASE_INDEX = (0, 3704, 2289, 2456, 3664, 3593, 1852, 1048)  # 1/PERIOD_CYCLES turn / 1/PERIOD_CYCLES 回転
MULTISINE_SCALE = 0.21551021058761696  # 1 / peak of the unit-amplitude sum / 単位振幅の合計のピークの逆数
MULTISINE_CREST_FACTOR = 2.320075687535566
MULTISINE_RAMP_UP_CYCLES = 200
MULTISINE_RAMP_DOWN_CYCLES = 200
STEPS_RAMP_UP_CYCLES = 40
STEPS_RAMP_DOWN_CYCLES = 100
ABORT_RAMP_CYCLES = 100
