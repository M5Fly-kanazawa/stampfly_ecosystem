"""
Plant Model Fitting from Closed-Loop Flight Data
閉ループフライトデータからのプラントモデル同定

Identifies open-loop plant parameters from P-control flight data:
  G_p(s) = K / (s * (tau_m * s + 1))

Where:
  K    : Plant gain [rad/s^2 per duty]
  tau_m: Motor time constant [s]

Algorithm:
  Plant I/O is reconstructed from an ALIGNED flight-log-bundle DataFrame
  (tools/sysid/loader.py's load_aligned() -- see that function's docstring
  for the full column contract; the bundle format itself is documented in
  docs/plans/flight-log-format-plan.md section 2.2) via one of two INPUT
  MODES (--input {auto,duty,kp}; see fit_plant()):

    "duty" -- the actual plant input, recovered from the bundle's 400Hz
      motor duty stream (duty_FR/RR/RL/FL, from the bundle's `motor` CSV --
      firmware sending the kPktDuty400 entry, see data_stream_wire.hpp).
      The 4 motor duties are inverted to recover the per-axis differential
      command the rate-loop PID actually output, without having to know Kp
      or assume it never changed (autotune, gain schedule) or that the
      actuator never saturated. WHICH inversion is used is selected by
      --mixer {legacy,vehicle} (default "legacy"):
        "legacy" -- inverts the SIMPLE LINEAR "voltage-scale" X-quad mixer
          (ws_internal.hpp motor_mixer / vehicle_old setMixerOutput): duty
          IS the differential command, additively mixed, no battery-voltage
          or motor-curve nonlinearity. Correct for firmware/workshop (`sf
          lesson`) and firmware/vehicle_old logs. See
          _duty_differential_legacy_linear().
        "vehicle" -- inverts firmware/vehicle's ACTUAL mixer
          (sf_actuator/actuator.cpp mixerCompute()): a physical-unit B^-1
          allocation (thrust [N] / torque [Nm]) followed by a NONLINEAR
          motor curve (duty = f(sqrt(T/Ct)) / Vbat). Required for logs from
          `sf app` (firmware/vehicle-based custom controllers) -- the
          "legacy" inversion silently fits the WRONG signal on those (looked
          like a fit failure / physically-impossible K,tau_m in practice).
          See _duty_differential_vehicle().
      The CSV column layout is IDENTICAL either way (same LogStreamSample
      wire struct), so this cannot be auto-detected from the file alone --
      the caller must know which firmware produced the log.
    "kp" -- the LEGACY reconstruction: since Kp is assumed known and
      constant, u_plant is approximated as Kp * (target - gyro):
          target(t) = rate_ref_<axis>(t)          <- already rad/s
      The bundle format has only this one schema -- the pre-migration
      "legacy" analysis-CSV schema this mode used to also auto-detect
      (normalized stick * rate_max, via the retired _detect_csv_format())
      no longer exists anywhere in the repo and has been removed along with
      the two independent CSV-format detectors that used to guard this
      function (see tools/sysid/loader.py's module docstring).
    "auto" (default) -- "duty" when duty_FR/RR/RL/FL columns exist AND
      --kp was NOT given; otherwise "kp" (which then requires --kp).

  Either way: y_plant(t) = gyro(t). The open-loop model is fitted directly
  via MSE minimization:
    u_plant -> G_p(s) -> y_simulated
    minimize |y_simulated - y_plant|^2  ->  K, tau_m

プラント入出力は、整列済みフライトログ一式 DataFrame
（tools/sysid/loader.py の load_aligned() -- 列の全契約は同関数の
docstring、一式形式そのものは docs/plans/flight-log-format-plan.md
2.2節参照）から、2つの「入力モード」（--input {auto,duty,kp}。fit_plant()
参照）のいずれかで再構成する:

  "duty" -- 一式の400Hz モータduty ストリーム（duty_FR/RR/RL/FL。一式の
    `motor` CSV 由来 -- ファームが kPktDuty400 エントリを送っている必要が
    ある。data_stream_wire.hpp 参照）から復元した「実際のプラント入力」。
    4モータduty を
    逆算し、レートループ PID が実際に出力した軸別の差動指令を復元する —
    Kp を知る必要も、Kp が飛行中に不変（自動チューニング・ゲイン
    スケジューリング無し）だったことも、アクチュエータが飽和しなかった
    ことも仮定しない。**どちらの逆算を使うかは --mixer {legacy,vehicle}
    で選ぶ（既定 "legacy"）**:
      "legacy" -- 単純な線形「電圧スケール」X字ミキサー（ws_internal.hpp
        motor_mixer / vehicle_old の setMixerOutput）を逆算する: duty が
        そのまま差動指令（加減算のみ、バッテリ電圧も非線形モータ曲線も
        無し）。firmware/workshop（`sf lesson`）と firmware/vehicle_old の
        ログに正しい。_duty_differential_legacy_linear() 参照。
      "vehicle" -- firmware/vehicle の実際のミキサー
        （sf_actuator/actuator.cpp mixerCompute()）を逆算する: 物理単位
        （推力[N]・トルク[Nm]）の B^-1 配分＋非線形モータ曲線
        （duty = f(√(T/Ct)) / Vbat）。`sf app`（firmware/vehicle ベースの
        自作コントローラ）のログにはこちらが必須 —
        "legacy" 逆算のまま使うと誤った信号を静かにフィットしてしまう
        （実際にはフィット失敗や物理的にありえない K・tau_m として現れる）。
        _duty_differential_vehicle() 参照。
    CSV の列構成はどちらの由来でも同一（同じ LogStreamSample 電文構造体）
    なので、ファイル単体からは自動判別できない — 呼び出し側がどちらの
    ファームで記録したログかを知っている必要がある。
  "kp" -- 従来の再構成方式: Kp が既知・一定と仮定し、
    u_plant を Kp × (target − gyro) で近似する:
      target(t) = rate_ref_<axis>(t)            <- 既に rad/s
    一式形式はこの1種類のスキーマしか持たない -- このモードがかつて併せて
    自動判別していた移行前の "legacy" 分析用 CSV スキーマ（正規化スティック
    × rate_max、廃止済みの _detect_csv_format() 経由）はリポジトリのどこにも
    もう存在せず、この関数を守っていた2つの独立した CSV 形式判別と共に削除
    した（tools/sysid/loader.py のモジュール docstring 参照）。
  "auto"（既定）-- duty_FR/RR/RL/FL 列があり、かつ --kp 未指定なら "duty"。
    それ以外は "kp"（この場合 --kp が必須）。

いずれも: y_plant(t) = gyro(t)。開ループモデルは MSE 最小化で直接フィット
する:
  u_plant -> G_p(s) -> y_simulated
  minimize |y_simulated - y_plant|^2  ->  K, tau_m
"""

from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
from scipy.optimize import minimize

import sflog

from .defaults import get_flat_defaults
from .loader import load_aligned
from ._generated_params import EXPECTED_ARM, EXPECTED_KAPPA, EXPECTED_CT, \
    EXPECTED_IXX, EXPECTED_IYY, EXPECTED_IZZ


# Reference plant gains from L06 System Modeling. Units match --input duty's
# "legacy" mixer (--mixer legacy, the default): [rad/s^2 per differential-duty
# unit]. Do NOT use these for --mixer vehicle -- see
# REFERENCE_PLANT_GAINS_VEHICLE below, which is in the different physical
# units (Nm) that mode's u_plant is reconstructed in.
# L06 システムモデリングからの参照プラントゲイン。単位は --input duty の
# "legacy" ミキサー（--mixer legacy、既定）用: [rad/s^2 / 差動duty単位]。
# --mixer vehicle には使わないこと -- 下の REFERENCE_PLANT_GAINS_VEHICLE
# 参照（あちらは u_plant の単位が異なる[Nm]）。

# Yaw's K, unlike roll/pitch, is NOT a hand-copied literal: it is DERIVED
# from the generated (SSOT-tracked) physical constants, the same way commit
# 7cb91277 (2026-09-07, "lesson 6 yaw plant gain K_yaw 19 -> 8.0 to match the
# current kappa") derived it for lesson_06/07's README -- but that commit did
# NOT touch this file, so REFERENCE_PLANT_GAINS['yaw'] silently kept the OLD
# value (19.0, from the pre-2026-07-17 kappa=9.71e-3) for 3 days, showing
# students the WRONG theoretical reference to compare their yaw fit against
# (found 2026-09-10 testing a real lesson_07 flight -- the 'ref' column read
# 19.0 while every other current document says 8.0). Deriving it here from
# EXPECTED_KAPPA/EXPECTED_ARM/EXPECTED_IZZ (regenerated by `sf params
# generate` whenever the physical model changes) means it can no longer go
# stale independently of those.
#   Km_rp (motor torque constant, roll/pitch) = roll_K * EXPECTED_IXX
#     = 102.0 * 9.16e-6 = 9.34e-4 N*m -- matches the commit's stated 9.3e-4
#   Km_yaw = Km_rp * kappa / arm = 9.34e-4 * 0.0041 / 0.023 = 1.665e-4 N*m
#   K_yaw  = Km_yaw / EXPECTED_IZZ = 1.665e-4 / 2.04e-5 = 8.16 rad/s^2
# ヨーの K は roll/pitch と違い手書きの定数ではなく、生成済み（SSOT管理）の
# 物理定数から導出する。コミット 7cb91277（2026-09-07、「lesson 6 yaw plant
# gain K_yaw 19 -> 8.0 to match the current kappa」）が lesson_06/07 の
# READMEに対して行ったのと同じ導出だが、あのコミットはこのファイルには
# 触れなかったため、REFERENCE_PLANT_GAINS['yaw'] は3日間、古い値
# （2026-07-17以前のkappa=9.71e-3由来の19.0）のまま黙って残り、学習者に
# 誤った理論参照値を見せていた（2026-09-10、実際のlesson_07フライトで
# テスト中に発見 -- 'ref' 列が19.0のままで、他の全ての最新文書は8.0と
# 言っていた）。EXPECTED_KAPPA/EXPECTED_ARM/EXPECTED_IZZ（物理モデルが
# 変わるたび `sf params generate` で再生成される）から導出すれば、それらと
# 独立に値が古くなることが無くなる。
_ROLL_K_LEGACY = 102.0  # [rad/s^2 per duty] -- hand value, unchanged since L06
_PITCH_K_LEGACY = 70.0  # [rad/s^2 per duty] -- hand value, unchanged since L06
_KM_RP = _ROLL_K_LEGACY * EXPECTED_IXX  # N*m -- motor torque constant, roll/pitch
_YAW_K_DERIVED = (_KM_RP * EXPECTED_KAPPA / EXPECTED_ARM) / EXPECTED_IZZ  # [rad/s^2 per duty]

REFERENCE_PLANT_GAINS: Dict[str, float] = {
    'roll': _ROLL_K_LEGACY,    # [rad/s^2 per duty]
    'pitch': _PITCH_K_LEGACY,
    'yaw': _YAW_K_DERIVED,     # ~8.0 -- see derivation above (was a stale 19.0)
}

# Reference plant gains for --mixer vehicle: u_plant there is a physical
# differential TORQUE [Nm] (see _duty_differential_vehicle()), so the
# steady-state (motor-lag-free) plant gain is simply 1/I_axis (Newton's
# second law for rotation: domega/dt = torque / I) -- NOT the empirical
# duty-unit REFERENCE_PLANT_GAINS above. Also used to SEED the optimizer
# (K_init in fit_plant()): the duty-unit reference is off by several orders
# of magnitude in Nm terms and gives L-BFGS-B a useless starting point.
# I_axis is the SAME EXPECTED_IXX/IYY/IZZ tracked by `sf params check`
# (tools/params_audit/params_manifest.py "Ixx"/"Iyy"/"Izz" groups), so this
# stays in sync with the confirmed body-inertia measurement automatically.
# --mixer vehicle 用の参照プラントゲイン: u_plant は物理的な差動トルク
# [Nm]（_duty_differential_vehicle() 参照）なので、定常（モータ遅れ無視）
# ゲインは単純に 1/I_axis（回転のニュートンの第2法則:
# domega/dt = torque / I）—— 上の duty単位版 REFERENCE_PLANT_GAINS とは別物。
# 最適化の初期値（fit_plant() の K_init）にも使う: duty単位の参照値は
# Nm換算で桁が何桁も違い、L-BFGS-B に無意味な初期点を与えてしまう。
# I_axis は `sf params check`（tools/params_audit/params_manifest.py の
# "Ixx"/"Iyy"/"Izz" グループ）が追跡しているのと同じ EXPECTED_IXX/IYY/IZZ
# なので、確定済みの機体慣性実測値と自動的に同期する。
REFERENCE_PLANT_GAINS_VEHICLE: Dict[str, float] = {
    'roll': 1.0 / EXPECTED_IXX,    # [rad/s^2 per Nm]
    'pitch': 1.0 / EXPECTED_IYY,
    'yaw': 1.0 / EXPECTED_IZZ,
}

# Body FRD axis order (roll about x, pitch about y, yaw about z) -- same
# convention used throughout the repo (see e.g. tools/log_analyzer/rate_sysid.py
# and data_stream_wire.hpp), kept identical here so `--axis` selection and the
# gyro sign/axis mapping never drift from the README's coordinate system.
# Body FRD 軸順（roll=x軸周り, pitch=y軸周り, yaw=z軸周り）— リポジトリ全体で
# 使われる規約と同一（例: tools/log_analyzer/rate_sysid.py, data_stream_wire.hpp）。
# --axis 選択とジャイロの軸・符号対応が README の座標系からずれないよう、
# ここでも同じ規約を使う。
_AXIS_NAMES: Tuple[str, str, str] = ('roll', 'pitch', 'yaw')

# Bundle column name per axis for the estimator-input gyro (aligned
# DataFrame's `imu` stream, bare names -- see
# tools/sysid/loader.py's load_aligned() docstring). Replaces the retired
# _STREAM_GYRO_COL/_LEGACY_GYRO_COL pair: the bundle format has only ONE
# schema now, so there is no "legacy" gyro_corrected_x/y/z fallback to
# choose between -- imu.csv's gyro_x/y/z IS already the corrected value
# (there is no separate raw-vs-corrected choice left at this call site).
# 整列済み DataFrame（`imu` ストリーム、素の列名 -- tools/sysid/loader.py の
# load_aligned() docstring 参照）における、推定器入力ジャイロの軸ごとの列名。
# 廃止した _STREAM_GYRO_COL/_LEGACY_GYRO_COL の対を置き換える: 一式形式は
# もうスキーマが1種類しかないため、"legacy" の gyro_corrected_x/y/z への
# フォールバックを選ぶ必要が無い -- imu.csv の gyro_x/y/z が既に補正済みの値
# そのもの（この呼び出し箇所にはもう生値/補正値の選択肢自体が無い）。
_GYRO_COL: Dict[str, str] = {'roll': 'gyro_x', 'pitch': 'gyro_y', 'yaw': 'gyro_z'}

# Bundle column name per axis for the inner-loop rate reference (aligned
# DataFrame's `rate_ref` stream, bare names) -- already rad/s (see
# protocol/spec/flight_log.yaml's rate_ref_roll/pitch/yaw description).
# Replaces the retired _STREAM_TARGET_COL/_LEGACY_CTRL_COL pair: the
# "legacy" normalized-stick*rate_max schema no longer exists.
# 整列済み DataFrame（`rate_ref` ストリーム、素の列名）における、内側ループ
# （レート制御）角速度目標の軸ごとの列名 -- 既に rad/s
# （protocol/spec/flight_log.yaml の rate_ref_roll/pitch/yaw 説明参照）。
# 廃止した _STREAM_TARGET_COL/_LEGACY_CTRL_COL の対を置き換える -- "legacy"
# の正規化スティック×rate_max スキーマはもう存在しない。
_TARGET_COL: Dict[str, str] = {
    'roll': 'rate_ref_roll', 'pitch': 'rate_ref_pitch', 'yaw': 'rate_ref_yaw',
}

# Aligned-DataFrame column names for the 4 motor duties -- bare names
# (no "motor_" prefix, unlike the retired flat-CSV format's
# motor_duty_FR/RR/RL/FL) because they come straight from the bundle's
# `motor`/`ctrl_ref` streams (see tools/sysid/loader.py's load_aligned()
# docstring for when each stream supplies them, and how to tell genuine
# 400Hz duty from a 50Hz-held fallback via
# `df.attrs["bundle_streams"]`). Order matches the wire layout FR,RR,RL,FL.
# 整列済み DataFrame におけるモータduty 4列の名前 -- 素の名前（廃止した
# 平坦CSV形式の motor_duty_FR/RR/RL/FL と違い "motor_" 接頭辞なし）。一式の
# `motor`/`ctrl_ref` ストリームからそのまま来るため（どちらが供給するか、
# 本物の400Hz dutyと50Hz保持フォールバックの見分け方は
# tools/sysid/loader.py の load_aligned() docstring と
# `df.attrs["bundle_streams"]` 参照）。電文と同じ順序 FR,RR,RL,FL。
_DUTY_COLS: Tuple[str, str, str, str] = ('duty_FR', 'duty_RR', 'duty_RL', 'duty_FL')

# X-quad mixer differential-duty coefficient (ws_internal.hpp motor_mixer,
# reproduced digit-for-digit from vehicle_old's setMixerOutput):
#   M1 FR = T + k*(-R+P+Y)   M2 RR = T + k*(-R-P-Y)
#   M3 RL = T + k*( R-P+Y)   M4 FL = T + k*( R+P-Y)      k = 0.25/3.7
# X字クアッドミキサの差動duty係数（ws_internal.hpp motor_mixer。vehicle_old
# の setMixerOutput と数値まで一致させた原典の再現）。
_MIXER_K: float = 0.25 / 3.7

# Gyro-magnitude sanity bound for automatic crash/anomaly truncation in
# _load_axis_data() -- see that function for the full rationale/calibration.
# _load_axis_data() の自動クラッシュ/異常検知トランケーション用ジャイロ
# 振幅の妥当性しきい値 -- 根拠・較正は同関数のコメント参照。
_GYRO_CRASH_MAX_RAD_S: float = 10.0


def _duty_differential_legacy_linear(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    axis: str,
) -> np.ndarray:
    """
    Invert the SIMPLE LINEAR "voltage-scale" X-quad mixer (ws_internal.hpp
    motor_mixer / vehicle_old setMixerOutput) to recover the per-axis
    differential duty command (R, P, or Y) that produced the 4 motor duties.
    --mixer legacy (the default). Correct for firmware/workshop (`sf lesson`)
    and firmware/vehicle_old logs ONLY -- for firmware/vehicle (`sf app`)
    logs use _duty_differential_vehicle() instead (see the module docstring
    for why the two firmwares need different inversions: vehicle's actual
    mixer is a physical B^-1 allocation through a NONLINEAR motor curve, not
    this linear one).
    単純な線形「電圧スケール」X字ミキサー（ws_internal.hpp motor_mixer /
    vehicle_old の setMixerOutput）を逆算し、4モータduty から軸別の差動
    duty指令（R/P/Y）を復元する。--mixer legacy（既定）。
    firmware/workshop（`sf lesson`）と firmware/vehicle_old のログにのみ
    正しい -- firmware/vehicle（`sf app`）のログには
    _duty_differential_vehicle() を使うこと（2つのファームで逆算が異なる
    理由はモジュール docstring 参照: vehicle の実ミキサーは非線形モータ
    曲線を介した物理単位の B^-1 配分であり、この線形式とは別物）。

    T cancels out in every combination below (it appears identically in all
    4 motor equations), so this needs no knowledge of thrust or hover duty --
    the result is u_plant in the SAME units the rate-loop PID's raw output
    entered the mixer with, whether or not the duty saturated.
    下式はどれも T が相殺する（4つのモータ式全てに同一に現れるため）ので、
    推力やホバーduty の知識は不要 -- 結果はレートループ PID の生出力が
    ミキサへ入ったのと同じ単位の u_plant になる（duty が飽和していても）。

    Args:
        duty_fr, duty_rr, duty_rl, duty_fl: motor duty arrays [0, 1]
        axis: 'roll', 'pitch', or 'yaw'

    Returns:
        Differential duty command array (same length/shape as the inputs).
    """
    if axis == 'roll':
        diff = -duty_fr - duty_rr + duty_rl + duty_fl
    elif axis == 'pitch':
        diff = duty_fr - duty_rr - duty_rl + duty_fl
    elif axis == 'yaw':
        diff = duty_fr - duty_rr + duty_rl - duty_fl
    else:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")
    return diff / (4.0 * _MIXER_K)


# -----------------------------------------------------------------------------
# --mixer vehicle: firmware/vehicle's ACTUAL mixer (sf_actuator/actuator.cpp
# mixerCompute() / thrustToDuty()), reproduced digit-for-digit. Unlike the
# "legacy" linear mixer above, duty here is NOT the differential command
# itself -- it is a NONLINEAR function of the per-motor thrust and the live
# battery voltage, so recovering the differential torque command requires
# inverting the full physical chain: duty -> volts -> omega -> thrust -> (B^-1
# combination) -> torque. See _thrust_from_duty() / _duty_differential_vehicle().
#
# --mixer vehicle: firmware/vehicle の実際のミキサー（sf_actuator/
# actuator.cpp の mixerCompute() / thrustToDuty()）を一字一句再現する。
# 上の "legacy" 線形ミキサーと異なり、duty はそのまま差動指令ではない --
# 各モータ推力と実電源電圧の非線形関数なので、差動トルク指令を復元するには
# 物理チェーン全体を逆算する必要がある: duty -> volts -> omega -> thrust ->
# （B^-1結合）-> torque。_thrust_from_duty() / _duty_differential_vehicle()
# 参照。
# -----------------------------------------------------------------------------

# B^-1 allocation geometry (actuator.cpp mixerCompute(), ARM_D/KAPPA):
#   T_FR = 0.25*(thrust - up/ARM_D + uq/ARM_D + ur/KAPPA)
#   T_RR = 0.25*(thrust - up/ARM_D - uq/ARM_D - ur/KAPPA)
#   T_RL = 0.25*(thrust + up/ARM_D - uq/ARM_D + ur/KAPPA)
#   T_FL = 0.25*(thrust + up/ARM_D + uq/ARM_D - ur/KAPPA)
# ARM_D/KAPPA are imported from tools/sysid/_generated_params.py (machine-
# generated from control/models/stampfly_physical.yaml by `sf params
# generate`) -- the SAME values `sf params check` verifies against
# actuator.cpp's ARM_D/KAPPA (tools/params_audit/params_manifest.py "arm"/
# "kappa" groups), so this cannot silently drift from firmware the way a
# hand-typed literal could.
# B^-1 配分幾何（actuator.cpp mixerCompute()、ARM_D/KAPPA）。ARM_D/KAPPA は
# tools/sysid/_generated_params.py から import する（`sf params generate` が
# control/models/stampfly_physical.yaml から機械生成）—— `sf params check`
# が actuator.cpp の ARM_D/KAPPA と照合しているのと同じ値（
# tools/params_audit/params_manifest.py の "arm"/"kappa" グループ）なので、
# 手書きリテラルのように黙ってファームからずれることがない。
_ARM_D: float = EXPECTED_ARM
_KAPPA: float = EXPECTED_KAPPA

# Motor curve (actuator.cpp thrustToDuty()): omega=sqrt(T/Ct);
# volts=Am*omega^2+Bm*omega+Cm; duty=volts/vbat. MOTOR_CT is imported the
# same way as ARM_D/KAPPA above (EXPECTED_CT, `sf params check` "C_T" group).
# MOTOR_AM/MOTOR_BM/MOTOR_CM are the "flight_anchored_motor_curve" family
# (control/models/stampfly_physical.yaml) -- NOT machine-generated on the
# Python side as of this writing (see params_manifest.py's own comment on its
# hand-typed EXPECTED_AM_FA/EXPECTED_BM_FA/EXPECTED_CM), so these three are
# hand-copied from actuator.cpp exactly as params_manifest.py's copies are,
# and registered in that same manifest ("Am_flight_anchored"/
# "Bm_flight_anchored"/"Cm" groups) so `sf params check` catches drift here
# too.
# モータ曲線（actuator.cpp の thrustToDuty()）: omega=sqrt(T/Ct);
# volts=Am*omega^2+Bm*omega+Cm; duty=volts/vbat。MOTOR_CT は上の ARM_D/KAPPA
# と同じく import する（EXPECTED_CT、`sf params check` "C_T" グループ）。
# MOTOR_AM/MOTOR_BM/MOTOR_CM は "flight_anchored_motor_curve" ファミリ
# （control/models/stampfly_physical.yaml）—— 本稿執筆時点で Python 側は
# 機械生成されていない（params_manifest.py 自身の EXPECTED_AM_FA/
# EXPECTED_BM_FA/EXPECTED_CM 手書きコメント参照）ため、この3つは
# params_manifest.py のコピーと同様に actuator.cpp から手動転記し、同じ
# マニフェスト（"Am_flight_anchored"/"Bm_flight_anchored"/"Cm" グループ）に
# 登録して `sf params check` がここのずれも検出できるようにする。
_MOTOR_CT: float = EXPECTED_CT
_MOTOR_AM: float = 6.0368e-8    # V/(rad/s)^2 -- flight_anchored_motor_curve.Am
_MOTOR_BM: float = 6.699042e-4  # V/(rad/s)   -- flight_anchored_motor_curve.Bm
_MOTOR_CM: float = 1.53e-2      # V           -- flight_anchored_motor_curve.Cm

# Nominal 1S LiPo voltage (actuator.cpp V_BATT_NOMINAL) -- fallback ONLY when
# the bundle DataFrame has no `voltage` column (no `status` stream in the
# bundle) or the voltage is implausible. A short flight's real cell
# voltage stays close enough to this for the fit to remain useful; it is
# reported (duty_reason) whenever used so the caller knows the accuracy
# caveat applies.
# 公称 1S LiPo 電圧（actuator.cpp の V_BATT_NOMINAL）—— バンドルの DataFrame
# に `voltage` 列が無い（一式に `status` ストリームが無い）か値が非現実的な
# 場合のみフォールバックする。短時間フライトなら実際のセル電圧はこの値に十分
# 近く、フィットは実用的なまま。使用時は必ず duty_reason で伝え、精度低下の
# 可能性を呼び出し側に示す。
_V_BATT_NOMINAL: float = 3.7
_V_BATT_MIN: float = 2.5   # below this, a logged vbat is treated as invalid (actuator.cpp V_BATT_MIN)


def _thrust_from_duty(duty: np.ndarray, vbat: np.ndarray) -> np.ndarray:
    """
    Invert actuator.cpp's thrustToDuty(): recover per-motor thrust [N] from
    an OBSERVED duty and the battery voltage at that instant.
    actuator.cpp の thrustToDuty() を逆算し、観測された duty とその瞬間の
    バッテリ電圧から各モータ推力 [N] を復元する。

    Forward model: omega = sqrt(T/Ct); volts = Am*omega^2 + Bm*omega + Cm;
    duty = volts/vbat (thrust<=0 -> duty=MIN_DUTY=0). Inverting: given
    volts = duty*vbat, solve the quadratic Am*omega^2 + Bm*omega +
    (Cm-volts) = 0 for the positive root, then T = Ct*omega^2. Uses the
    OBSERVED (possibly clamped/saturated) duty, so saturation is correctly
    reflected in the recovered thrust -- same design intent as the "legacy"
    inversion's T-cancellation comment above (no assumption the actuator
    never saturated).
    順方向モデル: omega=sqrt(T/Ct); volts=Am*omega^2+Bm*omega+Cm;
    duty=volts/vbat（thrust<=0 なら duty=MIN_DUTY=0）。逆算: volts=duty*vbat
    として、2次方程式 Am*omega^2+Bm*omega+(Cm-volts)=0 を正の根について解き、
    T=Ct*omega^2 とする。観測された（飽和していれば飽和済みの）duty を
    そのまま使うため、飽和も正しく復元推力に反映される（上の "legacy"
    逆算の T相殺コメントと同じ設計意図: アクチュエータが飽和しなかったと
    仮定しない）。

    Args:
        duty: observed motor duty array [0, 1]
        vbat: battery voltage array [V], same length as duty

    Returns:
        Per-motor thrust array [N] (same length), clamped to >= 0.
    """
    volts = duty * vbat
    # volts <= Cm -> omega=0 (below the curve's zero-speed intercept, same as
    # the forward function's thrust<=0 -> MIN_DUTY=0 boundary).
    # volts <= Cm なら omega=0（曲線のゼロ速度切片以下 -- 順方向関数の
    # thrust<=0 -> MIN_DUTY=0 という境界条件と対応）。
    discriminant = np.maximum(_MOTOR_BM ** 2 - 4.0 * _MOTOR_AM * (_MOTOR_CM - volts), 0.0)
    omega = np.maximum((-_MOTOR_BM + np.sqrt(discriminant)) / (2.0 * _MOTOR_AM), 0.0)
    return _MOTOR_CT * omega ** 2


def _duty_differential_vehicle(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    vbat: np.ndarray, axis: str,
) -> np.ndarray:
    """
    Invert firmware/vehicle's ACTUAL mixer (B^-1 allocation + nonlinear motor
    curve, sf_actuator/actuator.cpp) to recover the per-axis differential
    TORQUE command [Nm] that produced the 4 motor duties. --mixer vehicle.
    firmware/vehicle の実際のミキサー（B^-1配分＋非線形モータ曲線、
    sf_actuator/actuator.cpp）を逆算し、4モータduty から軸別の差動トルク
    指令 [Nm] を復元する。--mixer vehicle。

    Unlike _duty_differential_legacy_linear(), the per-motor thrust must be
    recovered FIRST (_thrust_from_duty(), which needs vbat -- the motor curve
    is nonlinear and voltage-dependent) before the B^-1 combination below,
    which is algebraically the SAME sign pattern as the legacy linear mixer
    (same derivation technique -- T/thrust cancels identically) but with
    ARM_D (roll/pitch) and KAPPA (yaw) as the per-channel scale instead of a
    single shared k:
      roll  = ARM_D * (-T_fr - T_rr + T_rl + T_fl)
      pitch = ARM_D * ( T_fr - T_rr - T_rl + T_fl)
      yaw   = KAPPA * ( T_fr - T_rr + T_rl - T_fl)
    _duty_differential_legacy_linear() と異なり、下の B^-1 結合の前に、まず
    各モータ推力を復元する必要がある（_thrust_from_duty()。モータ曲線が
    非線形・電圧依存のため vbat が要る）。結合式自体は legacy 線形ミキサー
    と代数的に同じ符号パターン（同じ導出技法 -- T/thrust が同様に相殺する）
    だが、単一の共有係数 k の代わりにチャンネルごとの ARM_D（roll/pitch）と
    KAPPA（yaw）を使う。

    Args:
        duty_fr, duty_rr, duty_rl, duty_fl: motor duty arrays [0, 1]
        vbat: battery voltage array [V], same length as the duty arrays
        axis: 'roll', 'pitch', or 'yaw'

    Returns:
        Differential torque command array [Nm] (same length as the inputs).
    """
    t_fr = _thrust_from_duty(duty_fr, vbat)
    t_rr = _thrust_from_duty(duty_rr, vbat)
    t_rl = _thrust_from_duty(duty_rl, vbat)
    t_fl = _thrust_from_duty(duty_fl, vbat)

    if axis == 'roll':
        return _ARM_D * (-t_fr - t_rr + t_rl + t_fl)
    elif axis == 'pitch':
        return _ARM_D * (t_fr - t_rr - t_rl + t_fl)
    elif axis == 'yaw':
        return _KAPPA * (t_fr - t_rr + t_rl - t_fl)
    else:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")


# Aligned-DataFrame column carrying the battery voltage (bundle's `status`
# stream, held/forward-filled onto every row -- see
# tools/sysid/loader.py's load_aligned() docstring). Replaces the retired
# flat-CSV format's `vbat` column (same physical quantity, renamed to match
# protocol/spec/flight_log.yaml's status.csv). ONLY needed for --mixer
# vehicle's _thrust_from_duty(); --mixer legacy never reads it.
# 整列済み DataFrame でバッテリ電圧を運ぶ列（一式の `status` ストリーム、
# 全行へ前方保持 -- tools/sysid/loader.py の load_aligned() docstring
# 参照）。廃止した平坦CSV形式の `vbat` 列を置き換える（同じ物理量、
# protocol/spec/flight_log.yaml の status.csv に合わせて改名）。--mixer
# vehicle の _thrust_from_duty() だけが必要とする列 -- --mixer legacy は
# 読まない。
_VOLTAGE_COL = 'voltage'

# NOTE: the retired flat-CSV format's `duty_rate_hz` and
# `ctrl_output_rate_hz` columns (and the CTRL_OUTPUT_TORQUE_COL name-per-axis
# dict) are gone -- see _load_axis_data() for how their job is done now:
# duty_rate_hz's role (tell genuine 400Hz duty apart from a 50Hz-held
# fallback) is now answered STRUCTURALLY via
# `"motor" in df.attrs["bundle_streams"]` rather than a data column, and
# ctrl_output_rate_hz's role (tell genuine 400Hz control_output apart from a
# forward-filled one) no longer applies at all -- the bundle's `ctrl_output`
# stream is ALWAYS native-rate when present (a lockstep stream merged on
# `seq`, never forward-filled), so presence of the `torque_<axis>` column
# already answers that.
# 注: 廃止した平坦CSV形式の `duty_rate_hz`・`ctrl_output_rate_hz` 列
# （および軸ごとの列名辞書 CTRL_OUTPUT_TORQUE_COL）はもう無い --
# それぞれの役目が今どう果たされるかは _load_axis_data() 参照:
# duty_rate_hz の役目（本物の400Hz dutyと50Hz保持フォールバックの見分け）は
# データ列ではなく `"motor" in df.attrs["bundle_streams"]` という構造的事実
# で答えが出るようになり、ctrl_output_rate_hz の役目（本物の400Hz
# control_output と前方補完の見分け）はそもそも不要になった -- 一式の
# `ctrl_output` ストリームは存在すれば常にネイティブレート（`seq` で結合する
# ロックステップ系ストリームであり前方補完されない）なので、
# `torque_<axis>` 列の有無が既にその答えになっている。

# Heuristic threshold for _classify_duty_source()'s fallback branch, used
# when its `duty_rate_hz_col` argument is unavailable -- kept UNCHANGED and
# shared verbatim with tools/log_analyzer/rate_sysid.py (see
# _classify_duty_source()'s docstring; both callers now always synthesize
# and pass a duty_rate_hz array, so in practice this fallback branch is
# dormant, but the function itself stays untouched rather than special-
# cased). If more than this fraction of rows repeat the previous row's 4
# duties exactly, the data looks like a 50Hz-forward-filled staircase (a
# genuine 50Hz CtrlRef entry repeats for ~8 consecutive 400Hz rows, ~87.5%
# duplicates) rather than real 400Hz duty (which essentially never repeats
# bit-for-bit).
# _classify_duty_source() のフォールバック分岐（`duty_rate_hz_col` 引数が
# 使えないとき）のヒューリスティック閾値 -- tools/log_analyzer/
# rate_sysid.py と全く同じものを変更せず共有する（_classify_duty_source()
# の docstring 参照。両呼び出し元とも今は常に duty_rate_hz 配列を合成して
# 渡すため、実際にはこのフォールバック分岐は休眠状態だが、関数自体は特別
# 扱いせずそのままにする）。直前行と4 duty が完全一致する行の割合がこれを
# 超えたら、50Hz前方補完の階段状データ（本物の50Hz CtrlRef エントリは
# 400Hz中約8行連続で同一値、重複率約87.5%）とみなす（本物の400Hz duty は
# ビット単位で繰り返すことがほぼ無い）。
_DUTY_STAIRSTEP_FRACTION_THRESHOLD = 0.5


def _mixer_conversion_factor(
    candidate: np.ndarray,
    actual_torque: np.ndarray,
) -> Optional[Tuple[float, float]]:
    """
    Robust regression between a candidate u_plant proxy and the duty-derived
    ACTUAL torque (_duty_differential_vehicle(), the real motor-curve
    inversion -- a statement about StampFly's hardware, not about which
    firmware's mixer flew). The slope is the mixer's static gain/conversion
    factor `c`: see the rate-sysid design memo (docs/events/sci_tutorial_2026,
    2026-09-09), §07 "K_measured = physical gain (K=1/I) x mixer gain (c)"
    and §08 "measure c from the log".

    Two calling conventions, same math:
      - candidate = control_output.torque [Nm] (commanded, pre-mixer) ->
        `c` is DIMENSIONLESS, ~1 for a mixer whose motor-curve model matches
        reality (e.g. firmware/vehicle's flight-anchored curve).
      - candidate = _duty_differential_legacy_linear()'s output (arbitrary
        duty-differential units, no physical meaning of its own -- --mixer
        legacy's u_plant) -> `c` is a CONVERSION FACTOR [Nm per legacy
        duty-unit], with no "should be 1" expectation. Useful to rescale
        firmware/workshop's lesson_06-style duty-differential K into the
        physical 1/I scale -- see auto-memory
        project_workshop_mixer_unification_deferred.md.

    候補の u_plant プロキシと、duty から逆算した実トルク
    （_duty_differential_vehicle()、実モータ曲線逆算 -- どのファームの
    ミキサーが飛んだかでなく StampFly の実ハードウェアについての事実）との
    頑健な回帰。傾きがミキサーの静的ゲイン/換算係数 `c`（2026-09-09
    レート同定設計メモ §07「K_measured = 物理ゲイン(K=1/I) x
    ミキサーゲイン(c)」・§08「ログから c を測る」参照）。

    Args:
        candidate: candidate u_plant proxy, same length as actual_torque
        actual_torque: _duty_differential_vehicle() output [Nm]

    Returns:
        (slope, r_squared), or None if candidate has too little independent
        variation to regress (e.g. a near-constant signal).
    """
    if len(candidate) < 8 or len(actual_torque) < 8 or np.std(candidate) < 1e-9:
        return None
    slope, intercept = np.polyfit(candidate, actual_torque, 1)
    predicted = slope * candidate + intercept
    ss_res = np.sum((actual_torque - predicted) ** 2)
    ss_tot = np.sum((actual_torque - np.mean(actual_torque)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return float(slope), float(r_squared)


def _classify_duty_source(
    duty_fr: np.ndarray, duty_rr: np.ndarray, duty_rl: np.ndarray, duty_fl: np.ndarray,
    duty_rate_hz_col: Optional[np.ndarray],
) -> Tuple[str, str]:
    """
    Classify whether the duty_FR/RR/RL/FL series is genuine 400Hz motor
    duty (safe input for the 'duty' fit mode) or a 50Hz-forward-filled
    staircase (would silently identify off a stale/quantized signal -- must
    NOT be auto-selected).
    duty_FR/RR/RL/FL の系列が本物の400Hz モータduty（'duty'入力モードで
    安全）か、50Hz前方補完の階段状データ（黙って使うと古い/粗い信号で
    誤同定する -- 自動選択してはならない）かを判別する。

    `duty_rate_hz_col` is the per-row duty rate [Hz]. For a flight-log
    bundle (`_load_axis_data()` here, `fit_from_df()` in rate_sysid.py) it
    is synthesized from the STRUCTURAL fact "is the `motor` stream present
    in the bundle" (400 if present, 50 if only ctrl_ref supplied the duty)
    -- the definitive answer, no data heuristic involved. For the SILS
    `rate_stream.csv` still read by rate_sysid.py's `load_csv()` (Phase 3
    removes it) it is that file's real `duty_rate_hz` column; None when
    that CSV predates the column, which falls back to the stairstep
    heuristic below.
    `duty_rate_hz_col` は行ごとの duty レート[Hz]。フライトログ一式
    （本ファイルの `_load_axis_data()`、rate_sysid.py の `fit_from_df()`）
    では「一式に `motor` ストリームが在るか」という構造的事実から合成する
    （在れば400、ctrl_ref だけが duty を供給していれば50）-- データの
    推測ではなく確定的な答え。rate_sysid.py の `load_csv()` がまだ読む
    SILS の `rate_stream.csv`（Phase 3 で削除）では同ファイルの実際の
    `duty_rate_hz` 列。その列を持たない古い CSV では None となり、下の
    階段状ヒューリスティックへフォールバックする。

    Returns:
        (quality, reason). quality is 'duty400' (safe) or 'duty50' (unsafe --
        callers should fall back to the 'kp' mode). `reason` is a
        human-readable explanation, always non-empty, for display/errors.
    """
    if duty_rate_hz_col is not None and len(duty_rate_hz_col) > 0:
        hz = float(np.median(duty_rate_hz_col))
        if hz >= 200.0:   # nominal 400, generous margin above the 50Hz case
            return ('duty400',
                    f"{hz:.0f} Hz duty -- genuine motor duty (bundle `motor` stream / "
                    "Duty400 0x4A entry present)")
        return ('duty50',
                f"{hz:.0f} Hz duty only -- no `motor` stream in the bundle / no Duty400 "
                "entry, so duty_FR/RR/RL/FL is the 50Hz CtrlRef value, forward-filled")

    # No duty_rate_hz column (CSV from an older udp_capture.py) -- fall back
    # to the consecutive-duplicate-row heuristic.
    # duty_rate_hz 列が無い（旧 udp_capture.py の CSV）-- 連続重複行の
    # ヒューリスティックにフォールバック。
    if len(duty_fr) < 2:
        return 'duty400', "too few rows for the stairstep heuristic -- assuming 400Hz"

    same = ((duty_fr[1:] == duty_fr[:-1]) & (duty_rr[1:] == duty_rr[:-1])
            & (duty_rl[1:] == duty_rl[:-1]) & (duty_fl[1:] == duty_fl[:-1]))
    dup_fraction = float(np.mean(same))
    threshold_pct = _DUTY_STAIRSTEP_FRACTION_THRESHOLD * 100.0
    if dup_fraction > _DUTY_STAIRSTEP_FRACTION_THRESHOLD:
        return ('duty50',
                f"no duty_rate_hz column; {dup_fraction * 100:.0f}% of rows repeat "
                f"the previous row's duty exactly (> {threshold_pct:.0f}% threshold) "
                "-- looks like a 50Hz-forward-filled staircase")
    return ('duty400',
            f"no duty_rate_hz column; only {dup_fraction * 100:.0f}% of rows repeat "
            f"the previous row's duty (<= {threshold_pct:.0f}% threshold) -- looks "
            "like genuine high-rate duty")


@dataclass
class PlantFitResult:
    """Plant model identification result / プラントモデル同定結果

    Model: G_p(s) = K / (s * (tau_m * s + 1))
    """
    K: float              # Plant gain [rad/s^2 per duty]
    tau_m: float          # Motor time constant [s]
    K_std: float          # K uncertainty (std dev across segments)
    tau_m_std: float      # tau_m uncertainty (std dev across segments)
    r_squared: float      # Fit quality (mean R^2 across segments)
    rmse: float           # RMSE [rad/s] (mean across segments)
    axis: str             # 'roll', 'pitch', or 'yaw'
    kp_used: Optional[float]  # Kp used for plant input reconstruction ('kp' mode only)
    n_segments: int       # Number of segments used for fitting
    input_mode: str = 'kp'    # 'duty' (mixer-inverse of motor_duty_*) or 'kp'
                               # (legacy Kp*(target-gyro) reconstruction)
    mixer: str = 'legacy'     # 'legacy' or 'vehicle' -- which duty inversion
                               # produced K (see module docstring --mixer).
                               # Meaningless when input_mode == 'kp'.
    duty_quality: Optional[str] = None  # 'duty400'/'duty50'/None -- see
                                         # _classify_duty_source()
    duty_reason: str = ''     # human-readable reason for duty_quality, always
                               # shown so the input-mode choice is explained
    mixer_gain: Optional[float] = None       # slope from _mixer_conversion_factor(),
                                              # None when not computable (see
                                              # mixer_gain_label for what it means)
    mixer_gain_r_squared: Optional[float] = None
    mixer_gain_label: str = ''   # e.g. "c (control_output vs duty-derived
                                  # actual torque, dimensionless)" or "legacy
                                  # duty-unit -> real torque [Nm/unit]" --
                                  # empty when mixer_gain is None
    n_segments_excitation_dropped: int = 0  # segments that passed the
                                  # u_plant activity filter but were DROPPED
                                  # for insufficient REFERENCE (target)
                                  # excitation -- see target_excitation_note.
                                  # 0 when every active segment had enough
                                  # target motion, or when no such check
                                  # applies.
    target_excitation_note: str = ''  # non-empty when n_segments_excitation_dropped > 0:
                                  # warns that a near-constant target(t) makes
                                  # corr(u, y) misleadingly negative in
                                  # closed-loop P-control data (u = Kp*(target
                                  # - y) is then dominated by -Kp*y), which is
                                  # a classic closed-loop identifiability
                                  # pitfall, not a plant-sign bug -- fly with
                                  # larger/more frequent stick motion instead.
    kp_source: Optional[str] = None  # 'user' (--kp given), 'fir_auto'
                                  # (_estimate_kp_fir() estimated it from
                                  # this log's own duty), or None
                                  # (kp_used is None -- direct fit, no Kp
                                  # involved). See fit_plant()'s auto ladder.
    kp_auto_r_squared: Optional[float] = None  # _estimate_kp_fir()'s R^2
                                  # for the u~e FIR regression -- only set
                                  # when kp_source == 'fir_auto'; a rough
                                  # confidence signal for the auto-estimated
                                  # Kp, NOT the plant fit's own r_squared.
    crash_truncated_at: Optional[float] = None  # flight-log time [s] where
                                  # _load_axis_data()'s automatic crash/
                                  # anomaly truncation cut the data, or None
                                  # if no truncation happened. Everything at
                                  # or after this time was excluded from
                                  # fitting.

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        defaults = get_flat_defaults()
        # K's reference/units depend on which mixer produced it (duty-diff
        # gain for 'legacy', torque gain 1/I_axis for 'vehicle') -- see
        # REFERENCE_PLANT_GAINS vs REFERENCE_PLANT_GAINS_VEHICLE above.
        # K の参照値・単位は、どちらのミキサーが生成したかで異なる
        # （'legacy' は duty差動ゲイン、'vehicle' はトルクゲイン 1/I_axis）
        # -- 上の REFERENCE_PLANT_GAINS / REFERENCE_PLANT_GAINS_VEHICLE 参照。
        ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if self.mixer == 'vehicle' else REFERENCE_PLANT_GAINS
        ref_K = ref_gains.get(self.axis, 0.0)
        ref_tau_m = defaults['tau_m']

        result: Dict[str, Any] = {
            'method': 'plant_fit',
            'timestamp': datetime.now().isoformat(),
            'model': 'K / (s * (tau_m * s + 1))',
            'axis': self.axis,
            'input_mode': self.input_mode,
            'mixer': self.mixer,
            'duty_quality': self.duty_quality,
            'duty_reason': self.duty_reason,
            'mixer_gain': self.mixer_gain,
            'mixer_gain_r_squared': self.mixer_gain_r_squared,
            'mixer_gain_label': self.mixer_gain_label,
            'kp_used': self.kp_used,
            'kp_source': self.kp_source,
            'kp_auto_r_squared': (round(self.kp_auto_r_squared, 4)
                                   if self.kp_auto_r_squared is not None else None),
            'crash_truncated_at': self.crash_truncated_at,
            'n_segments': self.n_segments,
            'n_segments_excitation_dropped': self.n_segments_excitation_dropped,
            'target_excitation_note': self.target_excitation_note,
            'estimated': {
                'K': round(self.K, 2),
                'K_uncertainty': round(self.K_std, 2),
                'tau_m': round(self.tau_m, 4),
                'tau_m_uncertainty': round(self.tau_m_std, 4),
            },
            'fit_quality': {
                'r_squared': round(self.r_squared, 4),
                'rmse': round(self.rmse, 4),
            },
            'reference': {
                'K': ref_K,
                'tau_m': ref_tau_m,
            },
            'comparison': {},
        }

        # Comparison with reference values
        # 参照値との比較
        if ref_K > 0:
            K_err = abs(self.K - ref_K) / ref_K * 100
            result['comparison']['K'] = {
                'error_percent': round(K_err, 1),
                'status': 'OK' if K_err < 30 else 'CHECK',
            }
        if ref_tau_m > 0:
            tau_err = abs(self.tau_m - ref_tau_m) / ref_tau_m * 100
            result['comparison']['tau_m'] = {
                'error_percent': round(tau_err, 1),
                'status': 'OK' if tau_err < 30 else 'CHECK',
            }

        # Derived: design Kp for zeta=0.7
        # Closed-loop: Kp = 1 / (4 * zeta^2 * K * tau_m)
        zeta = 0.7
        if self.K > 0 and self.tau_m > 0:
            Kp_design = 1.0 / (4.0 * zeta**2 * self.K * self.tau_m)
            result['derived'] = {
                'design_Kp_zeta_0_7': round(Kp_design, 4),
            }
            if ref_K > 0:
                Kp_ref = 1.0 / (4.0 * zeta**2 * ref_K * ref_tau_m)
                result['derived']['reference_Kp_zeta_0_7'] = round(Kp_ref, 4)

        return result


def _simulate_plant(
    K: float,
    tau_m: float,
    u: np.ndarray,
    dt: float,
    omega0: float,
    z0: float = 0.0,
) -> np.ndarray:
    """
    Simulate plant G_p(s) = K / (s * (tau_m * s + 1))
    プラントをシミュレート

    State space representation:
        x1_dot = x2               (omega_dot = z)
        x2_dot = -x2/tau_m + K*u/tau_m  (motor first-order dynamics)
        y      = x1               (output = angular velocity)

    Uses exact discretization for motor dynamics (x2)
    and trapezoidal integration for the integrator (x1).

    Args:
        K: Plant gain [rad/s^2 per duty]
        tau_m: Motor time constant [s]
        u: Plant input array (duty)
        dt: Sample period [s]
        omega0: Initial angular velocity [rad/s]
        z0: Initial motor-filtered acceleration [rad/s^2]

    Returns:
        Simulated angular velocity [rad/s]
    """
    n = len(u)
    alpha = np.exp(-dt / tau_m)
    gain = K * (1.0 - alpha)

    # Motor-filtered acceleration (x2 state)
    # 1次遅れ（モータ動特性）の厳密離散化
    z = np.empty(n)
    z[0] = z0
    for i in range(1, n):
        z[i] = alpha * z[i - 1] + gain * u[i - 1]

    # Integrate to angular velocity (trapezoidal rule)
    # 台形積分で角速度を計算
    omega = np.empty(n)
    omega[0] = omega0
    omega[1:] = omega0 + np.cumsum((z[:-1] + z[1:]) * 0.5 * dt)

    return omega


def _fit_segment(
    u_seg: np.ndarray,
    y_seg: np.ndarray,
    dt: float,
    K_init: float = 100.0,
    tau_m_init: float = 0.02,
    K_bounds: Tuple[float, float] = (1.0, 1000.0),
) -> Optional[Tuple[float, float, float, float]]:
    """
    Fit K and tau_m for a single data segment
    単一セグメントの K, tau_m をフィット

    Args:
        u_seg: Plant input for segment
        y_seg: Measured output (gyro) for segment
        dt: Sample period [s]
        K_init: Initial guess for K
        tau_m_init: Initial guess for tau_m
        K_bounds: (min, max) bounds for K in the optimizer. Default is the
            "legacy" duty-differential-gain scale ([rad/s^2 / duty],
            REFERENCE_PLANT_GAINS ~ 20-100). --mixer vehicle's K is a
            torque gain [rad/s^2 / Nm] several orders of magnitude larger
            (REFERENCE_PLANT_GAINS_VEHICLE ~ 1e5) -- callers in that mode
            MUST pass a matching K_bounds, or the true optimum sits outside
            this default and the fit degenerates to the bound.
            K の最適化境界（既定値）。"legacy" の duty差動ゲイン単位
            （[rad/s^2 / duty]、REFERENCE_PLANT_GAINS ~ 20-100）向け。
            --mixer vehicle の K はトルクゲイン単位 [rad/s^2 / Nm] で
            桁が何桁も大きい（REFERENCE_PLANT_GAINS_VEHICLE ~ 1e5）—— この
            モードの呼び出し側は対応する K_bounds を渡すこと。渡さないと
            真の最適値が既定境界の外にあり、フィットが境界に張り付いて
            縮退する。

    Returns:
        (K, tau_m, r_squared, rmse) or None if fitting fails
    """
    if len(u_seg) < 20:
        return None

    omega0 = y_seg[0]
    # Estimate initial angular acceleration from first few samples
    # 最初の数サンプルから初期角加速度を推定
    n_init = min(10, len(y_seg) - 1)
    z0 = float(y_seg[n_init] - y_seg[0]) / (n_init * dt)

    # BOTH params are log-transformed, not just tau_m. K_bounds spans up to
    # 9 orders of magnitude for --mixer vehicle / --input control_output
    # (REFERENCE_PLANT_GAINS_VEHICLE ~1e5, bounds (1, 1e9)) -- L-BFGS-B's
    # finite-difference gradient step is sized for O(1)-scale variables, so
    # an UNTRANSFORMED K at that magnitude gets a numerically negligible
    # gradient and the optimizer silently fails to move away from K_init at
    # all (discovered 2026-09-10: a synthetic fit with K_init deliberately
    # != K_true converged to EXACTLY K_init, not the true optimum -- a
    # previous 'vehicle'-mode selftest could not catch this because it
    # happened to seed K_init == K_true). Log-transforming K puts it on the
    # same O(1)-ish footing as log(tau_m) regardless of the physical scale.
    # KもtauMと同じく対数変換する（tau_mだけでなく）。K_bounds は --mixer
    # vehicle / --input control_output で最大9桁に及ぶ
    # （REFERENCE_PLANT_GAINS_VEHICLE ~1e5、bounds (1, 1e9)）—— L-BFGS-B の
    # 数値差分勾配ステップは O(1) スケール変数向けに設計されているため、
    # 変換なしの K がこの桁だと勾配が数値的に無視できるほど小さくなり、
    # 最適化器が K_init から全く動かず黙って失敗する（2026-09-10発見: K_init
    # を意図的に K_true と違えた合成データのフィットが、真の最適値ではなく
    # K_init そのものに収束した —— 従来の 'vehicle' モードのselftestは
    # たまたま K_init == K_true で種付けしていたためこれを検出できなかった）。
    # K を対数変換すれば、物理的な桁に関わらず log(tau_m) と同様 O(1) 相当の
    # 足場に乗る。
    def objective(params):
        K = np.exp(params[0])
        tau_m = np.exp(params[1])  # log transform ensures tau_m > 0
        y_sim = _simulate_plant(K, tau_m, u_seg, dt, omega0, z0)
        return np.mean((y_sim - y_seg) ** 2)

    log_K_bounds = (np.log(K_bounds[0]), np.log(K_bounds[1]))
    try:
        result = minimize(
            objective,
            x0=[np.log(K_init), np.log(tau_m_init)],
            method='L-BFGS-B',
            bounds=[log_K_bounds, (np.log(0.003), np.log(0.5))],
            options={'maxiter': 200},
        )
    except Exception:
        return None

    if not result.success and result.fun > 1.0:
        return None

    K_opt = np.exp(result.x[0])
    tau_m_opt = np.exp(result.x[1])

    # Compute fit quality metrics
    # フィット品質の計算
    y_sim = _simulate_plant(K_opt, tau_m_opt, u_seg, dt, omega0, z0)
    residuals = y_seg - y_sim
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y_seg - np.mean(y_seg)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    return K_opt, tau_m_opt, r_squared, rmse


def _simulate_closed_loop(
    K: float,
    tau_m: float,
    kp: float,
    target: np.ndarray,
    dt: float,
    omega0: float,
) -> np.ndarray:
    """
    Simulate the CLOSED loop directly: u(t) = kp*(target(t) - omega(t)),
    G_p(s) = K/(s*(tau_m*s+1)). Same exact/trapezoidal discretization as
    _simulate_plant(), just closing the loop causally (u[i-1] depends only
    on omega[i-1], never on the not-yet-computed omega[i], so no algebraic
    loop needs solving). Identical scheme to the closed-loop P-control
    synthesis already used by selftest() to GENERATE synthetic flight data.
    閉ループを直接シミュレートする: u(t) = kp*(target(t) - omega(t))、
    G_p(s) = K/(s*(tau_m*s+1))。_simulate_plant() と全く同じ厳密/台形離散化
    で、ループを因果的に閉じるだけ（u[i-1] は omega[i-1] のみに依存し、まだ
    計算されていない omega[i] には依存しない -- 代数ループを解く必要が無い）。
    selftest() が合成フライトデータの生成に既に使っている閉ループP制御合成
    と同一の方式。

    2026-09-10 rationale (see docs/events/sci_tutorial_2026, rate-sysid
    design memo addendum): fitting G_p directly against the RECONSTRUCTED
    u=kp*(target-gyro) (the "direct"/'kp' input mode) is a naive open-loop
    fit applied to closed-loop data -- textbook biased/ill-conditioned
    whenever the reference doesn't dominate the feedback term, which real
    human-piloted flight essentially never achieves (a human cannot produce
    persistent ~8Hz stick motion, and trying to on a bare-P airframe risks a
    crash). The INDIRECT approach instead fits the closed-loop transfer
    function target->gyro directly (target is a genuinely external,
    pilot-commanded signal, not algebraically entangled with gyro the way
    u is) and only uses the KNOWN kp to back out K, tau_m algebraically.
    Empirically far more robust on real flight data (see two real
    lesson_07 test flights, 2026-09-10: this recovered K within 1-25% of
    the L6 theoretical values on ALL THREE axes with positive R^2 on every
    segment, where the direct method gave R^2 < 0 and K off by 1-3 orders
    of magnitude on the same data).
    2026-09-10 の根拠（docs/events/sci_tutorial_2026、レート同定設計メモ
    追補参照）: 復元した u=kp*(target-gyro) に対して G_p を直接フィットする
    （"direct"/'kp' 入力モード）のは、閉ループデータに素朴な開ループフィット
    を適用しているに過ぎない -- 参照信号がフィードバック項を圧倒しない限り
    教科書通りバイアス・悪条件になる。人間の操縦飛行ではまずこれを達成でき
    ない（人間は持続的な~8Hzのスティック動作を出せず、単純P制御機体でそれを
    試みるのは墜落のリスクがある）。間接法は代わりに、target->gyro の閉ループ
    伝達関数を直接フィットする（target はパイロットが指令する真に外部の信号
    であり、u のように gyro と代数的に絡み合っていない）。既知の kp だけを
    使って代数的に K, tau_m を逆算する。実飛行データでは実証的に遥かに頑健
    （2026-09-10、実際の実習7テスト飛行2本で確認: 直接法は R^2<0・K が
    1〜3桁ズレたのに対し、間接法は全3軸・全セグメントでR^2が正、
    L6理論値との誤差1〜25%でKを復元した）。

    Args:
        K, tau_m: candidate plant parameters (same meaning as _simulate_plant)
        kp: KNOWN, constant P gain used during the flight (must match firmware)
        target: reference/target signal [rad/s] (rate_ref_<axis>)
        dt: sample period [s]
        omega0: initial gyro reading [rad/s]

    Returns:
        Simulated gyro (omega) array, same length as target.
    """
    n = len(target)
    alpha = np.exp(-dt / tau_m)
    gain = K * (1.0 - alpha)

    omega = np.empty(n)
    z = np.empty(n)
    omega[0] = omega0
    z[0] = kp * (target[0] - omega0)
    # A candidate K near the top of the wide vehicle-scale K_bounds
    # (1, 1e9), tried by the optimizer while probing on a poor-quality
    # segment, can overflow float64 in this loop -- objective() below
    # already checks np.all(np.isfinite(y_sim)) and rejects such a
    # candidate with a large penalty, so silence the resulting (harmless,
    # but noisy/confusing for a live tutorial audience) RuntimeWarning
    # rather than let numpy print it.
    # 探索境界（vehicle尺度の K_bounds=(1, 1e9)）の上限付近の候補を、質の
    # 低いセグメントで最適化器が試すと、このループで float64 がオーバー
    # フローすることがある -- 下の objective() は既に
    # np.all(np.isfinite(y_sim)) を確認し、そのような候補を大きなペナルティ
    # で棄却するので、無害だが（実習の場では紛らわしい）RuntimeWarning を
    # numpy に出させず抑制する。
    with np.errstate(over='ignore', invalid='ignore'):
        for i in range(1, n):
            u_prev = kp * (target[i - 1] - omega[i - 1])
            z[i] = alpha * z[i - 1] + gain * u_prev
            omega[i] = omega[i - 1] + 0.5 * dt * (z[i - 1] + z[i])

    return omega


def _fit_segment_indirect(
    target_seg: np.ndarray,
    y_seg: np.ndarray,
    dt: float,
    kp: float,
    K_init: float = 100.0,
    tau_m_init: float = 0.02,
    K_bounds: Tuple[float, float] = (0.1, 1.0e4),
) -> Optional[Tuple[float, float, float, float]]:
    """
    Indirect closed-loop identification for a single segment: fit K, tau_m
    by simulating the CLOSED loop (target -> gyro) and comparing directly to
    the measured gyro -- see _simulate_closed_loop() for why this is far
    more robust than the direct u=kp*(target-gyro) fit on real (weakly/
    moderately excited, human-piloted) flight data.
    単一セグメントの間接閉ループ同定: 閉ループ（target -> gyro）をシミュレー
    ションし実測ジャイロと直接比較して K, tau_m をフィットする -- 実飛行
    （人間操縦、弱〜中程度の励振）データで直接法 u=kp*(target-gyro) より
    遥かに頑健な理由は _simulate_closed_loop() 参照。

    Args:
        target_seg: reference/target segment [rad/s]
        y_seg: measured gyro segment [rad/s]
        dt: sample period [s]
        kp: KNOWN, constant P gain used during the flight
        K_init, tau_m_init: initial guesses
        K_bounds: (min, max) for K -- default spans both legacy
            (duty-differential-gain, ~10-1000) and vehicle (torque-gain
            ~1e5) scales loosely since this method's K is whatever scale
            matches the *duty differential* target (kp is in the SAME
            legacy units the firmware's rate loop used), not a fixed
            physical torque gain.

    Returns:
        (K, tau_m, r_squared, rmse) or None if fitting fails
    """
    if len(target_seg) < 20:
        return None

    omega0 = y_seg[0]

    def objective(params):
        K = np.exp(params[0])
        tau_m = np.exp(params[1])
        y_sim = _simulate_closed_loop(K, tau_m, kp, target_seg, dt, omega0)
        if not np.all(np.isfinite(y_sim)):
            return 1e12
        return np.mean((y_sim - y_seg) ** 2)

    log_K_bounds = (np.log(K_bounds[0]), np.log(K_bounds[1]))
    try:
        result = minimize(
            objective,
            x0=[np.log(K_init), np.log(tau_m_init)],
            method='L-BFGS-B',
            bounds=[log_K_bounds, (np.log(0.0005), np.log(1.0))],
            options={'maxiter': 300},
        )
    except Exception:
        return None

    if not result.success and result.fun > 1.0:
        return None

    K_opt = np.exp(result.x[0])
    tau_m_opt = np.exp(result.x[1])

    y_sim = _simulate_closed_loop(K_opt, tau_m_opt, kp, target_seg, dt, omega0)
    residuals = y_seg - y_sim
    ss_res = np.sum(residuals ** 2)
    ss_tot = np.sum((y_seg - np.mean(y_seg)) ** 2)
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    rmse = float(np.sqrt(np.mean(residuals ** 2)))

    return K_opt, tau_m_opt, r_squared, rmse


# Minimum fit quality for _estimate_kp_fir()'s result to be trusted as a
# stand-in for a manually-typed --kp. Calibrated against real lesson_07
# flights (2026-09-10): well-excited logs scored R^2 0.84-0.99; a log with
# no real proportional relationship between e and u (wrong axis, dead
# duty column, near-zero excitation) scores far below this.
# _estimate_kp_fir() の結果を、手入力の --kp の代わりとして信頼してよい
# 最低フィット品質。実際の実習7フライト（2026-09-10）で較正: 十分に励振
# されたログは R^2 0.84〜0.99。e と u の間に本物の比例関係が無いログ
# （軸違い、duty列が死んでいる、励振ほぼ無し）はこれを大きく下回る。
_KP_AUTO_R2_FLOOR = 0.5


def _estimate_kp_fir(
    e: np.ndarray,
    u: np.ndarray,
    throttle: np.ndarray,
    seg_samples: int,
    taps: int = 1,
    ridge_scale: float = 1e-6,
) -> Optional[Tuple[float, float, int]]:
    """
    Auto-estimate the controller's effective proportional gain from a
    SINGLE flight log, with NO prior knowledge of Kp and NO assumption of
    a PID structure -- a short, ridge-regularized FIR regression of u(t)
    (the actual/reconstructed control effort) against lags of e(t) =
    target(t) - gyro(t). Returns (kp_est, r_squared, n_samples) using only
    in-flight (throttle > 0.3) segments, or None when there is too little
    data to fit.
    単一フライトログから、Kp の事前知識もPID構造の仮定も無しに、制御器の
    実効比例ゲインを自動推定する -- u(t)（実測/逆算した制御出力）を
    e(t) = target(t) - gyro(t) の複数ラグに回帰する、短くリッジ正則化した
    FIR回帰。飛行中（throttle > 0.3）の区間のみを使い、
    (kp_est, r_squared, n_samples) を返す。フィットに足るデータが無ければ
    None。

    kp_est = h(0), the FIR's zero-lag coefficient -- the instantaneous
    proportional-equivalent gain.

    Default taps=1 (a plain scalar regression of u against e, no lags) is
    DELIBERATE, not a placeholder -- discovered 2026-09-10 while building
    this function's selftest coverage: e(t) from a real (or realistic
    synthetic chirp-excited) flight is strongly autocorrelated sample to
    sample, so with taps>1 the lagged e(t-1), e(t-2), ... columns are
    near-collinear with e(t). Ordinary/ridge least squares then has no way
    to know the true relationship is purely instantaneous (h(0)=Kp,
    h(k>0)=0) and instead SPREADS the true h(0) weight across several
    correlated lags -- e.g. on the exact synthetic closed loop this
    selftest uses (true Kp=0.5, zero lag by construction), taps=8 recovered
    h(0)=0.434 (13% low) while taps=1 recovered h(0)=0.4999... (0.03% low).
    The controller-identification exercise earlier today (see
    fir_vs_pid.json / "実習7 同定ログ比較" artifact, §"PID基底 vs
    数値的(FIR)推定") used taps up to 40 to reveal genuine EXTRA dynamics
    (useful there -- R^2 rising with taps means "PID undersells this axis");
    but for THIS function's one job -- a trustworthy scalar Kp to feed
    _simulate_closed_loop() -- more taps only relearns h(0) more poorly, so
    keep taps=1 unless a caller has a specific reason to widen it.
    既定taps=1（ラグ無しの単純なスカラー回帰）は仮置きではなく意図的な選択
    -- このセルフテストを組んでいる最中に2026-09-10発見: 実際の（あるいは
    現実的な合成チャープ励振の）フライトの e(t) はサンプル間で強く自己相関
    するため、taps>1 にすると e(t-1), e(t-2), ... のラグ列が e(t) とほぼ
    共線になる。最小二乗（リッジ込みでも）は真の関係が瞬時のみ
    （h(0)=Kp、h(k>0)=0）だと知る術が無く、真の h(0) の重みを複数の相関
    ラグへ分散させてしまう -- 例えばこのセルフテストが使う厳密な合成閉ループ
    （真のKp=0.5、構成上ラグ0）で、taps=8 は h(0)=0.434（13%低）を復元した
    のに対し taps=1 は h(0)=0.4999...（0.03%低）だった。今日先に行った
    制御器同定の実験（fir_vs_pid.json /「実習7 同定ログ比較」アーティファクト
    §「PID基底 vs 数値的(FIR)推定」）では taps を40まで使い、本物の追加
    ダイナミクスをあぶり出した（そこでは有用 -- タップを増やすほどR^2が
    上がる＝PIDがその軸を過小評価している証拠）。しかしこの関数の仕事は
    ただ一つ -- _simulate_closed_loop() に渡す信頼できるスカラーKp -- で、
    taps を増やすとむしろ h(0) の推定精度が悪化するだけなので、呼び出し側に
    taps を広げる具体的な理由が無い限り taps=1 のままにすること。

    Args:
        e: target(t) - gyro(t) [rad/s], full-length (same length as u)
        u: the control effort to regress against e -- fit_plant()'s auto
            ladder passes actual_torque_diag [Nm] (the REAL applied torque
            from the vehicle physical-inversion, valid regardless of which
            mixer actually flew) so the returned kp_est comes out in true
            physical units, comparable directly to firmware/vehicle's
            rate.<axis>.kp -- NOT duty_diff, whose scale depends on the
            --mixer that was assumed
        throttle: flight-activity signal for _find_flight_segments()
        seg_samples: segment length in samples (same as fit_plant()'s
            segment_length * fs)
        taps: number of FIR lags (default 1 -- see docstring above for why)
        ridge_scale: L2 penalty as a fraction of the sample count (keeps
            the normal-equations solve well-conditioned when a segment has
            near-collinear lags, and guards taps=1 against a near-silent
            segment with tiny std(e))
    """
    segments = _find_flight_segments(throttle, seg_samples)
    X_rows: List[np.ndarray] = []
    y_rows: List[float] = []
    for seg_start, seg_end in segments:
        e_seg = e[seg_start:seg_end]
        u_seg = u[seg_start:seg_end]
        n_seg = len(e_seg)
        for k in range(taps - 1, n_seg):
            X_rows.append(e_seg[k - taps + 1:k + 1][::-1])
            y_rows.append(u_seg[k])

    if len(y_rows) < taps * 20:
        return None

    X = np.asarray(X_rows)
    y = np.asarray(y_rows)
    ridge = ridge_scale * len(y)
    try:
        h = np.linalg.solve(X.T @ X + ridge * np.eye(taps), X.T @ y)
    except np.linalg.LinAlgError:
        return None

    u_fit = X @ h
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    if ss_tot <= 1e-12:
        return None
    ss_res = np.sum((y - u_fit) ** 2)
    r_squared = 1.0 - ss_res / ss_tot

    return float(h[0]), float(r_squared), len(y)


def _load_axis_data(
    df: pd.DataFrame,
    axis: str,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
    mixer: str = 'legacy',
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float,
           Optional[np.ndarray], Optional[str], str,
           Optional[np.ndarray], Optional[np.ndarray], bool, Optional[float]]:
    """
    Extract axis-specific plant I/O from an aligned flight-log bundle
    DataFrame.
    整列済みフライトログ一式 DataFrame から軸固有のプラント入出力を抽出する。

    `df` is the output of tools/sysid/loader.py's load_aligned() (base=
    "imu") -- see that function's docstring for the full column contract
    and docs/plans/flight-log-format-plan.md section 2.2 for the bundle
    format itself. This replaces the retired dual CSV-format machinery
    (_detect_csv_format(), a self-contained csv.DictReader-based reader):
    the bundle format has only ONE schema, so there is nothing left to
    auto-detect here -- every column this function reads is either present
    at its one fixed name or genuinely absent (an optional source stream
    the bundle does not have).
    `df` は tools/sysid/loader.py の load_aligned()（base="imu"）の戻り値
    -- 列の全契約は同関数の docstring、一式形式そのものは
    docs/plans/flight-log-format-plan.md 2.2節参照。廃止した二重CSV形式判別
    機構（_detect_csv_format()、自己完結の csv.DictReader ベースリーダ）を
    置き換える: 一式形式はスキーマが1種類しかないため、ここで自動判別する
    ものはもう無い -- 本関数が読む列はどれも、決まった1つの名前で存在する
    か、正当に不在（一式が持たない任意ソースストリーム）かのどちらか。

    Args:
        df: aligned DataFrame from load_aligned(). Must have
            'timestamp_us' and the axis's 'rate_ref_<axis>'/'gyro_<x/y/z>'
            columns (imu + rate_ref streams). Optionally 'duty_FR'/
            'duty_RR'/'duty_RL'/'duty_FL' (genuine 400Hz motor duty when
            the bundle's `motor` stream is present, else the `ctrl_ref`
            stream's 50Hz-held fallback under the SAME bare names --
            distinguished via `df.attrs["bundle_streams"]`, NOT column
            presence alone -- see load_aligned()'s docstring), 'voltage'
            (from the bundle's `status` stream), and 'torque_roll'/
            'torque_pitch'/'torque_yaw' (from the bundle's `ctrl_output`
            stream -- ALWAYS native-rate when present, a lockstep stream
            merged on `seq`, never forward-filled, unlike the retired
            flat-CSV format's ctrl_output_rate_hz column, so there is no
            separate rate check to do here any more).
        axis: 'roll', 'pitch', or 'yaw'.
        fs: nominal sample rate [Hz], refined from `df['timestamp_us']`'s
            own median sample spacing when that looks sane (same
            robustness check the retired CSV reader had).
        time_range: optional (start, end) in seconds to restrict analysis.
        mixer: 'legacy' (default) or 'vehicle' -- selects which duty
            inversion produces duty_diff (see the module docstring's
            --mixer section).

    Returns:
        (time_s, target, gyro, throttle, dt, duty_diff, duty_quality,
        duty_reason, actual_torque_diag, ctrl_output_torque,
        ctrl_output_available, crash_truncated_at)

        target is `rate_ref_<axis>` -- ALREADY rad/s. Unlike the retired
        "stream"/"legacy" CSV split, there is only one schema now, so
        there is no rate_max scaling decision left for the caller to make
        here (fit_plant() still accepts a `rate_max` argument, but only to
        scale the excitation-floor threshold -- see its docstring).
        target は `rate_ref_<axis>` -- 既に rad/s。廃止された
        "stream"/"legacy" CSV の使い分けと異なりスキーマは1種類しかない
        ため、呼び出し側がここで rate_max のスケーリングを判断する必要は
        もう無い（fit_plant() は引き続き rate_max 引数を受け付けるが、
        励振下限しきい値のスケールにのみ使う -- 同関数の docstring 参照）。
        duty_diff is the mixer-inverted per-axis differential duty/torque
        (see _duty_differential_legacy_linear() /
        _duty_differential_vehicle(), selected by `mixer`) when
        duty_FR/RR/RL/FL columns are present, else None. duty_quality/
        duty_reason classify that duty as genuine 400Hz data ('duty400')
        or a 50Hz-held fallback ('duty50') -- see _classify_duty_source();
        duty_quality is None (with an explanatory duty_reason) when
        duty_diff is None. For mixer='vehicle' without a `voltage` column,
        duty_reason additionally notes the V_BATT_NOMINAL fallback.
        duty_diff は duty_FR/RR/RL/FL 列があればミキサ逆算した軸別差動
        duty/トルク（_duty_differential_legacy_linear() /
        _duty_differential_vehicle()。`mixer` で選択）、無ければ None。
        duty_quality/duty_reason はその duty が本物の400Hzデータ
        （'duty400'）か50Hz保持フォールバック（'duty50'）かを判別する
        （_classify_duty_source() 参照）。duty_diff が None のときは
        duty_quality も None（duty_reason に理由）。mixer='vehicle' で
        `voltage` 列が無い場合、duty_reason に V_BATT_NOMINAL フォール
        バックの旨も追記する。
        Also returns (actual_torque_diag, ctrl_output_torque,
        ctrl_output_available): actual_torque_diag is ALWAYS the
        vehicle-inversion "real torque" (see _duty_differential_vehicle())
        when genuine 400Hz duty is present, regardless of `mixer` -- a
        diagnostic signal, not the primary duty_diff. ctrl_output_torque
        is the axis's PRE-MIXER commanded torque from `torque_<axis>` when
        the bundle's `ctrl_output` stream carries non-degenerate values
        (ctrl_output_available True), else None/False.
        (actual_torque_diag, ctrl_output_torque, ctrl_output_available) も
        返す: actual_torque_diag は本物の400Hz dutyがあれば `mixer` に
        関わらず常に vehicle逆算「実トルク」（_duty_differential_vehicle()
        参照）-- 主経路の duty_diff ではなく診断用信号。ctrl_output_torque
        は一式の `ctrl_output` ストリームが非退化な値を持っていれば
        （ctrl_output_available=True）`torque_<axis>` から得た軸別の指令
        トルク（ミキサー手前）、無ければ None/False。
        Also returns crash_truncated_at: the flight-log time [s] at which
        automatic crash/anomaly truncation cut the data (see the "Automatic
        crash/anomaly truncation" comment above the return statement), or
        None when no truncation happened.
        crash_truncated_at も返す: 自動クラッシュ/異常検知トランケーションが
        データを切り捨てたフライトログ内の時刻[s]（return 文の上の
        コメント参照）、切り捨てが起きなければ None。
    """
    if axis not in _AXIS_NAMES:
        raise ValueError(f"Unknown axis: {axis}. Choose from: {list(_AXIS_NAMES)}")
    if mixer not in ('legacy', 'vehicle'):
        raise ValueError(f"Unknown mixer: {mixer!r}. Choose from: legacy, vehicle")
    if len(df) < 100:
        raise ValueError(f"Too few samples: {len(df)}")

    # --- Timestamp -> time_s, refining fs from the data when it looks sane
    # (same robustness check the retired CSV reader had -- a bundle's
    # declared/nominal rate can drift slightly from what the hardware
    # actually delivered).
    # タイムスタンプ -> time_s。データから妥当な範囲ならサンプルレートを
    # 補正する（廃止した CSV リーダーと同じ頑健性チェック -- 一式の公称
    # レートは実機が実際に出したレートと僅かにずれることがある）。
    ts_us = df['timestamp_us'].to_numpy(dtype=np.int64)
    time_s = (ts_us - ts_us[0]) / 1e6
    deltas = np.diff(time_s)
    deltas = deltas[deltas > 0]
    if len(deltas) > 10:
        detected_fs = 1.0 / float(np.median(deltas))
        if 0.5 * fs <= detected_fs <= 2.0 * fs:   # reject obviously-wrong units
            fs = detected_fs
    dt = 1.0 / fs

    # --- Target + gyro -- one schema now, no format branch ---
    # 目標値 + ジャイロ -- スキーマは1種類のみ、分岐なし
    target = df[_TARGET_COL[axis]].to_numpy(dtype=float)
    gyro = df[_GYRO_COL[axis]].to_numpy(dtype=float)

    # --- Throttle-equivalent for flight-segment detection ---
    # 飛行区間検出用のスロットル相当量
    if 'total_thrust' in df.columns:
        throttle = df['total_thrust'].to_numpy(dtype=float)
    elif all(c in df.columns for c in _DUTY_COLS):
        throttle = np.mean([df[c].to_numpy(dtype=float) for c in _DUTY_COLS], axis=0)
    else:
        raise ValueError(
            "flight-log bundle needs a flight-activity column to find "
            "flight segments: total_thrust (the bundle's ctrl_ref stream) "
            "or duty_FR/RR/RL/FL (the bundle's motor or ctrl_ref stream). "
            "Neither was found in the aligned DataFrame."
        )

    # --- Motor duty -> mixer-inverted differential duty (the "duty" input
    # mode's u_plant) when the 4 duty_FR/RR/RL/FL columns are present, plus
    # a classification of whether that duty is genuine 400Hz data or a
    # 50Hz-held fallback (see _classify_duty_source()) -- the 'auto' input
    # mode must NOT identify off the latter.
    # モータduty -> ミキサ逆算した差動duty（"duty" 入力モードの u_plant）。
    # duty_FR/RR/RL/FL の4列が揃っていれば計算し、あわせてそのduty が本物の
    # 400Hz データか50Hz保持フォールバックかを判別する
    # （_classify_duty_source() 参照）-- 'auto' 入力モードは後者で同定して
    # はならない。
    duty_diff: Optional[np.ndarray] = None
    duty_quality: Optional[str] = None
    duty_reason = "no duty_FR/RR/RL/FL columns in bundle"
    actual_torque_diag: Optional[np.ndarray] = None
    if all(c in df.columns for c in _DUTY_COLS):
        duty_fr = df['duty_FR'].to_numpy(dtype=float)
        duty_rr = df['duty_RR'].to_numpy(dtype=float)
        duty_rl = df['duty_RL'].to_numpy(dtype=float)
        duty_fl = df['duty_FL'].to_numpy(dtype=float)

        # voltage comes from the bundle's `status` stream, held/forward-
        # filled onto every row (see load_aligned()'s docstring); fall back
        # to the nominal 1S LiPo voltage when the `status` stream is absent
        # or a value is implausible, and say so in duty_reason. Computed
        # UNCONDITIONALLY (not just for mixer=='vehicle') because the
        # vehicle-inversion "actual torque" diagnostic below (§08 of the
        # rate-sysid design memo, 2026-09-09) needs it regardless of which
        # mixer produced the PRIMARY duty_diff -- comparing a candidate
        # u_plant against this actual torque is how the mixer's
        # gain/conversion factor gets measured straight from the log, for
        # ANY firmware's duty.
        # voltage は一式の `status` ストリーム由来、全行へ前方保持
        # （load_aligned() の docstring 参照）。`status` ストリームが無い、
        # または値が非現実的な場合は公称1S LiPo電圧にフォールバックし、
        # duty_reason にその旨を記す。mixer=='vehicle' のときだけでなく
        # 常に計算する -- 下の vehicle逆算「実トルク」診断（2026-09-09 レート
        # 同定設計メモ §08）は、どちらのミキサーが主経路の duty_diff を
        # 作ったかに関わらず必要になる。候補の u_plant をこの実トルクと
        # 比較することで、どのファームの duty からでもミキサーのゲイン/
        # 換算係数をログだけから測定できる。
        vbat_note = ''
        if _VOLTAGE_COL in df.columns:
            vbat = df[_VOLTAGE_COL].to_numpy(dtype=float)
            bad = vbat < _V_BATT_MIN
            if np.any(bad):
                vbat = np.where(bad, _V_BATT_NOMINAL, vbat)
                vbat_note = (
                    f" ({int(np.sum(bad))}/{len(vbat)} rows had no/implausible "
                    f"voltage -- used the nominal {_V_BATT_NOMINAL}V there)"
                )
        else:
            vbat = np.full(len(df), _V_BATT_NOMINAL)
            vbat_note = (
                f" (no voltage column in bundle -- used the nominal "
                f"{_V_BATT_NOMINAL}V throughout; capture with the bundle's "
                "status stream present for the real battery-sag-corrected "
                "fit)"
            )

        if mixer == 'vehicle':
            duty_diff = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat, axis)
        else:
            duty_diff = _duty_differential_legacy_linear(duty_fr, duty_rr, duty_rl, duty_fl, axis)

        # Diagnostic-only "actual torque" via the vehicle (real motor-curve)
        # inversion -- this is a statement about the REAL StampFly hardware
        # (motor curve + geometry), not about which firmware's mixer flew, so
        # it is always computed once real 400Hz duty is available. When
        # mixer=='vehicle' this duplicates duty_diff exactly (same call);
        # kept as its own array for a uniform diagnostic code path either way.
        # 診断専用の「実トルク」— vehicle（実モータ曲線）逆算。実StampFly
        # ハードウェア（モータ曲線＋ジオメトリ）についての事実であり、どの
        # ファームのミキサーが飛んだかとは無関係なので、本物の400Hz dutyが
        # あれば常に計算する。mixer=='vehicle' のときは duty_diff と全く
        # 同じ計算になる（同じ呼び出し）が、診断側のコード経路を統一するため
        # 別配列として持つ。
        actual_torque_diag = _duty_differential_vehicle(duty_fr, duty_rr, duty_rl, duty_fl, vbat, axis)

        # Whether duty_FR/RR/RL/FL came from the bundle's genuine 400Hz
        # `motor` stream, or a 50Hz-held `ctrl_ref` fallback, is now a
        # STRUCTURAL fact -- WHICH STREAM supplied the column -- rather than
        # something guessed from the data (see load_aligned()'s docstring).
        # Feed that fact into the UNCHANGED _classify_duty_source() (shared
        # verbatim with tools/log_analyzer/rate_sysid.py, same trick used
        # there) by synthesizing a duty_rate_hz array from it, so both
        # callers keep exercising the exact same, unmodified classification
        # function and its existing hz>=200.0 threshold.
        # duty_FR/RR/RL/FL が一式の本物の400Hz `motor` ストリーム由来か、
        # 50Hz保持の `ctrl_ref` フォールバック由来かは、今や構造的な事実
        # （どのストリームがその列を供給したか）であり、データから推測する
        # ものではない（load_aligned() の docstring 参照）。その事実から
        # duty_rate_hz 配列を合成し、変更していない _classify_duty_source()
        # （tools/log_analyzer/rate_sysid.py と全く同じものを共有、同じ手法
        # をあちらでも使用）へ渡すことで、両呼び出し元が同じ未変更の判別
        # 関数とその既存の hz>=200.0 しきい値を使い続けられるようにする。
        motor_present = "motor" in df.attrs.get("bundle_streams", set())
        duty_rate_hz_arr = np.full(len(duty_fr), 400.0 if motor_present else 50.0)
        duty_quality, duty_reason = _classify_duty_source(
            duty_fr, duty_rr, duty_rl, duty_fl, duty_rate_hz_arr,
        )
        duty_reason += vbat_note
        if duty_quality != 'duty400':
            # The vehicle motor-curve inversion needs genuine 400Hz duty --
            # a 50Hz-held fallback is too coarse for either the primary
            # vehicle fit or the diagnostic (same reasoning as
            # fit_plant()'s --input duty rejection of duty50).
            # vehicle のモータ曲線逆算には本物の400Hz duty が要る -- 50Hz
            # 保持フォールバックは主経路のvehicleフィットにも診断にも粗
            # すぎる（fit_plant() の --input duty が duty50 を拒否するのと
            # 同じ理由）。
            actual_torque_diag = None

    # --- control_output: PRE-MIXER commanded thrust+torque, when the
    # bundle's `ctrl_output` stream is present -- see the rate-sysid design
    # memo (docs/events/sci_tutorial_2026, 2026-09-09). Mixer-agnostic
    # plant input: reading this needs no --mixer selection and no nonlinear
    # duty->thrust inversion at all.
    # control_output: ミキサー手前の指令推力+トルク、一式の `ctrl_output`
    # ストリームがあれば -- レート同定設計メモ（docs/events/
    # sci_tutorial_2026、2026-09-09）参照。ミキサー非依存のプラント入力:
    # 読むのに --mixer の選択も非線形な duty->thrust逆算も一切要らない。
    # Unlike the retired flat-CSV format, this stream is ALWAYS native-rate
    # (400Hz) when present -- it is a lockstep stream merged on `seq`,
    # never forward-filled (see load_aligned()'s docstring) -- so there is
    # no separate rate check to do any more. What remains (2026-09-10,
    # urgent fix, still relevant): firmware/workshop's WorkshopControlTask
    # never actually WRITES torque[] (see workshop_control_task.cpp -- it
    # fills only .thrust, from its own duty-scale MotorRequest.thrust, not
    # physical N), so a captured bundle's ctrl_output.csv can carry a
    # `torque_<axis>` column that is present but constant zero. A naive
    # presence-only check would then make 'auto' PREFER an all-zero,
    # meaningless control_output torque over the (working) duty-based
    # reconstruction for every workshop log -- a regression, not an
    # improvement. So: also require non-degenerate (non-constant-zero)
    # values before trusting this column.
    # 廃止した平坦CSV形式と異なり、このストリームは存在すれば常にネイティブ
    # レート（400Hz）-- `seq` で結合するロックステップ系ストリームであり
    # 前方補完されない（load_aligned() の docstring 参照）-- ので別途レート
    # チェックはもう不要。残る論点（2026-09-10、緊急修正、今も有効）:
    # firmware/workshop の WorkshopControlTask は torque[] を実際には一切
    # 書かない（workshop_control_task.cpp 参照 -- 埋めるのは .thrust だけで、
    # しかも物理量Nではなく自前のduty尺度のMotorRequest.thrust）ため、取得
    # した一式の ctrl_output.csv は `torque_<axis>` 列が存在しつつ定数ゼロ
    # ということがあり得る。存在チェックだけだと、'auto' が全ての workshop
    # ログで意味の無い全ゼロトルクを、動作する duty 逆算より優先してしまう
    # -- 改善ではなく退行になる。そこで値が非退化（定数ゼロでない）ことも
    # 合わせて要求する。
    torque_col = f'torque_{axis}'
    ctrl_output_torque: Optional[np.ndarray] = None
    ctrl_output_available = False
    if torque_col in df.columns:
        candidate = df[torque_col].to_numpy(dtype=float)
        # Constant-zero (or near enough to be numerically indistinguishable
        # from an unpopulated field) => not genuinely populated.
        # 定数ゼロ（または未使用フィールドと数値的に見分けが付かない
        # ほど小さい）なら、実際には値が入っていないとみなす。
        if np.std(candidate) > 1e-9:
            ctrl_output_available = True
            ctrl_output_torque = candidate

    # Automatic crash/anomaly truncation (2026-09-10, real lesson_07 test
    # flights): a violent tumble/impact spikes |gyro| far beyond anything a
    # controlled flight -- even an aggressive P-control transient -- ever
    # produces, but throttle typically stays > 0.3 through it (motors still
    # spinning), so _find_flight_segments()'s throttle-only criterion does
    # NOT exclude it. Left alone, a crash tail silently corrupts whichever
    # segment(s) straddle it (this is exactly what was previously found and
    # worked around by hand: test7_2.csv, manually restricting to t<21.5s --
    # see "実習7 同定ログ比較" artifact). Automate that: truncate everything
    # from the FIRST implausible sample onward. _GYRO_CRASH_MAX_RAD_S=10 is
    # deliberately generous (BMI270 noise floor here is ~0.003 rad/s, so
    # this is >3000-sigma -- zero risk of tripping on sensor noise) and
    # empirically robust: on the real crash this was calibrated against, the
    # first exceedance lands at the SAME instant (t=22.69s, all 3 axes) for
    # any threshold from 3 to 12 rad/s, so the exact cutoff value is not
    # sensitive within that whole range.
    # 自動クラッシュ/異常検知トランケーション（2026-09-10、実際の実習7
    # テスト飛行）: 激しいタンブル/衝突は |gyro| を制御された飛行（積極的な
    # P制御の過渡応答すら含め）では絶対に出ない大きさまで跳ね上げるが、
    # throttle は大抵 0.3 を超えたままなので（モータは回り続ける）、
    # _find_flight_segments() のスロットルのみの判定では除外されない。
    # 放置すると、クラッシュ区間にまたがるセグメントを静かに汚染する（まさに
    # 以前手作業で見つけて回避した現象 -- test7_2.csv を t<21.5s に手動制限、
    # 「実習7 同定ログ比較」アーティファクト参照）。それを自動化する: 最初に
    # あり得ない値が出たサンプル以降を全て切り捨てる。_GYRO_CRASH_MAX_RAD_S
    # =10 は意図的に余裕を持たせてある（ここでのBMI270ノイズ床は
    # 約0.003rad/sなので、これは3000シグマ超 -- センサノイズで誤発火する
    # 心配は皆無）上、実証的にも頑健（この較正に使った実際のクラッシュでは、
    # 3〜12 rad/s のどの閾値でも最初の超過は全3軸とも同じ瞬間 t=22.69s に
    # 発生し、この範囲内なら閾値の正確な値に結果は左右されない）。
    crash_truncated_at: Optional[float] = None
    anomaly = np.abs(gyro) > _GYRO_CRASH_MAX_RAD_S
    if np.any(anomaly):
        crash_idx = int(np.argmax(anomaly))
        crash_truncated_at = float(time_s[crash_idx])
        time_s = time_s[:crash_idx]
        target = target[:crash_idx]
        gyro = gyro[:crash_idx]
        throttle = throttle[:crash_idx]
        if duty_diff is not None:
            duty_diff = duty_diff[:crash_idx]
        if actual_torque_diag is not None:
            actual_torque_diag = actual_torque_diag[:crash_idx]
        if ctrl_output_torque is not None:
            ctrl_output_torque = ctrl_output_torque[:crash_idx]

    # Apply time range filter
    # 時間範囲フィルタを適用
    if time_range is not None:
        t_start, t_end = time_range
        mask = (time_s >= t_start) & (time_s <= t_end)
        time_s = time_s[mask]
        target = target[mask]
        gyro = gyro[mask]
        throttle = throttle[mask]
        if duty_diff is not None:
            duty_diff = duty_diff[mask]
        if actual_torque_diag is not None:
            actual_torque_diag = actual_torque_diag[mask]
        if ctrl_output_torque is not None:
            ctrl_output_torque = ctrl_output_torque[mask]

    return (time_s, target, gyro, throttle, dt, duty_diff, duty_quality,
            duty_reason, actual_torque_diag, ctrl_output_torque, ctrl_output_available,
            crash_truncated_at)


def _find_flight_segments(
    throttle: np.ndarray,
    seg_samples: int,
    throttle_threshold: float = 0.3,
) -> List[Tuple[int, int]]:
    """
    Find flight segments where throttle > threshold
    スロットルが閾値以上の飛行区間を検出

    Returns:
        List of (start_idx, end_idx) tuples for analysis segments
    """
    in_flight = throttle > throttle_threshold

    # Find contiguous flight regions
    # 連続的な飛行区間を検出
    flight_starts = []
    flight_ends = []
    in_region = False
    for i in range(len(in_flight)):
        if in_flight[i] and not in_region:
            flight_starts.append(i)
            in_region = True
        elif not in_flight[i] and in_region:
            flight_ends.append(i)
            in_region = False
    if in_region:
        flight_ends.append(len(in_flight))

    # Split flight regions into analysis segments
    # 飛行区間を分析セグメントに分割
    min_seg = seg_samples // 2
    segments = []
    for start, end in zip(flight_starts, flight_ends):
        if end - start < min_seg:
            continue
        for seg_start in range(start, end - min_seg, seg_samples):
            seg_end = min(seg_start + seg_samples, end)
            if seg_end - seg_start >= min_seg:
                segments.append((seg_start, seg_end))

    return segments


def fit_plant(
    df: pd.DataFrame,
    axis: str = 'roll',
    kp: Optional[float] = None,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
    segment_length: float = 3.0,
    min_activity: float = 0.01,
    min_target_std_frac: float = 0.1,
    input_mode: str = 'auto',
    mixer: str = 'legacy',
) -> PlantFitResult:
    """
    Fit open-loop plant model from closed-loop flight data
    閉ループフライトデータから開ループプラントモデルを同定

    Args:
        df: Aligned flight-log DataFrame from
            tools/sysid/loader.py's load_aligned() (base="imu") -- see that
            function's docstring for the full column contract.
        axis: 'roll', 'pitch', or 'yaw'
        kp: P gain used during flight (must match firmware value). Required
            when input_mode resolves to 'kp'; ignored (may be left None) in
            'duty' mode.
        rate_max: Maximum angular rate [rad/s]. The bundle format's
            `rate_ref_<axis>` is already physical rad/s (there is no
            "legacy" normalized-stick schema left to scale, unlike before
            this format unification), so rate_max no longer scales
            target(t) -- it is used ONLY to scale min_target_std_frac's
            excitation-floor threshold below.
        fs: Sample rate [Hz] (default: 400)
        time_range: Optional (start, end) in seconds to restrict analysis
        segment_length: Segment duration [s] for fitting (default: 3.0)
        min_activity: Minimum std of plant input to include segment
        min_target_std_frac: A segment is also dropped when std(target(t))
            over that segment is below this fraction of rate_max -- a
            near-constant reference makes u=Kp*(target-y) collapse onto the
            feedback term alone, which correlates strongly (and misleadingly)
            NEGATIVELY with y regardless of the true plant sign (closed-loop
            identifiability pitfall, not a hardware/sign bug -- see
            target_excitation_note on the result). Set to 0 to disable.
        input_mode: 'auto' (default), 'duty', or 'kp' -- see the module
            docstring. 'auto' picks 'duty' only when the bundle DataFrame
            has duty_FR/RR/RL/FL columns, kp is None, AND
            _classify_duty_source() says the duty is genuine 400Hz data
            (not a 50Hz-held fallback); otherwise 'kp'.
        mixer: 'legacy' (default) or 'vehicle' -- which duty inversion the
            'duty'/'auto' input modes use (see the module docstring's
            --mixer section). 'legacy' is correct for firmware/workshop
            (`sf lesson`) and firmware/vehicle_old logs; 'vehicle' is
            required for firmware/vehicle (`sf app`) logs. The bundle
            cannot say which firmware produced it -- the caller must know.
            Ignored when input_mode resolves to 'kp'.

    Returns:
        PlantFitResult with identified K, tau_m and fit metrics

    Raises:
        ValueError: If data is insufficient, fitting fails, or the resolved
            input mode's required data/argument is missing.
    """
    if mixer not in ('legacy', 'vehicle'):
        raise ValueError(f"Unknown mixer: {mixer!r}. Choose from: legacy, vehicle")

    # Load data
    # データ読み込み
    (time_s, target_raw, gyro, throttle, dt, duty_diff, duty_quality, duty_reason,
     actual_torque_diag, ctrl_output_torque, ctrl_output_available, crash_truncated_at) = (
        _load_axis_data(df, axis, fs, time_range, mixer=mixer)
    )

    # Resolve the input mode -- see the module docstring. 2026-09-10 rewrite
    # (this afternoon's tutorial deadline): the OLD ladder preferred any
    # DIRECT fit (control_output/duty, u -> y) over 'indirect' whenever --kp
    # was not typed by hand. We now know that preference was backwards --
    # ANY direct fit is closed-loop-biased regardless of whether u is
    # independently measured (duty/control_output) or reconstructed from a
    # known Kp (see _simulate_closed_loop()'s docstring and the real
    # lesson_07 comparison: direct gave R^2<0, indirect gave R^2 0.55-0.99
    # on the SAME logs). So 'indirect' should ALWAYS win once a Kp is
    # available -- and to make that not require the operator to type --kp,
    # we first try to AUTO-ESTIMATE it from this log's own duty via
    # _estimate_kp_fir() (short ridge-regularized FIR regression of u
    # against lags of e=target-gyro, no PID-structure assumption, h(0) is
    # the effective proportional gain). Only when no Kp is available at all
    # (given nor auto-estimated) does the ladder fall back to a direct fit,
    # and 'duty'/'control_output' still require the SAME
    # duty_quality=='duty50' guard as before (never silently fit a stale
    # 50Hz-forward-filled staircase).
    #
    # 2026-09-10, second/third pass (prompted by a sharp "shouldn't mixer
    # gain and plant gain be separable?" question, then a real-data catch by
    # peer session stampfly-ecosystem-40): the FIR auto-Kp estimate's `u`
    # follows --mixer, exactly like 'duty'/'control_output' already do --
    # mixer=='vehicle' uses actual_torque_diag (the REAL applied torque
    # [Nm] from the vehicle physical B^-1+motor-curve inversion -- a fact
    # about the fixed StampFly HARDWARE, true regardless of which mixer the
    # firmware that flew actually used), giving Kp/K in TRUE PHYSICAL units
    # comparable directly to firmware/vehicle's rate.roll/pitch/yaw.kp.
    # mixer=='legacy' (the default) uses duty_diff, same as before, giving
    # Kp/K in the legacy duty-differential scale. This second option is NOT
    # a compromise -- today's workshop curriculum (lesson_06/07, slides S4)
    # is built entirely around that legacy-scale K (REFERENCE_PLANT_GAINS,
    # derived without ever teaching moment of inertia); unconditionally
    # switching to vehicle/Nm scale (~1e5 instead of ~100) would make every
    # participant's auto-estimated result look nothing like the lesson
    # material's expected numbers. So: --mixer is the single, explicit
    # switch for BOTH which duty inversion AND which torque signal the
    # auto-Kp estimate uses -- the caller must still know which firmware
    # produced the log, same as it always has for 'duty'/'control_output'.
    # 入力モードを解決する — モジュール docstring 参照。2026-09-10 改訂
    # （本日のチュートリアル締切対応）: 旧ラダーは --kp を手入力しない限り
    # 直接法（control_output/duty、u -> y）を 'indirect' より優先していた
    # が、この優先順は誤りだったと今日わかった -- u が独立測定（duty/
    # control_output）か既知Kpからの再構成かに関わらず、直接法は常に閉ループ
    # バイアスを受ける（_simulate_closed_loop() の docstring、および実際の
    # 実習7比較: 同じログで直接法は R^2<0、間接法は R^2 0.55〜0.99 参照）。
    # よって Kp さえ手に入れば 'indirect' が常に勝つべきであり、そのために
    # オペレータに --kp を手入力させないよう、まずこのログ自身の duty から
    # _estimate_kp_fir()（e=target-gyro の複数ラグへの短いリッジ正則化FIR
    # 回帰、PID構造の仮定なし、h(0)が実効比例ゲイン）で自動推定を試みる。
    # Kp が（手入力・自動推定とも）一切得られない場合のみ、直接法へ縮退する
    # -- 'duty'/'control_output' は従来通り duty_quality=='duty50' ガード
    # （50Hz前方補完の階段状データを黙ってフィットしない）を維持する。
    #
    # 2026-09-10 第2/3弾（「ミキサーゲインとプラントゲインを分けられるはずで
    # は」という鋭い指摘、その後 peer session の stampfly-ecosystem-40 が
    # 実データ再検証で発見）: FIR自動Kp推定の `u` は、'duty'/'control_output'
    # が既にそうしているのと全く同じく --mixer に従う -- mixer=='vehicle'
    # では actual_torque_diag（vehicle の物理的なB^-1＋モータ曲線逆算による
    # 実際に加わったトルク[Nm]。実際に飛んだファームのミキサーが何であったか
    # とは無関係な、固定されたStampFlyハードウェアについての事実）を使い、
    # Kp/K は firmware/vehicle の rate.roll/pitch/yaw.kp と直接比較可能な
    # 真の物理単位で得られる。mixer=='legacy'（既定）では従来通り duty_diff
    # を使い、Kp/K は legacy の duty差動スケールで得られる。この後者は妥協
    # ではない -- 今日の実習カリキュラム（lesson_06/07、スライドS4）は、
    # 慣性モーメントを一切教えずに導出するその legacy スケールK
    # （REFERENCE_PLANT_GAINS）を前提に完全に組み立てられている。vehicle/Nm
    # スケール（〜100ではなく〜1e5）へ無条件で切り替えてしまうと、全受講者の
    # 自動推定結果が実習資料の期待値と全く違って見えてしまう。したがって
    # --mixer は、duty逆算方式**と**自動Kp推定が使うトルク信号の**両方**を
    # 決める単一の明示的なスイッチである -- 呼び出し側がどのファームで
    # 録ったログかを知っている必要があるのは、'duty'/'control_output' の
    # ときと変わらない。
    if input_mode not in ('auto', 'control_output', 'duty', 'indirect', 'kp'):
        raise ValueError(
            f"Unknown input_mode: {input_mode!r}. Choose from: auto, "
            "control_output, duty, indirect, kp"
        )
    seg_samples = int(segment_length * (1.0 / dt))
    kp_source: Optional[str] = 'user' if kp is not None else None
    kp_auto_r_squared: Optional[float] = None
    kp_auto_vehicle_scale = False
    resolved_mode = input_mode
    if resolved_mode == 'auto':
        if kp is not None:
            resolved_mode = 'indirect'
        else:
            # 2026-09-10, THIRD pass (peer session stampfly-ecosystem-40
            # caught this re-testing on real data): which signal feeds
            # _estimate_kp_fir() must follow the CALLER's --mixer, exactly
            # like 'duty'/'control_output' already do -- NOT switch to
            # vehicle/Nm scale unconditionally just because
            # actual_torque_diag happens to be available. Today's workshop
            # curriculum (lesson_06/07, slides S4) is built entirely around
            # the LEGACY duty-differential-scale K (REFERENCE_PLANT_GAINS,
            # ~100, derived without ever teaching moment of inertia) --
            # unconditionally reporting the (more "physically correct")
            # vehicle/Nm-scale K (~1e5) instead would make every
            # participant's result look nothing like what the lesson
            # material tells them to expect, on the one day it matters.
            # mixer=='vehicle' (opt-in, same as --input duty already
            # requires for that scale) is the right and only trigger for
            # the actual_torque_diag-based estimate.
            # 2026-09-10、3回目の修正（peer session の
            # stampfly-ecosystem-40 が実データでの再検証中に発見）:
            # _estimate_kp_fir() に渡す信号は、'duty'/'control_output' が
            # 既にそうしているのと全く同じく、呼び出し側の --mixer に従う
            # べきで、actual_torque_diag がたまたま使えるからといって
            # vehicle/Nm尺度に無条件で切り替えてはならない。今日の実習
            # カリキュラム（lesson_06/07、スライドS4）は、慣性モーメントを
            # 一切教えずに導出する legacy の duty差動スケールK
            # （REFERENCE_PLANT_GAINS、〜100）を前提に完全に組み立てられて
            # いる -- （より「物理的に正しい」）vehicle/Nm スケールK
            # （〜1e5）を無条件で返してしまうと、まさに今日、全受講者の
            # 結果が実習資料の期待値と全く違って見えてしまう。
            # mixer=='vehicle'（オプトイン、--input duty が同じスケールに
            # 既に要求しているのと同じ）が、actual_torque_diag ベースの
            # 推定への唯一正しいトリガーである。
            kp_auto: Optional[float] = None
            u_for_kp = actual_torque_diag if mixer == 'vehicle' else duty_diff
            u_for_kp_is_vehicle_scale = mixer == 'vehicle'
            if u_for_kp is not None and duty_quality == 'duty400':
                # target_raw is already rad/s (bundle format's one schema
                # -- no rate_max scaling needed any more, see fit_plant()'s
                # docstring).
                # target_raw は既に rad/s（一式形式のスキーマは1種類のみ --
                # rate_max のスケーリングはもう不要。fit_plant() の
                # docstring 参照）。
                target_for_kp = target_raw
                est = _estimate_kp_fir(target_for_kp - gyro, u_for_kp, throttle, seg_samples)
                if est is not None:
                    kp_est, kp_r2, _n_fir = est
                    if kp_est > 0.0 and kp_r2 > _KP_AUTO_R2_FLOOR:
                        kp_auto, kp_auto_r_squared = kp_est, kp_r2
            if kp_auto is not None:
                kp = kp_auto
                kp_source = 'fir_auto'
                kp_auto_vehicle_scale = u_for_kp_is_vehicle_scale
                resolved_mode = 'indirect'
            elif ctrl_output_available:
                resolved_mode = 'control_output'
            elif duty_diff is not None and duty_quality == 'duty400':
                resolved_mode = 'duty'
            else:
                resolved_mode = 'kp'

    # Mixer-gain diagnostic (rate-sysid design memo §07/§08): whenever the
    # log has genuine 400Hz duty (actual_torque_diag is not None), compare
    # the RESOLVED mode's candidate u_plant against the duty-derived actual
    # torque. Computed BEFORE segment filtering below (on the full series --
    # _mixer_conversion_factor() itself is robust to a few quiet stretches
    # via the least-squares fit); meaningless in 'kp' mode (that
    # reconstruction models something else entirely) so skipped there.
    # ミキサーゲイン診断（レート同定設計メモ §07/§08）: 本物の400Hz duty が
    # あれば（actual_torque_diag が None でなければ）常に、解決済みモードの
    # 候補 u_plant を duty 逆算の実トルクと突き合わせる。下のセグメント
    # フィルタより前に計算する（全系列に対して — _mixer_conversion_factor()
    # 自体が最小二乗フィットで多少の無音区間には頑健）。'kp' モードでは
    # 無意味（あの再構成は全く別のものをモデル化している）なのでスキップ。
    mixer_gain: Optional[float] = None
    mixer_gain_r2: Optional[float] = None
    mixer_gain_label = ''
    if actual_torque_diag is not None:
        if resolved_mode == 'control_output' and ctrl_output_torque is not None:
            diag = _mixer_conversion_factor(ctrl_output_torque, actual_torque_diag)
            if diag is not None:
                mixer_gain, mixer_gain_r2 = diag
                mixer_gain_label = ('mixer static gain c (commanded '
                                     'control_output vs duty-derived actual '
                                     'torque, dimensionless, ~1 for an '
                                     'accurate mixer)')
        elif resolved_mode == 'duty' and mixer == 'legacy':
            diag = _mixer_conversion_factor(duty_diff, actual_torque_diag)
            if diag is not None:
                mixer_gain, mixer_gain_r2 = diag
                mixer_gain_label = ('legacy duty-unit -> real torque '
                                     '[Nm per legacy duty-differential unit]')
        elif resolved_mode == 'duty' and mixer == 'vehicle' and ctrl_output_torque is not None:
            # duty_diff already equals actual_torque_diag here (same
            # computation) -- the meaningful comparison is against
            # control_output when it is ALSO present.
            diag = _mixer_conversion_factor(ctrl_output_torque, actual_torque_diag)
            if diag is not None:
                mixer_gain, mixer_gain_r2 = diag
                mixer_gain_label = ('mixer static gain c (commanded '
                                     'control_output vs duty-derived actual '
                                     'torque, dimensionless, ~1 for an '
                                     'accurate mixer)')

    if resolved_mode == 'control_output':
        if ctrl_output_torque is None:
            raise ValueError(
                "--input control_output requested but this bundle DataFrame "
                "has no non-degenerate torque_<axis> column -- needs a "
                "bundle whose `ctrl_output` stream is present AND actually "
                "populated (firmware sending the kPktCtrlOutput400 entry / "
                "0x4B with real torque values, not firmware/workshop's "
                "always-zero torque[]). Pass --input duty or --input kp "
                "instead."
            )
        # u_plant(t) = control_output.torque(t) -- the PRE-MIXER commanded
        # torque, already in the SAME physical units (Nm) regardless of
        # which mixer (legacy linear, vehicle B^-1+motor-curve, or a
        # learner's own) turned it into motor duty. No --mixer needed.
        # u_plant(t) = control_output.torque(t) -- ミキサー手前の指令トルク。
        # どのミキサー（legacy線形、vehicle B^-1+モータ曲線、学習者自作）が
        # duty に変換したかに関わらず、既に同じ物理単位[Nm]。--mixer 不要。
        u_plant = ctrl_output_torque
        kp_used: Optional[float] = None
    elif resolved_mode == 'duty':
        if duty_diff is None:
            raise ValueError(
                "--input duty requested but this bundle DataFrame has no "
                "duty_FR/RR/RL/FL columns -- needs a bundle with the "
                "`motor` stream (genuine 400Hz duty) or the `ctrl_ref` "
                "stream's 50Hz-held fallback. Pass --kp to use the legacy "
                "Kp*(target-gyro) reconstruction instead."
            )
        if duty_quality != 'duty400':
            raise ValueError(
                f"--input duty requested but this log is from OLD firmware "
                f"(duty_FR/RR/RL/FL is 50Hz-held here, not the bundle's "
                f"genuine 400Hz `motor` stream): {duty_reason}. Pass --kp "
                "instead (--input kp) -- duty here is too coarse to "
                "identify a ~20ms motor lag."
            )
        # u_plant(t) = mixer-inverse(duty_FR/RR/RL/FL)(t) -- the actual
        # differential command the rate-loop PID output this cycle, using
        # WHICHEVER mixer inversion `mixer` selected (legacy duty-diff vs
        # vehicle torque -- see the module docstring's --mixer section). If
        # the fit below fails or gives a physically-implausible K/tau_m,
        # the most likely cause is the WRONG --mixer for this log's firmware.
        # u_plant(t) = ミキサ逆算(duty_FR/RR/RL/FL)(t) -- レートループ
        # PID がその周期に実際に出力した差動指令。`mixer` が選んだ方の逆算
        # （legacy の duty差動 vs vehicle のトルク -- モジュール docstring の
        # --mixer 節参照）を使う。下のフィットが失敗する、または物理的に
        # ありえない K/tau_m になる場合、最も疑わしい原因はこのログの
        # ファームに対して --mixer が間違っていること。
        u_plant = duty_diff
        kp_used: Optional[float] = None
    elif resolved_mode == 'indirect':
        if kp is None:
            raise ValueError(
                "--input indirect requires --kp -- the P gain that flew. "
                "This mode fits the closed-loop target->gyro transfer "
                "function directly and uses the known Kp to back out K, "
                "tau_m algebraically (see _simulate_closed_loop() docstring "
                "for why this is far more robust than --input kp on real, "
                "human-piloted flight data)."
            )
        # Indirect mode's u_plant = kp*(target-gyro) is only a proxy for the
        # control-ACTIVITY filter a few lines below (kept so that filter
        # needs no special-casing) -- the actual fit below uses
        # target_physical and y_plant directly, never this u_plant.
        # 間接モードの u_plant = kp*(target-gyro) は、この少し下の制御活動量
        # フィルタ用の代用に過ぎない（このフィルタが特別扱い不要で済むよう
        # にするため）-- 実際のフィットは下で target_physical と y_plant を
        # 直接使い、この u_plant は一切使わない。
        target = target_raw
        u_plant = kp * (target - gyro)
        kp_used = kp
    else:  # 'kp'
        if kp is None:
            if duty_quality == 'duty50':
                raise ValueError(
                    f"this log is from OLD firmware: {duty_reason}. "
                    "duty_FR/RR/RL/FL is only 50Hz-resolution here (not "
                    "the real 400Hz `motor` stream), so --kp is required "
                    "for a reliable fit (--input kp)."
                )
            raise ValueError(
                "--input kp (explicit, or auto without duty_FR/RR/RL/FL "
                "columns) requires --kp -- the P gain that flew. Capture a "
                "log with `sf log wifi` on firmware sending the 400Hz duty "
                "entry to use --input duty instead (no --kp needed)."
            )
        # Reconstruct plant I/O. The bundle's rate_ref_<axis> already
        # records the physical rate target [rad/s] (vehicle rate_ref), so
        # rate_max is NOT applied (see fit_plant()'s docstring -- there is
        # no remaining "legacy" normalized-stick schema to scale).
        # プラント入出力を復元。一式の rate_ref_<axis> は既に物理量の角速度
        # 目標 [rad/s]（vehicle の rate_ref）を記録しているため rate_max は
        # 適用しない（fit_plant() の docstring 参照 -- スケールすべき
        # "legacy" 正規化スティックスキーマはもう残っていない）。
        #   u_plant(t) = Kp * (target(t) - gyro(t))
        target = target_raw
        u_plant = kp * (target - gyro)
        kp_used = kp

    y_plant = gyro

    # Physical-units reference signal, computed regardless of resolved_mode
    # (needed for the excitation check below even in 'duty'/'control_output'
    # mode, where target never otherwise gets scaled). target_raw is
    # already rad/s (bundle format's one schema) -- see fit_plant()'s
    # docstring.
    # resolved_mode に関わらず計算する物理量の参照信号（下の励振チェック用。
    # 'duty'/'control_output' モードでは他に target をスケールする箇所が
    # ないため、ここで用意する）。target_raw は既に rad/s（一式形式の
    # スキーマは1種類のみ）-- fit_plant() の docstring 参照。
    target_physical = target_raw
    min_target_std = min_target_std_frac * rate_max

    # Find flight segments
    # 飛行区間の検出 (seg_samples computed earlier, alongside mode resolution)
    segments = _find_flight_segments(throttle, seg_samples)

    if not segments:
        raise ValueError(
            "No valid flight segments found. "
            "Check that throttle > 0.3 during flight."
        )

    # K's scale/units -- and everything calibrated against it (the optimizer
    # bounds below, AND the min_activity floor just below) -- depend on
    # whether u_plant is in physical torque [Nm] (torque-input K is ~1000x
    # the duty-differential-input K -- see REFERENCE_PLANT_GAINS_VEHICLE's
    # docstring above). That's true for resolved_mode == 'control_output'
    # (control_output.torque is always Nm) and for resolved_mode == 'duty'
    # with mixer == 'vehicle'; 'kp' mode always uses the duty-differential
    # scale (u_plant there is directly comparable to the legacy mixer's
    # u_plant by construction).
    # K の尺度・単位 -- それに較正された最適化境界（下）と min_activity 閾値
    # （すぐ下）も -- は、u_plant が物理トルク[Nm]かどうかに依存する
    # （トルク入力の K は duty差動入力の K よりおよそ3桁大きい -- 上の
    # REFERENCE_PLANT_GAINS_VEHICLE のdocstring参照）。これは
    # resolved_mode=='control_output'（control_output.torque は常にNm）と、
    # resolved_mode=='duty' かつ mixer=='vehicle' の場合に成り立つ。'kp'
    # モードは常に duty差動スケール（そちらの u_plant は構成上 legacy
    # ミキサーの u_plant と直接比較可能）。
    # 'indirect' via FIR auto-Kp is ALSO vehicle/torque-scale now (kp_auto_
    # vehicle_scale, set above): the auto-estimate's u was actual_torque_diag
    # [Nm], not the legacy duty-differential scale an explicit --kp implies.
    # 'indirect' が FIR自動Kp経由の場合も（kp_auto_vehicle_scale）vehicle/
    # トルク尺度になる: 自動推定の u は actual_torque_diag[Nm] であり、
    # 明示的な --kp が前提とする legacy の duty差動スケールではないため。
    use_vehicle_scale = (resolved_mode == 'control_output'
                          or (resolved_mode == 'duty' and mixer == 'vehicle')
                          or (resolved_mode == 'indirect' and kp_auto_vehicle_scale))
    ref_gains = REFERENCE_PLANT_GAINS_VEHICLE if use_vehicle_scale else REFERENCE_PLANT_GAINS
    ref_K = ref_gains.get(axis, 100.0)
    # Vehicle-scale K sits ~1e5 (1/I_axis); bound wide around it rather than
    # the legacy (1.0, 1000.0) default, which would clip the true optimum.
    # ヴィークル尺度の K はおよそ1e5（1/I_axis）に位置する。legacy の既定
    # (1.0, 1000.0) では真の最適値を境界で切り詰めてしまうため、その周囲を
    # 広く取る。
    K_bounds = (1.0, 1.0e9) if use_vehicle_scale else (1.0, 1000.0)
    # min_activity is a floor on std(u_plant); the caller's default (0.01) is
    # calibrated for the legacy duty-differential scale. Roughly rescale it
    # for the vehicle torque scale by the same K ratio used for K_bounds
    # above (equal plant RESPONSE needs roughly K_legacy/K_vehicle times less
    # torque than duty-differential input) -- this only needs to reject
    # near-silent segments, not be exact.
    # min_activity は std(u_plant) の下限。呼び出し側の既定値（0.01）は
    # legacy の duty差動スケール用に較正されている。上の K_bounds と同じ K比
    # でヴィークルのトルクスケール向けに概算し直す（同じプラント応答を得る
    # のに必要なトルクは duty差動入力よりおおよそ K_legacy/K_vehicle 倍小さい
    # ）-- ほぼ無音のセグメントを排除できれば十分で、厳密さは不要。
    min_activity_effective = min_activity
    if use_vehicle_scale:
        legacy_ref_K = REFERENCE_PLANT_GAINS.get(axis, 100.0)
        min_activity_effective = min_activity * (legacy_ref_K / ref_K)

    # Filter segments by control activity level
    # 制御入力が十分な区間のみ抽出
    active_segments = []
    for seg_start, seg_end in segments:
        if np.std(u_plant[seg_start:seg_end]) > min_activity_effective:
            active_segments.append((seg_start, seg_end))

    if not active_segments:
        raise ValueError(
            f"No segments with sufficient control activity "
            f"(std > {min_activity_effective:.3g}). "
            "Ensure the pilot made stick inputs during flight."
        )

    # Drop segments where the REFERENCE (target/rate_ref) barely moves, even
    # if u_plant itself passed the activity filter above. In closed-loop
    # P-control, u = Kp*(target - y): when target is near-constant over the
    # segment, u collapses to (const - Kp*y), so u and y become strongly and
    # misleadingly ANTI-correlated by construction (not because of plant
    # dynamics or a hardware sign flip) -- a classic closed-loop
    # identifiability pitfall. std(u_plant) alone cannot catch this because
    # noise/oscillation in y alone can keep std(u) high.
    # 参照信号（target/rate_ref）がほぼ動いていない区間は、u_plant 自体は
    # 上の活動量フィルタを通過していても除外する。閉ループP制御では
    # u = Kp*(target - y) であり、target がその区間でほぼ一定だと u は
    # (定数 - Kp*y) に潰れてしまう。その結果 u と y はプラントの動特性とも
    # ハードウェアの符号とも無関係に、構造的に強く・誤解を招くほど負に
    # 相関する（閉ループ同定の典型的な落とし穴）。y だけのノイズ/振動でも
    # std(u) は高いままになり得るため、std(u_plant) 単独ではこれを検出
    # できない。
    # 'indirect' mode is EXEMPT from this filter: its rationale (u=Kp*
    # (target-y) collapsing onto -Kp*y when target is near-constant) simply
    # does not apply -- 'indirect' never constructs that u at all, it fits
    # target->gyro directly. Empirically it recovers good fits (R^2 0.35-0.82)
    # from segments with std(target) well below this filter's threshold
    # (2026-09-10, real lesson_07 flights) -- applying the direct-mode guard
    # here would reject genuinely good indirect fits. The R^2>0.3 filter a
    # few lines below is 'indirect's own, outcome-based quality gate.
    # 'indirect' モードはこのフィルタの対象外: その根拠（target がほぼ一定の
    # ときに u=Kp*(target-y) が -Kp*y に潰れる）がそもそも当てはまらない --
    # 'indirect' はその u を一切構成せず、target->gyro を直接フィットする。
    # 実測でも、このフィルタの閾値を大きく下回る std(target) のセグメントから
    # 良好なフィット（R^2 0.35〜0.82）を得ている（2026-09-10、実際の実習7
    # フライト）-- ここで直接法向けのガードを適用すると、間接法の正しい
    # フィットまで棄却してしまう。数行下の R^2>0.3 フィルタが、間接法自身の
    # 結果に基づく品質ゲートとして働く。
    excitation_check_applies = resolved_mode != 'indirect'
    excitation_ok_segments = []
    n_dropped_excitation = 0
    for seg_start, seg_end in active_segments:
        if (excitation_check_applies and min_target_std > 0.0
                and np.std(target_physical[seg_start:seg_end]) < min_target_std):
            n_dropped_excitation += 1
        else:
            excitation_ok_segments.append((seg_start, seg_end))

    target_excitation_note = ''
    if n_dropped_excitation > 0:
        target_excitation_note = (
            f"{n_dropped_excitation}/{len(active_segments)} segment(s) had "
            f"std(target) < {min_target_std:.3g} rad/s and were dropped: a "
            "near-constant stick reference makes u=Kp*(target-y) collapse "
            "onto -Kp*y, which looks like a plant/sign problem but is a "
            "closed-loop identifiability pitfall -- fly with larger, more "
            "frequent stick motion across the whole flight instead."
        )

    if not excitation_ok_segments:
        raise ValueError(
            "All active segments had insufficient REFERENCE excitation "
            f"(std(target) < {min_target_std:.3g} rad/s in every segment, "
            f"even though {len(active_segments)} segment(s) passed the "
            "control-activity filter). This is the classic closed-loop "
            "identifiability trap: with a near-constant stick target, "
            "u=Kp*(target-y) is dominated by -Kp*y, which fits a strongly "
            "negative/implausible K instead of the true plant gain -- it is "
            "NOT evidence of a sign-inverted mixer or gyro. Re-fly with "
            "continuous, large-amplitude, quasi-random stick motion on this "
            "axis (don't hold one direction for long), or pass "
            "--min-target-std-frac 0 to disable this check."
        )
    active_segments = excitation_ok_segments

    # Fit each segment
    # 各セグメントをフィット
    K_estimates: List[float] = []
    tau_m_estimates: List[float] = []
    r2_values: List[float] = []
    rmse_values: List[float] = []

    for seg_start, seg_end in active_segments:
        y_seg = y_plant[seg_start:seg_end]

        if resolved_mode == 'indirect':
            # Fit the CLOSED loop (target -> gyro) directly, never
            # reconstructing/using u_plant -- see _simulate_closed_loop().
            # 閉ループ（target -> gyro）を直接フィットする -- u_plant の
            # 復元・使用は一切しない（_simulate_closed_loop() 参照）。
            target_seg = target_physical[seg_start:seg_end]
            fit = _fit_segment_indirect(target_seg, y_seg, dt, kp,
                                         K_init=ref_K, tau_m_init=0.02,
                                         K_bounds=K_bounds)
        else:
            u_seg = u_plant[seg_start:seg_end]
            fit = _fit_segment(u_seg, y_seg, dt, K_init=ref_K, tau_m_init=0.02, K_bounds=K_bounds)
        if fit is not None:
            K, tau_m, r2, rmse = fit
            # Filter out unreasonable fits
            # 不合理なフィット結果を除外
            if r2 > 0.3 and K > 1.0 and 0.003 < tau_m < 0.5:
                K_estimates.append(K)
                tau_m_estimates.append(tau_m)
                r2_values.append(r2)
                rmse_values.append(rmse)

    if not K_estimates:
        # 2026-09-10: --mixer is meaningless for 'indirect' (it never
        # reconstructs u_plant at all, see the branch above), so don't
        # point a FIR-auto-Kp/indirect user at it -- that used to be the
        # only advice this message gave, which is actively misleading for
        # the now-default 'auto' path (peer session stampfly-ecosystem-40,
        # 2026-09-10: hit this on a weakly-excited log and found the
        # --mixer suggestion "a bit off the mark").
        # 2026-09-10: --mixer は 'indirect' には無関係（上の分岐の通り
        # u_plant を一切復元しない）ので、FIR自動Kp/indirect 経路の
        # ユーザーをそちらに誘導しない -- 以前はこのメッセージの唯一の助言が
        # それで、既定になった 'auto' 経路には的外れになっていた
        # （stampfly-ecosystem-40、2026-09-10: 弱励振ログでこれに当たり
        # 「--mixer への言及は少し的外れ」と報告）。
        if resolved_mode == 'indirect':
            kp_note = (f"Kp={kp:.4g} auto-estimated via FIR regression "
                       f"(R^2={kp_auto_r_squared:.3f} for that estimate) -- "
                       "sanity-check it against what you actually configured "
                       "and pass --kp explicitly to override it if it looks "
                       "wrong"
                       if kp_source == 'fir_auto'
                       else f"Kp={kp:.4g} given via --kp")
            raise ValueError(
                f"Fitting failed for all segments (indirect closed-loop fit, "
                f"{kp_note}). Most likely the flight had too little "
                "excitation to resolve K/tau_m even with a good Kp -- "
                "re-fly with continuous, large-amplitude, quasi-random "
                "stick motion on this axis. --mixer is NOT the issue here "
                "(indirect never reconstructs u_plant from duty)."
            )
        raise ValueError(
            "Fitting failed for all segments. Check data quality and Kp "
            "value -- if this is a --input duty fit, also check --mixer: "
            "'legacy' is for firmware/workshop (`sf lesson`) and "
            "firmware/vehicle_old logs, 'vehicle' is for firmware/vehicle "
            "(`sf app`) logs. Using the wrong one reconstructs a "
            "physically-wrong u_plant and degrades exactly like this."
        )

    # Aggregate results using median (robust to outliers)
    # 中央値で集約（外れ値に頑健）
    K_final = float(np.median(K_estimates))
    tau_m_final = float(np.median(tau_m_estimates))
    K_std = float(np.std(K_estimates)) if len(K_estimates) > 1 else 0.0
    tau_m_std = float(np.std(tau_m_estimates)) if len(tau_m_estimates) > 1 else 0.0
    r2_mean = float(np.mean(r2_values))
    rmse_mean = float(np.mean(rmse_values))

    return PlantFitResult(
        K=K_final,
        tau_m=tau_m_final,
        K_std=K_std,
        tau_m_std=tau_m_std,
        r_squared=r2_mean,
        rmse=rmse_mean,
        axis=axis,
        kp_used=kp_used,
        n_segments=len(K_estimates),
        input_mode=resolved_mode,
        # 'control_output' u_plant is always torque-scale [Nm], same
        # reference gains as --mixer vehicle (see to_dict()'s ref_gains
        # selection) -- report 'vehicle' even though no mixer was actually
        # inverted, so to_dict() picks REFERENCE_PLANT_GAINS_VEHICLE. Same
        # reasoning for 'indirect' with a FIR-auto-estimated Kp
        # (kp_auto_vehicle_scale): that Kp came from actual_torque_diag
        # [Nm], so K is torque-scale too.
        # 'control_output' の u_plant は常にトルク尺度[Nm]、--mixer vehicle
        # と同じ参照ゲイン（to_dict() の ref_gains 選択参照）— 実際には
        # ミキサーを逆算していなくても 'vehicle' と報告し、to_dict() が
        # REFERENCE_PLANT_GAINS_VEHICLE を選ぶようにする。FIR自動推定Kpの
        # 'indirect'（kp_auto_vehicle_scale）も同じ理由: その Kp は
        # actual_torque_diag[Nm] 由来なので K もトルク尺度になる。
        mixer=('vehicle' if resolved_mode == 'control_output'
               else mixer if resolved_mode == 'duty'
               else 'vehicle' if kp_auto_vehicle_scale else 'legacy'),
        duty_quality=duty_quality,
        duty_reason=duty_reason,
        mixer_gain=mixer_gain,
        mixer_gain_r_squared=mixer_gain_r2,
        mixer_gain_label=mixer_gain_label,
        n_segments_excitation_dropped=n_dropped_excitation,
        target_excitation_note=target_excitation_note,
        kp_source=kp_source,
        kp_auto_r_squared=kp_auto_r_squared,
        crash_truncated_at=crash_truncated_at,
    )


def compute_fit_timeseries(
    df: pd.DataFrame,
    result: PlantFitResult,
    rate_max: float = 1.0,
    fs: float = 400.0,
    time_range: Optional[Tuple[float, float]] = None,
) -> Dict[str, np.ndarray]:
    """
    Compute time series data for plotting fit results
    フィット結果のプロット用時系列データを計算

    Args:
        df: Aligned flight-log DataFrame from
            tools/sysid/loader.py's load_aligned() (same one used for
            fitting).
        result: PlantFitResult from fit_plant()
        rate_max: Maximum angular rate [rad/s] -- unused now that the
            bundle format's rate_ref_<axis> is already physical rad/s
            (see fit_plant()'s docstring); kept for CLI-argument
            compatibility.
        fs: Sample rate [Hz]
        time_range: Optional (start, end) in seconds

    Returns:
        Dictionary with keys:
            'time': Time array [s]
            'u_plant': Reconstructed plant input
            'y_measured': Measured angular velocity (gyro)
            'y_simulated': Simulated angular velocity
            'residual': y_measured - y_simulated
    """
    (time_s, target_raw, gyro, throttle, dt, duty_diff, _duty_quality, _duty_reason,
     _actual_torque_diag, ctrl_output_torque, _ctrl_output_available, _crash_truncated_at) = (
        _load_axis_data(df, result.axis, fs, time_range, mixer=result.mixer)
    )

    # Reconstruct plant I/O -- same input-mode logic as fit_plant(), using
    # whichever mode the fit ACTUALLY used (result.input_mode). target_raw
    # is already rad/s (bundle format's one schema) -- see fit_plant()'s
    # docstring.
    # プラント入出力を復元 -- fit_plant() と同じ入力モードのロジック。
    # フィットが実際に使ったモード（result.input_mode）に従う。target_raw は
    # 既に rad/s（一式形式のスキーマは1種類のみ）-- fit_plant() の
    # docstring 参照。
    target_physical = target_raw

    if result.input_mode == 'control_output':
        if ctrl_output_torque is None:
            raise ValueError(
                "fit used the 'control_output' input mode but this bundle "
                "DataFrame has no non-degenerate torque_<axis> column"
            )
        u_plant = ctrl_output_torque
    elif result.input_mode == 'duty':
        if duty_diff is None:
            raise ValueError(
                "fit used the 'duty' input mode but this bundle DataFrame "
                "has no duty_FR/RR/RL/FL columns"
            )
        u_plant = duty_diff
    else:
        # 'indirect' and 'kp' both reconstruct u = kp*(target-gyro) for
        # DISPLAY here; 'indirect' never fits against this u (see
        # fit_plant()'s 'indirect' branch) -- its y_simulated below uses
        # target_physical directly instead.
        # 'indirect' と 'kp' はどちらも表示用に u = kp*(target-gyro) を
        # 復元する -- 'indirect' はこの u に対してフィットしたことは一度も
        # ない（fit_plant() の 'indirect' 分岐参照）。下の y_simulated は
        # target_physical を直接使う。
        u_plant = result.kp_used * (target_physical - gyro)
    y_measured = gyro

    # Simulate full time series with identified parameters
    # 同定パラメータで全時系列をシミュレート
    omega0 = y_measured[0]

    if result.input_mode == 'indirect':
        # Simulate the CLOSED loop (target->gyro) directly, matching what
        # the fit was actually validated against (_fit_segment_indirect).
        # Open-loop integrating u=kp*(target-gyro) through _simulate_plant()
        # (like the other modes below) DIVERGES over a long series: the
        # simulation has no feedback correcting it, so any small K/tau_m
        # mismatch accumulates without bound through the plant's pure
        # integrator across tens of seconds -- unlike the real (measured)
        # closed loop, which stays bounded because of ACTUAL sensor
        # feedback. Confirmed via --plot on a real lesson_07 log: open-loop
        # here produced a simulated trace ~100x the real gyro's range while
        # the segment-level R^2 the fit reported was a modest but sane 0.59.
        # 閉ループ（target->gyro）を直接シミュレートする -- フィットが実際に
        # 検証されたのと同じ方式（_fit_segment_indirect）。下の他モードと
        # 同様に u=kp*(target-gyro) を _simulate_plant() で開ループ積分する
        # と、長い時系列全体で発散する: シミュレーションにはそれを補正する
        # フィードバックが無いため、わずかな K/tau_m のズレでもプラントの
        # 純粋な積分器を通じて数十秒かけて無制限に蓄積する -- 実際の
        # （実測の）閉ループは実センサのフィードバックにより有界に留まるのと
        # 対照的。実習7の実ログで --plot 検証済み: 開ループでは実測ジャイロ
        # の範囲の約100倍のシミュレーション軌跡になったが、フィットが報告
        # したセグメント単位の R^2 は 0.59 という地味だが妥当な値だった。
        y_simulated = _simulate_closed_loop(
            result.K, result.tau_m, result.kp_used, target_physical, dt, omega0,
        )
    else:
        n_init = min(10, len(y_measured) - 1)
        z0 = float(y_measured[n_init] - y_measured[0]) / (n_init * dt)

        y_simulated = _simulate_plant(
            result.K, result.tau_m, u_plant, dt, omega0, z0,
        )

    return {
        'time': time_s,
        'u_plant': u_plant,
        'y_measured': y_measured,
        'y_simulated': y_simulated,
        'residual': y_measured - y_simulated,
    }


# =============================================================================
# Self-test: synthesize a flight-log v1 bundle from a KNOWN plant, recover
# it. Run via `sf sysid fit --selftest`.
# 自己テスト: 既知プラントから StampFly フライトログ v1 一式を合成し、復元を
# 検証。`sf sysid fit --selftest` から実行。
# =============================================================================

def _selftest_bundle_df(streams: Dict[str, pd.DataFrame]) -> pd.DataFrame:
    """
    Build a synthetic StampFly flight-log v1 bundle from the given streams,
    save it to a temp directory, and load it back through
    tools.sysid.loader.load_aligned() -- the SAME function `sf sysid fit`
    uses -- so selftest() exercises the real bundle write/read/align path
    end to end, not a bespoke test-only reader.
    与えられたストリームから合成 StampFly フライトログ v1 一式を作り、一時
    ディレクトリへ保存し、tools.sysid.loader.load_aligned()（`sf sysid fit`
    が使うのと同じ関数）で読み戻す -- selftest() がテスト専用のリーダでは
    なく、本物の一式書き込み・読み込み・整列経路を一気通貫で検証するため。

    Args:
        streams: stream name (e.g. "imu", "rate_ref", "motor") -> DataFrame,
            same shape as sflog.FlightLog.streams (see lib/sflog/bundle.py).

    Returns:
        The aligned DataFrame (base="imu"), with df.attrs["bundle_streams"]
        set to the stream names actually written -- see
        tools/sysid/loader.py's load_aligned() docstring.
    """
    import shutil
    import tempfile

    tmp_dir = tempfile.mkdtemp(prefix='plant_fit_selftest_')
    try:
        log = sflog.FlightLog(
            meta=sflog.make_meta(
                source='sils', tool_name='tools.sysid.plant_fit.selftest',
                tool_version='1.0', streams=streams,
            ),
            schema={},
            streams=streams,
        )
        # A plain directory (not a .zip) round-trips through the same
        # save()/load() code path but skips zip compression -- faster for a
        # selftest that builds several of these, and still exercises
        # load_aligned() exactly as the CLI calls it (see the module
        # docstring's "Prefer running through load_aligned() at least once"
        # requirement, flight-log-format-plan.md section 5).
        # 一式は素のディレクトリ（.zip ではない）で往復させる -- save()/
        # load() と同じコード経路を通りつつ zip 圧縮を省く（このセルフ
        # テストは一式をいくつも作るため高速な方を選ぶ）。それでも
        # load_aligned() を CLI が呼ぶのと全く同じに検証できる。
        log.save(tmp_dir)
        return load_aligned(tmp_dir)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


def selftest(verbose: bool = True) -> bool:
    """
    Closed-loop self-test for the flight-log-bundle path.
    既知プラントを P 制御閉ループで離散シミュレーションし、`sf log wifi` が
    書く StampFly フライトログ v1 一式と同じ形の合成一式を作り、
    fit_plant() が K, tau_m を許容誤差内で復元することを確認する。

    Verifies end to end: each synthetic scenario's streams round-trip
    through sflog.FlightLog.save() / tools.sysid.loader.load_aligned() (the
    SAME function `sf sysid fit` uses -- see _selftest_bundle_df()),
    _load_axis_data() reads rate_ref_roll/gyro_x/duty_FR..FL correctly from
    the resulting aligned DataFrame, and the MSE fit in
    _fit_segment()/_fit_segment_indirect() recovers the plant that
    generated the data. Uses the EXACT same discretization as
    _simulate_plant()/_simulate_closed_loop() to generate each synthetic
    flight, so any recovery error reflects the fit tool's own accuracy, not
    a model mismatch.
    一気通貫の検証: 各シナリオの合成ストリームが sflog.FlightLog.save() /
    tools.sysid.loader.load_aligned()（`sf sysid fit` が使うのと同じ関数 --
    _selftest_bundle_df() 参照）を往復し、_load_axis_data() がその整列済み
    DataFrame から rate_ref_roll/gyro_x/duty_FR..FL を正しく読み、
    _fit_segment()/_fit_segment_indirect() の MSE フィットが生成元の
    プラントを復元できることを確認する。各合成フライトの生成には
    _simulate_plant()/_simulate_closed_loop() と全く同じ離散化を使うため、
    復元誤差はフィットツール自体の精度を反映し、モデル不整合には起因しない。
    """
    axis = 'roll'
    K_true = REFERENCE_PLANT_GAINS[axis]   # 102.0 [rad/s^2 per duty]
    # Needed early: 'auto' now resolves to 'indirect' with a FIR-auto Kp
    # estimated from actual_torque_diag [Nm] even on this legacy-mixer
    # synthetic flight (see fit_plant()'s auto ladder, 2026-09-10 second
    # pass), so its K comes out VEHICLE/torque-scale -- compared against
    # this reference, not K_true, in _check() below.
    # 早期に必要: 'auto' はこの legacy ミキサーの合成飛行でも、
    # actual_torque_diag[Nm] から推定したFIR自動Kpで 'indirect' に解決
    # される（fit_plant() の auto ラダー、2026-09-10 第2弾参照）ため、その
    # K は vehicle/トルク尺度になる -- 下の _check() では K_true ではなく
    # こちらと比較する。
    K_true_vehicle = REFERENCE_PLANT_GAINS_VEHICLE[axis]   # ~1/Ixx [rad/s^2/Nm]
    tau_m_true = 0.02                      # [s] -- L06 nominal motor lag
    kp = 0.5
    fs = 400.0
    dt = 1.0 / fs
    n = 8000                               # 20 s @ 400 Hz

    alpha = np.exp(-dt / tau_m_true)
    gain = K_true * (1.0 - alpha)

    t = np.arange(n) * dt
    ts_us = (t * 1e6).astype(np.int64)
    seq = np.arange(n)
    # Broadband log-chirp target (0.5->20 Hz over 5 s, repeated): similar
    # spectral richness to a pilot's stick doublets, wide enough to resolve
    # both the integrator gain K and the ~8 Hz motor-lag corner (tau_m=20ms).
    # 広帯域対数チャープ目標（0.5〜20Hz、5s周期で繰り返し）: パイロットの
    # スティックダブレットに近いスペクトルで、積分ゲイン K とモータ遅れの
    # コーナー周波数（tau_m=20ms、約8Hz）の両方を解ける帯域幅を持つ。
    f0, f1, period = 0.5, 20.0, 5.0
    k_chirp = (f1 / f0) ** (1.0 / period)
    tm = t % period
    target = 0.35 * np.sin(2 * np.pi * f0 * ((k_chirp ** tm) - 1.0) / np.log(k_chirp))

    # Closed-loop P-control simulation with the EXACT discretization
    # _simulate_plant() uses (motor-lag alpha filter + trapezoidal
    # integration) -- z[i]/omega[i] depend only on u[i-1], so this is causal
    # and needs no algebraic-loop solving.
    # _simulate_plant() と全く同じ離散化（モータ遅れの指数フィルタ + 台形
    # 積分）による閉ループ P 制御シミュレーション -- z[i]/omega[i] は
    # u[i-1] のみに依存するため、代数ループを解く必要のない因果的な計算。
    omega = np.zeros(n)
    z = np.zeros(n)
    u = np.zeros(n)
    for i in range(1, n):
        error = target[i - 1] - omega[i - 1]
        u[i - 1] = kp * error
        z[i] = alpha * z[i - 1] + gain * u[i - 1]
        omega[i] = omega[i - 1] + 0.5 * dt * (z[i - 1] + z[i])
    u[-1] = kp * (target[-1] - omega[-1])

    rng = np.random.default_rng(7)
    gyro_meas = omega + rng.normal(0.0, 0.003, n)   # [rad/s] BMI270-scale noise

    # Synthesize the 4 motor duties from `u` (the roll-axis PID output above)
    # through the FORWARD X-quad mixer (ws_internal.hpp motor_mixer, P=Y=0
    # for a roll-only excitation): FR=RR=T-k*R, RL=FL=T+k*R. This is the
    # exact inverse-of-_duty_differential() construction, so the "duty"
    # input mode below must recover u (and hence K, tau_m) as well as the
    # "kp" mode does.
    # `u`（上のロール軸PID出力）から4モータduty を「順」ミキサ
    # （ws_internal.hpp motor_mixer、ロール単独励振なので P=Y=0）で合成する:
    # FR=RR=T-k*R, RL=FL=T+k*R。これは _duty_differential() の厳密な逆構成
    # なので、下の "duty" 入力モードは "kp" モードと同等に u（ひいては
    # K, tau_m）を復元できるはずである。
    T_hover = 0.4
    duty_fr = T_hover - _MIXER_K * u
    duty_rr = T_hover - _MIXER_K * u
    duty_rl = T_hover + _MIXER_K * u
    duty_fl = T_hover + _MIXER_K * u

    # Build a bundle with a genuine 400Hz `motor` stream (same header shape
    # as `sf log wifi`'s output): only the test axis carries nonzero
    # rate_ref/gyro, and duty_FR/RR/RL/FL carry the synthesized duty above
    # (the "duty" input mode's data source). Throttle-equivalent for
    # _find_flight_segments() falls back to the mean of the 4 duty columns
    # (no `total_thrust`/`ctrl_ref` stream needed -- see
    # _load_axis_data()'s throttle-equivalent fallback): the roll-only
    # differential cancels in that mean, leaving exactly T_hover=0.4,
    # comfortably above the 0.3 flight threshold throughout.
    # 一式を、本物の400Hz `motor` ストリーム付きで作る（`sf log wifi` の
    # 出力と同じヘッダ形状）: テスト対象軸のみ rate_ref/gyro を非ゼロにし、
    # duty_FR/RR/RL/FL は上で合成した duty（"duty" 入力モードのデータ源）。
    # _find_flight_segments() 用のスロットル相当量は4 duty 列の平均に
    # フォールバックする（`total_thrust`/`ctrl_ref` ストリームは不要 --
    # _load_axis_data() のスロットル相当量フォールバック参照）: ロール
    # 単独励振の差動成分はその平均で相殺し、ちょうど T_hover=0.4 になり、
    # 飛行判定閾値0.3を終始十分に上回る。
    df_main = _selftest_bundle_df({
        'imu': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'gyro_x': gyro_meas}),
        'rate_ref': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'rate_ref_roll': target}),
        'motor': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq,
                                'duty_FR': duty_fr, 'duty_RR': duty_rr,
                                'duty_RL': duty_rl, 'duty_FL': duty_fl}),
    })

    result_kp = fit_plant(df_main, axis=axis, kp=kp, rate_max=1.0, fs=fs,
                           input_mode='kp')
    result_duty = fit_plant(df_main, axis=axis, rate_max=1.0, fs=fs,
                             input_mode='duty')
    # 2026-09-10: 'auto' on this genuine (continuously-varying) 400Hz
    # duty bundle, with NO --kp given, must now resolve to 'indirect' via
    # _estimate_kp_fir()'s auto-estimated Kp (h(0) from a short FIR
    # regression of u=duty_diff against lags of target-gyro) -- proving
    # both that the (unmodified) _classify_duty_source() correctly reads
    # this bundle's genuine 400Hz `motor` stream as 'duty400' AND that the
    # FIR auto-Kp estimate is accurate enough to drive a correct
    # indirect fit end to end, with zero manual parameters.
    # 2026-09-10: この本物の（連続的に変化する）400Hz duty 一式で、
    # --kp を一切与えない 'auto' は、_estimate_kp_fir() の自動推定Kp
    # （u=duty_diff を target-gyro の複数ラグに短いFIR回帰した h(0)）
    # 経由で 'indirect' に解決されること -- （変更していない）
    # _classify_duty_source() がこの一式の本物の400Hz `motor` ストリームを
    # 正しく 'duty400' と読むことと、FIR自動推定Kpが手動パラメータ一切無しで
    # 正しい間接フィットを駆動できる精度であることの両方を証明する。
    result_auto = fit_plant(df_main, axis=axis, rate_max=1.0, fs=fs,
                             input_mode='auto')
    # 2026-09-10: indirect closed-loop fit (target->gyro, known kp) on
    # the SAME synthetic closed-loop flight (target, gyro_meas were
    # generated by simulating exactly this kp/K_true/tau_m_true loop
    # above) -- must recover K_true, tau_m_true just as well as 'kp'/
    # 'duty' do. See _simulate_closed_loop()/_fit_segment_indirect().
    # 2026-09-10: 間接閉ループフィット（target->gyro、既知kp）を、同じ
    # 合成閉ループフライト（target, gyro_meas は上でこの
    # kp/K_true/tau_m_true のループをシミュレーションして生成した
    # もの）に対して行う -- 'kp'/'duty' と同様に K_true, tau_m_true を
    # 復元できること。_simulate_closed_loop()/_fit_segment_indirect()
    # 参照。
    result_indirect = fit_plant(df_main, axis=axis, kp=kp, rate_max=1.0,
                                 fs=fs, input_mode='indirect')

    # --- Additional case: a 50Hz-HELD duty (the bundle's `motor` stream is
    # ABSENT; only the `ctrl_ref` stream supplies duty_FR/RR/RL/FL, at its
    # own native 50Hz timestamps -- sflog.aligned()'s merge_asof does the
    # holding onto the 400Hz base, exercising the REAL alignment code path
    # rather than hand-building an 8-row-repeat staircase). This is the new,
    # definitive way to test what used to be a synthetic "stairstep" CSV:
    # subsampling the SAME continuous duty_fr/rr/rl/fl arrays above at every
    # 8th index (ctrl_ref_idx) and letting merge_asof hold them reconstructs
    # EXACTLY the old hand-built staircase (duty_fr[start] held for
    # [start, start+8) is what backward-asof against a 50Hz sample at
    # `start` produces for every base row in that range). 'auto' must NOT
    # silently fit off this stale/quantized signal: it must fall back to
    # 'kp' (via the 'indirect' ladder) and require --kp.
    # 追加ケース: 50Hz保持のduty（一式に `motor` ストリームが無く、
    # `ctrl_ref` ストリームだけが duty_FR/RR/RL/FL を、その素の50Hzタイム
    # スタンプで供給する -- sflog.aligned() の merge_asof が400Hz基準への
    # 保持を行う、本物の整列コード経路を検証する。8行連続保持の階段状データを
    # 手作業で作る代わりに）。これが、以前の合成「階段状」CSVが検証していた
    # ものを検証する新しい・決定的な方法: 上と同じ連続 duty_fr/rr/rl/fl
    # 配列を8個おき（ctrl_ref_idx）に間引き、merge_asof に保持させれば、
    # 旧来の手作業の階段状データ（duty_fr[start] を [start, start+8) の間
    # 保持）と厳密に一致する（`start` の50Hz標本に対する backward-asof は
    # その範囲の全基準行に同じ値を返すため）。'auto' はこの古い/粗い信号で
    # 黙ってフィットしてはならない -- （'indirect' ラダー経由で）'kp' へ
    # フォールバックし --kp を要求すること。
    ctrl_ref_idx = np.arange(0, n, 8)
    df_stair = _selftest_bundle_df({
        'imu': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'gyro_x': gyro_meas}),
        'rate_ref': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'rate_ref_roll': target}),
        'ctrl_ref': pd.DataFrame({
            'timestamp_us': ts_us[ctrl_ref_idx],
            'duty_FR': duty_fr[ctrl_ref_idx], 'duty_RR': duty_rr[ctrl_ref_idx],
            'duty_RL': duty_rl[ctrl_ref_idx], 'duty_FL': duty_fl[ctrl_ref_idx],
        }),
    })

    try:
        fit_plant(df_stair, axis=axis, kp=None, rate_max=1.0, fs=fs, input_mode='auto')
        stair_rejected_without_kp = False
        stair_reject_msg = "(did not raise)"
    except ValueError as e:
        stair_reject_msg = str(e)
        stair_rejected_without_kp = 'old firmware' in stair_reject_msg.lower()

    result_stair_kp = fit_plant(df_stair, axis=axis, kp=kp, rate_max=1.0, fs=fs,
                                 input_mode='auto')

    # 2026-09-10: 'auto' with --kp given now resolves to 'indirect', not the
    # old direct 'kp' -- same trigger (kp is not None, no genuine 400Hz
    # duty), strictly more robust reconstruction (see fit_plant()'s auto
    # ladder and _simulate_closed_loop()'s docstring). duty_quality stays
    # 'duty50' regardless (that describes the bundle's duty columns, which
    # 'indirect' never reads).
    # 2026-09-10: --kp が与えられた 'auto' は、もはや旧来の直接 'kp' では
    # なく 'indirect' に解決される -- トリガーは同じ（kp が None でない、
    # 本物の400Hz duty が無い）が、逆算はより頑健になった（fit_plant() の
    # auto ラダーと _simulate_closed_loop() の docstring 参照）。
    # duty_quality は変わらず 'duty50'（これは一式のduty列自体の性質を表す
    # もので、'indirect' はそもそもduty列を読まない）。
    stair_resolved_kp = (result_stair_kp.input_mode == 'indirect'
                          and result_stair_kp.duty_quality == 'duty50')
    K_err_stair = abs(result_stair_kp.K / K_true - 1.0)
    ok_stair = stair_rejected_without_kp and stair_resolved_kp and K_err_stair < 0.15
    if verbose:
        print(f"[stair] auto w/o --kp rejected ({stair_reject_msg[:70]}...): "
              f"{stair_rejected_without_kp}")
        print(f"[stair] auto w/ --kp resolves to 'indirect' (duty_quality="
              f"{result_stair_kp.duty_quality}): {stair_resolved_kp}  "
              f"K={result_stair_kp.K:.1f} ({K_err_stair * 100:.1f}% err)  "
              f"R^2={result_stair_kp.r_squared:.3f}")

    def _check(result, label):
        # 'indirect' via FIR auto-Kp reports mixer=='vehicle' (see
        # fit_plant()'s use_vehicle_scale/mixer= construction) because its
        # K came from actual_torque_diag [Nm], not the legacy duty-
        # differential scale -- compare against the matching reference.
        # FIR自動Kp経由の 'indirect' は mixer=='vehicle' と報告される
        # （fit_plant() の use_vehicle_scale/mixer= 構築参照）。その K は
        # actual_torque_diag[Nm] 由来で legacy の duty差動スケールではない
        # ため、対応する参照値と比較する。
        ref_K = K_true_vehicle if result.mixer == 'vehicle' else K_true
        K_err = abs(result.K / ref_K - 1.0)
        tau_err = abs(result.tau_m / tau_m_true - 1.0)
        passed = K_err < 0.15 and tau_err < 0.30 and result.r_squared > 0.9
        if verbose:
            print(f"[{label}] fit: K={result.K:.1f} (ref={ref_K:.1f}, "
                  f"{K_err * 100:.1f}% err)  "
                  f"tau_m={result.tau_m * 1000:.1f} ms ({tau_err * 100:.1f}% err)  "
                  f"R^2={result.r_squared:.3f}  n_segments={result.n_segments}  "
                  f"input_mode={result.input_mode}  mixer={result.mixer}")
        return passed

    if verbose:
        print(f"true : K={K_true:.1f} [rad/s^2/duty] / K_vehicle={K_true_vehicle:.1f} "
              f"[rad/s^2/Nm]  tau_m={tau_m_true * 1000:.1f} ms")
    ok_kp = _check(result_kp, 'kp')
    ok_duty = _check(result_duty, 'duty')
    # mixer defaults to 'legacy' for this synthetic flight (fit_plant() not
    # given --mixer), so the FIR auto-Kp estimate must use duty_diff (NOT
    # actual_torque_diag) and stay legacy-scale -- see the auto ladder's
    # 2026-09-10 third-pass comment for why unconditionally switching to
    # vehicle/Nm scale would be wrong for today's workshop curriculum.
    # mixer はこの合成飛行では既定の 'legacy'（fit_plant() に --mixer を
    # 渡していない）なので、FIR自動Kp推定は actual_torque_diag ではなく
    # duty_diff を使い、legacy スケールのままであること -- なぜ vehicle/Nm
    # スケールへ無条件で切り替えるのが今日の実習カリキュラムにとって誤り
    # かは、auto ラダーの2026-09-10第3弾コメント参照。
    ok_auto = (_check(result_auto, 'auto')
               and result_auto.input_mode == 'indirect'
               and result_auto.kp_source == 'fir_auto'
               and result_auto.mixer == 'legacy')
    if verbose:
        print(f"[auto] kp_source={result_auto.kp_source}  "
              f"kp_used={result_auto.kp_used:.6g} (mixer={result_auto.mixer})  "
              f"kp_auto_r2={result_auto.kp_auto_r_squared}")
    ok_indirect = _check(result_indirect, 'indirect') and result_indirect.input_mode == 'indirect'

    # --- --mixer vehicle regression: same closed-loop synthesis, but the
    # roll-axis PID output `u_v` is now a physical differential TORQUE [Nm]
    # (not a duty-differential command), converted to 4 motor duties through
    # the FORWARD physical chain (B^-1 allocation -> thrustToDuty(), the same
    # _ARM_D/_MOTOR_AM/_MOTOR_BM/_MOTOR_CM/_MOTOR_CT constants
    # _duty_differential_vehicle()/_thrust_from_duty() invert) instead of the
    # linear ws_internal mixer used by `u`/duty_fr above. Proves the vehicle
    # inversion recovers its own forward model correctly (sign, scale, AND
    # voltage -- vbat_true is deliberately off V_BATT_NOMINAL so a bug that
    # silently used the nominal fallback instead of the bundle's `voltage`
    # column would show up as a scale error here) -- the same sanity the
    # "duty" case above gets from the ws_internal forward mixer.
    # --mixer vehicle 回帰: 同じ閉ループ合成だが、ロール軸PID出力 `u_v` は
    # duty差動指令ではなく物理的な差動トルク[Nm]で、線形の ws_internal
    # ミキサーではなく順方向の物理チェーン（B^-1配分 -> thrustToDuty()。
    # _duty_differential_vehicle()/_thrust_from_duty() が逆算するのと同じ
    # _ARM_D/_MOTOR_AM/_MOTOR_BM/_MOTOR_CM/_MOTOR_CT 定数を使う）で4モータ
    # duty に変換する。vehicle 逆算が自身の順方向モデルを正しく逆算できる
    # こと（符号・スケール・**電圧** -- vbat_true は意図的に
    # V_BATT_NOMINAL からずらしてあるので、一式の `voltage` 列を使わず黙って
    # ノミナルへフォールバックするバグがあればここでスケール誤差として
    # 現れる）を証明する -- 上の "duty" ケースが順方向 ws_internal ミキサー
    # で得ているのと同じ健全性チェック。
    # (K_true_vehicle defined earlier, near K_true -- ok_auto needs it too)
    kp_vehicle = 1.0e-3    # [Nm/(rad/s)] -- same order as firmware rate.roll.kp
    vbat_true = 3.85       # [V] -- deliberately != V_BATT_NOMINAL (3.7)
    thrust_hover_total = 0.4   # [N] -- arbitrary in-flight value (> the 0.3
                               # flight-segment threshold, matching T_hover
                               # above), only needs to keep all 4 per-motor
                               # thrusts positive across the excitation
                               # amplitude

    alpha_v = np.exp(-dt / tau_m_true)
    gain_v = K_true_vehicle * (1.0 - alpha_v)

    omega_v = np.zeros(n)
    z_v = np.zeros(n)
    u_v = np.zeros(n)   # roll differential torque [Nm]
    for i in range(1, n):
        error = target[i - 1] - omega_v[i - 1]
        u_v[i - 1] = kp_vehicle * error
        z_v[i] = alpha_v * z_v[i - 1] + gain_v * u_v[i - 1]
        omega_v[i] = omega_v[i - 1] + 0.5 * dt * (z_v[i - 1] + z_v[i])
    u_v[-1] = kp_vehicle * (target[-1] - omega_v[-1])

    gyro_meas_v = omega_v + rng.normal(0.0, 0.003, n)

    # FORWARD B^-1 allocation (roll-only excitation: pitch=yaw=0, matching
    # actuator.cpp mixerCompute()) then the FORWARD motor curve (matching
    # actuator.cpp thrustToDuty()) -- the exact inverse of what
    # _duty_differential_vehicle()/_thrust_from_duty() compute.
    t_fr_v = 0.25 * (thrust_hover_total - u_v / _ARM_D)
    t_rr_v = 0.25 * (thrust_hover_total - u_v / _ARM_D)
    t_rl_v = 0.25 * (thrust_hover_total + u_v / _ARM_D)
    t_fl_v = 0.25 * (thrust_hover_total + u_v / _ARM_D)

    def _thrust_to_duty(thrust):
        omega_m = np.sqrt(np.maximum(thrust, 0.0) / _MOTOR_CT)
        volts = _MOTOR_AM * omega_m ** 2 + _MOTOR_BM * omega_m + _MOTOR_CM
        return volts / vbat_true

    duty_fr_v = _thrust_to_duty(t_fr_v)
    duty_rr_v = _thrust_to_duty(t_rr_v)
    duty_rl_v = _thrust_to_duty(t_rl_v)
    duty_fl_v = _thrust_to_duty(t_fl_v)

    # Common streams for the two vehicle-physics scenarios below (--mixer
    # vehicle "duty"/"auto", and the "control_output" scenario that reuses
    # the SAME vehicle-physics flight) -- both need genuine 400Hz duty (the
    # `motor` stream) and the deliberately-off-nominal battery voltage (the
    # `status` stream) for the vehicle motor-curve inversion.
    # 下の vehicle 物理合成を使う2シナリオ（--mixer vehicle の "duty"/
    # "auto"、および同じ vehicle 物理フライトを再利用する
    # "control_output" シナリオ）に共通のストリーム -- どちらも vehicle
    # モータ曲線逆算に、本物の400Hz duty（`motor` ストリーム）と、意図的に
    # ノミナルからずらしたバッテリ電圧（`status` ストリーム）が要る。
    imu_v = pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'gyro_x': gyro_meas_v})
    rate_ref_v = pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'rate_ref_roll': target})
    motor_v = pd.DataFrame({'timestamp_us': ts_us, 'seq': seq,
                             'duty_FR': duty_fr_v, 'duty_RR': duty_rr_v,
                             'duty_RL': duty_rl_v, 'duty_FL': duty_fl_v})
    status_v = pd.DataFrame({'timestamp_us': np.array([0], dtype=np.int64),
                              'voltage': np.array([vbat_true])})

    df_vehicle = _selftest_bundle_df({
        'imu': imu_v, 'rate_ref': rate_ref_v, 'motor': motor_v, 'status': status_v,
    })

    result_vehicle = fit_plant(df_vehicle, axis=axis, rate_max=1.0, fs=fs,
                                input_mode='duty', mixer='vehicle')
    # 2026-09-10: --mixer vehicle + 'auto' (no --kp) must use
    # actual_torque_diag for the FIR auto-Kp estimate (opt-in vehicle/Nm
    # scale, per the auto ladder's third-pass comment) and recover the
    # SAME K_true_vehicle -- the counterpart to ok_auto/ok_ws_zero
    # above, which check the mixer=='legacy' (default) case stays on
    # the legacy duty-differential scale instead.
    # 2026-09-10: --mixer vehicle + 'auto'（--kp無し）は FIR自動Kp推定に
    # actual_torque_diag を使い（auto ラダーの第3弾コメント通り、
    # vehicle/Nmスケールはオプトイン）、同じ K_true_vehicle を復元する
    # こと -- 上の ok_auto/ok_ws_zero（mixer=='legacy'既定では legacy
    # duty差動スケールのままであることを確認）の対になるテスト。
    result_vehicle_auto = fit_plant(df_vehicle, axis=axis, rate_max=1.0, fs=fs,
                                     input_mode='auto', mixer='vehicle')

    K_err_v = abs(result_vehicle.K / K_true_vehicle - 1.0)
    tau_err_v = abs(result_vehicle.tau_m / tau_m_true - 1.0)
    ok_vehicle = K_err_v < 0.15 and tau_err_v < 0.30 and result_vehicle.r_squared > 0.9
    if verbose:
        print(f"[vehicle] true: K={K_true_vehicle:.1f} [rad/s^2/Nm]  "
              f"tau_m={tau_m_true * 1000:.1f} ms")
        print(f"[vehicle] fit : K={result_vehicle.K:.1f} ({K_err_v * 100:.1f}% err)  "
              f"tau_m={result_vehicle.tau_m * 1000:.1f} ms ({tau_err_v * 100:.1f}% err)  "
              f"R^2={result_vehicle.r_squared:.3f}  n_segments={result_vehicle.n_segments}  "
              f"mixer={result_vehicle.mixer}")

    K_err_va = abs(result_vehicle_auto.K / K_true_vehicle - 1.0)
    tau_err_va = abs(result_vehicle_auto.tau_m / tau_m_true - 1.0)
    ok_vehicle_auto = (result_vehicle_auto.input_mode == 'indirect'
                        and result_vehicle_auto.kp_source == 'fir_auto'
                        and result_vehicle_auto.mixer == 'vehicle'
                        and K_err_va < 0.15 and tau_err_va < 0.30
                        and result_vehicle_auto.r_squared > 0.9)
    if verbose:
        print(f"[vehicle/auto] fit: K={result_vehicle_auto.K:.1f} "
              f"({K_err_va * 100:.1f}% err)  "
              f"tau_m={result_vehicle_auto.tau_m * 1000:.1f} ms "
              f"({tau_err_va * 100:.1f}% err)  "
              f"R^2={result_vehicle_auto.r_squared:.3f}  "
              f"kp_source={result_vehicle_auto.kp_source}  "
              f"mixer={result_vehicle_auto.mixer}")

    # --- control_output input mode + mixer-gain diagnostic (rate-sysid
    # design memo, 2026-09-09, §07 "K_measured = physical gain x mixer gain"
    # / §08 "measure c from the log"): reuses the roll-only vehicle-physics
    # synthesis above (u_v drives the REAL forward B^-1 + motor curve ->
    # duty_fr_v etc, gyro_meas_v), but adds a `ctrl_output` stream carrying a
    # DELIBERATELY MISCALIBRATED torque_roll = u_v / c_true (c_true != 1) --
    # i.e. the commanded torque a hypothetical controller asked for is NOT
    # what was actually delivered (u_v, which drives gyro_meas_v). Proves
    # two things at once: (1) fitting input_mode='control_output' against
    # the miscalibrated commanded signal correctly recovers K_measured =
    # c_true * K_true_vehicle (NOT K_true_vehicle) -- exactly the §07
    # relationship, not a bug; (2) the mixer_gain diagnostic (commanded vs
    # duty-derived actual torque) recovers c_true itself.
    # --- control_output 入力モード＋ミキサーゲイン診断（2026-09-09 レート
    # 同定設計メモ §07「K_measured = 物理ゲイン x ミキサーゲイン」/
    # §08「ログから c を測る」）: 上のロール単独励振・vehicle物理合成
    # （u_v が実際の順方向B^-1+モータ曲線を駆動 -> duty_fr_v等、
    # gyro_meas_v）を再利用しつつ、意図的に較正のズレた
    # torque_roll = u_v / c_true（c_true≠1）を運ぶ `ctrl_output`
    # ストリームを追加する -- つまり仮想のコントローラが要求した指令
    # トルクは、実際に配達されたもの（gyro_meas_v を駆動する u_v）とは
    # 異なる。これで2つを同時に証明する: (1) input_mode='control_output'
    # で較正のズレた指令信号に対してフィットすると、正しく
    # K_measured = c_true * K_true_vehicle（K_true_vehicle ではない）を
    # 復元する -- まさに§07の関係、バグではない。(2) mixer_gain 診断
    # （指令 vs duty逆算の実トルク）が c_true 自体を復元する。
    c_true = 0.85   # deliberately != 1 -- see the comment above

    ctrl_output_co = pd.DataFrame({
        'timestamp_us': ts_us, 'seq': seq,
        'torque_roll': u_v / c_true,
        'torque_pitch': np.zeros(n),
        'torque_yaw': np.zeros(n),
    })
    df_ctrl_output = _selftest_bundle_df({
        'imu': imu_v, 'rate_ref': rate_ref_v, 'motor': motor_v, 'status': status_v,
        'ctrl_output': ctrl_output_co,
    })

    result_ctrl_output = fit_plant(df_ctrl_output, axis=axis, rate_max=1.0, fs=fs,
                                    input_mode='control_output')

    K_expected_co = c_true * K_true_vehicle
    K_err_co = abs(result_ctrl_output.K / K_expected_co - 1.0)
    tau_err_co = abs(result_ctrl_output.tau_m / tau_m_true - 1.0)
    mixer_gain_err = (abs(result_ctrl_output.mixer_gain / c_true - 1.0)
                       if result_ctrl_output.mixer_gain is not None else 1.0)
    ok_ctrl_output = (K_err_co < 0.15 and tau_err_co < 0.30
                       and result_ctrl_output.r_squared > 0.9
                       and mixer_gain_err < 0.15)
    if verbose:
        print(f"[control_output] c_true={c_true}  K_expected={K_expected_co:.1f} "
              f"(= c_true * K_true_vehicle)")
        print(f"[control_output] fit: K={result_ctrl_output.K:.1f} "
              f"({K_err_co * 100:.1f}% err)  tau_m={result_ctrl_output.tau_m * 1000:.1f} ms "
              f"({tau_err_co * 100:.1f}% err)  R^2={result_ctrl_output.r_squared:.3f}  "
              f"input_mode={result_ctrl_output.input_mode}")
        mg = result_ctrl_output.mixer_gain
        mg_str = 'None' if mg is None else f'{mg:.3f}'
        print(f"[control_output] mixer_gain diagnostic: c={mg_str} "
              f"(true {c_true}, err {mixer_gain_err * 100:.1f}%)  "
              f"r2={result_ctrl_output.mixer_gain_r_squared}")

    # --- legacy conversion-factor diagnostic (plausibility, not exact -- the
    # legacy forward mixer composed with the REAL nonlinear motor curve has
    # no hand-derivable closed form, unlike the control_output case above):
    # the EXISTING legacy 'duty' fit (result_duty, from `u`/duty_fr near the
    # top of this function) already has genuine 400Hz duty and no `voltage`
    # column (falls back to nominal), so mixer_gain should come out
    # populated -- a finite, positive conversion factor [Nm per legacy
    # duty-unit], not None or garbage.
    # legacy側の換算係数診断（もっともらしさの確認 -- 正確な期待値ではない。
    # legacyの順方向ミキサーと実際の非線形モータ曲線の合成は手計算できる
    # 閉形式ではない、上のcontrol_outputケースと違って）: この関数冒頭の
    # legacy 'duty'フィット（result_duty、`u`/duty_fr由来）は既に本物の
    # 400Hz dutyを持ち `voltage` 列は無い（ノミナルにフォールバック）ので、
    # mixer_gainが populated されているはず -- 有限・正の換算係数
    # [Nm/legacy duty単位]、Noneでもおかしな値でもない。
    ok_legacy_diag = (result_duty.mixer_gain is not None
                       and np.isfinite(result_duty.mixer_gain)
                       and result_duty.mixer_gain > 0)
    if verbose:
        print(f"[duty/legacy] mixer_gain diagnostic: "
              f"{result_duty.mixer_gain} Nm/legacy-unit  "
              f"r2={result_duty.mixer_gain_r_squared}  "
              f"label={result_duty.mixer_gain_label!r}")

    # --- unit-level regression check for _mixer_conversion_factor() itself,
    # across two very different scales (dimensionless-like ~0.85, and a
    # legacy-conversion-like ~1.2e-3) -- proves the regression math (not the
    # bundle/fit_plant plumbing exercised above) recovers an EXACTLY-known
    # slope.
    # _mixer_conversion_factor() 自体の回帰の単体テスト、2つの大きく異なる
    # 尺度（無次元的な~0.85と、legacy換算係数的な~1.2e-3）で -- 回帰の数式
    # 自体（上で確認した一式/fit_plantの配線ではなく）が既知の傾きを正確に
    # 復元することを証明する。
    rng2 = np.random.default_rng(11)
    actual_synth = rng2.normal(0.0, 1.0, 500)
    ok_unit_diag = True
    for k_true_unit in (0.85, 1.2e-3):
        candidate_synth = actual_synth / k_true_unit
        diag = _mixer_conversion_factor(candidate_synth, actual_synth)
        if diag is None:
            ok_unit_diag = False
            continue
        slope, r2 = diag
        err = abs(slope / k_true_unit - 1.0)
        ok_unit_diag = ok_unit_diag and err < 1e-6 and r2 > 0.999
        if verbose:
            print(f"[mixer_conversion_factor unit test] k_true={k_true_unit:.3g}  "
                  f"recovered={slope:.6g}  err={err:.2e}  r2={r2:.6f}")

    # --- regression (2026-09-10, urgent): a genuine firmware/workshop bundle
    # can have a `ctrl_output` stream present (the firmware sends the 0x4B
    # entry every cycle) but never actually WRITES torque[] --
    # WorkshopControlTask fills only .thrust, from its own duty-scale
    # MotorRequest.thrust, not physical N (see workshop_control_task.cpp).
    # So torque_<axis> reads back as constant zero even though the stream
    # itself is present. Before the fix (2026-09-10) this guarded against,
    # 'auto' would have PREFERRED this all-zero, meaningless
    # control_output over the (working) legacy duty reconstruction -- a
    # regression for every real workshop/lesson_07 log, not an improvement.
    # Reuses the legacy-scale synthetic flight (u, gyro_meas,
    # duty_fr/rr/rl/fl) from the 'kp'/'duty'/'auto' block above.
    # --- 退行防止（2026-09-10、緊急）: 本物の firmware/workshop 一式は
    # `ctrl_output` ストリームが存在し得る（ファームが毎周期 0x4B
    # エントリを送るため）が、torque[] は実際には一切書かれない --
    # WorkshopControlTask が埋めるのは .thrust だけで、しかも物理量Nでは
    # なく自前のduty尺度のMotorRequest.thrust（workshop_control_task.cpp
    # 参照）。そのためストリーム自体は存在していても torque_<axis> は
    # 定数ゼロのまま読める。この修正（2026-09-10）が無ければ 'auto' は
    # この全ゼロで無意味な control_output を、動作する legacy duty
    # 逆算より優先してしまう -- 本物の workshop/実習7 ログすべてにとって
    # 改善ではなく退行になる。上の 'kp'/'duty'/'auto' ブロックの
    # legacy スケール合成飛行（u, gyro_meas, duty_fr/rr/rl/fl）を再利用する。
    ctrl_output_ws = pd.DataFrame({
        'timestamp_us': ts_us, 'seq': seq,
        # torque_* left at 0.0 (never written, like WorkshopControlTask).
        # torque_* はゼロのまま（WorkshopControlTask と同様に一度も
        # 書かれない）。
        'torque_roll': np.zeros(n),
        'torque_pitch': np.zeros(n),
        'torque_yaw': np.zeros(n),
    })
    df_ws_zero = _selftest_bundle_df({
        'imu': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'gyro_x': gyro_meas}),
        'rate_ref': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq, 'rate_ref_roll': target}),
        'motor': pd.DataFrame({'timestamp_us': ts_us, 'seq': seq,
                                'duty_FR': duty_fr, 'duty_RR': duty_rr,
                                'duty_RL': duty_rl, 'duty_FL': duty_fl}),
        'ctrl_output': ctrl_output_ws,
    })

    result_ws_zero = fit_plant(df_ws_zero, axis=axis, rate_max=1.0, fs=fs,
                                input_mode='auto')

    # 2026-09-10: since 'auto' now prefers 'indirect' (via FIR auto-Kp) over
    # any direct fit, this regression's bar is now stricter: it must not
    # only avoid the dead all-zero control_output, but resolve all the way
    # to 'indirect' -- the SAME accuracy check as ok_auto above, on the SAME
    # legacy-scale flight, just with a dead-but-present `ctrl_output`
    # stream.
    # 2026-09-10: 'auto' が（FIR自動Kp経由で）直接法より 'indirect' を優先
    # するようになったため、この回帰テストの基準はより厳しくなった -- 死んだ
    # 全ゼロ control_output を避けるだけでなく、'indirect' まで解決する
    # こと。上の ok_auto と同じ精度チェックを、`ctrl_output` ストリームは
    # 存在するが死んでいる同じ legacy スケールのフライトに対して行う。
    K_err_ws = abs(result_ws_zero.K / K_true - 1.0)
    ok_ws_zero = (result_ws_zero.input_mode == 'indirect'
                  and result_ws_zero.kp_source == 'fir_auto'
                  and result_ws_zero.mixer == 'legacy'
                  and K_err_ws < 0.15 and result_ws_zero.r_squared > 0.9)
    if verbose:
        print(f"[workshop-zero-torque regression] auto resolved to "
              f"'{result_ws_zero.input_mode}' (must be 'indirect', not "
              f"'control_output'/'duty'): K={result_ws_zero.K:.1f} "
              f"({K_err_ws * 100:.1f}% err)  "
              f"R^2={result_ws_zero.r_squared:.3f}")

    ok = (ok_kp and ok_duty and ok_auto and ok_indirect and ok_stair and ok_vehicle
          and ok_ctrl_output and ok_legacy_diag and ok_unit_diag
          and ok_ws_zero and ok_vehicle_auto)

    if verbose:
        print("SELFTEST:", "PASS" if ok else "FAIL")

    return ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if selftest() else 1)
