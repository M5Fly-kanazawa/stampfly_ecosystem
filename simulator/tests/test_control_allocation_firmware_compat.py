#!/usr/bin/env python3
"""
Control Allocation Verification Test
制御アロケーション検証テスト

Verifies that the firmware control allocation matches the simulator implementation.
ファームウェアの制御アロケーションがシミュレータの実装と一致することを検証。

This test compares:
1. Allocation matrix B and inverse B⁻¹
2. Thrust-to-duty conversion
3. Hover duty values

NOTE (2026-07-26): `control_allocation.hpp`/`.cpp` (the file names this test
originally referenced) belong to the frozen legacy `firmware/vehicle_old`
component `sf_algo_control`. The current mainline firmware `firmware/vehicle`
implements the same B⁻¹ math directly inside
`firmware/vehicle/components/sf_actuator/actuator.cpp`
(`mixerCompute()` for the allocation, `thrustToDuty()` for the motor curve;
there is no separate `buildMatrices()` — the B⁻¹ coefficients are inlined as
per-motor thrust formulas). References below point at that file.
NOTE(2026-07-26): このテストが元々参照していた `control_allocation.hpp`/`.cpp` は
凍結レガシー `firmware/vehicle_old` の `sf_algo_control` コンポーネントのもの。
現行主力ファーム `firmware/vehicle` は同じ B⁻¹ の数式を
`firmware/vehicle/components/sf_actuator/actuator.cpp` に直接実装している
（配分は `mixerCompute()`、モータ曲線は `thrustToDuty()`。独立した
`buildMatrices()` はなく、B⁻¹ の係数は各モータ推力の式に埋め込まれている）。
以下の参照はこのファイルを指す。

@see docs/architecture/control-allocation-migration.md
@see firmware/vehicle/components/sf_actuator/actuator.cpp (mixerCompute, thrustToDuty)
"""

import numpy as np
import sys
import os

# pytest is only needed for the test_* wrappers (xfail marker). Standalone
# execution (`python3 <this file>`) must keep working without pytest installed,
# so fall back to a no-op xfail decorator stub.
# pytest が要るのは test_* ラッパー（xfailマーカー）だけ。直接実行
# （`python3 <本ファイル>`）は pytest 未導入でも動く必要があるため、
# 無い場合は何もしない xfail デコレータのスタブに差し替える。
try:
    import pytest
except ImportError:
    class _PytestMarkStub:
        @staticmethod
        def xfail(*_args, **_kwargs):
            def _decorator(func):
                return func
            return _decorator

    class _PytestStub:
        mark = _PytestMarkStub()

    pytest = _PytestStub()

# Add simulator path — the `core` package this test needs (core.motors) lives
# under simulator/vpython/, not directly under simulator/. The old path
# (dirname(__file__)/..  == simulator/) was missing the vpython/ segment and
# broke `from core.motors import motor_prop` with ModuleNotFoundError.
# シミュレータのパスを追加 — 必要な `core` パッケージ (core.motors) の実体は
# simulator/ 直下ではなく simulator/vpython/ 配下にある。旧パス
# (dirname(__file__)/.. == simulator/) は vpython/ セグメントが抜けており
# `from core.motors import motor_prop` が ModuleNotFoundError になっていた。
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'vpython'))

# tools/sysid/_generated_params.py is the auto-generated SSOT for
# calibration-derived constants (kappa, arm length). Importing from there
# instead of re-hardcoding avoids yet another place these numbers can drift.
# tools/sysid/_generated_params.py は較正由来の定数（kappa・アーム長）の
# 自動生成 SSOT。ここから import することで値のドリフト元をこれ以上増やさない。
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'tools', 'sysid'))

from core.motors import motor_prop
from _generated_params import KAPPA_ADOPTED, EXPECTED_ARM


# =============================================================================
# Firmware Parameters (mirrors firmware/vehicle/components/sf_actuator/actuator.cpp)
# =============================================================================

class FirmwareParams:
    """Parameters matching firmware implementation (actuator.cpp)"""
    # X-quad geometry (ARM_D, motor layout) — actuator.cpp lines ~88-103
    d = EXPECTED_ARM  # Moment arm [m], == ARM_D
    kappa = KAPPA_ADOPTED  # Cq/Ct [m], == KAPPA (measured 2026-07-15)
    motor_x = [0.023, -0.023, -0.023, 0.023]  # M1:FR, M2:RR, M3:RL, M4:FL
    motor_y = [0.023, 0.023, -0.023, -0.023]
    motor_dir = [-1, 1, -1, 1]  # CCW, CW, CCW, CW
    max_thrust_per_motor = 0.15  # [N]

    # Motor curve coefficients: V = Am·ω² + Bm·ω + Cm (actuator.cpp MOTOR_AM/BM/CM).
    # These match simulator/vpython/core/motors.py's Am/Bm/Cm exactly (both are
    # fit to the same bench data), so they are NOT part of the known drift below.
    # モータ曲線係数: V = Am·ω² + Bm·ω + Cm（actuator.cpp の MOTOR_AM/BM/CM）。
    # simulator/vpython/core/motors.py の Am/Bm/Cm と完全一致（同じベンチデータで
    # フィット済み）のため、下記の既知のドリフトには含まれない。
    Am = 5.39e-8  # [V·s²/rad²]
    Bm = 6.33e-4  # [V·s/rad]
    Cm = 1.53e-2  # [V]

    # KNOWN DRIFT (2026-07-26): firmware's MOTOR_CT is still the pre-2026-07-15
    # value (1.00e-8). The 2026-07-15 sysid update replaced Ct with the bench-
    # measured 6.7e-9 (see simulator/vpython/core/motors.py) and updated KAPPA
    # in firmware at the same time, but did NOT update firmware's MOTOR_CT to
    # match — that update landed only in the simulator/sysid side. This test
    # deliberately mirrors firmware's *actual* (stale) value rather than the
    # updated plant value, per instruction: "firmware's real value is ground
    # truth for this compatibility test." See Test 4 for the resulting
    # firmware-vs-simulator duty discrepancy this produces at higher thrusts.
    # 既知のドリフト(2026-07-26): firmwareのMOTOR_CTは2026-07-15更新前の値
    # (1.00e-8)のまま。2026-07-15のsysid更新でCtはベンチ実測6.7e-9に置き換わり
    # (simulator/vpython/core/motors.py参照)、同時にKAPPAもfirmware側で更新
    # されたが、MOTOR_CTはfirmware側で追従更新されなかった（更新はシミュレータ/
    # sysid側だけに反映済み）。本テストは「firmwareの実値を正とする」という指示
    # に従い、更新済みのプラント値ではなくfirmwareの実際の（旧）値をそのまま
    # 鏡写しする。これにより高推力側でfirmware/シミュレータ間のduty差が生じる
    # 詳細はTest 4を参照。
    Ct = 1.0e-8  # [N/(rad/s)²], == MOTOR_CT (firmware actual value, stale)

    # Cq is NOT an independent firmware constant — actuator.cpp derives yaw
    # torque via kappa (T·kappa), not via a separate Cq. Deriving it here as
    # kappa*Ct (rather than hardcoding a third number) keeps kappa, Ct, Cq
    # mutually consistent and prevents silent drift between the three.
    # Cq はfirmware側の独立定数ではない — actuator.cppはヨートルクを別のCqでは
    # なくkappa経由(T·kappa)で扱う。ここでkappa*Ctとして導出する（3つ目の数値を
    # ハードコードしない）ことで、kappa・Ct・Cqの整合を保ち暗黙のドリフトを防ぐ。
    Cq = kappa * Ct  # [Nm/(rad/s)²], derived, == kappa*Ct

    Vbat = 3.7  # [V], == V_BATT_NOMINAL


def build_allocation_matrix_firmware():
    """
    Build allocation matrix B — the forward map implied by the B⁻¹
    coefficients firmware actuator.cpp:mixerCompute() uses (there is no
    explicit buildMatrices()/B in firmware; mixerCompute inlines B⁻¹ directly).

    B maps [T1, T2, T3, T4] -> [u_thrust, u_roll, u_pitch, u_yaw]
    """
    p = FirmwareParams()
    B = np.zeros((4, 4))

    for i in range(4):
        B[0, i] = 1.0  # Total thrust
        B[1, i] = -p.motor_y[i]  # Roll torque (around X-axis)
        B[2, i] = p.motor_x[i]   # Pitch torque (around Y-axis)
        B[3, i] = -p.motor_dir[i] * p.kappa  # Yaw torque

    return B


def build_inverse_matrix_firmware():
    """
    Build inverse matrix B⁻¹ as used in firmware
    actuator.cpp:mixerCompute() (per-motor thrust formulas
    T_i = 1/4*(u_t ± u_phi/d ± u_theta/d ± u_psi/kappa), one row per motor).

    B⁻¹ maps [u_thrust, u_roll, u_pitch, u_yaw] -> [T1, T2, T3, T4]
    """
    p = FirmwareParams()
    d = p.d
    kappa = p.kappa
    inv_d = 1.0 / d
    inv_kappa = 1.0 / kappa

    B_inv = np.zeros((4, 4))

    # M1 (FR): +x, +y, CCW (σ=-1)
    B_inv[0, 0] = 0.25
    B_inv[0, 1] = -0.25 * inv_d
    B_inv[0, 2] = 0.25 * inv_d
    B_inv[0, 3] = 0.25 * inv_kappa

    # M2 (RR): -x, +y, CW (σ=+1)
    B_inv[1, 0] = 0.25
    B_inv[1, 1] = -0.25 * inv_d
    B_inv[1, 2] = -0.25 * inv_d
    B_inv[1, 3] = -0.25 * inv_kappa

    # M3 (RL): -x, -y, CCW (σ=-1)
    B_inv[2, 0] = 0.25
    B_inv[2, 1] = 0.25 * inv_d
    B_inv[2, 2] = -0.25 * inv_d
    B_inv[2, 3] = 0.25 * inv_kappa

    # M4 (FL): +x, -y, CW (σ=+1)
    B_inv[3, 0] = 0.25
    B_inv[3, 1] = 0.25 * inv_d
    B_inv[3, 2] = 0.25 * inv_d
    B_inv[3, 3] = -0.25 * inv_kappa

    return B_inv


def thrust_to_omega(thrust: float, Ct: float) -> float:
    """Convert thrust [N] to angular velocity [rad/s]"""
    if thrust <= 0:
        return 0.0
    return np.sqrt(thrust / Ct)


def omega_to_voltage(omega: float, params: FirmwareParams) -> float:
    """
    Convert angular velocity to voltage using the motor curve polynomial.
    Mirrors firmware actuator.cpp:thrustToDuty()'s
    `volts = MOTOR_AM*omega*omega + MOTOR_BM*omega + MOTOR_CM` line exactly
    (no electrical model — firmware does not use Rm/Km/Dm/Qf at all).
    角速度から電圧への変換。firmware actuator.cpp:thrustToDuty() の
    `volts = MOTOR_AM*omega*omega + MOTOR_BM*omega + MOTOR_CM` をそのまま
    鏡写しする（電気モデルは不使用 — firmwareはRm/Km/Dm/Qfを一切使わない）。
    """
    p = params
    return p.Am * omega**2 + p.Bm * omega + p.Cm


def thrust_to_duty_firmware(thrust: float, params: FirmwareParams = None) -> float:
    """
    Convert thrust [N] to duty cycle [0-1]
    Matches firmware actuator.cpp:thrustToDuty() exactly:
        omega = sqrt(T / MOTOR_CT)
        volts = MOTOR_AM*omega^2 + MOTOR_BM*omega + MOTOR_CM
        duty  = volts / vbat, clamped to [MIN_DUTY, MAX_DUTY]
    firmware actuator.cpp:thrustToDuty() を厳密に鏡写しする。
    """
    if params is None:
        params = FirmwareParams()

    if thrust <= 0:
        return 0.0

    omega = thrust_to_omega(thrust, params.Ct)
    voltage = omega_to_voltage(omega, params)
    duty = voltage / params.Vbat

    return np.clip(duty, 0.0, 1.0)


def thrust_to_duty_simulator(thrust: float) -> float:
    """
    Convert thrust to duty using simulator's motor_prop class
    """
    motor = motor_prop(1)
    if thrust <= 0:
        return 0.0
    voltage = motor.equilibrium_voltage(thrust)
    duty = voltage / FirmwareParams.Vbat
    return np.clip(duty, 0.0, 1.0)


# =============================================================================
# Tests
# =============================================================================

def check_allocation_matrix():
    """Test 1: Verify allocation matrix B"""
    print("\n" + "="*60)
    print("Test 1: Allocation Matrix B")
    print("="*60)

    B = build_allocation_matrix_firmware()

    print("\nFirmware B matrix:")
    print(B)

    # Verify B × B⁻¹ = I
    B_inv = build_inverse_matrix_firmware()
    identity = B @ B_inv

    print("\nB × B⁻¹ (should be identity):")
    print(identity)

    is_identity = np.allclose(identity, np.eye(4), atol=1e-6)
    print(f"\n✓ B × B⁻¹ = I: {is_identity}")

    return is_identity


def check_inverse_matrix():
    """Test 2: Verify inverse matrix B⁻¹"""
    print("\n" + "="*60)
    print("Test 2: Inverse Matrix B⁻¹")
    print("="*60)

    B_inv = build_inverse_matrix_firmware()

    print("\nFirmware B⁻¹ matrix:")
    print(B_inv)

    # Check against direct matrix inverse
    B = build_allocation_matrix_firmware()
    B_inv_numpy = np.linalg.inv(B)

    print("\nNumPy inv(B):")
    print(B_inv_numpy)

    is_close = np.allclose(B_inv, B_inv_numpy, atol=1e-6)
    print(f"\n✓ Firmware B⁻¹ matches numpy inv(B): {is_close}")

    return is_close


def check_hover_mixing():
    """Test 3: Verify hover mixing (equal thrust distribution)"""
    print("\n" + "="*60)
    print("Test 3: Hover Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.035 * 9.81  # 35g × g = 0.343 N

    print(f"\nHover thrust (total): {hover_thrust:.4f} N")
    print(f"Hover thrust (per motor): {hover_thrust/4:.4f} N")

    # Control input for hover: [total_thrust, 0, 0, 0]
    control = np.array([hover_thrust, 0.0, 0.0, 0.0])

    # Mix to get motor thrusts
    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nMotor thrusts [T1, T2, T3, T4]: {thrusts}")

    # Verify equal distribution
    expected = hover_thrust / 4
    is_equal = np.allclose(thrusts, expected, atol=1e-6)
    print(f"\n✓ Equal thrust distribution: {is_equal}")

    return is_equal


def check_thrust_to_duty_comparison():
    """Test 4: Compare firmware and simulator thrust-to-duty conversion"""
    print("\n" + "="*60)
    print("Test 4: Thrust-to-Duty Comparison")
    print("="*60)

    test_thrusts = [0.0, 0.02, 0.05, 0.0857, 0.10, 0.15]

    print("\n  Thrust [N]   Firmware Duty   Simulator Duty   Difference")
    print("-" * 60)

    max_diff = 0.0
    for T in test_thrusts:
        duty_fw = thrust_to_duty_firmware(T)
        duty_sim = thrust_to_duty_simulator(T)
        diff = abs(duty_fw - duty_sim)
        max_diff = max(max_diff, diff)
        print(f"  {T:8.4f}      {duty_fw:8.4f}        {duty_sim:8.4f}        {diff:8.4f}")

    # Allow 5% difference due to parameter variations. This is intentionally
    # NOT loosened to force a PASS: the growing gap at higher thrust is the
    # real, known firmware MOTOR_CT=1.0e-8 vs simulator Ct=6.7e-9 mismatch
    # documented on FirmwareParams.Ct above (firmware missed the 2026-07-15
    # sysid Ct update that landed alongside the KAPPA update). A FAIL here is
    # this test correctly detecting that unresolved drift, not a test bug.
    # 5%許容。ここは無理にPASSさせるために緩めていない — 高推力側で広がる差は
    # 上のFirmwareParams.Ctに記載した実在の食い違い（firmware MOTOR_CT=1.0e-8 vs
    # シミュレータ Ct=6.7e-9。2026-07-15のKAPPA更新と同時のCt更新がfirmware側に
    # 反映されなかった）そのもの。ここでFAILするのはテストが未解消のドリフトを
    # 正しく検出している結果であり、テストの不具合ではない。
    is_close = max_diff < 0.05
    print(f"\n✓ Max difference < 5%: {is_close} (max={max_diff:.4f})")
    if not is_close:
        print(
            "  NOTE: This FAIL reflects a real firmware/plant parameter drift\n"
            "  (MOTOR_CT stale in firmware since the 2026-07-15 sysid update),\n"
            "  not a bug in this test. See FirmwareParams.Ct comment above and\n"
            "  firmware/vehicle/components/sf_actuator/actuator.cpp for details.\n"
            "  この FAIL は firmware/plant 間の実在するパラメータ食い違い\n"
            "  （2026-07-15のsysid更新でMOTOR_CTがfirmware側で追従更新されな\n"
            "  かった）を反映したもので、本テストの不具合ではない。詳細は上の\n"
            "  FirmwareParams.Ct コメントと actuator.cpp を参照。"
        )

    return is_close


def check_hover_duty():
    """Test 5: Verify hover duty cycle"""
    print("\n" + "="*60)
    print("Test 5: Hover Duty Verification")
    print("="*60)

    p = FirmwareParams()
    hover_thrust_total = 0.035 * 9.81  # 0.343 N
    hover_thrust_per_motor = hover_thrust_total / 4  # 0.0857 N

    duty_fw = thrust_to_duty_firmware(hover_thrust_per_motor)
    duty_sim = thrust_to_duty_simulator(hover_thrust_per_motor)

    # Expected: ~63% (calculated with full motor model)
    # Note: Documentation may have older value of 75% using simplified model
    expected_duty = 0.63

    print(f"\nHover thrust per motor: {hover_thrust_per_motor:.4f} N")
    print(f"Firmware duty: {duty_fw:.4f} ({duty_fw*100:.1f}%)")
    print(f"Simulator duty: {duty_sim:.4f} ({duty_sim*100:.1f}%)")
    print(f"Expected: {expected_duty:.4f} ({expected_duty*100:.1f}%)")

    # Allow 5% tolerance
    is_close = abs(duty_fw - expected_duty) < 0.05
    print(f"\n✓ Hover duty ≈ 63%: {is_close}")

    return is_close


def check_roll_torque():
    """Test 6: Verify roll torque mixing"""
    print("\n" + "="*60)
    print("Test 6: Roll Torque Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.343  # N
    roll_torque = 1e-4  # 0.1 mNm

    control = np.array([hover_thrust, roll_torque, 0.0, 0.0])

    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nControl input: thrust={hover_thrust:.3f}N, roll_torque={roll_torque*1000:.2f}mNm")
    print(f"Motor thrusts [T1, T2, T3, T4]: {thrusts}")

    # For positive roll torque (right wing down):
    # Left motors (M3, M4) should increase
    # Right motors (M1, M2) should decrease
    left_avg = (thrusts[2] + thrusts[3]) / 2  # M3, M4
    right_avg = (thrusts[0] + thrusts[1]) / 2  # M1, M2

    print(f"\nLeft motors avg: {left_avg:.4f} N")
    print(f"Right motors avg: {right_avg:.4f} N")
    print(f"Difference: {(left_avg - right_avg)*1000:.2f} mN")

    # Verify B × thrusts recovers original control
    B = build_allocation_matrix_firmware()
    recovered = B @ thrusts

    print(f"\nRecovered control: {recovered}")
    is_close = np.allclose(recovered, control, atol=1e-8)
    print(f"\n✓ B × T = u (round-trip): {is_close}")

    return is_close


def check_yaw_torque():
    """Test 7: Verify yaw torque mixing"""
    print("\n" + "="*60)
    print("Test 7: Yaw Torque Mixing")
    print("="*60)

    p = FirmwareParams()
    hover_thrust = 0.343  # N
    yaw_torque = 1e-5  # 0.01 mNm

    control = np.array([hover_thrust, 0.0, 0.0, yaw_torque])

    B_inv = build_inverse_matrix_firmware()
    thrusts = B_inv @ control

    print(f"\nControl input: thrust={hover_thrust:.3f}N, yaw_torque={yaw_torque*1000:.3f}mNm")
    print(f"Motor thrusts [T1, T2, T3, T4]: {thrusts}")

    # For positive yaw torque (nose right):
    # CCW motors (M1, M3) should increase (produce more CW reaction)
    # CW motors (M2, M4) should decrease
    ccw_avg = (thrusts[0] + thrusts[2]) / 2  # M1, M3
    cw_avg = (thrusts[1] + thrusts[3]) / 2   # M2, M4

    print(f"\nCCW motors (M1,M3) avg: {ccw_avg:.4f} N")
    print(f"CW motors (M2,M4) avg: {cw_avg:.4f} N")

    # Verify round-trip
    B = build_allocation_matrix_firmware()
    recovered = B @ thrusts

    is_close = np.allclose(recovered, control, atol=1e-8)
    print(f"\n✓ B × T = u (round-trip): {is_close}")

    return is_close


# =============================================================================
# Pytest wrappers
# =============================================================================
#
# The check_*() functions above are diagnostic-style: they print a report and
# return a bool. That is how this file is normally driven — as a standalone
# script via `if __name__ == "__main__"` (see run_all_tests() / bottom of
# file), not via pytest. Because they don't `assert`, collecting them
# directly under pytest with a `test_*` name only emits a
# PytestReturnNotNoneWarning and pytest reports every one of them as PASSED
# regardless of the returned bool — confirmed 2026-07-26: `pytest -v` on this
# file printed "7 passed" while check_thrust_to_duty_comparison() was, in
# fact, returning False. The thin wrappers below give pytest a real
# assertion to collect without touching the check_*() functions' print-heavy
# behavior that run_all_tests() depends on.
#
# 上の check_*() は診断向け（レポートを表示して bool を返す）。本ファイルを通常
# 駆動するのは `if __name__ == "__main__"` 経由の直接実行（run_all_tests() /
# ファイル末尾参照）であって pytest ではない。assert がないため、`test_*` の
# 名前でそのまま pytest に収集させると PytestReturnNotNoneWarning が出るだけで、
# 返り値の bool に関わらず全て PASSED 扱いになる — 2026-07-26 に確認済み:
# `pytest -v` はこのファイルに対し check_thrust_to_duty_comparison() が実際には
# False を返しているにもかかわらず "7 passed" と表示した。以下の薄いラッパーは
# run_all_tests() が依存する check_*() のprint主体の挙動に手を入れず、pytest に
# 本物のassertionを渡すためのもの。

def test_allocation_matrix():
    assert check_allocation_matrix()


def test_inverse_matrix():
    assert check_inverse_matrix()


def test_hover_mixing():
    assert check_hover_mixing()


@pytest.mark.xfail(
    strict=True,
    reason="firmware MOTOR_CT=1.0e-8 stale since the 2026-07-15 sysid Ct=6.7e-9 "
           "update; switch is deliberately deferred to backlog #3's firmware-side "
           "step with real-flight validation (simulation-policy.md §6, tracked as "
           "EXEMPT by `sf params check`). strict=True forces this marker to be "
           "removed the moment firmware catches up.",
)
def test_thrust_to_duty_comparison():
    # KNOWN drift (2026-07-26): real firmware/plant divergence, not a test bug —
    # same xfail philosophy as the SIL regression `xfail:` markers: the suite
    # stays green, the divergence stays visible in the xfail report, and
    # strict=True flips it to an error (forcing marker removal) once firmware's
    # MOTOR_CT is updated. See the FirmwareParams.Ct comment above.
    # 既知のドリフト(2026-07-26): firmware/plant間の実在する乖離であり、テストの
    # 不具合ではない。SIL回帰の `xfail:` マーカーと同じ運用哲学: スイートは緑を
    # 維持し、乖離は xfail 報告として可視のまま、firmware の MOTOR_CT が更新された
    # 瞬間に strict=True がエラー化してマーカー削除を強制する。上の
    # FirmwareParams.Ct コメント参照。
    assert check_thrust_to_duty_comparison(), (
        "firmware/plant Ct mismatch (firmware MOTOR_CT=1.0e-8 stale since "
        "2026-07-15 sysid update) -- see FirmwareParams.Ct comment"
    )


def test_hover_duty():
    assert check_hover_duty()


def test_roll_torque():
    assert check_roll_torque()


def test_yaw_torque():
    assert check_yaw_torque()


def run_all_tests():
    """Run all verification tests"""
    print("\n" + "#"*60)
    print("# Control Allocation Verification Tests")
    print("# 制御アロケーション検証テスト")
    print("#"*60)

    results = []

    results.append(("Allocation Matrix B", check_allocation_matrix()))
    results.append(("Inverse Matrix B⁻¹", check_inverse_matrix()))
    results.append(("Hover Mixing", check_hover_mixing()))
    results.append(("Thrust-to-Duty Comparison", check_thrust_to_duty_comparison()))
    results.append(("Hover Duty", check_hover_duty()))
    results.append(("Roll Torque Mixing", check_roll_torque()))
    results.append(("Yaw Torque Mixing", check_yaw_torque()))

    print("\n" + "="*60)
    print("Summary / サマリー")
    print("="*60)

    all_passed = True
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"  {status}: {name}")
        if not passed:
            all_passed = False

    print("\n" + "="*60)
    if all_passed:
        print("All tests passed! ファームウェアとシミュレータの実装が一致しています。")
    else:
        print("Some tests failed. Check implementation differences.")
    print("="*60)

    return all_passed


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
