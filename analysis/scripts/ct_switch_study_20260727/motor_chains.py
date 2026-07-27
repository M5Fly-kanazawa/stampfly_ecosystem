#!/usr/bin/env python3
"""
Motor V-omega-thrust chains: current firmware (OLD) vs. measured-parameter
proposal (NEW). Both expose:
  - forward:  omega(thrust) -> V(omega) -> duty(V, vbat)
  - inverse:  duty, vbat -> V -> omega -> thrust   (used on real log data)

All constants are either literal firmware values (OLD, copy-pasted from
firmware/vehicle/components/sf_actuator/actuator.cpp so a firmware diff shows
up here too) or derived from the 2026-07-15 bench-measured primitives (NEW),
so the NEW numbers are auditable back to Rm/Km/Cq/Qf/Ct rather than trusting
the rounded literals quoted in the task brief.

モータ V-ω-推力チェーン: 現行ファーム(OLD) vs 実測パラメータ提案(NEW)。
NEW は丸めた文字通りの定数を信用せず、実測の一次量(Rm/Km/Cq/Qf/Ct)から
毎回導出することで、値の出どころを監査可能にしている。
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class MotorChain:
    name: str
    Ct: float   # thrust T = Ct * omega^2                [N/(rad/s)^2]
    Am: float   # V = Am*omega^2 + Bm*omega + Cm          [V*s^2/rad^2]
    Bm: float   #                                          [V*s/rad]
    Cm: float   #                                          [V]

    def omega_of_thrust(self, thrust):
        thrust = np.maximum(thrust, 0.0)
        return np.sqrt(thrust / self.Ct)

    def voltage_of_omega(self, omega):
        return self.Am * omega**2 + self.Bm * omega + self.Cm

    def duty_of_thrust(self, thrust, vbat):
        """Forward chain: thrust [N] -> duty [0,1] (unclamped)."""
        omega = self.omega_of_thrust(thrust)
        v = self.voltage_of_omega(omega)
        return v / vbat

    def thrust_of_duty(self, duty, vbat):
        """Inverse chain: measured (duty, vbat) -> implied per-motor thrust [N].

        V = duty * vbat (motor terminal volts, matches actuator.cpp's
        duty = V / vbat exactly). Solve Am*w^2 + Bm*w + (Cm - V) = 0 for the
        positive root, clip to omega>=0 (a duty at/below the friction
        threshold Cm implies the motor is not really spinning yet).
        """
        duty = np.asarray(duty, dtype=float)
        vbat = np.asarray(vbat, dtype=float)
        v = duty * vbat
        disc = self.Bm**2 - 4.0 * self.Am * (self.Cm - v)
        disc = np.maximum(disc, 0.0)
        omega = (-self.Bm + np.sqrt(disc)) / (2.0 * self.Am)
        omega = np.maximum(omega, 0.0)
        return self.Ct * omega**2


# =============================================================================
# OLD chain — literal copy of firmware/vehicle/components/sf_actuator/actuator.cpp
# (constants MOTOR_CT, MOTOR_AM, MOTOR_BM, MOTOR_CM), read 2026-07 from the
# checked-in source. This is what actually produced every motor_duty sample
# in the logs (given the commanded total thrust hover_thrust_+correction).
# =============================================================================
OLD_CHAIN = MotorChain(
    name="old_firmware",
    Ct=1.00e-8,
    Am=5.39e-8,
    Bm=6.33e-4,
    Cm=1.53e-2,
)

# =============================================================================
# NEW chain — derived from the 2026-07-15 bench measurements quoted in the task
# brief (thrust stand + coast-down + DC motor bench):
#   Ct = 6.7e-9 N/(rad/s)^2       (thrust stand)
#   Cq = 4.10e-11 Nm/(rad/s)^2    (coast-down)
#   Km = 5.682e-4 V*s/rad         (back-EMF constant)
#   Rm = 0.593 Ohm                (winding resistance)
#   Dm ~= 0                       (viscous damping, negligible)
#   Qf = 9.507e-6 Nm              (static/Coulomb friction torque)
# V-omega curve (steady state, viscous term Dm*Rm/Km ~= 0):
#   V = (Rm*Cq/Km) * omega^2  +  (Km + Rm*Dm/Km) * omega  +  Rm*Qf/Km
#     = Am_new * omega^2 + Bm_new * omega + Cm_new
# =============================================================================
_Ct_new = 6.7e-9
_Cq_new = 4.10e-11
_Km_new = 5.682e-4
_Rm_new = 0.593
_Dm_new = 0.0
_Qf_new = 9.507e-6

_Am_new = _Rm_new * _Cq_new / _Km_new
_Bm_new = _Km_new + _Rm_new * _Dm_new / _Km_new
_Cm_new = _Rm_new * _Qf_new / _Km_new

NEW_CHAIN = MotorChain(
    name="new_measured",
    Ct=_Ct_new,
    Am=_Am_new,
    Bm=_Bm_new,
    Cm=_Cm_new,
)

# Sanity-check against the rounded literals quoted in the task brief (must
# agree to <1%, otherwise the derivation above has a bug).
_BRIEF_Am, _BRIEF_Bm, _BRIEF_Cm = 4.279e-8, 5.682e-4, 9.92e-3
assert abs(NEW_CHAIN.Am - _BRIEF_Am) / _BRIEF_Am < 0.01, (NEW_CHAIN.Am, _BRIEF_Am)
assert abs(NEW_CHAIN.Bm - _BRIEF_Bm) / _BRIEF_Bm < 0.01, (NEW_CHAIN.Bm, _BRIEF_Bm)
assert abs(NEW_CHAIN.Cm - _BRIEF_Cm) / _BRIEF_Cm < 0.01, (NEW_CHAIN.Cm, _BRIEF_Cm)

# =============================================================================
# Vehicle physical constants
# =============================================================================
GRAVITY = 9.80665              # [m/s^2], matches pid_controller.hpp kMassG derivation
MASS_KG_NOMINAL = 0.037        # [kg], firmware kMassG assumption (task brief formula)
MASS_KG_MEASURED = 0.0368      # [kg], 36.8 g bench-measured airframe mass
HOVER_THRUST_N = MASS_KG_NOMINAL * GRAVITY          # required TOTAL thrust at hover [N]
HOVER_THRUST_PER_MOTOR_N = HOVER_THRUST_N / 4.0


if __name__ == "__main__":
    print(f"OLD  chain: Ct={OLD_CHAIN.Ct:.3e} Am={OLD_CHAIN.Am:.3e} "
          f"Bm={OLD_CHAIN.Bm:.3e} Cm={OLD_CHAIN.Cm:.3e}")
    print(f"NEW  chain: Ct={NEW_CHAIN.Ct:.3e} Am={NEW_CHAIN.Am:.3e} "
          f"Bm={NEW_CHAIN.Bm:.3e} Cm={NEW_CHAIN.Cm:.3e}")
    print(f"Hover thrust (mass={MASS_KG_NOMINAL}kg): total={HOVER_THRUST_N:.4f} N, "
          f"per-motor={HOVER_THRUST_PER_MOTOR_N:.4f} N")

    # Round-trip sanity check: forward then inverse should return the input thrust.
    for chain in (OLD_CHAIN, NEW_CHAIN):
        t = HOVER_THRUST_PER_MOTOR_N
        vbat = 3.9
        d = chain.duty_of_thrust(t, vbat)
        t_back = chain.thrust_of_duty(d, vbat)
        print(f"{chain.name}: duty@hover/motor={d:.4f}  round-trip thrust "
              f"{t:.5f} -> {t_back:.5f} N (rel err {(t_back-t)/t:+.2e})")
