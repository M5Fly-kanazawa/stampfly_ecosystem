"""
audit_recalc.py
================
Independent spot-check recomputation for the code audit of yaw_cm_sim.py.

This script is written FRESH from the pid.hpp / pid_controller.cpp source and
the physical formulas in the task brief -- it does NOT import or call any
function from yaw_cm_sim.py (except reusing torque_budget.analyze_event() for
the tau_d(t) data source, and yawlib for raw log parsing, as directed by the
audit brief). The PID class, heading-hold law, seeding replay, and closed-loop
integrator below are independent re-implementations.

Target: event E2, scenarios S0 (cap=2.2mNm nominal) and S1 (cap=2.9mNm nominal).
Compares against /scratchpad/cm_sim_results.json.
"""
import sys
import numpy as np

SCRATCH = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"
sys.path.insert(0, SCRATCH)
import yawlib
import torque_budget as tb

R2D = 180.0 / np.pi

# ---- constants, independently transcribed from the same source files ----
IZZ = 20.4e-6
HOVER_THRUST_CORR = 1.12
KP = 1.901691e-3
TI = 0.8
TD = 0.01
ETA = 0.125
HH_KP = 3.0
HH_RATE_MAX = 2.0
DT = 0.0025
CAP_S0_NOM = 2.2e-3
CAP_S1_NOM = 2.9e-3


class PidIndep:
    """Independent re-implementation of sf::PID::compute (pid.hpp)."""
    def __init__(self, kp, ti, td, eta, out_lim):
        self.kp, self.ti, self.td, self.eta, self.out_lim = kp, ti, td, eta, out_lim
        self.I = 0.0
        self.Dz = 0.0
        self.e_prev = 0.0
        self.y_prev = 0.0
        self.first = True

    def step(self, sp, y, dt):
        if dt <= 0:
            return 0.0
        e = sp - y
        P = self.kp * e
        D = 0.0
        if self.td > 0:
            if self.first:
                self.y_prev = y
            else:
                alpha = 2.0 * self.eta * self.td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * self.td / ((alpha + 1.0) * dt)
                self.Dz = a * self.Dz - b * (y - self.y_prev)
                D = self.kp * self.Dz
        self.y_prev = y
        self.first = False

        if self.ti >= 0.01:
            I_next = self.I + (self.kp / self.ti) * (e + self.e_prev) * (0.5 * dt)
            test = P + I_next + D
            blocked = (test > self.out_lim and e > 0) or (test < -self.out_lim and e < 0)
            if not blocked:
                self.I = I_next
            self.I = min(max(self.I, -self.out_lim), self.out_lim)
        self.e_prev = e

        out = P + self.I + D
        return min(max(out, -self.out_lim), self.out_lim)

    def state(self):
        return (self.I, self.Dz, self.e_prev, self.y_prev, self.first)

    def load(self, st):
        self.I, self.Dz, self.e_prev, self.y_prev, self.first = st


def hh_cmd(tgt, psi, kp, rmax):
    err = tgt - psi
    err = (err + np.pi) % (2 * np.pi) - np.pi
    c = kp * err
    return min(max(c, -rmax), rmax)


ev = dict(name="E2", file="stampfly_udp_20260627T164611.jsonl", t0=110.0, t1=127.0)
data = tb.load_log(ev["file"])
imu = data["imu"]
rr_topic = data["rate_ref"]
ts_imu = imu["ts"]
r_full = imu["gyro"][:, 2] - imu["gyro_bias"][:, 2]
ts_rr = rr_topic["ts"]
rr_full = rr_topic["rate_ref"][:, 2]

t0, t1 = ev["t0"], ev["t1"]

# preroll: 10s back (E2 has no data-start clipping per cm_sim_results.json eff_t0==t0)
grid_start = t0 - 10.0
n_pre = int(round((t0 - grid_start) / DT)) + 1
t_pre = grid_start + np.arange(n_pre) * DT
r_pre = np.interp(t_pre, ts_imu, r_full)
rr_pre = np.interp(t_pre, ts_rr, rr_full)

pid_seed = PidIndep(KP, TI, TD, ETA, CAP_S0_NOM)
for i in range(n_pre):
    pid_seed.step(rr_pre[i], r_pre[i], DT)
seed_state = pid_seed.state()
print("Independent seed state at t0:", seed_state)

# --- reference tau_d(t), reused from torque_budget as directed ---
res = tb.analyze_event(ev)
t_cr = res["t"]
tau_d_raw = res["tau_d"]
nan_mask = np.isnan(tau_d_raw)
tau_d_filled = np.interp(t_cr, t_cr[~nan_mask], tau_d_raw[~nan_mask])

half_width = (res["ur_max_possible"] - res["ur_min_possible"]) / 2.0
envelope_real = float(np.nanmedian(half_width))
print("envelope_real (mNm):", envelope_real * 1e3)

n = int(round((t1 - t0) / DT)) + 1
t_sim = t0 + np.arange(n) * DT
tau_d_sim = np.interp(t_sim, t_cr, tau_d_filled)

r0 = float(np.interp(t0, ts_imu, r_full))
quat = imu["quat"]
psi_full = yawlib.quat_to_yaw(quat)
psi_unwrapped = np.unwrap(psi_full)
psi0 = float(np.interp(t0, ts_imu, psi_unwrapped))
print("psi0 (rad):", psi0, " r0 (rad/s):", r0)


def run_scenario(cap_nom, envelope, label):
    pid = PidIndep(KP, TI, TD, ETA, cap_nom)
    pid.load(seed_state)
    psi = np.zeros(n)
    r = np.zeros(n)
    psi[0] = psi0
    r[0] = r0
    psi_tgt = psi0
    for i in range(1, n):
        dt = t_sim[i] - t_sim[i - 1]
        rate_sp = hh_cmd(psi_tgt, psi[i - 1], HH_KP, HH_RATE_MAX)
        tau_nom = pid.step(rate_sp, r[i - 1], dt)
        tau_real = tau_nom / HOVER_THRUST_CORR
        cap_real = cap_nom / HOVER_THRUST_CORR
        tau_real = min(max(tau_real, -cap_real), cap_real)
        tau_applied = min(max(tau_real, -envelope), envelope)
        drdt = (tau_applied + tau_d_sim[i - 1]) / IZZ
        r[i] = r[i - 1] + drdt * dt
        psi[i] = psi[i - 1] + 0.5 * (r[i - 1] + r[i]) * dt
    dev_deg = (psi - psi_tgt) * R2D
    max_dev = float(np.max(np.abs(dev_deg)))
    idx = int(np.argmax(np.abs(dev_deg)))
    peak_t_rel = float(t_sim[idx] - t_sim[0])
    rec = None
    for i in range(idx, n):
        if abs(dev_deg[i]) < 10.0:
            rec = float(t_sim[i] - t_sim[0])
            break
    print(f"{label}: max_dev_deg={max_dev:.6f}  peak_t_rel_s={peak_t_rel:.6f}  recovery_time_s={rec}")
    return max_dev, peak_t_rel, rec


s0 = run_scenario(CAP_S0_NOM, envelope_real, "S0 (indep)")
s1 = run_scenario(CAP_S1_NOM, envelope_real, "S1 (indep)")

print("\nReference (cm_sim_results.json):")
print("  S0: max_dev_deg=59.516057237522446 peak_t_rel_s=12.415000000000006 recovery_time_s=13.2025")
print("  S1: max_dev_deg=15.62004934731358 peak_t_rel_s=14.762500000000003 recovery_time_s=15.037499999999994")
