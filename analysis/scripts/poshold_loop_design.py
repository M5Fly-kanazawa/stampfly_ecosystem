#!/usr/bin/env python3
"""
poshold_loop_design.py — identify the real POS_HOLD outer loop from the flight
log and design/compare damping options on a firmware-faithful closed-loop sim.

実機 POS_HOLD 外ループの同定 + ファーム忠実 sim による減衰補償の設計・比較。

Method:
  1. Firmware-exact discrete PID (pid.hpp replica): trapezoidal I (Ki=kp/ti),
     D-on-measurement incomplete derivative (eta=0.125), conditional-integration
     anti-windup, output_limit. Cascade pos-PI -> vel-PID -> tilt (clamp 10deg).
  2. Plant P: tilt_cmd -> measured horizontal pos = K * e^{-tau s} / s^2.
     K (= g * flow_scale * tilt_efficiency) and tau (att-loop + flow/ESKF lag)
     are calibrated so the CURRENT firmware gains reproduce the OBSERVED flight
     oscillation (omega ~ 0.68 rad/s, growing, sigma ~ +0.057 /s).
  3. Sweep damping options (vel-loop td, gain/integral changes); for each, fit
     the dominant closed-loop pole (omega, sigma -> zeta) and measure the hold
     deviation to a constant lateral disturbance. Robustness over K, tau ranges.
"""
import numpy as np

G = 9.80665
DT = 0.0025                 # 400 Hz control rate
MAX_POS_VEL = 1.0          # pos-loop output limit [m/s]
MAX_POS_TILT = 0.1745      # tilt clamp [rad] (10 deg)
VEL_OUT = G * MAX_POS_TILT  # vel-loop output limit [m/s^2] = 1.711

# Observed flight oscillation (driftE peaks -0.37@6.25, +0.50@11.0, -0.62@15.5)
OBS_OMEGA = 0.683          # rad/s (period 9.2 s)
OBS_SIGMA = 0.057          # /s   (growing: amp ratio ~1.30 per 4.6 s half-cycle)


class PID:
    """Exact replica of firmware pid.hpp PID::compute()."""
    def __init__(self, kp, ti, td=0.0, eta=0.125, output_limit=1.0):
        self.kp = kp; self.ti = ti; self.td = td; self.eta = eta
        self.output_limit = output_limit
        self.integral = 0.0; self.deriv_filter = 0.0
        self.prev_error = 0.0; self.prev_measurement = 0.0
        self.first_run = True

    def compute(self, setpoint, measurement, dt):
        if dt <= 0:
            return 0.0
        error = setpoint - measurement
        p_term = self.kp * error
        d_term = 0.0
        if self.td > 0:
            if self.first_run:
                self.prev_measurement = measurement
            else:
                alpha = 2.0 * self.eta * self.td / dt
                a = (alpha - 1.0) / (alpha + 1.0)
                b = 2.0 * self.td / ((alpha + 1.0) * dt)
                self.deriv_filter = a * self.deriv_filter - b * (measurement - self.prev_measurement)
                d_term = self.kp * self.deriv_filter
        self.prev_measurement = measurement
        self.first_run = False
        if self.ti >= 0.01:
            i_next = self.integral + (self.kp / self.ti) * (error + self.prev_error) * (dt * 0.5)
            out_test = p_term + i_next + d_term
            push_high = (out_test > self.output_limit) and (error > 0)
            push_low = (out_test < -self.output_limit) and (error < 0)
            if not (push_high or push_low):
                self.integral = i_next
            self.integral = max(-self.output_limit, min(self.output_limit, self.integral))
        self.prev_error = error
        out = p_term + self.integral + d_term
        return max(-self.output_limit, min(self.output_limit, out))


def simulate(pos_kp, pos_ti, pos_td, vel_kp, vel_ti, vel_td,
             K, tau, T=40.0, x0=0.3, dist_accel=0.0, dt=DT):
    """One-axis POS_HOLD closed loop. Returns (t, pos_err).

    Plant: tilt_cmd -> a = K*tilt_cmd(t-tau); v=int a; x=int v.  Setpoint x=0.
    x0 = initial position error [m]; dist_accel = constant lateral accel
    disturbance [m/s^2] (wind / flow bias / trim residual).
    """
    pos_pid = PID(pos_kp, pos_ti, pos_td, output_limit=MAX_POS_VEL)
    vel_pid = PID(vel_kp, vel_ti, vel_td, output_limit=VEL_OUT)
    n = int(T / dt)
    ndelay = int(round(tau / dt))
    x = x0; v = 0.0
    tilt_buf = [0.0] * (ndelay + 1)
    t_arr = np.empty(n); xe_arr = np.empty(n)
    for k in range(n):
        # cascade: pos error -> vel sp -> accel cmd -> tilt cmd (clamped)
        v_sp = pos_pid.compute(0.0, x, dt)
        a_cmd = vel_pid.compute(v_sp, v, dt)
        tilt_cmd = a_cmd / G
        tilt_cmd = max(-MAX_POS_TILT, min(MAX_POS_TILT, tilt_cmd))
        tilt_buf.append(tilt_cmd)
        tilt_del = tilt_buf.pop(0)
        # plant
        a = K * tilt_del + dist_accel
        v += a * dt
        x += v * dt
        t_arr[k] = k * dt; xe_arr[k] = x
    return t_arr, xe_arr


def fit_pole(t, x, t_skip=2.0):
    """Estimate dominant 2nd-order pole (omega, sigma) from zero-crossing
    spacing and successive-peak amplitude ratio of x(t)."""
    m = t >= t_skip
    t = t[m]; x = x[m]
    # peaks via sign change of derivative
    dx = np.diff(x)
    peaks = []
    for i in range(1, len(dx)):
        if dx[i - 1] > 0 and dx[i] <= 0:
            peaks.append((t[i], x[i]))
        elif dx[i - 1] < 0 and dx[i] >= 0:
            peaks.append((t[i], x[i]))
    if len(peaks) < 3:
        return None
    tp = np.array([p[0] for p in peaks]); ap = np.array([abs(p[1]) for p in peaks])
    halfper = np.median(np.diff(tp))
    omega = np.pi / halfper if halfper > 0 else float('nan')
    # sigma from log-amplitude slope of successive extrema
    good = ap > 1e-4
    if good.sum() >= 3:
        coef = np.polyfit(tp[good], np.log(ap[good]), 1)
        sigma = coef[0]
    else:
        sigma = float('nan')
    zeta = -sigma / np.hypot(sigma, omega) if np.isfinite(sigma) else float('nan')
    return omega, sigma, zeta


def metrics(gains, K, tau, label):
    """Pole + disturbance-hold for a gain set on plant (K, tau)."""
    pk, pi_, pt, vk, vi, vt = gains
    t, x = simulate(pk, pi_, pt, vk, vi, vt, K, tau, T=40, x0=0.3, dist_accel=0.0)
    pole = fit_pole(t, x)
    # constant disturbance hold (wind/flow bias 0.19 m/s^2): peak + settle
    td_, xd = simulate(pk, pi_, pt, vk, vi, vt, K, tau, T=40, x0=0.0, dist_accel=0.19)
    settle = xd[int(25 / DT):]            # 25-40 s steady portion
    peak_dev = np.max(np.abs(xd))
    ss_dev = np.mean(np.abs(settle))
    o, s, z = pole if pole else (float('nan'),) * 3
    return dict(label=label, omega=o, sigma=s, zeta=z,
                peak_dev=peak_dev, ss_dev=ss_dev, stable=(s < 0))


CURRENT = (1.0, 5.0, 0.0, 0.8, 2.0, 0.0)   # pos kp,ti,td ; vel kp,ti,td

# Plant-gain uncertainty: 3 independent estimates put the loop-relevant
# tilt->measured-velocity gain at ~0.25-0.66 x g. Design must be robust over it.
K_RANGE = [2.4, 4.0, 6.5, 9.81]
TAU_RANGE = [0.05, 0.15, 0.30]
VEL_NOISE = 0.08    # measured horizontal-velocity noise std [m/s] (from log)


def lin_pole(gains, K, tau):
    """Linear dominant pole from a SMALL initial offset (no saturation)."""
    pk, pi_, pt, vk, vi, vt = gains
    t, x = simulate(pk, pi_, pt, vk, vi, vt, K, tau, T=45, x0=0.03, dist_accel=0.0)
    if np.max(np.abs(x)) > 0.20:    # guard: stayed (mostly) linear
        pass
    return fit_pole(t, x)


def robust_worst_sigma(gains):
    worst = -9.0
    for K in K_RANGE:
        for tau in TAU_RANGE:
            p = lin_pole(gains, K, tau)
            if p and np.isfinite(p[1]):
                worst = max(worst, p[1])
    return worst


def hold_peak(gains, K, tau, dist=0.19):
    """Peak horizontal excursion to a constant lateral accel disturbance
    [m/s^2] (wind / flow bias). Steady state ~0 (position integrator)."""
    pk, pi_, pt, vk, vi, vt = gains
    _, xd = simulate(pk, pi_, pt, vk, vi, vt, K, tau, T=40, x0=0.0, dist_accel=dist)
    return float(np.max(np.abs(xd)))


def tilt_jitter_deg(gains):
    """RMS tilt jitter from velocity-measurement noise: kp_v*noise (P) plus a
    derivative term ~ kp_v*td*noise/dt_eff. Rough indoor-comfort proxy [deg]."""
    vk, vt = gains[3], gains[5]
    p = vk * VEL_NOISE / G
    d = vk * vt * VEL_NOISE / 0.05 / G if vt > 0 else 0.0   # eta filter ~0.05s effective
    return float(np.hypot(p, d) * 180 / np.pi)


if __name__ == "__main__":
    # ---- 1. calibrate (K, tau) so CURRENT gains reproduce the oscillation ----
    print("=== calibrate plant (K, tau) to observed flight oscillation ===")
    print(f"target: omega={OBS_OMEGA:.2f} rad/s, sigma={OBS_SIGMA:+.3f} /s (growing)\n")
    best = None
    for K in [2.4, 3.0, 4.0, 6.0]:
        for tau in np.arange(0.0, 0.40, 0.01):
            p = lin_pole(CURRENT, K, tau)
            if not p:
                continue
            o, s, _ = p
            err = abs(o - OBS_OMEGA) + 3 * abs(s - OBS_SIGMA)
            if best is None or err < best[0]:
                best = (err, K, tau, o, s)
    _, Kc, tauc, oc, sc = best
    print(f"nominal plant: K={Kc:.1f} ({Kc/G:.2f} g), tau={tauc*1000:.0f} ms -> "
          f"omega={oc:.3f}, sigma={sc:+.3f}  (matches observed)")
    print(f"sanity: CURRENT on K=g={G:.1f}: pole {lin_pole(CURRENT, G, tauc)}\n")

    # ---- 2. candidate compensators (named) ----
    cands = {
        "CURRENT  pos1.0/5  vel0.8/2  td0":   CURRENT,
        "soften pos0.5":                      (0.5, 5.0, 0.0, 0.8, 2.0, 0.0),
        "soften pos0.3":                      (0.3, 5.0, 0.0, 0.8, 2.0, 0.0),
        "soften pos0.2":                      (0.2, 5.0, 0.0, 0.8, 2.0, 0.0),
        "pos0.3 + vel1.6":                    (0.3, 5.0, 0.0, 1.6, 2.0, 0.0),
        "pos0.3 + vel1.6 + td0.3":            (0.3, 5.0, 0.0, 1.6, 2.0, 0.3),
        "pos0.3 + vel td0.4":                 (0.3, 5.0, 0.0, 0.8, 2.0, 0.4),
        "pos0.4 + vel1.2 + td0.3":            (0.4, 5.0, 0.0, 1.2, 2.0, 0.3),
        "pos0.25+ vel1.2 + td0.3":            (0.25, 5.0, 0.0, 1.2, 2.0, 0.3),
        "pos0.2 + vel1.6 + td0.3":            (0.2, 5.0, 0.0, 1.6, 2.0, 0.3),
    }
    print("=== candidates: pole on NOMINAL plant + robust worst-sigma + hold/noise ===")
    hdr = (f"{'design':<30}{'omega':>7}{'sigma':>8}{'zeta':>7}"
           f"{'worstSig':>9}{'peakDev':>8}{'jit_deg':>8}")
    print(hdr)
    for label, g in cands.items():
        p = lin_pole(g, Kc, tauc) or (float('nan'),) * 3
        ws = robust_worst_sigma(g)
        pk = hold_peak(g, Kc, tauc)
        jt = tilt_jitter_deg(g)
        flag = "  OK" if ws < -0.02 else ("  marg" if ws < 0 else "  UNSTABLE")
        print(f"{label:<30}{p[0]:7.3f}{p[1]:+8.3f}{p[2]:7.3f}"
              f"{ws:+9.3f}{pk:8.3f}{jt:8.2f}{flag}")

    # ---- 3. fine grid search for the most robust well-damped design ----
    print("\n=== grid search: minimize worst-case sigma (robust stability) ===")
    results = []
    for pk in [0.15, 0.2, 0.25, 0.3, 0.4, 0.5]:
        for vk in [0.8, 1.2, 1.6, 2.0]:
            for vt in [0.0, 0.2, 0.3, 0.4]:
                g = (pk, 5.0, 0.0, vk, 2.0, vt)
                ws = robust_worst_sigma(g)
                pkdev = hold_peak(g, Kc, tauc)
                jt = tilt_jitter_deg(g)
                results.append((ws, pkdev, jt, g))
    # keep robustly stable (worst sigma < -0.03), then sort by hold tightness
    stable = [r for r in results if r[0] < -0.03 and r[2] < 2.5]
    stable.sort(key=lambda r: r[1])
    print(f"top robustly-stable designs (worst_sig<-0.03, jitter<2.5deg), by hold:")
    print(f"{'pos_kp':>7}{'vel_kp':>7}{'vel_td':>7}{'worstSig':>9}{'peakDev_m':>10}{'jit_deg':>8}")
    for ws, pkdev, jt, g in stable[:8]:
        print(f"{g[0]:7.2f}{g[3]:7.2f}{g[5]:7.2f}{ws:+9.3f}{pkdev:10.3f}{jt:8.2f}")
