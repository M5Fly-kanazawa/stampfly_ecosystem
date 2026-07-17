"""
roll_gain_study.py — log-driven roll-axis gain study (2026-07-17 flight)
Reconstruct the roll disturbance torque from the flight log, replay the
attitude+rate cascade in closed loop, and sweep gain candidates.
Physics in TRUE units (kappa-fix world). Plant: I=Ixx, motor lag 20 ms,
1-sample (2.5 ms) transport delay.

Caveat (stated in the report): tau_d lumps aero disturbance AND gyro noise;
higher D gains look better against lumped tau_d than against real sensor
noise, so torque HF activity is tracked as the noise-injection proxy.
"""
import json, sys
import numpy as np
from scipy.signal import savgol_filter

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
import torque_budget as tb
from yaw_cm_sim import PID, PID_ETA

LOG = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260717T211648.jsonl"
IXX = 9.16e-6
ARM_D = 0.023
CORR = 1.12
TAU_M = 0.02
DT = 0.0025
R2D = 180 / np.pi

# current defaults (params.cpp)
ATT = dict(kp=5.0, ti=2.0, td=0.04, lim=3.0)          # attitude loop -> rate sp [rad/s]
RATE0 = dict(kp=9.759795e-4, ti=0.7, td=0.01)          # rate loop -> torque [Nm] cmd
RATE_LIM = 5.2e-3                                       # cmd units

topics = {}
for line in open(LOG):
    d = json.loads(line)
    topics.setdefault(d["id"], []).append(d)
def arr(t, k): return np.array([d[k] for d in topics[t]])
def ts(t): return np.array([d["ts"] for d in topics[t]]) / 1e6

imu_t = ts("imu"); gyro = arr("imu", "gyro"); gb = arr("imu", "gyro_bias")
quat = arr("imu", "quat")
cr_t = ts("ctrl_ref"); duty = arr("ctrl_ref", "motor_duty")
ang_ref = arr("ctrl_ref", "angle_ref")
st_t = ts("status"); volt = arr("status", "voltage")

w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
roll_meas = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))

T0, T1 = 140.9, 192.9
m = (imu_t >= T0) & (imu_t <= T1)
t = imu_t[m]
p_meas = (gyro[m, 0] - gb[m, 0])
roll_m = roll_meas[m]

# --- realized roll torque from duty (true units) ---
vbat = np.interp(cr_t, st_t, volt)
T_true = tb.duty_to_thrust(duty, vbat[:, None], corr=CORR)   # real thrust [N]
tau_roll_real = ARM_D * ((T_true[:, 3] + T_true[:, 2]) - (T_true[:, 0] + T_true[:, 1]))
tau_at_t = np.interp(t, cr_t, tau_roll_real)

# --- pdot via Savitzky-Golay (clean 400 Hz, no gaps in this log) ---
fs = 1 / np.median(np.diff(t))
win = int(round(0.05 * fs)) | 1
pdot = savgol_filter(p_meas, win, 3, deriv=1, delta=1 / fs)

tau_d = IXX * pdot - tau_at_t

# --- inputs on sim grid ---
n = len(t)
angref = np.interp(t, cr_t, ang_ref[:, 0])

def band_rms(sig, lo, hi):
    N = len(sig); sp = np.fft.rfft(sig - sig.mean()); fr = np.fft.rfftfreq(N, 1 / fs)
    sp[(fr < lo) | (fr >= hi)] = 0
    return np.std(np.fft.irfft(sp, N))

def simulate(att_kp, r_kp, r_ti, r_td):
    att = PID(att_kp, ATT["ti"], ATT["td"], PID_ETA, output_limit=ATT["lim"])
    rate = PID(r_kp, r_ti, r_td, PID_ETA, output_limit=RATE_LIM)
    phi = np.zeros(n); p = np.zeros(n); tau_cmd_h = np.zeros(n)
    phi[0], p[0] = roll_m[0], p_meas[0]
    tau_motor = 0.0
    tau_delay = 0.0    # 1-sample transport delay
    for i in range(1, n):
        rate_sp = att.compute(angref[i - 1], phi[i - 1], DT)
        tau_c = rate.compute(rate_sp, p[i - 1], DT)
        tau_cmd_h[i] = tau_c
        # 1-sample delay then motor first-order lag; cmd->true torque = /CORR
        tau_motor += (tau_delay / CORR - tau_motor) * (DT / TAU_M)
        tau_delay = tau_c
        pd = (tau_motor + tau_d[i - 1]) / IXX
        p[i] = p[i - 1] + pd * DT
        phi[i] = phi[i - 1] + 0.5 * (p[i] + p[i - 1]) * DT
    return phi, p, tau_cmd_h

def metrics(phi, p, tau_h):
    return dict(
        ang_rms=np.std(phi) * R2D,
        ang_lf=band_rms(phi, 0.2, 0.8) * R2D,
        ang_mid=band_rms(phi, 0.8, 3.0) * R2D,
        ang_hf=band_rms(phi, 3.0, 8.0) * R2D,
        rate_rms=np.std(p) * R2D,
        tau_hf=band_rms(tau_h, 3.0, 20.0) * 1e3,   # mNm, noise-injection proxy
        tau_p99=np.percentile(np.abs(tau_h), 99) * 1e3,
    )

# baseline validation
phi0, p0, tau0 = simulate(ATT["kp"], RATE0["kp"], RATE0["ti"], RATE0["td"])
print("=== baseline validation (sim vs measured) ===")
print(f"  angle rms: sim {np.std(phi0)*R2D:.2f} deg  meas {np.std(roll_m)*R2D:.2f} deg")
print(f"  rate  rms: sim {np.std(p0)*R2D:.2f} deg/s meas {np.std(p_meas)*R2D:.2f} deg/s")
print(f"  angle corr(sim,meas) {np.corrcoef(phi0, roll_m)[0,1]:.3f}   "
      f"rate corr {np.corrcoef(p0, p_meas)[0,1]:.3f}")
mb = metrics(phi0, p0, tau0)
print(f"  baseline bands: LF {mb['ang_lf']:.2f}  mid {mb['ang_mid']:.2f}  HF {mb['ang_hf']:.2f} deg  "
      f"tau_HF {mb['tau_hf']:.3f} mNm")

# linear margin check for the rate loop: C(s)*G(s), G = e^-Ls /(I s (tau_m s+1)), L=2.5ms
def rate_margins(kp, ti, td, eta=PID_ETA, L=0.0025):
    w_ = np.logspace(0.3, 3.3, 4000)   # 2..2000 rad/s
    s = 1j * w_
    C = kp * (1 + 1 / (ti * s) + td * s / (eta * td * s + 1))
    G = np.exp(-L * s) / (IXX * s * (TAU_M * s + 1)) / CORR
    Lw = C * G
    mag = np.abs(Lw); ph = np.unwrap(np.angle(Lw))
    ic = np.argmin(np.abs(mag - 1))
    pm = (ph[ic] + np.pi) * R2D
    below = np.where(ph <= -np.pi)[0]
    gm = 1 / mag[below[0]] if len(below) else np.inf
    wc = w_[ic]
    return wc, pm, (20 * np.log10(gm) if np.isfinite(gm) else 99)

print("\n=== gain sweep (attitude+rate closed loop under measured tau_d) ===")
print(f"{'config':34s}{'angLF':>7s}{'angMID':>7s}{'angHF':>7s}{'angRMS':>7s}"
      f"{'rateRMS':>8s}{'tauHF':>7s}{'wc':>6s}{'PM':>6s}{'GM':>6s}")
cands = []
for att_kp in [5.0, 7.0, 9.0]:
    for kmul in [1.0, 1.3, 1.6]:
        for td in [0.01, 0.02, 0.03]:
            r_kp = RATE0["kp"] * kmul
            phi, p, tau_h = simulate(att_kp, r_kp, RATE0["ti"], td)
            mm = metrics(phi, p, tau_h)
            wc, pm, gm = rate_margins(r_kp, RATE0["ti"], td)
            tag = f"att{att_kp:.0f} kp x{kmul:.1f} td{td:.3f}"
            cands.append((tag, mm, wc, pm, gm))
            print(f"{tag:34s}{mm['ang_lf']:7.2f}{mm['ang_mid']:7.2f}{mm['ang_hf']:7.2f}"
                  f"{mm['ang_rms']:7.2f}{mm['rate_rms']:8.2f}{mm['tau_hf']:7.3f}"
                  f"{wc:6.0f}{pm:6.1f}{gm:6.1f}")
print("\n(PM in deg, GM in dB, wc rad/s; tauHF = 3-20Hz cmd-torque rms in mNm = noise proxy)")
