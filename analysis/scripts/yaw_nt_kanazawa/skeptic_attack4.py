"""
Attack 4: レートPIDが「負けた」のか、単にゲインが弱いだけで飽和は結果に過ぎないのか。
E1 (165713, [173.5,185.5]s) でPID出力を再現し、モータ飽和とPID出力飽和の順序を見る。
"""
import json
import numpy as np
import sys
sys.path.insert(0, "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad")
from yawlib import load_jsonl, quat_to_yaw, unwrap_deg

LOG = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T165713.jsonl"
T0, T1 = 173.5, 185.5

data = load_jsonl(LOG)
imu = data["imu"]
rr = data["rate_ref"]
cr = data["ctrl_ref"]

t_imu = imu["ts"]  # yawlib's 'ts' is already ts_us_raw/1e6 (no offset subtraction) —
                    # matches the [173.5,185.5]s window convention used in pass 1.
t0abs = 0.0

# align window
mask = (t_imu >= T0) & (t_imu <= T1)
idx = np.where(mask)[0]
print("n samples in window:", len(idx))

t = t_imu[idx]
gyro_z = imu["gyro"][idx, 2]
bias_z = imu["gyro_bias"][idx, 2]
meas_z = gyro_z - bias_z

t_rr = rr["ts"] - t0abs
# nearest-neighbor match rate_ref to imu timestamps (both ~400Hz, should be same cadence)
rr_idx = np.searchsorted(rr["ts"], imu["ts"][idx])
rr_idx = np.clip(rr_idx, 0, len(rr["ts"]) - 1)
sp_z = rr["rate_ref"][rr_idx, 2]

# gap-aware dt
ts_us = imu["ts_us_raw"][idx]
dt = np.diff(ts_us) / 1e6
dt = np.concatenate(([dt[0] if len(dt) else 0.0025], dt))
GAP_THRESH = 0.01  # 10ms -> treat as gap, reset integrator contribution for that step (use small dt fallback)

# PID params (from task background)
kp = 1.901691e-3
ti = 0.8
td = 0.01
eta = 0.125
output_limit = 2.2e-3

integral = 0.0
deriv_filter = 0.0
prev_error = 0.0
prev_measurement = meas_z[0]
first_run = True

tau_cmd = np.zeros(len(idx))
p_terms = np.zeros(len(idx))
i_terms = np.zeros(len(idx))
saturated = np.zeros(len(idx), dtype=bool)

for k in range(len(idx)):
    step_dt = dt[k]
    if step_dt <= 0 or step_dt > GAP_THRESH:
        # gap: skip integration this step, just prime D
        prev_measurement = meas_z[k]
        prev_error = sp_z[k] - meas_z[k]
        tau_cmd[k] = np.clip(kp * prev_error + integral, -output_limit, output_limit)
        p_terms[k] = kp * prev_error
        i_terms[k] = integral
        continue

    error = sp_z[k] - meas_z[k]
    p_term = kp * error

    if first_run:
        d_term = 0.0
        prev_measurement = meas_z[k]
        first_run = False
    else:
        alpha = 2.0 * eta * td / step_dt
        a = (alpha - 1.0) / (alpha + 1.0)
        b = 2.0 * td / ((alpha + 1.0) * step_dt)
        deriv_filter = a * deriv_filter - b * (meas_z[k] - prev_measurement)
        d_term = kp * deriv_filter
    prev_measurement = meas_z[k]

    if ti >= 0.01:
        i_next = integral + (kp / ti) * (error + prev_error) * (step_dt * 0.5)
        out_test = p_term + i_next + d_term
        push_high = (out_test > output_limit) and (error > 0)
        push_low = (out_test < -output_limit) and (error < 0)
        if not push_high and not push_low:
            integral = i_next
        integral = np.clip(integral, -output_limit, output_limit)

    prev_error = error
    output = p_term + integral + d_term
    sat = abs(output) >= output_limit * 0.999
    output = np.clip(output, -output_limit, output_limit)

    tau_cmd[k] = output
    p_terms[k] = p_term
    i_terms[k] = integral
    saturated[k] = sat

# duty data (nearest ctrl_ref)
cr_idx = np.searchsorted(cr["ts"], imu["ts"][idx])
cr_idx = np.clip(cr_idx, 0, len(cr["ts"]) - 1)
duty = cr["motor_duty"][cr_idx]  # (N,4): FR,RR,RL,FL
duty_max = np.max(duty, axis=1)
motor_sat = duty_max >= 0.95

print(f"\nWindow {T0}-{T1}s, n={len(idx)}")
print(f"tau_cmd range: [{tau_cmd.min():.3e}, {tau_cmd.max():.3e}]  output_limit={output_limit:.3e}")
print(f"fraction of samples with |tau_cmd| >= 0.999*limit (PID saturated): {np.mean(saturated)*100:.1f}%")
print(f"fraction of samples with max motor duty >= 0.95 (motor saturated): {np.mean(motor_sat)*100:.1f}%")

# find first PID-saturation time and first motor-saturation time
if np.any(saturated):
    t_pid_sat = t[np.argmax(saturated)]
    print(f"first PID output saturation at t={t_pid_sat:.3f}s (rel)")
else:
    print("PID output NEVER saturates in this window")

if np.any(motor_sat):
    t_motor_sat = t[np.argmax(motor_sat)]
    print(f"first motor duty>=0.95 at t={t_motor_sat:.3f}s (rel)")
else:
    print("motor NEVER saturates >=0.95 in this window")

# rate_ref clamp check
print(f"\nsp_z (rate_ref yaw) range: [{sp_z.min():.4f}, {sp_z.max():.4f}] rad/s")
print(f"fraction at +/-2.0 rad/s clamp (within 1%): {np.mean(np.abs(np.abs(sp_z)-2.0)<0.02)*100:.1f}%")

# gyro_z actual range
print(f"gyro_z (measured yaw rate) range: [{meas_z.min():.4f}, {meas_z.max():.4f}] rad/s = "
      f"[{np.degrees(meas_z.min()):.1f}, {np.degrees(meas_z.max()):.1f}] deg/s")

# Print a coarse timeline every ~0.5s: sp, meas, tau_cmd, duty_max
print("\n t(s)   sp_z(rad/s)  meas_z(rad/s)  tau_cmd(Nm)   dutymax  PIDsat  MOTORsat")
step = max(1, len(idx)//30)
for k in range(0, len(idx), step):
    print(f"{t[k]:6.2f}  {sp_z[k]:10.4f}  {meas_z[k]:10.4f}  {tau_cmd[k]:10.3e}  {duty_max[k]:7.3f}  {saturated[k]!s:5}  {motor_sat[k]!s:5}")
