"""
Attack 5: E0 (020050, ALT_HOLD, [32.9,40]s, -88 deg) の再解析。
飽和なしで-88度も回された機構をE1-E5と同じ枠組みで検証する。
"""
import sys
sys.path.insert(0, "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad")
from yawlib import load_jsonl, quat_to_yaw, unwrap_deg
import numpy as np

LOG = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260627T020050.jsonl"
T0, T1 = 32.9, 40.0

data = load_jsonl(LOG)
imu = data["imu"]
rr = data["rate_ref"]
cr = data["ctrl_ref"]
ctrl = data["ctrl"]

t = imu["ts"]
mask = (t >= T0 - 1) & (t <= T1 + 1)   # pad a bit for context
idx = np.where(mask)[0]
t_w = t[idx]

psi = quat_to_yaw(imu["quat"][idx])
psi_deg = unwrap_deg(psi)
gyro_z = np.degrees(imu["gyro"][idx, 2])

# mode check
cr_idx = np.clip(np.searchsorted(cr["ts"], t_w), 0, len(cr["ts"]) - 1)
mode = cr["mode"][cr_idx]
duty = cr["motor_duty"][cr_idx]
duty_max = np.max(duty, axis=1)
thrust = cr["total_thrust"][cr_idx]

rr_idx = np.clip(np.searchsorted(rr["ts"], t_w), 0, len(rr["ts"]) - 1)
sp_yaw = rr["rate_ref"][rr_idx, 2]

ctrl_idx = np.clip(np.searchsorted(ctrl["ts"], t_w), 0, len(ctrl["ts"]) - 1)
stick_yaw = ctrl["yaw"][ctrl_idx]

print("mode values in window:", np.unique(mode))
print(f"psi_deg range: {psi_deg.min():.1f} to {psi_deg.max():.1f}, "
      f"net change over [{T0},{T1}]: ", end="")
m2 = (t_w >= T0) & (t_w <= T1)
print(psi_deg[m2][-1] - psi_deg[m2][0])

print(f"\nduty_max range: [{duty_max.min():.3f}, {duty_max.max():.3f}]")
print(f"fraction duty_max>=0.95: {np.mean(duty_max[m2]>=0.95)*100:.1f}%")
print(f"total_thrust range: [{thrust.min():.4f}, {thrust.max():.4f}] N")
print(f"gyro_z range: [{gyro_z.min():.1f}, {gyro_z.max():.1f}] deg/s")
print(f"sp_yaw (rate_ref) range: [{sp_yaw.min():.3f}, {sp_yaw.max():.3f}] rad/s")
print(f"stick_yaw range: [{np.nanmin(stick_yaw):.3f}, {np.nanmax(stick_yaw):.3f}]")

print(f"\n{'t(s)':>7} {'psi_deg':>9} {'gyro_z':>8} {'sp_yaw':>8} {'stick_yaw':>9} "
      f"{'thrust':>8} {'dFR':>6} {'dRR':>6} {'dRL':>6} {'dFL':>6} {'dmax':>6} mode")
step = max(1, len(idx)//60)
for k in range(0, len(idx), step):
    d = duty[k]
    print(f"{t_w[k]:7.3f} {psi_deg[k]:9.1f} {gyro_z[k]:8.1f} {sp_yaw[k]:8.3f} "
          f"{stick_yaw[k]:9.3f} {thrust[k]:8.4f} {d[0]:6.3f} {d[1]:6.3f} {d[2]:6.3f} "
          f"{d[3]:6.3f} {duty_max[k]:6.3f} {int(mode[k])}")
