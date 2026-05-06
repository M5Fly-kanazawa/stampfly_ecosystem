#!/usr/bin/env python3
"""
Crash sensor analysis for StampFly flight log.
Analyzes baro, tof_b, tof_f, flow, mag packets and packet timing.
"""

import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from collections import defaultdict

LOG_FILE = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs/stampfly_udp_20260405T140434.jsonl"
T0 = 42462547  # first timestamp (us)
CRASH_TIME = 21.3  # seconds from start

# --- Load data ---
data = defaultdict(list)
with open(LOG_FILE) as f:
    for line in f:
        pkt = json.loads(line)
        data[pkt['id']].append(pkt)

def ts_sec(pkt):
    return (pkt['ts'] - T0) / 1e6

# ============================================================
# 1. Barometer: altitude and pressure
# ============================================================
fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
baro = data['baro']
bt = [ts_sec(p) for p in baro]
alt = [p['altitude'] for p in baro]
pres = [p['pressure'] for p in baro]

axes[0].plot(bt, alt, 'b-', linewidth=0.8)
axes[0].axvline(CRASH_TIME, color='r', linestyle='--', label=f'Crash ~{CRASH_TIME}s')
axes[0].set_ylabel('Altitude (m)')
axes[0].set_title('Barometer - Altitude')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(bt, pres, 'g-', linewidth=0.8)
axes[1].axvline(CRASH_TIME, color='r', linestyle='--')
axes[1].set_ylabel('Pressure (Pa)')
axes[1].set_xlabel('Time (s)')
axes[1].set_title('Barometer - Pressure')
axes[1].grid(True, alpha=0.3)

fig.suptitle('Crash Sensor Analysis: Barometer', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_baro.png', dpi=150)
plt.close()

# Compute altitude stats in last 5 seconds
baro_last5 = [(t, a) for t, a in zip(bt, alt) if t > CRASH_TIME - 5]
if baro_last5:
    alt_last5 = [a for _, a in baro_last5]
    print(f"\n=== BARO (last 5s before crash) ===")
    print(f"  Alt range: {min(alt_last5):.2f} - {max(alt_last5):.2f} m")
    print(f"  Alt change: {alt_last5[-1] - alt_last5[0]:.3f} m")
    print(f"  Last alt: {alt_last5[-1]:.2f} m at t={baro_last5[-1][0]:.3f}s")

# ============================================================
# 2. ToF Bottom: flight altitude
# ============================================================
fig, ax = plt.subplots(figsize=(14, 5))
tof_b = data['tof_b']
tb_t = [ts_sec(p) for p in tof_b]
tb_d = [p['distance'] for p in tof_b]
tb_s = [p['status'] for p in tof_b]

ax.plot(tb_t, tb_d, 'b-', linewidth=0.8, label='Distance')
# Mark bad status
bad_idx = [i for i, s in enumerate(tb_s) if s != 0]
if bad_idx:
    ax.scatter([tb_t[i] for i in bad_idx], [tb_d[i] for i in bad_idx],
               c='red', s=10, zorder=5, label=f'Bad status ({len(bad_idx)})')
ax.axvline(CRASH_TIME, color='r', linestyle='--', label=f'Crash ~{CRASH_TIME}s')
ax.set_ylabel('Distance (m)')
ax.set_xlabel('Time (s)')
ax.set_title('ToF Bottom - Flight Altitude')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_tof_bottom.png', dpi=150)
plt.close()

# Last 5 seconds analysis
tof_b_last5 = [(t, d, s) for t, d, s in zip(tb_t, tb_d, tb_s) if t > CRASH_TIME - 5]
if tof_b_last5:
    print(f"\n=== TOF BOTTOM (last 5s) ===")
    print(f"  Distance range: {min(d for _,d,_ in tof_b_last5):.3f} - {max(d for _,d,_ in tof_b_last5):.3f} m")
    print(f"  Last 5 readings:")
    for t, d, s in tof_b_last5[-5:]:
        print(f"    t={t:.3f}s  dist={d:.3f}m  status={s}")

# ============================================================
# 3. ToF Front: obstacle distance
# ============================================================
fig, ax = plt.subplots(figsize=(14, 5))
tof_f = data['tof_f']
tf_t = [ts_sec(p) for p in tof_f]
tf_d = [p['distance'] for p in tof_f]
tf_s = [p['status'] for p in tof_f]

ax.plot(tf_t, tf_d, 'r-', linewidth=0.8, label='Distance')
bad_idx_f = [i for i, s in enumerate(tf_s) if s != 0]
if bad_idx_f:
    ax.scatter([tf_t[i] for i in bad_idx_f], [tf_d[i] for i in bad_idx_f],
               c='orange', s=10, zorder=5, label=f'Bad status ({len(bad_idx_f)})')
ax.axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5, label=f'Crash ~{CRASH_TIME}s')
ax.set_ylabel('Distance (m)')
ax.set_xlabel('Time (s)')
ax.set_title('ToF Front - Obstacle Distance (Wall Approach)')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_tof_front.png', dpi=150)
plt.close()

# Detailed front ToF analysis
print(f"\n=== TOF FRONT (wall approach analysis) ===")
print(f"  Total packets: {len(tof_f)}")
print(f"  Bad status packets: {len(bad_idx_f)}")

# Find when distance starts decreasing (approaching wall)
# Look at last 10 seconds
tof_f_late = [(t, d, s) for t, d, s in zip(tf_t, tf_d, tf_s) if t > CRASH_TIME - 10]
if tof_f_late:
    print(f"\n  Last 10s front ToF readings ({len(tof_f_late)} packets):")
    # Find minimum distance and when approach started
    min_d = min(d for _, d, _ in tof_f_late)
    max_d = max(d for _, d, _ in tof_f_late)
    print(f"  Distance range: {min_d:.3f} - {max_d:.3f} m")

    # Print last 10 readings
    print(f"\n  Last 10 readings (approach/impact):")
    for t, d, s in tof_f_late[-10:]:
        print(f"    t={t:.3f}s  dist={d:.3f}m  status={s}")

    # Find when distance started consistently decreasing
    # Use a simple rolling check
    dists_late = [d for _, d, _ in tof_f_late]
    times_late = [t for t, _, _ in tof_f_late]
    for i in range(len(dists_late) - 5):
        if all(dists_late[i+j] > dists_late[i+j+1] for j in range(4)):
            print(f"\n  Wall approach started at ~t={times_late[i]:.3f}s (dist={dists_late[i]:.3f}m)")
            print(f"  Time before crash: {CRASH_TIME - times_late[i]:.1f}s")
            break

# ============================================================
# 4. Optical Flow
# ============================================================
fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
flow = data['flow']
fl_t = [ts_sec(p) for p in flow]
fl_dx = [p['dx'] for p in flow]
fl_dy = [p['dy'] for p in flow]
fl_q = [p['quality'] for p in flow]

axes[0].plot(fl_t, fl_dx, 'b-', linewidth=0.5, alpha=0.7)
axes[0].axvline(CRASH_TIME, color='r', linestyle='--')
axes[0].set_ylabel('dx (pixels)')
axes[0].set_title('Optical Flow - X displacement')
axes[0].grid(True, alpha=0.3)

axes[1].plot(fl_t, fl_dy, 'g-', linewidth=0.5, alpha=0.7)
axes[1].axvline(CRASH_TIME, color='r', linestyle='--')
axes[1].set_ylabel('dy (pixels)')
axes[1].set_title('Optical Flow - Y displacement')
axes[1].grid(True, alpha=0.3)

axes[2].plot(fl_t, fl_q, 'purple', linewidth=0.5, alpha=0.7)
axes[2].axvline(CRASH_TIME, color='r', linestyle='--')
axes[2].set_ylabel('Quality')
axes[2].set_xlabel('Time (s)')
axes[2].set_title('Optical Flow - Quality')
axes[2].grid(True, alpha=0.3)

fig.suptitle('Crash Sensor Analysis: Optical Flow', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_flow.png', dpi=150)
plt.close()

# Flow analysis in last 5 seconds
flow_last5 = [(t, dx, dy, q) for t, dx, dy, q in zip(fl_t, fl_dx, fl_dy, fl_q) if t > CRASH_TIME - 5]
if flow_last5:
    print(f"\n=== OPTICAL FLOW (last 5s) ===")
    dxs = [dx for _, dx, _, _ in flow_last5]
    dys = [dy for _, _, dy, _ in flow_last5]
    qs = [q for _, _, _, q in flow_last5]
    print(f"  dx range: {min(dxs)} - {max(dxs)}")
    print(f"  dy range: {min(dys)} - {max(dys)}")
    print(f"  quality range: {min(qs)} - {max(qs)}")
    # Peak flow magnitude
    mags = [abs(dx) + abs(dy) for _, dx, dy, _ in flow_last5]
    peak_idx = np.argmax(mags)
    t_peak, dx_peak, dy_peak, q_peak = flow_last5[peak_idx]
    print(f"  Peak flow at t={t_peak:.3f}s: dx={dx_peak}, dy={dy_peak}, q={q_peak}")

    # Last 5 readings
    print(f"\n  Last 5 readings:")
    for t, dx, dy, q in flow_last5[-5:]:
        print(f"    t={t:.3f}s  dx={dx}  dy={dy}  quality={q}")

# ============================================================
# 5. Magnetometer
# ============================================================
fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
mag = data['mag']
mg_t = [ts_sec(p) for p in mag]
mg_x = [p['x'] for p in mag]
mg_y = [p['y'] for p in mag]
mg_z = [p['z'] for p in mag]
mg_mag = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(mg_x, mg_y, mg_z)]

axes[0].plot(mg_t, mg_x, 'r-', linewidth=0.8, label='X')
axes[0].plot(mg_t, mg_y, 'g-', linewidth=0.8, label='Y')
axes[0].plot(mg_t, mg_z, 'b-', linewidth=0.8, label='Z')
axes[0].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[0].set_ylabel('Field (uT)')
axes[0].set_title('Magnetometer - Individual Axes')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

axes[1].plot(mg_t, mg_mag, 'k-', linewidth=0.8)
axes[1].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5, label=f'Crash ~{CRASH_TIME}s')
axes[1].set_ylabel('|B| (uT)')
axes[1].set_xlabel('Time (s)')
axes[1].set_title('Magnetometer - Field Magnitude')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

fig.suptitle('Crash Sensor Analysis: Magnetometer', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_mag.png', dpi=150)
plt.close()

# Mag analysis
print(f"\n=== MAGNETOMETER ===")
print(f"  Total magnitude range: {min(mg_mag):.1f} - {max(mg_mag):.1f} uT")
mag_last5 = [(t, m) for t, m in zip(mg_t, mg_mag) if t > CRASH_TIME - 5]
if mag_last5:
    mags5 = [m for _, m in mag_last5]
    print(f"  Last 5s magnitude range: {min(mags5):.1f} - {max(mags5):.1f} uT")
    # Check for sudden changes (interference)
    mag_diffs = [abs(mags5[i+1] - mags5[i]) for i in range(len(mags5)-1)]
    if mag_diffs:
        print(f"  Max mag change between samples (last 5s): {max(mag_diffs):.2f} uT")

# ============================================================
# 6. Packet timing analysis (last 5 seconds)
# ============================================================
print(f"\n=== PACKET TIMING (last 5s before crash) ===")
sensor_types = ['imu', 'baro', 'tof_b', 'tof_f', 'flow', 'mag', 'ctrl', 'posvel']

fig, ax = plt.subplots(figsize=(14, 6))
colors = plt.cm.tab10(np.linspace(0, 1, len(sensor_types)))

for idx, sid in enumerate(sensor_types):
    pkts = data[sid]
    times = [ts_sec(p) for p in pkts]
    times_last5 = [t for t in times if t > CRASH_TIME - 5]
    if times_last5:
        ax.scatter(times_last5, [idx] * len(times_last5), c=[colors[idx]], s=2, label=sid)
        # Find gaps
        gaps = [(times_last5[i+1] - times_last5[i]) for i in range(len(times_last5)-1)]
        if gaps:
            avg_gap = np.mean(gaps)
            max_gap = max(gaps)
            max_gap_idx = np.argmax(gaps)
            max_gap_time = times_last5[max_gap_idx]
            print(f"  {sid:10s}: {len(times_last5):4d} pkts, avg interval={avg_gap*1000:.1f}ms, "
                  f"max gap={max_gap*1000:.1f}ms at t={max_gap_time:.3f}s")
            # Report last packet time
            print(f"             last packet at t={times_last5[-1]:.3f}s "
                  f"({CRASH_TIME - times_last5[-1]:.3f}s before crash)")

ax.axvline(CRASH_TIME, color='r', linestyle='--', label=f'Crash ~{CRASH_TIME}s')
ax.set_yticks(range(len(sensor_types)))
ax.set_yticklabels(sensor_types)
ax.set_xlabel('Time (s)')
ax.set_title('Packet Timing - Last 5 Seconds')
ax.legend(loc='upper left')
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_timing.png', dpi=150)
plt.close()

# ============================================================
# 7. Combined Timeline
# ============================================================
fig, axes = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

# Last 10 seconds for detailed view
t_start = CRASH_TIME - 10

# ToF Front
tf_sel = [(t, d) for t, d in zip(tf_t, tf_d) if t > t_start]
axes[0].plot([t for t, _ in tf_sel], [d for _, d in tf_sel], 'r-o', markersize=2, linewidth=0.8)
axes[0].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[0].set_ylabel('Front ToF (m)')
axes[0].set_title('Combined Timeline - Last 10 Seconds Before Crash')
axes[0].grid(True, alpha=0.3)

# ToF Bottom
tb_sel = [(t, d) for t, d in zip(tb_t, tb_d) if t > t_start]
axes[1].plot([t for t, _ in tb_sel], [d for _, d in tb_sel], 'b-o', markersize=2, linewidth=0.8)
axes[1].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[1].set_ylabel('Bottom ToF (m)')
axes[1].grid(True, alpha=0.3)

# Baro altitude
baro_sel = [(t, a) for t, a in zip(bt, alt) if t > t_start]
axes[2].plot([t for t, _ in baro_sel], [a for _, a in baro_sel], 'g-', linewidth=0.8)
axes[2].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[2].set_ylabel('Baro Alt (m)')
axes[2].grid(True, alpha=0.3)

# Flow magnitude
fl_mag = [abs(dx) + abs(dy) for dx, dy in zip(fl_dx, fl_dy)]
fl_sel = [(t, m, q) for t, m, q in zip(fl_t, fl_mag, fl_q) if t > t_start]
axes[3].plot([t for t, _, _ in fl_sel], [m for _, m, _ in fl_sel], 'purple', linewidth=0.5, alpha=0.7, label='|flow|')
ax3b = axes[3].twinx()
ax3b.plot([t for t, _, _ in fl_sel], [q for _, _, q in fl_sel], 'orange', linewidth=0.5, alpha=0.5, label='quality')
ax3b.set_ylabel('Quality', color='orange')
axes[3].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[3].set_ylabel('Flow |dx|+|dy|')
axes[3].grid(True, alpha=0.3)

# Mag magnitude
mg_sel = [(t, m) for t, m in zip(mg_t, mg_mag) if t > t_start]
axes[4].plot([t for t, _ in mg_sel], [m for _, m in mg_sel], 'k-', linewidth=0.8)
axes[4].axvline(CRASH_TIME, color='r', linestyle='--', alpha=0.5)
axes[4].set_ylabel('|B| (uT)')
axes[4].set_xlabel('Time (s)')
axes[4].grid(True, alpha=0.3)

fig.suptitle('Crash Sensor Timeline - Last 10 Seconds', fontsize=14, fontweight='bold')
plt.tight_layout()
plt.savefig('/tmp/crash_sensor_timeline.png', dpi=150)
plt.close()

# ============================================================
# Summary
# ============================================================
print(f"\n{'='*60}")
print(f"CRASH SENSOR ANALYSIS SUMMARY")
print(f"{'='*60}")
print(f"Log: {LOG_FILE}")
print(f"Duration: {ts_sec(data['imu'][-1]):.1f}s")
print(f"Crash time: ~{CRASH_TIME}s")

# Front ToF - wall visibility
print(f"\n--- Wall Visibility (Front ToF) ---")
# Check if front tof ever shows decreasing trend
for i in range(len(tf_t) - 1, -1, -1):
    if tf_d[i] < 0.1:
        print(f"  Impact detected: dist={tf_d[i]:.3f}m at t={tf_t[i]:.3f}s")
        break

# Find when front ToF was at maximum (drone was far from wall)
# and when it started decreasing
tf_arr = np.array(tf_d)
tf_t_arr = np.array(tf_t)
# Look for the point where it starts consistently decreasing toward crash
window = 5
for i in range(len(tf_arr) - window):
    if tf_t_arr[i] > CRASH_TIME - 15:  # Only look in last 15s
        segment = tf_arr[i:i+window]
        if all(segment[j] > segment[j+1] for j in range(window-1)):
            if tf_arr[i] > 0.5:  # Only if starting from a reasonable distance
                print(f"  Consistent approach started: t={tf_t_arr[i]:.3f}s, dist={tf_arr[i]:.3f}m")
                print(f"  Time to impact: ~{CRASH_TIME - tf_t_arr[i]:.1f}s")
                break

print(f"\nPlots saved to /tmp/crash_sensor_*.png")
