#!/usr/bin/env python3
"""
Generate combined SIL demo video: takeoff + level drop + tilted drop
SILデモ動画を生成: 離陸 + 水平落下 + 傾き落下
"""

import sys
import io
import numpy as np

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.animation import FuncAnimation, FFMpegWriter
except ImportError:
    print("Error: matplotlib required")
    sys.exit(1)

HX = 0.04075
HY = 0.04075
HZ = 0.0155


def load_csv(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('time'):
            header_idx = i
            break
    if header_idx is None:
        raise ValueError("No header")
    clean = [lines[header_idx]]
    for line in lines[header_idx + 1:]:
        if line.strip() and line[0].isdigit():
            clean.append(line)
    return np.genfromtxt(io.StringIO(''.join(clean)), delimiter=',', names=True)


def rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def box_vertices(pos, R):
    corners = np.array([
        [+HX,+HY,+HZ], [+HX,-HY,+HZ], [-HX,-HY,+HZ], [-HX,+HY,+HZ],
        [+HX,+HY,-HZ], [+HX,-HY,-HZ], [-HX,-HY,-HZ], [-HX,+HY,-HZ],
    ])
    return (R @ corners.T).T + pos


def box_faces(v):
    return [
        [v[0],v[1],v[2],v[3]], [v[4],v[5],v[6],v[7]],
        [v[0],v[1],v[5],v[4]], [v[2],v[3],v[7],v[6]],
        [v[0],v[3],v[7],v[4]], [v[1],v[2],v[6],v[5]],
    ]


def render_scenario(ax, ax_alt, data, title, frame_idx, indices, face_colors):
    idx = indices[min(frame_idx, len(indices)-1)]
    t = data['time']

    pos_x = data['true_x'] if 'true_x' in data.dtype.names else np.zeros_like(t)
    pos_y = data['true_y'] if 'true_y' in data.dtype.names else np.zeros_like(t)
    pos_z_ned = data['true_z'] if 'true_z' in data.dtype.names else np.zeros_like(t)
    roll_d = data['roll'] if 'roll' in data.dtype.names else np.zeros_like(t)
    pitch_d = data['pitch'] if 'pitch' in data.dtype.names else np.zeros_like(t)
    yaw_d = data['yaw'] if 'yaw' in data.dtype.names else np.zeros_like(t)

    disp_x, disp_y, disp_z = pos_x, pos_y, -pos_z_ned
    z_max = max(np.max(disp_z) + 0.1, 0.7)
    xy_r = max(np.max(np.abs(disp_x))+0.15, np.max(np.abs(disp_y))+0.15, 0.2)

    ax.cla()
    ax.set_xlim(-xy_r, xy_r); ax.set_ylim(-xy_r, xy_r); ax.set_zlim(-0.03, z_max)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('H [m]')

    # Ground
    gx = [-xy_r, xy_r, xy_r, -xy_r]
    gy = [-xy_r, -xy_r, xy_r, xy_r]
    gz = [0,0,0,0]
    ax.add_collection3d(Poly3DCollection([list(zip(gx,gy,gz))],
                        alpha=0.2, facecolor='#8B7355', edgecolor='brown'))

    # Drone
    T = np.diag([1,1,-1])
    R_ned = rotation_matrix(np.radians(roll_d[idx]), np.radians(pitch_d[idx]), np.radians(yaw_d[idx]))
    R_disp = T @ R_ned @ T
    pos = np.array([disp_x[idx], disp_y[idx], disp_z[idx]])
    verts = box_vertices(pos, R_disp)
    drone = Poly3DCollection(box_faces(verts), alpha=0.7, linewidths=1, edgecolors='black')
    drone.set_facecolor(face_colors)
    ax.add_collection3d(drone)
    for ci in [4,5,6,7]:
        ax.scatter(*verts[ci], color='red', s=15, zorder=5)

    # Trail + shadow
    ax.plot(disp_x[:idx+1], disp_y[:idx+1], disp_z[:idx+1], 'b-', alpha=0.3, linewidth=0.8)
    ax.plot([disp_x[idx]], [disp_y[idx]], [0], 'ko', alpha=0.2, markersize=4)
    ax.view_init(elev=25, azim=-60)
    ax.set_title(f'{title}  t={t[idx]:.2f}s  alt={disp_z[idx]*100:.1f}cm', fontsize=10)

    # Altitude subplot
    ax_alt.cla()
    ax_alt.plot(t, disp_z*100, 'b-', linewidth=0.8, alpha=0.5)
    ax_alt.axhline(y=0, color='brown', linewidth=1)
    ax_alt.plot(t[idx], disp_z[idx]*100, 'ro', markersize=6)
    ax_alt.axvline(x=t[idx], color='red', alpha=0.3)
    ax_alt.set_ylabel('Height [cm]')
    ax_alt.set_xlabel('Time [s]')
    ax_alt.set_title('Altitude', fontsize=9)
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(min(np.min(disp_z*100)-2, -2), max(np.max(disp_z*100)+5, 60))


def main():
    print("Loading data...")
    hover = load_csv('flight_log.csv')
    drop_l = load_csv('drop_level.csv')
    drop_t = load_csv('drop_tilted.csv')

    fps = 30
    # Subsample each scenario
    scenarios = [
        (hover, "Hover Flight", 5),     # skip=5 → 2x slower for 10s data
        (drop_l, "Level Drop 0.5m", 2),  # skip=2 for 1s data
        (drop_t, "Tilted Drop 30deg", 2),
    ]

    # Title frames between scenarios
    TITLE_FRAMES = 15  # 0.5s title card

    # Count total frames
    total_frames = 0
    scene_info = []
    for data, title, skip in scenarios:
        n = len(data['time'])
        indices = list(range(0, n, skip))
        scene_info.append((data, title, indices))
        total_frames += TITLE_FRAMES + len(indices)

    face_colors = ['#FF6666', '#6666FF', '#AADDFF', '#AADDFF', '#AADDFF', '#AADDFF']

    fig = plt.figure(figsize=(12, 6))
    ax = fig.add_subplot(121, projection='3d')
    ax_alt = fig.add_subplot(122)

    # Note about numerical symmetry breaking
    # 数値的対称性の破れに関する注記
    fig.text(0.5, 0.01,
             'Note: Small rotations during symmetric drops are numerical artifacts (floating-point symmetry breaking), not physical.',
             ha='center', fontsize=7, style='italic', color='gray')

    print(f"Total frames: {total_frames}")
    print("Rendering...")

    def update(frame):
        # Determine which scenario and frame within it
        f = frame
        for si, (data, title, indices) in enumerate(scene_info):
            if f < TITLE_FRAMES:
                # Title card
                ax.cla()
                ax.set_xlim(-1,1); ax.set_ylim(-1,1); ax.set_zlim(-1,1)
                ax.text2D(0.5, 0.5, title, transform=ax.transAxes,
                          fontsize=18, ha='center', va='center',
                          bbox=dict(boxstyle='round,pad=0.5', facecolor='wheat'))
                ax.set_axis_off()
                ax_alt.cla()
                ax_alt.text(0.5, 0.5, f'Scene {si+1}/3', transform=ax_alt.transAxes,
                            fontsize=14, ha='center', va='center')
                ax_alt.set_axis_off()
                return []
            f -= TITLE_FRAMES
            if f < len(indices):
                render_scenario(ax, ax_alt, data, title, f, indices, face_colors)
                return []
            f -= len(indices)
        return []

    anim = FuncAnimation(fig, update, frames=total_frames,
                          interval=1000//fps, blit=False)

    output = 'stampfly_sil_demo.mp4'
    try:
        writer = FFMpegWriter(fps=fps, bitrate=2000)
        anim.save(output, writer=writer, dpi=120)
        print(f"\nSaved: {output}")
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure ffmpeg is installed: brew install ffmpeg")
        sys.exit(1)


if __name__ == '__main__':
    main()
