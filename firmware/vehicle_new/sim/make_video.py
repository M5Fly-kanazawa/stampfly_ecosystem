#!/usr/bin/env python3
"""
Generate combined SIL demo video: hover + level drop + tilted drop
SILデモ動画を生成: ホバリング + 水平落下 + 傾き落下

Usage: python3 make_video.py
Output: stampfly_sil_demo.mp4
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

# Drone dimensions [m]
# ドローン寸法
HX = 0.04075  # 81.5mm / 2
HY = 0.04075
HZ = 0.0155   # 31mm / 2


def load_csv(filename):
    """Load CSV, skipping non-data lines (e.g. ESP-IDF log messages)"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('time'):
            header_idx = i
            break
    if header_idx is None:
        raise ValueError(f"No header found in {filename}")
    clean = [lines[header_idx]]
    for line in lines[header_idx + 1:]:
        stripped = line.strip()
        if stripped and stripped[0].isdigit():
            clean.append(line)
    return np.genfromtxt(io.StringIO(''.join(clean)), delimiter=',', names=True)


def get_xyz(data):
    """Extract x, y, z columns regardless of naming convention"""
    names = data.dtype.names
    if 'true_x' in names:
        return data['true_x'], data['true_y'], data['true_z']
    elif 'pos_x' in names:
        return data['pos_x'], data['pos_y'], data['pos_z']
    else:
        t = data['time']
        return np.zeros_like(t), np.zeros_like(t), np.zeros_like(t)


def rotation_matrix(roll, pitch, yaw):
    """ZYX Euler to rotation matrix (radians)"""
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
        [+HX, +HY, +HZ], [+HX, -HY, +HZ], [-HX, -HY, +HZ], [-HX, +HY, +HZ],
        [+HX, +HY, -HZ], [+HX, -HY, -HZ], [-HX, -HY, -HZ], [-HX, +HY, -HZ],
    ])
    return (R @ corners.T).T + pos


def box_faces(v):
    return [
        [v[0], v[1], v[2], v[3]],  # bottom (NED down = display bottom)
        [v[4], v[5], v[6], v[7]],  # top
        [v[0], v[1], v[5], v[4]],
        [v[2], v[3], v[7], v[6]],
        [v[0], v[3], v[7], v[4]],
        [v[1], v[2], v[6], v[5]],
    ]


def render_scenario(ax, ax_alt, data, title, frame_idx, indices, face_colors):
    """Render one frame of a scenario"""
    idx = indices[min(frame_idx, len(indices) - 1)]
    t = data['time']

    pos_x, pos_y, pos_z_ned = get_xyz(data)
    roll_d = data['roll'] if 'roll' in data.dtype.names else np.zeros_like(t)
    pitch_d = data['pitch'] if 'pitch' in data.dtype.names else np.zeros_like(t)
    yaw_d = data['yaw'] if 'yaw' in data.dtype.names else np.zeros_like(t)

    # NED → display (z up)
    # NED → 表示座標（z上向き）
    disp_x, disp_y, disp_z = pos_x, pos_y, -pos_z_ned
    z_max = max(np.max(disp_z) + 0.1, 0.7)
    xy_r = max(np.max(np.abs(disp_x)) + 0.15, np.max(np.abs(disp_y)) + 0.15, 0.2)

    ax.cla()
    ax.set_xlim(-xy_r, xy_r)
    ax.set_ylim(-xy_r, xy_r)
    ax.set_zlim(-0.03, z_max)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('H [m]')

    # Ground plane
    # 地面
    gx = [-xy_r, xy_r, xy_r, -xy_r]
    gy = [-xy_r, -xy_r, xy_r, xy_r]
    gz = [0, 0, 0, 0]
    ax.add_collection3d(Poly3DCollection([list(zip(gx, gy, gz))],
                        alpha=0.2, facecolor='#8B7355', edgecolor='brown'))

    # Target altitude (hover only)
    # 目標高度（ホバリングのみ）
    if 'true_x' in data.dtype.names:
        tgt_x = [-xy_r, xy_r, xy_r, -xy_r]
        tgt_y = [-xy_r, -xy_r, xy_r, xy_r]
        tgt_z = [0.5, 0.5, 0.5, 0.5]
        ax.add_collection3d(Poly3DCollection([list(zip(tgt_x, tgt_y, tgt_z))],
                            alpha=0.05, facecolor='green', edgecolor='green',
                            linestyle='--'))

    # Drone box
    # ドローン立方体
    T = np.diag([1, 1, -1])
    R_ned = rotation_matrix(np.radians(roll_d[idx]),
                            np.radians(pitch_d[idx]),
                            np.radians(yaw_d[idx]))
    R_disp = T @ R_ned @ T
    pos = np.array([disp_x[idx], disp_y[idx], disp_z[idx]])
    verts = box_vertices(pos, R_disp)
    drone = Poly3DCollection(box_faces(verts), alpha=0.7, linewidths=1,
                             edgecolors='black')
    drone.set_facecolor(face_colors)
    ax.add_collection3d(drone)

    # Motor markers (top 4 corners)
    # モーターマーカー（上面4頂点）
    for ci in [4, 5, 6, 7]:
        ax.scatter(*verts[ci], color='red', s=15, zorder=5)

    # Trail + shadow
    # 軌跡 + 影
    ax.plot(disp_x[:idx+1], disp_y[:idx+1], disp_z[:idx+1],
            'b-', alpha=0.3, linewidth=0.8)
    ax.plot([disp_x[idx]], [disp_y[idx]], [0], 'ko', alpha=0.2, markersize=4)
    ax.view_init(elev=25, azim=-60)
    ax.set_title(f'{title}  t={t[idx]:.2f}s  alt={disp_z[idx]*100:.1f}cm',
                 fontsize=10)

    # Altitude subplot
    # 高度サブプロット
    ax_alt.cla()
    ax_alt.plot(t, disp_z * 100, 'b-', linewidth=0.8, alpha=0.5)
    ax_alt.axhline(y=0, color='brown', linewidth=1)
    ax_alt.plot(t[idx], disp_z[idx] * 100, 'ro', markersize=6)
    ax_alt.axvline(x=t[idx], color='red', alpha=0.3)
    ax_alt.set_ylabel('Height [cm]')
    ax_alt.set_xlabel('Time [s]')
    ax_alt.set_title('Altitude', fontsize=9)
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(min(np.min(disp_z * 100) - 2, -2),
                    max(np.max(disp_z * 100) + 5, 60))

    # ESKF overlay (hover scenario only)
    # ESKFオーバーレイ（ホバリングシナリオのみ）
    if 'eskf_z' in data.dtype.names:
        eskf_z_disp = -data['eskf_z'] * 100
        ax_alt.plot(t, eskf_z_disp, 'r--', linewidth=0.8, alpha=0.4,
                    label='ESKF')
        ax_alt.legend(fontsize=7, loc='lower right')


def main():
    print("Loading data...")
    hover = load_csv('flight_log.csv')
    drop_l = load_csv('drop_level.csv')
    drop_t = load_csv('drop_tilted.csv')

    fps = 30
    # Subsample each scenario
    # 各シナリオをサブサンプル
    scenarios = [
        (hover,  "Hover Flight (target 50cm)", 5),
        (drop_l, "Level Drop 0.5m",            2),
        (drop_t, "Tilted Drop 30\u00b0",       2),
    ]

    TITLE_FRAMES = 15  # 0.5s title card

    # Count total frames
    # 総フレーム数を計算
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

    fig.text(0.5, 0.01,
             'StampFly SIL — Correct accelerometer model '
             '(accel_output = a_inertial \u2212 g_body, no +2g hack)',
             ha='center', fontsize=7, style='italic', color='gray')

    print(f"Total frames: {total_frames}")
    print("Rendering...")

    def update(frame):
        f = frame
        for si, (data, title, indices) in enumerate(scene_info):
            if f < TITLE_FRAMES:
                # Title card
                # タイトルカード
                ax.cla()
                ax.set_xlim(-1, 1)
                ax.set_ylim(-1, 1)
                ax.set_zlim(-1, 1)
                ax.text2D(0.5, 0.5, title, transform=ax.transAxes,
                          fontsize=18, ha='center', va='center',
                          bbox=dict(boxstyle='round,pad=0.5',
                                    facecolor='wheat'))
                ax.set_axis_off()
                ax_alt.cla()
                ax_alt.text(0.5, 0.5, f'Scene {si+1}/{len(scene_info)}',
                            transform=ax_alt.transAxes,
                            fontsize=14, ha='center', va='center')
                ax_alt.set_axis_off()
                return []
            f -= TITLE_FRAMES
            if f < len(indices):
                render_scenario(ax, ax_alt, data, title, f, indices,
                                face_colors)
                return []
            f -= len(indices)
        return []

    anim = FuncAnimation(fig, update, frames=total_frames,
                         interval=1000 // fps, blit=False)

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
