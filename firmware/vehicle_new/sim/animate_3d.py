#!/usr/bin/env python3
"""
StampFly SIL 3D Flight Animator
StampFly SIL 3Dフライトアニメーション

Renders the drone as a 3D box (81.5x81.5x31mm) with correct proportions.
"""

import sys
import io
import argparse
import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Error: matplotlib required")
    sys.exit(1)


# Drone dimensions [m]
HX = 0.04075  # 81.5mm / 2
HY = 0.04075
HZ = 0.0155   # 31mm / 2


def load_csv(filename):
    with open(filename, 'r') as f:
        lines = f.readlines()
    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('time'):
            header_idx = i
            break
    if header_idx is None:
        raise ValueError("No header found")
    clean = [lines[header_idx]]
    for line in lines[header_idx + 1:]:
        if line.strip() and line[0].isdigit():
            clean.append(line)
    return np.genfromtxt(io.StringIO(''.join(clean)), delimiter=',', names=True)


def rotation_matrix(roll, pitch, yaw):
    """Build rotation matrix from Euler angles (radians), ZYX order"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R


def box_vertices(pos, R):
    """Get 8 vertices of the drone box in world frame (display coords)"""
    # Body frame corners
    corners_body = np.array([
        [+HX, +HY, +HZ],
        [+HX, -HY, +HZ],
        [-HX, -HY, +HZ],
        [-HX, +HY, +HZ],
        [+HX, +HY, -HZ],
        [+HX, -HY, -HZ],
        [-HX, -HY, -HZ],
        [-HX, +HY, -HZ],
    ])

    # Rotate and translate
    corners_world = (R @ corners_body.T).T + pos
    return corners_world


def box_faces(verts):
    """Get 6 faces as vertex lists for Poly3DCollection"""
    faces = [
        [verts[0], verts[1], verts[2], verts[3]],  # bottom
        [verts[4], verts[5], verts[6], verts[7]],  # top
        [verts[0], verts[1], verts[5], verts[4]],  # front
        [verts[2], verts[3], verts[7], verts[6]],  # back
        [verts[0], verts[3], verts[7], verts[4]],  # right
        [verts[1], verts[2], verts[6], verts[5]],  # left
    ]
    return faces


def animate_3d(data, save_file=None, fps=25):
    t = data['time']
    # NED to display: x=forward, y=right→left(flip), z=up(flip)
    pos_x = data['true_x'] if 'true_x' in data.dtype.names else np.zeros_like(t)
    pos_y = data['true_y'] if 'true_y' in data.dtype.names else np.zeros_like(t)
    pos_z_ned = data['true_z'] if 'true_z' in data.dtype.names else np.zeros_like(t)

    roll_deg = data['roll'] if 'roll' in data.dtype.names else np.zeros_like(t)
    pitch_deg = data['pitch'] if 'pitch' in data.dtype.names else np.zeros_like(t)
    yaw_deg = data['yaw'] if 'yaw' in data.dtype.names else np.zeros_like(t)

    # Convert NED to display coordinates (X=forward, Y=right, Z=up)
    disp_x = pos_x
    disp_y = pos_y
    disp_z = -pos_z_ned  # NED z-down → display z-up

    # View bounds
    z_max = max(np.max(disp_z) + 0.15, 0.7)
    xy_range = max(np.max(np.abs(disp_x)) + 0.2, np.max(np.abs(disp_y)) + 0.2, 0.3)

    fig = plt.figure(figsize=(14, 8))
    ax = fig.add_subplot(121, projection='3d')
    ax_alt = fig.add_subplot(222)
    ax_att = fig.add_subplot(224)
    fig.suptitle('StampFly SIL 3D Animation', fontsize=14)

    # Ground plane
    ground_x = np.array([-xy_range, xy_range, xy_range, -xy_range])
    ground_y = np.array([-xy_range, -xy_range, xy_range, xy_range])
    ground_z = np.array([0, 0, 0, 0])
    ground = Poly3DCollection([list(zip(ground_x, ground_y, ground_z))],
                               alpha=0.3, facecolor='#8B7355', edgecolor='brown')
    ax.add_collection3d(ground)

    # Trail
    trail_line, = ax.plot([], [], [], 'b-', alpha=0.3, linewidth=0.8)

    # Time text
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=11,
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    # Setup timeline plots
    ax_alt.plot(t, disp_z, 'b-', linewidth=0.8, alpha=0.5)
    ax_alt.axhline(y=0, color='brown', linewidth=1)
    ax_alt.axhline(y=0.5, color='green', linestyle='--', alpha=0.3)
    ax_alt.set_ylabel('Height [m]')
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(min(np.min(disp_z) - 0.05, -0.05), z_max)
    ax_alt.set_title('Altitude')
    alt_marker, = ax_alt.plot([], [], 'ro', markersize=8)
    alt_vline = ax_alt.axvline(x=0, color='red', alpha=0.3)

    ax_att.plot(t, roll_deg, 'r-', linewidth=0.8, alpha=0.5, label='Roll')
    ax_att.plot(t, pitch_deg, 'g-', linewidth=0.8, alpha=0.5, label='Pitch')
    ax_att.plot(t, yaw_deg, 'b-', linewidth=0.8, alpha=0.5, label='Yaw')
    ax_att.set_ylabel('Angle [deg]')
    ax_att.set_xlabel('Time [s]')
    ax_att.set_xlim(t[0], t[-1])
    ax_att.legend(fontsize=8)
    ax_att.set_title('Attitude')
    att_vline = ax_att.axvline(x=0, color='red', alpha=0.3)

    # Subsample
    skip = max(1, len(t) // (int(max(t[-1], 0.1) * fps)))
    indices = list(range(0, len(t), skip))

    # Face colors: bottom=red, top=blue, sides=lightblue
    face_colors = ['#FF6666', '#6666FF', '#AADDFF', '#AADDFF', '#AADDFF', '#AADDFF']

    def update(frame_idx):
        ax.cla()

        idx = indices[frame_idx] if frame_idx < len(indices) else indices[-1]

        # Set view
        ax.set_xlim(-xy_range, xy_range)
        ax.set_ylim(-xy_range, xy_range)
        ax.set_zlim(-0.05, z_max)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Height [m]')

        # Ground
        ax.add_collection3d(Poly3DCollection(
            [list(zip(ground_x, ground_y, ground_z))],
            alpha=0.2, facecolor='#8B7355', edgecolor='brown'))

        # Target altitude
        target_x = [-xy_range, xy_range, xy_range, -xy_range]
        target_y = [-xy_range, -xy_range, xy_range, xy_range]
        target_z = [0.5, 0.5, 0.5, 0.5]
        ax.add_collection3d(Poly3DCollection(
            [list(zip(target_x, target_y, target_z))],
            alpha=0.05, facecolor='green', edgecolor='green', linestyle='--'))

        # Rotation matrix from Euler angles
        # NED Euler → display rotation
        roll_r = np.radians(roll_deg[idx])
        pitch_r = np.radians(pitch_deg[idx])
        yaw_r = np.radians(yaw_deg[idx])

        # NED rotation matrix
        R_ned = rotation_matrix(roll_r, pitch_r, yaw_r)

        # Convert NED body rotation to display:
        # Display: x=NED_x, y=NED_y, z=-NED_z
        # Need to flip z in rotation output
        T = np.diag([1, 1, -1])  # NED→display transform for z
        R_disp = T @ R_ned @ T   # Transform rotation to display coords

        pos = np.array([disp_x[idx], disp_y[idx], disp_z[idx]])
        verts = box_vertices(pos, R_disp)
        faces = box_faces(verts)

        drone = Poly3DCollection(faces, alpha=0.7, linewidths=1, edgecolors='black')
        drone.set_facecolor(face_colors)
        ax.add_collection3d(drone)

        # Motor markers (4 top corners)
        for ci in [4, 5, 6, 7]:
            ax.scatter(*verts[ci], color='red', s=20, zorder=5)

        # Trail
        ax.plot(disp_x[:idx+1], disp_y[:idx+1], disp_z[:idx+1],
                'b-', alpha=0.3, linewidth=0.8)

        # Shadow on ground
        ax.plot([disp_x[idx]], [disp_y[idx]], [0], 'ko', alpha=0.2, markersize=5)

        # Time text
        alt = disp_z[idx]
        r = roll_deg[idx]
        ax.set_title(f't = {t[idx]:.2f}s  |  alt = {alt:.3f}m  |  roll = {r:.1f}°',
                      fontsize=11)

        # Camera angle
        ax.view_init(elev=25, azim=-60)

        # Update timeline markers
        alt_marker.set_data([t[idx]], [disp_z[idx]])
        alt_vline.set_xdata([t[idx]])
        att_vline.set_xdata([t[idx]])

        return []

    anim = FuncAnimation(fig, update, frames=len(indices),
                          interval=1000//fps, blit=False, repeat=True)

    if save_file:
        try:
            anim.save(save_file, writer='ffmpeg', fps=fps, dpi=100)
            print(f"Saved to {save_file}")
        except Exception as e:
            print(f"Save failed ({e}), showing interactive")
            plt.show()
    else:
        plt.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='StampFly 3D Animator')
    parser.add_argument('-i', '--input', default='flight_log.csv')
    parser.add_argument('--save', default=None)
    parser.add_argument('--fps', type=int, default=25)
    args = parser.parse_args()

    print(f"Loading {args.input}...")
    data = load_csv(args.input)
    print(f"Loaded {len(data)} samples")
    animate_3d(data, save_file=args.save, fps=args.fps)


if __name__ == '__main__':
    main()
