#!/usr/bin/env python3
"""
3D Attitude Animation Viewer
Displays attitude as animated 3D body frame axes
NED convention: X=North, Y=East, Z=Down
Right-hand rule preserved
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import numpy as np
import sys

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (rad) to rotation matrix (NED convention)"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R

def transform_ned_to_display(p):
    """
    Transform NED coordinates to display coordinates.
    Rotate 180° around Y axis to preserve right-handedness
    while making Z appear to go down.

    NED: X=North, Y=East, Z=Down
    Display: X'=-X (South), Y'=Y (East), Z'=-Z (Up in matplotlib = Down visually)

    When viewed from azim=180 (from -X' = +X_NED = North side):
    - Y appears to the right (East) ✓
    - Z appears down ✓
    - Right-hand rule preserved ✓
    """
    return np.array([-p[0], p[1], -p[2]])

def draw_arrow_3d(ax, origin, direction, color, linewidth=2):
    """Draw a 3D arrow with cone head"""
    origin_t = transform_ned_to_display(origin)
    end = origin + direction
    end_t = transform_ned_to_display(end)
    direction_t = end_t - origin_t

    # Draw the shaft
    ax.plot([origin_t[0], end_t[0]],
            [origin_t[1], end_t[1]],
            [origin_t[2], end_t[2]],
            color=color, linewidth=linewidth)

    # Draw cone head
    length = np.linalg.norm(direction)
    if length > 0.1:
        cone_length = 0.2
        cone_radius = 0.08

        d = direction_t / np.linalg.norm(direction_t)

        # Find perpendicular vectors
        if abs(d[0]) < 0.9:
            perp1 = np.cross(d, np.array([1, 0, 0]))
        else:
            perp1 = np.cross(d, np.array([0, 1, 0]))
        perp1 = perp1 / np.linalg.norm(perp1)
        perp2 = np.cross(d, perp1)

        # Cone base center
        base_center = end_t - d * cone_length

        # Create cone triangles
        n_segments = 12
        triangles = []
        for i in range(n_segments):
            angle1 = 2 * np.pi * i / n_segments
            angle2 = 2 * np.pi * (i + 1) / n_segments

            p1 = base_center + cone_radius * (np.cos(angle1) * perp1 + np.sin(angle1) * perp2)
            p2 = base_center + cone_radius * (np.cos(angle2) * perp1 + np.sin(angle2) * perp2)

            # Side triangle
            triangles.append([end_t, p1, p2])

            # Base triangle
            triangles.append([base_center, p2, p1])

        cone = Poly3DCollection(triangles, alpha=1.0)
        cone.set_facecolor(color)
        cone.set_edgecolor(color)
        ax.add_collection3d(cone)

def draw_drone_body(ax, R):
    """Draw drone body (X shape with front marker)"""
    arm_len = 0.5
    points = [
        R @ np.array([arm_len, arm_len, 0]),
        R @ np.array([-arm_len, -arm_len, 0]),
        R @ np.array([arm_len, -arm_len, 0]),
        R @ np.array([-arm_len, arm_len, 0]),
    ]
    points_t = [transform_ned_to_display(p) for p in points]

    ax.plot([points_t[0][0], points_t[1][0]],
            [points_t[0][1], points_t[1][1]],
            [points_t[0][2], points_t[1][2]], 'k-', linewidth=3)
    ax.plot([points_t[2][0], points_t[3][0]],
            [points_t[2][1], points_t[3][1]],
            [points_t[2][2], points_t[3][2]], 'k-', linewidth=3)

    # Front marker (red dot at nose)
    front = transform_ned_to_display(R @ np.array([0.7, 0, 0]))
    ax.scatter([front[0]], [front[1]], [front[2]], c='red', s=100, marker='o', zorder=10)

def main():
    if len(sys.argv) < 2:
        csv_file = "test_3d_mag_result.csv"
    else:
        csv_file = sys.argv[1]

    print(f"Loading {csv_file}...")
    df = pd.read_csv(csv_file)

    # Time in seconds
    time_s = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0

    # Get attitude data (degrees -> radians)
    roll_deg = df['roll_deg'].values
    pitch_deg = df['pitch_deg'].values
    yaw_deg = df['yaw_deg'].values

    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    n_frames = len(df)

    # Subsample for smoother animation (target ~30 fps playback)
    skip = max(1, n_frames // 300)
    indices = np.arange(0, n_frames, skip)

    # Create figure
    fig = plt.figure(figsize=(14, 6))

    # 3D attitude view
    ax1 = fig.add_subplot(121, projection='3d')

    # Euler angles plot
    ax2 = fig.add_subplot(122)
    ax2.set_xlim([0, time_s.iloc[-1]])
    ax2.set_ylim([-180, 180])
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Angle [deg]')
    ax2.set_title('Euler Angles')
    ax2.grid(True, alpha=0.3)

    # Plot full Euler angle traces (faded)
    ax2.plot(time_s, roll_deg, 'r-', alpha=0.3, linewidth=0.5)
    ax2.plot(time_s, pitch_deg, 'g-', alpha=0.3, linewidth=0.5)
    ax2.plot(time_s, yaw_deg, 'b-', alpha=0.3, linewidth=0.5)

    # Current position markers on Euler plot
    roll_marker, = ax2.plot([], [], 'ro', markersize=8, label='Roll')
    pitch_marker, = ax2.plot([], [], 'go', markersize=8, label='Pitch')
    yaw_marker, = ax2.plot([], [], 'bo', markersize=8, label='Yaw')
    ax2.legend(loc='upper right')

    # Vertical time line on Euler plot
    time_line = ax2.axvline(x=0, color='k', linestyle='--', alpha=0.5)

    def update(frame_idx):
        idx = indices[frame_idx]
        t = time_s.iloc[idx]
        r, p, y = roll[idx], pitch[idx], yaw[idx]

        # Compute rotation matrix (body to NED)
        R = euler_to_rotation_matrix(r, p, y)

        # Body frame axes in NED frame
        x_axis = R @ np.array([1, 0, 0])
        y_axis = R @ np.array([0, 1, 0])
        z_axis = R @ np.array([0, 0, 1])

        # Clear and redraw 3D axes
        ax1.cla()

        # Set limits and labels
        ax1.set_xlim([-1.5, 1.5])
        ax1.set_ylim([-1.5, 1.5])
        ax1.set_zlim([-1.5, 1.5])
        ax1.set_xlabel('South ← → North')
        ax1.set_ylabel('West ← → East')
        ax1.set_zlabel('Down ← → Up')
        ax1.set_title('Body Frame (NED, view from behind)')

        # View from behind and slightly above
        # azim=180: viewing from +X_NED (North) direction, looking at origin
        ax1.view_init(elev=25, azim=160)

        # Draw NED reference frame (faded)
        draw_arrow_3d(ax1, np.zeros(3), np.array([0.8, 0, 0]), 'lightcoral', linewidth=1)
        draw_arrow_3d(ax1, np.zeros(3), np.array([0, 0.8, 0]), 'lightgreen', linewidth=1)
        draw_arrow_3d(ax1, np.zeros(3), np.array([0, 0, 0.8]), 'lightblue', linewidth=1)

        # Draw body frame axes (bold)
        draw_arrow_3d(ax1, np.zeros(3), x_axis, 'red', linewidth=3)
        draw_arrow_3d(ax1, np.zeros(3), y_axis, 'green', linewidth=3)
        draw_arrow_3d(ax1, np.zeros(3), z_axis, 'blue', linewidth=3)

        # Draw drone body
        draw_drone_body(ax1, R)

        # Add axis labels at arrow tips
        tip_x = transform_ned_to_display(x_axis * 1.15)
        tip_y = transform_ned_to_display(y_axis * 1.15)
        tip_z = transform_ned_to_display(z_axis * 1.15)
        ax1.text(*tip_x, 'X', color='red', fontsize=10, fontweight='bold')
        ax1.text(*tip_y, 'Y', color='green', fontsize=10, fontweight='bold')
        ax1.text(*tip_z, 'Z', color='blue', fontsize=10, fontweight='bold')

        # Update text
        ax1.text2D(0.02, 0.95, f'Time: {t:.2f} s', transform=ax1.transAxes, fontsize=12, fontweight='bold')
        ax1.text2D(0.02, 0.88, f'Roll:  {np.degrees(r):7.2f}°', transform=ax1.transAxes, fontsize=11, color='red')
        ax1.text2D(0.02, 0.81, f'Pitch: {np.degrees(p):7.2f}°', transform=ax1.transAxes, fontsize=11, color='green')
        ax1.text2D(0.02, 0.74, f'Yaw:   {np.degrees(y):7.2f}°', transform=ax1.transAxes, fontsize=11, color='blue')

        # Right-hand rule reminder
        ax1.text2D(0.02, 0.02,
                   '+Roll: Right wing down\n+Pitch: Nose up\n+Yaw: CW from above',
                   transform=ax1.transAxes, fontsize=8, color='gray', verticalalignment='bottom')

        # Update Euler angle markers
        roll_marker.set_data([t], [roll_deg[idx]])
        pitch_marker.set_data([t], [pitch_deg[idx]])
        yaw_marker.set_data([t], [yaw_deg[idx]])

        # Update time line
        time_line.set_xdata([t, t])

        return []

    print(f"Creating animation with {len(indices)} frames...")
    ani = animation.FuncAnimation(fig, update, frames=len(indices),
                                   blit=False, interval=33)

    plt.tight_layout()

    # Determine output format
    save_mp4 = False
    save_gif = False

    if len(sys.argv) > 2:
        if sys.argv[2] == '--mp4':
            save_mp4 = True
        elif sys.argv[2] == '--save' or sys.argv[2] == '--gif':
            save_gif = True

    if save_mp4:
        output_file = csv_file.replace('.csv', '_attitude.mp4')
        print(f"Saving animation to {output_file}...")
        try:
            writer = animation.FFMpegWriter(fps=30, bitrate=2000)
            ani.save(output_file, writer=writer)
            print(f"Saved: {output_file}")
        except Exception as e:
            print(f"FFmpeg error: {e}")
            print("Trying with default writer...")
            ani.save(output_file, fps=30)
            print(f"Saved: {output_file}")
    elif save_gif:
        output_file = csv_file.replace('.csv', '_attitude.gif')
        print(f"Saving animation to {output_file}...")
        ani.save(output_file, writer='pillow', fps=30)
        print(f"Saved: {output_file}")
    else:
        print("Showing animation (close window to exit)...")
        plt.show()

if __name__ == "__main__":
    main()
