#!/usr/bin/env python3
"""
3D Pose Animation Viewer
Displays position and attitude together
NED convention: X=North, Y=East, Z=Down

Usage:
  python3 visualize_pose_3d.py data.bin [--mp4]   # From binary log
  python3 visualize_pose_3d.py data.csv [--mp4]   # From CSV
"""

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import numpy as np
import subprocess
import os
import sys

def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert Euler angles (rad) to rotation matrix (NED convention)"""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]
    ])
    return R

def transform_ned_to_display(p):
    """
    Transform NED to display coordinates (right-handed, Z-down visible)
    Rotate 180° around Y axis: X'=-X, Y'=Y, Z'=-Z
    """
    return np.array([-p[0], p[1], -p[2]])

def draw_drone(ax, pos, R, scale=0.05):
    """Draw drone at position with orientation R"""
    arm_len = scale

    # Drone arms in body frame
    body_points = [
        np.array([arm_len, arm_len, 0]),
        np.array([-arm_len, -arm_len, 0]),
        np.array([arm_len, -arm_len, 0]),
        np.array([-arm_len, arm_len, 0]),
        np.array([arm_len * 1.5, 0, 0]),  # Nose
    ]

    # Transform to NED frame and then to display
    world_points = [transform_ned_to_display(pos + R @ p) for p in body_points]

    # Draw arms
    ax.plot([world_points[0][0], world_points[1][0]],
            [world_points[0][1], world_points[1][1]],
            [world_points[0][2], world_points[1][2]], 'k-', linewidth=2)
    ax.plot([world_points[2][0], world_points[3][0]],
            [world_points[2][1], world_points[3][1]],
            [world_points[2][2], world_points[3][2]], 'k-', linewidth=2)

    # Draw nose marker
    ax.scatter([world_points[4][0]], [world_points[4][1]], [world_points[4][2]],
               c='red', s=50, marker='o', zorder=10)

    # Draw body axes
    axis_len = scale * 1.5
    axes_body = [
        np.array([axis_len, 0, 0]),  # X - red
        np.array([0, axis_len, 0]),  # Y - green
        np.array([0, 0, axis_len]),  # Z - blue
    ]
    colors = ['red', 'green', 'blue']

    pos_display = transform_ned_to_display(pos)
    for i, (axis, color) in enumerate(zip(axes_body, colors)):
        end = transform_ned_to_display(pos + R @ axis)
        ax.plot([pos_display[0], end[0]],
                [pos_display[1], end[1]],
                [pos_display[2], end[2]],
                color=color, linewidth=2)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_pose_3d.py <data.bin|data.csv> [--mp4]")
        sys.exit(1)

    input_file = sys.argv[1]

    # Determine script directory for finding eskf_replay
    script_dir = os.path.dirname(os.path.abspath(__file__))
    eskf_debug_dir = os.path.join(script_dir, '..', 'eskf_debug')
    eskf_replay = os.path.join(eskf_debug_dir, 'build', 'eskf_replay')

    # Handle .bin files by running eskf_replay first
    if input_file.endswith('.bin'):
        csv_file = input_file.replace('.bin', '_pose.csv')
        print(f"Processing binary log: {input_file}")
        print(f"Running eskf_replay...")

        if not os.path.exists(eskf_replay):
            print(f"Error: eskf_replay not found at {eskf_replay}")
            print("Please build it first: cd tools/eskf_debug && cmake -B build && cmake --build build")
            sys.exit(1)

        result = subprocess.run([eskf_replay, input_file, csv_file],
                                capture_output=True, text=True)
        if result.returncode != 0:
            print(f"eskf_replay failed:\n{result.stderr}")
            sys.exit(1)

        # Print summary from eskf_replay
        for line in result.stdout.split('\n'):
            if 'PC ESKF' in line or 'Position:' in line or 'Attitude:' in line:
                print(line)
    else:
        csv_file = input_file

    print(f"Loading {csv_file}...")
    df = pd.read_csv(csv_file)

    # Time in seconds
    time_s = (df['timestamp_ms'] - df['timestamp_ms'].iloc[0]) / 1000.0

    # Position (convert to cm for better visualization)
    pos_x = df['pos_x'].values
    pos_y = df['pos_y'].values
    pos_z = df['pos_z'].values

    # Attitude (degrees -> radians)
    roll = np.radians(df['roll_deg'].values)
    pitch = np.radians(df['pitch_deg'].values)
    yaw = np.radians(df['yaw_deg'].values)

    n_frames = len(df)

    # Subsample for animation
    skip = max(1, n_frames // 200)
    indices = np.arange(0, n_frames, skip)

    # Pre-compute trajectory for display
    traj_display = np.array([transform_ned_to_display(np.array([pos_x[i], pos_y[i], pos_z[i]]))
                             for i in range(n_frames)])

    # Figure setup
    fig = plt.figure(figsize=(14, 6))

    # 3D view
    ax1 = fig.add_subplot(121, projection='3d')

    # 2D top view
    ax2 = fig.add_subplot(122)

    def update(frame_idx):
        idx = indices[frame_idx]
        t = time_s.iloc[idx]

        pos = np.array([pos_x[idx], pos_y[idx], pos_z[idx]])
        R = euler_to_rotation_matrix(roll[idx], pitch[idx], yaw[idx])

        # Clear 3D axis
        ax1.cla()

        # Set limits based on trajectory
        margin = 0.05
        x_range = [traj_display[:, 0].min() - margin, traj_display[:, 0].max() + margin]
        y_range = [traj_display[:, 1].min() - margin, traj_display[:, 1].max() + margin]
        z_range = [traj_display[:, 2].min() - margin, traj_display[:, 2].max() + margin]

        ax1.set_xlim(x_range)
        ax1.set_ylim(y_range)
        ax1.set_zlim(z_range)
        ax1.set_xlabel('South ← X → North')
        ax1.set_ylabel('West ← Y → East')
        ax1.set_zlabel('Up ← Z → Down')
        ax1.set_title(f'3D Pose (NED) - t={t:.2f}s')

        # View angle
        ax1.view_init(elev=30, azim=160)

        # Draw trajectory (past)
        ax1.plot(traj_display[:idx+1, 0], traj_display[:idx+1, 1], traj_display[:idx+1, 2],
                 'b-', linewidth=1, alpha=0.5)

        # Draw start point
        start_display = transform_ned_to_display(np.array([pos_x[0], pos_y[0], pos_z[0]]))
        ax1.scatter([start_display[0]], [start_display[1]], [start_display[2]],
                    c='green', s=80, marker='o', label='Start')

        # Draw ground plane reference (Z=0 in NED)
        ground_z = transform_ned_to_display(np.array([0, 0, 0]))[2]
        xx, yy = np.meshgrid(np.linspace(x_range[0], x_range[1], 2),
                             np.linspace(y_range[0], y_range[1], 2))
        ax1.plot_surface(xx, yy, np.full_like(xx, ground_z), alpha=0.1, color='gray')

        # Draw drone
        draw_drone(ax1, pos, R, scale=0.03)

        # Attitude text
        ax1.text2D(0.02, 0.95, f'Roll:  {np.degrees(roll[idx]):6.1f}°',
                   transform=ax1.transAxes, fontsize=10, color='red')
        ax1.text2D(0.02, 0.90, f'Pitch: {np.degrees(pitch[idx]):6.1f}°',
                   transform=ax1.transAxes, fontsize=10, color='green')
        ax1.text2D(0.02, 0.85, f'Yaw:   {np.degrees(yaw[idx]):6.1f}°',
                   transform=ax1.transAxes, fontsize=10, color='blue')
        ax1.text2D(0.02, 0.78, f'Pos: ({pos_x[idx]*100:.1f}, {pos_y[idx]*100:.1f}, {pos_z[idx]*100:.1f}) cm',
                   transform=ax1.transAxes, fontsize=9)

        # 2D top view
        ax2.cla()
        ax2.set_xlabel('Y (East) [cm]')
        ax2.set_ylabel('X (North) [cm]')
        ax2.set_title('Top View (looking down Z)')
        ax2.grid(True, alpha=0.3)
        ax2.axis('equal')

        # Trajectory
        ax2.plot(pos_y[:idx+1] * 100, pos_x[:idx+1] * 100, 'b-', linewidth=1, alpha=0.5)
        ax2.scatter([pos_y[0] * 100], [pos_x[0] * 100], c='green', s=80, marker='o', label='Start')

        # Current position with orientation arrow
        arrow_len = 3  # cm
        dx = arrow_len * np.cos(yaw[idx])
        dy = arrow_len * np.sin(yaw[idx])
        ax2.arrow(pos_y[idx] * 100, pos_x[idx] * 100, dy, dx,
                  head_width=1, head_length=0.5, fc='red', ec='red')
        ax2.scatter([pos_y[idx] * 100], [pos_x[idx] * 100], c='blue', s=50, marker='o')

        # 20cm reference square
        square_x = [0, 0, -20, -20, 0]
        square_y = [0, 20, 20, 0, 0]
        ax2.plot(square_y, square_x, 'k--', alpha=0.3, label='20cm ref')

        ax2.legend(loc='upper right', fontsize=8)

        return []

    print(f"Creating animation with {len(indices)} frames...")
    ani = animation.FuncAnimation(fig, update, frames=len(indices),
                                   blit=False, interval=50)

    plt.tight_layout()

    # Output
    save_mp4 = '--mp4' in sys.argv

    if save_mp4:
        # Use input file name for output
        base_name = input_file.replace('.bin', '').replace('.csv', '')
        output_file = base_name + '_pose.mp4'
        print(f"Saving to {output_file}...")
        writer = animation.FFMpegWriter(fps=20, bitrate=2000)
        ani.save(output_file, writer=writer)
        print(f"Saved: {output_file}")
    else:
        print("Showing animation...")
        plt.show()

if __name__ == "__main__":
    main()
