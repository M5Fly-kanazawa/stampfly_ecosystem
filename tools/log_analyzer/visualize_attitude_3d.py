#!/usr/bin/env python3
"""
visualize_attitude_3d.py - 3D attitude animation from a StampFly flight-log bundle
visualize_attitude_3d.py - StampFly フライトログ一式から姿勢の3Dアニメーションを描く

Displays attitude as animated 3D body-frame axes plus an Euler-angle time
history. The Euler angles are computed from the `attitude` stream's unit
quaternion (quat_w/x/y/z) of a StampFly flight-log v1 bundle (`.sflog.zip`
file or an extracted directory; see lib/sflog and
docs/plans/flight-log-format-plan.md section 2.2 for the format).
NED convention: X=North, Y=East, Z=Down. Right-hand rule preserved.
姿勢を、アニメーションする3D機体軸とオイラー角の時系列で表示する。オイラー角は
StampFly フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ。形式は
lib/sflog・計画書2.2節を参照）の `attitude` ストリームの単位クォータニオン
（quat_w/x/y/z）から計算する。NED規約: X=北, Y=東, Z=下。右手系を維持する。

Usage / 使い方:
    python3 visualize_attitude_3d.py logs/flight_<ts>.sflog.zip
    python3 visualize_attitude_3d.py logs/flight_<ts>.sflog.zip --save out.mp4
    python3 visualize_attitude_3d.py logs/flight_<ts>.sflog.zip --no-show --frames 100
"""

import argparse
import sys
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
import numpy as np

import sflog

# Unit conversion: attitude.csv's timestamp_us -> seconds since first sample.
# 単位変換: attitude.csv の timestamp_us -> 先頭サンプルからの経過秒。
MICROSECONDS_PER_SECOND = 1_000_000.0

# arcsin() domain guard: 2(wy-zx) can drift slightly outside [-1, 1] from
# float round-off even for a normalized quaternion.
# arcsin() の定義域保護: 正規化済みクォータニオンでも浮動小数点誤差で
# 2(wy-zx) が [-1, 1] をわずかに外れることがある。
QUAT_ASIN_CLAMP = 1.0

# Animation defaults (subsample target for smooth ~30fps playback, and the
# matplotlib FuncAnimation frame interval / save fps).
# アニメーションの既定値（滑らかな再生のための間引き目標、および
# FuncAnimation のフレーム間隔・保存時 fps）。
DEFAULT_FRAMES = 300
ANIMATION_INTERVAL_MS = 33
ANIMATION_FPS = 30
ANIMATION_BITRATE = 2000


def quat_to_euler_rad(qw, qx, qy, qz):
    """Convert a unit quaternion (w,x,y,z) to roll/pitch/yaw [rad].
    単位クォータニオン(w,x,y,z)を roll/pitch/yaw [rad] に変換する。

    Standard body-to-world Euler extraction (protocol/spec/flight_log.yaml
    `attitude` stream; docs/plans/flight-log-format-plan.md section 2.2):
      roll  = atan2(2(wx+yz), 1-2(x^2+y^2))
      pitch = asin(clamp(2(wy-zx), -1, 1))
      yaw   = atan2(2(wz+xy), 1-2(y^2+z^2))
    標準の body-to-world オイラー角抽出（上記参照）。

    Args:
        qw, qx, qy, qz: scalars or numpy arrays (unit quaternion components).

    Returns:
        (roll, pitch, yaw): same shape as input, radians.
    """
    qw = np.asarray(qw, dtype=float)
    qx = np.asarray(qx, dtype=float)
    qy = np.asarray(qy, dtype=float)
    qz = np.asarray(qz, dtype=float)

    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    pitch_sin = np.clip(2.0 * (qw * qy - qz * qx), -QUAT_ASIN_CLAMP, QUAT_ASIN_CLAMP)
    pitch = np.arcsin(pitch_sin)
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy**2 + qz**2))
    return roll, pitch, yaw


def load_bundle_attitude(path):
    """Load time + roll/pitch/yaw from a bundle's `attitude` stream.
    一式の `attitude` ストリームから時刻と roll/pitch/yaw を読み込む。

    Args:
        path: `.sflog.zip` file or extracted bundle directory.

    Returns:
        (time_s, roll, pitch, yaw): time_s is a pandas Series of seconds
        since the first sample (0-based index, one entry per attitude row);
        roll/pitch/yaw are numpy arrays in radians, same length.
        (time_s, roll, pitch, yaw): time_s は先頭サンプルからの経過秒
        （0始まりindex、attitude の行ごとに1つ）の pandas Series。
        roll/pitch/yaw は同じ長さのラジアン numpy 配列。

    Raises:
        ValueError: the bundle has no `attitude` stream.
    """
    log = sflog.load(path)
    if 'attitude' not in log.streams:
        raise ValueError(f"bundle has no 'attitude' stream: {path}")

    attitude = log.streams['attitude']
    time_s = (attitude['timestamp_us'] - attitude['timestamp_us'].iloc[0]) / MICROSECONDS_PER_SECOND
    roll, pitch, yaw = quat_to_euler_rad(
        attitude['quat_w'].to_numpy(), attitude['quat_x'].to_numpy(),
        attitude['quat_y'].to_numpy(), attitude['quat_z'].to_numpy(),
    )
    return time_s.reset_index(drop=True), roll, pitch, yaw


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


def build_animation(time_s, roll, pitch, yaw, frames=DEFAULT_FRAMES):
    """Build the attitude animation figure/axes without showing or saving it.
    姿勢アニメーションの図を組み立てる（表示・保存はしない）。

    Kept identical to the original single-file script's drawing/animation
    logic; only the data source (time_s/roll/pitch/yaw, now loaded from a
    flight-log bundle) and the frame-count parameterization changed.
    描画・アニメーションのロジックは元の単一ファイル版のまま。変わったのは
    データの出所（time_s/roll/pitch/yaw。今はフライトログ一式から読み込む）と
    フレーム数のパラメータ化のみ。

    Args:
        time_s: pandas Series/array-like of seconds since the first sample.
        roll, pitch, yaw: numpy arrays, radians, same length as time_s.
        frames: maximum number of animation frames (subsampled from the
            full sample count for smooth playback).

    Returns:
        (fig, anim): the matplotlib Figure and FuncAnimation.
    """
    time_s = pd.Series(time_s).reset_index(drop=True)
    roll_display_deg = np.degrees(roll)
    pitch_display_deg = np.degrees(pitch)
    yaw_display_deg = np.degrees(yaw)

    n_samples = len(roll)

    # Subsample for smoother animation (target ~30 fps playback)
    skip = max(1, n_samples // frames)
    indices = np.arange(0, n_samples, skip)

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
    ax2.plot(time_s, roll_display_deg, 'r-', alpha=0.3, linewidth=0.5)
    ax2.plot(time_s, pitch_display_deg, 'g-', alpha=0.3, linewidth=0.5)
    ax2.plot(time_s, yaw_display_deg, 'b-', alpha=0.3, linewidth=0.5)

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
        roll_marker.set_data([t], [roll_display_deg[idx]])
        pitch_marker.set_data([t], [pitch_display_deg[idx]])
        yaw_marker.set_data([t], [yaw_display_deg[idx]])

        # Update time line
        time_line.set_xdata([t, t])

        return []

    anim = animation.FuncAnimation(fig, update, frames=len(indices),
                                    blit=False, interval=ANIMATION_INTERVAL_MS)
    return fig, anim


def _save_animation(anim, out_path):
    """Save `anim` to `out_path`; writer picked by extension.
    `anim` を `out_path` へ保存する（拡張子で書き出し方式を選ぶ）。

    `.gif` -> PillowWriter, `.mp4` -> FFMpegWriter. Exits with a clear
    message if the requested writer isn't available (e.g. ffmpeg not on
    PATH) or the extension is neither.
    `.gif` は PillowWriter、`.mp4` は FFMpegWriter を使う。要求した書き出し
    方式が使えない場合（例: ffmpeg が PATH に無い）や、対応しない拡張子の
    場合は分かりやすいメッセージを出して終了する。
    """
    suffix = Path(out_path).suffix.lower()
    if suffix == '.gif':
        writer = animation.PillowWriter(fps=ANIMATION_FPS)
    elif suffix == '.mp4':
        writer = animation.FFMpegWriter(fps=ANIMATION_FPS, bitrate=ANIMATION_BITRATE)
    else:
        print(f"Error: unsupported --save extension '{suffix}' (use .mp4 or .gif)")
        sys.exit(1)

    print(f"Saving animation to {out_path}...")
    try:
        anim.save(out_path, writer=writer)
    except (RuntimeError, FileNotFoundError, OSError) as exc:
        print(f"Error: animation writer for '{suffix}' is unavailable: {exc}")
        sys.exit(1)
    print(f"Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Animate attitude (3D body axes + Euler angle plot) from a StampFly flight-log bundle',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('bundle', help='.sflog.zip file or extracted flight-log bundle directory')
    parser.add_argument('--save', metavar='FILE', help='Save animation to FILE (.mp4 or .gif)')
    parser.add_argument('--no-show', action='store_true', help="Don't display the animation window")
    parser.add_argument('--frames', type=int, default=DEFAULT_FRAMES,
                         help=f'Maximum animation frames (default: {DEFAULT_FRAMES})')
    args = parser.parse_args()

    print(f"Loading {args.bundle}...")
    time_s, roll, pitch, yaw = load_bundle_attitude(args.bundle)

    print(f"Creating animation with up to {args.frames} frames...")
    fig, anim = build_animation(time_s, roll, pitch, yaw, frames=args.frames)

    plt.tight_layout()

    if args.save:
        _save_animation(anim, args.save)

    if args.no_show:
        plt.close(fig)
    else:
        print("Showing animation (close window to exit)...")
        plt.show()


if __name__ == "__main__":
    main()
