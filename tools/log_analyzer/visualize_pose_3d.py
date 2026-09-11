#!/usr/bin/env python3
"""
visualize_pose_3d.py - 3D pose (position + attitude) animation from a StampFly flight-log bundle
visualize_pose_3d.py - StampFly フライトログ一式から位置＋姿勢の3Dアニメーションを描く

Displays position and attitude together: a 3D trajectory + drone pose view
and a 2D top-down view. Position comes from the `posvel` stream (pos_x/y/z,
NED, meters) and attitude from the `attitude` stream's unit quaternion
(quat_w/x/y/z), joined by `seq` since both are lockstep 400 Hz streams of a
StampFly flight-log v1 bundle (`.sflog.zip` file or an extracted directory;
see lib/sflog and docs/plans/flight-log-format-plan.md section 2.2 for the
format). NED convention: X=North, Y=East, Z=Down.
位置と姿勢を同時に表示する: 3D軌跡＋機体姿勢のビューと、真上から見た2D
ビュー。位置は `posvel` ストリーム（pos_x/y/z、NED、メートル）、姿勢は
`attitude` ストリームの単位クォータニオン（quat_w/x/y/z）から取り、どちらも
StampFly フライトログ v1 一式（`.sflog.zip` または展開済みフォルダ。形式は
lib/sflog・計画書2.2節を参照）の400Hzロックステップ・ストリームなので
`seq` で結合する。NED規約: X=北, Y=東, Z=下。

Usage / 使い方:
    python3 visualize_pose_3d.py logs/flight_<ts>.sflog.zip
    python3 visualize_pose_3d.py logs/flight_<ts>.sflog.zip --save out.mp4
    python3 visualize_pose_3d.py logs/flight_<ts>.sflog.zip --no-show --frames 100
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

# Animation defaults (subsample target for smooth playback, and the
# matplotlib FuncAnimation frame interval / save fps).
# アニメーションの既定値（滑らかな再生のための間引き目標、および
# FuncAnimation のフレーム間隔・保存時 fps）。
DEFAULT_FRAMES = 200
ANIMATION_INTERVAL_MS = 50
ANIMATION_FPS = 20
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


def load_bundle_pose(path):
    """Load time + position + roll/pitch/yaw from a bundle, joined by `seq`.
    一式から時刻・位置・roll/pitch/yaw を `seq` で結合して読み込む。

    `attitude` and `posvel` are both lockstep 400 Hz streams sharing the
    same per-control-cycle `seq` (protocol/spec/flight_log.yaml section
    2.2), so they are joined on `seq` rather than on timestamp (timestamps
    can repeat when a control cycle reuses the previous IMU sample).
    `attitude` と `posvel` はどちらも制御周期ごとの `seq` を共有する400Hz
    ロックステップ・ストリーム（計画書2.2節）。制御周期が前回のIMU標本を
    再利用すると時刻が重複しうるため、時刻ではなく `seq` で結合する。

    Args:
        path: `.sflog.zip` file or extracted bundle directory.

    Returns:
        (time_s, pos_xyz, roll, pitch, yaw): time_s is a pandas Series of
        seconds since the first sample; pos_xyz is an (N,3) numpy array of
        NED position [m]; roll/pitch/yaw are numpy arrays in radians. All
        four share the same length N (one row per matched `seq`).
        (time_s, pos_xyz, roll, pitch, yaw): time_s は先頭サンプルからの
        経過秒の pandas Series。pos_xyz は NED 位置[m]の (N,3) numpy 配列。
        roll/pitch/yaw はラジアンの numpy 配列。4つとも同じ長さ N
        （`seq` が一致した行数）を持つ。

    Raises:
        ValueError: the bundle has no `attitude` or no `posvel` stream.
    """
    log = sflog.load(path)
    if 'attitude' not in log.streams:
        raise ValueError(f"bundle has no 'attitude' stream: {path}")
    if 'posvel' not in log.streams:
        raise ValueError(f"bundle has no 'posvel' stream (position is required for pose): {path}")

    attitude = log.streams['attitude']
    posvel = log.streams['posvel']
    merged = attitude.merge(posvel, on='seq', suffixes=('', '_posvel'))

    time_us = merged['timestamp_us']
    time_s = (time_us - time_us.iloc[0]) / MICROSECONDS_PER_SECOND
    pos_xyz = merged[['pos_x', 'pos_y', 'pos_z']].to_numpy()
    roll, pitch, yaw = quat_to_euler_rad(
        merged['quat_w'].to_numpy(), merged['quat_x'].to_numpy(),
        merged['quat_y'].to_numpy(), merged['quat_z'].to_numpy(),
    )
    return time_s.reset_index(drop=True), pos_xyz, roll, pitch, yaw


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


def build_animation(time_s, pos_xyz, roll, pitch, yaw, frames=DEFAULT_FRAMES):
    """Build the pose animation figure/axes without showing or saving it.
    ポーズアニメーションの図を組み立てる（表示・保存はしない）。

    Kept identical to the original single-file script's drawing/animation
    logic; only the data source (time_s/pos_xyz/roll/pitch/yaw, now loaded
    from a flight-log bundle) and the frame-count parameterization changed.
    描画・アニメーションのロジックは元の単一ファイル版のまま。変わったのは
    データの出所（time_s/pos_xyz/roll/pitch/yaw。今はフライトログ一式から
    読み込む）とフレーム数のパラメータ化のみ。

    Args:
        time_s: pandas Series/array-like of seconds since the first sample.
        pos_xyz: (N,3) numpy array, NED position [m].
        roll, pitch, yaw: numpy arrays, radians, length N.
        frames: maximum number of animation frames (subsampled from N).

    Returns:
        (fig, anim): the matplotlib Figure and FuncAnimation.
    """
    time_s = pd.Series(time_s).reset_index(drop=True)
    pos_x = pos_xyz[:, 0]
    pos_y = pos_xyz[:, 1]
    pos_z = pos_xyz[:, 2]

    n_samples = len(roll)

    # Subsample for animation
    skip = max(1, n_samples // frames)
    indices = np.arange(0, n_samples, skip)

    # Pre-compute trajectory for display
    traj_display = np.array([transform_ned_to_display(np.array([pos_x[i], pos_y[i], pos_z[i]]))
                             for i in range(n_samples)])

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
        description='Animate position + attitude (3D trajectory/pose + top view) from a StampFly flight-log bundle',
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
    time_s, pos_xyz, roll, pitch, yaw = load_bundle_pose(args.bundle)

    print(f"Creating animation with up to {args.frames} frames...")
    fig, anim = build_animation(time_s, pos_xyz, roll, pitch, yaw, frames=args.frames)

    plt.tight_layout()

    if args.save:
        _save_animation(anim, args.save)

    if args.no_show:
        plt.close(fig)
    else:
        print("Showing animation...")
        plt.show()


if __name__ == "__main__":
    main()
