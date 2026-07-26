#!/usr/bin/env python3
"""
visualize_sil_trajectory.py - SIL trajectory.csv Visualization Tool
SIL（Software-in-the-Loop）trajectory.csv 可視化ツール

Visualizes the trajectory.csv produced by `sf sil scenario` runs
(simulator/sil/viz/out_scn_<name>/trajectory.csv): altitude, attitude
(roll/pitch), yaw rate, and motor duty, each with truth overlaid on
estimate/command where applicable. The panel choices mirror
simulator/sil/viz/render_video.py's graph_frame() so the static plot and
the review video agree on what "truth vs estimate" means for this format.

`sf sil scenario` 実行が生成する trajectory.csv を可視化する。高度・姿勢
（roll/pitch）・ヨーレート・モータduty の4段で、該当するものは真値と
推定値/指令値を重ねて描く。パネル選定は simulator/sil/viz/render_video.py の
graph_frame() に合わせてあり、静止画とレビュー動画で「真値 vs 推定」の
意味が食い違わないようにしている。

Usage:
    python visualize_sil_trajectory.py <trajectory.csv> [options]

Options:
    --save FILE     Save figure to file
    --no-show       Don't display window (use with --save)

Examples:
    python visualize_sil_trajectory.py out_scn_acro_flight/trajectory.csv
    python visualize_sil_trajectory.py trajectory.csv --save traj.png --no-show
"""

import argparse
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Column set that identifies a SIL trajectory.csv (see run_viz() in
# lib/sfcli/commands/log.py for the format-detection dispatch this backs).
# SIL trajectory.csv を識別する列集合（判定本体は lib/sfcli/commands/log.py の
# run_viz() 側にある。ここでは可視化のみを担当）。
REQUIRED_COLUMNS = {
    't', 'px', 'py', 'pz', 'qw', 'alt', 'roll', 'pitch',
    'yawrate', 'yawcmd', 'alt_est', 'm0', 'm1', 'm2', 'm3',
}


def load_trajectory_csv(filename):
    """Load a SIL trajectory.csv into a DataFrame with a zero-based time column.
    SIL trajectory.csv を読み込み、0始まりの time_s 列を付与する。

    Columns (50 Hz, seconds): t,px,py,pz,qw,qx,qy,qz,alt,roll,pitch,yawrate,
    yawcmd,alt_est,roll_est,pitch_est,m0,m1,m2,m3. roll/pitch/roll_est/pitch_est
    are in DEGREES (verified against simulator/sil/viz/render_video.py, which
    plots them directly on a "[deg]" axis with no conversion); yawrate/yawcmd
    are in rad/s; alt/alt_est in meters; m0-m3 are motor duty in [0, 1].
    roll/pitch/roll_est/pitch_est は度[deg]単位（render_video.py が変換なしで
    "[deg]" 軸へ直接プロットしていることから確認済み）。yawrate/yawcmd は rad/s、
    alt/alt_est は m、m0-m3 はモータduty [0,1]。
    """
    df = pd.read_csv(filename)
    df['time_s'] = df['t'] - df['t'].iloc[0]

    print(f"Loaded {len(df)} samples from {os.path.basename(filename)}")
    if len(df) > 1:
        duration = df['time_s'].iloc[-1]
        print(f"Duration: {duration:.1f}s")
        print(f"Sample rate: {len(df) / duration:.1f} Hz")

    return df


def plot_altitude(ax, df):
    """Altitude: truth (solid) vs estimate (dashed), same pairing as graph_frame().
    高度: 真値(実線)と推定値(破線)。graph_frame() と同じ組み合わせ。"""
    t = df['time_s']
    ax.plot(t, df['alt'], 'C0-', linewidth=1.2, label='truth')
    ax.plot(t, df['alt_est'], 'C1--', linewidth=1.2, label='estimate')
    ax.set_ylabel('Altitude [m]')
    ax.set_title('Altitude')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)


def plot_attitude(ax_roll, ax_pitch, df):
    """Roll and pitch in separate panels, each with truth (solid) vs estimate
    (dashed) overlaid. Split panels (rather than 4 lines in one axes) keep the
    truth/estimate pairing readable per angle.
    ロール・ピッチを別パネルに分け、各々真値(実線)と推定値(破線)を重ねる。
    1パネルに4本重ねるより、角度ごとの真値/推定の対応が読み取りやすい。"""
    t = df['time_s']

    ax_roll.plot(t, df['roll'], 'C0-', linewidth=1.0, label='truth')
    ax_roll.plot(t, df['roll_est'], 'C0--', linewidth=1.0, label='estimate')
    ax_roll.set_ylabel('Roll [deg]')
    ax_roll.set_title('Roll')
    ax_roll.legend(loc='upper right', fontsize=8)
    ax_roll.grid(True, alpha=0.3)
    ax_roll.axhline(y=0, color='k', linewidth=0.5)

    ax_pitch.plot(t, df['pitch'], 'C3-', linewidth=1.0, label='truth')
    ax_pitch.plot(t, df['pitch_est'], 'C3--', linewidth=1.0, label='estimate')
    ax_pitch.set_ylabel('Pitch [deg]')
    ax_pitch.set_title('Pitch')
    ax_pitch.legend(loc='upper right', fontsize=8)
    ax_pitch.grid(True, alpha=0.3)
    ax_pitch.axhline(y=0, color='k', linewidth=0.5)


def plot_yawrate(ax, df):
    """Yaw rate: truth (solid) vs commanded (dashed).
    ヨーレート: 真値(実線)と指令値(破線)を重ねて表示。"""
    t = df['time_s']
    ax.plot(t, df['yawrate'], 'C0-', linewidth=1.0, label='truth')
    ax.plot(t, df['yawcmd'], 'C3--', linewidth=1.0, label='command')
    ax.set_ylabel('Yaw rate [rad/s]')
    ax.set_title('Yaw Rate')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linewidth=0.5)


def plot_motors(ax, df):
    """All 4 motor duty channels overlaid in one panel (m0-m3 -> M1-M4 labels,
    matching the legend convention in render_video.py's graph_frame()).
    4モータのduty[0-1]を1パネルに重ねて表示（m0-m3 -> M1-M4 のラベル対応は
    render_video.py の graph_frame() の凡例規約に合わせた）。"""
    t = df['time_s']
    ax.plot(t, df['m0'], 'C0-', linewidth=0.8, label='M1')
    ax.plot(t, df['m1'], 'C1-', linewidth=0.8, label='M2')
    ax.plot(t, df['m2'], 'C2-', linewidth=0.8, label='M3')
    ax.plot(t, df['m3'], 'C3-', linewidth=0.8, label='M4')
    ax.set_ylabel('Motor duty [0-1]')
    ax.set_xlabel('Time [s]')
    ax.set_title('Motor Duty')
    ax.set_ylim(-0.05, 1.05)
    ax.legend(loc='upper right', fontsize=8, ncol=4)
    ax.grid(True, alpha=0.3)


def visualize_all(df, filename, save_path=None, show=True):
    """Create the 4-stage SIL trajectory overview: altitude / attitude
    (roll | pitch) / yaw rate / motor duty.
    SIL trajectory の4段プロット: 高度 / 姿勢(roll | pitch) / ヨーレート / モータduty。"""
    fig = plt.figure(figsize=(12, 11))
    fig.suptitle(f'SIL Trajectory: {os.path.basename(filename)}', fontsize=14)

    # 4 rows: altitude (full width), attitude (roll | pitch side by side),
    # yaw rate (full width), motors (full width).
    # 4行構成: 高度(全幅)、姿勢(roll|pitchを横並び)、ヨーレート(全幅)、モータ(全幅)。
    gs = GridSpec(4, 2, figure=fig, hspace=0.45, wspace=0.3)

    ax_alt = fig.add_subplot(gs[0, :])
    plot_altitude(ax_alt, df)

    ax_roll = fig.add_subplot(gs[1, 0])
    ax_pitch = fig.add_subplot(gs[1, 1], sharex=ax_roll)
    plot_attitude(ax_roll, ax_pitch, df)

    ax_yaw = fig.add_subplot(gs[2, :])
    plot_yawrate(ax_yaw, df)

    ax_motor = fig.add_subplot(gs[3, :])
    plot_motors(ax_motor, df)

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved figure to {save_path}")

    if show:
        plt.show()
    else:
        plt.close()


def main():
    parser = argparse.ArgumentParser(
        description='Visualize SIL trajectory.csv (sf sil scenario output)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('file', help='trajectory.csv path')
    parser.add_argument('--save', metavar='FILE', help='Save figure to file')
    parser.add_argument('--no-show', action='store_true', help="Don't display window")

    args = parser.parse_args()

    if not os.path.exists(args.file):
        print(f"Error: File not found: {args.file}")
        sys.exit(1)

    df = load_trajectory_csv(args.file)
    visualize_all(df, args.file, save_path=args.save, show=not args.no_show)


if __name__ == '__main__':
    main()
