#!/usr/bin/env python3
"""
StampFly SIL Flight Animator — 2D side-view drone animation
StampFly SILフライトアニメーション — 2D側面ドローンアニメーション

Renders the drone as a rectangle (81.5x31mm) with correct aspect ratio.
ドローンを正しいアスペクト比の矩形（81.5×31mm）で描画する。

Usage:
    python3 animate_flight.py                      # From flight_log.csv
    python3 animate_flight.py -i drop_log.csv      # Custom input
    python3 animate_flight.py --save anim.mp4      # Save as video
"""

import sys
import argparse
import io
import numpy as np

try:
    import matplotlib
    matplotlib.use('TkAgg')  # Interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.patches as patches
    import matplotlib.transforms as transforms
    from matplotlib.animation import FuncAnimation
except ImportError:
    print("Error: matplotlib required. pip3 install matplotlib")
    sys.exit(1)


# Drone dimensions (meters)
DRONE_W = 0.0815  # width (81.5mm)
DRONE_H = 0.031   # height (31mm)

# Scale for display
SCALE = 1.0  # 1 pixel = 1 meter


def load_csv(filename):
    """Load flight CSV, skip non-data lines"""
    with open(filename, 'r') as f:
        lines = f.readlines()

    header_idx = None
    for i, line in enumerate(lines):
        if line.startswith('time'):
            header_idx = i
            break

    if header_idx is None:
        raise ValueError("CSV header not found")

    clean = [lines[header_idx]]
    for line in lines[header_idx + 1:]:
        if line.strip() and line[0].isdigit():
            clean.append(line)

    return np.genfromtxt(io.StringIO(''.join(clean)), delimiter=',', names=True)


def draw_drone(ax, x, z_ned, roll_deg, color='blue'):
    """Draw drone as a rotated rectangle at position (x, z_ned)
    z_ned is in NED (negative = up), display uses z_disp = -z_ned (positive = up)
    """
    z_disp = -z_ned  # NED to display: flip sign

    # Rectangle centered at drone position
    rect = patches.FancyBboxPatch(
        (-DRONE_W/2, -DRONE_H/2), DRONE_W, DRONE_H,
        boxstyle="round,pad=0.002",
        linewidth=1.5, edgecolor=color, facecolor=color, alpha=0.7)

    # Apply rotation and translation
    t = transforms.Affine2D().rotate_deg(-roll_deg).translate(x, z_disp) + ax.transData
    rect.set_transform(t)

    ax.add_patch(rect)

    # Draw motor indicators (4 small circles at corners)
    hw = DRONE_W / 2 * 0.9
    hh = DRONE_H / 2
    for mx, mz in [(-hw, -hh), (hw, -hh), (-hw, hh), (hw, hh)]:
        # Rotate motor position
        cos_r = np.cos(np.radians(-roll_deg))
        sin_r = np.sin(np.radians(-roll_deg))
        rx = mx * cos_r - mz * sin_r + x
        rz = mx * sin_r + mz * cos_r + z_disp
        motor = plt.Circle((rx, rz), 0.003, color='red', zorder=5)
        ax.add_patch(motor)

    return rect


def animate(data, save_file=None, fps=25):
    """Create animation from flight data"""

    # Extract columns
    t = data['time']
    true_x = data['true_x'] if 'true_x' in data.dtype.names else np.zeros_like(t)
    true_z = data['true_z'] if 'true_z' in data.dtype.names else np.zeros_like(t)
    roll = data['roll'] if 'roll' in data.dtype.names else np.zeros_like(t)

    # Determine view bounds
    z_vals = -true_z  # NED to display
    x_vals = true_x
    z_min = min(np.min(z_vals) - 0.1, -0.05)
    z_max = max(np.max(z_vals) + 0.2, 0.8)
    x_min = np.min(x_vals) - 0.3
    x_max = np.max(x_vals) + 0.3
    if x_max - x_min < 0.5:
        cx = (x_min + x_max) / 2
        x_min, x_max = cx - 0.25, cx + 0.25

    # Setup figure
    fig, (ax_main, ax_alt) = plt.subplots(2, 1, figsize=(10, 8),
                                           gridspec_kw={'height_ratios': [3, 1]})
    fig.suptitle('StampFly SIL Animation', fontsize=14)

    # Main view (side view)
    ax_main.set_xlim(x_min, x_max)
    ax_main.set_ylim(z_min, z_max)
    ax_main.set_aspect('equal')
    ax_main.set_xlabel('X [m]')
    ax_main.set_ylabel('Height [m]')

    # Ground
    ax_main.axhline(y=0, color='brown', linewidth=3, zorder=0)
    ax_main.fill_between([x_min, x_max], [z_min, z_min], [0, 0],
                          color='#8B7355', alpha=0.3, zorder=0)

    # Target altitude line
    ax_main.axhline(y=0.5, color='green', linestyle='--', alpha=0.3,
                     label='Target 0.5m')

    # Trail
    trail_line, = ax_main.plot([], [], 'b-', alpha=0.3, linewidth=0.5)

    # Time text
    time_text = ax_main.text(0.02, 0.95, '', transform=ax_main.transAxes,
                              fontsize=12, verticalalignment='top',
                              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    # Altitude plot
    ax_alt.set_xlim(t[0], t[-1])
    ax_alt.set_ylim(z_min, z_max)
    ax_alt.set_xlabel('Time [s]')
    ax_alt.set_ylabel('Height [m]')
    ax_alt.axhline(y=0, color='brown', linewidth=1)
    ax_alt.axhline(y=0.5, color='green', linestyle='--', alpha=0.3)
    ax_alt.plot(t, z_vals, 'b-', linewidth=0.5, alpha=0.5, label='Altitude')
    alt_marker, = ax_alt.plot([], [], 'ro', markersize=8)
    ax_alt.legend(fontsize=8)

    # Subsample for animation speed
    skip = max(1, len(t) // (int(t[-1] * fps)))
    indices = list(range(0, len(t), skip))

    def init():
        trail_line.set_data([], [])
        time_text.set_text('')
        alt_marker.set_data([], [])
        return trail_line, time_text, alt_marker

    def update(frame_idx):
        idx = indices[frame_idx] if frame_idx < len(indices) else indices[-1]

        # Clear previous drone patches
        for p in list(ax_main.patches):
            p.remove()

        # Redraw ground fill
        ax_main.fill_between([x_min, x_max], [z_min, z_min], [0, 0],
                              color='#8B7355', alpha=0.3, zorder=0)

        # Draw drone
        x = x_vals[idx]
        z = true_z[idx]
        r = roll[idx]
        draw_drone(ax_main, x, z, r)  # z is NED (negative=up), draw_drone flips

        # Trail
        trail_x = x_vals[:idx+1]
        trail_z = -true_z[:idx+1]
        trail_line.set_data(trail_x, trail_z)

        # Time text
        time_text.set_text(f't = {t[idx]:.2f}s\nalt = {-z:.3f}m\nroll = {r:.1f}°')

        # Altitude marker
        alt_marker.set_data([t[idx]], [-z])

        return trail_line, time_text, alt_marker

    anim = FuncAnimation(fig, update, init_func=init,
                          frames=len(indices), interval=1000//fps,
                          blit=False, repeat=True)

    if save_file:
        try:
            anim.save(save_file, writer='ffmpeg', fps=fps, dpi=100)
            print(f"Saved animation to {save_file}")
        except Exception as e:
            print(f"Could not save video ({e}). Showing interactive instead.")
            plt.show()
    else:
        plt.tight_layout()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='StampFly SIL Animator')
    parser.add_argument('-i', '--input', default='flight_log.csv',
                        help='Input CSV file')
    parser.add_argument('--save', default=None,
                        help='Save as video file (requires ffmpeg)')
    parser.add_argument('--fps', type=int, default=25,
                        help='Animation FPS (default: 25)')
    args = parser.parse_args()

    print(f"Loading {args.input}...")
    data = load_csv(args.input)
    print(f"Loaded {len(data)} samples")

    animate(data, save_file=args.save, fps=args.fps)


if __name__ == '__main__':
    main()
