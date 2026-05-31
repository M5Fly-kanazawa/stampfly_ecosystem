#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Kouhei Ito
# Part of StampFly Ecosystem (SIL review video).
#
# Render a SIL milestone review video: a MuJoCo 3D flight animation (left) beside
# synchronized state graphs (right), composed into one MP4. The 3D frames replay
# the recorded MuJoCo qpos and the graphs are the recorded truth/estimate/command
# time series — so "what was computed" and "what is shown" come from the same run
# (reproducible: same trajectory.csv -> same video).  RESET_PLAN.md §9.
#
# SIL マイルストーンのレビュー動画を描く: MuJoCo の 3D 飛行アニメ（左）と同期した
# 状態グラフ（右）を 1 本の MP4 に合成する。3D は記録した MuJoCo qpos を再生し、
# グラフは記録した真値/推定/指令の時系列。計算と映像が同じ実行から来る（再現性あり）。

import argparse
import csv
import json
import os

import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import imageio.v2 as imageio


def load_trajectory(path):
    """Read trajectory.csv into a dict of numpy arrays keyed by column name."""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(x) for x in row] for row in reader]
    cols = np.array(rows).T
    return {name: cols[i] for i, name in enumerate(header)}


def render_3d(model, data, renderer, qpos, cam):
    """Set the body pose, run forward kinematics, render one RGB frame."""
    data.qpos[:7] = qpos
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, cam)
    return renderer.render()


def title_banner(text, w_px, h_px=48):
    """Render a full-width title strip as an RGB array (drawn once, reused)."""
    dpi = 100
    fig = plt.figure(figsize=(w_px / dpi, h_px / dpi), dpi=dpi)
    fig.patch.set_facecolor("white")
    fig.text(0.5, 0.5, text, ha="center", va="center",
             fontsize=15, fontweight="bold")
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return buf


def graph_frame(traj, i, w_px, h_px):
    """Render the state graphs up to frame i with a time cursor, as an RGB array."""
    t = traj["t"]
    now = t[i]
    dpi = 100
    fig, axes = plt.subplots(2, 2, figsize=(w_px / dpi, h_px / dpi), dpi=dpi)

    def panel(ax, ylabel, series, ylim=None):
        for label, y, style in series:
            ax.plot(t, y, style, label=label, linewidth=1.4)
        ax.axvline(now, color="0.5", linewidth=1.0)          # time cursor
        ax.set_xlim(t[0], t[-1])
        if ylim:
            ax.set_ylim(*ylim)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, loc="upper right")
        ax.tick_params(labelsize=7)

    panel(axes[0, 0], "altitude [m]",
          [("truth", traj["alt"], "C0-"), ("estimate", traj["alt_est"], "C1--")],
          ylim=(-0.03, 0.62))
    # Tilt stays under ~0.12° (no roll/pitch command; yaw doesn't tilt the craft), so
    # a fine ±0.3° scale resolves the tiny takeoff/landing perturbations instead of a
    # flat line on a wide axis — it reads as "rock-solid level" with visible detail.
    # tilt は ~0.12° 未満（ロール/ピッチ指令なし、ヨーは傾けない）。細かい ±0.3° 目盛で
    # 離着陸の微小擾乱まで見えるようにする（広軸の直線にしない）。
    panel(axes[0, 1], "tilt [deg]",
          [("truth", traj["tilt"], "C0-"), ("estimate", traj["tilt_est"], "C1--")],
          ylim=(-0.03, 0.3))
    panel(axes[1, 0], "yaw rate [rad/s]",
          [("command", traj["yawcmd"], "C3-"), ("truth", traj["yawrate"], "C0-")],
          ylim=(-0.2, 1.4))
    panel(axes[1, 1], "motor duty",
          [("M1", traj["m0"], "C0-"), ("M2", traj["m1"], "C1-"),
           ("M3", traj["m2"], "C2-"), ("M4", traj["m3"], "C3-")],
          ylim=(0.0, 1.0))
    for ax in axes[1, :]:
        ax.set_xlabel("time [s]", fontsize=9)

    fig.tight_layout(pad=0.6)   # use the full height; the title is a separate banner
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--bundle", required=True, help="dir with trajectory.csv + results.json")
    ap.add_argument("--out", required=True, help="output MP4 path")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--test-frame", action="store_true", help="render one frame to a PNG and exit")
    args = ap.parse_args()

    traj = load_trajectory(os.path.join(args.bundle, "trajectory.csv"))
    n = len(traj["t"])
    results = {}
    rpath = os.path.join(args.bundle, "results.json")
    if os.path.exists(rpath):
        results = json.load(open(rpath))
    verdict = "PASS" if results.get("pass") else "FAIL"
    ms = results.get("milestone", "P1")
    est = results.get("estimator", "eskf")
    title = f"StampFly SIL — {ms}: takeoff → hover → yaw → landing  ({est}, G3 {verdict})"

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    H = args.height
    W3 = int(H * 4 / 3)            # 3D pane 4:3
    renderer = mujoco.Renderer(model, height=H, width=W3)

    # Frame the full vertical flight (ground z≈0.01 → peak z≈0.54). The 3D pane is
    # 4:3 landscape, so the vertical field of view is the limit: at distance d the
    # visible height ≈ 2·d·tan(fovy/2) ≈ 0.83·d. distance 0.85 → ~0.70 m tall,
    # centered on lookat z=0.27 → covers roughly z∈[-0.08, 0.62] with margin so
    # takeoff and touchdown stay in frame.
    # 鉛直飛行（地上 z≈0.01 → ピーク z≈0.54）を画角に収める。3Dペインは4:3横長なので
    # 鉛直視野が律速: 距離 d で可視高さ ≈ 0.83·d。0.85 → 約0.70m、lookat z=0.27 を中心に
    # z∈[-0.08, 0.62] を余裕を持って覆い、離陸・着地が切れない。
    cam = mujoco.MjvCamera()
    cam.azimuth = 130
    cam.elevation = -10
    cam.distance = 0.85
    cam.lookat[:] = [0.0, 0.0, 0.27]

    qcols = ["px", "py", "pz", "qw", "qx", "qy", "qz"]

    if args.test_frame:
        i = n // 2
        img = render_3d(model, data, renderer,
                        np.array([traj[c][i] for c in qcols]), cam)
        imageio.imwrite(args.out, img)
        print(f"[render_video] wrote test frame {args.out} ({img.shape})")
        return

    Wg = W3                       # graph pane same width as the 3D pane
    # Full-width title banner, drawn once and stacked on top of every frame so the
    # title spans both panes and never clips (the old per-pane suptitle overflowed).
    # 全幅タイトルバナーを一度だけ描き、各フレーム上端に重ねる。タイトルが両ペインに
    # またがり切れない（旧来のペイン内 suptitle は幅を超過して両端が切れていた）。
    banner = title_banner(title, W3 + Wg)
    writer = imageio.get_writer(args.out, fps=args.fps, codec="libx264",
                                quality=8, macro_block_size=None)
    for i in range(n):
        img3d = render_3d(model, data, renderer,
                          np.array([traj[c][i] for c in qcols]), cam)
        graphs = graph_frame(traj, i, Wg, H)
        # Match heights and concatenate side by side, then add the title banner on top.
        if graphs.shape[0] != img3d.shape[0]:
            graphs = graphs[:img3d.shape[0], :, :]
        body = np.concatenate([img3d, graphs], axis=1)
        if banner.shape[1] != body.shape[1]:
            banner = banner[:, :body.shape[1], :]
        frame = np.concatenate([banner, body], axis=0)
        writer.append_data(frame)
    writer.close()
    print(f"[render_video] wrote {args.out} ({n} frames @ {args.fps} fps)")


if __name__ == "__main__":
    main()
