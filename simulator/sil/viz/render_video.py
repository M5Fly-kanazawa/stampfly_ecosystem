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
    """Read trajectory.csv into a dict of numpy arrays keyed by column name.
    Fails with a clear diagnostic on an empty/header-only/ragged file (a run that
    crashed mid-write) instead of an opaque IndexError/inhomogeneous-shape error.
    空/見出しのみ/不揃いのファイル（書き込み途中で異常終了）は不明瞭な例外でなく明確に失敗。"""
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"{path}: empty trajectory (no header)")
        rows = [[float(x) for x in row] for row in reader if row]
    if not rows:
        raise ValueError(f"{path}: trajectory has a header but no data rows")
    if any(len(r) != len(header) for r in rows):
        raise ValueError(f"{path}: ragged rows (expected {len(header)} columns)")
    cols = np.array(rows).T
    return {name: cols[i] for i, name in enumerate(header)}


def render_3d(model, data, renderer, qpos, cam):
    """Set the body pose, run forward kinematics, render one RGB frame."""
    data.qpos[:7] = qpos
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, cam)
    return renderer.render()


def title_banner(text, w_px, h_px=48, fontsize=15):
    """Render a full-width title strip as an RGB array (drawn once, reused)."""
    dpi = 100
    fig = plt.figure(figsize=(w_px / dpi, h_px / dpi), dpi=dpi)
    fig.patch.set_facecolor("white")
    fig.text(0.5, 0.5, text, ha="center", va="center",
             fontsize=fontsize, fontweight="bold")
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return buf


def label_strip(text, w_px, h_px=30, facecolor="white", textcolor="black"):
    """Render a small colored caption strip (stacked above a 3D pane to name it).
    3D ペインの上に重ねる、推定器名の小さな見出し帯。"""
    dpi = 100
    fig = plt.figure(figsize=(w_px / dpi, h_px / dpi), dpi=dpi)
    fig.patch.set_facecolor(facecolor)
    fig.text(0.5, 0.5, text, ha="center", va="center",
             fontsize=12, fontweight="bold", color=textcolor)
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
    # Signed roll & pitch (not the tilt magnitude). The magnitude uses arccos, which
    # is always ≥0: it folds the estimate's near-zero ± noise onto the positive side
    # ("pulses") and drops the sign; roll/pitch keep both. The axis auto-scales per
    # run (symmetric, ≥±0.02° floor): the ESKF run leans a real ~0.1° (the yaw spin
    # induces it, fed back through control), the complementary run stays within
    # ~0.01° — so each video resolves its own attitude detail without clipping.
    # 符号付きロール・ピッチ（傾斜の大きさではない）。大きさは arccos で常に≥0なので、推定の
    # ゼロ近傍の±ノイズを正側へ折り返し（「パルス」化）符号も落とす。roll/pitch は両方残す。
    # 縦軸は実行ごとに自動スケール（0中心・下限±0.02°）: ESKF 実行は実際に~0.1°傾く
    # （ヨー旋回が制御を介して誘起）、相補実行は~0.01°内 — 各動画が切れずに姿勢の詳細を出す。
    rp = np.concatenate([traj["roll"], traj["pitch"],
                         traj["roll_est"], traj["pitch_est"]])
    rp_max = max(0.02, 1.15 * float(np.abs(rp).max()))
    panel(axes[0, 1], "roll / pitch [deg]",
          [("roll", traj["roll"], "C0-"), ("pitch", traj["pitch"], "C3-"),
           ("roll est", traj["roll_est"], "C0--"), ("pitch est", traj["pitch_est"], "C3--")],
          ylim=(-rp_max, rp_max))
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


# --- side-by-side comparison (P4) / 並置比較動画（P4） ------------------------
# Two estimators (A, B) flown through the SAME bench, same flight. The twin 3D
# panes (top) show the two runs in lockstep; the overlay graphs (bottom) put both
# estimators' estimates on the SAME truth axes — so "different algorithm, same
# flight, both fly" reads at a glance (RESET_PLAN §9, P4 algorithm-independence).
# 2つの推定器(A,B)を同じベンチ・同じ飛行で走らせる。上のツイン3Dは2機がロックステップで
# 飛ぶ様子、下の重ね描きグラフは両推定値を同一の真値軸へ重ねる ＝「中身が違っても同じ
# 飛行で両方飛ぶ」を一目で（RESET_PLAN §9, P4 アルゴリズム非依存）。
def overlay_graph_frame(A, B, i, w_px, h_px, la, lb):
    """Render the comparison graphs up to frame i: truth (solid) with A and B
    estimates overlaid (dashed / dotted), plus a synchronized time cursor."""
    t = A["t"]
    now = t[i]
    dpi = 100
    fig, axes = plt.subplots(1, 4, figsize=(w_px / dpi, h_px / dpi), dpi=dpi)

    def panel(ax, ylabel, series, ylim=None):
        for label, tt, y, style in series:
            ax.plot(tt, y, style, label=label, linewidth=1.3)
        ax.axvline(now, color="0.5", linewidth=1.0)          # time cursor
        ax.set_xlim(t[0], t[-1])
        if ylim:
            ax.set_ylim(*ylim)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_xlabel("time [s]", fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=6, loc="upper right")
        ax.tick_params(labelsize=6)

    # Consistent legend wording across all four panels: a reference series
    # (truth / command) plus the two run labels (la, lb). No per-panel "est" suffix.
    # 4パネルで凡例表記を統一: 基準（truth/command）＋2実行ラベル(la, lb)。"est"接尾辞なし。
    panel(axes[0], "altitude [m]",
          [("truth", A["t"], A["alt"], "k-"),
           (la, A["t"], A["alt_est"], "C0--"),
           (lb, B["t"], B["alt_est"], "C1:")],
          ylim=(-0.03, 0.62))
    # Roll/pitch auto-scale across BOTH runs' truth + estimates so neither clips.
    # The ESKF leans a real ~0.1° (yaw spin fed through control); complementary
    # stays ~0.01° — the shared axis makes that difference visible, not hidden.
    # nan_to_num so a partial/NaN estimate can't poison the axis limit (order-safe).
    # 縦軸は両実行の真値＋推定を跨いで自動スケール（切れ防止）。ESKF は実際に~0.1°傾き、
    # 相補は~0.01°内 — 共通軸でその差が見える。nan_to_num で NaN が軸を壊さない。
    rp = np.concatenate([A["roll"], A["pitch"], A["roll_est"], A["pitch_est"],
                         B["roll_est"], B["pitch_est"]])
    rpm = max(0.05, 1.15 * float(np.nan_to_num(np.abs(rp), nan=0.0).max()))
    panel(axes[1], "roll [deg]",
          [("truth", A["t"], A["roll"], "k-"),
           (la, A["t"], A["roll_est"], "C0--"),
           (lb, B["t"], B["roll_est"], "C1:")],
          ylim=(-rpm, rpm))
    panel(axes[2], "pitch [deg]",
          [("truth", A["t"], A["pitch"], "k-"),
           (la, A["t"], A["pitch_est"], "C0--"),
           (lb, B["t"], B["pitch_est"], "C1:")],
          ylim=(-rpm, rpm))
    panel(axes[3], "yaw rate [rad/s]",
          [("command", A["t"], A["yawcmd"], "k-"),
           (la, A["t"], A["yawrate"], "C0--"),
           (lb, B["t"], B["yawrate"], "C1:")],
          ylim=(-0.2, 1.4))

    fig.tight_layout(pad=0.5)
    fig.canvas.draw()
    buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    return buf


def render_compare(args):
    """Render the P4 side-by-side video: twin 3D panes (A | B) over full-width
    overlay graphs. Both runs share one flight, so the panes move in lockstep."""
    A = load_trajectory(os.path.join(args.bundle, "trajectory.csv"))
    B = load_trajectory(os.path.join(args.compare, "trajectory.csv"))
    n = min(len(A["t"]), len(B["t"]))                    # same flight → equal, but be safe
    A = {k: v[:n] for k, v in A.items()}                 # one time base: graphs ↔ 3D agree
    B = {k: v[:n] for k, v in B.items()}                 # （長さが食い違っても自己整合）
    la, lb = args.label_a, args.label_b

    def verdict(bundle):
        p = os.path.join(bundle, "results.json")
        return json.load(open(p)) if os.path.exists(p) else {}
    ra, rb = verdict(args.bundle), verdict(args.compare)
    va = "PASS" if ra.get("pass") else "FAIL"
    vb = "PASS" if rb.get("pass") else "FAIL"
    title = (f"StampFly SIL  ·  same flight, two estimators  ·  "
             f"{la} (G3 {va})  vs  {lb} (G3 {vb})  ·  algorithm-independent")

    model = mujoco.MjModel.from_xml_path(args.model)
    data = mujoco.MjData(model)
    H3 = args.height                 # 3D pane height
    W3 = int(H3 * 4 / 3)             # 3D pane 4:3
    renderer = mujoco.Renderer(model, height=H3, width=W3)
    # Tighter framing than the single-bundle render (distance 0.85): the twin panes
    # are the hero shot, so pull in to make the drones larger and shrink the empty
    # sky/floor. distance 0.74 → visible height ≈ 0.83·0.74 ≈ 0.61 m centered on
    # lookat z=0.28 → covers z∈[−0.03, 0.59]: peak hover (≈0.55) and touchdown
    # (≈0.01) both stay in frame with margin.
    # 単一描画(距離0.85)より寄せる: ツインペインが主役なので機体を大きく、空/床の余白を縮小。
    cam = mujoco.MjvCamera()
    cam.azimuth = 130
    cam.elevation = -10
    cam.distance = 0.74
    cam.lookat[:] = [0.0, 0.0, 0.28]
    qcols = ["px", "py", "pz", "qw", "qx", "qy", "qz"]

    Wtot = 2 * W3
    Hg = args.graph_height
    banner = title_banner(title, Wtot, fontsize=13)
    # Pane captions: A bluish (C0), B orange (C1) — matching the graph line colors.
    # ペイン見出し: A=青系(C0), B=橙系(C1)。グラフの線色と対応させる。
    cap_a = label_strip(f"{la}", W3, facecolor="#dbe7ff")
    cap_b = label_strip(f"{lb}", W3, facecolor="#ffe7cc")

    writer = imageio.get_writer(args.out, fps=args.fps, codec="libx264",
                                quality=8, macro_block_size=None)
    for i in range(n):
        a3d = render_3d(model, data, renderer,
                        np.array([A[c][i] for c in qcols]), cam)
        b3d = render_3d(model, data, renderer,
                        np.array([B[c][i] for c in qcols]), cam)
        pane_a = np.concatenate([cap_a, a3d], axis=0)
        pane_b = np.concatenate([cap_b, b3d], axis=0)
        top = np.concatenate([pane_a, pane_b], axis=1)       # twin 3D row
        graphs = overlay_graph_frame(A, B, i, Wtot, Hg, la, lb)
        # Match widths defensively (matplotlib/render rounding), then stack.
        w = min(top.shape[1], graphs.shape[1], banner.shape[1])
        body = np.concatenate([top[:, :w], graphs[:, :w]], axis=0)
        frame = np.concatenate([banner[:, :w], body], axis=0)
        # libx264 needs even width/height; trim a row/column if odd.
        frame = frame[:frame.shape[0] // 2 * 2, :frame.shape[1] // 2 * 2]
        writer.append_data(frame)
    writer.close()
    print(f"[render_video] wrote {args.out} (compare, {n} frames @ {args.fps} fps)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--bundle", required=True, help="dir with trajectory.csv + results.json")
    ap.add_argument("--out", required=True, help="output MP4 path")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--test-frame", action="store_true", help="render one frame to a PNG and exit")
    # Side-by-side comparison (P4): --compare is run B's bundle (A is --bundle).
    # 並置比較（P4）: --compare は実行Bのバンドル（Aは --bundle）。
    ap.add_argument("--compare", default=None, help="run B's bundle dir → side-by-side video")
    ap.add_argument("--label-a", default="ESKF", help="caption for run A")
    ap.add_argument("--label-b", default="Complementary", help="caption for run B")
    ap.add_argument("--graph-height", type=int, default=300, help="overlay-graph row height (compare)")
    args = ap.parse_args()

    if args.compare:
        render_compare(args)
        return

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
