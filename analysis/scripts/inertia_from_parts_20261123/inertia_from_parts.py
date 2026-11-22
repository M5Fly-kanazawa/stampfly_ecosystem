#!/usr/bin/env python3
"""Moments of inertia of the StampFly from the part STL meshes and the measured part masses.
部品の STL（3D モデル）と実測の部品質量から、StampFly の慣性モーメントを計算する。

Definition (decided by the developer, 2026-11-23) / 定義（開発者が決定）:
  * Masses are the central measured values (kitchen scale):
      motor 17/6 g x 4, battery 8 g, frame 3.5 g, StampS3 module 3.25 g,
      front ToF board (STL: battery_adapter) 1 g, propeller 1.01/4 g x 4,
      main board = 37 g - (everything else).
    質量は実測の中央の値（モータ 17/6 g x 4、電池 8 g、フレーム 3.5 g、StampS3 3.25 g、
    前方 ToF 基板（STL では battery_adapter）1 g、プロペラ 1.01/4 g x 4、
    主基板 = 全質量 37 g - ほかの全部）。
  * Every part is uniform in density over its STL volume (the main-board remainder is spread
    uniformly over the main-board STL shape).
    各部品は STL の体積に一様。主基板の残りは主基板の STL の形に一様。
  * STL axes (mm): X = left, Y = up, Z = forward. Body frame FRD: x = +Z, y = -X, z = -Y.
    STL の軸（mm）: X = 左、Y = 上、Z = 前。機体座標 FRD: x = +Z、y = -X、z = -Y。
  * Tensor about the centre of gravity; no random numbers; one computation with the central masses.
    重心まわり。乱数なし。中央の質量での 1 回の計算。
  * The adopted value is rounded ONCE to 3 significant digits.
    採用値は有効 3 桁に 1 回だけ丸める。

Only numpy and the standard library are used. / numpy と標準ライブラリのみ使う。
Usage / 使い方:  python3 -I inertia_from_parts.py
"""
import os
import re
import struct
import sys
from collections import Counter, defaultdict

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
PARTS_DIR = os.path.join(REPO, "simulator", "shared", "assets", "meshes", "parts")

MM = 1e-3  # mm -> m
# STL (X left, Y up, Z forward) -> FRD (x forward, y right, z down); det = +1
# STL（X 左・Y 上・Z 前）-> FRD（x 前・y 右・z 下）
R_FRD = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)

# ---- measured masses [g] (central values) / 実測の質量 [g]（中央の値） ----
MASS_TOTAL_G = 37.0
MASS_MOTOR_EACH_G = 17.0 / 6.0
MASS_BATTERY_G = 8.0
MASS_FRAME_G = 3.5
MASS_M5_G = 3.25
MASS_ADAPTER_G = 1.0  # front ToF board / 前方 ToF 基板
MASS_PROP_EACH_G = 1.01 / 4.0

# ---- propeller (blade) constants / プロペラ（羽根）の定数 ----
# The four-blade propeller STL is not in the repository, so the shape enters as constants.
# They come from a four-blade mesh built from the measured photographs of the public repository
# kouhei1970/prop_sim, with the blade pivot placed so that the mean blade-tip height is Y = 6.4 mm.
# 羽根 4 枚の STL はリポジトリに無いので、形は定数で与える。公開リポジトリ kouhei1970/prop_sim の
# 写真の実測から作った羽根 4 枚の形の値。羽根の先端の高さの平均が Y = 6.4 mm になる位置に置く。
PROP_I_PER_MASS_AXIS_MM2 = 55.28   # about the rotor axis (STL Y), I/m [mm^2] / 回転軸まわり
PROP_I_PER_MASS_INPLANE_MM2 = 28.17  # about an axis in the rotor plane, I/m [mm^2] / 面内の軸まわり
PROP_CENTROID_Y_MM = 6.6252        # STL Y of the blade centroid (tip height 6.4 mm + 0.2252 mm) / 羽根の重心の高さ
# Rotor centres, STL (X, Z) [mm]: front-right, rear-right, rear-left, front-left
# ロータ中心 STL (X, Z) [mm]: 前右・後右・後左・前左
ROTOR_CENTRES_XZ_MM = [(-22.8042, 22.7975), (-22.8042, -22.8109), (22.8042, -22.8109), (22.8042, 22.7975)]

# 2-point (bifilar) suspension values, for the ratio only / 2点吊り法の値（比を出すためだけ）
BIFILAR = np.array([9.16e-6, 13.3e-6, 20.4e-6])

# Widths quoted from analysis/notes/mass_inertia_measurements_20261123.md chapter 3 (1e-6 kg m^2)
# 幅は解析メモ 3 章の引用（参考。この計算では使わない）
WIDTH_ENVELOPE_95 = {"Ixx": (10.82, 11.84), "Iyy": (14.38, 15.96), "Izz": (21.44, 23.61)}
WIDTH_FRAME_DIST = {"Ixx": (-0.13, 0.23), "Iyy": (-0.13, 0.24), "Izz": (-0.25, 0.44)}


# ---------------------------------------------------------------- STL / mesh helpers
def read_stl(path):
    """Binary or ASCII STL -> (n, 3, 3) array in mm. / STL を読む。"""
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        v = re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data)
        return np.array(v, dtype=float).reshape(-1, 3, 3)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3, 3).astype(float)


def cap_holes(tri, tol=1e-3):
    """Close boundary loops of a slightly open mesh with a centroid fan. / 開いた境界を重心ファンで塞ぐ。"""
    q = np.round(tri.reshape(-1, 3) / tol).astype(np.int64)
    u, ids = np.unique(q, axis=0, return_inverse=True)
    ids = ids.reshape(-1, 3)
    edges = Counter()
    for t in ids:
        for i in range(3):
            edges[(t[i], t[(i + 1) % 3])] += 1
    boundary = [(a, b) for (a, b), c in edges.items() if edges.get((b, a), 0) == 0]
    if not boundary:
        return tri
    adj = defaultdict(set)
    for a, b in boundary:
        adj[a].add(b)
        adj[b].add(a)
    seen, comps = set(), []
    for s in adj:
        if s in seen:
            continue
        stack, comp = [s], set()
        while stack:
            x = stack.pop()
            if x in comp:
                continue
            comp.add(x)
            stack += list(adj[x])
        seen |= comp
        comps.append(comp)
    new = []
    for comp in comps:
        centre = np.mean([u[i] * tol for i in comp], axis=0)
        for a, b in boundary:
            if a in comp:
                new.append([u[b] * tol, u[a] * tol, centre])
    return np.concatenate([tri, np.array(new)])


def mesh_moments(tri):
    """Signed volume, centroid and second moment integral(x x^T dV) of a closed mesh, uniform density.
    閉じたメッシュの体積・重心・2 次モーメント（一様密度）。Tetrahedra from the origin."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    d = np.einsum("ij,ij->i", a, np.cross(b, c))  # 6 x signed tetrahedron volume
    vol = d.sum() / 6
    centroid = ((a + b + c) / 4 * (d / 6)[:, None]).sum(0) / vol
    tot = a + b + c
    second = (np.einsum("n,ni,nj->ij", d, tot, tot) + sum(np.einsum("n,ni,nj->ij", d, v, v) for v in (a, b, c))) / 120
    return vol, centroid, second


def part_unit(path):
    """One STL part -> dict(volume m^3, centroid FRD m, K = inertia per unit mass about the centroid m^2).
    STL の 1 部品 -> 体積・重心（FRD）・重心まわりの質量あたりの慣性 K。"""
    tri = cap_holes(read_stl(path)) @ R_FRD.T * MM
    vol, c, second = mesh_moments(tri)
    cov = second / vol - np.outer(c, c)
    return dict(volume=vol, c=c, K=np.trace(cov) * np.eye(3) - cov)


def prop_unit(centre_xz_mm):
    """One propeller as an axisymmetric rigid body (constants above). / 軸対称の剛体としての 1 枚のプロペラ。"""
    x_stl, z_stl = centre_xz_mm
    c = R_FRD @ np.array([x_stl, PROP_CENTROID_Y_MM, z_stl]) * MM
    inplane, axis = PROP_I_PER_MASS_INPLANE_MM2 * MM**2, PROP_I_PER_MASS_AXIS_MM2 * MM**2
    return dict(c=c, K=np.diag([inplane, inplane, axis]))  # FRD z = rotor axis


# ---------------------------------------------------------------- assembly
def build_parts(prop_each_g=MASS_PROP_EACH_G):
    """List of (name, mass kg, centroid FRD m, K per kg). Main-board mass is the remainder.
    (名前、質量、重心、質量あたりの慣性) のリスト。主基板の質量は残り。"""
    def stl(name):
        return part_unit(os.path.join(PARTS_DIR, name + ".stl"))

    motors = [stl("motor_" + k) for k in ("fl", "fr", "rl", "rr")]
    props = [prop_unit(xz) for xz in ROTOR_CENTRES_XZ_MM]
    others_g = (MASS_FRAME_G + MASS_BATTERY_G + MASS_M5_G + MASS_ADAPTER_G
                + 4 * MASS_MOTOR_EACH_G + 4 * prop_each_g)
    pcb_g = MASS_TOTAL_G - others_g
    parts = []
    singles = [("frame", MASS_FRAME_G, "frame"), ("main_board_remainder", pcb_g, "pcb"),
               ("battery", MASS_BATTERY_G, "battery"), ("front_tof_board", MASS_ADAPTER_G, "battery_adapter"),
               ("stamps3_module", MASS_M5_G, "m5stamps3")]
    for label, mass_g, fname in singles:
        u = stl(fname)
        parts.append((label, mass_g * 1e-3, u["c"], u["K"]))
    for i, u in enumerate(motors):
        parts.append(("motor_%d" % (i + 1), MASS_MOTOR_EACH_G * 1e-3, u["c"], u["K"]))
    for i, u in enumerate(props):
        parts.append(("propeller_%d" % (i + 1), prop_each_g * 1e-3, u["c"], u["K"]))
    return parts, pcb_g


def assemble(parts):
    """Total mass, CG, tensor about the CG, and per-part (parallel-axis + own-shape) tensors.
    全質量・重心・重心まわりのテンソル・部品ごとの寄与（平行軸の項 + 自身の形の項）。"""
    mass = sum(p[1] for p in parts)
    cg = sum(p[1] * p[2] for p in parts) / mass
    tensor = np.zeros((3, 3))
    contrib = []
    for name, m, c, K in parts:
        d = c - cg
        t = m * ((d @ d) * np.eye(3) - np.outer(d, d)) + m * K
        contrib.append((name, m, t))
        tensor += t
    return mass, cg, tensor, contrib


def round_sig(x, digits=3):
    """Round to significant digits (once). / 有効桁へ丸める（1 回だけ）。"""
    from decimal import Decimal, ROUND_HALF_EVEN
    d = Decimal(repr(float(x)))
    exp = d.adjusted() - digits + 1
    return float(d.quantize(Decimal(1).scaleb(exp), rounding=ROUND_HALF_EVEN))


def main():
    parts, pcb_g = build_parts()
    mass, cg, tensor, contrib = assemble(parts)
    diag = np.diag(tensor)
    adopted = np.array([round_sig(v) for v in diag])

    print("== StampFly inertia from part STL + measured masses (central values) ==")
    print("total mass %.4f g; main-board remainder %.4f g" % (mass * 1e3, pcb_g))
    print("centre of gravity (FRD, from the STL origin) [mm]: x %.3f  y %.3f  z %.3f" % tuple(cg * 1e3))
    print("\n-- adopted values [kg m^2] (before rounding / after rounding to 3 significant digits)")
    for i, nm in enumerate(("Ixx", "Iyy", "Izz")):
        print("%s  %.6e  ->  %.3e" % (nm, diag[i], adopted[i]))
    print("YAML style: Ixx %.3ge-6, Iyy %.3ge-6, Izz %.3ge-6" % tuple(adopted * 1e6))
    print("Ixx + Iyy = %.3f e-6 >= Izz = %.3f e-6 : %s" % ((adopted[0] + adopted[1]) * 1e6, adopted[2] * 1e6,
                                                          adopted[0] + adopted[1] >= adopted[2]))

    print("\n-- per-part contribution [1e-6 kg m^2] (groups; sums of parallel-axis and own-shape terms)")
    groups = {}
    for name, m, t in contrib:
        key = re.sub(r"_\d$", "s", name)
        g = groups.setdefault(key, [0.0, np.zeros(3)])
        g[0] += m
        g[1] += np.diag(t)
    print("%-22s %8s %9s %9s %9s" % ("part", "mass g", "Ixx", "Iyy", "Izz"))
    for key, (m, d) in groups.items():
        print("%-22s %8.3f %9.3f %9.3f %9.3f" % (key, m * 1e3, *(d * 1e6)))
    print("%-22s %8.3f %9.3f %9.3f %9.3f" % ("TOTAL", mass * 1e3, *(diag * 1e6)))

    print("\n-- not adopted, recorded only (products of inertia and centre-of-gravity offset)")
    print("inertia-matrix elements about the CG (FRD): I13 (Ixz-type) %+.3f e-6, I12 %+.3f e-6, I23 %+.3f e-6 kg m^2"
          % (tensor[0, 2] * 1e6, tensor[0, 1] * 1e6, tensor[1, 2] * 1e6))
    print("(the STL is left-right symmetric, so I12 and I23 are ~0 here; the real values are unknown)")

    print("\n-- ratio of the 2-point (bifilar) value to the adopted value")
    for i, nm in enumerate(("Ixx", "Iyy", "Izz")):
        print("%s  bifilar %.2f e-6 / adopted %.2f e-6 = %.3f" % (nm, BIFILAR[i] * 1e6, adopted[i] * 1e6, BIFILAR[i] / adopted[i]))

    print("\n-- widths quoted from the analysis note (chapter 3; not used in the computation), 1e-6 kg m^2")
    for nm in ("Ixx", "Iyy", "Izz"):
        print("%s  95%% envelope of the 3 placements %s, frame-distribution extra width %s" % (nm, WIDTH_ENVELOPE_95[nm], WIDTH_FRAME_DIST[nm]))

    # Cross-check: propeller 0.96 g (0.24 g each) must reproduce the earlier working values
    # 検算: プロペラ 0.96 g（1 枚 0.24 g）で、以前の作業値 11.460 / 15.281 / 22.592 と一致すること
    parts96, _ = build_parts(prop_each_g=0.24)
    d96 = np.diag(assemble(parts96)[2]) * 1e6
    ref = np.array([11.460, 15.281, 22.592])
    ok = bool(np.all(np.abs(d96 - ref) < 0.0015))
    print("\n-- cross-check with propeller 0.96 g: %s vs reference %s -> %s" % (np.round(d96, 3), ref, "MATCH" if ok else "MISMATCH"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
