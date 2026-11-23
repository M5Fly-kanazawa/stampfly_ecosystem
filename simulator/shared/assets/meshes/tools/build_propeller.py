#!/usr/bin/env python3
"""Build the StampFly propeller STL files (display version) from propeller_geometry.json.
StampFly のプロペラ形状ファイル(表示用に三角形を減らした版)を propeller_geometry.json から作る。

Dependencies / 依存: numpy and scipy (PCHIP) only. No trimesh, no boolean library.
Output / 出力 (all in simulator/shared/assets/meshes/parts/, binary STL, mm):
  propeller_rotor_ccw.stl, propeller_rotor_cw.stl   origin = hub centre, rotation axis = +Y
  propeller_{fr,rr,rl,fl}.stl                       the same shapes placed at the motor positions
  (../propeller_derived.json                        derived values: triangle counts, mass properties, tip height)

Frame / 座標: STL frame of the body (X = left, Y = up, Z = front).
  Hub frame used inside the builder: x_h = Z, y_h = X, z_h = Y (cyclic permutation, right-handed);
  the angle about z_h is positive = counter-clockwise seen from +Y (above).
  The CW propeller is the mirror image of the CCW one (X -> -X, winding reversed).

Shape / 形状: 4 closed shells for the blades + 1 closed shell for the hub. The shells overlap at the
  blade roots (the blade starts root_overlap_mm inside the hub); they are NOT united by a boolean
  operation, so each shell is watertight but the union has interpenetrating surfaces at the root.
  羽根 4 枚とハブの計 5 個の閉じた殻。根元で重なっている(ブーリアン和にはしていない)。

Unconfirmed assumptions / 未確定の仮定: pitch (nominal 0.9 inch), one section for all radii, hub shape,
  blade face orientation. See simulator/shared/README.md.

Usage / 使い方:
  python3 build_propeller.py                       # regenerate the 6 repository STL files + propeller_derived.json
  python3 build_propeller.py --out fine_ccw.stl --fidelity fine --hand ccw --pitch-offset-deg -4
                                                   # a detailed single file (about 23,000 triangles), anywhere
  Options of the single-file mode: --hand, --fidelity {display,fine}, --diameter-mm, --pitch-inch,
  --pitch-offset-deg, --place X Y Z (translate to a body position, mm). Reads only propeller_geometry.json.
  The detailed version is for computational fluid dynamics etc. and is NOT kept in the repository.
  詳細版(数値流体計算用など)はリポジトリに置かず、利用者が手元で作る。
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import PchipInterpolator

HERE = Path(__file__).resolve().parent
MESHES_DIR = HERE.parent
PARTS_DIR = MESHES_DIR / "parts"
GEOMETRY_JSON = MESHES_DIR / "propeller_geometry.json"
DERIVED_JSON = MESHES_DIR / "propeller_derived.json"

INCH_MM = 25.4
# Radial stations (r/R) of the blade loft; the first one is the root (set from the hub radius).
# 羽根を積層する半径位置。先頭は根元(ハブ半径から決める)。
# Fidelity presets: radial stations (None = every chord-table radius plus mid-points), chordwise
# points on each of the upper / lower surfaces, polygon sides of the hub.
# 忠実度の設定: 半径方向の積層位置、翼弦方向の点数(上面・下面それぞれ)、ハブの多角形の辺数。
FIDELITY = {
    "display": dict(stations=[0.226, 0.30, 0.40, 0.50, 0.60, 0.70, 0.78, 0.86, 0.92, 0.96, 0.985, 1.0],
                    points=9, hub_sections=24),
    "fine": dict(stations=None, points=41, hub_sections=96),
}
STL_HEADER = b"StampFly propeller (build_propeller.py)".ljust(80, b" ")


def load_geometry(path=GEOMETRY_JSON):
    """Read the geometry data file. / 形状データを読む。"""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return {name: item["value"] for name, item in data["items"].items()}


def section_curves(section_table, n_points):
    """Camber and thickness of the one measured section at n_points chord positions (cosine spacing).
    実測断面の(x/c, キャンバ, 厚み)を余弦分布の n_points 点で返す。前縁は 0 に閉じる。"""
    tab = np.asarray(section_table, float)
    xs, camber, thick = tab[:, 0], tab[:, 1], tab[:, 2].copy()
    thick[0] = 0.0                               # close the leading edge to a point / 前縁を点に閉じる
    root_x = np.sqrt(xs)                         # sqrt(x) axis gives a round nose / 前縁を丸くする
    f_camber = PchipInterpolator(root_x, camber)
    f_thick = PchipInterpolator(root_x, thick)
    x = 0.5 * (1.0 - np.cos(np.linspace(0.0, np.pi, n_points)))
    return x, f_camber(np.sqrt(x)), f_thick(np.sqrt(x))


def lag_angle(r_mm, radius_mm, r_root_mm, sweep_table, sweep_root_deg):
    """Lag angle psi(r) of the blade centre line [rad]: r dpsi/dr = tan(sweep) (polar definition).
    翼の中心線の遅れ角。後退角 = 中心線の接線と動径のなす角、r dpsi/dr = tan(後退角)。"""
    tab = np.asarray(sweep_table, float)
    rr = np.linspace(r_root_mm, radius_mm, 2001)
    sweep = np.deg2rad(np.interp(rr / radius_mm, tab[:, 0], tab[:, 1], left=sweep_root_deg))
    integrand = np.tan(sweep) / rr
    cumulative = np.concatenate([[0.0], np.cumsum(0.5 * (integrand[1:] + integrand[:-1]) * np.diff(rr))])
    return np.interp(r_mm, rr, cumulative)


def blade_shell(g, pitch_inch=None, pitch_offset_deg=None, fidelity="display", diameter_mm=None):
    """One closed blade shell in the hub frame (z_h = axis, positive angle = rotation direction).
    ハブ座標(z_h=回転軸)の羽根 1 枚の閉じた殻。(vertices, faces, stations) を返す。"""
    pitch_inch = g["pitch_inch"] if pitch_inch is None else pitch_inch
    pitch_offset_deg = g["pitch_offset_deg"] if pitch_offset_deg is None else pitch_offset_deg
    fid = FIDELITY[fidelity]
    radius = 0.5 * (g["diameter_mm"] if diameter_mm is None else diameter_mm)
    r_root = 0.5 * g["hub_diameter_mm"] - g["root_overlap_mm"]
    pivot = g["twist_pivot_chord_fraction"]
    chord_tab = np.asarray(g["chord_table_r_over_R_mm"], float)
    sweep_tab = np.asarray(g["sweep_table_r_over_R_deg"], float)

    if fid["stations"] is None:        # every table radius and the mid-points between them
        tab_r = chord_tab[:, 0]
        r_over_R = np.unique(np.concatenate([tab_r, 0.5 * (tab_r[1:] + tab_r[:-1]), [r_root / radius]]))
    else:
        r_over_R = np.array([r_root / radius] + fid["stations"][1:])
    r = r_over_R * radius
    chord_top = np.interp(r_over_R, chord_tab[:, 0], chord_tab[:, 1])     # top-view chord [mm]
    sweep = np.deg2rad(np.interp(r_over_R, sweep_tab[:, 0], sweep_tab[:, 1], left=g["sweep_root_deg"]))
    beta = np.minimum(np.rad2deg(np.arctan2(pitch_inch * INCH_MM, 2.0 * np.pi * r)), g["twist_cap_deg"])
    beta = np.deg2rad(beta + pitch_offset_deg)
    # The table chord is the width seen from above (photo): arc width W = c / cos(sweep); the chord
    # slanted by beta in the unrolled cylinder has true length L = W / cos(beta).
    # 表の翼弦長は上から見た幅。円弧幅 W=c/cos(後退角)、ねじり角 beta で傾いた実コード長 L=W/cos(beta)。
    length = chord_top / np.cos(sweep) / np.cos(beta)
    psi = -lag_angle(r, radius, r_root, sweep_tab, g["sweep_root_deg"])

    n = fid["points"]
    x, camber, thick = section_curves(g["section_table_x_camber_thickness"], n)
    upper, lower = camber + 0.5 * thick, camber - 0.5 * thick
    # ring order: leading edge, upper surface to trailing edge, lower surface back to the edge
    # 一周の順序: 前縁 -> 上面(後縁まで) -> 下面(前縁手前まで)
    h = np.concatenate([[0.0], upper[1:], lower[1:][::-1]])
    xx = np.concatenate([[x[0]], x[1:], x[1:][::-1]])
    rings = []
    for i in range(len(r)):
        chord_dir = np.array([-np.cos(beta[i]), -np.sin(beta[i])])    # LE -> TE in (tangent, axis)
        normal = np.array([-np.sin(beta[i]), np.cos(beta[i])])        # suction side (+axis side)
        tangent = (xx - pivot) * length[i] * chord_dir[0] + h * length[i] * normal[0]
        axial = (xx - pivot) * length[i] * chord_dir[1] + h * length[i] * normal[1]
        angle = psi[i] + tangent / r[i]                               # + = rotation direction
        rings.append(np.stack([r[i] * np.cos(angle), r[i] * np.sin(angle), axial], axis=1))
    rings = np.array(rings)
    n_station, n_ring, _ = rings.shape
    verts = rings.reshape(-1, 3)
    vid = np.arange(n_station * n_ring).reshape(n_station, n_ring)
    faces = []
    for i in range(n_station - 1):
        for j in range(n_ring):
            a, b = vid[i, j], vid[i, (j + 1) % n_ring]
            c, d = vid[i + 1, (j + 1) % n_ring], vid[i + 1, j]
            faces += [(a, b, c), (a, c, d)]

    def cap(i, flip):
        up = [vid[i, k] for k in range(n)]
        lo = [vid[i, 0]] + [vid[i, n_ring - k] for k in range(1, n)]
        f = [(up[0], up[1], lo[1])]
        for k in range(1, n - 1):
            f += [(up[k], up[k + 1], lo[k + 1]), (up[k], lo[k + 1], lo[k])]
        return [(a, c, b) for (a, b, c) in f] if flip else f

    for flip_root in (False, True):          # choose the cap orientation that closes the shell
        trial = np.array(faces + cap(0, flip_root) + cap(n_station - 1, not flip_root))
        if is_closed_consistent(trial):
            if signed_volume(verts, trial) < 0:
                trial = trial[:, ::-1]
            stations = dict(r_mm=r, beta_deg=np.rad2deg(beta), length_mm=length, lag_deg=np.rad2deg(psi))
            return verts, trial, stations
    raise RuntimeError("could not close the blade shell")


def hub_shell(g, fidelity="display"):
    """Closed hub shell (annulus cylinder), hub frame, centred on z_h = 0. / ハブ(穴あき円柱)。"""
    n, r_out, r_in = FIDELITY[fidelity]["hub_sections"], 0.5 * g["hub_diameter_mm"], g["hub_hole_radius_mm"]
    z0, z1 = -0.5 * g["hub_height_mm"], 0.5 * g["hub_height_mm"]
    ang = np.linspace(0, 2 * np.pi, n, endpoint=False)
    c, s = np.cos(ang), np.sin(ang)
    # vertex blocks: outer bottom, outer top, inner bottom, inner top
    verts = np.concatenate([np.stack([r_out * c, r_out * s, np.full(n, z0)], 1),
                            np.stack([r_out * c, r_out * s, np.full(n, z1)], 1),
                            np.stack([r_in * c, r_in * s, np.full(n, z0)], 1),
                            np.stack([r_in * c, r_in * s, np.full(n, z1)], 1)])
    ob, ot, ib, it = (np.arange(n) + k * n for k in range(4))
    faces = []
    for k in range(n):
        m = (k + 1) % n
        faces += [(ob[k], ob[m], ot[m]), (ob[k], ot[m], ot[k]),        # outer wall, normal outward
                  (ib[k], it[m], ib[m]), (ib[k], it[k], it[m]),        # hole wall, normal toward the axis
                  (ot[k], ot[m], it[m]), (ot[k], it[m], it[k]),        # top face, normal +z
                  (ob[k], ib[m], ob[m]), (ob[k], ib[k], ib[m])]        # bottom face, normal -z
    faces = np.array(faces)
    if signed_volume(verts, faces) < 0:
        faces = faces[:, ::-1]
    return verts, faces


def hub_to_stl_frame(verts):
    """(x_h, y_h, z_h) -> (X, Y, Z) = (y_h, z_h, x_h): a proper rotation. / 巡回置換(右手系のまま)。"""
    return verts[:, [1, 2, 0]]


def build_propeller(g, hand="ccw", pitch_inch=None, pitch_offset_deg=None, fidelity="display", diameter_mm=None):
    """Whole propeller (hub + 4 blades) in the STL frame, origin at the hub centre.
    Returns a list of shells [(vertices, faces), ...]. / ハブ中心を原点とする STL 座標の殻の一覧。"""
    bv, bf, _ = blade_shell(g, pitch_inch, pitch_offset_deg, fidelity, diameter_mm)
    shells = [hub_shell(g, fidelity)]
    for k in range(g["blade_count"]):
        a = 2.0 * np.pi * k / g["blade_count"]
        rot = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1.0]])
        shells.append((bv @ rot.T, bf))
    shells = [(hub_to_stl_frame(v), f) for v, f in shells]
    if hand == "cw":                                     # mirror image / 鏡像
        shells = [(v * np.array([-1.0, 1.0, 1.0]), f[:, ::-1]) for v, f in shells]
    return shells


def merge_shells(shells, offset=(0.0, 0.0, 0.0)):
    """Concatenate shells into one vertex / face array, translated by offset. / 殻を連結して平行移動する。"""
    verts, faces, base = [], [], 0
    for v, f in shells:
        verts.append(v + np.asarray(offset))
        faces.append(f + base)
        base += len(v)
    return np.concatenate(verts), np.concatenate(faces)


# ----------------------------------------------------------------------------
# Mesh checks and mass properties / メッシュの検査と質量特性
# ----------------------------------------------------------------------------
def is_closed_consistent(faces):
    """True if every directed edge appears once and its reverse appears once (closed, consistent winding).
    すべての有向辺が 1 回ずつ現れ、逆向きも 1 回ずつある(閉じていて向きが一貫)。"""
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    fwd = e[:, 0].astype(np.int64) * 10**7 + e[:, 1]
    rev = e[:, 1].astype(np.int64) * 10**7 + e[:, 0]
    return len(np.unique(fwd)) == len(fwd) and np.array_equal(np.sort(fwd), np.sort(rev))


def signed_volume(verts, faces):
    """Signed volume of a closed shell (positive when the normals point outward). / 符号付き体積。"""
    a, b, c = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    return float(np.sum(np.einsum("ij,ij->i", a, np.cross(b, c))) / 6.0)


def mass_properties(shells, mass_g):
    """Volume, centroid and inertia tensor about the origin (hub centre) at uniform density.
    The overlapping root volume is counted once per shell (small overestimate, see README).
    一様密度での体積・重心・慣性テンソル(ハブ中心まわり)。重なった根元は殻ごとに数える(わずかな過大評価)。
    Returns mm^3, mm, g*mm^2."""
    volume = 0.0
    first = np.zeros(3)
    second = np.zeros((3, 3))
    for v, f in shells:
        a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
        vol = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
        volume += vol.sum()
        first += (vol[:, None] * (a + b + c) / 4.0).sum(0)
        s = a + b + c
        cov = (np.einsum("ni,nj,n->ij", a, a, vol) + np.einsum("ni,nj,n->ij", b, b, vol)
               + np.einsum("ni,nj,n->ij", c, c, vol) + np.einsum("ni,nj,n->ij", s, s, vol)) / 20.0
        second += cov
    rho = mass_g / volume
    centroid = first / volume
    inertia = rho * (np.trace(second) * np.eye(3) - second)
    return dict(volume_mm3=volume, centroid_mm=centroid, density_g_per_mm3=rho, inertia_g_mm2=inertia)


def write_binary_stl(path, verts, faces):
    """Write a binary STL (little-endian float32), face normals from the winding. / バイナリ STL を書く。"""
    v = verts[faces].astype("<f4")
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    n = (n / np.where(norm > 0, norm, 1.0)).astype("<f4")
    rec = np.zeros(len(faces), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, v
    with open(path, "wb") as fp:
        fp.write(STL_HEADER)
        fp.write(np.uint32(len(faces)).tobytes())
        fp.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--geometry", default=str(GEOMETRY_JSON), help="geometry data file")
    ap.add_argument("--out", default=None, help="single-file mode: output STL path (needs --hand)")
    ap.add_argument("--hand", choices=["ccw", "cw"], default="ccw", help="single-file mode: spin seen from above")
    ap.add_argument("--fidelity", choices=sorted(FIDELITY), default="display", help="triangle density")
    ap.add_argument("--diameter-mm", type=float, default=None, help="tip diameter [mm] (default: data file)")
    ap.add_argument("--pitch-inch", type=float, default=None, help="pitch [inch] (default: data file, 0.9)")
    ap.add_argument("--pitch-offset-deg", type=float, default=None,
                    help="uniform offset of the twist angle [deg] (default: data file, 0)")
    ap.add_argument("--place", type=float, nargs=3, metavar=("X", "Y", "Z"), default=None,
                    help="single-file mode: translate to a body position (STL frame, mm)")
    ap.add_argument("--out-dir", default=str(PARTS_DIR), help="repository mode: output directory")
    args = ap.parse_args()
    g = load_geometry(args.geometry)
    kw = dict(pitch_inch=args.pitch_inch, pitch_offset_deg=args.pitch_offset_deg,
              fidelity=args.fidelity, diameter_mm=args.diameter_mm)
    if args.out:
        shells = build_propeller(g, args.hand, **kw)
        verts, faces = merge_shells(shells, args.place or (0.0, 0.0, 0.0))
        write_binary_stl(args.out, verts, faces)
        mp = mass_properties(shells, g["mass_g"])
        print(f"{args.out}: triangles={len(faces)} volume={mp['volume_mm3']:.2f} mm3 "
              f"max_radius={np.linalg.norm(verts[:, [0, 2]] - np.array(args.place or (0, 0, 0))[[0, 2]], axis=1).max():.3f} mm")
        return
    write_repository_files(g, Path(args.out_dir), kw)


def write_repository_files(g, out, kw):
    """Write the 2 origin-centred and 4 placed display STL files and propeller_derived.json.
    原点形 2 個と機体の位置に置いた 4 個の表示用 STL、および導出値の JSON を書く。"""
    out.mkdir(parents=True, exist_ok=True)
    derived = dict(note="generated by build_propeller.py; do not edit / build_propeller.py の出力。手で直さない",
                   pitch_inch=kw["pitch_inch"] or g["pitch_inch"],
                   pitch_offset_deg=g["pitch_offset_deg"] if kw["pitch_offset_deg"] is None else kw["pitch_offset_deg"],
                   fidelity=kw["fidelity"], files={})
    for hand in ("ccw", "cw"):
        shells = build_propeller(g, hand, **kw)
        verts, faces = merge_shells(shells)
        name = f"propeller_rotor_{hand}.stl"
        write_binary_stl(out / name, verts, faces)
        derived["files"][name] = dict(triangles=int(len(faces)), bytes=(out / name).stat().st_size)
        mp = mass_properties(shells, g["mass_g"])
        r = np.linalg.norm(verts[:, [0, 2]], axis=1)
        derived[hand] = dict(
            max_radius_mm=float(r.max()), y_range_mm=[float(verts[:, 1].min()), float(verts[:, 1].max())],
            volume_mm3=mp["volume_mm3"], centroid_mm=mp["centroid_mm"].tolist(),
            inertia_axis_g_mm2=float(mp["inertia_g_mm2"][1, 1]),
            inertia_in_plane_g_mm2=float(0.5 * (mp["inertia_g_mm2"][0, 0] + mp["inertia_g_mm2"][2, 2])))
        for part, rotor in g["rotors"].items():
            if rotor["hand"] != hand:
                continue
            offset = (rotor["x_mm"], g["axis_height_mm"], rotor["z_mm"])
            pv, pf = merge_shells(shells, offset)
            name = f"propeller_{part}.stl"
            write_binary_stl(out / name, pv, pf)
            derived["files"][name] = dict(triangles=int(len(pf)), bytes=(out / name).stat().st_size)
        tip = verts[r > r.max() - 1e-6]                  # vertices at the maximum radius
        derived[hand]["tip_y_range_at_axis_height_mm"] = [float(tip[:, 1].min() + g["axis_height_mm"]),
                                                           float(tip[:, 1].max() + g["axis_height_mm"])]
    if out.resolve() == PARTS_DIR.resolve():
        DERIVED_JSON.write_text(json.dumps(derived, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(derived, indent=2))


if __name__ == "__main__":
    main()
