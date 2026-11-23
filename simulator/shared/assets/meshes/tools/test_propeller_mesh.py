"""Checks of the propeller STL files and of their link to the simulator definitions.
プロペラ STL と、シミュレータの定義との整合の検査。

Run / 実行: python3 -m pytest -q simulator/shared/assets/meshes/tools   (numpy and scipy only)
"""
import json
import re
import struct
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
MESHES = HERE.parent
PARTS = MESHES / "parts"
REPO = HERE.parents[4]

GEOMETRY = {k: v["value"] for k, v in json.loads((MESHES / "propeller_geometry.json").read_text())["items"].items()}
DERIVED = json.loads((MESHES / "propeller_derived.json").read_text())
MAX_BYTES = 100 * 1024


def read_stl(path):
    """Binary STL -> (triangle vertices (n,3,3) float32, stored normals (n,3), declared count)."""
    data = Path(path).read_bytes()
    n = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * n, "not a binary STL of the declared size"
    rec = np.frombuffer(data[84:], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].copy(), rec["n"].copy(), n


def shells(tri):
    """Split triangles into connected shells by shared vertices. Returns a list of triangle-index arrays."""
    pts, inv = np.unique(tri.reshape(-1, 3), axis=0, return_inverse=True)
    face = inv.reshape(-1, 3)
    parent = np.arange(len(pts))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for a, b, c in face:
        for u, w in ((a, b), (b, c)):
            ru, rw = find(u), find(w)
            if ru != rw:
                parent[ru] = rw
    root = np.array([find(f) for f in face[:, 0]])
    return [np.where(root == r)[0] for r in np.unique(root)], face


def closed_consistent(face):
    """Every directed edge once, and its reverse once (closed, consistent winding)."""
    e = np.concatenate([face[:, [0, 1]], face[:, [1, 2]], face[:, [2, 0]]]).astype(np.int64)
    fwd, rev = e[:, 0] * 10**7 + e[:, 1], e[:, 1] * 10**7 + e[:, 0]
    return len(np.unique(fwd)) == len(fwd) and np.array_equal(np.sort(fwd), np.sort(rev))


def volume(tri):
    t = tri.astype(np.float64)
    return float(np.einsum("ij,ij->i", t[:, 0], np.cross(t[:, 1], t[:, 2])).sum() / 6.0)


ALL_FILES = ["propeller_rotor_ccw.stl", "propeller_rotor_cw.stl"] + [f"propeller_{p}.stl" for p in GEOMETRY["rotors"]]


@pytest.mark.parametrize("name", ALL_FILES)
def test_size_and_triangle_count(name):
    tri, _, declared = read_stl(PARTS / name)
    assert declared == len(tri) == DERIVED["files"][name]["triangles"]
    assert (PARTS / name).stat().st_size <= MAX_BYTES


@pytest.mark.parametrize("name", ALL_FILES)
def test_five_closed_shells_with_outward_normals(name):
    """Hub + 4 blades: 5 shells, each closed with consistent winding, positive volume (normals outward)."""
    tri, stored, _ = read_stl(PARTS / name)
    parts, face = shells(tri)
    assert len(parts) == 1 + GEOMETRY["blade_count"]
    for idx in parts:
        sub = face[idx]
        _, local = np.unique(sub.ravel(), return_inverse=True)
        assert closed_consistent(local.reshape(-1, 3))
        assert volume(tri[idx]) > 0
    # the stored normals follow the winding
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]).astype(np.float64)
    ok = np.linalg.norm(n, axis=1) > 1e-9
    n = n[ok] / np.linalg.norm(n[ok], axis=1, keepdims=True)
    assert np.all(np.einsum("ij,ij->i", n, stored[ok]) > 0.99)


@pytest.mark.parametrize("hand", ["ccw", "cw"])
def test_diameter_and_blade_symmetry(hand):
    tri, _, _ = read_stl(PARTS / f"propeller_rotor_{hand}.stl")
    v = tri.reshape(-1, 3).astype(np.float64)
    assert np.linalg.norm(v[:, [0, 2]], axis=1).max() * 2 == pytest.approx(GEOMETRY["diameter_mm"], abs=0.005)
    parts, _ = shells(tri)
    blades = [tri[i] for i in parts if len(i) > 400]
    assert len(blades) == 4
    vol = [volume(b) for b in blades]
    assert max(vol) - min(vol) < 1e-3 * max(vol)          # 4 equal blades (90 degrees apart)


def test_cw_is_mirror_of_ccw():
    ccw = read_stl(PARTS / "propeller_rotor_ccw.stl")[0].astype(np.float64)
    cw = read_stl(PARTS / "propeller_rotor_cw.stl")[0].astype(np.float64)
    mirrored = ccw * np.array([-1.0, 1.0, 1.0])
    key = lambda t: sorted(map(tuple, np.round(t.reshape(-1, 3), 4)))   # noqa: E731
    assert key(mirrored) == key(cw)
    assert volume(cw) == pytest.approx(volume(ccw), rel=1e-6)           # winding reversed with the mirror


@pytest.mark.parametrize("hand,sign", [("ccw", 1.0), ("cw", -1.0)])
def test_blade_orientation_gives_upward_thrust(hand, sign):
    """Leading edge on the rotation side and higher than the trailing edge; convex face up (+Y)."""
    tri, _, _ = read_stl(PARTS / f"propeller_rotor_{hand}.stl")
    parts, _ = shells(tri)
    blade = max((tri[i] for i in parts if len(i) > 400), key=lambda t: t.reshape(-1, 3)[:, 2].mean())
    v = blade.reshape(-1, 3).astype(np.float64)
    r = np.linalg.norm(v[:, [0, 2]], axis=1)
    near = np.abs(r - 0.70 * GEOMETRY["diameter_mm"] / 2) < 0.05      # the station at r/R = 0.70
    sel = v[near]
    az = np.arctan2(sel[:, 0], sel[:, 2])            # ccw seen from +Y: Z -> X
    arc = sign * (az - az.mean()) * r[near]
    le, te = np.argmax(arc), np.argmin(arc)
    assert sel[le, 1] > sel[te, 1] + 1.0             # leading edge on the rotation side and above the trailing edge
    slope = (sel[le, 1] - sel[te, 1]) / (arc[le] - arc[te])
    dev = sel[:, 1] - (sel[te, 1] + slope * (arc - arc[te]))
    assert dev.max() > 2 * abs(dev.min())            # the bulge is on the +Y (suction) side


@pytest.mark.parametrize("part", ["fr", "rr", "rl", "fl"])
def test_placed_files_equal_rotor_plus_offset(part):
    rotor = GEOMETRY["rotors"][part]
    base = read_stl(PARTS / f"propeller_rotor_{rotor['hand']}.stl")[0].astype(np.float64)
    placed = read_stl(PARTS / f"propeller_{part}.stl")[0].astype(np.float64)
    off = np.array([rotor["x_mm"], GEOMETRY["axis_height_mm"], rotor["z_mm"]])
    assert np.allclose(placed, base + off, atol=2e-5)
    motor = read_stl(PARTS / f"motor_{part}.stl")[0].reshape(-1, 3)
    top = motor[motor[:, 1] > motor[:, 1].max() - 1e-3]
    assert top[:, [0, 2]].mean(0) == pytest.approx([rotor["x_mm"], rotor["z_mm"]], abs=0.01)   # hub on the motor shaft


def test_tip_height_is_in_the_measured_range():
    tip = DERIVED["ccw"]["tip_y_range_at_axis_height_mm"]
    lo, hi = GEOMETRY["tip_height_target_mm"]
    assert lo <= 0.5 * (tip[0] + tip[1]) <= hi


def test_urdf_matches_derived_mass_properties():
    text = (PARTS / "stampfly.urdf").read_text()
    prop_links = re.findall(r'<link name="prop\d_link">.*?</link>', text, re.S)
    assert len(prop_links) == 4
    total = float(re.search(r'<mass value="([0-9.eE+-]+)"/>\s*\n\s*<inertia ixx="11.5e-6"', text).group(1))
    for link in prop_links:
        m = float(re.search(r'<mass value="([0-9.eE+-]+)"', link).group(1))
        ixx, iyy, izz = (float(x) for x in re.search(r'ixx="([^"]+)".*?iyy="([^"]+)".*?izz="([^"]+)"', link, re.S).groups())
        assert m == pytest.approx(GEOMETRY["mass_g"] * 1e-3)
        assert izz == pytest.approx(DERIVED["ccw"]["inertia_axis_g_mm2"] * 1e-9, rel=5e-3)
        assert ixx == iyy == pytest.approx(DERIVED["ccw"]["inertia_in_plane_g_mm2"] * 1e-9, rel=5e-3)
        total += m
    assert total == pytest.approx(0.037, abs=1e-9)    # vehicle total mass is kept


def test_parts_config_counts():
    cfg = json.loads((PARTS / "parts_config.json").read_text())
    for part in cfg["parts"]:
        if part["name"].startswith("propeller_"):
            assert part["triangles"] == DERIVED["files"][part["file"]]["triangles"]


def test_landing_copies_are_identical():
    for hand in ("ccw", "cw"):
        name = f"propeller_rotor_{hand}.stl"
        assert (REPO / "landing" / "assets" / "model" / name).read_bytes() == (PARTS / name).read_bytes()


def test_source_photos_hash():
    sources = json.loads((MESHES / "propeller_geometry.json").read_text())["primary_sources"]["files"]
    import hashlib
    for rel, info in sources.items():
        assert hashlib.sha256((MESHES / rel).read_bytes()).hexdigest() == info["sha256"]
