#!/usr/bin/env python3
"""Check that propeller_geometry.json equals the values in a prop_sim clone (one-time copy check).
propeller_geometry.json の値が prop_sim の値と一致することを確かめる(写しの照合)。

This is NOT needed to build the STL files; build_propeller.py reads only propeller_geometry.json.
STL の生成には不要。build_propeller.py は propeller_geometry.json だけを読む。

Usage: python3 verify_against_prop_sim.py --prop-sim /path/to/prop_sim   (the clone, containing prop_sim/presets.py)
Compares exactly (no tolerance) / 許容差なしの完全一致で比べる。Exit code 1 on any mismatch.
"""
import argparse
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from build_propeller import GEOMETRY_JSON, INCH_MM, load_geometry  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--prop-sim", required=True, help="path of a prop_sim clone")
    args = ap.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, args.prop_sim)
    from prop_sim import presets as P      # noqa: E402  (the clone; read only)
    g = load_geometry(GEOMETRY_JSON)
    M = P.STAMPFLY_1209_MEASURED
    geo = P.stampfly_1209()
    results = []

    def check(name, ours, theirs):
        ok = np.array_equal(np.asarray(ours, float), np.asarray(theirs, float))
        results.append((name, ok, np.size(ours)))

    check("chord_table (17 rows x 2)", g["chord_table_r_over_R_mm"], P._STAMPFLY_1209_CHORD)
    check("sweep_table (8 rows x 2)", g["sweep_table_r_over_R_deg"], P.STAMPFLY_1209_SWEEP_DEG)
    check("section_table (24 rows x 3)", g["section_table_x_camber_thickness"], P._STAMPFLY_1209_SECTION)
    check("diameter_mm", g["diameter_mm"], M["diameter_mm"])
    check("blade_count", g["blade_count"], M["n_blades"])
    check("hub_diameter_mm", g["hub_diameter_mm"], M["hub_outer_diameter_mm"])
    check("max_chord_mm", g["max_chord_mm"], M["max_chord_mm"])
    check("projected_area_mm2", g["projected_area_mm2"], M["projected_area_mm2"])
    check("tip_sweep_deg", g["tip_sweep_deg"], M["tip_sweep_deg"])
    check("mass_estimate_prop_sim_g", g["mass_estimate_prop_sim_g"], M["estimated_mass_g"])
    check("polar_inertia_prop_sim_kgm2", g["polar_inertia_prop_sim_kgm2"], M["polar_inertia_kgm2"])
    check("section_summary", [g["section_summary"][k] for k in
                              ("thickness_max", "thickness_max_x", "camber_max", "camber_max_x", "alpha0_deg_inviscid")],
          [M["section_thickness_max"], M["section_thickness_max_x"], M["section_camber_max"],
           M["section_camber_max_x"], M["section_alpha0_deg_inviscid"]])
    # twist formula, pitch and cap: compare with the twist prop_sim builds (to float precision, not bit-exact)
    r_mm = geo.r_R * 0.5 * g["diameter_mm"]
    beta = np.minimum(np.rad2deg(np.arctan2(g["pitch_inch"] * INCH_MM, 2 * np.pi * r_mm)), g["twist_cap_deg"])
    close = bool(np.allclose(beta, geo.twist_deg, rtol=0, atol=1e-9))
    results.append(("twist formula (pitch 0.9 inch, cap 48 deg) vs stampfly_1209().twist_deg", close, len(beta)))
    # mass x R^2 factor 0.264 of the polar inertia
    factor = g["polar_inertia_prop_sim_kgm2"] / (g["mass_estimate_prop_sim_g"] * 1e-3 * (0.5 * g["diameter_mm"] * 1e-3) ** 2)
    print(f"polar inertia / (m R^2) in prop_sim = {factor:.4f}")
    bad = 0
    for name, ok, n in results:
        print(f"{'OK      ' if ok else 'MISMATCH'} {name} ({n} numbers)")
        bad += not ok
    print(f"{len(results) - bad}/{len(results)} items match")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
