#!/usr/bin/env python3
"""
Generate the SILS hover scenarios of part B (written outside the repo scenarios dir).
B 部の SILS ホバー シナリオを生成する (リポジトリの scenarios ディレクトリには置かない)。

  N : POS_HOLD hover, force gusts only (translational disturbance; NO torque disturbance)
  D : same + low-frequency torque disturbance by modulating motor thrust health
Usage: make_hover_scn.py <N|D> <out.scn>
"""
import sys
import numpy as np

SLOT_MS = 100                      # one RC hold / disturbance update slot
HOVER_START_S = 18.0               # same take-off + climb as sysid_gate.scn
HOVER_LEN_S = 56.0
GUST_FREQS_HZ = (0.23, 0.41, 0.77, 1.3, 1.9)   # force gust components (0.2..2 Hz)
GUST_AMP_N = 0.01                  # per component, per axis [N] (weight is 0.36 N)
HEALTH_FREQS_HZ = {0: 0.7, 1: 1.3, 3: 0.4}     # motor -> torque-disturbance frequency [Hz]
HEALTH_DEPTH = 0.03                # peak thrust-health deviation (3 %)
SEED = 20261101


def main(case, out):
    rng = np.random.default_rng(SEED)
    ph = rng.uniform(0, 2 * np.pi, (2, len(GUST_FREQS_HZ)))
    lines = [
        "0    rc   2048  2048   2048    2048  0     6000      50",
        '+    api  "command"',
        '+    api  "takeoff"',
        "+    rc   2048  2048   2048    2048  0     7000      50",
        '+    api  "up 65"',
        "+    rc   2048  2048   2048    2048  0     5000      50",
    ]
    n = int(HOVER_LEN_S * 1000 / SLOT_MS)
    for k in range(n):
        t = k * SLOT_MS * 1e-3
        fx = sum(GUST_AMP_N * np.sin(2 * np.pi * f * t + ph[0, i]) for i, f in enumerate(GUST_FREQS_HZ))
        fy = sum(GUST_AMP_N * np.sin(2 * np.pi * f * t + ph[1, i]) for i, f in enumerate(GUST_FREQS_HZ))
        lines.append(f"+    wind {fx:.5f} {fy:.5f} 0.0")
        if case == "D":
            for motor, f in HEALTH_FREQS_HZ.items():
                gain = 1.0 - HEALTH_DEPTH * 0.5 * (1.0 + np.sin(2 * np.pi * f * t))
                lines.append(f"+    fault {motor} {gain:.5f}")
        lines.append(f"+    rc   2048  2048   2048    2048  0     {SLOT_MS}      50")
    lines.append("+    wind 0.0 0.0 0.0")
    if case == "D":
        lines += [f"+    fault {m} 1.0" for m in range(4)]
    lines += ['+    api  "land"', "+    rc   2048  2048   2048    2048  0     8000      50"]
    open(out, "w").write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
