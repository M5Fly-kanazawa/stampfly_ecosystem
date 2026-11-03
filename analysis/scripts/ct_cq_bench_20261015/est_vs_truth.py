#!/usr/bin/env python3
"""Mean estimated (ESKF posvel) vs true altitude over the hover window. Usage: est_vs_truth.py <bundle dir> [more dirs]"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "lib"))
import sflog  # noqa: E402

for d in sys.argv[1:]:
    log = sflog.load(sorted(Path(d).glob("*.sflog.zip"))[-1])
    tr = log.streams["truth"].sort_values("timestamp_us")
    pv = log.streams["posvel"].sort_values("timestamp_us")
    t = tr["timestamp_us"].to_numpy() * 1e-6
    tp = pv["timestamp_us"].to_numpy() * 1e-6
    at = -tr["pos_z"].to_numpy()
    ae = -pv["pos_z"].to_numpy()
    m = (tp > 28) & (tp < 198)
    ae_i = ae[m]
    at_i = np.interp(tp[m], t, at)
    print("%s: est mean %.4f m, truth mean %.4f m, (est-truth) mean %.4f m, est std %.4f" % (
        Path(d).name, ae_i.mean(), at_i.mean(), (ae_i - at_i).mean(), ae_i.std()))
