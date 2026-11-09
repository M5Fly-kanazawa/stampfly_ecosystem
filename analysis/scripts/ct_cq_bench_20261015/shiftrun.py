#!/usr/bin/env python3
"""DIAGNOSTIC ONLY (classification of regression differences; no scenario/threshold in the repo is changed).

Copies the given scenarios to a scratch directory with every open-loop throttle value
(rc rows whose throttle raw is not 2048) rescaled as raw' = 2048 + K*(raw-2048), then runs them
with the ORIGINAL .expect. If a scenario that failed with the original inputs passes with the
rescaled inputs, the failure is an input-level (calibration) artifact of the thrust conversion.

Usage: shiftrun.py <K> <outdir> <scenario-stem> [...]    (run with SF_ROOT_OVERRIDE/PYTHONPATH set as for sf)
"""
import re
import shlex
import subprocess
import sys
from pathlib import Path

K = float(sys.argv[1])
out = Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
root = Path(__file__).resolve().parents[3]
scn_dir = root / "simulator/sils/scenarios"
for stem in sys.argv[3:]:
    lines = (scn_dir / (stem + ".scn")).read_text().splitlines()
    new = []
    for ln in lines:
        body = ln.split("#", 1)[0]
        f = body.split()
        if len(f) >= 8 and f[1] == "rc" and f[2].isdigit() and int(f[2]) != 2048:
            raw = int(f[2])
            nraw = int(round(2048 + K * (raw - 2048)))
            ln = re.sub(r"(\brc\s+)%d\b" % raw, r"\g<1>%d" % nraw, ln, count=1)
        new.append(ln)
    p = out / (stem + ".scn")
    p.write_text("\n".join(new) + "\n")
    # same invocation the scenario documents for itself (as `sf sils regression` parses it)
    extra = []
    for ln in lines:
        m = re.search(r"sf sils scenario\s+\S*?scenarios/%s\.scn(?P<rest>.*)" % re.escape(stem), ln)
        if m:
            extra = shlex.split(m.group("rest"))
            break
    if "--target" not in extra:
        extra = ["--target", "vehicle"] + extra
    r = subprocess.run(["sf", "sils", "scenario", str(p)] + extra +
                       ["--expect", str(scn_dir / (stem + ".expect"))],
                       capture_output=True, text=True, cwd=root)
    txt = r.stdout + r.stderr
    verdict = "PASS" if re.search(r"scenario .*: PASS", txt) else ("XFAIL?" if "KNOWN" in txt else "FAIL")
    print("=== %s (K=%.4f): %s" % (stem, K, verdict))
    for l in txt.splitlines():
        if "[FAIL]" in l or ("metric" in l and "[PASS]" in l):
            print(l[:190])
