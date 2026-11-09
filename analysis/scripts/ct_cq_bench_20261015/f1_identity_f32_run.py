#!/usr/bin/env python3
"""Extract the F1 block from actuator.cpp, compile f1_identity_f32.cpp, run it.
actuator.cpp から F1 のブロックを抜き出し、f1_identity_f32.cpp をコンパイルして実行する。"""
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
src = (ROOT / "firmware/vehicle/components/sf_actuator/actuator.cpp").read_text()
a = src.index("namespace plant_inverse {")
b = src.index("// -----------------------------------------------------------------------------\n// mixerCompute")
with tempfile.TemporaryDirectory() as td:
    td = Path(td)
    (td / "f1_block.inc").write_text(src[a:b])
    exe = td / "f1_id32"
    here = Path(__file__).parent
    subprocess.check_call(["c++", "-std=c++17", "-O2", f"-I{td}", f"-I{ROOT}", str(here / "f1_identity_f32.cpp"), "-o", str(exe)])
    out = subprocess.check_output([str(exe)], text=True)
    print(out, end="")
    if len(sys.argv) > 1:
        Path(sys.argv[1]).write_text(out)
