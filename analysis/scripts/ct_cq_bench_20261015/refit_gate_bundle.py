#!/usr/bin/env python3
"""Refit a `sf sils sysid-gate` bundle with overridable motor-chain constants of tools/sysid/plant_fit.py.
`sf sils sysid-gate` のログ束を、tools/sysid/plant_fit.py のモータ式定数を差し替えて再同定する。

Why / 目的:
  plant_fit.py inverts the firmware chain (duty -> thrust) to recover torque. _MOTOR_CT and
  _KAPPA follow the generated parameters, but _MOTOR_AM/_BM/_CM are hand-typed literals.
  After a Ct change in the yaml the recovered thrust is scaled by Ct_new/Ct_old and b is
  inflated by Ct_old/Ct_new. Override AM/BM here to match the Ct that the firmware used.
  plant_fit.py はファームの式を逆算してトルクを復元する。Ct だけ変えると Am/Bm が古いままになり
  b が Ct_old/Ct_new 倍に膨らむ。ここで Am/Bm を差し替えて正しい b を得る。

Usage / 使い方:
    python3 -I refit_gate_bundle.py <repo_root> <bundle.sflog.zip> [--am X] [--bm X] [--cm X] [--ct X] [--kappa X]
  <repo_root>: the checkout (with the variant patch applied) whose lib/ and tools/ are imported.
  Without overrides the constants of that checkout are used (mode a); with --am/--bm they are
  replaced (mode b). Requires the environment of `sf` (numpy, scipy, pandas).
  Example for V1 (VF gauge): --am 4.587968e-8 --bm 5.840089e-4
"""
import argparse
import json
import sys
import zipfile

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument('repo_root')
parser.add_argument('bundle')
parser.add_argument('--am', type=float)
parser.add_argument('--bm', type=float)
parser.add_argument('--cm', type=float)
parser.add_argument('--ct', type=float)
parser.add_argument('--kappa', type=float)
args = parser.parse_args()

# The repository's own modules are imported on purpose / 検証対象の checkout のモジュールを読み込む
sys.path[:0] = [args.repo_root + '/lib', args.repo_root + '/tools', args.repo_root + '/tools/log_analyzer']
from sysid.loader import load_aligned  # noqa: E402
import rate_sysid as rs  # noqa: E402
import sysid.plant_fit as pf  # noqa: E402

overrides = {'_MOTOR_AM': args.am, '_MOTOR_BM': args.bm, '_MOTOR_CM': args.cm,
             '_MOTOR_CT': args.ct, '_KAPPA': args.kappa}
for name, value in overrides.items():
    if value is not None:
        setattr(pf, name, value)
print('constants: CT=%g AM=%g BM=%g CM=%g KAPPA=%g' % (pf._MOTOR_CT, pf._MOTOR_AM, pf._MOTOR_BM,
                                                      pf._MOTOR_CM, pf._KAPPA))
with zipfile.ZipFile(args.bundle) as z:
    gains = json.loads(z.read('gains.json'))
df = load_aligned(args.bundle)
for axis in ('roll', 'pitch', 'yaw'):
    fit = rs.fit_from_df(df, axis, gains=gains[axis])
    print('  %-5s b=%.0f T_ms=%.3f L_ms=%.3f coh=%.3f' % (axis, fit['b'], fit['T'] * 1e3,
                                                          fit['L'] * 1e3, fit['coherence_mean']))
