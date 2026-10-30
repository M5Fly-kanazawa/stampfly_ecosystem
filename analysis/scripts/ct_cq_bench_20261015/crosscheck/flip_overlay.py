import sys, os, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'flipcheck'))
import flipcheck as fc
import numpy as np
out = {}
for name, (a, b) in (("flip_test1", (15.80, 16.30)), ("flip_test2", (6.60, 7.12))):
    L, _ = fc.load(name); t = L["t"]
    am = fc.alpha_meas(L["gyro_x"])
    win = (t >= a) & (t <= b)
    rec = dict(t=(t[win] - a).round(4).tolist(), meas=am[win].round(1).tolist())
    for key, mp in (("now", fc.MODELS["M_now"]), ("rout", fc.MODELS["M_rout_eta1.0"])):
        w, _ = fc.simulate(mp["kind"], L["duty"], L["V"])
        T = mp["eta"] * mp["Ct"] * w * w
        p = fc.smooth(fc.ARM * (T @ fc.SR) / fc.IXX)
        rec[key] = p[win].round(1).tolist()
    m = np.array(rec["meas"])
    for key in ("now", "rout"):
        p = np.array(rec[key]); rec[key + "_rel_rms"] = float(np.sqrt(np.mean((m - p) ** 2)) / np.sqrt(np.mean(m ** 2)))
    rec["t0"] = a
    out[name] = rec
    print(name, 'n', len(rec['t']), 'unfitted rel RMS: now %.2f rout %.2f' % (rec['now_rel_rms'], rec['rout_rel_rms']), 'peak meas %.0f now %.0f rout %.0f' % (max(abs(m)), max(abs(np.array(rec['now']))), max(abs(np.array(rec['rout'])))))
json.dump(out, open(os.path.join(HERE, 'results', 'flip_overlay.json'), 'w'))
