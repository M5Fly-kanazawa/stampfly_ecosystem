#!/usr/bin/env python3
"""
poshold_attID.py — root-cause the POS_HOLD effective tilt->velocity gain deficit
(~0.4 g) from an ALT_HOLD tilt-wobble identification flight.

ALT_HOLD では roll/pitch スティック = 直接の傾き指令（位置ループに上書きされない）。
大振幅・単軸ウォブルから、K≈0.4g の主犯を切り分ける:

  cmd_tilt (angle_ref) --att loop--> true_tilt (gyro) --physics g*sin--> accel
                                       est_tilt (ESKF quat, contamination?)
  true horizontal accel --(int)--> inertial velocity   vs   flow/ESKF velocity

Independent references (no floor-texture dependence for the attitude part):
  - true tilt   = GYRO-integrated quaternion (bias-corrected gyro), AC at wobble freq
  - true accel  = accelerometer specific force rotated to NED (gravity is vertical)
Measures:
  A) attitude achievement  = true_tilt / cmd_tilt         (<1 => att loop under-drives)
  B) est truthfulness      = est_tilt  / true_tilt        (<1 => ESKF reads low / contam)
  C) accel per true tilt   = a_horiz / (g*sin(true_tilt)) (~1 validates physics+rotation)
  D) flow velocity scale   = v_eskf / v_inertial          (<1 => flow under-reads)

Usage: python3 analysis/scripts/poshold_attID.py <altlog.jsonl> [out_dir]
"""
import sys, os, json, math
import numpy as np

G = 9.80665
R2D = 180.0 / math.pi


def load(p):
    S = {}
    for line in open(p):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        S.setdefault(d.get("id"), []).append(d)
    return S


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw])


def quat_to_euler(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    s = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw


def Rnb(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def bandpass(x, fs, f1, f2):
    X = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1/fs)
    X[(f < f1) | (f > f2)] = 0
    return np.fft.irfft(X, len(x))


def gain_no_intercept(u, y):
    """y ~ k*u least squares, plus R^2."""
    k = np.dot(u, y) / np.dot(u, u)
    r2 = 1 - np.sum((y - k*u)**2) / np.sum((y - y.mean())**2)
    return k, r2


def main(path, out="analysis/out/poshold"):
    os.makedirs(out, exist_ok=True)
    S = load(path)
    t0 = S["imu"][0]["ts"]
    timu = np.array([(r["ts"]-t0)/1e6 for r in S["imu"]])
    quat = np.array([r["quat"] for r in S["imu"]])
    gyro = np.array([r["gyro"] for r in S["imu"]])      # bias-corrected body rate [rad/s]
    accel = np.array([r["accel"] for r in S["imu"]])    # specific force [m/s^2]
    tcr = np.array([(r["ts"]-t0)/1e6 for r in S["ctrl_ref"]])
    ar = np.array([r["angle_ref"] for r in S["ctrl_ref"]])   # [roll,pitch] rad
    tpv = np.array([(r["ts"]-t0)/1e6 for r in S["posvel"]])
    vel = np.array([r["vel"] for r in S["posvel"]])
    flq = (np.array([(r["ts"]-t0)/1e6 for r in S["flow"]]),
           np.array([r["quality"] for r in S["flow"]])) if "flow" in S else None

    fs = 1.0 / np.median(np.diff(timu))

    # ---- GYRO-integrated quaternion (independent attitude truth) ----
    qg = np.empty_like(quat)
    qg[0] = quat[0]                       # seed from ESKF at t0
    for k in range(1, len(timu)):
        dt = timu[k] - timu[k-1]
        w = gyro[k]
        ang = np.linalg.norm(w) * dt
        if ang > 1e-9:
            axis = w / np.linalg.norm(w)
            dq = np.array([math.cos(ang/2), *(axis*math.sin(ang/2))])
        else:
            dq = np.array([1.0, 0, 0, 0])
        qg[k] = quat_mul(qg[k-1], dq)
        qg[k] /= np.linalg.norm(qg[k])

    eul_est = np.array([quat_to_euler(q) for q in quat])     # ESKF
    eul_gyro = np.array([quat_to_euler(q) for q in qg])      # gyro-integrated

    cmd_r = np.interp(timu, tcr, ar[:, 0])
    cmd_p = np.interp(timu, tcr, ar[:, 1])

    # ---- find single-axis wobble windows (roll-dominant / pitch-dominant) ----
    # use a sliding test on the commanded tilt energy per axis
    def dominant_windows(prim, sec, thresh=0.08):
        # prim/sec = |cmd| of the primary/secondary axis (rad)
        active = (prim > thresh) & (prim > 2.0*sec)
        # group contiguous
        wins = []
        i = 0
        n = len(active)
        while i < n:
            if active[i]:
                j = i
                while j < n and active[j]:
                    j += 1
                if timu[j-1]-timu[i] > 0.4:
                    wins.append((timu[i], timu[j-1]))
                i = j
            else:
                i += 1
        return wins

    roll_wins = dominant_windows(np.abs(cmd_r), np.abs(cmd_p))
    pitch_wins = dominant_windows(np.abs(cmd_p), np.abs(cmd_r))

    print(f"=== {os.path.basename(path)} : ALT_HOLD tilt-ID ===")
    print(f"sample rate {fs:.0f} Hz, {timu[-1]:.1f} s")
    print(f"roll-dominant windows: {[f'{a:.1f}-{b:.1f}' for a,b in roll_wins]}")
    print(f"pitch-dominant windows: {[f'{a:.1f}-{b:.1f}' for a,b in pitch_wins]}")

    def analyze_axis(name, wins, cmd, est_col, gyro_col, vel_ned_idx):
        if not wins:
            print(f"\n[{name}] no clean window")
            return
        # concatenate windows
        mask = np.zeros(len(timu), bool)
        for a, b in wins:
            mask |= (timu >= a) & (timu <= b)
        idx = np.where(mask)[0]
        cu = cmd[idx]
        ge = eul_gyro[idx, gyro_col]
        ee = eul_est[idx, est_col]
        # de-mean (AC) — gyro integration carries slow drift; wobble is the AC part
        cu = cu - cu.mean(); ge = ge - ge.mean(); ee = ee - ee.mean()
        # bandpass around wobble band 0.2-3 Hz to kill drift + HF noise
        cu_b = bandpass(cu, fs, 0.15, 3.0)
        ge_b = bandpass(ge, fs, 0.15, 3.0)
        ee_b = bandpass(ee, fs, 0.15, 3.0)
        kA, r2A = gain_no_intercept(cu_b, ge_b)   # gyro/cmd  = achievement
        kB, r2B = gain_no_intercept(ge_b, ee_b)   # est/gyro  = truthfulness
        print(f"\n[{name}]  ({len(idx)} samples, cmd rms {cu_b.std()*R2D:.1f} deg, "
              f"gyro rms {ge_b.std()*R2D:.1f} deg)")
        print(f"  A) attitude achievement  true_tilt/cmd_tilt = {kA:.2f}  (R2={r2A:.2f})"
              f"   {'<-- att loop UNDER-drives' if kA<0.8 else ''}")
        print(f"  B) est truthfulness      est_tilt/true_tilt = {kB:.2f}  (R2={r2B:.2f})"
              f"   {'<-- ESKF reads LOW (contam)' if kB<0.8 else ''}")
        # C) accel per true tilt: horizontal a from accel rotated by GYRO attitude
        aH = np.array([(Rnb(qg[i]) @ accel[i])[vel_ned_idx] for i in idx])
        gsin = G*np.sin(ge + eul_gyro[idx, gyro_col].mean()*0)   # use true tilt AC
        gsin = G*np.sin(eul_gyro[idx, gyro_col])
        gsin = gsin - gsin.mean()
        aH = aH - aH.mean()
        aH_b = bandpass(aH, fs, 0.15, 3.0); gs_b = bandpass(gsin, fs, 0.15, 3.0)
        kC, r2C = gain_no_intercept(gs_b, aH_b)
        print(f"  C) accel / (g*sin true_tilt) = {kC:.2f}  (R2={r2C:.2f})  (~1 validates)")
        # D) flow scale: eskf velocity vs inertial-integrated accel (this axis)
        vN = np.interp(timu[idx], tpv, vel[:, vel_ned_idx])
        # inertial velocity = integral of true horizontal accel
        dt = np.median(np.diff(timu[idx])) if len(idx) > 1 else 1/fs
        vin = np.cumsum(aH - aH.mean()) * (1/fs)
        vin -= vin.mean(); vN = vN - vN.mean()
        vin_b = bandpass(vin, fs, 0.1, 2.0); vN_b = bandpass(vN, fs, 0.1, 2.0)
        kD, r2D = gain_no_intercept(vin_b, vN_b)
        print(f"  D) flow scale  v_eskf/v_inertial = {kD:.2f}  (R2={r2D:.2f})"
              f"   {'<-- flow UNDER-reads' if kD<0.8 and r2D>0.4 else ''}")
        return dict(name=name, achievement=kA, truthful=kB, accel_chk=kC, flow_scale=kD,
                    r2=(r2A, r2B, r2C, r2D))

    # roll -> body x rate -> euler roll(0); horizontal NED accel for roll ~ East(1)
    rA = analyze_axis("ROLL", roll_wins, cmd_r, 0, 0, 1)
    pA = analyze_axis("PITCH", pitch_wins, cmd_p, 1, 1, 0)

    print("\n=== verdict ===")
    for a in (rA, pA):
        if not a:
            continue
        ach, tru = a["achievement"], a["truthful"]
        eff = ach * tru   # rough loop-relevant tilt factor seen via est
        print(f"  {a['name']:5s}: achievement {ach:.2f} x truthful {tru:.2f}"
              f"  -> effective tilt factor ~{ach*tru:.2f}")
    print("  If achievement<<1: attitude loop is the culprit (fix att/rate gains).")
    print("  If achievement~1 but flow_scale<<1: optical-flow scale is the culprit.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "analysis/out/poshold")
