#!/usr/bin/env python3
"""
alt_hold_analysis.py
=====================
StampFly POS_HOLD/ALT_HOLD 高度上下動の反証形式定量解析。

結論を先に決めず、以下を測定する:
  1. 高度誤差の特性（RMS, p2p, 支配周波数）
  2. 電圧依存性
  3. ヨートルク飽和イベントとの連成検定（新仮説）
  4. センサ整合性（ToF/baro/ESKF高度）
  5. 制御構造の観察（追従・飽和・位相）
  6. 5つの機構仮説の supported/refuted/inconclusive 判定
  7. 代表飛行の重ね描きプロット

このスクリプトは stdout にすべての数値根拠を出力する（後で report にコピーする）。
"""
import sys
import json
import numpy as np
from scipy import signal, stats

sys.path.insert(0, "/Users/kouhei/tmp/github/stampfly_ecosystem/analysis/scripts/yaw_nt_kanazawa")
from yawlib import load_jsonl, detect_flight_segments, mode_str  # noqa: E402

LOG_DIR = "/Users/kouhei/tmp/github/stampfly_ecosystem/logs"
OUT_DIR = "/private/tmp/claude-501/-Users-kouhei-tmp-github-stampfly-ecosystem/118f45f4-e4af-456f-81fa-701422871251/scratchpad"

# ---------------------------------------------------------------------------
# ログ定義（プロンプトで与えられた絶対 ts[s] の飛行区間・電圧帯・ヨー飽和イベント窓）
# Log definitions: absolute ts[s] flight windows, voltage band, yaw-saturation event windows
# ---------------------------------------------------------------------------
LOGS = {
    "020050": dict(
        file="stampfly_udp_20260627T020050.jsonl",
        label="night ALT_HOLD 4.2V",
        flight=(32.9, 47.4),
        voltage_band="4.2V",
        yaw_events=[(32.9, 40.0)],
    ),
    "020137": dict(
        file="stampfly_udp_20260627T020137.jsonl",
        label="night POS_HOLD 4.2V",
        flight=(80.2, 258.3),
        voltage_band="4.2V",
        yaw_events=[],
    ),
    "164611": dict(
        file="stampfly_udp_20260627T164611.jsonl",
        label="venue POS_HOLD 3.6-3.8V",
        flight=(33.1, 199.6),
        voltage_band="3.6-3.8V",
        yaw_events=[(114.1, 124.8), (182.2, 194.9)],
    ),
    "165713": dict(
        file="stampfly_udp_20260627T165713.jsonl",
        label="venue POS_HOLD 3.5-4.0V",
        flight=[(21.2, 54.8), (56.3, 196.5)],
        voltage_band="3.5-4.0V",
        yaw_events=[(29, 50), (92.2, 103.8), (173.5, 185.5)],
    ),
    "145526": dict(
        file="stampfly_udp_20260629T145526.jsonl",
        label="home POS_HOLD (comparison)",
        flight=(34.98, 213.6),  # will be auto-detected below; placeholder
        voltage_band="unknown",
        yaw_events=[],
    ),
}

GAP_S = 0.5  # UDP欠損ギャップ閾値。これをまたぐ微分/補間はしない


# ---------------------------------------------------------------------------
# ユーティリティ
# ---------------------------------------------------------------------------
def interp_with_gap_mask(t_src, y_src, t_query, max_gap=GAP_S):
    """
    t_src(昇順) 上の y_src を t_query に線形補間する。
    ブラケットする区間の gap が max_gap を超える場合は NaN にする
    (UDP 欠損ギャップをまたぐ補間の禁止)。
    """
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    t_query = np.asarray(t_query, dtype=float)

    y_out = np.interp(t_query, t_src, y_src, left=np.nan, right=np.nan)
    idx = np.searchsorted(t_src, t_query)
    idx = np.clip(idx, 1, len(t_src) - 1)
    gap = t_src[idx] - t_src[idx - 1]
    invalid = (gap > max_gap) | (t_query < t_src[0]) | (t_query > t_src[-1])
    y_out[invalid] = np.nan
    return y_out


def zoh_with_gap_mask(t_src, y_src, t_query, max_gap=GAP_S):
    """ゼロ次ホールド（直前値）で t_query に写像。gap超過はNaN。"""
    t_src = np.asarray(t_src, dtype=float)
    y_src = np.asarray(y_src, dtype=float)
    t_query = np.asarray(t_query, dtype=float)
    idx = np.searchsorted(t_src, t_query, side="right") - 1
    idx_c = np.clip(idx, 0, len(t_src) - 1)
    y_out = y_src[idx_c].copy()
    bad = (idx < 0) | (idx >= len(t_src) - 1)
    gap = np.full_like(t_query, np.nan)
    ok = (idx >= 0) & (idx < len(t_src) - 1)
    gap[ok] = t_src[idx_c[ok] + 1] - t_src[idx_c[ok]]
    invalid = bad | (gap > max_gap)
    y_out = y_out.astype(float)
    y_out[invalid] = np.nan
    return y_out


def contiguous_valid_runs(mask):
    """boolマスクから連続True区間の(start,end)インデックスを返す(end含まない)。"""
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return []
    breaks = np.where(np.diff(idx) > 1)[0]
    starts = np.concatenate(([idx[0]], idx[breaks + 1]))
    ends = np.concatenate((idx[breaks], [idx[-1]])) + 1
    return list(zip(starts, ends))


def welch_dominant(t_grid, y, fs, band=(0.05, 5.0)):
    """
    最長の連続有効区間で Welch PSD を計算し、指定帯域内の支配周波数とその
    帯域内エネルギー比率を返す。NaN 区間は除外。
    """
    valid = ~np.isnan(y)
    runs = contiguous_valid_runs(valid)
    if not runs:
        return None
    s, e = max(runs, key=lambda r: r[1] - r[0])
    seg = y[s:e]
    if len(seg) < 32:
        return None
    seg = seg - np.mean(seg)
    nperseg = min(len(seg), 4096)
    noverlap = nperseg // 2
    f, pxx = signal.welch(seg, fs=fs, nperseg=nperseg, noverlap=noverlap, detrend="linear")
    band_mask = (f >= band[0]) & (f <= band[1])
    if not np.any(band_mask):
        return None
    f_band = f[band_mask]
    p_band = pxx[band_mask]
    dom_idx = np.argmax(p_band)
    total_power = np.trapz(pxx, f)
    band_power = np.trapz(p_band, f_band)
    return dict(
        f_dominant=float(f_band[dom_idx]),
        p_dominant=float(p_band[dom_idx]),
        band_power_frac=float(band_power / total_power) if total_power > 0 else np.nan,
        seg_len_s=float(len(seg) / fs),
        f=f, pxx=pxx,
    )


def autocorr_period(y, fs, min_period_s=0.2, max_period_s=20.0):
    """
    自己相関関数の最初の非ゼロ極大から周期性の強さと周期を推定する。
    Welch PSDは低周波(赤色雑音的なドリフト)に引っ張られやすいため、
    「本当に周期的か、それとも不規則な低周波揺らぎか」の判別に使う独立指標。
    戻り値: (period_s, peak_normalized_autocorr) or (None, None)
    """
    valid = ~np.isnan(y)
    runs = contiguous_valid_runs(valid)
    if not runs:
        return None, None
    s, e = max(runs, key=lambda r: r[1] - r[0])
    seg = y[s:e]
    seg = seg - np.mean(seg)
    if len(seg) < 32:
        return None, None
    ac = np.correlate(seg, seg, mode="full")
    ac = ac[len(ac) // 2:]
    ac = ac / ac[0]
    min_lag = max(1, int(min_period_s * fs))
    max_lag = min(len(ac) - 1, int(max_period_s * fs))
    if max_lag <= min_lag:
        return None, None
    # 最初の極大 (直後に下降に転じる点) を探す
    sub = ac[min_lag:max_lag]
    peaks, props = signal.find_peaks(sub)
    if len(peaks) == 0:
        return None, None
    first_peak = peaks[np.argmax(sub[peaks])]  # 最も強い極大を採用
    lag = min_lag + first_peak
    return float(lag / fs), float(sub[first_peak])


def circular_shift_indices(n, shift):
    return (np.arange(n) + shift) % n


# ---------------------------------------------------------------------------
# ログごとのメイン特徴抽出
# ---------------------------------------------------------------------------
class FlightData:
    def __init__(self, key, meta):
        self.key = key
        self.meta = meta
        self.d = load_jsonl(f"{LOG_DIR}/{meta['file']}")

        # flight windows: 単一 tuple か tuple のリストの両対応
        fw = meta["flight"]
        self.windows = [fw] if isinstance(fw[0], (int, float)) else list(fw)

        self._build()

    def _mask_windows(self, ts):
        m = np.zeros(len(ts), dtype=bool)
        for t0, t1 in self.windows:
            m |= (ts >= t0) & (ts <= t1)
        return m

    def _build(self):
        d = self.d
        cr = d["ctrl_ref"]
        pv = d["posvel"]
        st = d["status"]
        tofb = d["tof_b"]

        t_cr = cr["ts"]
        mask_cr = self._mask_windows(t_cr)
        self.t = t_cr[mask_cr]

        self.alt_sp = cr["alt_sp"][mask_cr]  # 参照値は既にctrl_ref格子上なのでそのままスライス
        self.alt_vel_target = cr["alt_vel_target"][mask_cr]
        self.climb_cmd = cr["climb_cmd"][mask_cr]
        self.total_thrust = cr["total_thrust"][mask_cr]
        self.motor_duty = cr["motor_duty"][mask_cr]  # (N,4)
        self.mode = cr["mode"][mask_cr]

        # データ品質確認: alt_sp/alt_vel_target/climb_cmd はテレメトリ未実装で
        # 全ログ・全フライトで恒常的に 0（配線されていない）ことを検証済み。
        # そのため「alt - alt_sp」は使えない。目視/throttleスティック確認により、
        # POS_HOLD中はスティックがほぼ中立（venue: |throttle|<=0.04, night: <=0.06）
        # であることを別途確認した上で、各飛行区間の「中央値高度」を代理の
        # 保持基準とし、そこからの偏差を上下動プロキシとして使う。
        # Data-quality finding: alt_sp/alt_vel_target/climb_cmd are always exactly
        # 0 in every log (telemetry not wired). Verified stick (ctrl.throttle) stays
        # near neutral during POS_HOLD in both night and venue flights, so we use
        # the flight-window median altitude as a proxy hold reference instead.
        self.alt_sp_populated = bool(np.any(np.abs(cr["alt_sp"]) > 1e-9))

        # posvel -> altitude (NED: alt = -pos_z), ctrl_ref格子に補間 (gapは NaN)
        alt_full = -pv["pos"][:, 2]
        self.alt = interp_with_gap_mask(pv["ts"], alt_full, self.t)
        vel_z_full = pv["vel"][:, 2]  # NED down-positive
        self.vel_z_up = -interp_with_gap_mask(pv["ts"], vel_z_full, self.t)

        # 保持基準 = 各離陸から5秒以降（上昇過渡を除く）の中央値高度
        # Hold reference = median altitude excluding the first 5s (climb-in) of
        # each flight window.
        settle_mask = np.zeros(len(self.t), dtype=bool)
        for t0, t1 in self.windows:
            settle_mask |= (self.t >= t0 + 5.0) & (self.t <= t1)
        self.alt_ref = float(np.nanmedian(self.alt[settle_mask])) if np.any(settle_mask) else float(np.nanmedian(self.alt))
        self.alt_dev = self.alt - self.alt_ref  # 上下動プロキシ (proxy wobble signal, 離陸過渡含む全区間)
        self.settle_mask = settle_mask
        # RMS/P2P/PSD/電圧相関/ヨー連成統計に使う"誤差"は離陸過渡(最初の5秒)を除外
        # する。除外しないと短い020050フライトで離陸上昇そのものが「上下動」に
        # 誤カウントされる。
        self.alt_err = np.where(settle_mask, self.alt_dev, np.nan)

        # voltage (status, 1Hz) -> ctrl_ref格子へZOH
        self.voltage = zoh_with_gap_mask(st["ts"], st["voltage"], self.t, max_gap=2.0)

        # tof, baro
        self.tof_dist_raw_ts = tofb["ts"]
        self.tof_dist_raw = tofb["distance"]
        self.tof_status_raw = tofb["status"]
        tof_valid = tofb["status"] == 0
        tof_dist_v = np.where(tof_valid, tofb["distance"], np.nan)
        self.tof_dist = interp_with_gap_mask(tofb["ts"], tof_dist_v, self.t, max_gap=1.0)

        self.has_baro = "baro" in d
        if self.has_baro:
            baro = d["baro"]
            baro_alt_full = baro["altitude"]
            self.baro_alt0 = baro_alt_full[np.isfinite(baro_alt_full)][0]
            baro_rel = baro_alt_full - self.baro_alt0
            self.baro_alt = interp_with_gap_mask(baro["ts"], baro_rel, self.t, max_gap=1.0)
        else:
            self.baro_alt = np.full_like(self.t, np.nan)

        # motor saturation flag (>=0.95 or <=0.02 on any of 4 motors)
        with np.errstate(invalid="ignore"):
            sat_hi = np.any(self.motor_duty >= 0.95, axis=1)
            sat_lo = np.any(self.motor_duty <= 0.02, axis=1)
        self.sat_flag = sat_hi | sat_lo

        # given yaw event windows -> boolean mask on self.t
        self.event_mask = np.zeros(len(self.t), dtype=bool)
        for t0, t1 in self.meta["yaw_events"]:
            self.event_mask |= (self.t >= t0) & (self.t <= t1)

        self.fs = 1.0 / np.median(np.diff(t_cr))


def rms(x):
    x = x[~np.isnan(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else np.nan


def p2p(x):
    x = x[~np.isnan(x)]
    return float(np.max(x) - np.min(x)) if len(x) else np.nan


# ---------------------------------------------------------------------------
# 1+5. 特性の定量 + 制御構造の観察
# ---------------------------------------------------------------------------
def analyze_characterization(fd: FlightData):
    out = {}
    out["rms_alt_err"] = rms(fd.alt_err)
    out["p2p_alt_err"] = p2p(fd.alt_err)
    out["rms_alt"] = rms(fd.alt - np.nanmean(fd.alt))
    out["mean_alt"] = float(np.nanmean(fd.alt))
    psd = welch_dominant(fd.t, fd.alt_err, fd.fs)
    if psd:
        out["dominant_freq_hz"] = psd["f_dominant"]
        out["dominant_period_s"] = 1.0 / psd["f_dominant"] if psd["f_dominant"] > 0 else np.inf
        out["band_power_frac"] = psd["band_power_frac"]
        out["_psd"] = psd
    else:
        out["dominant_freq_hz"] = np.nan

    # 自己相関による独立クロスチェック（Welchの低周波バイアスに引っ張られていないか）
    ac_period, ac_peak = autocorr_period(fd.alt_err, fd.fs)
    out["autocorr_period_s"] = ac_period
    out["autocorr_peak"] = ac_peak

    out["alt_sp_populated"] = fd.alt_sp_populated
    out["alt_ref_m"] = fd.alt_ref

    # 追従: alt_vel_target vs 実測 vel_z, 相関 & 遅延（クロス相関のピークラグ）
    valid = ~(np.isnan(fd.alt_vel_target) | np.isnan(fd.vel_z_up))
    if np.sum(valid) > 50:
        a = fd.alt_vel_target[valid] - np.nanmean(fd.alt_vel_target[valid])
        b = fd.vel_z_up[valid] - np.nanmean(fd.vel_z_up[valid])
        corr = np.corrcoef(a, b)[0, 1]
        out["velz_tracking_corr"] = float(corr)
    else:
        out["velz_tracking_corr"] = np.nan

    # total_thrust 飽和（既知上限に近いか）: 全飛行の thrust 分布
    tt = fd.total_thrust[~np.isnan(fd.total_thrust)]
    out["thrust_mean"] = float(np.mean(tt)) if len(tt) else np.nan
    out["thrust_max"] = float(np.max(tt)) if len(tt) else np.nan
    out["thrust_p99"] = float(np.percentile(tt, 99)) if len(tt) else np.nan

    # 推力→高度の位相関係（振動的な場合）: total_thrust と alt_err のクロス相関
    valid2 = ~(np.isnan(fd.total_thrust) | np.isnan(fd.alt_err))
    if np.sum(valid2) > 50:
        tt2 = fd.total_thrust[valid2] - np.nanmean(fd.total_thrust[valid2])
        ae2 = fd.alt_err[valid2] - np.nanmean(fd.alt_err[valid2])
        # 正規化相互相関、ラグ [-2s, +2s]
        max_lag = int(2.0 * fd.fs)
        xc = signal.correlate(ae2, tt2, mode="full")
        lags = signal.correlation_lags(len(ae2), len(tt2), mode="full")
        center = len(xc) // 2
        window = slice(center - max_lag, center + max_lag + 1)
        xc_w = xc[window]
        lags_w = lags[window]
        norm = np.sqrt(np.sum(ae2 ** 2) * np.sum(tt2 ** 2))
        xc_n = xc_w / norm if norm > 0 else xc_w
        peak_i = np.argmax(np.abs(xc_n))
        out["thrust_alt_err_xcorr_peak"] = float(xc_n[peak_i])
        out["thrust_alt_err_xcorr_lag_s"] = float(lags_w[peak_i] / fd.fs)
    return out


# ---------------------------------------------------------------------------
# 2. 電圧依存性
# ---------------------------------------------------------------------------
def analyze_voltage(fd: FlightData, n_bins=4):
    out = {}
    v = fd.voltage
    valid = ~np.isnan(v)
    out["voltage_min"] = float(np.nanmin(v)) if np.any(valid) else np.nan
    out["voltage_max"] = float(np.nanmax(v)) if np.any(valid) else np.nan
    out["voltage_mean"] = float(np.nanmean(v)) if np.any(valid) else np.nan

    # 時間区間分割 (n_bins) で誤差RMS・thrust平均・電圧平均の推移
    bins = []
    n = len(fd.t)
    edges = np.linspace(0, n, n_bins + 1).astype(int)
    for i in range(n_bins):
        s, e = edges[i], edges[i + 1]
        seg_err = fd.alt_err[s:e]
        seg_v = fd.voltage[s:e]
        seg_tt = fd.total_thrust[s:e]
        bins.append(dict(
            t0=float(fd.t[s]), t1=float(fd.t[e - 1]),
            rms_err=rms(seg_err), p2p_err=p2p(seg_err),
            voltage_mean=float(np.nanmean(seg_v)) if np.any(~np.isnan(seg_v)) else np.nan,
            thrust_mean=float(np.nanmean(seg_tt)) if np.any(~np.isnan(seg_tt)) else np.nan,
        ))
    out["time_bins"] = bins

    # 電圧 vs |alt_err| の相関（瞬時, サンプル単位）
    valid2 = ~(np.isnan(v) | np.isnan(fd.alt_err))
    if np.sum(valid2) > 50 and np.nanstd(v[valid2]) > 1e-6:
        corr = np.corrcoef(v[valid2], np.abs(fd.alt_err[valid2]))[0, 1]
        out["voltage_vs_abs_err_corr"] = float(corr)
    else:
        out["voltage_vs_abs_err_corr"] = np.nan

    # 電圧 vs total_thrust の相関・回帰
    valid3 = ~(np.isnan(v) | np.isnan(fd.total_thrust))
    if np.sum(valid3) > 50 and np.nanstd(v[valid3]) > 1e-6:
        slope, intercept, r, p, se = stats.linregress(v[valid3], fd.total_thrust[valid3])
        out["thrust_vs_voltage_slope"] = float(slope)
        out["thrust_vs_voltage_r"] = float(r)
        out["thrust_vs_voltage_p"] = float(p)
    else:
        out["thrust_vs_voltage_slope"] = np.nan
        out["thrust_vs_voltage_r"] = np.nan
    return out


# ---------------------------------------------------------------------------
# 3. ヨー連成の検定
# ---------------------------------------------------------------------------
def analyze_yaw_coupling(fd: FlightData, n_perm=2000, rng=None):
    out = {}
    if len(fd.meta["yaw_events"]) == 0:
        out["has_events"] = False
        return out
    out["has_events"] = True
    ev = fd.event_mask
    n_in = int(np.sum(ev))
    n_out = int(np.sum(~ev))
    out["n_in"] = n_in
    out["n_out"] = n_out

    out["rms_err_in"] = rms(fd.alt_err[ev])
    out["rms_err_out"] = rms(fd.alt_err[~ev])

    # d(alt)/dt: ctrl_ref格子上、gapを跨がない前提（既にNaNマスク済みalt使用。
    # 隣接差分でgapを跨ぐ箇所はdtが大きくなるので別途チェック）
    dt = np.diff(fd.t)
    dalt = np.diff(fd.alt)
    with np.errstate(invalid="ignore"):
        dalt_dt = dalt / dt
    # gapを跨ぐ(dt異常に大)差分は除外
    dt_median = np.median(dt[~np.isnan(dt)])
    bad_dt = dt > (dt_median * 5)
    dalt_dt[bad_dt] = np.nan
    ev_mid = ev[:-1] & ev[1:]  # 差分点は両端がイベント内である場合のみ「内」

    out["mean_abs_dalt_dt_in"] = float(np.nanmean(np.abs(dalt_dt[ev_mid]))) if np.any(ev_mid) else np.nan
    out["mean_abs_dalt_dt_out"] = float(np.nanmean(np.abs(dalt_dt[~ev_mid]))) if np.any(~ev_mid) else np.nan
    out["p95_abs_dalt_dt_in"] = float(np.nanpercentile(np.abs(dalt_dt[ev_mid]), 95)) if np.any(ev_mid) else np.nan
    out["p95_abs_dalt_dt_out"] = float(np.nanpercentile(np.abs(dalt_dt[~ev_mid]), 95)) if np.any(~ev_mid) else np.nan

    # 「急変」定義: 全体(飛行全区間)のP90を閾値に、その超過割合を in/out で比較
    thr = np.nanpercentile(np.abs(dalt_dt), 90)
    out["fast_change_thresh_mps"] = float(thr)
    frac_fast_in = float(np.nanmean(np.abs(dalt_dt[ev_mid]) > thr)) if np.any(ev_mid) else np.nan
    frac_fast_out = float(np.nanmean(np.abs(dalt_dt[~ev_mid]) > thr)) if np.any(~ev_mid) else np.nan
    out["frac_fast_change_in"] = frac_fast_in
    out["frac_fast_change_out"] = frac_fast_out

    # duty飽和割合 (検出ベース) in/out - 定義との整合確認
    out["sat_frac_in"] = float(np.mean(fd.sat_flag[ev])) if n_in else np.nan
    out["sat_frac_out"] = float(np.mean(fd.sat_flag[~ev])) if n_out else np.nan

    # クロス相関: sat_flag(0/1) と |高度加速度|(d2alt/dt2 相当 = d(vel_z)/dt)
    dvel = np.diff(fd.vel_z_up)
    with np.errstate(invalid="ignore"):
        acc = dvel / dt
    acc[bad_dt] = np.nan
    sat_mid = fd.sat_flag[:-1].astype(float)
    valid_xc = ~np.isnan(acc)
    if np.sum(valid_xc) > 50:
        a = np.nan_to_num(sat_mid[valid_xc] - np.mean(sat_mid[valid_xc]))
        b = np.nan_to_num(np.abs(acc[valid_xc]) - np.mean(np.abs(acc[valid_xc])))
        max_lag = int(2.0 * fd.fs)
        xc = signal.correlate(b, a, mode="full")
        lags = signal.correlation_lags(len(b), len(a), mode="full")
        center = len(xc) // 2
        window = slice(max(0, center - max_lag), center + max_lag + 1)
        xc_w = xc[window]
        lags_w = lags[window]
        norm = np.sqrt(np.sum(a ** 2) * np.sum(b ** 2))
        xc_n = xc_w / norm if norm > 0 else xc_w
        peak_i = np.argmax(xc_n)
        out["sat_accel_xcorr_peak"] = float(xc_n[peak_i])
        out["sat_accel_xcorr_lag_s"] = float(lags_w[peak_i] / fd.fs)
    else:
        out["sat_accel_xcorr_peak"] = np.nan

    # 高度ディップの列挙: alt_err の局所最小 (down excursion) の時刻がイベント窓と重なるか
    err = fd.alt_err.copy()
    err_f = np.where(np.isnan(err), 0.0, err)
    # ディップ = 誤差が -0.15m 以下に落ちる区間
    dip_thr = -0.15
    dip_mask = err < dip_thr
    dips = contiguous_valid_runs(dip_mask)
    dip_list = []
    for s, e in dips:
        t0, t1 = fd.t[s], fd.t[e - 1]
        overlap = any(not (t1 < w0 or t0 > w1) for w0, w1 in fd.meta["yaw_events"])
        dip_list.append(dict(t0=float(t0), t1=float(t1), min_err=float(np.min(err[s:e])), overlaps_event=overlap))
    out["dips"] = dip_list
    out["n_dips"] = len(dip_list)
    out["n_dips_overlap_event"] = int(sum(1 for x in dip_list if x["overlaps_event"]))

    # 置換検定 (permutation test): イベント窓の位置を円環シフトしランダム化し、
    # 「イベント内RMS / イベント外RMS」比の帰無分布を作る。
    if rng is None:
        rng = np.random.default_rng(0)
    obs_ratio = out["rms_err_in"] / out["rms_err_out"] if out["rms_err_out"] else np.nan
    n = len(fd.t)
    err_arr = fd.alt_err
    null_ratios = []
    valid_err = ~np.isnan(err_arr)
    for _ in range(n_perm):
        shift = rng.integers(1, n - 1)
        ev_shift = np.roll(ev, shift)
        in_v = err_arr[ev_shift & valid_err]
        out_v = err_arr[(~ev_shift) & valid_err]
        if len(in_v) < 5 or len(out_v) < 5:
            continue
        r_in = np.sqrt(np.mean(in_v ** 2))
        r_out = np.sqrt(np.mean(out_v ** 2))
        if r_out > 0:
            null_ratios.append(r_in / r_out)
    null_ratios = np.array(null_ratios)
    out["perm_null_mean_ratio"] = float(np.mean(null_ratios)) if len(null_ratios) else np.nan
    out["perm_null_std_ratio"] = float(np.std(null_ratios)) if len(null_ratios) else np.nan
    if len(null_ratios):
        p_val = float(np.mean(null_ratios >= obs_ratio))
    else:
        p_val = np.nan
    out["obs_ratio_in_over_out"] = float(obs_ratio)
    out["perm_p_value"] = p_val
    return out


# ---------------------------------------------------------------------------
# 4. センサ整合性
# ---------------------------------------------------------------------------
def analyze_sensor_consistency(fd: FlightData):
    out = {}
    # alt vs tof (tofが有効レンジ内: <1.3m and status0) のときの差
    diff_tof = fd.alt - fd.tof_dist
    valid_tof = ~np.isnan(diff_tof) & (fd.tof_dist < 1.3)
    out["n_tof_valid_compare"] = int(np.sum(valid_tof))
    out["mean_alt_minus_tof"] = float(np.nanmean(diff_tof[valid_tof])) if np.any(valid_tof) else np.nan
    out["rms_alt_minus_tof"] = float(np.sqrt(np.nanmean(diff_tof[valid_tof] ** 2))) if np.any(valid_tof) else np.nan
    out["frac_time_tof_in_range"] = float(np.mean(fd.tof_dist < 1.3) ) if len(fd.tof_dist) else np.nan

    # alt vs baro (相対値, 平均を合わせて比較)
    if fd.has_baro:
        valid_b = ~np.isnan(fd.baro_alt) & ~np.isnan(fd.alt)
        if np.any(valid_b):
            offset = np.nanmean(fd.alt[valid_b]) - np.nanmean(fd.baro_alt[valid_b])
            diff_b = fd.alt - (fd.baro_alt + offset)
            out["rms_alt_minus_baro_detrended"] = float(np.sqrt(np.nanmean(diff_b[valid_b] ** 2)))
            out["max_abs_alt_minus_baro_detrended"] = float(np.nanmax(np.abs(diff_b[valid_b])))
        else:
            out["rms_alt_minus_baro_detrended"] = np.nan
    else:
        out["rms_alt_minus_baro_detrended"] = np.nan

    # alt(ESKF)のジャンプ検出: 400Hz生pos_zの隣接差分が異常閾値超（gapは除外済み変数を再構築）
    pv = fd.d["posvel"]
    alt_full = -pv["pos"][:, 2]
    t_full = pv["ts"]
    mask = fd._mask_windows(t_full)
    tf = t_full[mask]
    af = alt_full[mask]
    dtf = np.diff(tf)
    daf = np.diff(af)
    dt_median = np.median(dtf[dtf > 0])
    # dt==0 の重複タイムスタンプ(UDP重複パケット等、全ログ共通で4-8%発生する既知の
    # ロギング特性。venue固有ではない)を除外しないと 0除算でinf/nanが混入し、
    # 見かけ上の"ジャンプ"を大量に誤検出する。
    ok = (dtf > 0) & (dtf < dt_median * 5)
    rate = np.full_like(daf, np.nan)
    rate[ok] = daf[ok] / dtf[ok]
    out["n_dt_zero_dup"] = int(np.sum(dtf == 0))
    out["frac_dt_zero_dup"] = float(np.mean(dtf == 0))
    jump_thr = 2.0  # m/s相当の瞬間変化(400Hzで物理的にありえない急変)をジャンプとみなす
    jump_idx = np.where(np.abs(rate) > jump_thr)[0]
    out["n_eskf_jumps"] = int(len(jump_idx))
    out["eskf_jump_times"] = [float(tf[i]) for i in jump_idx[:20]]

    # tof status異常率
    tofb = fd.d["tof_b"]
    mask_tof = fd._mask_windows(tofb["ts"])
    st = tofb["status"][mask_tof]
    out["tof_status254_frac"] = float(np.mean(st == 254)) if len(st) else np.nan
    out["tof_status_other_frac"] = float(np.mean((st != 0) & (st != 254))) if len(st) else np.nan
    return out


# ---------------------------------------------------------------------------
# メイン
# ---------------------------------------------------------------------------
def main():
    rng = np.random.default_rng(42)
    results = {}
    flight_data = {}

    for key, meta in LOGS.items():
        print(f"\n{'='*70}\n[{key}] {meta['label']}  file={meta['file']}\n{'='*70}")
        fd = FlightData(key, meta)
        flight_data[key] = fd

        char = analyze_characterization(fd)
        volt = analyze_voltage(fd)
        yaw = analyze_yaw_coupling(fd, rng=rng)
        sens = analyze_sensor_consistency(fd)

        results[key] = dict(char=char, volt=volt, yaw=yaw, sens=sens)

        print(f"-- characterization --")
        print(f"  [data quality] alt_sp populated (nonzero anywhere)? {char['alt_sp_populated']} "
              f"-> using alt_ref(median)={char['alt_ref_m']:.3f}m as hold-reference proxy")
        print(f"  RMS alt_dev = {char['rms_alt_err']*100:.2f} cm, P2P = {char['p2p_alt_err']*100:.2f} cm")
        print(f"  dominant freq in [0.05,5]Hz = {char.get('dominant_freq_hz', float('nan')):.3f} Hz "
              f"(period {char.get('dominant_period_s', float('nan')):.2f} s), "
              f"band_power_frac={char.get('band_power_frac', float('nan')):.3f}")
        if char.get("autocorr_period_s") is not None:
            print(f"  autocorr check: strongest peak at lag={char['autocorr_period_s']:.2f}s, "
                  f"normalized ACF={char['autocorr_peak']:.3f} "
                  f"({'periodic-ish' if char['autocorr_peak'] > 0.3 else 'weak/irregular'})")
        else:
            print("  autocorr check: no clear peak found")
        print(f"  alt_vel_target vs measured vel_z corr = {char['velz_tracking_corr']:.3f}")
        print(f"  thrust mean={char['thrust_mean']:.3f} max={char['thrust_max']:.3f} p99={char['thrust_p99']:.3f}")
        if "thrust_alt_err_xcorr_peak" in char:
            print(f"  thrust vs alt_err xcorr peak={char['thrust_alt_err_xcorr_peak']:.3f} "
                  f"at lag={char['thrust_alt_err_xcorr_lag_s']:.3f}s")

        print(f"-- voltage --")
        print(f"  V range [{volt['voltage_min']:.2f}, {volt['voltage_max']:.2f}] mean={volt['voltage_mean']:.2f}")
        for b in volt["time_bins"]:
            print(f"   t[{b['t0']:.1f},{b['t1']:.1f}]: RMSerr={b['rms_err']*100:.2f}cm "
                  f"V={b['voltage_mean']:.2f} thrust={b['thrust_mean']:.3f}")
        print(f"  corr(V, |alt_err|) = {volt['voltage_vs_abs_err_corr']:.3f}")
        print(f"  thrust = {volt['thrust_vs_voltage_slope']:.4f}*V + const,  r={volt['thrust_vs_voltage_r']:.3f} "
              f"p={volt.get('thrust_vs_voltage_p', float('nan')):.4g}")

        print(f"-- yaw coupling --")
        if yaw.get("has_events"):
            print(f"  n_in={yaw['n_in']} n_out={yaw['n_out']}")
            print(f"  RMS err in={yaw['rms_err_in']*100:.2f}cm out={yaw['rms_err_out']*100:.2f}cm "
                  f"ratio={yaw['obs_ratio_in_over_out']:.3f}")
            print(f"  mean|dalt/dt| in={yaw['mean_abs_dalt_dt_in']:.3f} out={yaw['mean_abs_dalt_dt_out']:.3f} m/s")
            print(f"  frac(|dalt/dt|>P90) in={yaw['frac_fast_change_in']:.3f} out={yaw['frac_fast_change_out']:.3f}")
            print(f"  duty-sat frac in={yaw['sat_frac_in']:.3f} out={yaw['sat_frac_out']:.3f}")
            print(f"  sat-flag vs |accel| xcorr peak={yaw['sat_accel_xcorr_peak']:.3f} "
                  f"lag={yaw.get('sat_accel_xcorr_lag_s', float('nan')):.3f}s")
            print(f"  n_dips(<-15cm)={yaw['n_dips']}  overlap_event={yaw['n_dips_overlap_event']}")
            for dp in yaw["dips"]:
                print(f"     dip [{dp['t0']:.1f},{dp['t1']:.1f}]s min={dp['min_err']*100:.1f}cm "
                      f"overlap_event={dp['overlaps_event']}")
            print(f"  permutation test: null_mean_ratio={yaw['perm_null_mean_ratio']:.3f}+-"
                  f"{yaw['perm_null_std_ratio']:.3f}, obs={yaw['obs_ratio_in_over_out']:.3f}, "
                  f"p={yaw['perm_p_value']:.4f}")
        else:
            print("  (no yaw events defined for this flight)")

        print(f"-- sensor consistency --")
        print(f"  alt-tof: n={sens['n_tof_valid_compare']} mean={sens['mean_alt_minus_tof']*100:.2f}cm "
              f"rms={sens['rms_alt_minus_tof']*100:.2f}cm frac_in_range={sens['frac_time_tof_in_range']:.3f}")
        print(f"  alt-baro(detrended) rms={sens.get('rms_alt_minus_baro_detrended', float('nan'))*100:.2f}cm "
              f"max={sens.get('max_abs_alt_minus_baro_detrended', float('nan'))*100:.2f}cm")
        print(f"  [dt==0 dup ts] n={sens['n_dt_zero_dup']} frac={sens['frac_dt_zero_dup']:.4f} "
              f"(known cross-flight UDP dup artifact, excluded from jump calc)")
        print(f"  ESKF alt jumps(>2m/s step) n={sens['n_eskf_jumps']} times={sens['eskf_jump_times']}")
        print(f"  tof status254 frac={sens['tof_status254_frac']:.3f} other_bad_frac={sens['tof_status_other_frac']:.3f}")

    # ------------------------------------------------------------------
    # 電圧依存の横断比較 (夜間4.2V vs 会場3.5-3.8V)
    # ------------------------------------------------------------------
    print(f"\n{'='*70}\nCROSS-FLIGHT: voltage-band comparison\n{'='*70}")
    night_keys = ["020050", "020137"]
    venue_keys = ["164611", "165713"]
    for group, keys in [("night(4.2V)", night_keys), ("venue(3.5-3.8V)", venue_keys)]:
        rmss = [results[k]["char"]["rms_alt_err"] for k in keys]
        p2ps = [results[k]["char"]["p2p_alt_err"] for k in keys]
        thr = [results[k]["char"]["thrust_mean"] for k in keys]
        print(f"  {group}: RMS_err(cm)={[f'{x*100:.2f}' for x in rmss]} "
              f"P2P(cm)={[f'{x*100:.2f}' for x in p2ps]} thrust_mean={[f'{x:.3f}' for x in thr]}")

    # ------------------------------------------------------------------
    # ヨー連成の全ログ集約 (対立仮説棄却判定用の統合 p 値: Fisher's method)
    # ------------------------------------------------------------------
    print(f"\n{'='*70}\nCROSS-FLIGHT: yaw coupling aggregate\n{'='*70}")
    p_vals = []
    ratios = []
    for k in ["020050", "164611", "165713"]:
        y = results[k]["yaw"]
        if y.get("has_events"):
            p_vals.append(max(y["perm_p_value"], 1e-4))
            ratios.append(y["obs_ratio_in_over_out"])
            print(f"  {k}: ratio(in/out RMS)={y['obs_ratio_in_over_out']:.3f} p={y['perm_p_value']:.4f}")
    if p_vals:
        chi2 = -2 * np.sum(np.log(p_vals))
        dof = 2 * len(p_vals)
        p_combined = 1 - stats.chi2.cdf(chi2, dof)
        print(f"  Fisher's combined p-value across {len(p_vals)} flights: chi2={chi2:.2f} dof={dof} p={p_combined:.4f}")
        results["_combined_yaw_p"] = float(p_combined)
        results["_combined_yaw_ratios"] = ratios

    # ------------------------------------------------------------------
    # プロット: 代表飛行 165713 セグメント2 と 020137
    # ------------------------------------------------------------------
    plot_flight(flight_data["165713"], window=(56.3, 196.5),
                fname=f"{OUT_DIR}/alt_hold_165713.png",
                title="165713 venue POS_HOLD seg2 (3.5-4.0V)")
    plot_flight(flight_data["020137"], window=(80.2, 258.3),
                fname=f"{OUT_DIR}/alt_hold_020137.png",
                title="020137 night POS_HOLD (4.2V)")

    # 数値サマリをJSONに保存(次段のレポート作成用)
    def _clean(o):
        if isinstance(o, dict):
            return {k: _clean(v) for k, v in o.items() if k not in ("_psd",)}
        if isinstance(o, (list, tuple)):
            return [_clean(v) for v in o]
        if isinstance(o, (np.floating,)):
            return float(o)
        if isinstance(o, (np.integer,)):
            return int(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        return o

    with open(f"{OUT_DIR}/alt_hold_results.json", "w") as f:
        json.dump(_clean(results), f, indent=2, ensure_ascii=False)
    print(f"\nSaved: {OUT_DIR}/alt_hold_results.json")


def plot_flight(fd: FlightData, window, fname, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t0, t1 = window
    m = (fd.t >= t0) & (fd.t <= t1)
    t = fd.t[m]

    fig, axes = plt.subplots(5, 1, figsize=(14, 12), sharex=True)

    ax = axes[0]
    ax.plot(t, fd.alt[m], label="alt (-pos_z)", lw=0.8)
    ax.axhline(fd.alt_ref, color="k", lw=0.8, ls="--",
               label=f"alt_ref (window median, {fd.alt_ref:.2f}m)\n[alt_sp telemetry is always 0 - not used]")
    ax.set_ylabel("altitude [m]")
    ax.legend(loc="upper right", fontsize=7)
    ax.set_title(title)

    ax = axes[1]
    ax.plot(t, fd.alt_err[m] * 100, color="tab:red", lw=0.8)
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("alt dev [cm]\n(alt - alt_ref)")

    ax = axes[2]
    ax.plot(t, fd.total_thrust[m], color="tab:green", lw=0.8, label="total_thrust")
    ax.set_ylabel("total_thrust")
    ax2 = ax.twinx()
    ax2.plot(t, fd.voltage[m], color="tab:orange", lw=0.8, label="voltage")
    ax2.set_ylabel("voltage [V]", color="tab:orange")
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right", fontsize=8)

    ax = axes[3]
    for i in range(4):
        ax.plot(t, fd.motor_duty[m, i], lw=0.6, label=f"m{i}")
    ax.set_ylabel("motor duty")
    ax.legend(loc="upper right", fontsize=7, ncol=4)

    ax = axes[4]
    ax.plot(t, fd.sat_flag[m].astype(int), color="black", lw=0.8, label="motor sat flag")
    for w0, w1 in fd.meta["yaw_events"]:
        if w1 < t0 or w0 > t1:
            continue
        ax.axvspan(max(w0, t0), min(w1, t1), color="red", alpha=0.2)
        for a in axes:
            a.axvspan(max(w0, t0), min(w1, t1), color="red", alpha=0.08)
    ax.set_ylabel("sat flag")
    ax.set_xlabel("t [s] (absolute log ts)")
    ax.set_ylim(-0.1, 1.1)

    fig.tight_layout()
    fig.savefig(fname, dpi=130)
    plt.close(fig)
    print(f"Saved plot: {fname}")


if __name__ == "__main__":
    main()
