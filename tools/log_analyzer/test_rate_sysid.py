"""
test_rate_sysid.py - host tests for the rate-loop identification backend
test_rate_sysid.py - レートループ同定バックエンドのホスト試験

Covers what real-vehicle logs exposed (roll-rate excitation flights on 2026-11-23) and
noise-free SILS logs never did: leading-NaN battery voltage, the excitation window taken
from the bundle's text_log, unexcited noisy segments dominating a whole-log fit, and the
fail-loudly guard on an empty/NaN ETFE.
実機ログで露呈し、ノイズの無い SILS ログでは見えなかった点の試験: 先頭が NaN の電圧、
text_log から取る励振区間、励振していないノイズ区間が全体フィットを支配すること、
空/NaN の ETFE で黙らず失敗するガード。
"""

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rate_sysid as rs  # noqa: E402

B_TRUE, T_TRUE, L_TRUE = 48000.0, 0.025, 0.006
VBAT = 3.85
THRUST_HOVER = 0.4


def _duties(u):
    """Forward mixer: roll torque u -> 4 motor duties (as in rate_sysid.selftest()).
    順方向ミキサー: ロールトルク u -> 4 モータ duty（selftest() と同じ）。"""
    def to_duty(t):
        w = np.sqrt(np.maximum(t, 0.0) / rs._MOTOR_CT)
        return (rs._MOTOR_AM * w ** 2 + rs._MOTOR_BM * w + rs._MOTOR_CM) / VBAT
    fr = 0.25 * (THRUST_HOVER - u / rs._ARM_D)
    rl = 0.25 * (THRUST_HOVER + u / rs._ARM_D)
    return to_duty(fr), to_duty(fr), to_duty(rl), to_duty(rl)


def _plant(u):
    """Roll rate of G(s)=b e^{-Ls}/(s(Ts+1)) driven by u. / u で駆動したロール角速度。"""
    delay = int(round(L_TRUE / rs.DT))
    ud = np.concatenate([np.zeros(delay), u[:len(u) - delay]])
    y = np.zeros(len(u))
    acc = rate = 0.0
    for k, uk in enumerate(ud):
        acc += rs.DT / T_TRUE * (uk * B_TRUE - acc)
        rate += acc * rs.DT
        y[k] = rate
    return y


def _make_df(lead_s=3.0, chirp_s=6.0, tail_s=3.0, nan_voltage_rows=400, polluted=False, seed=3,
             noise_y=0.3, noise_u=4e-3):
    """Synthetic bundle-like DataFrame: unexcited lead, chirp, unexcited tail.
    合成のバンドル風 DataFrame: 励振なし前部、チャープ、励振なし後部。
    Returns (df, start_us, end_us)."""
    rng = np.random.default_rng(seed)
    n_lead, n_ch, n_tail = (int(round(s * rs.FS)) for s in (lead_s, chirp_s, tail_s))
    t_ch = np.arange(n_ch) * rs.DT
    k = 25.0 ** (1.0 / chirp_s)
    chirp = 2e-3 * np.sin(2 * np.pi * ((k ** t_ch) - 1) / np.log(k))
    u = np.concatenate([np.zeros(n_lead), chirp, np.zeros(n_tail)])
    y = _plant(u)
    if polluted:
        # unexcited segments: sensor noise + an input uncorrelated with it
        # (what a closed loop does on the ground / in a quiet hover)
        # 励振なし区間: センサノイズと、それと無相関な入力（地上・静かなホバーの閉ループ相当）
        quiet = np.ones(len(u), bool)
        quiet[n_lead:n_lead + n_ch] = False
        y = y + np.where(quiet, rng.normal(0, noise_y, len(u)), 0.0)
        u = u + np.where(quiet, rng.normal(0, noise_u, len(u)), 0.0)
    duty = _duties(u)
    n = len(u)
    t_us = (np.arange(n) * rs.DT * 1e6).astype("int64") + 40_000_000
    volt = np.full(n, VBAT)
    volt[:nan_voltage_rows] = np.nan
    df = pd.DataFrame({
        "timestamp_us": t_us, "rate_ref_roll": np.zeros(n), "gyro_x": y,
        "duty_FR": duty[0], "duty_RR": duty[1], "duty_RL": duty[2], "duty_FL": duty[3],
        "voltage": volt})
    df.attrs["bundle_streams"] = {"imu", "rate_ref", "motor", "status"}
    start_us = int(t_us[n_lead])
    end_us = int(t_us[n_lead + n_ch - 1])
    return df, start_us, end_us


def _text_log(rows):
    return pd.DataFrame({"timestamp_us": [r[0] for r in rows], "text": [r[1] for r in rows]})


START = "I (1000) PID: Sysid excitation start: axis={ax} chirp amp=0.17 rad/s dur=5.0 s"
DONE = "I (6000) PID: Sysid excitation done"


# --- (1) leading NaN voltage ------------------------------------------------

def test_leading_nan_voltage_gives_finite_fit():
    df, _, _ = _make_df()
    res = rs.fit_from_df(df, "roll", f_lo=1.0, f_hi=25.0, input_mode="duty")
    assert np.isfinite(res["coherence_mean"])
    assert res["b"] == pytest.approx(B_TRUE, rel=0.15)
    assert res["T"] == pytest.approx(T_TRUE, rel=0.35)
    assert "400/" in res["duty_reason"] and "filled" in res["duty_reason"]


def test_all_nan_voltage_falls_back_to_nominal():
    df, _, _ = _make_df(nan_voltage_rows=10 ** 6)
    res = rs.fit_from_df(df, "roll", f_lo=1.0, f_hi=25.0, input_mode="duty")
    assert np.isfinite(res["coherence_mean"])
    assert "nominal" in res["duty_reason"]


# --- (2) window detection from text_log -------------------------------------

def test_window_start_and_done():
    w = rs.find_excitation_windows(_text_log([(10_000_000, START.format(ax=0)),
                                              (15_000_000, DONE)]), "roll")
    assert len(w) == 1
    assert (w[0]["start_us"], w[0]["end_us"], w[0]["end_source"]) == (10_000_000, 15_000_000, "done")


def test_window_without_done_uses_duration():
    w = rs.find_excitation_windows(_text_log([(10_000_000, START.format(ax=0))]), "roll")
    assert w[0]["end_us"] == 15_000_000 and w[0]["end_source"] == "start+dur"


def test_window_other_axis_ignored():
    log = _text_log([(10_000_000, START.format(ax=1)), (15_000_000, DONE)])
    assert rs.find_excitation_windows(log, "roll") == []
    assert len(rs.find_excitation_windows(log, "pitch")) == 1


def test_window_several_uses_last_and_says_so():
    df, _, _ = _make_df()
    df.attrs["text_log"] = _text_log([
        (41_000_000, START.format(ax=0)), (46_000_000, DONE),
        (50_000_000, START.format(ax=0)), (55_000_000, DONE)])
    win, info = rs.resolve_window(df, "roll", pad_s=0.5)
    assert win == (49_500_000, 55_500_000)
    assert "last" in info["note"]


def test_resolve_window_fallbacks():
    df, _, _ = _make_df()
    df.attrs["text_log"] = None
    win, info = rs.resolve_window(df, "roll")
    assert win is None and info["warning"]
    win, _ = rs.resolve_window(df, "roll", time_range=(1.0, 2.0))
    assert win == (41_000_000, 42_000_000)


# --- (3) windowed vs whole-log fit on a polluted log ------------------------

def test_windowed_fit_recovers_plant_where_whole_log_does_not():
    df, s_us, e_us = _make_df(lead_s=6.0, tail_s=6.0, polluted=True)
    df.attrs["text_log"] = _text_log([(s_us, START.format(ax=0)), (e_us, DONE)])
    win, _ = rs.resolve_window(df, "roll")
    kw = dict(f_lo=1.0, f_hi=25.0, input_mode="duty", nperseg=512)
    windowed = rs.fit_from_df(df, "roll", window_us=win, **kw)
    whole = rs.fit_from_df(df, "roll", **kw)
    assert windowed["b"] == pytest.approx(B_TRUE, rel=0.10)
    assert windowed["window"]["n_samples"] < whole["window"]["n_samples"]
    assert abs(whole["b"] / B_TRUE - 1) > abs(windowed["b"] / B_TRUE - 1)


# --- (4) fail loudly --------------------------------------------------------

def test_etfe_nan_input_raises():
    u = np.full(2048, np.nan)
    y = np.random.default_rng(0).normal(size=2048)
    with pytest.raises(ValueError, match="no finite points"):
        rs.etfe(u, y)


def test_etfe_empty_band_raises():
    u = np.random.default_rng(1).normal(size=2048)
    with pytest.raises(ValueError, match="no finite points"):
        rs.etfe(u, u, f_lo=300.0, f_hi=390.0)


def test_fit_plant_empty_or_nan_raises():
    with pytest.raises(ValueError):
        rs.fit_plant(np.array([]), np.array([], dtype=complex), np.array([]))
    with pytest.raises(ValueError):
        rs.fit_plant(np.array([10.0]), np.array([1 + 0j]), np.array([np.nan]))


# --- (5) Welch coverage and the auto pad ------------------------------------
# The padded excitation window is cut into Hann segments from its first row; the
# leftover rows at the end are dropped. A fixed 0.5 s pad left the top of a 10 s
# 1-25 Hz chirp (above ~18 Hz) under-weighted and the last ~0.26 s unanalysed.
# パッド込みの励振区間は先頭行から Hann 区間に切られ、末尾の端数は捨てられる。
# 固定 0.5 s のパッドでは 10 s・1-25 Hz チャープの上端（約 18 Hz 以上）の重みが足りず、
# 最後の約 0.26 s は解析されなかった。

def test_auto_pads_and_nperseg_resolution():
    # 10 s chirp -> 1024 (13.84 s window); 5 s chirp -> 512 with its own pads
    assert rs.resolve_nperseg_and_pads(10.0) == (1024, pytest.approx(1.28), pytest.approx(2.56))
    n, pb, pa = rs.resolve_nperseg_and_pads(10.0)
    assert (10.0 + pb + pa) * rs.FS >= 4096
    n, pb, pa = rs.resolve_nperseg_and_pads(5.0)
    assert (n, pb, pa) == (512, pytest.approx(0.64), pytest.approx(1.28))
    # an explicit nperseg decides the auto pads; a number is symmetric
    assert rs.resolve_nperseg_and_pads(5.0, nperseg=256) == (
        256, pytest.approx(0.32), pytest.approx(0.64))
    assert rs.resolve_nperseg_and_pads(10.0, pad_s=0.5) == (1024, 0.5, 0.5)
    assert rs.resolve_nperseg_and_pads(5.0, pad_s=0.5) == (512, 0.5, 0.5)


def test_resolve_window_auto_pad_and_numeric_pad():
    df, s_us, e_us = _make_df(chirp_s=10.0)
    df.attrs["text_log"] = _text_log([(s_us, START.format(ax=0)), (e_us, DONE)])
    win, info = rs.resolve_window(df, "roll")                  # default: auto
    assert info["nperseg"] == 1024
    assert win == (pytest.approx(s_us - 1.28e6), pytest.approx(e_us + 2.56e6))
    assert info["exc_us"] == (s_us, e_us) and info["waveform"] == "chirp"
    win, info = rs.resolve_window(df, "roll", pad_s=0.5)       # old symmetric meaning
    assert win == (s_us - 0.5e6, e_us + 0.5e6) and info["nperseg"] == 1024
    assert (info["pad_before_s"], info["pad_after_s"]) == (0.5, 0.5)
    win, info = rs.resolve_window(df, "roll", nperseg=512)     # pads follow the override
    assert info["nperseg"] == 512
    assert win == (pytest.approx(s_us - 0.64e6), pytest.approx(e_us + 1.28e6))


def _rows(window_s, exc_dur_s, jitter=0.0, seed=0, pre_s=0.0):
    """Row timestamps [us] for a window of `window_s` seconds around an excitation
    that starts `pre_s` after the first row. / 励振が先頭から pre_s 後に始まる窓の行時刻。"""
    n = int(round(window_s * rs.FS)) + 1
    t = np.arange(n) / rs.FS
    if jitter:
        t = t + np.random.default_rng(seed).uniform(-jitter, jitter, n)
        t = np.maximum.accumulate(t)
    ts = (t * 1e6).astype("int64") + 40_000_000
    start = 40_000_000 + int(pre_s * 1e6)
    return ts, (start, start + int(exc_dur_s * 1e6))


def test_welch_coverage_fixed_pad_loses_the_chirp_top():
    ts, exc = _rows(11.0, 10.0, pre_s=0.5)                     # --pad 0.5 on a 10 s chirp
    cov = rs.welch_coverage(ts, exc, 1024)
    assert cov["last_row_s"] == pytest.approx(9.74, abs=1.5 / rs.FS)
    assert cov["covered_to_s"] == pytest.approx(8.96, abs=0.08)
    assert cov["min_weight"] < 0.01                            # the unanalysed tail
    assert cov["uncovered_tail_s"] > 1.0


def test_welch_coverage_auto_pad_covers_everything():
    for exc_dur in (10.0, 9.37):                               # also a non-multiple of the hop
        pb, pa = rs.auto_pads(1024)
        ts, exc = _rows(pb + exc_dur + pa, exc_dur, pre_s=pb)
        cov = rs.welch_coverage(ts, exc, 1024)
        assert cov["min_weight"] >= rs.COVER_MIN_WEIGHT - 5e-3
        assert cov["covered_from_s"] == pytest.approx(0.0, abs=1.0 / rs.FS)
        assert cov["covered_to_s"] == pytest.approx(exc_dur, abs=1.0 / rs.FS)
        assert cov["uncovered_head_s"] < rs.COVER_WARN_S and cov["uncovered_tail_s"] < rs.COVER_WARN_S


def test_welch_coverage_irregular_rows():
    # real logs jitter by a few ms (mean 2.5 ms, max ~5.7 ms between rows): the
    # check works on the rows themselves, not on a nominal 400 Hz grid
    pb, pa = rs.auto_pads(1024)
    ts, exc = _rows(pb + 10.0 + pa, 10.0, jitter=1.2e-3, seed=5, pre_s=pb)
    assert np.all(np.diff(ts) >= 0)
    cov = rs.welch_coverage(ts, exc, 1024)
    assert cov["min_weight"] >= rs.COVER_MIN_WEIGHT - 5e-3
    # 16 dropped rows (a 40 ms gap) shift the segment grid in time: the last analysed
    # row is the same row index, hence ~40 ms later; seen only when working on rows
    keep = np.ones(len(ts), bool)
    keep[2000:2016] = False
    cov_gap = rs.welch_coverage(ts[keep], exc, 1024)
    assert cov_gap["last_row_s"] == pytest.approx(cov["last_row_s"] + 16 / rs.FS, abs=3e-3)
    # an explicit short window: the same pure function reports the loss
    ts2, exc2 = _rows(10.5 + 0.2, 10.0, jitter=1.2e-3, seed=6, pre_s=0.2)
    assert rs.welch_coverage(ts2, exc2, 1024)["uncovered_tail_s"] > rs.COVER_WARN_S


def _noisy_chirp_fit(pad_s, monkeypatch, seed=11):
    """Fit a noisy 10 s 1-25 Hz chirp with hover before/after; also capture the ETFE
    (omega, G, coherence) that fit_from_df computed. / ETFE も取り出す。"""
    rng = np.random.default_rng(seed)
    df, s_us, e_us = _make_df(lead_s=4.0, chirp_s=10.0, tail_s=4.0)
    df["gyro_x"] = df["gyro_x"] + rng.normal(0, 0.1, len(df))   # sensor noise throughout
    df.attrs["text_log"] = _text_log([(s_us, START.format(ax=0)), (e_us, DONE)])
    win, info = rs.resolve_window(df, "roll", pad_s=pad_s)
    seen = {}
    real_etfe = rs.etfe

    def spy(*a, **k):
        seen["etfe"] = real_etfe(*a, **k)
        return seen["etfe"]
    monkeypatch.setattr(rs, "etfe", spy)
    res = rs.fit_from_df(df, "roll", f_lo=1.0, f_hi=25.0, input_mode="duty", window_us=win,
                         nperseg=info["nperseg"], exc_us=info["exc_us"])
    return (res,) + seen["etfe"]


def test_auto_pad_raises_high_band_coherence_on_a_noisy_chirp(monkeypatch):
    res_old, om, G_old, coh_old = _noisy_chirp_fit(0.5, monkeypatch)
    res_new, om2, G_new, coh_new = _noisy_chirp_fit("auto", monkeypatch)
    hi = om / (2 * np.pi) >= 20.0
    assert hi.sum() > 10
    assert coh_new[hi].mean() > coh_old[hi].mean() + 0.1
    assert coh_new[hi].mean() > 0.6
    # 20-25 Hz FRF vs the true plant within a few dB (the old pad is checked to be worse)
    truth = rs.plant_response(om2[hi], B_TRUE, T_TRUE, L_TRUE)
    err_old = np.abs(20 * np.log10(np.abs(G_old[hi]) / np.abs(truth))).mean()
    err_new = np.abs(20 * np.log10(np.abs(G_new[hi]) / np.abs(truth))).mean()
    assert err_new < 3.0 and err_old > 2 * err_new
    assert res_new["window"]["coverage"]["min_weight"] >= rs.COVER_MIN_WEIGHT - 5e-3
    assert res_old["window"]["coverage"]["covered_to_s"] < 9.0
    assert res_new["window"]["nperseg"] == 1024 and res_old["window"]["nperseg"] == 1024
