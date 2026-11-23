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
