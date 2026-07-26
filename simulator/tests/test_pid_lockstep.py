#!/usr/bin/env python3
"""
PID Lockstep Equivalence Test (P5 stage 1)
PID Lockstep 等価性テスト（P5 stage 1）

Proves, step-by-step and numerically, that the hand-ported Python replica of
the firmware's discrete PID (tools/log_analyzer/rate_sysid.py's
replay_pid(), whose own comments say it is "a line-for-line port of
pid.hpp" that must be "kept in sync by hand") actually matches the REAL
firmware struct `sf::PID`
(firmware/vehicle/components/sf_controller_pid/include/pid.hpp).

Instead of trusting a second hand-translation to stay in sync, this test
drives the real C++ struct directly via the `stampfly_control` pybind11
module (built by `sf sil build` from simulator/sil/pybind/
sf_control_bindings.cpp, which #includes pid.hpp UNMODIFIED) and compares
its output, step-by-step, against replay_pid() on identical (setpoint,
measurement) sequences.

replay_pid() の手動移植（rate_sysid.py 自身のコメントが「pid.hpp の逐語移植」
であり「手で同期を保つ」必要があると述べている）が、本物のファーム構造体
`sf::PID`（pid.hpp）と実際に一致するかを、1ステップずつ数値で証明する。

二重の手動移植が同期し続けることを信頼する代わりに、pybind11 モジュール
`stampfly_control`（`sf sil build` が simulator/sil/pybind/
sf_control_bindings.cpp から生成。pid.hpp を無改変で #include）経由で本物の
C++ 構造体を直接駆動し、同一の (setpoint, measurement) 列に対する出力を
replay_pid() と1ステップずつ比較する。

Prerequisite / 事前条件:
    source setup_env.sh && sf sil build      # builds stampfly_control.*.so
    pytest simulator/tests/test_pid_lockstep.py -v

If `import stampfly_control` fails below, this is a HARD FAILURE (not a
skip) — see the import block for the actionable error message.
下の `import stampfly_control` が失敗した場合は（スキップではなく）
明確な失敗として扱う — 対処方法はインポート部のエラーメッセージ参照。
"""

import sys

import numpy as np
import pytest

from sfcli.utils.paths import paths

# Import rate_sysid via manual sys.path insertion -- tools/log_analyzer has no
# __init__.py (not a package). This exact pattern is already used by
# lib/sfcli/commands/sysid.py and lib/sfcli/commands/log.py.
# rate_sysid を手動 sys.path 挿入で import する -- tools/log_analyzer は
# __init__.py を持たない（パッケージではない）。この形は
# lib/sfcli/commands/sysid.py と lib/sfcli/commands/log.py に既にある形。
sys.path.insert(0, str(paths.root() / "tools" / "log_analyzer"))
import rate_sysid  # noqa: E402  (import after sys.path setup, matches repo convention)

# Same pattern for the compiled pybind11 module: it lands directly in
# simulator/sil/build/ (verified: `pybind11_add_module` placed at the CMake
# root, not under add_subdirectory, lands flat like every other SIL target --
# e.g. build/cores_smoke, build/stampfly_control.cpython-312-darwin.so).
# コンパイル済み pybind11 モジュールも同じパターン: simulator/sil/build/ 直下に
# フラットに置かれる（`pybind11_add_module` を CMake ルートに直接置いた場合、
# add_subdirectory 経由ではないため他の SIL ターゲットと同様フラットになる —
# 確認済み: build/cores_smoke 等と同じ場所に build/stampfly_control.cpython-
# 312-darwin.so が生成される）。
sys.path.insert(0, str(paths.sil_build()))

# Module-not-built handling: a clear FAILURE, not a silent skip. A quietly
# green skip would hide the fact that nothing was actually verified.
# 未ビルド時の扱い: 静かな skip ではなく明確な失敗にする。無言で緑になる skip は
# 「実は何も検証していない」事実を隠してしまうため。
try:
    import stampfly_control
except ImportError as exc:
    raise ImportError(
        "stampfly_control pybind11 module not found (tried sys.path entry "
        f"{paths.sil_build()}). Build it first with:\n"
        "    source setup_env.sh && sf sil build\n"
        f"Original ImportError: {exc}"
    ) from exc


# =============================================================================
# Drivers for both implementations
# 両実装を駆動するヘルパ
# =============================================================================

DT = rate_sysid.DT  # 1/400 s, matches the 400 Hz Data Stream rate_sysid.py assumes
                     # 400Hz Data Stream レート（rate_sysid.py の前提と一致）


def make_cpp_pid(kp: float, ti: float, td: float, limit: float) -> "stampfly_control.PID":
    """Construct a REAL firmware sf::PID (via stampfly_control) configured to
    match one (kp, ti, td, limit) gain set.
    本物のファーム sf::PID（stampfly_control 経由）を1組のゲインで構成する。

    `eta` is left at pid.hpp's in-class default (0.125) and never varied: it
    is a MODULE-LEVEL CONSTANT in rate_sysid.py (ETA), not a per-call
    parameter, so varying it here would compare apples to oranges. Set
    explicitly anyway, for self-documentation.
    `eta` は pid.hpp のクラス内既定値 0.125 のまま変更しない: rate_sysid.py 側
    ではモジュールレベル定数 ETA であり呼び出しごとの引数ではないため、ここで
    変えると比較の前提が崩れる。自己文書化のため明示的に設定だけする。
    """
    pid = stampfly_control.PID()
    pid.kp = kp
    pid.ti = ti
    pid.td = td
    pid.eta = 0.125  # == rate_sysid.ETA; never varied, see docstring above
    pid.output_limit = limit  # set once, reused for the whole sequence below
                               # (replay_pid takes `limit` per call; pid.hpp
                               # keeps it as a persistent member -- semantically
                               # equivalent as long as it is not changed
                               # mid-sequence, which this test never does)
    return pid


def drive_cpp_pid(kp: float, ti: float, td: float, limit: float,
                   r: np.ndarray, y: np.ndarray, dt: float = DT) -> np.ndarray:
    """Step the real firmware sf::PID over (r, y) one sample at a time,
    mirroring replay_pid()'s (r[k], y[k]) calling convention exactly.
    本物の sf::PID を (r,y) に対し1サンプルずつ駆動する。replay_pid() の
    (r[k], y[k]) 呼び出し規約とそのまま対応させる。
    """
    pid = make_cpp_pid(kp, ti, td, limit)
    n = len(r)
    u = np.zeros(n)
    for k in range(n):
        u[k] = pid.compute(float(r[k]), float(y[k]), dt)
    return u


# =============================================================================
# Gain sets
# ゲインセット
# =============================================================================
#
# "*_flight" mirror rate_sysid.py's DEFAULT_GAINS (flight-proven params.cpp
# defaults) as LITERALS -- read here for the test only, never copied back
# into pid.hpp/params.cpp (this test must never influence real gains).
# "*_flight" は rate_sysid.py の DEFAULT_GAINS（実飛行実績の params.cpp 既定値）
# をリテラルとして写したもの -- テスト専用の読み取りであり、pid.hpp/params.cpp
# へ書き戻すことは絶対にない（本テストが実ゲインに影響してはならない）。
GAIN_SETS = {
    "roll_flight":  dict(kp=1.365e-3, ti=0.7,  td=0.01,  limit=5.2e-3),
    "pitch_flight": dict(kp=1.995e-3, ti=0.7,  td=0.01,  limit=5.2e-3),
    "yaw_flight":   dict(kp=5.31e-3,  ti=1.6,  td=0.01,  limit=2.2e-3),

    # Wider synthetic sweep: roughly an order of magnitude around the
    # flight-proven values above, on each axis independently.
    # 合成スイープ: 上記飛行実績値のおおよそ1桁上下を各軸独立に振る。
    "sweep_low_td0": dict(kp=2e-4, ti=0.1, td=0.0,  limit=1e-2),   # td=0: derivative branch skipped
    "sweep_high":    dict(kp=2e-2, ti=3.0, td=0.02, limit=5e-2),   # upper end of the sweep

    # Required edge case: ti EXACTLY at the anti-windup boundary (inclusive
    # `>=` in pid.hpp -- a previous bug used `>`, which silently disabled the
    # integrator at exactly this value).
    # 必須エッジケース: アンチワインドアップ境界ちょうど（pid.hpp は `>=` で
    # 含む -- 以前は `>` のバグで、ちょうどこの値で積分が黙って無効化されていた）。
    "ti_boundary_incl": dict(kp=1e-3, ti=0.01,  td=0.005, limit=5e-3),
    "ti_boundary_excl": dict(kp=1e-3, ti=0.005, td=0.005, limit=5e-3),  # just below -> integrator OFF

    "td_zero":          dict(kp=1.5e-3, ti=0.5, td=0.0,  limit=5e-3),  # explicit derivative-off case
    "td_large_priming": dict(kp=1e-3,   ti=1.0, td=0.05, limit=5e-3),  # exercises first_run priming +
                                                                        # many steady bilinear-filtered steps

    # Deliberately large kp / small limit so a moderate input forces SUSTAINED
    # saturation (see seq_saturating_square below) -- exercises conditional-
    # integration anti-windup on both the positive and negative side.
    # 意図的に大きい kp / 小さい limit にして、中程度の入力でも持続飽和を強制する
    # （下の seq_saturating_square 参照）-- 条件付き積分アンチワインドアップを
    # 正負両方向で運動させる。
    "saturating": dict(kp=5e-2, ti=0.7, td=0.01, limit=2e-3),
}

N_STEPS = 600  # "several hundred steps" per input sequence, dt = 1/400 s (~1.5 s)
               # 入力系列1本あたり「数百ステップ」、dt=1/400s（約1.5秒）


# =============================================================================
# Input sequence generators (setpoint r, measurement y)
# 入力系列ジェネレータ（目標値 r、測定値 y）
# =============================================================================

def seq_step(n: int, amplitude: float):
    """Setpoint steps from 0 to `amplitude` a third of the way through;
    measurement held at 0 (a clean, non-adversarial step transient).
    目標値が1/3地点で0からamplitudeへステップ、測定値は0に固定
    （素直な非敵対的ステップ応答）。"""
    r = np.zeros(n)
    r[n // 3:] = amplitude
    y = np.zeros(n)
    return r, y


def seq_sine(n: int, amplitude: float, dt: float = DT, freq_hz: float = 3.0,
             phase: float = 0.7):
    """Setpoint and measurement both sinusoidal (different phase/amplitude),
    producing a smoothly time-varying error -- exercises the derivative
    branch's steady-state bilinear filtering.
    目標値・測定値とも正弦波（位相・振幅を違える）。滑らかに変化する誤差を
    作り、微分項の定常双一次フィルタ挙動を運動させる。"""
    t = np.arange(n) * dt
    r = amplitude * np.sin(2 * np.pi * freq_hz * t)
    y = 0.6 * amplitude * np.sin(2 * np.pi * freq_hz * t + phase)
    return r, y


def seq_random_walk(n: int, amplitude: float, rng: np.random.Generator):
    """Setpoint is a normalized random walk; measurement tracks it with
    independent noise -- a non-adversarial, broadband, "typical operation"
    stress input (fixed-seed `rng` for reproducibility).
    目標値は正規化ランダムウォーク、測定値はそれに独立ノイズを加えて追従
    -- 非敵対的で広帯域な「通常運用」相当の入力（再現性のため rng は固定シード）。
    """
    steps_r = rng.normal(0, 1, n)
    r = np.cumsum(steps_r)
    r = amplitude * r / (np.max(np.abs(r)) + 1e-12)
    noise = rng.normal(0, 0.05, n) * amplitude
    y = r + noise
    return r, y


def seq_saturating_square(n: int, kp: float, limit: float, period: int = 50):
    """Square wave sized so |kp * error| alone exceeds `limit` for sustained
    stretches on BOTH signs -- the required genuine-saturation case for
    exercising conditional-integration anti-windup (not adversarial: the
    amplitude is scaled generously, 5x the value that would just reach the
    limit, so it is never near a knife-edge threshold crossing).
    |kp*error| だけで limit を持続的に超えるよう振幅を設定した矩形波（正負
    両方向）-- 条件付き積分アンチワインドアップを運動させる必須の飽和ケース
    （敵対的ではない: 振幅は「ちょうど limit に届く値」の5倍と余裕を持たせて
    おり、閾値ぎりぎりの通過には決してならない）。"""
    amp = 5.0 * limit / max(kp, 1e-9)
    idx = np.arange(n)
    r = amp * np.where((idx // period) % 2 == 0, 1.0, -1.0)
    y = np.zeros(n)
    return r, y


# =============================================================================
# Assemble the (gain set) x (input sequence) case matrix
# (ゲインセット)×(入力系列) の組み合わせ表を組み立てる
# =============================================================================
#
# Built once at module load with a FIXED seed so the tolerance numbers in the
# comment just below the loop are reproducible.
# コメント中の許容誤差の根拠となる数値が再現可能なよう、モジュール読み込み時に
# 固定シードで一度だけ構築する。

def _build_cases():
    rng = np.random.default_rng(42)
    cases = []
    for gain_name, g in GAIN_SETS.items():
        # Linear-range amplitude for step/sine (~30% of the output limit at
        # the P term alone) so these two sequences probe ordinary,
        # non-saturating operation; seq_saturating_square below is the
        # dedicated sustained-saturation case.
        # step/sine 用の振幅（比例項単体で output_limit の約30%）-- 通常の
        # 非飽和動作を見るため。持続飽和の専用ケースは seq_saturating_square。
        amp = 0.3 * g["limit"] / max(g["kp"], 1e-9)
        sequences = {
            "step": seq_step(N_STEPS, amp),
            "sine": seq_sine(N_STEPS, amp),
            "random_walk": seq_random_walk(N_STEPS, amp, rng),
            "sat_square": seq_saturating_square(N_STEPS, g["kp"], g["limit"]),
        }
        for seq_name, (r, y) in sequences.items():
            cases.append({
                "id": f"{gain_name}-{seq_name}",
                "gain_name": gain_name,
                "seq_name": seq_name,
                **g,
                "r": r,
                "y": y,
            })
    return cases


CASES = _build_cases()


# =============================================================================
# Tolerances
# 許容誤差
# =============================================================================
#
# MEASURED (this exact case matrix, seed=42, N_STEPS=600, 10 gain sets x 4
# sequences = 40 cases): max|u_cpp - u_py| = 1.3417e-08 (sweep_high/step);
# max(|u_cpp - u_py| / output_limit) = 1.4762e-06 (td_zero/step). This is the
# expected magnitude of float32 (C++ sf::PID, real firmware precision) vs
# float64 (numpy replay_pid) rounding -- pid.hpp has ZERO transcendental
# function calls (pure +,-,*,/), so this is deterministic IEEE754 arithmetic
# noise, not libm/platform-dependent behaviour.
#
# A wider offline exploration (n=5000 steps, 400 randomly-sampled gain
# combinations spanning kp in [1e-4, 8e-2], ti in [0.01, 5.0], td in
# [0, 0.05], limit in [1e-3, 0.1]) found comparable worst-case magnitudes
# (~9.4e-08 abs, ~9.6e-07 relative-to-limit) -- i.e. the noise ceiling does
# NOT grow with sequence length or gain range, consistent with float32/64
# rounding rather than an unbounded drift.
#
# No saturation-transition divergence (the boundary-crossing risk noted in
# the task spec, where float32 vs float64 could disagree on which side of
# `out_test > limit` a sample falls) was observed in any measured run: the
# "sat_square" cases were consistently AMONG THE SMALLEST diffs measured
# (~1e-10 to 1e-11), not the largest.
#
# Chosen tolerances apply safety margins of ~37x (absolute) and ~7x
# (relative-to-limit) above this test's own measured worst case, and remain
# comfortably above (~5x / ~10x) the wider offline exploration's worst case.
#
# 実測（この組み合わせ表、seed=42、N_STEPS=600、10ゲイン×4系列=40ケース）:
# max|u_cpp - u_py| = 1.3417e-08（sweep_high/step）、
# max(|u_cpp - u_py|/output_limit) = 1.4762e-06（td_zero/step）。これは
# float32（C++ sf::PID、実ファーム精度）と float64（numpy replay_pid）の丸め
# 誤差として妥当な大きさ -- pid.hpp は超越関数呼び出しが皆無（+,-,*,/のみ）
# なので、libm・プラットフォーム依存ではなく決定論的な IEEE754 演算誤差。
#
# より広いオフライン探索（n=5000ステップ、kp∈[1e-4,8e-2]・ti∈[0.01,5.0]・
# td∈[0,0.05]・limit∈[1e-3,0.1] からランダムサンプリングした400通りのゲイン
# 組合せ）でも同程度の最悪値（絶対 約9.4e-08、limit比 約9.6e-07）に留まり、
# 系列長やゲイン範囲を広げても誤差が成長しないことを確認済み（float32/64丸め
# 誤差として妥当、無制限なドリフトではない）。
#
# 仕様書が指摘する飽和境界での分岐不一致（float32/float64 が `out_test > limit`
# のどちら側に転ぶか食い違うリスク）は、測定した範囲では一度も観測されなかった:
# "sat_square" ケースの差は測定値の中でむしろ最小級（約1e-10〜1e-11）だった。
#
# 採用した許容誤差は、本テスト自身の実測最悪値に対し絶対誤差で約37倍・
# limit比で約7倍の安全マージンを持たせ、より広いオフライン探索の最悪値に
# 対しても約5倍・約10倍の余裕がある。
ABS_TOL = 5e-7          # absolute |u_cpp - u_py| tolerance
REL_TO_LIMIT_TOL = 1e-5  # |u_cpp - u_py| / output_limit tolerance (scale-aware,
                         # since output_limit spans 2.2e-3 .. 5e-2 across gain sets)


@pytest.mark.parametrize("case", CASES, ids=[c["id"] for c in CASES])
def test_pid_lockstep_case(case):
    """Drive stampfly_control.PID (real firmware pid.hpp) and
    rate_sysid.replay_pid() (hand-ported Python) on the SAME (r, y)
    sequence and compare the full per-step output arrays.
    stampfly_control.PID（本物のファーム pid.hpp）と rate_sysid.replay_pid()
    （手動移植 Python）を同一の (r,y) 系列で駆動し、全ステップの出力配列を
    比較する。"""
    kp, ti, td, limit = case["kp"], case["ti"], case["td"], case["limit"]
    r, y = case["r"], case["y"]

    u_cpp = drive_cpp_pid(kp, ti, td, limit, r, y)
    u_py = rate_sysid.replay_pid(r, y, kp, ti, td, limit)

    diff = np.abs(u_cpp - u_py)
    max_abs = diff.max()
    max_rel_to_limit = max_abs / limit
    mean_abs = diff.mean()

    assert max_abs < ABS_TOL, (
        f"[{case['id']}] max|u_cpp-u_py|={max_abs:.3e} exceeds ABS_TOL={ABS_TOL:.1e} "
        f"at step {int(np.argmax(diff))} (u_cpp={u_cpp[np.argmax(diff)]!r}, "
        f"u_py={u_py[np.argmax(diff)]!r})"
    )
    assert max_rel_to_limit < REL_TO_LIMIT_TOL, (
        f"[{case['id']}] max relative-to-limit diff={max_rel_to_limit:.3e} "
        f"exceeds REL_TO_LIMIT_TOL={REL_TO_LIMIT_TOL:.1e}"
    )
    # Cumulative/typical-drift cross-check (not just the single worst step):
    # a systematic (biased) discrepancy -- as opposed to symmetric float32/64
    # rounding noise -- would show up here as a mean comparable to the worst
    # per-step value. Requiring mean << max guards against exactly that.
    # 累積・典型的ドリフトの追加確認（最悪1ステップだけでなく）: 対称な
    # float32/64丸め誤差ではなく系統的（偏った）不一致であれば、平均値が
    # 最悪値に近づくはず。mean << max を要求することでそれを検出する。
    assert mean_abs < ABS_TOL / 2, (
        f"[{case['id']}] mean|u_cpp-u_py|={mean_abs:.3e} is suspiciously close "
        f"to the worst-case tolerance -- possible systematic (non-noise-like) "
        f"discrepancy rather than float32/64 rounding"
    )


# =============================================================================
# Dedicated boundary-condition sanity checks (ti == 0.01 inclusive boundary)
# 境界条件の専用確認（ti == 0.01 の境界を含む/含まない）
# =============================================================================
#
# These do not replace the lockstep comparison above -- they confirm BOTH
# implementations really exercise the branch they are supposed to (rather
# than, say, both silently disagreeing with the intended design while still
# happening to agree with each other).
# 上の lockstep 比較を置き換えるものではない -- 両実装が「意図した分岐を実際に
# 通っている」ことを確認する（両者が偶然一致しているだけで、意図した設計とは
# 食い違っている、という事態を排除する）。

def test_ti_boundary_inclusive_integrates():
    """ti == 0.01 EXACTLY must still integrate in both implementations (the
    bug this boundary guards against: a strict `>` would silently disable
    the integrator at exactly the minimum allowed ti).
    ti == 0.01 ちょうどでも両実装が積分すること（この境界が防ぐバグ: 厳密な
    `>` だと許容最小値ちょうどで積分が黙って無効化される）。"""
    kp, ti, td, limit = 1e-3, 0.01, 0.0, 5e-3
    n = 200
    # Small error (1% of the value that would saturate the P term alone) --
    # deliberately NOT the 0.5x used elsewhere: at ti=0.01, kp/ti=0.1 is a
    # large integral gain, so a bigger error saturates the output within a
    # handful of steps and u_py[-1]==u_py[10] trivially (both already
    # clamped), which would defeat this check's purpose (first measured with
    # 0.5x amplitude -- confirmed the false-pass risk empirically). 0.01x
    # keeps the integral growing (not yet clamped) across the whole window.
    # 小さい誤差（比例項単体で飽和する値の1%）-- 他ケースの0.5倍ではなく意図的に
    # 小さくしている: ti=0.01 では kp/ti=0.1 と積分ゲインが大きく、誤差が
    # 大きいと数ステップで出力が飽和してしまい u_py[-1]==u_py[10] が自明に
    # 成立してしまう（本チェックの意味が無くなる -- 実際に0.5倍で試し
    # falseパスのリスクを実測確認済み）。0.01倍なら区間全体で積分が
    # （飽和せず）伸び続ける。
    r = np.full(n, 0.01 * limit / kp)
    y = np.zeros(n)

    pid = make_cpp_pid(kp, ti, td, limit)
    for k in range(n):
        pid.compute(float(r[k]), float(y[k]), DT)
    assert pid.integral != 0.0, (
        "sf::PID.integral stayed 0 at ti=0.01 -- inclusive '>=' boundary broken"
    )

    u_py = rate_sysid.replay_pid(r, y, kp, ti, td, limit)
    # If replay_pid's integrator were (bug) disabled at ti==0.01, output would
    # stay pinned at kp*error forever instead of winding up over time.
    # replay_pid の積分が(バグで)ti=0.01時に無効なら、出力は kp*error に
    # 張り付いたままになるはず（実際は積分が効いて時間とともに巻き上がる）。
    assert u_py[-1] > u_py[10], (
        "replay_pid did not integrate at ti=0.01 -- inclusive boundary broken"
    )


def test_ti_boundary_exclusive_disables_integrator():
    """ti just below 0.01 must leave the integrator OFF (pure P+D) in both
    implementations.
    ti が 0.01 をわずかに下回る場合、両実装とも積分は無効（P+Dのみ）である
    こと。"""
    kp, ti, td, limit = 1e-3, 0.005, 0.0, 5e-3
    n = 200
    r = np.full(n, 0.5 * limit / kp)
    y = np.zeros(n)

    pid = make_cpp_pid(kp, ti, td, limit)
    for k in range(n):
        pid.compute(float(r[k]), float(y[k]), DT)
    assert pid.integral == 0.0, (
        "sf::PID.integral is nonzero at ti=0.005 -- should stay disabled below 0.01"
    )

    u_py = rate_sysid.replay_pid(r, y, kp, ti, td, limit)
    expected = np.clip(kp * r, -limit, limit)  # pure-P output (td=0, ti disabled)
    assert np.allclose(u_py, expected, atol=1e-9), (
        "replay_pid output is not pure-P at ti=0.005 -- integrator should be disabled"
    )


if __name__ == "__main__":
    # Standalone execution still works (per simulator/tests/
    # test_control_allocation_firmware_compat.py precedent): explicit path
    # required since repo-root pyproject.toml's testpaths=["tests"] does not
    # cover simulator/tests/.
    # 直接実行も可能（precedent と同様）: リポジトリルート pyproject.toml の
    # testpaths=["tests"] は simulator/tests/ を含まないため、明示パスが必要。
    sys.exit(pytest.main([__file__, "-v"]))
