#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_fallback.py — 9/10 SCI チュートリアル SILS 保険素材ジェネレータ
make_fallback.py — SILS-derived fallback material generator for the 9/10 SCI tutorial

会場で実機デモが失敗した場合の保険として、SILS（MuJoCo）で飛ばした結果からグラフ
PNG・テキストサマリを生成する。動画（*.mp4）は `sf sils scenario ... --video` が
既に書き出したファイルをそのままこのディレクトリへコピーするだけで、このスクリプト
自身は動画エンコードを行わない。

Generates PNG graphs + text summaries from SILS (MuJoCo) flight bundles, as a
fallback in case the live hardware demo fails at the venue. Videos (*.mp4) are
produced separately by `sf sils scenario ... --video`; this script only copies
them into place (no video encoding here).

--------------------------------------------------------------------------------
使い方 / Usage
--------------------------------------------------------------------------------

1) 既定（速い）: 既に生成済みの SILS バンドル（simulator/sils/viz/out_scn_*/）
   から PNG・テキストを再生成するだけ。バンドルが無ければエラーで止まり、下の
   RUN コマンドを案内する。
   Default (fast): rebuild PNGs/text from SILS bundles that already exist under
   simulator/sils/viz/out_scn_*/. Errors out with the RUN command below if a
   bundle is missing.

     python3 docs/events/sci_tutorial_2026/fallback/make_fallback.py

2) --run: S1 (pos_roll) と S5 (stab_flight) の SILS シナリオを実行してから
   生成する。vehicle ターゲットのみを動かすので、workshop レッスンの状態を
   変更しない（安全・冪等）。
   --run: (re-)executes the S1 (pos_roll) and S5 (stab_flight) SILS scenarios
   first, then builds outputs. Only runs the `vehicle` target, so it never
   touches the workshop lesson state (safe, idempotent).

     python3 docs/events/sci_tutorial_2026/fallback/make_fallback.py --run

3) --run-s4: S4（Lesson 5 P制御 vs Lesson 8 PID のロールステップ比較）も
   再実行する。`sf lesson switch` で firmware/workshop/main/user_code.cpp を
   書き換えるため、実行前のレッスンを --restore-lesson で指定した番号に必ず
   戻す（既定 0 = Lesson 0 student）。事前に `sf lesson list` で現在のレッスン
   を確認し、必要なら --restore-lesson で上書きすること。
   --run-s4: also re-runs S4 (Lesson 5 P-control vs Lesson 8 PID roll-step
   comparison). This REWRITES firmware/workshop/main/user_code.cpp via
   `sf lesson switch`, so it always restores the lesson given by
   --restore-lesson afterward (default 0 = Lesson 0 student). Check the
   current lesson with `sf lesson list` first and override
   --restore-lesson if it is not 0.

     python3 docs/events/sci_tutorial_2026/fallback/make_fallback.py --run --run-s4 --restore-lesson 0

4) --plots-only: PNG のラベルだけを直したい場合に使う。既存の SILS バンドル・
   永続化済み一式（.sflog.zip）からグラフのみ再生成し、4本の動画ファイルには
   一切触れない（バンドルに新しい動画があっても使わない）。--run/--run-s4 と
   同時指定は不可。
   --plots-only: use this when only a label needs fixing. Re-plots from
   already-persisted bundles (.sflog.zip) only — never touches any of the
   four video files (even if the bundle happens to carry a fresh one).
   Incompatible with --run/--run-s4.

     python3 docs/events/sci_tutorial_2026/fallback/make_fallback.py --plots-only

生成ファイル一覧は docs/events/sci_tutorial_2026/fallback/README.md の表を参照。
See docs/events/sci_tutorial_2026/fallback/README.md's table for the full output file list.

@design docs/events/sci_tutorial_2026/verification_checklist.md §5 — 動画・ログの保険取得
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt

# --- paths -------------------------------------------------------------------
# Walk up from this file to find the repo root (marked by setup_env.sh) instead
# of hardcoding a parents[] index, so this script keeps working if the docs/
# tree it lives under is ever restructured again (it moved once already, from
# docs/sci_tutorial/fallback/ to docs/events/sci_tutorial_2026/fallback/).
# このファイルから上へ辿って setup_env.sh のあるリポジトリルートを探す。
# parents[]のインデックス固定をやめることで、docsツリーが再編されても
# 追従できる（実際に docs/sci_tutorial/fallback/ から
# docs/events/sci_tutorial_2026/fallback/ へ一度移動済み）。
ROOT = next(
    p for p in Path(__file__).resolve().parents if (p / "setup_env.sh").exists()
)
SILS_DIR = ROOT / "simulator" / "sils"
VIZ_DIR = SILS_DIR / "viz"
OUT_DIR = Path(__file__).resolve().parent

# lib/sflog reads the flight-log v1 bundle the SILS emulator now writes per run
# (docs/plans/flight-log-format-plan.md section 3.3, replacing the old per-run CSV).
# This script runs under the project's own venv (numpy/pandas/matplotlib are already
# importable above), which normally already has sflog importable too -- this sys.path
# insertion is a defensive fallback for an interpreter that does not.
# lib/sflog は SILS エミュレータが実行ごとに書くフライトログ v1 一式を読む
# （計画書 3.3節、旧・実行ごとの CSV の後継）。本スクリプトはプロジェクト自身の venv
# （numpy/pandas/matplotlib は上で既に import 可能）で動き、通常は sflog も import
# 可能だが、そうでない実行環境向けの保険としてパスを追加する。
LIB_DIR = ROOT / "lib"
if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))
import sflog  # noqa: E402  flight-log v1 bundle reader

# Slide-readable plotting defaults: 150dpi, landscape, >=14pt fonts (project
# slide rules — docs/events/sci_tutorial_2026/fallback is reviewed at data-projector size;
# figures are embedded on the SCI appendix "demo fallback" slides scaled to
# ~0.68\textheight, so undersized fonts there vanish at projector distance).
# スライド投影で読めるデフォルト: 150dpi・横長・14pt 以上。SCI付録「デモ保険」
# スライドで約0.68\textheightに縮小埋め込みされるため、フォントが小さいと
# 投影距離で読めなくなる。
plt.rcParams.update({
    "figure.dpi": 150,
    "savefig.dpi": 150,
    "font.size": 14,
    "axes.titlesize": 16,
    "axes.labelsize": 14,
    "legend.fontsize": 13,
    "xtick.labelsize": 13,
    "ytick.labelsize": 13,
})


def _find_cjk_font() -> str | None:
    """Return the name of an installed Japanese-capable font, checked in a
    preference order covering common macOS/Windows/Linux fonts, or None if
    none is installed. Used to decide whether in-plot text may use Japanese
    labels; without a CJK font matplotlib silently draws missing glyphs as
    empty tofu boxes instead of raising an error, so this check must run
    before any Japanese text is placed on a figure.
    インストール済みの日本語対応フォント名を返す（無ければ None）。macOS /
    Windows / Linux の代表的な日本語フォントを優先順で確認する。プロット文字
    を日本語にしてよいか判定するために使う — CJKフォントが無い環境では
    matplotlib はエラーを出さず文字化け（tofu ボックス）を描くだけなので、
    日本語を置く前に必ずこのチェックを通す。
    """
    candidates = [
        "Hiragino Sans", "Hiragino Kaku Gothic ProN",   # macOS
        "Noto Sans CJK JP", "Noto Sans JP",              # Linux (Noto)
        "Yu Gothic", "Meiryo", "MS Gothic",              # Windows
        "IPAexGothic", "IPAGothic", "TakaoGothic",       # Linux (IPA/Takao)
    ]
    available = {f.name for f in fm.fontManager.ttflist}
    return next((name for name in candidates if name in available), None)


# Detected once at import time: drives the Japanese/English fallback choice
# for the few in-plot labels that carry Japanese text (S1 pass-criterion
# label, S4 lesson/exercise titles). All other plot titles in this script are
# English-only and unaffected.
# 起動時に一度だけ判定: 日本語テキストを含む一部のプロット表示（S1の判定条件
# ラベル、S4のレッスン／実習タイトル）で日本語／英語のどちらを使うか決める。
# このスクリプトの他のタイトルは元々英語のみで影響を受けない。
_CJK_FONT = _find_cjk_font()
if _CJK_FONT:
    plt.rcParams["font.family"] = _CJK_FONT
    print(f"[make_fallback] CJK font found ({_CJK_FONT}) — using Japanese plot labels")
else:
    print("[make_fallback] no CJK font found — falling back to English plot labels")

RAD2DEG = 57.29577951308232


# --- shell helper for --run / --run-s4 ----------------------------------------
def sf(cmd: str, extra_env: str = "") -> None:
    """Run an `sf ...` command with the sourced ESP-IDF env + SF_ROOT_OVERRIDE
    pointed at THIS checkout (see task note: this Mac's default `sf` resolves
    against a different clone). Raises on non-zero exit.
    ESP-IDF 環境＋SF_ROOT_OVERRIDE（このチェックアウトを指す）付きで `sf ...`
    を実行する。非ゼロ終了で例外。
    """
    full = (
        f"source setup_env.sh >/dev/null 2>&1 && "
        f"{extra_env}SF_ROOT_OVERRIDE={ROOT} PYTHONPATH={ROOT}/lib {cmd}"
    )
    print(f"$ {cmd}")
    r = subprocess.run(["bash", "-lc", full], cwd=ROOT)
    if r.returncode != 0:
        raise SystemExit(f"command failed (exit {r.returncode}): {cmd}")


def bundle_dir(scn_stem: str) -> Path:
    return VIZ_DIR / f"out_scn_{scn_stem}"


def _find_bundle_zip(bundle: Path):
    """Newest `sils_*.sflog.zip` under a SILS run's bundle dir, or None if the
    run has not produced one yet.
    SILS 実行のバンドルディレクトリ直下にある最新の `sils_*.sflog.zip`
    （まだ無ければ None）。
    """
    candidates = sorted(Path(bundle).glob("sils_*.sflog.zip"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def require_bundle(scn_stem: str, run_hint: str) -> Path:
    b = bundle_dir(scn_stem)
    if _find_bundle_zip(b) is None:
        raise SystemExit(
            f"missing SILS flight-log bundle: {b}/sils_*.sflog.zip\n"
            f"Run first (or pass --run):\n  {run_hint}"
        )
    return b


def _copy_bundle_zip(bundle: Path, dest_dir: Path) -> None:
    """Copy a SILS run's flight-log bundle (`sils_*.sflog.zip`) into
    `dest_dir`, keeping its original filename (`_find_bundle_zip()`/
    `load_traj()` look it up by name pattern + mtime, not a fixed name).
    SILS 実行のフライトログ一式（`sils_*.sflog.zip`）を、元のファイル名の
    まま `dest_dir` へコピーする（`_find_bundle_zip()`/`load_traj()` は
    固定名ではなく名前パターン＋mtimeで探すため）。
    """
    zip_path = _find_bundle_zip(bundle)
    if zip_path is None:
        raise SystemExit(f"missing SILS flight-log bundle to persist: {bundle}/sils_*.sflog.zip")
    shutil.copyfile(zip_path, dest_dir / zip_path.name)


_SQRT_HALF = 0.7071067811865476  # 1/sqrt(2)


def _euler_from_quat_ned(qw, qx, qy, qz):
    """roll/pitch/yaw [rad] from a body(FRD)->NED quaternion (w,x,y,z), vectorized
    over numpy arrays. Standard aerospace 3-2-1 Euler extraction; pitch is clipped
    to arcsin's domain to absorb float round-off near +-90 deg (gimbal lock).
    機体(FRD)→NED のクォータニオンから roll/pitch/yaw [rad]（numpy 配列対応）。
    標準的な航空 3-2-1 オイラー角抽出。pitch は arcsin の定義域にクランプする
    （±90度付近の浮動小数点丸め対策）。
    """
    roll = np.arctan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def _ned_to_mujoco_pos(pos_x, pos_y, pos_z):
    """StampFly NED position -> MuJoCo/ENU world position (px, py, pz). The exact
    inverse of simulator/sils/frames/frames.hpp's `ned_to_enu()` ({n.y, n.x, -n.z}),
    reimplemented here because this Python reader does not link the C++ frames module.
    StampFly の NED 位置 -> MuJoCo/ENU 世界座標 (px, py, pz)。frames.hpp の
    `ned_to_enu()`（{n.y, n.x, -n.z}）そのままの逆変換 -- この Python 読み込み側は
    C++ の frames モジュールをリンクしないため、ここに再実装する。
    """
    return pos_y, pos_x, -pos_z


def _quat_mul(w1, x1, y1, z1, w2, x2, y2, z2):
    """Hamilton product (w1,x1,y1,z1) * (w2,x2,y2,z2), vectorized."""
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return w, x, y, z


def _qnb_to_mujoco_quat(qw, qx, qy, qz):
    """StampFly q_nb (body FRD -> world NED) -> MuJoCo framequat (body FLU -> world
    ENU), vectorized. The exact inverse of frames.hpp's `qnb_to_mujoco_quat()`:
        q_mj = q_enu_to_ned().conj() * q_nb * q_frd_to_flu().conj()   (normalized)
    where q_enu_to_ned = (w=0, x=y=1/sqrt(2), z=0) and q_frd_to_flu = (w=0, x=1,
    y=z=0). Reimplemented in plain numpy since this reader does not link the C++
    frames module.
    StampFly の q_nb（機体FRD→世界NED）-> MuJoCo framequat（機体FLU→世界ENU）、
    配列対応。frames.hpp の `qnb_to_mujoco_quat()` そのままの逆変換（式は英語側参照）
    -- この読み込み側は C++ frames モジュールをリンクしないため素の numpy で再実装。
    """
    ew, ex, ey, ez = 0.0, _SQRT_HALF, _SQRT_HALF, 0.0            # q_enu_to_ned
    ew_c, ex_c, ey_c, ez_c = ew, -ex, -ey, -ez                    # its conjugate
    fw, fx, fy, fz = 0.0, 1.0, 0.0, 0.0                           # q_frd_to_flu
    fw_c, fx_c, fy_c, fz_c = fw, -fx, -fy, -fz                    # its conjugate
    w1, x1, y1, z1 = _quat_mul(ew_c, ex_c, ey_c, ez_c, qw, qx, qy, qz)
    w2, x2, y2, z2 = _quat_mul(w1, x1, y1, z1, fw_c, fx_c, fy_c, fz_c)
    norm = np.sqrt(w2 * w2 + x2 * x2 + y2 * y2 + z2 * z2)
    return w2 / norm, x2 / norm, y2 / norm, z2 / norm


def _asof_nearest(base: pd.DataFrame, other, col: str, n: int):
    """merge_asof(nearest) `col` from `other` onto `base`'s timestamp_us; an
    all-NaN array of length `n` if `other` is absent (the stream is missing
    from the bundle).
    `other` の `col` 列を `base` の timestamp_us へ merge_asof(nearest) で結合
    する。`other` が無ければ（バンドルにそのストリームが無ければ）長さ `n`
    の全 NaN 配列。
    """
    if other is None or col not in other.columns:
        return np.full(n, np.nan)
    other_sorted = other[["timestamp_us", col]].sort_values("timestamp_us", kind="stable")
    merged = pd.merge_asof(base[["timestamp_us"]], other_sorted, on="timestamp_us",
                            direction="nearest")
    return merged[col].to_numpy()


def load_traj(bundle: Path) -> pd.DataFrame:
    """Load a SILS run's flight-log v1 bundle (`sils_*.sflog.zip` under
    `bundle`) and rebuild the legacy trajectory-table columns this script's
    plotting code expects (t, px/py/pz, qw/qx/qy/qz in the MuJoCo/ENU frame,
    alt/roll/pitch [deg], yawrate [rad/s], yawcmd (always 0.0 -- retired, plan
    section 3.3), alt_est/roll_est/pitch_est, m0..m3), derived from the
    bundle's `truth`/`posvel`/`attitude`/`motor` streams -- see
    docs/plans/flight-log-format-plan.md section 3.3.

    Also stashes, in the returned DataFrame's `.attrs` (not a visible column,
    so callers that only know the legacy columns are unaffected):
      * `gyro_x_deg`/`gyro_y_deg`: the bundle's real gyro measurement
        (imu.gyro_x/y, matched onto `truth`'s timeline) -- ONLY when the
        bundle's `rate_ref` stream is present (see roll_rate_deg()'s
        docstring for why gating on `rate_ref`, not `imu`, matters here).
      * `rate_ref_roll_deg`: the bundle's real commanded roll-rate
        (rate_ref.rate_ref_roll) when that stream is present (see
        s4_rate_ref()'s docstring).

    SILS 実行のフライトログ v1 一式（`bundle` 直下の `sils_*.sflog.zip`）を
    読み、このスクリプトの描画コードが期待する旧トラジェクトリ表の列
    （t, MuJoCo/ENU 座標系の px/py/pz・qw/qx/qy/qz, alt/roll/pitch [度],
    yawrate [rad/s], yawcmd（常に0.0、廃止済み -- 計画書3.3節）,
    alt_est/roll_est/pitch_est, m0..m3）を、一式の `truth`/`posvel`/
    `attitude`/`motor` ストリームから再構築する -- 計画書 3.3節参照。

    返す DataFrame の `.attrs`（可視の列ではないため、旧列名しか知らない
    呼び出し側には影響しない）にも以下を格納する:
      * `gyro_x_deg`/`gyro_y_deg`: 一式の実測ジャイロ（imu.gyro_x/y を
        `truth` の時間軸へ対応付け）-- バンドルの `rate_ref` ストリームが
        ある場合のみ（`imu` ではなく `rate_ref` の有無で条件分けする理由は
        roll_rate_deg() の docstring 参照）。
      * `rate_ref_roll_deg`: そのストリームがあれば一式の実際の指令ロール
        レート（rate_ref.rate_ref_roll）（s4_rate_ref() の docstring 参照）。
    """
    zip_path = _find_bundle_zip(bundle)
    if zip_path is None:
        raise SystemExit(f"missing SILS flight-log bundle: {bundle}/sils_*.sflog.zip")
    log = sflog.load(zip_path)
    truth = log.streams.get("truth")
    if truth is None or len(truth) == 0:
        raise SystemExit(f"{zip_path}: bundle has no 'truth' stream (SILS-only stream)")
    truth = truth.sort_values("timestamp_us", kind="stable").reset_index(drop=True)
    n = len(truth)

    t_us = truth["timestamp_us"].to_numpy()
    qw, qx, qy, qz = (truth[c].to_numpy() for c in ("quat_w", "quat_x", "quat_y", "quat_z"))
    roll, pitch, _yaw = _euler_from_quat_ned(qw, qx, qy, qz)
    px, py, pz = _ned_to_mujoco_pos(*(truth[c].to_numpy() for c in ("pos_x", "pos_y", "pos_z")))
    mj_qw, mj_qx, mj_qy, mj_qz = _qnb_to_mujoco_quat(qw, qx, qy, qz)

    posvel, attitude, motor, rate_ref, imu = (
        log.streams.get(s) for s in ("posvel", "attitude", "motor", "rate_ref", "imu")
    )
    alt_est = -_asof_nearest(truth, posvel, "pos_z", n)
    if attitude is not None:
        cols = ["timestamp_us", "quat_w", "quat_x", "quat_y", "quat_z"]
        other_sorted = attitude[cols].sort_values("timestamp_us", kind="stable")
        merged = pd.merge_asof(truth[["timestamp_us"]], other_sorted, on="timestamp_us",
                                direction="nearest")
        roll_est_rad, pitch_est_rad, _ = _euler_from_quat_ned(
            merged["quat_w"].to_numpy(), merged["quat_x"].to_numpy(),
            merged["quat_y"].to_numpy(), merged["quat_z"].to_numpy(),
        )
    else:
        roll_est_rad = pitch_est_rad = np.full(n, np.nan)

    df = pd.DataFrame({
        "t": t_us / 1e6,
        "px": px, "py": py, "pz": pz,
        "qw": mj_qw, "qx": mj_qx, "qy": mj_qy, "qz": mj_qz,
        "alt": -truth["pos_z"].to_numpy(),
        "roll": roll * RAD2DEG, "pitch": pitch * RAD2DEG,
        "yawrate": truth["rate_z"].to_numpy(), "yawcmd": np.zeros(n),
        "alt_est": alt_est,
        "roll_est": roll_est_rad * RAD2DEG, "pitch_est": pitch_est_rad * RAD2DEG,
        "m0": _asof_nearest(truth, motor, "duty_FR", n),
        "m1": _asof_nearest(truth, motor, "duty_RR", n),
        "m2": _asof_nearest(truth, motor, "duty_RL", n),
        "m3": _asof_nearest(truth, motor, "duty_FL", n),
    })

    # rate_ref present -> the run's full control-topic instrumentation was active
    # (vehicle target): stash the real commanded rate and the real gyro measurement
    # for s4_rate_ref()/roll_rate_deg()/pitch_rate_deg() to prefer over their
    # scripted-stick / truth-differentiation fallbacks.
    # rate_ref がある -> 実行の制御系トピック計装が有効（vehicle ターゲット）:
    # s4_rate_ref()/roll_rate_deg()/pitch_rate_deg() がそれぞれの台本化スティック／
    # 真値微分フォールバックより優先して使えるよう、実際の指令レートと実測
    # ジャイロを格納しておく。
    if rate_ref is not None:
        cols = rate_ref[["timestamp_us", "rate_ref_roll"]].sort_values("timestamp_us", kind="stable")
        merged = pd.merge_asof(truth[["timestamp_us"]], cols, on="timestamp_us", direction="nearest")
        df.attrs["rate_ref_roll_deg"] = np.degrees(merged["rate_ref_roll"].to_numpy())
        if imu is not None:
            gyro_cols = imu[["timestamp_us", "gyro_x", "gyro_y"]].sort_values(
                "timestamp_us", kind="stable")
            merged = pd.merge_asof(truth[["timestamp_us"]], gyro_cols, on="timestamp_us",
                                    direction="nearest")
            df.attrs["gyro_x_deg"] = np.degrees(merged["gyro_x"].to_numpy())
            df.attrs["gyro_y_deg"] = np.degrees(merged["gyro_y"].to_numpy())

    return df


def roll_rate_deg(df: pd.DataFrame) -> np.ndarray:
    """Roll angular rate [deg/s]. Prefers the bundle's real gyro measurement
    (imu.gyro_x, body FRD roll axis, stashed in `df.attrs["gyro_x_deg"]` by
    load_traj()) when the bundle's `rate_ref` stream is present -- meaning the
    run's full control-topic instrumentation was active (vehicle target).
    Falls back to numerically differentiating the truth roll ANGLE (this
    script's "roll" column) ONLY when `rate_ref` is absent -- the workshop
    target's `WorkshopControlTask` never publishes vehicle's
    `sf::control_output` topic, so the bundle carries no `rate_ref` (or,
    consequently, real gyro comparison) for it (see this script's module
    docstring / README). Noise is off for every fallback run, so the
    truth-derived rate is numerically equivalent to what a noiseless gyro
    would read.
    ロール角速度[deg/s]。バンドルの `rate_ref` ストリームがある（実行の制御系
    トピック計装が有効 = vehicle ターゲット）場合は実測ジャイロ
    （imu.gyro_x、機体FRDロール軸。load_traj() が `df.attrs["gyro_x_deg"]` に
    格納）を優先する。`rate_ref` が無い（workshop ターゲットの
    `WorkshopControlTask` は vehicle の `sf::control_output` トピックを
    発行しないためバンドルに `rate_ref`（延いては実測ジャイロとの比較）が
    無い -- 詳しくはこのファイルの docstring / README 参照）場合のみ、真値
    ロール角（このスクリプトの "roll" 列）の数値微分にフォールバックする。
    全ての保険実行は noise=off なので、真値の微分はノイズ無しジャイロの
    読み値と数値的に一致する。
    """
    gyro_x_deg = df.attrs.get("gyro_x_deg")
    if gyro_x_deg is not None:
        return gyro_x_deg
    return np.gradient(df["roll"].values, df["t"].values)


def pitch_rate_deg(df: pd.DataFrame) -> np.ndarray:
    """Pitch angular rate [deg/s] -- see roll_rate_deg() for the gyro-vs-
    differentiation selection rule (df.attrs["gyro_y_deg"] here)."""
    gyro_y_deg = df.attrs.get("gyro_y_deg")
    if gyro_y_deg is not None:
        return gyro_y_deg
    return np.gradient(df["pitch"].values, df["t"].values)


# =============================================================================
# S1 — POS_HOLD fallback (pos_roll.scn, target=vehicle)
# =============================================================================
S1_SCN = "pos_roll"
S1_RUN_CMD = (
    "sf sils scenario simulator/sils/scenarios/pos_roll.scn --target vehicle --video"
)
# Schedule derived from simulator/sils/scenarios/pos_roll.scn's own event
# timeline (cumulative "+" hold_ms): ARM edge at row B start, roll-disturbance
# step C2 in [6.3,6.9]s, POS_HOLD engages at the start of row D (6.9s), the
# G3 numeric gate window is [7.6,21.6]s (0.7s settle margin after engage),
# scripted DISARM edge ~21.3s.
# pos_roll.scn 自身のイベントタイムラインから算出: ARM は行B開始、ロール外乱
# ステップC2は[6.3,6.9]s、POS_HOLD係合は行D開始(6.9s)、G3数値ゲート窓は
# [7.6,21.6]s（係合後0.7sの整定余裕）、台本DISARMエッジは約21.3s。
S1_T_ARM = 4.0
S1_T_DISTURB_START = 6.3
S1_T_POSHOLD_ENGAGE = 6.9
S1_T_DISARM = 21.3
S1_DRIFT_GATE_M = 3.0


def build_s1(bundle: Path, copy_video: bool = True) -> None:
    df = load_traj(bundle)
    t = df["t"].values

    # Hold-target position: the POS_HOLD engage-time (px,py) — same convention
    # _traj_metric()'s horizontal_drift_max uses (first row of the [7.6,21.6]
    # window). We use the exact engage sample so the error curve starts at 0.
    # 保持目標: POS_HOLD係合時刻の(px,py) — _traj_metric() の
    # horizontal_drift_max と同じ規約（[7.6,21.6]窓の先頭行）。エラー曲線が
    # 0から始まるよう係合時刻の実サンプルを使う。
    i_engage = int(np.argmin(np.abs(t - S1_T_POSHOLD_ENGAGE)))
    cx, cy = df["px"].values[i_engage], df["py"].values[i_engage]
    drift = np.hypot(df["px"].values - cx, df["py"].values - cy)
    drift_max_hold = drift[t >= S1_T_POSHOLD_ENGAGE].max()

    # --- (a) XY trajectory + horizontal position-error time series ----------
    fig, (ax_xy, ax_err) = plt.subplots(1, 2, figsize=(13, 5.5))

    pre = t < S1_T_POSHOLD_ENGAGE
    post = ~pre
    ax_xy.plot(df["py"].values[pre], df["px"].values[pre], "--", color="#999999",
               lw=1.6, label="STABILIZE (roll disturbance)")
    ax_xy.plot(df["py"].values[post], df["px"].values[post], "-", color="#1f77b4",
               lw=2.0, label="POS_HOLD")
    ax_xy.scatter([df["py"].values[0]], [df["px"].values[0]], marker="o", s=70,
                  color="black", zorder=5, label="Start")
    ax_xy.scatter([cy], [cx], marker="s", s=90, color="#d62728", zorder=5,
                  label="POS_HOLD engage / hold target")
    ax_xy.scatter([df["py"].values[-1]], [df["px"].values[-1]], marker="*", s=160,
                  color="#2ca02c", zorder=5, label="End (disarmed)")
    ax_xy.set_xlabel("East  py  [m]")
    ax_xy.set_ylabel("North  px  [m]")
    # One-line title: scenario name is already in the slide caption
    # (sci_appendix.tex) — a second title line at the bigger 16pt font
    # crowded the legend box below it.
    # タイトルは1行: シナリオ名はスライドのキャプション（sci_appendix.tex）
    # 側にある — 16ptフォントで2行にすると下の凡例と接近しすぎていた。
    ax_xy.set_title("S1: POS_HOLD horizontal trajectory (top-down)")
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend(loc="best", framealpha=0.9)

    ax_err.plot(t, drift, color="#1f77b4", lw=1.8, label="Horizontal error from hold target")
    # "Gate" is avoided here per project terminology rules (insider jargon for
    # a lay/projector audience) — Japanese "判定条件" (pass criterion) when a
    # CJK font is installed, else the English label.
    # 「ゲート」は内輪語のため使わない（投影・一般向け資料）。CJKフォントが
    # あれば日本語ラベル「判定条件」、無ければ英語ラベルを使う。
    _s1_gate_label = (f"判定条件: drift < {S1_DRIFT_GATE_M:.1f} m" if _CJK_FONT
                       else f"Pass criterion: drift < {S1_DRIFT_GATE_M:.1f} m")
    ax_err.axhline(S1_DRIFT_GATE_M, color="#d62728", ls="--", lw=1.4,
                    label=_s1_gate_label)
    ax_err.axvline(S1_T_DISTURB_START, color="#999999", ls=":", lw=1.2)
    ax_err.axvline(S1_T_POSHOLD_ENGAGE, color="#d62728", ls=":", lw=1.2)
    ax_err.axvline(S1_T_DISARM, color="#888888", ls=":", lw=1.2)
    _label_box = dict(boxstyle="round,pad=0.2", facecolor="white", edgecolor="none", alpha=0.85)
    ax_err.text(S1_T_DISTURB_START, ax_err.get_ylim()[1] * 0.55, " roll\n disturbance",
                fontsize=10, color="#666666", va="center", bbox=_label_box)
    # y-fraction lowered from 0.9 to 0.68: at the bigger 13pt legend font the
    # "upper right" legend box's left edge reaches back to ~x=7.6s (close to
    # this label's x=6.9s anchor); placing the label below the legend's
    # bottom edge, instead of relying on horizontal separation alone, avoids
    # the overlap regardless of legend width (T2).
    # yの比率を0.9→0.68に変更: 13pt凡例フォントでは「upper right」凡例の
    # 左端がx≈7.6sまで達し、このラベルのx=6.9sアンカーに接近する。水平方向の
    # 分離だけに頼らず凡例の下端より下に配置することで、凡例の幅に関わらず
    # 重なりを避ける（T2）。
    ax_err.text(S1_T_POSHOLD_ENGAGE, ax_err.get_ylim()[1] * 0.68, "POS_HOLD\nengage",
                fontsize=10, color="#d62728", va="top", bbox=_label_box)
    ax_err.set_xlabel("Time  t  [s]")
    ax_err.set_ylabel("Horizontal error [m]")
    # One-line title (same rationale as ax_xy above): drop the "vs time"
    # wording to keep it comfortably within one line at 16pt. "limit" (not
    # "pass criterion", which overflows the panel width at this font size) —
    # avoids the banned "gate" jargon while staying short.
    # 1行タイトル（ax_xyと同じ理由）: 16ptで1行に収まるよう「vs time」を省略。
    # "pass criterion" だとこのフォントサイズでパネル幅からはみ出すため
    # "limit" を使う — 内輪語「gate」は避けつつ短く保つ。
    ax_err.set_title(f"S1: position-hold error — max {drift_max_hold:.2f} m "
                      f"(limit {S1_DRIFT_GATE_M:.1f} m)")
    ax_err.grid(True, alpha=0.3)
    ax_err.legend(loc="upper right", framealpha=0.9)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "S1_pos_hold_xy.png")
    plt.close(fig)

    # --- (b) altitude + attitude time series ---------------------------------
    fig, (ax_alt, ax_att) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)

    ax_alt.plot(t, df["alt"].values, color="#1f77b4", lw=1.8, label="Altitude (truth)")
    ax_alt.plot(t, df["alt_est"].values, color="#ff7f0e", lw=1.2, ls="--",
                label="Altitude (estimate)")
    ax_alt.set_ylabel("Altitude  [m]")
    ax_alt.set_title("S1: altitude and attitude — pos_roll.scn (SILS/MuJoCo, vehicle)")
    ax_alt.grid(True, alpha=0.3)
    ax_alt.legend(loc="best", framealpha=0.9)

    ax_att.plot(t, df["roll"].values, color="#1f77b4", lw=1.8, label="Roll [deg]")
    ax_att.plot(t, df["pitch"].values, color="#2ca02c", lw=1.5, label="Pitch [deg]")
    for ax in (ax_alt, ax_att):
        ax.axvline(S1_T_DISTURB_START, color="#999999", ls=":", lw=1.2)
        ax.axvline(S1_T_POSHOLD_ENGAGE, color="#d62728", ls=":", lw=1.2)
        ax.axvline(S1_T_DISARM, color="#888888", ls=":", lw=1.2)
    ax_att.set_xlabel("Time  t  [s]")
    ax_att.set_ylabel("Angle  [deg]")
    ax_att.grid(True, alpha=0.3)
    ax_att.legend(loc="best", framealpha=0.9)

    fig.tight_layout()
    fig.savefig(OUT_DIR / "S1_altitude_attitude.png")
    plt.close(fig)

    # --- (c) video (skipped in --plots-only mode, or if the bundle has none) ---
    src = bundle / f"scn_{S1_SCN}.mp4"
    if copy_video and src.exists():
        shutil.copyfile(src, OUT_DIR / "S1_pos_hold_flight.mp4")
    elif copy_video:
        print(f"S1: no video in bundle ({src}) — leaving S1_pos_hold_flight.mp4 as-is")

    print(f"S1: drift_max after engage = {drift_max_hold:.3f} m "
          f"(pass criterion < {S1_DRIFT_GATE_M} m)")


# =============================================================================
# S4 — Lesson 5 (P) vs Lesson 8 (PID) roll-step comparison
#      (workshop_acro_step.scn, target=workshop)
# =============================================================================
S4_SCN = "workshop_acro_step"
S4_RUN_CMDS = [
    "sf lesson switch 5 --solution",
    "sf sils build --target workshop",
    "sf sils scenario simulator/sils/scenarios/workshop_acro_step.scn --target workshop --video",
    "# copy the bundle aside before switching lessons — same bundle dir is reused",
    "sf lesson switch 8 --solution",
    "sf sils build --target workshop",
    "sf sils scenario simulator/sils/scenarios/workshop_acro_step.scn --target workshop --video",
    "sf lesson switch <original lesson>",
]
# Roll-step schedule from workshop_acro_step.scn's own event timeline
# (cumulative "+" hold_ms).
# workshop_acro_step.scn 自身のイベントタイムラインから算出。
S4_T_ARM = 4.0
S4_T_LIFTOFF_BURST = 4.5
S4_T_STEP_START = 5.9
S4_T_STEP_END = 6.4
S4_T_DISARM = 7.8

# rate_target = ws::rc_roll() * rate_max_rp; rc_roll() = deadband(normalizeAxis(raw))
# (firmware/vehicle/components/sf_command/command.cpp: kAdcCenter=kAdcHalfSpan=2048,
# deadband_=0.05); rate_max_rp = 1.0 rad/s (Lesson 5/8 solution.cpp). raw=2662 here.
# rate_target = ws::rc_roll() * rate_max_rp（command.cpp のデッドバンド式）、
# rate_max_rp=1.0 rad/s（Lesson 5/8 解答）。raw=2662。
_STICK_RAW = 2662
_V = (_STICK_RAW - 2048) / 2048.0
_DB = 0.05
_V_DB = 0.0 if abs(_V) < _DB else np.sign(_V) * (abs(_V) - _DB) / (1 - _DB)
S4_RATE_TARGET_DEG_S = np.degrees(_V_DB * 1.0)  # ~15.07 deg/s

# Panel/suptitle numbering words: the workshop deck now numbers these
# tutorial-local exercises 実習5/実習8 (sf lesson switch sci2026:5 / :8), not
# "Lesson 5/8" — kept in sync with the CJK-font check above (English fallback
# "Exercise N" when no CJK font is installed).
# パネル・総合タイトルの呼称: workshop の教材は現在これらを「実習5」「実習8」
# と呼ぶ（sf lesson switch sci2026:5 / :8）。「Lesson 5/8」ではない — 上の
# CJKフォント判定と連動（CJKフォントが無ければ英語 "Exercise N"）。
_S4_L5 = "実習 5" if _CJK_FONT else "Exercise 5"
_S4_L8 = "実習 8" if _CJK_FONT else "Exercise 8"


def s4_rate_ref(df: pd.DataFrame, t: np.ndarray) -> np.ndarray:
    """Commanded roll-rate step [deg/s]. Prefers the bundle's real `rate_ref`
    stream (rate_ref_roll, stashed in `df.attrs["rate_ref_roll_deg"]` by
    load_traj()) when present. Falls back to the scripted stick profile
    (stick value * rate_max) ONLY when `rate_ref` is absent — the workshop
    target's `WorkshopControlTask` never publishes vehicle's
    `sf::control_output` topic, so the bundle carries no `rate_ref` stream
    for it (see this script's module docstring / README; roll_rate_deg()'s
    docstring has the same fallback rule for the measured side).
    指令ロールレート・ステップ[deg/s]。バンドルの実際の `rate_ref` ストリーム
    （rate_ref_roll。load_traj() が `df.attrs["rate_ref_roll_deg"]` に格納）が
    あればそれを優先する。`rate_ref` が無い（workshop ターゲットの
    `WorkshopControlTask` は vehicle の `sf::control_output` トピックを
    発行しないためバンドルに `rate_ref` ストリームが無い -- 詳しくはこの
    ファイルの docstring / README 参照。実測側の同じフォールバック規則は
    roll_rate_deg() の docstring 参照）場合のみ、台本化したスティック値
    プロファイル（スティック値 × rate_max）にフォールバックする。
    """
    rate_ref_roll_deg = df.attrs.get("rate_ref_roll_deg")
    if rate_ref_roll_deg is not None:
        return rate_ref_roll_deg
    return np.where((t >= S4_T_STEP_START) & (t < S4_T_STEP_END),
                     S4_RATE_TARGET_DEG_S, 0.0)


def build_s4(bundle_l5: Path, bundle_l8: Path) -> None:
    """Builds S4_roll_step_p_vs_pid.png from two flight-log-bundle-bearing
    directories (each either a freshly rendered SILS bundle, or the persisted
    docs/events/sci_tutorial_2026/fallback/_s4_raw/lessonN/ snapshot). Does NOT touch the
    S4 videos — those are copied straight to their final names by run_s4()
    at render time, since only a fresh SILS bundle (not the persisted
    bundle-only snapshot) carries the rendered .mp4.
    S4_roll_step_p_vs_pid.png をフライトログ一式を持つ2ディレクトリから生成
    する（新規レンダリングの SILS バンドル、または永続化済み
    _s4_raw/lessonN/ のいずれか）。動画は扱わない — 動画は SILS バンドルに
    しかない（永続スナップショットには含めない）ため、run_s4() がレンダリング
    時に最終ファイル名へ直接コピーする。
    """
    df5, df8 = load_traj(bundle_l5), load_traj(bundle_l8)

    # Side-by-side (not stacked) layout: a wide, short canvas (~2.5:1) keeps
    # the >=14pt fonts legible after the appendix slide's height-constrained
    # scale-down, instead of the old 2-row stack (13x9in, near-square 1.44:1)
    # that forced a much larger shrink factor to fit the slide height.
    # 縦積みではなく左右並び: 横長 (~2.5:1) にすることで、付録スライドの
    # 高さ制約による縮小後も14pt以上のフォントが読める。旧版の2段積み
    # (13x9in, 1.44:1に近い正方形寄り) は縮小率が大きくなりすぎていた。
    fig, (ax5, ax8) = plt.subplots(1, 2, figsize=(14, 5.5))
    # Plot window ends at D3 (6.4-7.4s, sticks re-centred, clean settle) and
    # deliberately EXCLUDES the E/F/G release-ARM -> DISARM -> free-fall/ground-
    # contact tail (7.4-8.4s): differentiating the truth roll angle through a
    # ground impact produces a huge (~100s of deg/s) spike that swamps the
    # y-axis and hides the actual step response. The step-response story is
    # fully contained in [ARM+1.3, D3 end] — the tail carries no more signal.
    # プロット窓は行D3終端(7.4s)で止め、行E/F/G（ARM解放→DISARM→自由落下・接地、
    # 7.4-8.4s）を意図的に除外する: 接地の瞬間まで真値ロール角を微分すると
    # 巨大なスパイク（数百deg/s）が出て y 軸を支配し、本来見せたいステップ応答が
    # 埋もれる。ステップ応答の物語は [ARM+1.3s, D3終端] に収まっており、
    # 末尾区間に追加情報は無い。
    window = (S4_T_ARM + 1.3, S4_T_STEP_END + 1.0)

    # Short one-line titles: scenario name, gains and the derivative caveat
    # already live in the slide caption (sci_appendix.tex) — the in-figure
    # text only needs to say which lesson/gain each panel is.
    # タイトルは1行の短文: シナリオ名・ゲイン・微分の注記はスライドの
    # キャプション（sci_appendix.tex）側にあるため、図内は各パネルが
    # どのレッスン・ゲインかだけ示せば十分。
    metrics = {}
    for ax, df, title, legend_label, color in (
        (ax5, df5, f"{_S4_L5} (P, Kp=0.5)", "roll rate (P only)", "#1f77b4"),
        (ax8, df8, f"{_S4_L8} (PID)", "roll rate (PID)", "#d62728"),
    ):
        t_full = df["t"].values
        rate_full = roll_rate_deg(df)
        ref_full = s4_rate_ref(df, t_full)
        wmask = (t_full >= window[0]) & (t_full <= window[1])
        t, rate, ref = t_full[wmask], rate_full[wmask], ref_full[wmask]
        ax.plot(t, ref, color="black", ls="--", lw=1.6,
                label=f"rate_ref (target {S4_RATE_TARGET_DEG_S:.1f} deg/s)")
        ax.plot(t, rate, color=color, lw=1.8, label=legend_label)
        ax.axvline(S4_T_STEP_START, color="#999999", ls=":", lw=1.1)
        ax.axvline(S4_T_STEP_END, color="#999999", ls=":", lw=1.1)
        ax.set_xlim(*window)
        ax.set_xlabel("Time  t  [s]")
        ax.set_ylabel("Roll rate  [deg/s]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right", framealpha=0.9)

        mask_step = (t >= S4_T_STEP_START) & (t <= S4_T_STEP_END)
        peak = rate[mask_step].max()
        val_end = rate[int(np.argmin(np.abs(t - S4_T_STEP_END)))]
        mask_post = (t > S4_T_STEP_END) & (t <= S4_T_STEP_END + 0.3)
        undershoot = rate[mask_post].min()
        metrics[title] = (peak, val_end, undershoot)

    fig.suptitle(f"S4: roll rate-loop step response — {_S4_L5} (P) vs {_S4_L8} (PID)",
                 fontsize=16)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(OUT_DIR / "S4_roll_step_p_vs_pid.png")
    plt.close(fig)

    for label, (peak, val_end, undershoot) in metrics.items():
        print(f"S4 {label}: peak={peak:.2f} deg/s, val@step_end={val_end:.2f} deg/s, "
              f"post-step min={undershoot:.2f} deg/s "
              f"(target={S4_RATE_TARGET_DEG_S:.2f} deg/s)")


def run_s4(restore_lesson: str) -> tuple[Path, Path]:
    """Execute the S4 pipeline end to end (mutates the shared workshop lesson
    state; always restores it in a finally: block). Persists each lesson's
    flight-log bundle under docs/events/sci_tutorial_2026/fallback/_s4_raw/lessonN/ (small,
    committed — lets the fast/default path rebuild the PNG without a rebuild)
    and copies the rendered videos straight to their final deliverable names.
    S4 パイプラインをE2E実行する（workshop レッスン状態を変更するため、
    finally: で必ず復元する）。各レッスンのフライトログ一式を
    _s4_raw/lessonN/ に永続化（小さい・コミット対象 — 既定の速い経路が
    リビルド無しで PNG を再生成できるように）し、レンダリング済み動画は
    最終成果物名へ直接コピーする。
    """
    raw = OUT_DIR / "_s4_raw"
    l5_dir, l8_dir = raw / "lesson5", raw / "lesson8"
    l5_dir.mkdir(parents=True, exist_ok=True)
    l8_dir.mkdir(parents=True, exist_ok=True)
    try:
        sf("sf lesson switch 5 --solution")
        sf("sf sils build --target workshop")
        sf(f"sf sils scenario simulator/sils/scenarios/{S4_SCN}.scn --target workshop --video")
        b = bundle_dir(S4_SCN)
        _copy_bundle_zip(b, l5_dir)
        shutil.copyfile(b / f"scn_{S4_SCN}.mp4", OUT_DIR / "S4_ex5_p_flight.mp4")

        sf("sf lesson switch 8 --solution")
        sf("sf sils build --target workshop")
        sf(f"sf sils scenario simulator/sils/scenarios/{S4_SCN}.scn --target workshop --video")
        _copy_bundle_zip(b, l8_dir)
        shutil.copyfile(b / f"scn_{S4_SCN}.mp4", OUT_DIR / "S4_ex8_pid_flight.mp4")

        return l5_dir, l8_dir
    finally:
        sf(f"sf lesson switch {restore_lesson}")


# =============================================================================
# S5 — vehicle regression representative scenario (stab_flight.scn, vehicle)
# =============================================================================
S5_SCN = "stab_flight"
S5_RUN_CMD = (
    "sf sils scenario simulator/sils/scenarios/stab_flight.scn --target vehicle --video"
)
S5_REGRESSION_CMD = "sf sils regression"
# Step schedule from stab_flight.scn's own event timeline.
# stab_flight.scn 自身のイベントタイムラインから算出。
S5_T_ARM = 4.0
S5_T_CLIMB_START = 5.0
S5_T_STEP_D = 7.0   # roll-right step
S5_T_STEP_E = 8.5   # roll-left step
S5_T_STEP_F = 10.0  # pitch-forward step
S5_T_STEP_G = 11.5  # centre / self-level
S5_T_DISARM = 13.4


def build_s5_graph(bundle: Path, copy_video: bool = True) -> None:
    df = load_traj(bundle)
    t_full = df["t"].values

    # Plot window ends at the end of row G (self-level, 13.0s) and deliberately
    # EXCLUDES the release-ARM -> DISARM -> free-fall/ground-bounce tail
    # (13.0-14.0s): differentiating truth pitch through that bounce produces a
    # ~79 deg/s transient that swamps the y-axis and hides the intentional
    # +-8deg step responses (which peak at roughly 10-14 deg/s). Same rationale
    # as the S4 window — see that function's comment.
    # プロット窓は行G終端(13.0s)で止め、ARM解放→DISARM→自由落下・地面バウンドの
    # 尾部(13.0-14.0s)を意図的に除外する: そのバウンドを真値ピッチで微分すると
    # 約79deg/sの過渡が出て y 軸を支配し、本来の±8度ステップ応答（ピークは
    # 概ね10-14deg/s）が埋もれる。理由は S4 の窓と同じ（同関数のコメント参照）。
    window = (S5_T_ARM - 0.3, S5_T_STEP_G + 1.5)
    wmask = (t_full >= window[0]) & (t_full <= window[1])
    t = t_full[wmask]
    roll_rate_full, pitch_rate_full = roll_rate_deg(df), pitch_rate_deg(df)

    # Side-by-side (not stacked) layout: same rationale as build_s4 — a wide,
    # short canvas (~2.5:1) keeps >=14pt fonts legible after the appendix
    # slide's height-constrained scale-down. Scenario name / gate result live
    # in the slide caption, so the in-figure titles are one short word each.
    # 縦積みではなく左右並び: build_s4 と同じ理由。シナリオ名・ゲート結果は
    # スライドのキャプションにあるため、図内タイトルは短い1語ずつでよい。
    fig, (ax_att, ax_rate) = plt.subplots(1, 2, figsize=(14, 5.5))

    ax_att.plot(t, df["roll"].values[wmask], color="#1f77b4", lw=1.8, label="Roll [deg]")
    ax_att.plot(t, df["pitch"].values[wmask], color="#2ca02c", lw=1.6, label="Pitch [deg]")
    ax_att.set_xlabel("Time  t  [s]")
    ax_att.set_ylabel("Angle  [deg]")
    ax_att.set_title("Attitude")
    ax_att.grid(True, alpha=0.3)
    ax_att.legend(loc="best", framealpha=0.9)

    ax_rate.plot(t, roll_rate_full[wmask], color="#1f77b4", lw=1.4, label="Roll rate [deg/s]")
    ax_rate.plot(t, pitch_rate_full[wmask], color="#2ca02c", lw=1.2, label="Pitch rate [deg/s]")
    ax_rate.plot(t, np.degrees(df["yawrate"].values[wmask]), color="#9467bd", lw=1.0, ls="--",
                 label="Yaw rate [deg/s] (truth column)")
    ax_rate.set_xlabel("Time  t  [s]")
    ax_rate.set_ylabel("Angular rate  [deg/s]")
    ax_rate.set_title("Angular rate")
    ax_rate.set_xlim(*window)
    ax_rate.grid(True, alpha=0.3)
    # Fixed "upper left" placement (not "best"): the roll/pitch-rate transients
    # at the roll+8/roll-8/pitch+8 step edges reach +-45..70 deg/s and "best"
    # can still auto-place the legend box close enough to graze a peak. The
    # pre-climb region (t in [ARM-0.3, climb start]) is provably all-zero rate
    # (boot/idle, no attitude command yet), so anchoring there is always clear.
    # "best" ではなく固定配置「upper left」: ロール/ピッチのステップ切替点での
    # 角速度過渡は+-45〜70deg/sに達し、"best" でも凡例枠がピークに接近しうる。
    # 離陸前区間（t∈[ARM-0.3, 上昇開始]）は起動・アイドルでレートが確実にゼロ
    # なので、そこに固定すれば常に衝突しない。
    ax_rate.legend(loc="upper left", framealpha=0.9)

    _step_lines = ((S5_T_ARM, "ARM"), (S5_T_STEP_D, "roll+8"),
                   (S5_T_STEP_E, "roll-8"), (S5_T_STEP_F, "pitch+8"),
                   (S5_T_STEP_G, "centre"))
    for ax in (ax_att, ax_rate):
        ax.set_xlim(*window)
        for tt, _lab in _step_lines:
            ax.axvline(tt, color="#999999", ls=":", lw=1.0)
    # Labels only on the attitude panel, rotated and anchored near the top of
    # the axis (clear of the roll/pitch curves, which dip well below this
    # region during the steps) — avoids the bottom-anchored label touching
    # the roll-8 curve's trough that an earlier draft had.
    # ラベルは姿勢パネルのみ、回転させ軸上部に固定する（ステップ中に大きく
    # 沈むロール/ピッチ曲線から離れる）— 旧版で roll-8 の谷にラベルが
    # 接触していた問題を回避。
    att_top = ax_att.get_ylim()[1]
    for tt, lab in _step_lines:
        ax_att.text(tt, att_top * 0.97, lab, fontsize=10, color="#666666",
                     rotation=90, va="top", ha="right")

    fig.suptitle("S5: attitude and angular rate (STABILIZE)", fontsize=16)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(OUT_DIR / "S5_attitude_rate.png")
    plt.close(fig)

    # Skipped in --plots-only mode, or if the bundle has no rendered video.
    # --plots-only モード、またはバンドルに動画が無い場合はスキップする。
    src = bundle / f"scn_{S5_SCN}.mp4"
    if copy_video and src.exists():
        shutil.copyfile(src, OUT_DIR / "S5_stab_flight.mp4")
    elif copy_video:
        print(f"S5: no video in bundle ({src}) — leaving S5_stab_flight.mp4 as-is")


def build_s5_gate_text(bundle: Path) -> None:
    import json
    results = json.loads((bundle / "results.json").read_text(encoding="utf-8"))
    lines = [
        "S5 gate result — stab_flight.scn (target=vehicle)",
        f"Run: {S5_RUN_CMD}",
        "",
        f"verdict: {'PASS' if results['pass'] else 'FAIL'}  (exit_code={results['exit_code']}, "
        f"noise={results['noise']}, seed={results['seed']})",
        "",
        "checks:",
    ]
    for c in results["checks"]:
        status = "SKIP" if c.get("skipped") else ("PASS" if c["pass"] else "FAIL")
        lines.append(f"  [{status}] {c['name']}  ({c.get('detail', '')})")
    (OUT_DIR / "S5_gate_result.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"S5 gate: {'PASS' if results['pass'] else 'FAIL'}")


def build_s5_regression_text() -> None:
    """Writes the pinned regression summary. Only overwritten if the caller
    reruns `sf sils regression` themselves and pastes the new summary line in
    here — this function does not invoke it (34 scenarios, ~1-2 minutes; not
    worth paying on every `make_fallback.py` invocation).
    固定済みの回帰サマリを書く。`sf sils regression` はここでは実行しない
    （34本・約1-2分、毎回のスクリプト実行で払うほどではない）。呼び出し側が
    別途再実行して新しいサマリ行に更新した場合はこの関数を編集すること。
    """
    text = (
        "sf sils regression summary\n"
        f"Run: {S5_REGRESSION_CMD}\n"
        "\n"
        "SILS regression: 28 PASS + 5 KNOWN-FAIL + 1 SKIP (34 total) known-fail=5\n"
        "\n"
        "KNOWN-FAIL (tracked, not gating — docs/architecture/simulation-policy.md backlog):\n"
        "  calib                 — injected IMU bias x ODE-plant takeoff dynamics produce a\n"
        "                          0.6-0.7 Hz self-oscillation (backlog #4/#13)\n"
        "  commloss_land_level   — LANDING leveling gate roll convergence insufficient under\n"
        "                          the ODE motor plant (backlog #13)\n"
        "  pos_flight            — ODE plant yaw torque-authority saturation under combined\n"
        "                          maneuvers, NT-Kanazawa-class issue (backlog #12)\n"
        "  pos_yaw               — same root cause as pos_flight (backlog #12)\n"
        "  yaw_hold              — yaw-band degradation, same root cause (backlog #11/#12)\n"
        "\n"
        "SKIP: workshop_acro (target=workshop; needs `sf lesson switch N --solution` +\n"
        "      --include-workshop, skipped by default on a fresh checkout)\n"
    )
    (OUT_DIR / "S5_regression_summary.txt").write_text(text, encoding="utf-8")


# =============================================================================
# main
# =============================================================================
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", action="store_true",
                     help="re-run the S1 (pos_roll) and S5 (stab_flight) vehicle-target "
                          "SILS scenarios before building outputs (safe/idempotent)")
    ap.add_argument("--run-s4", action="store_true",
                     help="also re-run S4 (Lesson 5 vs Lesson 8 roll-step). Mutates "
                          "firmware/workshop/main/user_code.cpp via `sf lesson switch` — "
                          "restored afterward via --restore-lesson")
    ap.add_argument("--restore-lesson", default="0",
                     help="lesson id/number to restore after --run-s4 (default: 0, "
                          "= Lesson 0 student — check `sf lesson list` first if unsure)")
    ap.add_argument("--plots-only", action="store_true",
                     help="rebuild only the PNGs/text from already-persisted bundles "
                          "(.sflog.zip; e.g. after a label-only edit to this script) "
                          "without touching any of the four .mp4 videos, even if the "
                          "bundle happens to carry a freshly rendered one. Incompatible "
                          "with --run/--run-s4")
    args = ap.parse_args()

    if args.plots_only and (args.run or args.run_s4):
        raise SystemExit("--plots-only cannot be combined with --run/--run-s4")

    OUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.run:
        sf(S1_RUN_CMD)
        sf(S5_RUN_CMD)

    s1_bundle = require_bundle(S1_SCN, S1_RUN_CMD)
    build_s1(s1_bundle, copy_video=not args.plots_only)

    if args.run_s4:
        l5_dir, l8_dir = run_s4(args.restore_lesson)
        build_s4(l5_dir, l8_dir)
    else:
        # Fast path: reuse the flight-log bundle snapshots already staged by a
        # prior --run-s4 (or hand-copied) under
        # docs/events/sci_tutorial_2026/fallback/_s4_raw/{lesson5,lesson8}. The S4 videos
        # themselves are NOT re-derived here — they were copied to their final
        # names the last time --run-s4 ran and are left untouched.
        # 速い経路: 事前の --run-s4（または手動コピー）で
        # docs/events/sci_tutorial_2026/fallback/_s4_raw/{lesson5,lesson8} に置かれた
        # フライトログ一式のスナップショットを再利用する。S4 の動画自体はここでは
        # 再生成しない — 前回 --run-s4 実行時に最終ファイル名へコピー済みで、
        # そのまま変更しない。
        raw = OUT_DIR / "_s4_raw"
        l5_dir, l8_dir = raw / "lesson5", raw / "lesson8"
        if _find_bundle_zip(l5_dir) is None:
            raise SystemExit(
                f"missing S4 raw data under {raw} — pass --run-s4 to generate it, "
                f"or see README.md's S4 command sequence."
            )
        build_s4(l5_dir, l8_dir)

    s5_bundle = require_bundle(S5_SCN, S5_RUN_CMD)
    build_s5_graph(s5_bundle, copy_video=not args.plots_only)
    build_s5_gate_text(s5_bundle)
    build_s5_regression_text()

    print("\nDone. Outputs in", OUT_DIR)
    return 0


if __name__ == "__main__":
    sys.exit(main())
