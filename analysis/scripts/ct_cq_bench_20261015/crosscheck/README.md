# 実機ログとの照合スクリプト / Cross-check scripts against flight logs

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

`analysis/notes/ct_cq_bench_20261015.md` の 6 章（実機ログとの照合）の数値を出したスクリプトと結果。**入力のログはこのリポジトリに無い**（実機の飛行記録は別管理）。ログは `<リポジトリのルート>/logs/` に置く（環境変数 `SF_LOGS` で場所を変えられる。リポジトリのルートは `SF_REPO` で変えられる）。R_out の表は `../rout/rout_table_c01.csv` を読む。実行は `python3 -I <script>`（`tools/` と `lib/` を参照するため、リポジトリのルートが分かる配置で）。

### 入力ログ（ファイル名のみ。リポジトリには含まれない）

| 種類 | ファイル名 |
|---|---|
| 飛行ログ 14 本（UDP テレメトリの JSON Lines） | `logs/stampfly_udp_<日時>.jsonl`（日時 = 20260614T201629, 20260614T214537, 20260627T020137, 20260627T164611, 20260627T165713, 20260629T145526, 20260717T211648, 20260717T223206, 20260717T224508, 20260717T224906, 20260717T231940, 20260718T022929, 20260718T064222, 20260908T121243） |
| 宙返り 2 本（400 Hz の Duty 付き） | `logs/flip_test1.sflog.zip`, `logs/flip_test2.sflog.zip` |
| 地上でモータを回したログ 2 本 | `logs/motor_sweep_bench.sflog.zip`, `logs/ground_spin_stab.sflog.zip` |

### スクリプトと対応する節

| スクリプト | 内容 | 出力（`results/`） | ノートの節 |
|---|---|---|---|
| `logcheck/common.py`, `data.py`, `hover.py`, `models.py` | 読み込み、ホバリング区間の抽出、R_out モデル・単純な平均モデル・現行 SILS の静的モデル（ライブラリ） | — | 全般 |
| `logcheck/explore.py`, `ex2.py`, `sanity.py` | ログの構成確認、時刻の重複や欠落の健全性検査 | `sanity.json` | 6 章の前提（使ったログ） |
| `logcheck/test1.py`（`test12.py` はその前身） | ホバリング Duty と電池電圧の関係（傾き、予測との比較） | `test1.json` | 6 章「ホバリング Duty と電池電圧」 |
| `logcheck/test2.py` | 電池電流の比（実測 ÷ 各モデル） | `test2.json` | 6 章「電池電流」の表 |
| `logcheck/test3.py` | 飛行ログでのレート同定（遅れの合計・ゲイン） | `test3.json` | 6 章「同定手順の較正」の末尾 |
| `logcheck/report.py` | 上の JSON から詳細報告（Markdown）を作る | `logcheck_report.md`（今回は同梱せず） | — |
| `flipcheck/flipcheck.py` | 宙返りの角加速度を各モデルで予測し、倍率 $s$・追加の遅れを当てはめる | `flipcheck_results.json` | 6 章「宙返り」の表 |
| `flip_overlay.py` | 宙返りの実測と予測の重ね描き用データ | `flip_overlay.json` | 6 章「宙返り」の図（図そのものは同梱せず） |
| `sysid_gate_small.scn`, `calfit.py` | SILS 側の同定手順の較正（ノイズの有無で時定数 16 ms / 38 ms の機体を同定）。`calfit.py <ラベル> <記録.sflog.zip> [リポジトリのルート]` | `sils_cal/V0/cal_*.txt`（現行）, `sils_cal/R1/cal_*.txt`（R_out） | 6 章「同定手順の較正」の表 |

`sysid_gate_small.scn` はシナリオ（`sf sils` で実行）。`results/sils_cal/` のテキストは、その記録に `calfit.py` を適用した出力。R_out モデルを入れた SILS は `../rout/R1.patch`。

### 同梱しなかったもの

`.sflog.zip`（SILS の記録、実機ログとも）、仮想環境、`__pycache__`、2 MB を超えるファイル（該当なし）、SILS の実行ログと中間 JSON、`logcheck_report.md` / `flipcheck_report.md`（スクリプトから再生成できる）、`hover_diag.py`、図の描画用スクリプト。

---

<a id="english"></a>

## 1. Overview

Scripts and results behind the numbers of chapter 6 (cross-check against flight logs) of `analysis/notes/ct_cq_bench_20261015.md`. **The input logs are not in this repository** (flight recordings are kept elsewhere). Put them in `<repository root>/logs/` (override with the environment variable `SF_LOGS`; the repository root can be overridden with `SF_REPO`). The R_out table is read from `../rout/rout_table_c01.csv`. Run with `python3 -I <script>`; the scripts use `tools/` and `lib/` of the repository, so keep this directory layout.

### Input logs (file names only; not in the repository)

| Kind | File names |
|---|---|
| 14 flight logs (UDP telemetry, JSON Lines) | `logs/stampfly_udp_<timestamp>.jsonl` (timestamps as listed in the Japanese section above) |
| 2 flips (400 Hz with Duty) | `logs/flip_test1.sflog.zip`, `logs/flip_test2.sflog.zip` |
| 2 ground motor-spin logs | `logs/motor_sweep_bench.sflog.zip`, `logs/ground_spin_stab.sflog.zip` |

### Scripts and the note sections they feed

| Script | Content | Output (`results/`) | Note section |
|---|---|---|---|
| `logcheck/common.py`, `data.py`, `hover.py`, `models.py` | Loading, hover-segment extraction, static models (R_out, simple average, current SILS); library code | none | general |
| `logcheck/explore.py`, `ex2.py`, `sanity.py` | Log structure check; sanity check for duplicated or missing timestamps | `sanity.json` | premise of chapter 6 (logs used) |
| `logcheck/test1.py` (`test12.py` is its predecessor) | Hover Duty vs battery voltage (slope, comparison with predictions) | `test1.json` | chapter 6, "Hover Duty and battery voltage" |
| `logcheck/test2.py` | Battery-current ratios (measured / each model) | `test2.json` | chapter 6, "Battery current" table |
| `logcheck/test3.py` | Rate identification on flight logs (total delay, gain) | `test3.json` | chapter 6, end of "Calibration of the identification procedure" |
| `logcheck/report.py` | Builds a detailed Markdown report from the JSON above | `logcheck_report.md` (not included) | none |
| `flipcheck/flipcheck.py` | Predicts flip angular acceleration with each model and fits the scale $s$ and extra delay | `flipcheck_results.json` | chapter 6, "Flips" table |
| `flip_overlay.py` | Data for the measured/predicted flip overlay | `flip_overlay.json` | chapter 6, "Flips" figure (figure itself not included) |
| `sysid_gate_small.scn`, `calfit.py` | Calibration of the identification procedure on SILS (identifies plants with time constants of 16 ms / 38 ms, with and without noise). `calfit.py <label> <recording.sflog.zip> [repo root]` | `sils_cal/V0/cal_*.txt` (current), `sils_cal/R1/cal_*.txt` (R_out) | chapter 6, "Calibration of the identification procedure" table |

`sysid_gate_small.scn` is a scenario (run with `sf sils`). The texts in `results/sils_cal/` are the output of `calfit.py` on those recordings. The SILS with the R_out model is `../rout/R1.patch`.

### Not included

`.sflog.zip` files (SILS recordings and flight logs alike), virtual environments, `__pycache__`, files over 2 MB (none), SILS run logs and intermediate JSON, `logcheck_report.md` / `flipcheck_report.md` (regenerable from the scripts), `hover_diag.py`, and the chart-drawing script.
