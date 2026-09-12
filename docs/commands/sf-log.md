# sf log

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

テレメトリログの取得・検査・変換・解析・可視化を行います。一次記録（測定値をそのまま書いた、最初の記録）は
**StampFly フライトログ v1 一式**（`.sflog.zip`。信号ごとの CSV と、取得条件を記した
`meta.json`・列定義を記した `schema.json` を 1 個の zip にまとめたもの。以下「一式」と呼ぶ）です。
取得経路は WiFi の UDP（User Datagram Protocol：軽量な通信方式）のみで、USB シリアル経由の取得
（旧 `sf log capture`）は `vehicle_old` 専用機能として廃止済みです。

整列表（複数の信号を共通の時刻に揃えて 1 枚の表にしたもの）や JSONL（JSON Lines：1 行に 1 件の
JSON レコードを並べたテキスト形式）は、一式から必要なときに `sf log convert` で作る**派生物**であり、
一次記録として書き出されることはありません。

読み込み系の全サブコマンドは `.sflog.zip` ファイルと、展開済みのフォルダ一式のどちらも受け付け、
省略時は `logs/` 内の最新の一式を使います。

## 2. サブコマンド一覧

| サブコマンド | 説明 |
|-------------|------|
| `wifi` | WiFi UDP でテレメトリを取得し、一式として保存 |
| `list` | `logs/` 配下の一式を一覧表示 |
| `info` | 一式の `meta.json` 要約とストリームごとの統計を表示 |
| `check` | 一式の構造・単位・時刻整合性を検査 |
| `convert` | 旧 `.jsonl` → 一式、または 一式 → 派生の整列/JSONL CSV |
| `viz` | 一式を可視化（各信号を原レートのまま描画） |
| `analyze` | 一式を解析（ジャイロ統計・PSD・チューニング所見。`--health` でモータ健全性診断） |

## 3. sf log wifi

WiFi 経由（UDP、WebSocket ではない）でフルレートのテレメトリを取得し、一式として保存します。
StampFly の電源を入れて WiFi AP に接続するだけでよく、USB での事前設定は不要です。

```bash
sf log wifi -d 30
```

```bash
sf log wifi -d 60
```

```bash
sf log wifi -o flight_test.sflog.zip
```

```bash
sf log wifi -o logs/session01/
```

```bash
sf log wifi --no-save
```

```bash
sf log wifi -i 192.168.10.5 --port 8890
```

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `-o, --output` | 出力先。拡張子を省いた名前（例 `-o flight1`）には `.sflog.zip` が自動で付く。`.sflog.zip` で終わるパスはその zip に書く。既存のディレクトリ（またはパス区切り文字で終わるパス）を渡すと、その中に同名の一式をフォルダとして書く。`.csv`/`.jsonl`/`.bin` など他の拡張子は拒否される（一式が唯一の取得形式。派生ファイルは取得後に `sf log convert --aligned`/`--jsonl` で作る） | 自動生成 `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| `-d, --duration` | キャプチャ時間（秒） | 30 |
| `-i, --ip` | StampFly の IP アドレス | 192.168.10.1 |
| `--port` | UDP テレメトリのポート番号 | 8890 |
| `--no-save` | ファイルに保存せず統計のみ表示 | - |

### 取得手順

1. StampFly の電源を入れる（バッテリー駆動）
2. PC を StampFly の WiFi AP（既定 IP `192.168.10.1`）に接続する
3. `sf log wifi -d 30` を実行する

保存した直後に `sf log check`（後述）が自動実行され、電文の解析エラーや構造の異常があれば
その場でエラーとして報告します（壊れた取得結果を解析に持ち込まないための確認です）。

## 4. sf log list

`logs/` 配下の一式（`*.sflog.zip` ファイルおよびフォルダ形式の一式）を、更新日時の新しい順に
一覧表示します。

```bash
sf log list
```

```bash
sf log list -n 50
```

```bash
sf log list --all
```

表には名前・サイズ・取得元（`vehicle`/`sils`/`sim`）・作成日時・収録時間・ストリーム数が並びます。

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `-n, --limit` | 表示する直近ファイル数 | 20 |
| `--all` | 全件表示（`--limit` を無視） | - |

## 5. sf log info

一式の `meta.json` 要約（取得元・作成日時・取得条件）と、ストリームごとの統計（行数・公称レート・
実測レート・最初/最後の時刻・収録時間）を表示します。

```bash
sf log info
```

```bash
sf log info logs/flight_20260911T121243.sflog.zip
```

### オプション

| 引数 | 説明 | 既定値 |
|------|------|-------|
| `bundle`（位置引数、省略可） | 一式のパス（`.sflog.zip` またはフォルダ） | `logs/` 内の最新の一式 |

## 6. sf log check

一式の構造・単位・時刻整合性を検査します（実体は `lib/sflog` の `check_bundle`）。列の欠落・
単位の妥当範囲外れ・時刻の非単調・重複時刻の件数・`seq`（制御周期の通し番号）の飛び（パケット
欠落の疑い）を warning/error として報告します。`sf log wifi` の保存直後には自動で実行されますが、
単独でも呼び出せます。

```bash
sf log check
```

```bash
sf log check logs/flight_20260911T121243.sflog.zip
```

### オプション

| 引数 | 説明 | 既定値 |
|------|------|-------|
| `bundle`（位置引数、省略可） | 一式のパス（`.sflog.zip` またはフォルダ） | `logs/` 内の最新の一式 |

error が 1 件でもあれば終了コード 1 を返します。

## 7. sf log convert

旧形式の `.jsonl` ログを一式へ変換するか、一式から派生ファイル（整列 CSV または JSONL）を
作ります。**一式そのものは既に一次記録なので、一式を一式へ変換することはできません**（`--aligned`
または `--jsonl` のどちらかを必ず指定します）。

```bash
sf log convert logs/stampfly_udp_20260908T121243.jsonl
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --aligned
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --aligned --base imu --method nearest
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --jsonl
```

### オプション

| 引数/オプション | 説明 | 既定値 |
|----------------|------|-------|
| `input`（位置引数） | 旧 `.jsonl` ログ、または一式（`.sflog.zip`/フォルダ） | 必須 |
| `-o, --output` | 出力パス | 入力名から自動生成 |
| `--aligned` | 一式 → 派生の `<名前>_aligned<レート>.csv`（1 行 = `--base` の 1 サンプル。他のストリームは前方値保持または最近傍で結合） | - |
| `--base` | `--aligned` 時の基準ストリーム（この時刻列が整列表の行になる） | `imu` |
| `--method` | `--aligned` 時の結合方法（`hold`=前方値保持、`nearest`=最近傍） | `hold` |
| `--jsonl` | 一式 → 派生の旧形式 `.jsonl`（`lib/sflog` 未対応の `analysis/scripts/` 向け） | - |

`--aligned` の出力には、どのストリーム由来かを示す `<ストリーム名>_timestamp_us` 列が付き、
同名 `.meta.json` に `derived: true`（派生物であること）が記録されます。整列そのものは
`sf sysid fit` などの読み込み側がメモリ上で行うため、ファイル化は「Excel で 1 枚の表として見たい」
「外部ツールへ渡したい」ときだけ使います。

## 8. sf log viz

一式を可視化します。全ての信号をそれぞれの原レート（送信元パケットが本来持っていたサンプリング
周期）のまま、1 つの描画処理で表示します。

```bash
sf log viz
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode attitude
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode sensors
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode position
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode eskf
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --save overview.png
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --time-range 5 15
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip -i
```

パネルの一覧・各モードの内訳は `docs/guides/flight-log-viz.md` を参照してください。

### オプション

| オプション | 説明 | 既定値 |
|-----------|------|-------|
| `--mode` | パネル群（`all`/`attitude`/`sensors`/`position`/`eskf`） | `all` |
| `--save FILE` | 画面表示せずファイルへ保存（`-i` 併用時は PNG ではなくダッシュボード HTML） | - |
| `--time-range START END` | プロットする時間範囲（最初のサンプルからの秒数） | 全範囲 |
| `-i, --interactive` | インタラクティブモード（Plotly、ブラウザで開く） | - |

### 表示ウィンドウが使えない環境での動作

matplotlib の GUI バックエンド（Tk/Qt 等の画面描画の仕組み）が使えない環境では、ウィンドウ表示の
代わりに一式の隣へ `<一式の名前>.png` を自動保存し、OS 標準の画像ビューアで開きます。ウィンドウを
開けた場合は使用したバックエンド名を 1 行表示します。インタラクティブモード（`-i`、Plotly）は
ブラウザで開くためこのフォールバック（代替動作）の対象外です。

## 9. sf log analyze

フライトログを解析します。引数なしの通常解析と、`--health` によるモータ健全性診断の 2 系統が
あります。

### 通常解析

```bash
sf log analyze
```

```bash
sf log analyze logs/flight_20260911T121243.sflog.zip
```

`imu.csv` の**真の実測レート**（重複時刻を除いた、実際に届いた標本の頻度。一定レートに揃え直した
ものではない）でジャイロ統計・PSD（パワースペクトル密度：周波数ごとの振動の強さ）を計算し、
姿勢・位置・操縦入力の統計、ホバー区間の統計、5 秒区切りの安定性の表、チューニングの所見を出力
します。図は既定で `<一式の名前>_analysis.png` として一式の隣に自動保存されます。

### モータ健全性診断（`--health`）

```bash
sf log analyze --health
```

```bash
sf log analyze --health --batch
```

```bash
sf log analyze --health --batch "logs/flight_202609*.sflog.zip"
```

```bash
sf log analyze --health --batch --json
```

対応するのは一式のみ。バックエンドは `tools/log_analyzer/motor_health.py`。duty（モータの
駆動デューティ比）は `motor.csv`（無ければ `ctrl_ref.csv`）から、ジャイロは `imu.csv` の
`gyro_z` から、電圧は `status.csv` から読みます。

| オプション | 説明 |
|-----------|------|
| `--save FILE` | 通常解析の図の保存先（既定: `<一式の名前>_analysis.png`） |
| `--health` | ホバートリムから劣化ロータを検出するモータ健全性レポート |
| `--batch` | `--health` と併用。`logs/` 内の直近 12 件の一式（または `bundle` にグロブを渡した場合はそれに一致する全件）を横断して重心オフセットを除去した隅特定を行う |
| `--json` | `--health` と併用。機械可読な JSON 判定結果を出力する |

**診断ロジックの要点:**
- ヨートリム `ur = (M1+M3) - (M2+M4)` は機体重心のオフセットに依存せず、弱い回転方向グループ
  （CW/CCW）を確定できる。
- どの隅（M1〜M4）が弱いかは、1 本のホバーログだけでは重心オフセットと絡み合ってしまうため、
  `--batch` で重症度の異なる複数の一式を使い、`corr(ur, up)`／`corr(ur, uq)`（ロール・ピッチ
  トリムとの相関）のスケーリングで分離する。
- `--batch` は重心が一定（同一機体）であることを前提とするため、既定では最新 12 件に限定する。
  別機体のログが混在する環境では、対象セッションをグロブで明示する。
- 最終確認にはベンチでの入れ替え試験（疑わしい隅と対角のモータを入れ替えて再計測）を推奨する。

## 10. 典型的なワークフロー

```bash
sf log wifi -d 60
```

```bash
sf log viz
```

```bash
sf log analyze
```

モータ健全性を確認したい場合は、複数回のホバーログを取得したうえで:

```bash
sf log wifi -d 30
```

```bash
sf log wifi -d 30
```

```bash
sf log analyze --health --batch
```

---

<a id="english"></a>

## 1. Overview

Capture, check, convert, analyze, and visualize telemetry logs. The primary record (the first
record of raw measurements, with nothing filled in) is the **StampFly flight-log v1 bundle**
(`.sflog.zip` — one zip holding a CSV per signal, `meta.json` describing the capture, and
`schema.json` describing the columns; called "the bundle" below). The only capture path is WiFi
UDP (User Datagram Protocol, a lightweight network transport); USB-serial capture (the old
`sf log capture`) has been removed as a `vehicle_old`-only feature.

An aligned table (several signals resampled onto one shared time base) or JSONL (JSON Lines — a
text format with one JSON record per line) is a **derived product** built from the bundle on
demand with `sf log convert`; neither is ever written as the primary capture.

Every reader accepts a `.sflog.zip` file or an extracted bundle directory, and defaults to the
newest bundle in `logs/` when none is given.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `wifi` | Capture telemetry via WiFi UDP, saved as a bundle |
| `list` | List bundles under `logs/` |
| `info` | Show a bundle's `meta.json` summary and per-stream stats |
| `check` | Validate a bundle's structure, units, and timing |
| `convert` | Convert a legacy `.jsonl` to a bundle, or a bundle to a derived aligned/JSONL CSV |
| `viz` | Visualize a bundle (every stream plotted at its own native rate) |
| `analyze` | Analyze a bundle (gyro statistics/PSD, tuning insights; `--health` for motor-health diagnosis) |

## 3. sf log wifi

Captures full-rate telemetry over WiFi (UDP, not WebSocket) and saves it as a bundle. Just power
on StampFly and connect to its WiFi AP — no USB setup step is needed beforehand.

```bash
sf log wifi -d 30
```

```bash
sf log wifi -d 60
```

```bash
sf log wifi -o flight_test.sflog.zip
```

```bash
sf log wifi -o logs/session01/
```

```bash
sf log wifi --no-save
```

```bash
sf log wifi -i 192.168.10.5 --port 8890
```

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-o, --output` | Output path. A bare name gets `.sflog.zip` appended (`-o flight1` -> `flight1.sflog.zip`); a path ending in `.sflog.zip` writes that zip; an existing directory (or a path ending in a path separator) gets a same-named bundle written inside it as a directory. Other extensions (`.csv`/`.jsonl`/`.bin`) are rejected — the bundle is the only capture format; derive a file afterwards with `sf log convert --aligned`/`--jsonl` | Auto-generated `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| `-d, --duration` | Capture duration (seconds) | 30 |
| `-i, --ip` | StampFly IP address | 192.168.10.1 |
| `--port` | UDP telemetry port | 8890 |
| `--no-save` | Don't save to file, just display stats | - |

### Capture procedure

1. Power on StampFly (on battery)
2. Connect the PC to StampFly's WiFi AP (default IP `192.168.10.1`)
3. Run `sf log wifi -d 30`

Right after saving, `sf log check` (below) runs automatically and reports any wire-format parse
error or structural problem immediately — so a broken capture is caught before it reaches
analysis.

## 4. sf log list

Lists bundles under `logs/` (`*.sflog.zip` files and directory-form bundles), newest first.

```bash
sf log list
```

```bash
sf log list -n 50
```

```bash
sf log list --all
```

The table shows name, size, source (`vehicle`/`sils`/`sim`), creation time, duration, and stream
count.

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `-n, --limit` | Number of recent files to show | 20 |
| `--all` | Show all files (ignore `--limit`) | - |

## 5. sf log info

Shows a bundle's `meta.json` summary (source, creation time, capture conditions) and per-stream
statistics (row count, nominal/measured rate, first/last timestamp, duration).

```bash
sf log info
```

```bash
sf log info logs/flight_20260911T121243.sflog.zip
```

### Options

| Argument | Description | Default |
|----------|-------------|---------|
| `bundle` (positional, optional) | Bundle path (`.sflog.zip` or directory) | Newest bundle in `logs/` |

## 6. sf log check

Validates a bundle's structure, units, and timing (backed by `lib/sflog`'s `check_bundle`).
Reports missing columns, out-of-range units, non-monotonic timestamps, duplicate-timestamp
counts, and gaps in `seq` (the control-cycle sequence number — a suspected dropped packet) as
warnings/errors. Runs automatically right after `sf log wifi` saves, and can also be run on its
own.

```bash
sf log check
```

```bash
sf log check logs/flight_20260911T121243.sflog.zip
```

### Options

| Argument | Description | Default |
|----------|-------------|---------|
| `bundle` (positional, optional) | Bundle path (`.sflog.zip` or directory) | Newest bundle in `logs/` |

Returns exit code 1 when at least one error is found.

## 7. sf log convert

Converts a legacy `.jsonl` log into a bundle, or builds a derived file (an aligned CSV or JSONL)
from a bundle. **A bundle is already a primary record, so a bundle cannot be converted into
another bundle** — `--aligned` or `--jsonl` must be given.

```bash
sf log convert logs/stampfly_udp_20260908T121243.jsonl
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --aligned
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --aligned --base imu --method nearest
```

```bash
sf log convert logs/flight_20260911T121243.sflog.zip --jsonl
```

### Options

| Argument/Option | Description | Default |
|-----------------|-------------|---------|
| `input` (positional) | A legacy `.jsonl` log, or a bundle (`.sflog.zip`/directory) | Required |
| `-o, --output` | Output path | Derived from the input name |
| `--aligned` | Bundle -> a derived `<stem>_aligned<rate>.csv` (one row per `--base` sample; other streams held/nearest-matched) | - |
| `--base` | With `--aligned`: base stream whose timestamps become the aligned table's rows | `imu` |
| `--method` | With `--aligned`: match method (`hold`=forward-fill, `nearest`) | `hold` |
| `--jsonl` | Bundle -> a derived legacy-format `.jsonl` (for `analysis/scripts/` not yet on `lib/sflog`) | - |

The `--aligned` output carries a `<stream>_timestamp_us` column per merged stream to show its
provenance, and a same-named `.meta.json` marks it `derived: true`. The alignment itself is also
done in memory by readers such as `sf sysid fit`, so writing the file is only needed to view one
table in Excel or hand it to an external tool.

## 8. sf log viz

Visualizes a bundle: every signal plotted at its own native rate (the sampling period the
originating packet actually had), through one renderer.

```bash
sf log viz
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode attitude
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode sensors
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode position
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --mode eskf
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --save overview.png
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip --time-range 5 15
```

```bash
sf log viz logs/flight_20260911T121243.sflog.zip -i
```

See `docs/guides/flight-log-viz.md` for the panel list and what each mode shows.

### Options

| Option | Description | Default |
|--------|-------------|---------|
| `--mode` | Panel group (`all`/`attitude`/`sensors`/`position`/`eskf`) | `all` |
| `--save FILE` | Save instead of opening a window (with `-i`: dashboard HTML instead of PNG) | - |
| `--time-range START END` | Time range to plot (seconds from the first sample) | Full range |
| `-i, --interactive` | Interactive mode (Plotly dashboard, opens in the browser) | - |

### Behavior without a display window

When no matplotlib GUI backend (Tk/Qt, etc.) is usable, the plot is saved as
`<bundle name>.png` next to the bundle instead of opening a window, then opened with the OS's
default image viewer. When a window does open, the backend name used is printed on one line.
Interactive mode (`-i`, Plotly) opens in a browser and is unaffected by this fallback.

## 9. sf log analyze

Analyzes a flight log. There are two paths: plain analysis, and the `--health` motor-health
diagnosis.

### Plain analysis

```bash
sf log analyze
```

```bash
sf log analyze logs/flight_20260911T121243.sflog.zip
```

Computes gyro statistics and PSD (power spectral density — vibration strength per frequency)
from `imu.csv` at its **true measured rate** (duplicate timestamps removed, never resampled to a
uniform rate), plus attitude/position/pilot statistics, hover-window statistics, a 5-second
segment table, and tuning observations. A figure is saved by default as
`<bundle name>_analysis.png` next to the bundle.

### Motor health diagnosis (`--health`)

```bash
sf log analyze --health
```

```bash
sf log analyze --health --batch
```

```bash
sf log analyze --health --batch "logs/flight_202609*.sflog.zip"
```

```bash
sf log analyze --health --batch --json
```

Only bundles are supported. Backend: `tools/log_analyzer/motor_health.py`. Duty (the motor drive
duty cycle) comes from `motor.csv` (else `ctrl_ref.csv`), gyro from `imu.csv`'s `gyro_z`, and
voltage from `status.csv`.

| Option | Description |
|--------|-------------|
| `--save FILE` | Where to save the plain-analysis figure (default: `<bundle name>_analysis.png`) |
| `--health` | Motor health report: detect a degraded rotor from the hover trim |
| `--batch` | With `--health`: cross-bundle corner test with CG (center of gravity) removed, over the 12 most-recent bundles in `logs/` (or every bundle matching a glob passed as `bundle`) |
| `--json` | With `--health`: emit a machine-readable JSON verdict |

**How the diagnosis works:**
- The yaw trim `ur = (M1+M3) - (M2+M4)` is independent of CG offset, and pins down the weaker
  spin-direction group (CW/CCW).
- Identifying which corner (M1-M4) is weak is confounded with CG offset from a single hover
  bundle alone, so `--batch` uses several bundles of differing severity and separates it via the
  scaling of `corr(ur, up)` / `corr(ur, uq)` (correlation with roll/pitch trim).
- `--batch` assumes one airframe with constant CG, so it defaults to the 12 most-recent bundles;
  scope explicitly with a glob when other airframes' logs are mixed in.
- A bench swap test (suspected corner motor swapped with its diagonal) is recommended for final
  confirmation.

## 10. Typical workflow

```bash
sf log wifi -d 60
```

```bash
sf log viz
```

```bash
sf log analyze
```

To check motor health, capture a few hover bundles first:

```bash
sf log wifi -d 30
```

```bash
sf log wifi -d 30
```

```bash
sf log analyze --health --batch
```
