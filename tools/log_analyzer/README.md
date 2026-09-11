# StampFly ログ解析ツール

StampFlyの取得・可視化・ESKF（Error-State Kalman Filter：誤差状態カルマンフィルタ、姿勢・位置
推定に使う推定器）開発・検証・最適化のためのPythonツール群です。

> **重要:** ツールは全て **sf CLI** 経由で使用することを推奨します（3Dアニメーション2本は
> sf コマンド未統合のため直接実行）。一次記録（測定値をそのまま書いた、最初の記録）は
> **StampFlyフライトログv1一式**（`.sflog.zip`。信号ごとのCSVと`meta.json`/`schema.json`を
> zipにまとめたもの。以下「一式」）です。詳細は `docs/plans/flight-log-format-plan.md` と
> `docs/commands/sf-log.md` を参照してください。

## クイックスタート

開発環境のセットアップ:

```bash
source setup_env.sh
```

一式の一覧:

```bash
sf log list
```

WiFi経由で400Hzテレメトリをキャプチャする（一式として保存）:

```bash
sf log wifi -d 30
```

最新の一式を可視化する:

```bash
sf log viz
```

## sf log コマンド一覧

| コマンド | 説明 | 例 |
|---------|------|-----|
| `sf log list` | 一式の一覧 | `sf log list --all` |
| `sf log wifi` | WiFi経由400Hzキャプチャ（一式として保存） | `sf log wifi -d 60` |
| `sf log check` | 一式の構造・単位・時刻整合性を検査 | `sf log check` |
| `sf log convert` | 旧`.jsonl`→一式、一式→派生の整列/JSONL CSV | `sf log convert flight.sflog.zip --aligned` |
| `sf log info` | 一式の情報表示 | `sf log info` |
| `sf log analyze` | フライト解析 | `sf log analyze` |
| `sf log analyze --health` | モータ故障診断 | `sf log analyze --health --batch` |
| `sf log viz` | 一式の可視化 | `sf log viz --mode all` |

サブコマンドごとの全オプションは `docs/commands/sf-log.md` を参照してください。

## sf log analyze --health - モータ健全性レポート

ホバー飛行の一式から、劣化したロータ（同じdutyで推力・反トルクが低下したモータ）を定常ホバー
トリムで検出する。バックエンドは `motor_health.py`。duty（モータの駆動デューティ比）は
`motor.csv`（無ければ `ctrl_ref.csv`）、ジャイロは `imu.csv` の `gyro_z`、電圧は `status.csv`
から読む。

最新の一式1本で診断する（回転方向グループを確定、隅は傾向）:

```bash
sf log analyze --health
```

同一機体の複数の一式でクロスログ隅特定する（CG除去。既定は最新12件）:

```bash
sf log analyze --health --batch
```

セッション/機体をグロブで明示する（CG一定の前提を守るため推奨）:

```bash
sf log analyze --health --batch "logs/flight_202609*.sflog.zip"
```

AI/スクリプト連携用の機械可読 JSON:

```bash
sf log analyze --health --batch --json
```

- ヨートリム `ur=(M1+M3)-(M2+M4)` はCG（機体重心）非依存で、弱い回転方向グループ（CW/CCW）を
  確定する。
- 隅（M1〜M4）は単一ホバーだとCGオフセットと交絡するため、`--batch` で重症度の異なる複数の
  一式を使い、`corr(ur, up)` / `corr(ur, uq)` のスケーリングで分離する。
- 確定にはベンチでの入れ替え試験（疑い隅 ↔ 対角）を推奨。

## sf log viz - ログ可視化

全パネル表示（既定）:

```bash
sf log viz
```

モード指定（センサ生値のみ・姿勢のみ・位置速度のみ・ESKF推定値のみ）:

```bash
sf log viz flight.sflog.zip --mode sensors
```

```bash
sf log viz flight.sflog.zip --mode attitude
```

```bash
sf log viz flight.sflog.zip --mode position
```

```bash
sf log viz flight.sflog.zip --mode eskf
```

画像保存:

```bash
sf log viz flight.sflog.zip --save output.png
```

時間範囲指定:

```bash
sf log viz flight.sflog.zip --time-range 5 15
```

インタラクティブ表示（Plotly、ブラウザで開く）:

```bash
sf log viz flight.sflog.zip -i
```

パネルの一覧・モードごとの内訳は `docs/guides/flight-log-viz.md` を参照してください。

## 典型的なワークフロー

### 1. フライトログ取得と解析

StampFly WiFi APに接続してログをキャプチャする（一式として保存）:

```bash
sf log wifi -d 60
```

可視化する:

```bash
sf log viz
```

詳細解析する:

```bash
sf log analyze
```

### 2. 振動解析（PSD）

ジャイロのPSD（パワースペクトル密度：周波数ごとの振動の強さ）解析は `sf log analyze` の解析
処理に常時組み込まれており、別途フラグを指定する必要はない。`imu.csv` の真の実測レート（重複
時刻を除いた、実際に届いた標本の頻度）で計算する。

```bash
sf log wifi -d 30
```

```bash
sf log analyze
```

## ファイル一覧

> **注:** これらのスクリプトは sf CLI のバックエンド実装です。3Dアニメーション2本を除き、
> 直接実行せず sf CLI を使用してください。

| ファイル | sf コマンド | 説明 |
|---------|------------|------|
| `udp_capture.py` | `sf log wifi` | WiFi UDP テレメトリ取得。一式（`.sflog.zip`）の書き出しを担う |
| `visualize_stream.py` | `sf log viz` | 一式の描画処理。全ストリームをそれぞれの原レートで描く単一の描画処理 |
| `visualize_interactive.py` | `sf log viz -i` | インタラクティブダッシュボード（Plotly、信号選択・重ね描き対応の自己完結HTML） |
| `flight_analysis.py` | `sf log analyze` | フライト解析（ジャイロ統計・PSD・姿勢/位置/操縦入力統計・チューニング所見） |
| `motor_health.py` | `sf log analyze --health` | モータ健全性レポート（劣化ロータ検出） |
| `rate_sysid.py` | `sf sysid rate-fit` | レートループ同定＋PID自動チューニング（ETFEフィット＋Nelder-Mead） |
| `visualize_attitude_3d.py` | - （直接実行） | 姿勢の3Dアニメーション（3D姿勢軸＋オイラー角の時系列） |
| `visualize_pose_3d.py` | - （直接実行） | 位置＋姿勢の3Dアニメーション（3D軌跡/姿勢＋上面図） |
| `conftest.py` / `test_*.py` | - | pytest: 合成一式フィクスチャ・単体テスト |

**廃止済み（旧CSV/JSONL形式専用だったため一式への統一で削除）:** `visualize_extended.py`、
`visualize_telemetry.py`、`visualize_jsonl.py`、`sils_trajectory` 対応コード、
`reconstruct_duties.py`、サンプルPNG群。`log_capture.py`（USBバイナリ取得、`vehicle_old`専用）
は `tools/log_capture/` ごと削除済み。

## 必要なライブラリ

```bash
pip install numpy pandas matplotlib scipy pyyaml
```

## トラブルシューティング

### WiFi接続できない

環境診断:

```bash
sf doctor
```

WiFi接続確認（`sf log wifi` の既定IP）:

```bash
ping 192.168.10.1
```

### 一式が見つからない

`logs/` 内の一式一覧:

```bash
sf log list --all
```

検索ディレクトリは `logs/` のみです（一式は `*.sflog.zip` ファイルまたはフォルダ形式）。

### 可視化でエラー

一式の構造・単位・時刻整合性を確認する:

```bash
sf log check
```

必要なライブラリを確認する:

```bash
pip install matplotlib numpy pandas
```
