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

読み込み系の全サブコマンドは `.sflog.zip` ファイルと、展開済みのフォルダ一式のどちらも受け付け（判定は拡張子ではなく中身なので、`.sflog` や `.zip` に改名したファイルも読める）、
省略時は `logs/` 内の最新の一式を使います。バンドル名は拡張子を省略でき（例 `flight_20260912T093015`）、裸の名前は `logs/` 内も検索対象になるため、どのディレクトリからでも `sf log viz flight_20260912T093015` のように呼び出せます。

## 2. サブコマンド一覧

| サブコマンド | 説明 |
|-------------|------|
| `wifi` | WiFi UDP でテレメトリを取得し、一式として保存 |
| `list` | `logs/` 配下の一式を一覧表示 |
| `info` | 一式の `meta.json` 要約・ストリームごとの統計・診断値（ループ周期・有限でない数・ToF・ログ経路の欠落）を表示。`--text` で機体のテキストログ（`ESP_LOG` の行）を表示 |
| `stats` | 実機確認用の要約（床の ToF・ループのカウンタ・着陸のモータ停止・回頭・気圧高度・起動の理由）を表で表示 |
| `check` | 一式の構造・単位・時刻整合性を検査 |
| `convert` | 旧 `.jsonl` → 一式、または 一式 → 派生の整列/JSONL CSV |
| `viz` | 一式を可視化（各信号を原レートのまま描画） |
| `analyze` | 一式を解析（ジャイロ統計・PSD・チューニング所見。`--health` でモータ健全性診断） |
| `video` | ログを、3D の機体が記録どおりに飛ぶ mp4 にする（Chrome と ffmpeg が必要） |

## 3. sf log wifi

WiFi 経由（UDP）でフルレートのテレメトリを取得し、一式として保存します。
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
| `-o, --output` | 出力先。拡張子を省いた名前（例 `-o flight1`）には `.sflog.zip` が自動で付く。`.sflog.zip` で終わるパスはその zip に書く。既存のディレクトリ（またはパス区切り文字で終わるパス）を渡すと、その中に既定名の `.sflog.zip` を書く。`.csv`/`.jsonl`/`.bin` など他の拡張子は拒否される（一式が唯一の取得形式。派生ファイルは取得後に `sf log convert --aligned`/`--jsonl` で作る） | 自動生成 `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
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

### 数値のほかに取得するもの（診断値とテキストログ）

数値の 400Hz ストリームのほかに、次の 2 つが同じ取得（同じ UDP ポート）で一式に入ります。

| 内容 | 一式のストリーム | 読み方 |
|------|-----------------|--------|
| 1 Hz の診断値（ステータスパケット v4）: ToF の測定品質（range status・信号レート・環境光レート・σ）、2 つの 400Hz タスクのループ周期の数え上げ、有限でない数の数え上げ、ログ経路の欠落の計数 | `tof_diagnostics`・`loop_timing`・`nonfinite`・`link_health` | `sf log info`（最後の値を表示）。時系列は `sf log viz`・解析スクリプトで列を読む |
| 機体の `ESP_LOG` の行（USB コンソールに出るもの。状態遷移・「Landing detected (...)」・フェイルセーフ・拒否の理由）。機体時刻つき・順序どおり | `text_log` | `sf log info --text`（下記） |

取得の終わりに、`print_stats` が診断値の最後の値と、テキストログの行数・途中で失われたパケット数・機体で断った行数を表示します。
取得が始まる前に機体が出したログは届きません（取得は「今」から始まります）。旧ファーム（ステータスが v3 まで・テキストパケットなし）の取得では、これらのストリームは単に一式にありません。

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

```bash
sf log info --text
```

```bash
sf log info --text --grep "Landing detected"
```

```bash
sf log info --text --since 12.0 --until 18.5
```

診断値のストリーム（`loop_timing`・`nonfinite`・`tof_diagnostics`・`link_health`）や `text_log` を持つ一式では、ストリームの表の後に「Diagnostics」の欄を表示します（取得の最後の値。ループ周期は `late` が遅れた回数・`period_max` が起床間隔の最大・`busy_max` が処理時間の最大、`armed:` は ARM 中だけの値。読み方は `firmware/vehicle/docs/operation_manual.md`）。持たない一式（旧ファームの取得・診断値のないログ）には何も加えず、そのまま動きます。

### オプション

| 引数 | 説明 | 既定値 |
|------|------|-------|
| `bundle`（位置引数、省略可） | 一式のパス（`.sflog.zip` またはフォルダ） | `logs/` 内の最新の一式 |
| `--text` | 要約の代わりに、機体のテキストログを `[機体時刻 s] 本文` の形で 1 行ずつ表示する。機体が 1 行の上限で切り詰めた行は末尾に `[truncated]`、機体のログシンクが行を断った位置には `*** N lines dropped here ***` を出す（欠落は `--grep` で隠れない）。テキストログを持たない一式は警告を出して終了コード 0 | - |
| `--grep TEXT` | `--text` と併用。TEXT を含む行だけ（部分文字列。パターンではない） | - |
| `-I, --ignore-case` | `--text --grep` と併用。大文字小文字を区別しない | - |
| `--since SECONDS`、`--until SECONDS` | `--text` と併用。機体時刻（起動からの秒）が範囲内の行だけ（両端を含む） | - |

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
| `--aligned` | 一式 → 派生の `<名前>_aligned<レート>.csv`（1 行 = `--base` の 1 サンプル。他のストリームは前方値保持または最近傍で結合。解析用の信号でない `text_log` は表に入れない） | - |
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
| `--cols` | 一覧の列数（1〜4）。パネルを格子に並べ、時間軸は全パネルで共有 | 3 |
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

## 11. sf log video

ログを mp4 にします。左 70% に 3D の StampFly が記録どおりに飛ぶ様子、右 30% に数値・姿勢の 3 つの図（ロールは後ろから・ピッチは左から・ヨーは上から）・モータ Duty と電池電圧の棒グラフを描きます。位置と姿勢は機体の推定値（ESKF）です。床は推定高度 0 m に固定しており、推定高度がマイナスになると機体は床の下に描かれます（推定の異常に気づけるように、そのまま見せます）。Google Chrome（または Chromium / Edge）と ffmpeg が必要です（`sf doctor` で確認できます）。

```bash
sf log video logs/flight_20260911T121243.sflog.zip
```

見本（2026-06-27 の POS_HOLD のホバーの始めの 30 秒。1280×720 に縮めた版、2 MB）: [sf_log_video_sample.mp4](../assets/sf_log_video_sample.mp4)

<video src="../assets/sf_log_video_sample.mp4" controls width="640"></video>

| 項目 | 内容 |
|------|------|
| ログの指定 | `.sflog.zip`、一式のフォルダ、UDP の `.jsonl`。省略時は `logs/` の最新 |
| 出力先 | 既定は `logs/videos/<ログ名>_3d.mp4`（`--time-range S E` のときは `<ログ名>_3d_<S>-<E>s.mp4`）。既存のファイルは上書きせず `_2`, `_3` … を付ける。`--force` で上書き。`--out FILE` で名前を指定 |
| 範囲 | 既定は推力のある最長区間の前後 1 秒。`--time-range S E` は先頭サンプルからの秒 |
| 見た目 | `--view oblique/top/side`、`--tilt-gain`（傾きの強調、既定 3）、`--vehicle-scale`（機体の拡大、既定 2）、`--zoom`、`--size 1920x1080/1280x720` |
| その他 | `--speed`（再生速度）、`--fps`、`--frames-dir DIR`（mp4 の代わりに JPEG 連番）、`--swiftshader`（Chrome のソフトウェア描画） |

電池電圧は status の記録値（約 1 秒ごと）を次の記録まで保持して表示します。低電圧の目盛り 3.4 V はファームの既定値（`safety.battery.low_v`）で、ログには記録されません。

## 12. sf log stats

実機確認（`firmware/vehicle/docs/hardware_checkout.md`）の問いに、一式から答える表を出します。モードを 1 つ選び、その問いに必要なストリームだけを読みます。出すのは数値と事実だけで、合否の線は手順書の側にあります（この命令は合否を判定しません）。実体は `lib/sflog/stats.py` です。

```bash
sf log stats <モード> [一式] [--since 秒] [--until 秒]
```

`一式` を省くと `logs/` 内の最新の一式を使います。時刻は機体時刻（起動からの秒）で、`sf log info <一式> --text` の行頭の時刻と同じです。

| モード | 答える問い | 読むストリーム | 区間の指定 |
|--------|-----------|----------------|-----------|
| `tof` | ARMED_GROUND（ARM 済み・地上）の間、床の下向き ToF（機体底面の距離センサ）は何 cm を読んだか。床の帯（5 cm）を超えた有効な読みはあるか | `flight_phase`・`tof_bottom` | 任意 |
| `spread` | 有効な ToF の読みは、1 秒ごとにどれだけばらつくか（最大 − 最小） | `tof_bottom` | **必須** |
| `loop` | ARM 中の 400 Hz ループの `late`（遅れた回数）・`over budget`（処理が周期を超えた回数）はいつ増えたか。有限でない数の検出、ログ経路の欠落、テキストログの欠落、ToF 出力停止の行数 | `loop_timing`・`nonfinite`・`link_health`（任意: `text_log`・`tof_silence`） | 任意 |
| `events` | 最高到達の高さ、保持高度（`--target` ± `--band`）に最後に収まらなかった時刻、LANDING 開始から全モータが止まるまでの時間と、止まる前の最後の有効な ToF | `tof_bottom`・`flight_phase`（LANDING があれば `motor`） | 任意 |
| `turn` | 区間内の向きの変化（符号つき。時計回りが正）、ToF の高さの最小・最大、水平の流れ（区間の最初の位置からの最大の距離。推定器の値） | `attitude`・`tof_bottom`・`posvel` | **必須** |
| `baro` | 気圧から求めた床基準の高さと ToF の高さの差（rms: 二乗平均平方根と最大）、学習したオフセットの範囲と経過時間 | `baro_height`・`tof_bottom` | **必須** |
| `boot` | 起動の理由（`reset_reason`）と電池電圧の最小・最大 | `status` | 任意 |

出力の例（シミュレーション SILS の飛行の一式。値は例）:

```text
$ sf log stats tof logs/checkout/s1_tof_floor.sflog.zip
Floor ToF while ARMED_GROUND
  ARMED_GROUND time       : 40.2 s in 1 run(s), from 12.0 s
  ToF samples             : 1206 (valid 1200, invalid 6)
  valid minimum           : 3.1 cm
  valid maximum           : 3.5 cm
  valid above 5 cm        : 0

$ sf log stats events logs/checkout/s3_takeoff.sflog.zip
Flight events
  peak ToF height         : 0.816 m at 10.897 s
  last time outside 0.50 +- 0.07 m : 12.745 s (after the peak, before landing)
  LANDING                 : started at 13.022 s; all motors stopped 3.140 s later
  last valid ToF before stop: 3.8 cm, 0.080 s before the stop; invalid samples in the last 0.5 s: 3

$ sf log stats turn logs/checkout/s3_turn.sflog.zip --since 22 --until 36
Turn in the window
  yaw change              : -270.0 deg (min -270.0, max +0.0); positive = clockwise seen from above
  ToF height              : min 1.113 m, max 1.256 m
  horizontal drift        : max 0.000 m from the first position (ESKF estimate)

$ sf log stats loop logs/checkout/s4_final.sflog.zip
Armed 400 Hz loop (counters are totals since boot)
  no armed counter changed (late and over budget stayed 0 while ARMed)
  imu      armed: cycles 3456, late 0, over budget 0, period max 2.50 ms, busy max 0.00 ms
  control  armed: cycles 3456, late 0, over budget 0, period max 2.50 ms, busy max 0.00 ms
  non-finite              : detected 0 of 3456 armed cycles
  log paths               : log_stream_overflow=0, send_drops=0, entry_drops=0, text_dropped_lines=0
  text log                : 136 lines, 0 dropped before a line (loss markers)
  ToF output stopped      : tof_silent=1 in 0 of 1250 rows
```

`loop` の表は、ARM 中のカウンタ（`late`・`over budget`）が増えた行だけを時刻つきで並べます。カウンタは起動以降の累計です。

### オプション

| 引数 | 説明 | 既定値 |
|------|------|-------|
| `モード`（位置引数、必須） | `tof`・`spread`・`loop`・`events`・`turn`・`baro`・`boot` のどれか | - |
| `bundle`（位置引数、省略可） | 一式のパス（`.sflog.zip` またはフォルダ） | `logs/` 内の最新の一式 |
| `--since SECONDS`、`--until SECONDS` | 区間（機体時刻の秒、両端を含む）。`spread`・`turn`・`baro` は両方が必須 | 全区間 |
| `--target METERS` | `events` のみ。整定を判定する保持高度 | 0.5（自動離陸の高度） |
| `--band METERS` | `events` のみ。保持高度の前後で「整定」とみなす半幅 | 0.07（POS_HOLD の保持精度） |

### 終了コードと欠落したストリーム

- 終了コード 0: 表を出した。
- 終了コード 1: 一式が読めない、必要なストリームが無い、または区間に標本が無い。メッセージにストリーム名を書く（例: `stream 'loop_timing' is missing or empty in this bundle`）。旧ファームの取得や、診断値のストリームを持たない一式（`logs/` 内の過去の実機ログなど）では、`loop` などがこの形で終わる。`boot` は `status` を持つ全ての一式で動く。
- `events` で、ARM 中だけモータを記録する一式（SILS の一式）には、停止後の全 duty 0 の行が無い。その場合は「全 duty 0 の行が無い」と述べ、代わりに LANDING の状態が終わった時刻を出す。実機の一式は ARM 前後もモータの行を記録する。

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
newest bundle in `logs/` when none is given. A bundle name's extension may be omitted (e.g.
`flight_20260912T093015`), and a bare name is also looked up inside `logs/`, so
`sf log viz flight_20260912T093015` works from any directory.

## 2. Subcommands

| Subcommand | Description |
|------------|-------------|
| `wifi` | Capture telemetry via WiFi UDP, saved as a bundle |
| `list` | List bundles under `logs/` |
| `info` | Show a bundle's `meta.json` summary, per-stream stats and diagnostics (loop timing, non-finite counters, ToF, log-path losses); `--text` prints the vehicle's text log (the `ESP_LOG` lines) |
| `stats` | Checkout summaries as tables (floor ToF, loop counters, motor stop at landing, turn, barometric height, boot reason) |
| `check` | Validate a bundle's structure, units, and timing |
| `convert` | Convert a legacy `.jsonl` to a bundle, or a bundle to a derived aligned/JSONL CSV |
| `viz` | Visualize a bundle (every stream plotted at its own native rate) |
| `analyze` | Analyze a bundle (gyro statistics/PSD, tuning insights; `--health` for motor-health diagnosis) |
| `video` | Render a log as an mp4 in which a 3D vehicle flies the logged path (needs Chrome and ffmpeg) |

## 3. sf log wifi

Captures full-rate telemetry over WiFi (UDP) and saves it as a bundle. Just power
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
| `-o, --output` | Output path. A bare name gets `.sflog.zip` appended (`-o flight1` -> `flight1.sflog.zip`); a path ending in `.sflog.zip` writes that zip; an existing directory (or a path ending in a path separator) gets the default-named `.sflog.zip` written inside it. Other extensions (`.csv`/`.jsonl`/`.bin`) are rejected — the bundle is the only capture format; derive a file afterwards with `sf log convert --aligned`/`--jsonl` | Auto-generated `logs/flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
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

### What is captured besides the numbers (diagnostics and the text log)

Besides the numeric 400Hz streams, the same capture (the same UDP port) puts two more things into the bundle.

| Content | Bundle streams | How to read |
|---------|----------------|-------------|
| 1 Hz diagnostics (status packet v4): ToF measurement quality (range status, signal rate, ambient rate, sigma), the loop-timing counters of both 400Hz tasks, the non-finite counters, the loss counters of the log paths | `tof_diagnostics`, `loop_timing`, `nonfinite`, `link_health` | `sf log info` (prints the last values); read the columns of the time series with `sf log viz` or analysis scripts |
| The `ESP_LOG` lines of the vehicle (what the USB console shows: state transitions, "Landing detected (...)", failsafe, reasons for refusals), in order with the vehicle time | `text_log` | `sf log info --text` (below) |

At the end of the capture `print_stats` prints the last diagnostics and, for the text log, the number of lines, of packets lost on the way and of lines refused by the vehicle. Logs the vehicle emitted before the capture started do not arrive (a capture starts from "now"). A capture from older firmware (status up to v3, no text packet) simply lacks these streams.

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

```bash
sf log info --text
```

```bash
sf log info --text --grep "Landing detected"
```

```bash
sf log info --text --since 12.0 --until 18.5
```

For a bundle with the diagnostics streams (`loop_timing`, `nonfinite`, `tof_diagnostics`, `link_health`) or a `text_log`, a "Diagnostics" block follows the stream table (the last values of the capture; for the loop timing `late` is the number of late wakes, `period_max` the longest wake interval, `busy_max` the longest busy time, and `armed:` the values for armed cycles only; how to read them: `firmware/vehicle/docs/operation_manual.md`). A bundle without them (a capture from older firmware, a log without diagnostics) gets nothing added and works as before.

### Options

| Argument | Description | Default |
|----------|-------------|---------|
| `bundle` (positional, optional) | Bundle path (`.sflog.zip` or directory) | Newest bundle in `logs/` |
| `--text` | Print the vehicle's text log one line at a time as `[vehicle time s] text` instead of the summary. A line the vehicle cut at its line limit ends with `[truncated]`; where the vehicle's log sink refused lines, `*** N lines dropped here ***` is printed (a loss is never hidden by `--grep`). A bundle without a text log gets a warning and exit code 0 | - |
| `--grep TEXT` | With `--text`: only the lines containing TEXT (a plain substring, not a pattern) | - |
| `-I, --ignore-case` | With `--text --grep`: ignore case | - |
| `--since SECONDS`, `--until SECONDS` | With `--text`: only the lines whose vehicle time (seconds since boot) is in range (both ends included) | - |

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
| `--aligned` | Bundle -> a derived `<stem>_aligned<rate>.csv` (one row per `--base` sample; other streams held/nearest-matched; `text_log`, which is not an analysis signal, is left out of the table) | - |
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
| `--cols` | Number of panel columns in the overview grid (1-4); all panels share the time axis | 3 |
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

## 11. sf log video

Renders a log as an mp4: a 3D StampFly flies the logged path on the left 70 %, and the right 30 % shows numbers, three attitude views (roll seen from behind, pitch from the left, yaw from above) and bars for the motor duties and the battery voltage. Position and attitude are the vehicle's own estimate (ESKF). The floor is fixed at an estimated height of 0 m; when the estimated height goes negative, the vehicle is drawn below the floor (shown as is, so that an estimation fault is noticed). Needs Google Chrome (or Chromium / Edge) and ffmpeg (`sf doctor` checks both).

```bash
sf log video logs/flight_20260911T121243.sflog.zip
```

Sample (the first 30 s of a POS_HOLD hover on 2026-06-27, scaled down to 1280×720, 2 MB): [sf_log_video_sample.mp4](../assets/sf_log_video_sample.mp4)

<video src="../assets/sf_log_video_sample.mp4" controls width="640"></video>

| Item | Description |
|------|-------------|
| Log | `.sflog.zip`, a bundle folder or a UDP `.jsonl`; newest in `logs/` if omitted |
| Output | Default `logs/videos/<log name>_3d.mp4` (`<log name>_3d_<S>-<E>s.mp4` with `--time-range S E`). An existing file is never overwritten (`_2`, `_3`, ... is appended); `--force` overwrites; `--out FILE` sets the name |
| Range | Default: the longest stretch with thrust, padded by 1 s each side. `--time-range S E` is seconds from the first sample |
| Look | `--view oblique/top/side`, `--tilt-gain` (tilt exaggeration, default 3), `--vehicle-scale` (default 2), `--zoom`, `--size 1920x1080/1280x720` |
| Other | `--speed`, `--fps`, `--frames-dir DIR` (JPEG frames instead of an mp4), `--swiftshader` (Chrome software rendering) |

The battery voltage is the logged status value (about once a second), held until the next sample. The 3.4 V low-voltage mark is the firmware default (`safety.battery.low_v`); the log does not record it.

## 12. sf log stats

Answers the questions of the hardware checkout (`firmware/vehicle/docs/hardware_checkout.md`) from a bundle as tables. Pick one mode; it reads only the streams that its question needs. It prints numbers and facts only: the pass/fail line belongs to the procedure (this command does not judge). The implementation is `lib/sflog/stats.py`.

```bash
sf log stats <mode> [bundle] [--since seconds] [--until seconds]
```

Without `bundle` the newest bundle in `logs/` is used. Times are vehicle times (seconds since boot), the same as the time stamps at the start of the lines of `sf log info <bundle> --text`.

| Mode | Question | Streams read | Window |
|------|----------|--------------|--------|
| `tof` | While ARMED_GROUND (armed, on the ground), what did the downward floor ToF (the distance sensor under the vehicle) read in cm? Is there a valid reading above the floor band (5 cm)? | `flight_phase`, `tof_bottom` | optional |
| `spread` | How much do the valid ToF readings scatter in each 1 s window (maximum - minimum)? | `tof_bottom` | **required** |
| `loop` | When did the armed 400 Hz loop's `late` (late wakes) and `over budget` (work longer than the period) counters grow? The non-finite count, the log-path losses, the text-log losses, the rows with the ToF output stopped | `loop_timing`, `nonfinite`, `link_health` (optional: `text_log`, `tof_silence`) | optional |
| `events` | The peak height, the last time outside the hold height (`--target` +- `--band`), the time from the LANDING start until all motors stopped, and the last valid ToF before the stop | `tof_bottom`, `flight_phase` (`motor` when there is a LANDING) | optional |
| `turn` | Heading change in the window (signed; clockwise is positive), minimum and maximum ToF height, horizontal drift (the largest distance from the first position of the window; the estimator's value) | `attitude`, `tof_bottom`, `posvel` | **required** |
| `baro` | Barometric floor-relative height minus the ToF height (rms: root mean square, and maximum), the range and age of the learned offset | `baro_height`, `tof_bottom` | **required** |
| `boot` | The boot reasons (`reset_reason`) and the minimum and maximum battery voltage | `status` | optional |

Example outputs (flights of a SILS simulation bundle; the values are examples):

```text
$ sf log stats tof logs/checkout/s1_tof_floor.sflog.zip
Floor ToF while ARMED_GROUND
  ARMED_GROUND time       : 40.2 s in 1 run(s), from 12.0 s
  ToF samples             : 1206 (valid 1200, invalid 6)
  valid minimum           : 3.1 cm
  valid maximum           : 3.5 cm
  valid above 5 cm        : 0

$ sf log stats events logs/checkout/s3_takeoff.sflog.zip
Flight events
  peak ToF height         : 0.816 m at 10.897 s
  last time outside 0.50 +- 0.07 m : 12.745 s (after the peak, before landing)
  LANDING                 : started at 13.022 s; all motors stopped 3.140 s later
  last valid ToF before stop: 3.8 cm, 0.080 s before the stop; invalid samples in the last 0.5 s: 3

$ sf log stats turn logs/checkout/s3_turn.sflog.zip --since 22 --until 36
Turn in the window
  yaw change              : -270.0 deg (min -270.0, max +0.0); positive = clockwise seen from above
  ToF height              : min 1.113 m, max 1.256 m
  horizontal drift        : max 0.000 m from the first position (ESKF estimate)

$ sf log stats loop logs/checkout/s4_final.sflog.zip
Armed 400 Hz loop (counters are totals since boot)
  no armed counter changed (late and over budget stayed 0 while ARMed)
  imu      armed: cycles 3456, late 0, over budget 0, period max 2.50 ms, busy max 0.00 ms
  control  armed: cycles 3456, late 0, over budget 0, period max 2.50 ms, busy max 0.00 ms
  non-finite              : detected 0 of 3456 armed cycles
  log paths               : log_stream_overflow=0, send_drops=0, entry_drops=0, text_dropped_lines=0
  text log                : 136 lines, 0 dropped before a line (loss markers)
  ToF output stopped      : tof_silent=1 in 0 of 1250 rows
```

The `loop` table lists, with the time, only the rows in which an armed counter (`late`, `over budget`) grew. The counters are totals since boot.

### Options

| Argument | Description | Default |
|----------|-------------|---------|
| `mode` (positional, required) | One of `tof`, `spread`, `loop`, `events`, `turn`, `baro`, `boot` | - |
| `bundle` (positional, optional) | Bundle path (`.sflog.zip` or directory) | Newest bundle in `logs/` |
| `--since SECONDS`, `--until SECONDS` | Window (vehicle time in seconds, both ends included). Both are required for `spread`, `turn` and `baro` | the whole bundle |
| `--target METERS` | `events` only: the hold height the settling is judged against | 0.5 (the auto-takeoff height) |
| `--band METERS` | `events` only: the half-width around the hold height that counts as settled | 0.07 (the POS_HOLD hold accuracy) |

### Exit codes and missing streams

- Exit code 0: a table was printed.
- Exit code 1: the bundle cannot be read, a needed stream is missing, or the window holds no sample. The message names the stream (for example `stream 'loop_timing' is missing or empty in this bundle`). A capture from older firmware, or any bundle without the diagnostics streams (past real-vehicle logs in `logs/`, for example), ends this way in `loop` and similar modes. `boot` works on every bundle that has `status`.
- In `events`, a bundle that records the motors only while ARMed (a SILS bundle) has no all-zero row after the stop. The command then says that no all-zero row exists and prints the time the LANDING state ended instead. A real-vehicle bundle also records motor rows before and after ARM.
