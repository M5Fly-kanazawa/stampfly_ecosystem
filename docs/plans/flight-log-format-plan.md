# 標準フライトログ形式の統一計画

作成: 2026-09-11。**状態: 決定（2026-09-11 ユーザー確認）。実装中。**

発端: 2026-09-10 の SCI チュートリアルで `sf log wifi -o flight.csv` の例を示したが、
`sf log wifi` の既定の保存形式は CSV ではなく JSONL（JSON Lines: 1 行に 1 件の JSON
レコードを並べたテキスト形式）だった。調べると、問題は「JSONL か CSV か」より広く、
**リポジトリ内に少なくとも 14 種類の相異なるログ形式があり、標準が決まっておらず、
どのツールがどれを読めるかが一致していない**ことにあった。

方針（2026-09-11 ユーザー決定）:

- **一次記録には観測値以外を書かない。** サンプリング周期の違うデータを前の値で埋める
  形式は、後から見たときに実測値か埋め値か区別できず、研究データとして不適格。整列や
  補間は解析時の操作であり、必ず派生物として区別する。
- **標準は 1 ファイル。** 信号ごとに CSV を分けるのは保管・共有で不利なので、zip に
  固めて 1 ファイルにする。先行例（rosbag / MCAP / PX4 ULog / ArduPilot DataFlash）は
  いずれも「1 ファイルの自己記述コンテナに各信号を原レートのまま時刻付きで入れ、CSV は
  書き出し時に信号ごとに分ける」で収束しており、それに倣う。
- **リポジトリ内の全ツールをこの形式に統一する。**
- **旧ファーム（`firmware/vehicle_old`）はアーカイブ予定のため考慮しない。** 旧ファーム
  専用の形式・コードは統一の対象外とし、整理する。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| なぜ JSONL が既定か | 2026-04-01 のコミット ba63edf9 が「疎な（空欄だらけの）結合 CSV を避ける」目的で CSV→JSONL に切り替えた。5 か月後の 2026-09-04 に 55a19404 が `sf sysid fit` 用に 400 Hz 整列 CSV（`-o *.csv`）を局所的に復活させた。両判断は上位文書で整理されず併存していた |
| 現状の問題 | (a) 標準が未定、(b) 14 種の形式が混在、(c) `sf log analyze` は現行のどの出力も読めない、(d) SILS の CSV は `sf sysid` 系が読めない、(e) 形式定義がコードにしか無い |
| 決定 | **標準 = 「StampFly フライトログ一式」**: 1 個の zip ファイル `flight_<日時>.sflog.zip` に、`meta.json`（取得条件）、`schema.json`（列の定義・単位）、パケット種別ごとの CSV（`imu.csv` `attitude.csv` `posvel.csv` … `status.csv`）を入れる。各 CSV は「そのセンサが出した値だけ」を原レートで時刻付きに持ち、埋め値を持たない。展開すれば普通の CSV で、Excel・MATLAB・pandas で直接開ける |
| 整列表 | 400 Hz に揃えた 1 枚の表が要る解析（同定など）は、共通の読み込み処理がメモリ上で作る。ファイルとして欲しいときは `sf log convert --aligned` で明示的に作り、名前と `meta.json` に派生物と記す。既定では書かない |
| 全ツール対応 | `sf log wifi/list/info/check/convert/viz/analyze`・`sf trim analyze`・`sf sysid *`・`sf cal plot`・SILS（書き出し・合否判定・GUI・動画）・`sf sim headless`・教育パッケージを一式形式に統一。JSONL 書き出しと旧ファーム系の入出力は削除 |
| 仕様の置き場 | `protocol/spec/flight_log.yaml` を正本（Single Source of Truth: 定義を 1 か所にだけ置き他は全てそこを参照する考え方）とし、Python の列定数・`schema.json`・文書の列表を生成。`sf log check` で適合検査。CI（変更のたびに自動で検査を走らせる仕組み）で「書き出し側の出力が適合」「全読み込み側が基準ファイルを読める」を検査 |
| 段階 | Phase 0 仕様と共通処理 → 1 書き出し側 → 2 読み込み側と旧コード整理 → 3 SILS・シミュレータ → 4 文書（§5） |

## 1. 現状の事実（調査 2026-09-11）

### 1.1 JSONL が既定になった経緯

| 日付 | コミット | 内容 |
|------|---------|------|
| 2026-04-01 15:53 | b16a5e4e | `tools/log_analyzer/udp_capture.py` 新設。当初は「タイムスタンプで結合した CSV」を出力（値が無い箇所は空欄になる疎な表） |
| 2026-04-01 17:01 | ba63edf9 | 約 1 時間後に CSV→JSONL へ切替。理由（原文要約）: 自己記述的（列の定義を外に持たなくてよい）、NaN・空欄が無い、1 行 = 1 センサ標本で列不一致が起きない、種別で grep できる、どの言語でも読める |
| 2026-09-04 | 55a19404 | `save_stream_csv()` 追加。`-o *.csv` のときだけ 400 Hz 整列 CSV を書く。理由: 「`sf log wifi` から `sf sysid fit` が読める CSV へ至る経路が無かった」。JSONL 既定は「変更しない」と明記 |
| 2026-09-08 | 4707c934 | 整列 CSV にモータ duty（400 Hz）を追加 |
| 2026-09-09 | a6956f81 | 整列 CSV に `vbat` を追加 |
| 2026-09-10 | b7d64631 | 整列 CSV に `ctrl_output_*` を追加 |

JSONL 既定の根拠は 4 月の 1 コミットのメッセージだけで、設計文書に理由の記載は無い。
その理由（1 行 1 観測、埋め値なし）自体は本計画の方針と一致しており、問題だったのは
「1 ファイルに種別の違う行が混ざり表計算ソフトで開けない」点と「整列表を別形式で
足したこと」である。唯一ファイル形式に触れる設計文書
`firmware/vehicle/docs/development_roadmap.md:283` は「`logs/<date>_<mode>_<seq>.jsonl`
形式で保存」と書くが、実装の命名は `stampfly_udp_<timestamp>.jsonl` で、`<mode>`
`<seq>` は存在しない。

### 1.2 現行 2 形式の内容差

UDP Data Stream（ポート 8890）で機体から届くパケット種別と、2 形式への収容状況:

| パケット | 公称レート | 主な内容 | JSONL | 整列 CSV（現行 35 列） |
|---------|-----------|---------|-------|----------------------|
| IMU+ESKF (0x40) | 400 Hz | gyro/accel（補正後・生値）、quat、gyro/accel バイアス | あり | 補正後・quat・バイアスのみ。**生値なし** |
| PosVel (0x41) | 400 Hz | 位置・速度推定（ESKF） | あり | **なし** |
| RateRef | 400 Hz | 角速度目標（統合パケットの固定部） | あり | あり |
| Duty400 (0x4A) | 400 Hz | モータ duty 4 本 | あり | あり |
| ControlOutput400 (0x4B) | 400 Hz | 推力・トルク指令 | あり | あり |
| Control (0x42) | 50 Hz | 操縦入力（スティック 4 軸） | あり | **なし** |
| CtrlRef (0x48) | 50 Hz | 飛行モード、角度目標、総推力、モータ duty、高度目標、上昇率指令、位置目標 | あり | モード・角度目標・総推力・duty のみ |
| Flow (0x43) | 100 Hz | オプティカルフロー | あり | **なし** |
| ToF 下/前 (0x44/0x47) | 30 Hz | 距離・状態 | あり | **なし** |
| Baro (0x45) | 50 Hz | 気圧高度・気圧 | あり | **なし** |
| Mag (0x46) | 25 Hz | 地磁気 3 軸 | あり | **なし** |
| ESKF P-diag (0x49) | 未送信 | 共分散対角 | 空 | なし |
| Status (0x4F) | 1 Hz | 電圧、電流、飛行状態、センサ健全性、ESKF 状態、PID ゲイン | あり | 電圧のみ |

ファイルサイズ（実測、30 秒飛行、`logs/` の実ファイル）: JSONL 約 300 KB/s、整列 CSV
約 145 KB/s。JSONL は各行にキー名を繰り返すため約 2 倍。

### 1.3 ログ形式の棚卸し

**現行**は firmware/vehicle と対になるもの、**旧**は vehicle_old（WebSocket 時代）由来で
現行ファームでは発生しないもの。旧ファームはアーカイブ予定のため、**旧に分類した 5 形式は
統一の対象外**とし、専用コードは Phase 2 で削除する。

| # | 形式 | 系統 | 書き出し元 | 読み込み先 | 本計画での扱い |
|---|------|------|-----------|-----------|--------------|
| 1 | JSONL（id 別 1 行 1 標本） | 現行 | `sf log wifi`（既定） | `sf log viz`、`viz -i`、`sf log analyze --health`、`sf trim analyze`、`analysis/scripts/` 約 15〜30 本 | 書き出し廃止。既存ファイルは `sf log convert` で一式へ変換。研究用スクリプト向けに一式→JSONL の書き出しを残す |
| 2 | 整列 CSV（400 Hz、35 列） | 現行 | `sf log wifi -o *.csv` | `sf log viz`、`viz -i`、`sf sysid fit/rate-fit/noise/motor/drag/inertia` | 一次記録としては廃止。`sf log convert --aligned` の派生物として残す |
| 3 | 50 Hz monitor CSV（`t_us, mode, …` 25 列） | 現行 | `sf telemetry --csv`、`--web --csv`（別実装で重複） | **無し**（孤立） | `--csv` オプションを削除（記録は `sf log wifi` に一本化） |
| 4 | SILS trajectory CSV（20 列、50 fps、**角度が度**） | 現行 | `sf sils scenario/run`（C++ `emu_trajectory.cpp`、`hover_smoke.cpp` にも別実装） | `sf log viz`、`sf sils gui`、動画化、`.expect` 合否判定 | 一式形式へ移行（真値は `truth.csv`）。Phase 3 |
| 5 | オンボード Blackbox（SPIFFS `.bin`、76 B レコード） | 現行 | firmware/vehicle `sf_logger` | **無し**（PC 側ツール皆無） | 本計画の後続。取得コマンドを作るときは一式へ変換する |
| 6 | sim compare CSV（`time,x,y,z,roll,…` + `#` コメント） | 現行 | `sf sim headless`（VPython/Genesis） | 同ツール内のみ | 一式形式へ移行（`truth.csv` + `pilot.csv`）。Phase 3 |
| 7 | 教育パッケージ CSV（別名表で正規化） | 現行 | `generate_samples.py`（合成データ） | ノートブック 15 本・`examples/education/` | 読み込み処理を一式対応にする。合成データは一式で生成し直す |
| 8 | V2 バイナリ `.bin`（128 B） | 旧 | `sf log capture`（USB、vehicle_old のみ） | `sf log convert`、`sf cal plot` | 削除（`sf log capture`、`.bin` 変換、`sf cal plot` の `.bin` 読み込み） |
| 9 | legacy convert CSV | 旧 | `sf log convert` | `sf sysid noise/motor/drag/inertia`（"convert" 判定） | 削除 |
| 10 | Extended CSV（`timestamp_us`+`quat_w`） | 旧 | 無し（`wifi_capture.py` は 0c8dd6e1 で削除済み） | `sf log viz` | 描画コード削除 |
| 11 | FFT batch CSV（`timestamp_ms`+`gyro_corrected_x`） | 旧 | 無し（機体側 FFT 配信は 0c8dd6e1 で「不要」と判断済み） | `sf log viz`、**`sf log analyze`（無印）はこの列を無条件要求**、`reconstruct_duties.py` | 描画コード削除、`sf log analyze` は一式前提に作り直し |
| 12 | Normal WiFi telemetry CSV（`timestamp_ms`+`roll_deg`） | 旧 | 無し（`TelemetryWSPacket` 由来） | `sf log viz`、`sf sysid fit`（"legacy" 判定） | 描画コード・判定分岐を削除 |
| 13 | モータベンチ CSV（`voltage, omega`） | 対象外 | 手計測 | `tools/sysid/steady_state.py` | フライトログではないため対象外 |
| 14 | 解析結果 JSON（`metrics.json` 等） | 対象外 | `sf sysid` 系 | `sf sysid rate-tune`、参照値生成 | 解析結果であり対象外 |

### 1.4 現行ツールが読める形式（抜粋）

| ツール | JSONL | 整列 CSV | SILS trajectory CSV | 備考 |
|-------|-------|---------|---------------------|------|
| `sf log viz` | 対応 | 対応 | 対応 | 5 種の CSV を列名で自動判別 |
| `sf log viz -i` | 対応 | 対応 | 未確認 | |
| `sf log analyze`（無印） | 非対応 | **非対応** | 非対応 | FFT batch CSV の列 `gyro_corrected_x` を無条件要求（`flight_analysis.py:14-26` で確認） |
| `sf log analyze --health` | 対応 | 非対応 | 非対応 | 使う値（duty・gyro_z・電圧）は CSV にもあるが実装が JSONL 専用 |
| `sf log info` / `list` | 非対応 | 対応 | 対応 | `list` は `*.bin` と `*.csv` しか列挙しない（`log.py:302-308`） |
| `sf trim analyze` | 対応 | 非対応 | 非対応 | 位置・速度推定が必要 |
| `sf sysid fit` / `rate-fit` | 非対応 | 対応 | **非対応** | 判定ロジックが `plant_fit.py` と `loader.py` で二重実装 |
| `sf sysid noise/motor/drag/inertia` | 非対応 | 対応 | 非対応 | |
| 教育パッケージ `load_flight_log` | 非対応 | 一部 | 一部 | `timestamp_us` が別名表に無い |

### 1.5 文書の矛盾

| 文書 | 記述 |
|------|------|
| `docs/guides/tools.md:91-145` | 「JSONL が既定、`.csv` 指定時は整列 CSV」と現行どおり記載 |
| `docs/guides/flight-log-viz.md`、`tools/log_analyzer/README.md` | 例が全て `-o flight.csv`。JSONL が既定との記載なし |
| `docs/commands/sf-log.md` | 2026-09-11 朝の 0c8dd6e1 で全面改訂され現行実装と一致。本計画で既定を変えるので再改訂が要る |
| `lib/sfcli/commands/log.py:398` | docstring「UDP full-rate or legacy WebSocket」だが WebSocket 分岐は無い |
| `firmware/vehicle/docs/development_roadmap.md:283` | `logs/<date>_<mode>_<seq>.jsonl` と規定するが実装は `stampfly_udp_<timestamp>.jsonl` |
| SCI チュートリアル S2/S4/S5、実習ガイド、チートシート | 全例が `-o *.csv`。既定が JSONL であることの記載なし。S5「`sf log wifi` で取得した CSV」、S4 デモ手順は引数なし |
| `protocol/spec/websocket.yaml`、`messages.yaml` の `TelemetryWSPacket` | 現行ファームで撤去済みの WebSocket 電文を定義したまま |

### 1.6 仕様定義の所在

ログファイル形式を機械可読な仕様として定義した箇所は**存在しない**。現行の UDP Data
Stream の電文定義も `protocol/spec/` には無く、
`firmware/vehicle/components/sf_telemetry/include/data_stream_wire.hpp` と
`tools/log_analyzer/udp_capture.py` の 2 実装が互いに一致していることだけが拠り所である。
`protocol/generated/`・`protocol/tools/` は空。

### 1.7 先行例

| 形式 | 一次記録 | 多レートの扱い | パラメータ | CSV への出し方 |
|------|---------|--------------|-----------|---------------|
| rosbag（ROS 1）`.bag` | 1 ファイル。トピックごとの型定義を埋め込み、本体は「トピック + 時刻 + バイト列」。チャンク圧縮、末尾に索引 | トピックごとに独立、結合しない | 別途 | トピックごとに 1 CSV。突き合わせは解析側（pandas の merge_asof、MATLAB の synchronize） |
| rosbag2（ROS 2） | フォルダが単位。`metadata.yaml` + 保存ファイル（SQLite → Iron 以降 MCAP） | 同上 | 同上 | 同上 |
| MCAP | 1 ファイル。任意スキーマを内包、チャンク圧縮、索引、書きかけ復旧可 | 同上 | メタデータ欄 | `mcap` CLI、Foxglove Studio |
| PX4 ULog | 1 ファイル。型定義と全パラメータ初期値、**飛行中のパラメータ変更も時刻付きで記録** | 同上 | 本体に記録 | `ulog2csv` でトピックごとに 1 CSV |
| ArduPilot DataFlash | 1 ファイル。FMT/UNIT で列名・型・単位を自己記述 | 同上 | PARM で記録 | メッセージ種別ごとに CSV |
| Betaflight Blackbox | 1 ファイル。高速 I/P フレームと低速 S フレームを別々に記録 | 別フレーム | ヘッダ | `blackbox_decode` の CSV は S フレームの値を各行に繰り返す（本計画が避ける方式） |

共通点: 一次記録は 1 ファイル、信号は原レートで時刻付き、埋め値なし、パラメータは
時刻付きで本体に記録。CSV は派生物で信号ごとに分ける。信号別 CSV を一次記録にしている
例は無い。

## 2. 標準形式「StampFly フライトログ一式」v1

### 2.1 容器

| 項目 | 規定 |
|------|------|
| ファイル | zip（deflate 圧縮）。拡張子は `.sflog.zip`（Windows/macOS で普通に展開できる。ツールは `.sflog.zip` と、展開済みフォルダの両方を同じ読み込み処理で受け付ける） |
| 命名 | 実機 `flight_<YYYYMMDD>T<HHMMSS>.sflog.zip`（時刻は PC の取得開始時刻）、SILS `sils_<scenario>_<YYYYMMDD>T<HHMMSS>.sflog.zip`、シミュレータ `sim_<backend>_<日時>.sflog.zip` |
| 中身 | 平坦（サブフォルダ無し）。`meta.json`、`schema.json`、ストリームごとの CSV。未知のファイルは読み込み側が無視する（SILS の `results.json` などを同居させてよい） |
| CSV | UTF-8、1 行目が列名、コメント行なし。全ストリームの 1 列目は `timestamp_us`（機体起動基準のマイクロ秒。SILS は仮想時計）。数値は `%.7g` 相当の桁で書く（32 bit float の有効桁に合わせ、容量を抑える） |
| 単位 | SI。角度 rad、角速度 rad/s、加速度 m/s²、位置 m（NED）、推力 N、トルク N·m、duty 0〜1、電圧 V、電流 mA、地磁気 µT、気圧 Pa |
| 埋め値 | **無し。** 各 CSV の行はそのパケットが届いた時刻の観測だけ。列が空になるのは、そのファーム版がその項目を送らない場合のみ（`schema.json` に記す） |

### 2.2 ストリーム（1 パケット種別 = 1 CSV）

| ファイル | 由来パケット | 公称レート | 列（`timestamp_us` の後） |
|---------|-------------|-----------|--------------------------|
| `imu.csv` | IMU+ESKF (0x40) | 400 Hz | `seq`, `gyro_x/y/z`, `accel_x/y/z`, `gyro_raw_x/y/z`, `accel_raw_x/y/z` |
| `attitude.csv` | IMU+ESKF (0x40) | 400 Hz | `seq`, `quat_w/x/y/z`, `gyro_bias_x/y/z`, `accel_bias_x/y/z`（`imu.csv` と同一パケットなので時刻・`seq` は同じ値） |
| `posvel.csv` | PosVel (0x41) | 400 Hz | `seq`, `pos_x/y/z`, `vel_x/y/z` |
| `rate_ref.csv` | 統合パケット固定部 | 400 Hz | `seq`, `rate_ref_roll/pitch/yaw` |
| `motor.csv` | Duty400 (0x4A) | 400 Hz | `seq`, `duty_FR/RR/RL/FL` |
| `ctrl_output.csv` | ControlOutput400 (0x4B) | 400 Hz | `seq`, `thrust`, `torque_roll/pitch/yaw` |
| `pilot.csv` | Control (0x42) | 50 Hz | `throttle`, `roll`, `pitch`, `yaw` |
| `ctrl_ref.csv` | CtrlRef (0x48) | 50 Hz | `flight_mode`, `angle_ref_roll/pitch`, `total_thrust`, `duty_FR/RR/RL/FL`, `alt_setpoint`, `alt_vel_target`, `climb_rate_cmd`, `pos_setpoint_x/y` |
| `baro.csv` | Baro (0x45) | 50 Hz | `altitude`, `pressure` |
| `tof_bottom.csv` / `tof_front.csv` | ToF (0x44 / 0x47) | 30 Hz | `distance`, `status` |
| `flow.csv` | Flow (0x43) | 100 Hz | `dx`, `dy`, `quality` |
| `mag.csv` | Mag (0x46) | 25 Hz | `x`, `y`, `z` |
| `status.csv` | Status (0x4F) | 1 Hz | `uptime_ms`, `voltage`, `current_ma`, `flight_state`, `sensor_health`, `eskf_status`, `reset_reason`, `pid_roll_kp/ti/td`, `pid_pitch_kp/ti/td`, `pid_yaw_kp/ti/td`（ゲインは時刻付きで残る。飛行中の自動チューニングによる変更を追える） |
| `eskf_cov.csv` | ESKF P-diag (0x49) | 送信時のみ | `p_pos_x/y/z`, `p_vel_x/y/z`, `p_att_x/y/z`, `p_bg_x/y/z`, `p_ba_x/y/z` |
| `truth.csv` | SILS / シミュレータのみ | シミュレータ刻み | `pos_x/y/z`, `quat_w/x/y/z`, `vel_x/y/z`, `rate_x/y/z`（物理モデルの真値） |
| `events.csv` | SILS のみ（任意） | 事象ごと | `event`, `value`（シナリオ入力の事象） |

パケットが無いストリームのファイルは作らない（読み込み側は「無い」を許容する）。

**`seq` 列（制御周期の通し番号）:** 400 Hz の 6 ストリームは 1 行 = 機体の制御周期 1 回で、
`timestamp_us` は「その周期が使った IMU 標本の時刻」である（`LogStreamSample.timestamp`、
`data_types.hpp:260`）。制御周期が新しい IMU 標本を得られなかった周期では前の標本が
再利用され、**同じ時刻が連続する**（実機ログで 13% の周期に観測。§7）。時刻だけでは周期を
一意に識別できないため、統合パケットのヘッダ通し番号 × 8 + パケット内位置を `seq` として
持つ。`seq` はパケット欠落の検出にも使う。旧 JSONL からの変換では `imu.csv` の行番号を
`seq` とし、他の 400 Hz ストリームは（時刻, 同一時刻内の出現順）で対応付ける。

**重複と欠落の扱い:** 一次記録は観測を捨てない。同一時刻の行も全て残し、`sf log check` は
重複時刻の件数と `seq` の飛び（欠落パケット）の件数を warning で報告する。

**単位の例外:** 気圧は配線上 hPa で届くが、記録は SI の Pa に統一する（変換時に ×100）。

### 2.3 `meta.json` と `schema.json`

| ファイル | 内容 |
|---------|------|
| `meta.json` | `format`（固定文字列 `stampfly-flight-log`）、`version`（1）、`source`（`vehicle` / `sils` / `sim`）、`created_at`（ISO 8601）、`tool`（名前・版・git ハッシュ）、`firmware`（分かれば版・git ハッシュ）、`capture`（IP・ポート・要求秒数）、`streams`（ストリームごとの行数・最初と最後の時刻・公称レート・実測レート）、`derived`（一次記録は `false`）、`notes` |
| `schema.json` | `protocol/spec/flight_log.yaml` から生成した、そのファイルに含まれるストリームの列名・型・単位・説明。zip 単体で自己記述になる |

### 2.4 派生物

| コマンド | 出力 | 用途 |
|---------|------|------|
| `sf log convert <一式> --aligned [--rate 400]` | `<名前>_aligned400.csv`（1 枚の表。基準は `imu.csv` の時刻。他ストリームは直近値保持で結合し、ストリームごとに `<stream>_timestamp_us` 列を付けて保持であることを明示。`meta.json` 相当の情報は先頭列ではなく同名 `.meta.json` に書き `derived: true`） | Excel で 1 枚の表として眺める、外部ツールへ渡す |
| `sf log convert <一式> --jsonl` | 従来の JSONL | `analysis/scripts/` の研究用スクリプト |
| `sf log convert <旧 JSONL> ` | 一式 | 2026-04〜09 に取得した JSONL の救済 |

同定ツールは派生ファイルを経由せず、共通の読み込み処理 `aligned()` がメモリ上で同じ表を作る。

### 2.5 共通の読み込み・書き出し処理

新設パッケージ `lib/sflog/`（`pyproject.toml` は `lib/` 配下を自動探索するので、`__init__.py`
を置くだけで `pip install -e .` 済みの環境から import できる。依存は pandas・numpy・PyYAML
のみとし、いずれも `requirements.txt` で保証済み。plotly は保証されていないので `-i`
表示以外では使わない）:

| モジュール | 役割 |
|-----------|------|
| `schema.py` | `flight_log.yaml` から生成した列定数・単位・必須列 |
| `bundle.py` | `FlightLog.load(path)`（zip・フォルダ両対応。`streams` は名前→pandas DataFrame、`meta`）、`FlightLog.save(path)`、`write_bundle(...)` |
| `align.py` | `aligned(log, base="imu", streams=[...], method="hold")`。保持の出所を `<stream>_timestamp_us` 列で明示 |
| `check.py` | `sf log check` の実体。列の存在・型・単位の妥当範囲・時刻の単調増加・実測レートと公称の乖離 |
| `convert.py` | JSONL ↔ 一式、一式 → 整列 CSV |

## 3. 全ツールの対応（変更一覧）

### 3.1 書き出し側

| 対象 | 変更 |
|------|------|
| `sf log wifi` | 一式を書く。既定ファイル名 `logs/flight_<日時>.sflog.zip`。`-o` は保存先の指定のみ（拡張子による形式切替を廃止）。JSONL・整列 CSV の直接書き出しを廃止 |
| `udp_capture.py` | パケット解析は現状維持。`save_jsonl` / `save_stream_csv` を `lib/sflog` の書き出しに置き換え |
| `sf log convert` | JSONL → 一式、一式 → 整列 CSV / JSONL。旧ファームの `.bin` 変換は削除 |
| `sf log capture` | 削除（vehicle_old の USB バイナリ取得専用） |
| SILS | 一式を書く（§3.3） |
| `sf sim headless` | 一式を書く（`truth.csv` + `pilot.csv`） |
| `sf telemetry --csv` / `--web --csv` | オプション削除。記録は `sf log wifi` に一本化 |
| 教育パッケージ `generate_samples.py` | 合成データを一式で生成 |

### 3.2 読み込み側

| ツール | 変更 |
|-------|------|
| `sf log list` / `info` | 一式を列挙・要約（`meta.json` の内容とストリーム表） |
| `sf log check` | 新設 |
| `sf log viz` | 一式用の 1 つの描画処理に統合（現行 `visualize_stream.py` を基に、位置・速度・センサ・操縦入力のパネルを追加）。`-i`（Plotly）も一式対応。5 種の CSV 自動判別と旧描画コード（`visualize_extended.py`、`visualize_telemetry.py`、`visualize_jsonl.py`）を削除 |
| `sf log analyze` | 一式前提に作り直す。FFT は `imu.csv` から計算 |
| `sf log analyze --health` | 一式から読む（`ctrl_ref.csv` の duty、`imu.csv` の gyro_z、`status.csv` の電圧） |
| `sf trim analyze` | 一式から読む（`attitude.csv`、`posvel.csv`、`flow.csv`） |
| `sf sysid fit/rate-fit/noise/motor/drag/inertia` | 読み込みを `lib/sflog` の `aligned()` に統一。`plant_fit.py` の "stream"/"legacy" 判定と `tools/sysid/loader.py` の "stream"/"convert" 判定を削除。解析ロジックは変えない |
| `sf cal plot` | `.bin` ではなく一式の `mag.csv` から地磁気 XY を描く |
| `sf sils`（`.expect` 合否判定・GUI・動画化） | 一式から読む |
| 教育パッケージ `log_utils.py` | `load_flight_log()` が一式を受け付ける。別名表は残す |
| `analysis/scripts/`（研究用、一回性） | 書き換えは義務にしない。`sf log convert --jsonl` で従来入力を得る |

### 3.3 SILS と実機の同一化

`simulation-policy.md` は「実機と同一の同定パイプラインを SILS ログに適用できる」と述べる
が、現行の SILS trajectory.csv は `sf sysid` 系が読めない。

エミュレータ内の事実（2026-09-11 調査）: 機体側の Data Stream 送信部
（`sf_telemetry/data_stream.cpp`）はエミュレータにコンパイルされ実行もされるが、ソケット層
は不活性な代替実装（`simulator/sils/esp_idf_host/lwip/sockets.h`）で、`sendto` は捨てて
成功を装い、受信は常に空を返す。Data Stream は UDP で開始コマンドを受けて初めて送り
始めるため、電文は一切生成されない。一方、エミュレータ側の計装
（`emu_rate_stream.cpp`、`emu_vehicle_glue.cpp`）は既に、ファーム無改変のまま発行済み
トピック（`sf::sensor_imu`、`sf::estimate_state`、`sf::control_output`）を `latest()` で
読んで CSV を書いている。

決定: **エミュレータ側の計装がトピックを読んで一式の CSV を直接書く**（上記の前例と同じ
方式）。電文を横取りする案は、開始コマンドの注入と送信の横取りをソケット代替実装に
足す必要があり、決定論的な仮想時計の性質を守りながら実装する手間に見合わない。制御・
推定のコードは無改変で同一なので Code Identity は保たれる。電文の符号化・復号は実機経路
でしか検証されないが、これは現状と同じで後退ではない。

書き出すストリーム: `imu.csv` `attitude.csv` `posvel.csv` `rate_ref.csv` `motor.csv`
`ctrl_output.csv` `ctrl_ref.csv` `status.csv`（発行トピックから）、`truth.csv`（MuJoCo）、
`events.csv`（シナリオ入力）。現行の `trajectory.csv`（20 列、`sf sils` の `.expect` 合否
判定・GUI・動画化が使う）と `emu_rate_stream.cpp` の CSV は、消費側を一式へ移した時点で
廃止する。合格基準は「SILS 退行試験 34 本が全て通る」「ロールステップシナリオの一式で
`sf sysid fit` が動き、同定値が設定した物理パラメータと一致する」こと。

### 3.4 削除する旧ファーム系コード

`tools/log_capture/` 一式、`sf log capture`、`sf log convert` の `.bin` 経路、
`tools/log_analyzer/visualize_extended.py`・`visualize_telemetry.py`・`visualize_jsonl.py`・
`reconstruct_duties.py`・`stampfly_fft_*.csv`、`tools/calibration/plot_mag_xy.py` の `.bin`
解析、`tools/sysid/loader.py` の "convert" 分岐、`tools/sysid/plant_fit.py` の "legacy"
分岐、`lib/sfcli/commands/log.py` の 5 種 CSV 判別。削除は Git から復旧できるため、
中途半端に残さず消す（互換のための分岐を残さない）。

### 3.5 仕様の置き場と検査

| 成果物 | 内容 |
|-------|------|
| `protocol/spec/flight_log.yaml` | v1 のストリーム・列名・型・単位・レート・由来パケット・各ツールの必須列。**正本** |
| 生成物 | `lib/sflog/schema.py`（列定数）、`docs/reference/flight-log-format.md`（列表）。生成スクリプト `protocol/tools/gen_flight_log.py`、`--check` で生成物の鮮度を CI で検査 |
| `protocol/spec/data_stream.yaml` | 現行 UDP Data Stream 電文（`data_stream_wire.hpp` の内容）を仕様に収録。`websocket.yaml` と `TelemetryWSPacket` は「撤去済み」と注記して凍結（別作業でもよい） |
| CI | 現状の CI（`.github/workflows/sils-regression.yml`）は `tools/` や `lib/` の pytest を回していない。同ワークフローに pytest の工程を足し、(1) `lib/sflog` の単体テスト、(2) 基準一式（実機 1 本・SILS 1 本、`analysis/datasets/flightlog/` に置く）が `sf log check` を通る、(3) 全読み込み側コマンドが基準一式を読める、(4) `sf sysid fit --selftest` と参照同定値（`analysis/reports/rate_sysid_reference/`）が変わらない、を検査する |

## 4. 決定事項の記録

| # | 論点 | 決定 |
|---|------|------|
| 1 | 標準の容器 | zip に固めた信号別 CSV 一式（`.sflog.zip`）。1 枚の整列 CSV は派生物 |
| 2 | JSONL の扱い | 書き出し廃止。旧ファイルは `sf log convert` で救済。研究用スクリプト向けに一式→JSONL の書き出しだけ残す |
| 3 | 多レート信号の表現 | 一次記録は原レートのまま。整列は解析時にメモリ上で行い、ファイル化するときは派生物と明示 |
| 4 | メタデータ | zip 内の `meta.json` と `schema.json` |
| 5 | 旧ファーム系形式 | 対象外。専用コードは削除 |
| 6 | `sf telemetry --csv` | 削除 |
| 7 | 既定ファイル名 | `flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| 8 | SILS の出力方式 | エミュレータ側の計装が発行済みトピックを読んで一式を直接書く（§3.3。電文横取り案は不採用） |
| 9 | PID ゲイン | `status.csv` に時刻付きで残す（レビュー指摘: 飛行中の自動チューニングで変わり得る） |

## 5. 段階計画

**進捗（2026-09-11 時点、ブランチ `feature/flight-log-bundle`）:** Phase 0 = d85b7971、Phase 1 = c9f5ac1b、
Phase 2a（sysid/trim/cal plot/教育パッケージ）= 968a8bbd、基準一式 `analysis/datasets/flightlog/` = 870c8460、
Phase 2b（`sf log viz`/`analyze`/`--health`・旧描画コード削除、§3.2・§3.4）= e46817d2、
Phase 3（SILS・`sf sim headless`・SILS 退行試験 34 本 = 28 PASS + 5 既知の失敗 + 1 SKIP、§3.3）= 60a421b1、
Phase 4（文書、§6）= 本コミット。Phase 3 で判明した仕様の穴として、必須ストリームを取得元別にした
（`required_streams`: vehicle = imu、sils = imu + truth、sim = truth。§2.2 の補足）。
**未了:** (1) 実機での `sf log wifi -d 30` → `sf log check/viz/analyze` の確認（本計画の全セッションで実機なし）、
(2) SCI 資料の PDF は再ビルド済みだが Docswell への再アップロードは利用者のアカウントが必要、
(3) `sf sils sysid-gate`（SILS のモデル一致の合否判定）は一式経由で動くが判定は FAIL のまま
（SILS プラントと実機の差。simulation-policy のバックログ。一式化で motor.csv の duty が使えるように
なり入力経路が変わったため、以前の数値とは直接比較できない）、(4) Genesis のヘッドレス書き出しは
Genesis 未導入のため未検証。

| Phase | 内容 | 合格基準 | コミット |
|-------|------|---------|---------|
| 0 仕様と共通処理 | `flight_log.yaml`、生成スクリプト、`lib/sflog`（読み書き・整列・検査・変換）、単体テスト | 単体テストが通る。`logs/` の既存 JSONL を変換した一式が `check` を通り、整列表が現行 35 列の整列 CSV と数値一致する | 1 |
| 1 書き出し側 | `udp_capture.py`、`sf log wifi/list/info/check/convert` | 変換した一式で `list/info/check` が動く。**実機 30 秒取得は次回の実機セッションで確認（本セッションでは未検証）** | 1 |
| 2 読み込み側と整理 | §3.2 の全ツール、§3.4 の削除 | 基準一式を全コマンドが読める。`sf sysid fit --selftest` と参照同定値に変化なし（既存動作維持） | 2（対応、削除） |
| 3 SILS・シミュレータ | §3.3、`sf sim headless`、SILS 合否判定・GUI・動画 | SILS 退行試験が全て通る。ロールステップの一式で `sf sysid fit` が動く | 1〜2 |
| 4 文書 | §6 の全文書、SCI 資料と Docswell 再アップロード、メモリ更新 | 文書内の `sf log wifi` 例が全て一式前提で一貫 | 1 |

## 6. 影響を受ける文書

| 種別 | 対象 |
|------|------|
| ガイド | `docs/guides/tools.md`、`docs/guides/flight-log-viz.md`、`docs/commands/sf-log.md`、`tools/log_analyzer/README.md`、`tools/sysid/README.md`（`tools/log_capture/README.md` は削除） |
| 設計文書 | `firmware/vehicle/docs/development_roadmap.md:283`（命名規則）、`docs/architecture/simulation-policy.md`（形式の同一を明記）、`PROJECT_PLAN.md`（protocol/ の正本にログ形式を含める）、`firmware/vehicle/docs/coding_and_education.md`（Blackbox 例の記述） |
| 仕様 | 新設 `flight_log.yaml`・`data_stream.yaml`、`websocket.yaml`・`messages.yaml` の撤去済み注記 |
| 講習資料 | `docs/events/sci_tutorial_2026/slides/chapters/sci_s2_setup_sensors.tex`、`sci_s4_pid.tex`、`sci_s5_sim_analysis.tex`、`handson_guide.md`、`cheatsheet.md`、Docswell の PDF |
| ワークショップ | `firmware/workshop/lessons/lesson_07_sysid/README.md` |
| コード内文書 | `lib/sfcli/commands/log.py:398` docstring、`udp_capture.py` 冒頭 |

## 7. 実データで判明した機体側の課題（本計画の範囲外、要フォローアップ）

Phase 0 の検証で `logs/stampfly_udp_20260908T121243.jsonl`（30 秒、ホバー）を調べた結果:

| 観測 | 値 |
|------|-----|
| 400 Hz ストリームの行数（= 制御周期数） | 11,736（29.96 秒、約 392 Hz） |
| 同一時刻の重複行（IMU 標本の再利用） | 1,561（13%）。imu/posvel は内容も同一、rate_ref は値が異なる |
| 一意な IMU 時刻 | 10,175（約 340 Hz）。一意時刻の間隔で 2 周期分の飛びが 1,677 回 |

解釈: ControlTask は制御周期ごとに `LogStreamSample` を 1 件発行し、時刻に IMU 標本の
時刻を入れる。制御周期と IMU 更新がずれると、(a) 新しい IMU 標本が無い周期は前の標本で
制御し（13%）、(b) 制御周期に拾われなかった IMU 標本は記録に残らない（15%）。制御の
観点では 13% の周期が古い IMU 値で動いていることになり、ログの観点では制御周期そのものの
時刻が記録されていない。

推奨するフォローアップ（別計画）:
1. `LogStreamSample` と統合パケットに制御周期の時刻（または周期カウンタ）を追加する
   （電文 v2）。一式の `seq` 列はその受け皿になる。
2. IMU 更新と制御周期のずれの原因を調べる（IMU の実効レート、バス競合、タスク優先度）。

## 8. 未確認事項

- 一式のファイルサイズ実測（zip 圧縮後）。Phase 1 で既存 JSONL を変換して測る。
- ~~実機での `sf log wifi` 一式書き出し~~ → 2026-09-12 実機で 30 秒取得と `sf log viz` の表示を確認済み（ユーザー報告）。
- `lib/stampfly` SDK が `vehicle_connection.py` のどのクラスを使うか（旧 WebSocket 電文の
  解析コード `packet_parser.py` は生きた操縦経路が使っている可能性があるため、本計画では
  触らない）。

解消済み: SILS 内の Data Stream 経路（§3.3 に記載）。`analysis/scripts/` の JSONL 消費
スクリプトは 33 本。
