# StampFly Flight Log Format v1

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

<!-- GENERATED FILE - do not edit; run protocol/tools/gen_flight_log.py
生成ファイル - 手で編集しないこと。protocol/tools/gen_flight_log.py を実行して再生成する。 -->
<!-- Source of truth / 正本: protocol/spec/flight_log.yaml -->

## 1. 概要

StampFly フライトログ一式（`.sflog.zip`、拡張子固定）は、1回の飛行/シミュレーション実行につき1個の zip ファイルに、取得条件を記した `meta.json`、列定義を記した `schema.json`、パケット種別ごとの CSV を平坦に格納したもの。決定文書: `docs/plans/flight-log-format-plan.md`。

### 容器

種別: zip（deflate 圧縮）／拡張子: `.sflog.zip`／レイアウト: flat（サブフォルダ無し）。展開済みフォルダも同じレイアウトで読み込み可能。未知のファイルは無視する。

### ファイル命名

| 由来 | 命名パターン |
|---|---|
| 実機 (vehicle) | `flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| SILS | `sils_<scenario>_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| シミュレータ (sim) | `sim_<backend>_<YYYYMMDD>T<HHMMSS>.sflog.zip` |

### CSV の規則

文字コード utf-8、1行目が列名、コメント行なし。1列目は常に `timestamp_us`（int）。数値は `%.7g` 形式。セルが空になるのは、このファイルを書いたファーム版がその項目を そもそも送らない場合だけ（既知の例は各列の description_ja/description_en を参照）。空欄が「直前値の保持」を意味することは無い -- 多レート信号の 整列・補間は lib/sflog/align.py が読み込み時に行う操作であり、一次記録の CSV には書き込まない。

### 単位（SI 系）

| 物理量 | 単位 |
|---|---|
| timestamp | us (microseconds) |
| angle | rad |
| angular_rate | rad/s |
| acceleration | m/s^2 |
| position | m (NED frame) |
| velocity | m/s (NED frame) |
| distance | m |
| thrust | N |
| torque | N*m |
| duty | 0..1 (fraction, dimensionless) |
| voltage | V |
| current | mA |
| magnetic_field | uT |
| pressure | Pa |
| quaternion | unitless (w,x,y,z) |

## 2. ストリーム一覧

| ストリーム | ファイル | 由来 | 公称レート | 必須 |
|---|---|---|---|---|
| `imu` | `imu.csv` | IMU+ESKF (0x40) | 400 Hz | 必須 |
| `attitude` | `attitude.csv` | IMU+ESKF (0x40) | 400 Hz | - |
| `posvel` | `posvel.csv` | PosVel (0x41) | 400 Hz | - |
| `rate_ref` | `rate_ref.csv` | RateRef (unified packet 0x50 fixed part) | 400 Hz | - |
| `motor` | `motor.csv` | Duty400 (0x4A) | 400 Hz | - |
| `ctrl_output` | `ctrl_output.csv` | ControlOutput400 (0x4B) | 400 Hz | - |
| `pilot` | `pilot.csv` | Control (0x42) | 50 Hz | - |
| `ctrl_ref` | `ctrl_ref.csv` | CtrlRef (0x48) | 50 Hz | - |
| `baro` | `baro.csv` | Baro (0x45) | 50 Hz | - |
| `tof_bottom` | `tof_bottom.csv` | ToF bottom (0x44) | 30 Hz | - |
| `tof_front` | `tof_front.csv` | ToF front (0x47) | 30 Hz | - |
| `flow` | `flow.csv` | Flow (0x43) | 100 Hz | - |
| `mag` | `mag.csv` | Mag (0x46) | 25 Hz | - |
| `status` | `status.csv` | Status (0x4F) | 1 Hz | - |
| `eskf_cov` | `eskf_cov.csv` | ESKF P-diag (0x49) | (可変/事象駆動) | - |
| `truth` | `truth.csv` | sils/sim | (可変/事象駆動) | - |
| `events` | `events.csv` | sils | (可変/事象駆動) | - |

## 3. 各ストリームの列

#### imu（`imu.csv`）

由来: IMU+ESKF (0x40) ／ 公称レート: 400 Hz ／ 必須: 必須

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ（SILS は仮想時計）。 制御周期が新しい IMU 標本を得られなかった周期では、前の IMU 標本と同じ値が連続する（実機で観測済み。行の一意な識別には `seq` を使うこと）。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `gyro_x` | float | rad/s | 推定器入力の角速度 X（機体座標系、フィルタ後）。 |
| `gyro_y` | float | rad/s | 推定器入力の角速度 Y（機体座標系、フィルタ後）。 |
| `gyro_z` | float | rad/s | 推定器入力の角速度 Z（機体座標系、フィルタ後）。 |
| `accel_x` | float | m/s^2 | 推定器入力の加速度 X（機体座標系、フィルタ後）。 |
| `accel_y` | float | m/s^2 | 推定器入力の加速度 Y（機体座標系、フィルタ後）。 |
| `accel_z` | float | m/s^2 | 推定器入力の加速度 Z（機体座標系、フィルタ後）。 |
| `gyro_raw_x` | float | rad/s | フィルタ前の生角速度 X。vehicle ファームは IMU 側 LPF を持たない ため raw == filtered（電文互換のため両方送っている）。 |
| `gyro_raw_y` | float | rad/s | フィルタ前の生角速度 Y（vehicle では gyro_y と同値）。 |
| `gyro_raw_z` | float | rad/s | フィルタ前の生角速度 Z（vehicle では gyro_z と同値）。 |
| `accel_raw_x` | float | m/s^2 | フィルタ前の生加速度 X（vehicle では accel_x と同値）。 |
| `accel_raw_y` | float | m/s^2 | フィルタ前の生加速度 Y（vehicle では accel_y と同値）。 |
| `accel_raw_z` | float | m/s^2 | フィルタ前の生加速度 Z（vehicle では accel_z と同値）。 |

#### attitude（`attitude.csv`）

由来: IMU+ESKF (0x40) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。imu.csv と同一パケット 由来なので、同じ行番号の imu.csv と同じ値になる。制御周期が 新しい IMU 標本を得られなかった周期では、前の IMU 標本と同じ 値が連続する（実機で観測済み。行の一意な識別には `seq` を 使うこと）。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `quat_w` | float | unitless | 姿勢推定クォータニオンの実部 w（機体座標系）。 |
| `quat_x` | float | unitless | 姿勢推定クォータニオンの虚部 x。 |
| `quat_y` | float | unitless | 姿勢推定クォータニオンの虚部 y。 |
| `quat_z` | float | unitless | 姿勢推定クォータニオンの虚部 z。 |
| `gyro_bias_x` | float | rad/s | ESKF が推定したジャイロバイアス X。 |
| `gyro_bias_y` | float | rad/s | ESKF が推定したジャイロバイアス Y。 |
| `gyro_bias_z` | float | rad/s | ESKF が推定したジャイロバイアス Z。 |
| `accel_bias_x` | float | m/s^2 | ESKF が推定した加速度バイアス X。 |
| `accel_bias_y` | float | m/s^2 | ESKF が推定した加速度バイアス Y。 |
| `accel_bias_z` | float | m/s^2 | ESKF が推定した加速度バイアス Z。 |

#### posvel（`posvel.csv`）

由来: PosVel (0x41) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。制御周期が新しい IMU 標本を得られなかった周期では、前の IMU 標本と同じ値が連続する （実機で観測済み。行の一意な識別には `seq` を使うこと）。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `pos_x` | float | m | ESKF が推定した位置 X（NED 座標系）。 |
| `pos_y` | float | m | ESKF が推定した位置 Y（NED 座標系）。 |
| `pos_z` | float | m | ESKF が推定した位置 Z（NED 座標系、下向き正）。 |
| `vel_x` | float | m/s | ESKF が推定した速度 X（NED 座標系）。 |
| `vel_y` | float | m/s | ESKF が推定した速度 Y（NED 座標系）。 |
| `vel_z` | float | m/s | ESKF が推定した速度 Z（NED 座標系、下向き正）。 |

#### rate_ref（`rate_ref.csv`）

由来: RateRef (unified packet 0x50 fixed part) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU 標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を 得られなかった周期ではこの値が前の周期と同じになる -- しかし rate_ref 自体は制御則がその周期に計算した新しい値であり、 timestamp_us が同じでも値は異なり得る（実機で観測済み）。 行の対応付けには timestamp_us ではなく `seq` を使うこと。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `rate_ref_roll` | float | rad/s | 内側ループ（レート制御）のロール角速度目標。 |
| `rate_ref_pitch` | float | rad/s | 内側ループ（レート制御）のピッチ角速度目標。 |
| `rate_ref_yaw` | float | rad/s | 内側ループ（レート制御）のヨー角速度目標。 |

#### motor（`motor.csv`）

由来: Duty400 (0x4A) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU 標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を 得られなかった周期ではこの値が前の周期と同じになる -- しかし duty 自体はミキサーがその周期に計算した新しい値であり、 timestamp_us が同じでも値は異なり得る（実機で観測済み、 rate_ref と同じ理由）。行の対応付けには timestamp_us ではなく `seq` を使うこと。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `duty_FR` | float | 0..1 | 前右モータの実際の duty 比。`sf sysid fit` のプラント入力 （レートループ同定）。 |
| `duty_RR` | float | 0..1 | 後右モータの実際の duty 比。 |
| `duty_RL` | float | 0..1 | 後左モータの実際の duty 比。 |
| `duty_FL` | float | 0..1 | 前左モータの実際の duty 比。 |

#### ctrl_output（`ctrl_output.csv`）

由来: ControlOutput400 (0x4B) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ControlTask は IMU 標本の時刻をそのまま使うため、制御周期が新しい IMU 標本を 得られなかった周期ではこの値が前の周期と同じになる -- しかし 指令推力・トルク自体は制御則がその周期に計算した新しい値であり、 timestamp_us が同じでも値は異なり得る（実機で観測済み、 rate_ref と同じ理由）。行の対応付けには timestamp_us ではなく `seq` を使うこと。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `thrust` | float | N | ミキサー手前のコントローラ指令推力（合計）。どのミキサーで 飛んだかに依存しないプラント入力。 |
| `torque_roll` | float | N*m | ミキサー手前のコントローラ指令ロールトルク。 |
| `torque_pitch` | float | N*m | ミキサー手前のコントローラ指令ピッチトルク。 |
| `torque_yaw` | float | N*m | ミキサー手前のコントローラ指令ヨートルク。 |

#### pilot（`pilot.csv`）

由来: Control (0x42) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `throttle` | float | 0..1 | 操縦スティックのスロットル入力。 |
| `roll` | float | -1..1 | 操縦スティックのロール入力。 |
| `pitch` | float | -1..1 | 操縦スティックのピッチ入力。 |
| `yaw` | float | -1..1 | 操縦スティックのヨー入力。 |

#### ctrl_ref（`ctrl_ref.csv`）

由来: CtrlRef (0x48) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `flight_mode` | int | enum | 飛行モード。0=ACRO, 1=STABILIZE, 2=ALT_HOLD, 3=POS_HOLD。 |
| `angle_ref_roll` | float | rad | 外側ループ（姿勢制御）のロール角目標。 |
| `angle_ref_pitch` | float | rad | 外側ループ（姿勢制御）のピッチ角目標。 |
| `total_thrust` | float | N | コントローラが計算した合計推力指令（50Hz）。 |
| `duty_FR` | float | 0..1 | 前右モータの duty 比（50Hz、CtrlRef パケット由来）。400Hz の 実測値は motor.csv を使うこと。 |
| `duty_RR` | float | 0..1 | 後右モータの duty 比（50Hz、CtrlRef パケット由来）。 |
| `duty_RL` | float | 0..1 | 後左モータの duty 比（50Hz、CtrlRef パケット由来）。 |
| `duty_FL` | float | 0..1 | 前左モータの duty 比（50Hz、CtrlRef パケット由来）。 |
| `alt_setpoint` | float | m | 高度保持の目標高度（ALT_HOLD/POS_HOLD）。 |
| `alt_vel_target` | float | m/s | 高度制御の目標上昇率。 |
| `climb_rate_cmd` | float | m/s | 操縦入力から生成した上昇率指令。 |
| `pos_setpoint_x` | float | m | 水平位置保持の目標位置 X（POS_HOLD、NED 座標系）。 |
| `pos_setpoint_y` | float | m | 水平位置保持の目標位置 Y（POS_HOLD、NED 座標系）。 |

#### baro（`baro.csv`）

由来: Baro (0x45) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `altitude` | float | m | 気圧高度（相対値、基準は起動時気圧）。 |
| `pressure` | float | Pa | 気圧。本書の SI 規約どおり Pa で統一する。電文上は hPa で 送られる（data_stream_wire.hpp の WireBaro 構造体コメント 「電文上は hPa、ファーム内部は Pa」参照）ため、変換時に ×100 して記録する。 |

#### tof_bottom（`tof_bottom.csv`）

由来: ToF bottom (0x44) ／ 公称レート: 30 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `distance` | float | m | 下向き ToF センサが測定した距離。 |
| `status` | int | enum | センサ状態。0 = 有効値。 |

#### tof_front（`tof_front.csv`）

由来: ToF front (0x47) ／ 公称レート: 30 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `distance` | float | m | 前向き ToF センサが測定した距離。 |
| `status` | int | enum | センサ状態。0 = 有効値。 |

#### flow（`flow.csv`）

由来: Flow (0x43) ／ 公称レート: 100 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `dx` | int | counts | オプティカルフローの X 方向積算カウント。 |
| `dy` | int | counts | オプティカルフローの Y 方向積算カウント。 |
| `quality` | int | 0..255 | フローセンサの信頼度指標。 |

#### mag（`mag.csv`）

由来: Mag (0x46) ／ 公称レート: 25 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `x` | float | uT | 地磁気 X（機体座標系）。 |
| `y` | float | uT | 地磁気 Y（機体座標系）。 |
| `z` | float | uT | 地磁気 Z（機体座標系）。 |

#### status（`status.csv`）

由来: Status (0x4F) ／ 公称レート: 1 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。Status パケットは uptime_ms しか持たないため timestamp_us = uptime_ms * 1000。 |
| `uptime_ms` | int | ms | 起動からの経過時間。 |
| `voltage` | float | V | バッテリ電圧。 |
| `current_ma` | float | mA | バッテリ電流。StatusPacket v3 (57B) 以降のみ送信。旧ファーム （v1/v2）を読んだ場合はこの列が空欄になる。 |
| `flight_state` | int | enum | 飛行状態（FlightState、起動シーケンス〜飛行〜着陸）。 |
| `sensor_health` | int | bitmask | センサ健全性のビットマスク。 |
| `eskf_status` | int | bitmask | ESKF 状態のビットマスク。bit0 = 推定器初期化済み。 |
| `reset_reason` | int | enum | 起動時リセット理由（esp_reset_reason()）。1=POWERON, 3=SW, 4=PANIC, 5=INT_WDT, 6=TASK_WDT, 9=BROWNOUT 等。 |
| `pid_roll_kp` | float | unitless (gain) | ロールレート PID の比例ゲイン（時刻付きで記録することで飛行中の 自動チューニングによる変更を追える）。 |
| `pid_roll_ti` | float | s | ロールレート PID の積分時間。 |
| `pid_roll_td` | float | s | ロールレート PID の微分時間。 |
| `pid_pitch_kp` | float | unitless (gain) | ピッチレート PID の比例ゲイン。 |
| `pid_pitch_ti` | float | s | ピッチレート PID の積分時間。 |
| `pid_pitch_td` | float | s | ピッチレート PID の微分時間。 |
| `pid_yaw_kp` | float | unitless (gain) | ヨーレート PID の比例ゲイン。 |
| `pid_yaw_ti` | float | s | ヨーレート PID の積分時間。 |
| `pid_yaw_td` | float | s | ヨーレート PID の微分時間。 |

#### eskf_cov（`eskf_cov.csv`）

由来: ESKF P-diag (0x49) ／ 公称レート: (可変/事象駆動) ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `p_pos_x` | float | m^2 | ESKF 共分散行列 P の対角成分、位置 X の分散。 |
| `p_pos_y` | float | m^2 | ESKF 共分散行列 P の対角成分、位置 Y の分散。 |
| `p_pos_z` | float | m^2 | ESKF 共分散行列 P の対角成分、位置 Z の分散。 |
| `p_vel_x` | float | (m/s)^2 | ESKF 共分散行列 P の対角成分、速度 X の分散。 |
| `p_vel_y` | float | (m/s)^2 | ESKF 共分散行列 P の対角成分、速度 Y の分散。 |
| `p_vel_z` | float | (m/s)^2 | ESKF 共分散行列 P の対角成分、速度 Z の分散。 |
| `p_att_x` | float | rad^2 | ESKF 共分散行列 P の対角成分、姿勢誤差角 X の分散。 |
| `p_att_y` | float | rad^2 | ESKF 共分散行列 P の対角成分、姿勢誤差角 Y の分散。 |
| `p_att_z` | float | rad^2 | ESKF 共分散行列 P の対角成分、姿勢誤差角 Z の分散。 |
| `p_bg_x` | float | (rad/s)^2 | ESKF 共分散行列 P の対角成分、ジャイロバイアス X の分散。 |
| `p_bg_y` | float | (rad/s)^2 | ESKF 共分散行列 P の対角成分、ジャイロバイアス Y の分散。 |
| `p_bg_z` | float | (rad/s)^2 | ESKF 共分散行列 P の対角成分、ジャイロバイアス Z の分散。 |
| `p_ba_x` | float | (m/s^2)^2 | ESKF 共分散行列 P の対角成分、加速度バイアス X の分散。 |
| `p_ba_y` | float | (m/s^2)^2 | ESKF 共分散行列 P の対角成分、加速度バイアス Y の分散。 |
| `p_ba_z` | float | (m/s^2)^2 | ESKF 共分散行列 P の対角成分、加速度バイアス Z の分散。 |

#### truth（`truth.csv`）

由来: sils/sim ／ 公称レート: (可変/事象駆動) ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | シミュレータの仮想時計によるマイクロ秒タイムスタンプ。 |
| `pos_x` | float | m | 物理モデルの真の位置 X（NED 座標系）。 |
| `pos_y` | float | m | 物理モデルの真の位置 Y（NED 座標系）。 |
| `pos_z` | float | m | 物理モデルの真の位置 Z（NED 座標系、下向き正）。 |
| `quat_w` | float | unitless | 物理モデルの真の姿勢クォータニオン実部 w。 |
| `quat_x` | float | unitless | 物理モデルの真の姿勢クォータニオン虚部 x。 |
| `quat_y` | float | unitless | 物理モデルの真の姿勢クォータニオン虚部 y。 |
| `quat_z` | float | unitless | 物理モデルの真の姿勢クォータニオン虚部 z。 |
| `vel_x` | float | m/s | 物理モデルの真の速度 X（NED 座標系）。 |
| `vel_y` | float | m/s | 物理モデルの真の速度 Y（NED 座標系）。 |
| `vel_z` | float | m/s | 物理モデルの真の速度 Z（NED 座標系、下向き正）。 |
| `rate_x` | float | rad/s | 物理モデルの真の角速度 X（機体座標系）。 |
| `rate_y` | float | rad/s | 物理モデルの真の角速度 Y（機体座標系）。 |
| `rate_z` | float | rad/s | 物理モデルの真の角速度 Z（機体座標系）。 |

#### events（`events.csv`）

由来: sils ／ 公称レート: (可変/事象駆動) ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | シミュレータの仮想時計によるマイクロ秒タイムスタンプ。 |
| `event` | str | n/a | 事象名（シナリオ入力: モード切替・ステップ入力等）。 |
| `value` | str | n/a | 事象の値（自由形式の文字列。数値の場合も文字列化して記録）。 |

## 4. meta.json

| キー | 型 | 説明 |
|---|---|---|
| `format` | str | 固定文字列 "stampfly-flight-log"。 |
| `version` | int | 形式のバージョン番号（本書は v1）。 |
| `source` | str | 取得元。"vehicle" / "sils" / "sim" のいずれか。 |
| `created_at` | str | バンドル生成日時。タイムゾーン付き ISO 8601 形式。 |
| `tool` | object | 生成ツールの情報。{name, version, git_hash} を持つ。git_hash は リポジトリルートで `git rev-parse --short HEAD` が取得できたときのみ 値が入り、それ以外は null。 |
| `firmware` | object\|null | 分かる場合のみのファーム情報 {version, git_hash}。不明なら null。 |
| `capture` | object\|null | 実機取得時の接続情報 {ip, port, duration_s}。SILS・シミュレータでは null。 |
| `streams` | object | ストリーム名 -> 要約統計の辞書。各要約は {rows, first_timestamp_us, last_timestamp_us, nominal_rate_hz, measured_rate_hz} を持つ。 measured_rate_hz は先頭・末尾のタイムスタンプと行数から計算した 実測レート。 |
| `derived` | bool | 一次記録は false。整列表など解析時に作った派生物は true。 |
| `notes` | str\|null | 自由記述の注記（例: "repeated timestamps: 1561 (imu)" のような、 制御周期が IMU 標本を再利用した回数の記録。JSONL 変換で imu の seq に対応付けられなかった行数も記録する）。 |

`schema.json` はこの文書のもとになった `protocol/spec/flight_log.yaml` から、バンドルに実際に含まれるストリームだけを抜き出して埋め込む （`lib/sflog/schema.py` の `schema_for()` が生成）。

---

<a id="english"></a>

## 1. Overview

A StampFly flight-log bundle (`.sflog.zip`, fixed extension) packs one zip file per flight/simulation run, containing `meta.json` (capture conditions), `schema.json` (column definitions), and one CSV per packet type, all flat (no subfolders). Decision document: `docs/plans/flight-log-format-plan.md`.

### Container

Type: zip (deflate compression) / Extension: `.sflog.zip` / Layout: flat (no subfolders). Readers also accept an already-extracted directory with the same layout. Unknown files are ignored.

### File naming

| Source | Naming pattern |
|---|---|
| vehicle | `flight_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| SILS | `sils_<scenario>_<YYYYMMDD>T<HHMMSS>.sflog.zip` |
| sim | `sim_<backend>_<YYYYMMDD>T<HHMMSS>.sflog.zip` |

### CSV rules

Encoding utf-8, header row, no comment rows. Column 1 is always `timestamp_us` (int). Numbers use `%.7g`. A cell is empty ONLY when the firmware version that produced this file does not send that field at all (see each column's description_ja/ description_en below for known cases). An empty cell never means "value held from an earlier sample" -- multi-rate alignment/interpolation is a read-time operation performed by lib/sflog/align.py and is never written into a primary-record CSV.

### Units (SI)

| Quantity | Unit |
|---|---|
| timestamp | us (microseconds) |
| angle | rad |
| angular_rate | rad/s |
| acceleration | m/s^2 |
| position | m (NED frame) |
| velocity | m/s (NED frame) |
| distance | m |
| thrust | N |
| torque | N*m |
| duty | 0..1 (fraction, dimensionless) |
| voltage | V |
| current | mA |
| magnetic_field | uT |
| pressure | Pa |
| quaternion | unitless (w,x,y,z) |

## 2. Stream list

| Stream | File | Source | Nominal rate | Required |
|---|---|---|---|---|
| `imu` | `imu.csv` | IMU+ESKF (0x40) | 400 Hz | yes |
| `attitude` | `attitude.csv` | IMU+ESKF (0x40) | 400 Hz | - |
| `posvel` | `posvel.csv` | PosVel (0x41) | 400 Hz | - |
| `rate_ref` | `rate_ref.csv` | RateRef (unified packet 0x50 fixed part) | 400 Hz | - |
| `motor` | `motor.csv` | Duty400 (0x4A) | 400 Hz | - |
| `ctrl_output` | `ctrl_output.csv` | ControlOutput400 (0x4B) | 400 Hz | - |
| `pilot` | `pilot.csv` | Control (0x42) | 50 Hz | - |
| `ctrl_ref` | `ctrl_ref.csv` | CtrlRef (0x48) | 50 Hz | - |
| `baro` | `baro.csv` | Baro (0x45) | 50 Hz | - |
| `tof_bottom` | `tof_bottom.csv` | ToF bottom (0x44) | 30 Hz | - |
| `tof_front` | `tof_front.csv` | ToF front (0x47) | 30 Hz | - |
| `flow` | `flow.csv` | Flow (0x43) | 100 Hz | - |
| `mag` | `mag.csv` | Mag (0x46) | 25 Hz | - |
| `status` | `status.csv` | Status (0x4F) | 1 Hz | - |
| `eskf_cov` | `eskf_cov.csv` | ESKF P-diag (0x49) | (variable / event-driven) | - |
| `truth` | `truth.csv` | sils/sim | (variable / event-driven) | - |
| `events` | `events.csv` | sils | (variable / event-driven) | - |

## 3. Columns per stream

#### imu (`imu.csv`)

Source: IMU+ESKF (0x40) / Nominal rate: 400 Hz / Required: yes

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp (SILS: virtual clock). Repeats the previous cycle's value when a control cycle did not receive a new IMU sample (observed on real hardware -- use `seq` to uniquely identify a row). |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `gyro_x` | float | rad/s | Estimator-input angular rate X (body frame, post-filter). |
| `gyro_y` | float | rad/s | Estimator-input angular rate Y (body frame, post-filter). |
| `gyro_z` | float | rad/s | Estimator-input angular rate Z (body frame, post-filter). |
| `accel_x` | float | m/s^2 | Estimator-input acceleration X (body frame, post-filter). |
| `accel_y` | float | m/s^2 | Estimator-input acceleration Y (body frame, post-filter). |
| `accel_z` | float | m/s^2 | Estimator-input acceleration Z (body frame, post-filter). |
| `gyro_raw_x` | float | rad/s | Pre-filter raw angular rate X. firmware/vehicle has no IMU-side LPF, so raw == filtered here (sent for wire compatibility with firmware that does filter). |
| `gyro_raw_y` | float | rad/s | Pre-filter raw angular rate Y (equals gyro_y on vehicle). |
| `gyro_raw_z` | float | rad/s | Pre-filter raw angular rate Z (equals gyro_z on vehicle). |
| `accel_raw_x` | float | m/s^2 | Pre-filter raw acceleration X (equals accel_x on vehicle). |
| `accel_raw_y` | float | m/s^2 | Pre-filter raw acceleration Y (equals accel_y on vehicle). |
| `accel_raw_z` | float | m/s^2 | Pre-filter raw acceleration Z (equals accel_z on vehicle). |

#### attitude (`attitude.csv`)

Source: IMU+ESKF (0x40) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. Comes from the same packet as imu.csv, so the value matches imu.csv at the same row index. Repeats the previous cycle's value when a control cycle did not receive a new IMU sample (observed on real hardware -- use `seq` to uniquely identify a row). |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `quat_w` | float | unitless | Attitude estimate quaternion real part w (body frame). |
| `quat_x` | float | unitless | Attitude estimate quaternion imaginary part x. |
| `quat_y` | float | unitless | Attitude estimate quaternion imaginary part y. |
| `quat_z` | float | unitless | Attitude estimate quaternion imaginary part z. |
| `gyro_bias_x` | float | rad/s | ESKF-estimated gyro bias X. |
| `gyro_bias_y` | float | rad/s | ESKF-estimated gyro bias Y. |
| `gyro_bias_z` | float | rad/s | ESKF-estimated gyro bias Z. |
| `accel_bias_x` | float | m/s^2 | ESKF-estimated accelerometer bias X. |
| `accel_bias_y` | float | m/s^2 | ESKF-estimated accelerometer bias Y. |
| `accel_bias_z` | float | m/s^2 | ESKF-estimated accelerometer bias Z. |

#### posvel (`posvel.csv`)

Source: PosVel (0x41) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. Repeats the previous cycle's value when a control cycle did not receive a new IMU sample (observed on real hardware -- use `seq` to uniquely identify a row). |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `pos_x` | float | m | ESKF-estimated position X (NED frame). |
| `pos_y` | float | m | ESKF-estimated position Y (NED frame). |
| `pos_z` | float | m | ESKF-estimated position Z (NED frame, positive down). |
| `vel_x` | float | m/s | ESKF-estimated velocity X (NED frame). |
| `vel_y` | float | m/s | ESKF-estimated velocity Y (NED frame). |
| `vel_z` | float | m/s | ESKF-estimated velocity Z (NED frame, positive down). |

#### rate_ref (`rate_ref.csv`)

Source: RateRef (unified packet 0x50 fixed part) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. ControlTask reuses the IMU sample's own timestamp, so this repeats the previous cycle's value when no new IMU sample arrived that cycle -- however rate_ref itself is freshly computed by the control law every cycle, so its value can differ even when timestamp_us is identical (observed on real hardware). Use `seq`, not timestamp_us, to pair rows across streams. |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `rate_ref_roll` | float | rad/s | Inner-loop (rate control) roll angular-rate reference. |
| `rate_ref_pitch` | float | rad/s | Inner-loop (rate control) pitch angular-rate reference. |
| `rate_ref_yaw` | float | rad/s | Inner-loop (rate control) yaw angular-rate reference. |

#### motor (`motor.csv`)

Source: Duty400 (0x4A) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. Repeats the previous cycle's value when no new IMU sample arrived that cycle -- but the duty values themselves are freshly computed by the mixer every cycle, so they can differ even when timestamp_us is identical (observed on real hardware, same reason as rate_ref). Use `seq`, not timestamp_us, to pair rows across streams. |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `duty_FR` | float | 0..1 | Actual commanded duty ratio, front-right motor. The real plant input `sf sysid fit` uses for rate-loop identification. |
| `duty_RR` | float | 0..1 | Actual commanded duty ratio, rear-right motor. |
| `duty_RL` | float | 0..1 | Actual commanded duty ratio, rear-left motor. |
| `duty_FL` | float | 0..1 | Actual commanded duty ratio, front-left motor. |

#### ctrl_output (`ctrl_output.csv`)

Source: ControlOutput400 (0x4B) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. Repeats the previous cycle's value when no new IMU sample arrived that cycle -- but the commanded thrust/torque themselves are freshly computed by the control law every cycle, so they can differ even when timestamp_us is identical (observed on real hardware, same reason as rate_ref). Use `seq`, not timestamp_us, to pair rows across streams. |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `thrust` | float | N | PRE-MIXER commanded total thrust. A mixer-agnostic plant input (does not depend on which mixer flew). |
| `torque_roll` | float | N*m | PRE-MIXER commanded roll body torque. |
| `torque_pitch` | float | N*m | PRE-MIXER commanded pitch body torque. |
| `torque_yaw` | float | N*m | PRE-MIXER commanded yaw body torque. |

#### pilot (`pilot.csv`)

Source: Control (0x42) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `throttle` | float | 0..1 | Pilot stick throttle input. |
| `roll` | float | -1..1 | Pilot stick roll input. |
| `pitch` | float | -1..1 | Pilot stick pitch input. |
| `yaw` | float | -1..1 | Pilot stick yaw input. |

#### ctrl_ref (`ctrl_ref.csv`)

Source: CtrlRef (0x48) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `flight_mode` | int | enum | Flight mode. 0=ACRO, 1=STABILIZE, 2=ALT_HOLD, 3=POS_HOLD. |
| `angle_ref_roll` | float | rad | Outer-loop (attitude control) roll angle reference. |
| `angle_ref_pitch` | float | rad | Outer-loop (attitude control) pitch angle reference. |
| `total_thrust` | float | N | Controller-computed total thrust command (50 Hz). |
| `duty_FR` | float | 0..1 | Front-right motor duty ratio (50 Hz, from the CtrlRef packet). Prefer motor.csv for the 400 Hz measured value. |
| `duty_RR` | float | 0..1 | Rear-right motor duty ratio (50 Hz, from the CtrlRef packet). |
| `duty_RL` | float | 0..1 | Rear-left motor duty ratio (50 Hz, from the CtrlRef packet). |
| `duty_FL` | float | 0..1 | Front-left motor duty ratio (50 Hz, from the CtrlRef packet). |
| `alt_setpoint` | float | m | Altitude-hold target altitude (ALT_HOLD/POS_HOLD). |
| `alt_vel_target` | float | m/s | Altitude-loop target climb rate. |
| `climb_rate_cmd` | float | m/s | Climb-rate command derived from pilot input. |
| `pos_setpoint_x` | float | m | Position-hold target X (POS_HOLD, NED frame). |
| `pos_setpoint_y` | float | m | Position-hold target Y (POS_HOLD, NED frame). |

#### baro (`baro.csv`)

Source: Baro (0x45) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `altitude` | float | m | Barometric altitude (relative to the power-on reference pressure). |
| `pressure` | float | Pa | Pressure, unified to Pa per this document's SI convention. The wire sends hPa (see the WireBaro struct comment in data_stream_wire.hpp: "hPa on the wire, Pa inside the firmware"), so the conversion multiplies by 100 before recording. |

#### tof_bottom (`tof_bottom.csv`)

Source: ToF bottom (0x44) / Nominal rate: 30 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `distance` | float | m | Distance measured by the downward-facing ToF sensor. |
| `status` | int | enum | Sensor status. 0 = valid reading. |

#### tof_front (`tof_front.csv`)

Source: ToF front (0x47) / Nominal rate: 30 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `distance` | float | m | Distance measured by the forward-facing ToF sensor. |
| `status` | int | enum | Sensor status. 0 = valid reading. |

#### flow (`flow.csv`)

Source: Flow (0x43) / Nominal rate: 100 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `dx` | int | counts | Optical flow accumulated count, X axis. |
| `dy` | int | counts | Optical flow accumulated count, Y axis. |
| `quality` | int | 0..255 | Flow sensor quality/confidence metric. |

#### mag (`mag.csv`)

Source: Mag (0x46) / Nominal rate: 25 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `x` | float | uT | Magnetic field X (body frame). |
| `y` | float | uT | Magnetic field Y (body frame). |
| `z` | float | uT | Magnetic field Z (body frame). |

#### status (`status.csv`)

Source: Status (0x4F) / Nominal rate: 1 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. The Status packet only carries uptime_ms, so timestamp_us = uptime_ms * 1000. |
| `uptime_ms` | int | ms | Elapsed time since boot. |
| `voltage` | float | V | Battery voltage. |
| `current_ma` | float | mA | Battery current. Sent only by StatusPacket v3 (57B) and later; empty when the source firmware only sent v1/v2. |
| `flight_state` | int | enum | Flight state (FlightState enum: boot sequence, flight, landing, ...). |
| `sensor_health` | int | bitmask | Sensor health bitmask. |
| `eskf_status` | int | bitmask | ESKF status bitmask. bit0 = estimator initialized. |
| `reset_reason` | int | enum | Boot-time reset reason from esp_reset_reason(). E.g. 1=POWERON, 3=SW, 4=PANIC, 5=INT_WDT, 6=TASK_WDT, 9=BROWNOUT. |
| `pid_roll_kp` | float | unitless (gain) | Roll-rate PID proportional gain (timestamped so an in-flight autotune/gain change over the flight can be traced). |
| `pid_roll_ti` | float | s | Roll-rate PID integral time. |
| `pid_roll_td` | float | s | Roll-rate PID derivative time. |
| `pid_pitch_kp` | float | unitless (gain) | Pitch-rate PID proportional gain. |
| `pid_pitch_ti` | float | s | Pitch-rate PID integral time. |
| `pid_pitch_td` | float | s | Pitch-rate PID derivative time. |
| `pid_yaw_kp` | float | unitless (gain) | Yaw-rate PID proportional gain. |
| `pid_yaw_ti` | float | s | Yaw-rate PID integral time. |
| `pid_yaw_td` | float | s | Yaw-rate PID derivative time. |

#### eskf_cov (`eskf_cov.csv`)

Source: ESKF P-diag (0x49) / Nominal rate: (variable / event-driven) / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `p_pos_x` | float | m^2 | ESKF covariance matrix P diagonal entry, position-X variance. |
| `p_pos_y` | float | m^2 | ESKF covariance matrix P diagonal entry, position-Y variance. |
| `p_pos_z` | float | m^2 | ESKF covariance matrix P diagonal entry, position-Z variance. |
| `p_vel_x` | float | (m/s)^2 | ESKF covariance matrix P diagonal entry, velocity-X variance. |
| `p_vel_y` | float | (m/s)^2 | ESKF covariance matrix P diagonal entry, velocity-Y variance. |
| `p_vel_z` | float | (m/s)^2 | ESKF covariance matrix P diagonal entry, velocity-Z variance. |
| `p_att_x` | float | rad^2 | ESKF covariance matrix P diagonal entry, attitude-error-angle-X variance. |
| `p_att_y` | float | rad^2 | ESKF covariance matrix P diagonal entry, attitude-error-angle-Y variance. |
| `p_att_z` | float | rad^2 | ESKF covariance matrix P diagonal entry, attitude-error-angle-Z variance. |
| `p_bg_x` | float | (rad/s)^2 | ESKF covariance matrix P diagonal entry, gyro-bias-X variance. |
| `p_bg_y` | float | (rad/s)^2 | ESKF covariance matrix P diagonal entry, gyro-bias-Y variance. |
| `p_bg_z` | float | (rad/s)^2 | ESKF covariance matrix P diagonal entry, gyro-bias-Z variance. |
| `p_ba_x` | float | (m/s^2)^2 | ESKF covariance matrix P diagonal entry, accel-bias-X variance. |
| `p_ba_y` | float | (m/s^2)^2 | ESKF covariance matrix P diagonal entry, accel-bias-Y variance. |
| `p_ba_z` | float | (m/s^2)^2 | ESKF covariance matrix P diagonal entry, accel-bias-Z variance. |

#### truth (`truth.csv`)

Source: sils/sim / Nominal rate: (variable / event-driven) / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Microsecond timestamp from the simulator's virtual clock. |
| `pos_x` | float | m | Physics-model ground-truth position X (NED frame). |
| `pos_y` | float | m | Physics-model ground-truth position Y (NED frame). |
| `pos_z` | float | m | Physics-model ground-truth position Z (NED frame, positive down). |
| `quat_w` | float | unitless | Physics-model ground-truth attitude quaternion real part w. |
| `quat_x` | float | unitless | Physics-model ground-truth attitude quaternion imaginary part x. |
| `quat_y` | float | unitless | Physics-model ground-truth attitude quaternion imaginary part y. |
| `quat_z` | float | unitless | Physics-model ground-truth attitude quaternion imaginary part z. |
| `vel_x` | float | m/s | Physics-model ground-truth velocity X (NED frame). |
| `vel_y` | float | m/s | Physics-model ground-truth velocity Y (NED frame). |
| `vel_z` | float | m/s | Physics-model ground-truth velocity Z (NED frame, positive down). |
| `rate_x` | float | rad/s | Physics-model ground-truth angular rate X (body frame). |
| `rate_y` | float | rad/s | Physics-model ground-truth angular rate Y (body frame). |
| `rate_z` | float | rad/s | Physics-model ground-truth angular rate Z (body frame). |

#### events (`events.csv`)

Source: sils / Nominal rate: (variable / event-driven) / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Microsecond timestamp from the simulator's virtual clock. |
| `event` | str | n/a | Event name (scenario input: mode switch, step input, etc.). |
| `value` | str | n/a | Event value (free-form string; numeric values are stringified). |

## 4. meta.json

| Key | Type | Description |
|---|---|---|
| `format` | str | Fixed literal string "stampfly-flight-log". |
| `version` | int | Format version number (this document defines v1). |
| `source` | str | Capture origin, one of "vehicle", "sils", "sim". |
| `created_at` | str | Bundle creation timestamp, ISO 8601 with timezone. |
| `tool` | object | Generating tool info: {name, version, git_hash}. git_hash is set only when `git rev-parse --short HEAD` succeeds at the repository root, otherwise null. |
| `firmware` | object\|null | Firmware info {version, git_hash} when known, otherwise null. |
| `capture` | object\|null | Vehicle capture connection info {ip, port, duration_s}. Null for SILS/sim sources. |
| `streams` | object | Dict of stream name -> summary stats {rows, first_timestamp_us, last_timestamp_us, nominal_rate_hz, measured_rate_hz}. measured_rate_hz is computed from row count and the first/last timestamps. |
| `derived` | bool | false for a primary record; true for a derived product such as an aligned table. |
| `notes` | str\|null | Free-form notes (e.g. "repeated timestamps: 1561 (imu)" recording how many control cycles reused a stale IMU sample; JSONL conversion also records how many rows could not be matched to an imu seq). |

`schema.json` is embedded per-bundle from the same `protocol/spec/flight_log.yaml` this document is generated from, restricted to the streams actually present (built by `schema_for()` in `lib/sflog/schema.py`).
