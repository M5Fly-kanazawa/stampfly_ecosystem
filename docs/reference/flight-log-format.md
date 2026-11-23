# StampFly Flight Log Format v1

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

<!-- GENERATED FILE - do not edit; run protocol/tools/gen_flight_log.py
生成ファイル - 手で編集しないこと。protocol/tools/gen_flight_log.py を実行して再生成する。 -->
<!-- Source of truth / 基準: protocol/spec/flight_log.yaml -->

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
| `flight_phase` | `flight_phase.csv` | FlightPhase400 (0x4C) | 400 Hz | - |
| `flight_flags` | `flight_flags.csv` | FlightFlags (0x4D) | 50 Hz | - |
| `tof_silence` | `tof_silence.csv` | ToFSilence (0x4E) | 50 Hz | - |
| `pilot` | `pilot.csv` | Control (0x42) | 50 Hz | - |
| `ctrl_ref` | `ctrl_ref.csv` | CtrlRef (0x48) | 50 Hz | - |
| `baro` | `baro.csv` | Baro (0x45) | 50 Hz | - |
| `baro_height` | `baro_height.csv` | BaroHeight (0x52) | 50 Hz | - |
| `tof_bottom` | `tof_bottom.csv` | ToF bottom (0x44) | 30 Hz | - |
| `tof_front` | `tof_front.csv` | ToF front (0x47) | 30 Hz | - |
| `flow` | `flow.csv` | Flow (0x43) | 100 Hz | - |
| `mag` | `mag.csv` | Mag (0x46) | 25 Hz | - |
| `status` | `status.csv` | Status (0x4F) | 1 Hz | - |
| `tof_diagnostics` | `tof_diagnostics.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `loop_timing` | `loop_timing.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `nonfinite` | `nonfinite.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `link_health` | `link_health.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `text_log` | `text_log.csv` | TextLog (0x51) | (可変/事象駆動) | - |
| `eskf_cov` | `eskf_cov.csv` | ESKF P-diag (0x49) | (可変/事象駆動) | - |
| `truth` | `truth.csv` | sils/sim | (可変/事象駆動) | - |
| `events` | `events.csv` | sils | (可変/事象駆動) | - |

### 取得元ごとの必須ストリーム

`meta.json` の `source` に応じて `sf log check` が存在を要求するストリーム（上の表の「必須」列は実機の既定）。

| 取得元 | 必須ストリーム |
|---|---|
| vehicle | `imu` |
| sils | `imu`, `truth` |
| sim | `truth` |

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

#### flight_phase（`flight_phase.csv`）

由来: FlightPhase400 (0x4C) ／ 公称レート: 400 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。imu.csv と同一パケット由来で、 同じ `seq` の imu.csv 行と同じ値になる（重複し得る。行の識別は `seq`）。 |
| `seq` | int | n/a | 制御周期の通し番号。実機取得では統合パケット 0x50 のヘッダ sequence（16bit、巻き戻りを取得側で展開）× 8 + パケット内 インデックス。SILS では制御周期カウンタ。旧 JSONL からの変換 では、imu ストリームの (timestamp_us, 同一時刻内の出現順) に 対応付けた imu 側の行番号を使う。対応が取れない行は空欄。 |
| `flight_state` | int | enum | 飛行状態（FlightState）を 400Hz で記録。status.csv の同名列は 1Hz なので、状態遷移の時刻はこちらで読むこと。 |
| `flip_phase` | int | enum | フリップ列生成器のフェーズ。0=Idle（未係合）, 1=Boost, 2=Spin, 3=Brake, 4=Recover, 5=Done。 |
| `flip_result` | int | enum | フリップの結果（FlipResult）。0=None, 1=Ok, 2=AbortedSpinTimeout, 3=AbortedGyroLimit, 4=AbortedRecoverTimeout。次のフリップが始まる まで直前の結果を保持する（disarm 中は 0）。 |
| `flip_phi` | float | rad | フリップ開始からの積算回転角（符号付き ∫ω dt をフリップ方向に正に 取ったもの、0 未満はクランプ）。Idle では 0。電文は rad×1000 の int16 （分解能 0.001 rad）。 |

#### flight_flags（`flight_flags.csv`）

由来: FlightFlags (0x4D) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ（統合パケット最終サンプルの IMU 時刻）。 |
| `pilot_arm` | int | 0/1 | 送信機の ARM スイッチ（制御フラグ bit0）。 |
| `pilot_flip` | int | 0/1 | 送信機の FLIP ボタン（制御フラグ bit1）。 |
| `flip_ready` | int | 0/1 | 制御器のフリップ実行条件 C2-C8 が成立（ControllerStatus.flip_ready）。 制御器が最後に発行した値で、disarm 中は更新されない（flight_phase の flight_state と合わせて読む）。 |
| `flip_block_reason` | int | enum | フリップ不成立理由（FlipBlockReason）。0=None, 1=NotFlying, 2=TooLow, 3=NotSteady, 4=BatteryLow, 5=EstimatorUnhealthy, 6=Cooldown, 7=Busy, 8=SourceConflict, 9=Disabled（flip.enable=0）。flip_ready=0 のときのみ意味を持つ。 |
| `arm_block` | int | enum | ARM 前判定の阻害理由（ArmBlock、SystemMode.arm_block）。0=None（判定を通る）, 1=Pairing, 2=Battery, 3=Calibrating, 4=BenchRelevel, 5=TiltPending, 6=TiltMismatch, 7=TofSilent（下向き ToF の出力停止のエピソード中: 新サンプルが来ないか、推定器がまだ ToF を取り込み直していない。ACRO を含む全モードで ARM を拒否）。 |
| `attitude_mismatch` | int | 0/1 | 地上で推定姿勢が重力方向と不一致（SystemStatus.attitude_mismatch）。 |
| `attitude_verified` | int | 0/1 | 地上で姿勢チェックが判定済みかつ一致（SystemStatus.attitude_verified）。ARM に必須。 |

#### tof_silence（`tof_silence.csv`）

由来: ToFSilence (0x4E) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ（統合パケット最終サンプルの IMU 時刻）。 |
| `vertical_velocity_up` | float | m/s | 加速度計の積分に基づく鉛直速度の推定（上が正。ControllerStatus.vertical_velocity_up）。ToF なし降下の速度ループが使う値で、ESKF の速度ではない。制御器が最後に発行した値で、disarm 中は更新されない（flight_phase の flight_state と合わせて読む）。 |
| `vertical_rise_m` | float | m | 機体が地上で最後に静止してからの上昇量の推定（ControllerStatus.vertical_rise_m。静止中は 0）。加速度の積分なのでオフセット誤差で時間とともにずれる。制御器が最後に発行した値で、disarm 中は更新されない。 |
| `descent_thrust_ratio` | float | - | ToF なし降下の指令推力 ÷ 支持推力（ControllerStatus.blind_thrust_ratio）。0 は ToF なし降下をしていない。上限は 0.95（機体を支える推力より小さく、上昇できない）。 |
| `tof_silent` | int | 0/1 | 下向き ToF の出力停止のエピソード中（FailsafeStatus.tof_silent）。新サンプルが 0.3 s 来ない、飛行中に無効サンプルだけが続く、または推定器がまだ ToF を取り込み直していない間 1。 |
| `tof_reason` | int | enum | 出力停止の理由（ToFSilenceReason、FailsafeStatus.tof_reason）。0=None, 1=NoSample（新サンプルが来ない）, 2=InvalidStuck（飛行中に無効サンプルだけ）, 3=Reacquiring（サンプルは戻ったが推定器が取り込み直していない）, 4=StatusStale（failsafe_status 自体が更新されない。読む側が付ける）。tof_silent=0 のときは 0。 |
| `tof_altitude_reliable` | int | 0/1 | ToF の高度を信用できる（ToFHealth.altitude_reliable）: 推定器が小さなイノベーションでサンプルを連続して採用し、ToF が一定時間生存している。 |
| `tof_invalid_stuck` | int | 0/1 | 飛行中に無効サンプルだけが続いている（ToFHealth.invalid_stuck）。地上の無効は正常なので 0。 |
| `tof_status_stale` | int | 0/1 | failsafe_status 自体が更新されていない（表示用の判定。1 のとき読む側は ToF を出力停止と扱う）。 |
| `mode_block` | int | enum | 直近のフライトモード変更要求が拒否された理由（ModeBlock、SystemMode.mode_block）。0=None（拒否中の要求なし）, 1=TofSilent（地上で ALT_HOLD/POS_HOLD を拒否。要求は持続し、エピソードが終わると自動で適用される）, 2=TofSwitchConsumed（飛行中のエピソードで ALT_HOLD/POS_HOLD を拒否。スイッチのエッジは消費され、パイロットが入れ直す必要がある）。 |

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
| `thrust_excitation_state` | int | bitfield | 推力励振の状態。下位 3 ビット（値 & 7）が状態（0 なし・1 待機・2 立ち上げ・3 実行・4 立ち下げ）、ビット 3〜4 （値 >> 3）が波形（1 多重正弦波・2 階段）。励振が動いていない間と、旧ファームと DISARM 中は 0。励振の信号そのものは記録せず、 `protocol/spec/thrust_excitation.yaml` の表から PC が再生成する（実行の始まりは `ctrl_output` の推力の応答に合わせる）。 |
| `thrust_excitation_amplitude` | int | 1/1000 of the hover thrust | 受理した推力励振の振幅（ホバー推力の千分率。10 % なら 100）。励振が動いていない間は 0。 |

#### baro（`baro.csv`）

由来: Baro (0x45) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。 |
| `altitude` | float | m | BMP280 の絶対の気圧高度 44330·(1 − (p/101325)^0.1903)（起動時気圧に対する相対値ではない）。天候と場所で変わる （実機ログで −50 m、−14 m、+80 m、+780 m を観測）。センサの変換は約 13 Hz で本ストリームは 50 Hz のため、同じ値が 3〜4 回繰り返す（新しい変換は気圧の変化で見分ける）。床からの高さは baro_height ストリームを使う。 |
| `pressure` | float | Pa | 気圧。本書の SI 規約どおり Pa で統一する。電文上は hPa で 送られる（data_stream_wire.hpp の WireBaro 構造体コメント 「電文上は hPa、ファーム内部は Pa」参照）ため、変換時に ×100 して記録する。 |

#### baro_height（`baro_height.csv`）

由来: BaroHeight (0x52) ／ 公称レート: 50 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ（気圧計サンプルの時刻）。 |
| `height` | float | m | 床からの気圧高度（オフセット補正済み）。valid=1 のときだけ有効。 |
| `offset` | float | m | ToF から学習したオフセット（プロペラの吹き下ろしとドリフトの分）。 |
| `valid` | int | 0/1 | 床のゼロを持つか、オフセットを学習済みで、height が有効。 |
| `learned` | int | 0/1 | 最後の ARM 以降にオフセットを学習した。 |
| `offset_age` | float | s | オフセットを最後に学習してからの経過時間（電文は 0.1 s 単位。−1 は未学習）。 |

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
| `current_ma` | float | mA | バッテリ電流。StatusPacket v3 (57B) 以降のみ送信。旧ファーム （v1/v2）を読んだ場合はこの列が空欄になる。2026-10-30 の修正 （シャント抵抗値 0.1 Ω → 0.01 Ω）より前のファームで記録した ログの値は、真の電池電流の 1/10 である。 |
| `flight_state` | int | enum | 飛行状態（FlightState、起動シーケンス〜飛行〜着陸）。 |
| `sensor_health` | int | bitmask | センサ健全性（鮮度OK）のビットマスク。bit0 = IMU, bit1 = 地磁気, bit2 = 気圧, bit3 = 下向き ToF, bit4 = オプティカルフロー, bit5 = 電源モニタ, bit6 = 前方 ToF（任意センサ。非搭載・未検出なら常に 0。旧ログには bit6 がなく 0 と読める）。 |
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

#### tof_diagnostics（`tof_diagnostics.csv`）

由来: Status v4 (0x4F, 224 B) ／ 公称レート: 1 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ステータスパケットは uptime_ms しか持たないため timestamp_us = uptime_ms * 1000。 |
| `bottom_range_status` | int | enum | 下向き ToF の最新サンプルの VL53LX range status。0 = 有効、254 = 全ターゲット無効、255 = ターゲットなし、その他はドライバのコード（たとえば 1 = シグマ不良、2 = 信号不良、4 = 範囲外）。 |
| `bottom_signal_rate` | float | Mcps | 下向き ToF の反射信号レート（メガカウント毎秒）。小さいほど反射が弱く（遠い・暗色・斜めの面）、測距が不安定になる。ターゲットなしのとき 0。 |
| `bottom_ambient_rate` | float | Mcps | 下向き ToF の環境光レート（メガカウント毎秒）。日光や強い照明で大きくなり、信号レートに対して大きいと測距が乱れる。 |
| `bottom_sigma_mm` | float | mm | 下向き ToF の距離の標準偏差の推定。大きいほど測距の不確かさが大きい。ターゲットなしのとき 0。 |
| `front_range_status` | int | enum | 前方 ToF の最新サンプルの VL53LX range status。0 = 有効、254 = 全ターゲット無効、255 = ターゲットなし、その他はドライバのコード（たとえば 1 = シグマ不良、2 = 信号不良、4 = 範囲外）。前方センサが無いときは 0 のまま（有効の意味ではない。sensor_health の bit6 で有無を判断する）。 |
| `front_signal_rate` | float | Mcps | 前方 ToF の反射信号レート（メガカウント毎秒）。小さいほど反射が弱く（遠い・暗色・斜めの面）、測距が不安定になる。ターゲットなしのとき 0。 |
| `front_ambient_rate` | float | Mcps | 前方 ToF の環境光レート（メガカウント毎秒）。日光や強い照明で大きくなり、信号レートに対して大きいと測距が乱れる。 |
| `front_sigma_mm` | float | mm | 前方 ToF の距離の標準偏差の推定。大きいほど測距の不確かさが大きい。ターゲットなしのとき 0。 |

#### loop_timing（`loop_timing.csv`）

由来: Status v4 (0x4F, 224 B) ／ 公称レート: 1 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ステータスパケットは uptime_ms しか持たないため timestamp_us = uptime_ms * 1000。 |
| `imu_cycles` | int | count | ImuTask: 数えた周期の実行開始の回数。 |
| `imu_period_max_us` | int | us | ImuTask: 連続する 2 回の周期の実行開始の間隔の最大。400 Hz の周期は 2500 us で、約 5000 us なら 400 Hz のティックを 1 つ取りこぼした。 |
| `imu_late_count` | int | count | ImuTask: 間隔が 1.5 周期（3750 us）を超えた回数（ティックの取りこぼし）。 |
| `imu_busy_max_us` | int | us | ImuTask: 実行開始から次の待ちまでの最大（上位タスクに割り込まれた時間を含む）。 |
| `imu_over_budget_count` | int | count | ImuTask: 処理が 1 周期（2500 us）を超えた周期の数。 |
| `imu_last_late_us` | int | us | ImuTask: 最後に実行開始が遅れた周期の時刻（機体起動基準。0 = まだ無い）。 |
| `imu_armed_cycles` | int | count | ImuTask: ARM 中に始まった周期の数。 |
| `imu_armed_period_max_us` | int | us | ImuTask: ARM 中の実行開始の間隔の最大。 |
| `imu_armed_late_count` | int | count | ImuTask: ARM 中に遅れた回数。 |
| `imu_armed_busy_max_us` | int | us | ImuTask: ARM 中の処理時間の最大。 |
| `imu_armed_over_budget_count` | int | count | ImuTask: ARM 中に処理が 1 周期を超えた周期の数。 |
| `control_cycles` | int | count | ControlTask: 数えた周期の実行開始の回数。 |
| `control_period_max_us` | int | us | ControlTask: 連続する 2 回の周期の実行開始の間隔の最大。400 Hz の周期は 2500 us で、約 5000 us なら 400 Hz のティックを 1 つ取りこぼした。 |
| `control_late_count` | int | count | ControlTask: 間隔が 1.5 周期（3750 us）を超えた回数（ティックの取りこぼし）。 |
| `control_busy_max_us` | int | us | ControlTask: 実行開始から次の待ちまでの最大（上位タスクに割り込まれた時間を含む）。 |
| `control_over_budget_count` | int | count | ControlTask: 処理が 1 周期（2500 us）を超えた周期の数。 |
| `control_last_late_us` | int | us | ControlTask: 最後に実行開始が遅れた周期の時刻（機体起動基準。0 = まだ無い）。 |
| `control_armed_cycles` | int | count | ControlTask: ARM 中に始まった周期の数。 |
| `control_armed_period_max_us` | int | us | ControlTask: ARM 中の実行開始の間隔の最大。 |
| `control_armed_late_count` | int | count | ControlTask: ARM 中に遅れた回数。 |
| `control_armed_busy_max_us` | int | us | ControlTask: ARM 中の処理時間の最大。 |
| `control_armed_over_budget_count` | int | count | ControlTask: ARM 中に処理が 1 周期を超えた周期の数。 |

#### nonfinite（`nonfinite.csv`）

由来: Status v4 (0x4F, 224 B) ／ 公称レート: 1 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ステータスパケットは uptime_ms しか持たないため timestamp_us = uptime_ms * 1000。 |
| `checked_cycles` | int | count | 検査した ARM 中の制御周期の数。 |
| `detected_cycles` | int | count | どれかの境界に有限でない値があった周期の数。 |
| `estimate_input` | int | count | 推定値の入力（姿勢・位置・速度・角速度の 13 個）が有限でなかった周期の数。 |
| `setpoint_input` | int | count | 目標値の入力（パイロットの目標値 5 個と誘導目標 9 個）が有限でなかった周期の数。 |
| `control_reference` | int | count | 制御器の外側ループの出力（rate_ref・angle_ref の 5 個）が有限でなかった周期の数。 |
| `control_output` | int | count | 制御器の出力（推力とトルク 3 軸の 4 個）が有限でなかった周期の数。 |
| `mixer_output` | int | count | ミキサの出力（上下限の前の 4 モータの duty）が有限でなかった周期の数。 |
| `motor_command` | int | count | モータへ渡す直前の duty が有限でなかった周期の数（そのモータには 0 を渡す）。 |
| `first_us` | int | us | 最初の検出の時刻（機体起動基準。0 = まだ無い）。 |
| `first_stage` | int | enum | 最初の検出の境界（0 = estimate_input、1 = setpoint_input、2 = control_reference、3 = control_output、4 = mixer_output、5 = motor_command、255 = まだ無い）。 |

#### link_health（`link_health.csv`）

由来: Status v4 (0x4F, 224 B) ／ 公称レート: 1 Hz ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 機体起動基準のマイクロ秒タイムスタンプ。ステータスパケットは uptime_ms しか持たないため timestamp_us = uptime_ms * 1000。 |
| `log_stream_overflow` | int | count | log_stream リングが落とした 400 Hz サンプル数（Data Stream が間に合って取り出せなかった）。0 でなければ 400 Hz ストリームに seq の飛びがある。 |
| `send_drops` | int | count | sendto の失敗数（機体の送信側の数え方。PC は通し番号の飛びとして欠落を見る）。 |
| `entry_drops` | int | count | 統合パケットが満杯で棄却したエントリ数。0 でなければ一部のストリームにサンプルが抜けている。 |
| `text_dropped_lines` | int | count | テキストログ用リングが断った行数の合計（正確。text_log の dropped_before の総和より大きいことがある: 1 行が運べる数は 255 まで）。 |

#### text_log（`text_log.csv`）

由来: TextLog (0x51) ／ 公称レート: (可変/事象駆動) ／ 必須: 任意

| 列名 | 型 | 単位 | 説明 |
|---|---|---|---|
| `timestamp_us` | int | us | 行がログシンクに入った機体起動基準のマイクロ秒時刻（他のストリームと同じ時計。32 ビットで巻き戻る）。 行の順序はパケットの通し番号とパケット内の順序で復元済みで、時刻は同値の連続を含みうる。 |
| `truncated` | int | 0/1 | 1 は行が機体で 1 行の上限（384 バイト）に切り詰められたこと（切った行も改行で終わる）。 |
| `dropped_before` | int | count | この行の直前に機体のログシンクが断った（満杯で入れなかった）行数。0 以外のとき、その位置で行が抜けている。255 で飽和する（正確な合計は link_health.text_dropped_lines）。 |
| `text` | str | text | ESP_LOG の 1 行（末尾の改行を除く。「I (12345) Tag: message」の形。括弧内は機体のミリ秒）。 UTF-8 として読めないバイトは置換文字になる。 |

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
| `flight_phase` | `flight_phase.csv` | FlightPhase400 (0x4C) | 400 Hz | - |
| `flight_flags` | `flight_flags.csv` | FlightFlags (0x4D) | 50 Hz | - |
| `tof_silence` | `tof_silence.csv` | ToFSilence (0x4E) | 50 Hz | - |
| `pilot` | `pilot.csv` | Control (0x42) | 50 Hz | - |
| `ctrl_ref` | `ctrl_ref.csv` | CtrlRef (0x48) | 50 Hz | - |
| `baro` | `baro.csv` | Baro (0x45) | 50 Hz | - |
| `baro_height` | `baro_height.csv` | BaroHeight (0x52) | 50 Hz | - |
| `tof_bottom` | `tof_bottom.csv` | ToF bottom (0x44) | 30 Hz | - |
| `tof_front` | `tof_front.csv` | ToF front (0x47) | 30 Hz | - |
| `flow` | `flow.csv` | Flow (0x43) | 100 Hz | - |
| `mag` | `mag.csv` | Mag (0x46) | 25 Hz | - |
| `status` | `status.csv` | Status (0x4F) | 1 Hz | - |
| `tof_diagnostics` | `tof_diagnostics.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `loop_timing` | `loop_timing.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `nonfinite` | `nonfinite.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `link_health` | `link_health.csv` | Status v4 (0x4F, 224 B) | 1 Hz | - |
| `text_log` | `text_log.csv` | TextLog (0x51) | (variable / event-driven) | - |
| `eskf_cov` | `eskf_cov.csv` | ESKF P-diag (0x49) | (variable / event-driven) | - |
| `truth` | `truth.csv` | sils/sim | (variable / event-driven) | - |
| `events` | `events.csv` | sils | (variable / event-driven) | - |

### Required streams per source

Streams `sf log check` requires depending on `meta.json`'s `source` (the "Required" column above is the vehicle default).

| Source | Required streams |
|---|---|
| vehicle | `imu` |
| sils | `imu`, `truth` |
| sim | `truth` |

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

#### flight_phase (`flight_phase.csv`)

Source: FlightPhase400 (0x4C) / Nominal rate: 400 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. Same packet as imu.csv, so equal to the imu.csv row with the same `seq` (may repeat; identify rows by `seq`). |
| `seq` | int | n/a | Control-cycle sequential number. For a real vehicle capture: the unified packet (0x50) header sequence (16-bit, unwrapped by the capture tool) x 8 + the in-packet sub-index. For SILS: the control-cycle counter. When converted from legacy JSONL: the imu stream's row number, matched by (timestamp_us, occurrence order within that timestamp). Empty when no match is found. |
| `flight_state` | int | enum | Flight state (FlightState) at 400 Hz. status.csv has the same column at only 1 Hz -- read state-transition times from this stream. |
| `flip_phase` | int | enum | Flip-sequencer phase. 0=Idle (not engaged), 1=Boost, 2=Spin, 3=Brake, 4=Recover, 5=Done. |
| `flip_result` | int | enum | Flip outcome (FlipResult). 0=None, 1=Ok, 2=AbortedSpinTimeout, 3=AbortedGyroLimit, 4=AbortedRecoverTimeout. Holds the last result until the next flip starts (0 while disarmed). |
| `flip_phi` | float | rad | Rotation angle accumulated since the flip started (integral of omega dt, positive along the flip direction, clamped at 0). 0 when Idle. Sent as int16 rad x 1000 (0.001 rad resolution). |

#### flight_flags (`flight_flags.csv`)

Source: FlightFlags (0x4D) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp (IMU time of the unified packet's last sample). |
| `pilot_arm` | int | 0/1 | Transmitter ARM switch (control flags bit0). |
| `pilot_flip` | int | 0/1 | Transmitter FLIP button (control flags bit1). |
| `flip_ready` | int | 0/1 | Controller flip execution conditions C2-C8 hold (ControllerStatus.flip_ready). The last value the controller published; not updated while disarmed (read together with flight_phase.flight_state). |
| `flip_block_reason` | int | enum | Why a flip is blocked (FlipBlockReason). 0=None, 1=NotFlying, 2=TooLow, 3=NotSteady, 4=BatteryLow, 5=EstimatorUnhealthy, 6=Cooldown, 7=Busy, 8=SourceConflict, 9=Disabled (flip.enable=0). Meaningful only when flip_ready=0. |
| `arm_block` | int | enum | Pre-arm gate block reason (ArmBlock, SystemMode.arm_block). 0=None (gates pass), 1=Pairing, 2=Battery, 3=Calibrating, 4=BenchRelevel, 5=TiltPending, 6=TiltMismatch, 7=TofSilent (a downward-ToF silence episode: no new samples, or the estimator has not re-acquired the ToF yet; ARM is refused in every mode, ACRO included). |
| `attitude_mismatch` | int | 0/1 | On the ground the estimated attitude disagrees with gravity (SystemStatus.attitude_mismatch). |
| `attitude_verified` | int | 0/1 | On the ground the attitude check has judged and agrees (SystemStatus.attitude_verified). Required for ARM. |

#### tof_silence (`tof_silence.csv`)

Source: ToFSilence (0x4E) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp (IMU time of the unified packet's last sample). |
| `vertical_velocity_up` | float | m/s | Accelerometer-integral vertical velocity estimate, up positive (ControllerStatus.vertical_velocity_up). The value the velocity loop of the descent without a ToF uses, not the ESKF velocity. The last value the controller published; not updated while disarmed (read together with flight_phase.flight_state). |
| `vertical_rise_m` | float | m | Estimated rise since the craft last rested on the ground (ControllerStatus.vertical_rise_m; 0 at rest). An accelerometer integral, so an offset error makes it drift with time. The last value the controller published; not updated while disarmed. |
| `descent_thrust_ratio` | float | - | Commanded thrust / support thrust of the descent without a ToF (ControllerStatus.blind_thrust_ratio). 0 = no such descent. The ceiling is 0.95 (below the thrust that holds the craft, so it cannot climb). |
| `tof_silent` | int | 0/1 | A downward-ToF silence episode is in progress (FailsafeStatus.tof_silent): no new sample for 0.3 s, only invalid samples while flying, or the estimator has not re-acquired the ToF yet. |
| `tof_reason` | int | enum | Why the ToF is silent (ToFSilenceReason, FailsafeStatus.tof_reason). 0=None, 1=NoSample, 2=InvalidStuck (only invalid samples while flying), 3=Reacquiring (samples are back, the estimator has not re-acquired the ToF), 4=StatusStale (failsafe_status itself is not refreshed; set by the reader). 0 while tof_silent=0. |
| `tof_altitude_reliable` | int | 0/1 | The ToF altitude can be trusted (ToFHealth.altitude_reliable): the estimator adopted samples in a row with a small innovation and the ToF has been alive for a set time. |
| `tof_invalid_stuck` | int | 0/1 | Flying, and only invalid samples for a set time (ToFHealth.invalid_stuck). 0 on the ground, where invalid is normal. |
| `tof_status_stale` | int | 0/1 | failsafe_status itself is no longer refreshed (a display test; a reader treats the ToF as silent while it is 1). |
| `mode_block` | int | enum | Why the latest flight-mode change request was refused (ModeBlock, SystemMode.mode_block). 0=None (no refused request pending), 1=TofSilent (ALT_HOLD/POS_HOLD refused on the ground; the request persists and applies by itself when the episode ends), 2=TofSwitchConsumed (refused in flight during an episode; the switch edge is consumed and the pilot must flip the switch again). |

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
| `thrust_excitation_state` | int | bitfield | State of the thrust excitation. The low 3 bits (value & 7) are the state (0 none, 1 standby, 2 ramp-up, 3 running, 4 ramp-down) and bits 3 to 4 (value >> 3) the waveform (1 multisine, 2 steps). 0 while no excitation runs, on older firmware and while disarmed. The signal itself is not logged: the PC regenerates it from the table of `protocol/spec/thrust_excitation.yaml` (the start of a run is aligned with the response of the `ctrl_output` thrust). |
| `thrust_excitation_amplitude` | int | 1/1000 of the hover thrust | Amplitude of the accepted thrust excitation in 1/1000 of the hover thrust (100 for 10 %). 0 while no excitation runs. |

#### baro (`baro.csv`)

Source: Baro (0x45) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. |
| `altitude` | float | m | ABSOLUTE pressure altitude of the BMP280, 44330·(1 − (p/101325)^0.1903) (not relative to the power-on pressure). It depends on weather and place (real logs read −50 m, −14 m, +80 m, +780 m). The sensor converts at about 13 Hz while this stream is 50 Hz, so each value repeats 3-4 times (a new conversion is recognised by a change of pressure). For the height above the floor use the baro_height stream. |
| `pressure` | float | Pa | Pressure, unified to Pa per this document's SI convention. The wire sends hPa (see the WireBaro struct comment in data_stream_wire.hpp: "hPa on the wire, Pa inside the firmware"), so the conversion multiplies by 100 before recording. |

#### baro_height (`baro_height.csv`)

Source: BaroHeight (0x52) / Nominal rate: 50 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp (time of the barometer sample). |
| `height` | float | m | Barometric height above the floor, offset-corrected; valid only when `valid`=1. |
| `offset` | float | m | Offset learned from the ToF (the propeller-wash term and the drift). |
| `valid` | int | 0/1 | A floor zero is held or an offset is learned, so `height` is valid. |
| `learned` | int | 0/1 | The offset was learned since the last ARM. |
| `offset_age` | float | s | Time since the offset was last learned (0.1 s units on the wire; -1 = never learned). |

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
| `current_ma` | float | mA | Battery current. Sent only by StatusPacket v3 (57B) and later; empty when the source firmware only sent v1/v2. Logs recorded by firmware before the 2026-10-30 fix (shunt resistor 0.1 ohm -> 0.01 ohm) contain 1/10 of the true battery current. |
| `flight_state` | int | enum | Flight state (FlightState enum: boot sequence, flight, landing, ...). |
| `sensor_health` | int | bitmask | Sensor health (fresh-data) bitmask. bit0 = IMU, bit1 = magnetometer, bit2 = barometer, bit3 = bottom ToF, bit4 = optical flow, bit5 = power monitor, bit6 = front ToF (optional; always 0 when absent or not detected; older logs have no bit6 and read as 0). |
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

#### tof_diagnostics (`tof_diagnostics.csv`)

Source: Status v4 (0x4F, 224 B) / Nominal rate: 1 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. The status packet only carries uptime_ms, so timestamp_us = uptime_ms * 1000. |
| `bottom_range_status` | int | enum | VL53LX range status of the latest downward ToF sample. 0 = valid, 254 = every target invalid, 255 = no target, others are the driver's codes (for example 1 = sigma failure, 2 = signal failure, 4 = out of range). |
| `bottom_signal_rate` | float | Mcps | Return signal rate of the downward ToF in megacounts per second. The smaller it is, the weaker the reflection (far, dark or slanted surface) and the less stable the range. 0 when there is no target. |
| `bottom_ambient_rate` | float | Mcps | Ambient light rate of the downward ToF in megacounts per second. Grows in sunlight or strong lighting; a large value against the signal rate disturbs the range. |
| `bottom_sigma_mm` | float | mm | Estimated standard deviation of the downward ToF range. The larger, the more uncertain the range. 0 when there is no target. |
| `front_range_status` | int | enum | VL53LX range status of the latest front ToF sample. 0 = valid, 254 = every target invalid, 255 = no target, others are the driver's codes (for example 1 = sigma failure, 2 = signal failure, 4 = out of range). Stays 0 when the front sensor is absent (that does not mean valid; judge presence by sensor_health bit6). |
| `front_signal_rate` | float | Mcps | Return signal rate of the front ToF in megacounts per second. The smaller it is, the weaker the reflection (far, dark or slanted surface) and the less stable the range. 0 when there is no target. |
| `front_ambient_rate` | float | Mcps | Ambient light rate of the front ToF in megacounts per second. Grows in sunlight or strong lighting; a large value against the signal rate disturbs the range. |
| `front_sigma_mm` | float | mm | Estimated standard deviation of the front ToF range. The larger, the more uncertain the range. 0 when there is no target. |

#### loop_timing (`loop_timing.csv`)

Source: Status v4 (0x4F, 224 B) / Nominal rate: 1 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. The status packet only carries uptime_ms, so timestamp_us = uptime_ms * 1000. |
| `imu_cycles` | int | count | ImuTask: Wakes counted. |
| `imu_period_max_us` | int | us | ImuTask: Longest interval between two wakes. The 400 Hz period is 2500 us; about 5000 us means one 400 Hz tick was missed. |
| `imu_late_count` | int | count | ImuTask: Intervals longer than 1.5 periods (3750 us), i.e. missed ticks. |
| `imu_busy_max_us` | int | us | ImuTask: Longest time from a wake to the next wait (preemption by higher-priority tasks included). |
| `imu_over_budget_count` | int | count | ImuTask: Cycles whose busy time exceeded one period (2500 us). |
| `imu_last_late_us` | int | us | ImuTask: Time of the latest late wake (vehicle boot-relative; 0 = none yet). |
| `imu_armed_cycles` | int | count | ImuTask: Cycles that began while armed. |
| `imu_armed_period_max_us` | int | us | ImuTask: Longest wake interval while armed. |
| `imu_armed_late_count` | int | count | ImuTask: Late wakes while armed. |
| `imu_armed_busy_max_us` | int | us | ImuTask: Longest busy time while armed. |
| `imu_armed_over_budget_count` | int | count | ImuTask: Over-budget cycles while armed. |
| `control_cycles` | int | count | ControlTask: Wakes counted. |
| `control_period_max_us` | int | us | ControlTask: Longest interval between two wakes. The 400 Hz period is 2500 us; about 5000 us means one 400 Hz tick was missed. |
| `control_late_count` | int | count | ControlTask: Intervals longer than 1.5 periods (3750 us), i.e. missed ticks. |
| `control_busy_max_us` | int | us | ControlTask: Longest time from a wake to the next wait (preemption by higher-priority tasks included). |
| `control_over_budget_count` | int | count | ControlTask: Cycles whose busy time exceeded one period (2500 us). |
| `control_last_late_us` | int | us | ControlTask: Time of the latest late wake (vehicle boot-relative; 0 = none yet). |
| `control_armed_cycles` | int | count | ControlTask: Cycles that began while armed. |
| `control_armed_period_max_us` | int | us | ControlTask: Longest wake interval while armed. |
| `control_armed_late_count` | int | count | ControlTask: Late wakes while armed. |
| `control_armed_busy_max_us` | int | us | ControlTask: Longest busy time while armed. |
| `control_armed_over_budget_count` | int | count | ControlTask: Over-budget cycles while armed. |

#### nonfinite (`nonfinite.csv`)

Source: Status v4 (0x4F, 224 B) / Nominal rate: 1 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. The status packet only carries uptime_ms, so timestamp_us = uptime_ms * 1000. |
| `checked_cycles` | int | count | Armed control cycles checked. |
| `detected_cycles` | int | count | Cycles in which any boundary held a non-finite value. |
| `estimate_input` | int | count | Cycles in which the estimate input (13 values) was not finite. |
| `setpoint_input` | int | count | Cycles in which the setpoint input (5 pilot setpoint values and 9 guidance target values) was not finite. |
| `control_reference` | int | count | Cycles in which the outer-loop output (rate_ref, angle_ref: 5 values) was not finite. |
| `control_output` | int | count | Cycles in which the controller output (thrust and 3 torques: 4 values) was not finite. |
| `mixer_output` | int | count | Cycles in which the mixer output (4 duties before the limits) was not finite. |
| `motor_command` | int | count | Cycles in which a duty about to go to a motor was not finite (that motor gets 0). |
| `first_us` | int | us | Time of the first detection (vehicle boot-relative; 0 = none yet). |
| `first_stage` | int | enum | Boundary of the first detection (0 = estimate_input, 1 = setpoint_input, 2 = control_reference, 3 = control_output, 4 = mixer_output, 5 = motor_command, 255 = none yet). |

#### link_health (`link_health.csv`)

Source: Status v4 (0x4F, 224 B) / Nominal rate: 1 Hz / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond timestamp. The status packet only carries uptime_ms, so timestamp_us = uptime_ms * 1000. |
| `log_stream_overflow` | int | count | 400 Hz samples the log_stream ring dropped (the Data Stream did not drain it in time). When not 0 the 400 Hz streams have seq gaps. |
| `send_drops` | int | count | sendto failures counted on the vehicle (the PC sees the loss as a sequence jump). |
| `entry_drops` | int | count | Entries the unified packet refused because it was full. When not 0 some stream is missing samples. |
| `text_dropped_lines` | int | count | Total of text lines the text-log ring refused (exact; can exceed the sum of text_log.dropped_before, which one line carries up to 255). |

#### text_log (`text_log.csv`)

Source: TextLog (0x51) / Nominal rate: (variable / event-driven) / Required: no

| Column | Type | Unit | Description |
|---|---|---|---|
| `timestamp_us` | int | us | Vehicle boot-relative microsecond time at which the line entered the log sink (the same clock as the other streams; 32 bits, wraps). The order of the rows is restored from the packet sequence and the order inside the packet; consecutive equal times are possible. |
| `truncated` | int | 0/1 | 1 when the vehicle cut the line to its limit of 384 bytes (a cut line still ends with a newline). |
| `dropped_before` | int | count | Lines the vehicle's log sink refused (ring full) just before this line. When not 0 lines are missing at this place. Saturates at 255 (the exact total is link_health.text_dropped_lines). |
| `text` | str | text | One ESP_LOG line without its trailing newline, as "I (12345) Tag: message" (the number in parentheses is vehicle milliseconds). Bytes that are not UTF-8 become replacement characters. |

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
