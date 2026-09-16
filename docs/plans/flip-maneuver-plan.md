# 宙返り（Flip）マニューバ実装計画

作成: 2026-09-16。最終更新: 2026-09-16。
状態: **計画中**（設計案の提示段階。オリジナルファームとの比較・数値シミュレーション結果は §6・§7 に追記予定）。

発端: Tello 互換 UDP API（`firmware/vehicle/tasks/api_task.cpp`、ポート 8889）は
`command`/`takeoff`/`land`/移動/回頭/`rc`/クエリまで実装済みだが、Tello SDK の
`flip l/r/f/b`（宙返り）だけは「小型機で高リスク」として明示的に拒否している
（`api_task.cpp:991-996`、2026-06-23 の判断）。オーナーは 2026-09-16 にこの判断を撤回し、
Tello 互換を完成させる第一歩として **vehicle ファームに Flip を実装する** と決めた。
工場出荷ファーム（`M5Fly-kanazawa/StampFly`、オーナー自身の実装）には Flip が実装されて
おり、本計画では **著者（Claude）の設計案を先に提示し、その後にオリジナル実装を読んで
比較分析する** 手順をとる（比較の独立性を保つため）。

## 0. 要旨

| 観点 | 内容 |
|------|------|
| 目的 | (1) Tello SDK `flip x`（x = l/r/f/b）を vehicle ファームで実行できるようにする。(2) コントローラの FLIP ボタン（ESP-NOW `CTRL_FLAG_FLIP`、`firmware/common/protocol/include/espnow_protocol.hpp:61`。機体側は未使用）を有効にする |
| 前提の変更 | 2026-06-23 の「flip は拒否」判断を撤回（オーナー指示 2026-09-16）。`requirements.md:52`「シーケンス（FLIP, AUTO_LAND 等）は将来拡張可能」の当初構想に戻る |
| 方針（案 A） | **角度スケジュール型**: 1 軸のレート制御だけで 360° 回し、集合推力を回転角に応じて「高→低→高」と切り替え、回転角が約 290° を過ぎてから姿勢制御に戻す。既存の「単一の姿勢＋レートパイプライン」の中で **有限時間（0.6 s 以内）だけ設定点の出所を切り替える** 実装（レートループ同定励振 `excite_active_` と同じ型） |
| 主要な障壁 | (a) 角速度異常フェイルセーフ（800 °/s で即 DISARM）、(b) 姿勢ループの ZYX オイラー角依存（大角度で破綻）、(c) 反転中の ToF・光学フロー観測の扱い、(d) SILS のジャイロレンジ飽和が未実装 |
| 検証 | 平面 2 自由度モデルでパラメータを決める → SILS シナリオ `api_flip`（4 方向＋摂動族）→ 実機（ネット・高天井・段階的） |
| 見積り | Phase 0〜5（§8）。実装本体（Phase 2）は 3〜4 日、実機検証込みで 1〜2 週間 |

## 1. 何を作るか（要件）

| # | 要件 | 備考 |
|---|------|------|
| F1 | `flip l` / `flip r` / `flip f` / `flip b` を受理し、宙返り完了後に `ok` を返す。実行条件を満たさないときは `error <理由>` を返す | Tello SDK 2.0 の応答仕様（`ok`/`error`）に合わせる |
| F2 | 宙返りは 1 軸（ロールまたはピッチ）まわりの 360° 回転。開始と終了はホバリング | 方向: l = 左ロール、r = 右ロール、f = 前方（機首下げ）、b = 後方（機首上げ） |
| F3 | 高度損失は見積り値以内（数値シミュレーションで確定、暫定 0.5 m）。終了後 1 s 以内に姿勢 ±10°、高度制御が復帰 | 合格基準の数値は §8 |
| F4 | 実行条件（高度・姿勢・速度・電池電圧・推定器の健全性・前回からの間隔）を満たさなければ開始しない | §3.1 |
| F5 | 宙返り中に想定外の事態（回転が進まない・角速度がジャイロ計測範囲に近い・衝撃検出）が起きたら回復フェーズへ打ち切る。緊急停止は常に最優先 | §3.5 |
| F6 | コントローラの FLIP ボタン押下でも同じマニューバを起動できる（方向はスティック位置で決める、既定値は §9 の未決事項） | Phase 3 |
| F7 | SILS で 4 方向すべてを自動判定できる（新メトリクス） | Phase 1 |

## 2. 現状の事実（2026-09-16 調査）

Flip の設計に効く現行実装の事実。行番号は調査時点。

| 項目 | 事実 | 場所 |
|------|------|------|
| API の拒否 | `flip` は `"error flip not supported on StampFly"` を返す | `firmware/vehicle/tasks/api_task.cpp:991-996` |
| API の完了待ち | `waitUntil(timeout_ms, pred)` で 50 ms 周期に完了条件を確認してから `ok`。多段シーケンスの例は `cmdAutotune`（`sysid_result.latest().seq` の変化を待つ） | `api_task.cpp:236-247`, `:659-684` |
| API のコマンド型 | `ApiCmd` は Arm/Disarm/Takeoff/Land/Emergency の 5 値。誘導系は `GuidanceTarget`（位置＋ヨー／速度） | `sf_types/.../data_types.hpp:658-665`, `:721-738` |
| 状態機械 | `FlightState` = INIT/IDLE_GROUND/IDLE_HELD/ARMED_GROUND/TAKEOFF/FLYING/LANDING。FLYING のサブモード `FlightMode` = ACRO/STABILIZE/ALT_HOLD/POS_HOLD。規範表は `detailed_design.md` §3.1 | `sf_state/include/flight_state.hpp:37-49`, `:74-83` |
| 制御ループ | ControlTask 400 Hz。レート PID（最内）→ 姿勢 PID（STABILIZE 以上）→ 高度・位置カスケード。鉛直フェーズ `VerticalPhase` = Grounded/TakeoffClimb/Airborne/Landing が唯一の鉛直分岐点（INV-1） | `sf_controller_pid/pid_controller.hpp:127`, `pid_controller.cpp:229-231, :253-255, :455-456` |
| 姿勢ループの入力 | `q.to_euler()`（ZYX）の roll/pitch を毎周期使う。ACRO はこのブロックを通らない | `pid_controller.cpp:229-231, :455-456` |
| リミット | レート PID 出力上限 `max_roll_pitch_torque_` = 5.2e-3 N·m（幾何上限 ≈ 7.7e-3）、姿勢ループ出力上限 `max_att_rate_sp_` = 3.0 rad/s、`max_thrust_` = 0.672 N（4 × 0.168 N）、`hover_thrust_` = 0.407 N | `pid_controller.hpp:157-224, :594-595` |
| ミキサー | 推力[N]・トルク[N·m] → B⁻¹ 配分（`ARM_D` = 0.023 m、`KAPPA` = 4.10e-3）→ モータ曲線で duty | `sf_actuator/actuator.cpp:255-273, :225-231` |
| ジャイロ | BMI270 ±2000 dps、ODR 1600 Hz | `bmi270_wrapper.hpp:119-121` |
| 推定器 | ESKF、姿勢はクォータニオン。加速度観測は χ² 判定と適応 R（`k_adaptive` = 50）で大加速度時に重みが下がる。線形化はホバー近傍限定と文書が明記 | `detailed_design.md:360-386` |
| フェイルセーフ | 角速度 800 °/s を 1 軸でも 2 サンプル連続（400 Hz）で超えると `GYRO_ANOMALY`（CRITICAL）→ StateManager が armed 中は無条件で IDLE_GROUND（モータ停止）。衝撃 3.0 G × 2 サンプルも同様 | `sf_failsafe/failsafe.cpp:237-304`, `sf_state/state_manager.cpp:320-346` |
| 傾き上限の安全則 | 存在しない（要件 §9 の表に「傾きすぎ」の項目は無い） | `requirements.md:225-234` |
| 同定励振の前例 | レートループに 1 軸・時限（10 s 以内）・振幅制限（±1.5 rad/s）で信号を注入する状態機械。Airborne 限定、モード切替・着陸で強制終了 | `pid_controller.hpp:258-296` |
| プロトコル | `CTRL_FLAG_FLIP`（bit1）はコントローラが送信済み、機体は未使用。`protocol/spec/messages.yaml:79-81` に予約済み | `espnow_protocol.hpp:61`, `controller/main/main.cpp:1814` |
| SILS プラント | MuJoCo（RK4、4 kHz）＋自作モータ電気機械 ODE（時定数 ≈ 16 ms）。姿勢はクォータニオン。**ジャイロレンジ飽和なし**、機体の空力抗力なし。ToF は傾き 90° 超で自動的に無効 | `simulator/sils/plant/plant.hpp:337`, `plant.cpp:348-366, :810-819`, `models/stampfly.xml:86, :124-127` |
| SILS メトリクス | `tilt_max` 等は小角度前提。360° 回転の完遂・高度損失を判定するメトリクスは無い | `lib/sfcli/commands/sils.py` `_traj_metric` |
| 物理パラメータ | m = 0.037 kg、Ixx = 9.16e-6、Iyy = 1.33e-5、Izz = 2.04e-5 kg·m²、モータ物理上限 0.2 N | `control/models/stampfly_physical.yaml` → `generated_params.hpp:15-36` |

**孤立コードの発見（本計画とは別件）:** `sf flight jump/hover/takeoff/land` は TCP:23 の CLI に文字列を送るが、
現行 `cli_task.cpp:1198-1212` にこれらのコマンドは登録されていない（`vehicle_old` 削除に伴う取り残し）。
`feature_status.md:108-114` の「未移植」記述と整合。別途、削除または Tello API 経由への付け替えを判断する。

## 3. 提案アルゴリズム（案 A: 角度スケジュール型）

### 3.1 実行条件

以下をすべて満たすときだけ開始する。満たさないときは API へ `error flip: <理由>` を返し、機体は何もしない。

| # | 条件 | 暫定値 | 理由 |
|---|------|--------|------|
| C1 | `FlightState::FLYING`（TAKEOFF/LANDING 中は不可） | — | 規範表（§4.3） |
| C2 | 対地高度 h ≥ h_min | 1.2 m（シミュレーションで確定） | 高度損失（§5）＋余裕 |
| C3 | 姿勢 \|roll\|, \|pitch\| ≤ 15°、角速度 \|ω\| ≤ 60 °/s | — | 回転角の積算を 0 から始める |
| C4 | 速度 \|v_h\| ≤ 0.3 m/s、\|v_z\| ≤ 0.2 m/s | — | 回復後の位置制御の復帰を容易にする |
| C5 | 電池電圧 ≥ V_min（負荷時） | 3.6 V（シミュレーションと実機ログで確定） | 減速・回復に推力とトルクの余裕が要る |
| C6 | 推定器が正常、ToF が有効（高度の観測が生きている） | — | h_min の判定根拠 |
| C7 | 前回の Flip 終了から t_gap 以上 | 2 s | 推定器の再収束・電池の回復 |
| C8 | 同定励振・オートチューン・誘導移動（`cmdMove`/`cmdRotate`）が実行中でない | — | 設定点の出所が二重にならないようにする |

天井までの距離は機体に上向きセンサが無いため判定できない。**開始高度 + 0.5 m 以上の天井余裕は操縦者の責任**とし、操作手引きに明記する。

### 3.2 フェーズ

回転軸の単位ベクトル e_f（機体座標 FRD）と符号 s を方向から決める:

| 方向 | 軸 | レート指令の符号 | 備考 |
|------|----|-----------------|------|
| `r` | x（ロール） | p_cmd = +ω_flip | 右翼下げ方向 |
| `l` | x（ロール） | p_cmd = −ω_flip | |
| `b` | y（ピッチ） | q_cmd = +ω_flip | 機首上げから後方へ |
| `f` | y（ピッチ） | q_cmd = −ω_flip | 機首下げから前方へ |

回転角 φ は開始時刻からのジャイロ（バイアス補正後）の軸成分の積分 φ = ∫ s·ω_f dt で定義する（常に正方向に増える）。
クォータニオンから求めた相対回転角は 180° で折り返すため、積算にはジャイロ積分を使い、
姿勢制御へ戻す時点でクォータニオンとの整合を確認する。

| フェーズ | 入るとき | レート指令（3 軸） | 集合推力 T_c | 出るとき | 時間の目安 |
|---------|---------|-------------------|-------------|---------|-----------|
| P1 Boost | 実行条件成立 | 姿勢ループを通す通常制御（水平保持） | T_hi = `max_thrust_`（0.672 N） | t ≥ t_boost（0.10 s） | 0.10 s |
| P2 Spin | P1 終了 | 回転軸 = s·ω_flip、他 2 軸 = 0。**姿勢ループは通さない**（ACRO と同じ経路） | φ < φ_a: T_hi ／ φ_a ≤ φ < φ_b: T_lo ／ φ ≥ φ_b: T_hi | φ ≥ φ_brake | 0.25〜0.3 s |
| P3 Brake | φ ≥ φ_brake | 回転軸 = 0（レート PID が最大トルクで減速）、他 2 軸 = 0 | T_hi | φ ≥ 290° かつ \|ω_f\| ≤ ω_h（300 °/s）、または φ ≥ 350° | 0.05〜0.1 s |
| P4 Recover | P3 終了 | 姿勢ループに戻す（roll = pitch = 0、yaw = 開始時の値） | T_hi（上昇速度 v_z ≥ 0 まで） | v_z ≥ 0、または t_recover ≥ 0.8 s | 0.2〜0.4 s |
| 完了 | P4 終了 | 通常モードの設定点（ALT/POS: 高度目標 = 開始時の高度、位置目標 = 現在位置。ACRO/STABILIZE: スティック） | 通常 | — | — |

角度のしきい値:

- φ_a = 60°（加速が終わり推力がまだ上向きの範囲を過ぎる角度）
- φ_brake = 360° − Δφ_brake、Δφ_brake = ω_f² / (2·α_brake) + ω_f · t_lag。α_brake = k_b · τ_max / I_axis（k_b = 0.6、τ_max = 5.2e-3 N·m、t_lag = 0.016 s のモータ遅れ）。ロール（Ixx）では ω_flip = 1500 °/s で Δφ_brake ≈ 82°、ピッチ（Iyy）では ≈ 108°。**計測した ω_f から毎周期計算する**ので、電池電圧の低下で回転が遅いときは自動的に遅く減速に入る
- φ_b = φ_brake − 20°（減速の直前にトルク余裕を確保する）

推力の切り替えを角度で行う理由: 推力ベクトルの鉛直成分は cos φ に比例するため、
反転している間（90°〜270°）の推力は機体を下へ押す。加速（φ < φ_a）と減速（φ ≥ φ_b）に
必要なトルク余裕は T_c/4 と (0.168 − T_c/4) の小さい方で決まるので、その区間だけ T_hi にし、
惰性で回っている区間は T_lo まで下げて下向きの推力を減らす。

### 3.3 制御則（既存パイプラインの流用）

- P2/P3 のレート制御は既存のレート PID（`rate_roll_`/`rate_pitch_`/`rate_yaw_`）をそのまま使う。設定点だけを列生成器が与える。積分項は P2 開始時と P4 開始時にリセットする（減速時の巻き込みを防ぐ）
- P3 の減速はレート PID の飽和（`max_roll_pitch_torque_`）で行う。指令 0 と計測 ω_f の差が大きいので出力は上限に張り付き、最大トルクで減速する。姿勢角の計算には依存しない
- P4 の姿勢制御は既存の姿勢 PID。φ ≥ 290° まで待つのは、ピッチ回転で ZYX オイラー角が 270° 手前まで roll/yaw = 180° の枝にいるため（`to_euler()` の pitch は asin で ±90° に折り返し、roll/yaw が 180° ずれて表現される）。ロール回転では roll = atan2 が連続なので 180° 過ぎから使えるが、規則を 1 つにするため両軸とも 290° で統一する
- 鉛直チャネル: P1〜P4 は `VerticalPhase::Airborne` のまま、集合推力を列生成器の値で上書きする（高度カスケードは休止）。完了時に高度目標を開始時の値に戻し、高度カスケードの積分項をリセットして再開する

### 3.4 暫定パラメータ

すべて `config` 定数またはパラメータ名を持たせる（マジックナンバー禁止）。数値は §5 のシミュレーションで確定する。

| 名前 | 暫定値 | 掃引範囲 | 決め方 |
|------|--------|---------|--------|
| `flip.rate_dps` (ω_flip) | 1500 | 1000〜1800 | ジャイロ 2000 dps に対し 25 % の余裕。ピッチ軸は慣性が 1.45 倍なので別値になり得る |
| `flip.boost_ms` (t_boost) | 100 | 50〜200 | 上昇速度を付けて高度損失を減らす |
| `flip.thrust_hi_n` (T_hi) | 0.672（`max_thrust_`） | — | 既存上限 |
| `flip.thrust_lo_n` (T_lo) | 0.06 | 0〜0.2 | 反転中の下向き推力と、モータ停止（再始動遅れ）回避の折り合い |
| `flip.angle_a_deg` (φ_a) | 60 | 45〜90 | 加速終了角 |
| `flip.brake_gain` (k_b) | 0.6 | 0.5〜0.8 | 減速角の安全係数 |
| `flip.handoff_min_deg` | 290 | 280〜320 | オイラー角の枝の条件 |
| `flip.handoff_rate_dps` (ω_h) | 300 | 200〜400 | 姿勢ループへ渡すときの残り角速度 |
| `flip.min_height_m` (h_min) | 1.2 | — | 高度損失の見積り + 0.5 m |
| `flip.min_voltage_v` (V_min) | 3.6 | — | 実機ログで確定 |
| `flip.spin_timeout_ms` | 700 | — | 回転が進まないときの打ち切り |
| `flip.recover_timeout_ms` | 800 | — | v_z ≥ 0 に達しないときの打ち切り |
| `flip.gyro_abort_dps` | 1800 | — | 計測範囲（2000）への接近で打ち切り |

### 3.5 打ち切りと安全

| 事象 | 処置 |
|------|------|
| P2 で `spin_timeout_ms` 以内に φ_brake に達しない | P4 へ（水平化＋上昇） |
| \|ω\| > `gyro_abort_dps` | P3 へ（即減速） |
| 衝撃検出（IMPACT 3 G） | 既存どおり即 DISARM（宙返り中も有効） |
| API `emergency` | 既存どおり即 DISARM |
| API `stop`/`land`、モードスイッチのエッジ、誘導目標 | **宙返り完了まで保留**し、完了後に適用（0.6 s 以内。途中で止める方が危険） |
| リンク途絶 | 宙返り完了後に通常則（R16 の単一判定）に従う |
| P4 で `recover_timeout_ms` 経過 | 通常モードへ戻す。高度が h_land 未満なら着陸へ |
| 角速度異常（GYRO_ANOMALY） | **マニューバ窓（P2 開始〜P4 終了 + 200 ms）の間は記録のみで無視**（§3.7） |

### 3.6 推定器の扱い

- 姿勢: ESKF のクォータニオン伝播（ジャイロ）はそのまま。加速度観測は適応 R と χ² 判定で自動的に重みが下がるため、宙返り中は実質ジャイロ積分になる。0.5 s 程度なら誤差は小さい（ジャイロバイアスは事前に収束している）。**明示的な「加速度観測の停止」は入れない**（既存の判定に任せ、実機ログで再収束時間を確認する）
- ToF: 反転中は天井までの距離を測るので、**傾き（\|roll\| または \|pitch\|）が 45° を超える間は ToF 観測を採用しない**判定が必要。ESKF に無ければ Phase 2 で追加する（SILS プラントは 90° 超で無効にするだけなので、45°〜90° の区間は SILS でも観測が入る。ファーム側で判定する）
- 光学フロー: 傾き判定または品質判定で自動的に棄却されるはず。Phase 2 で確認
- 気圧: 継続。反転中の気圧の変動は小さいと見込む（推測、実機ログで確認）
- 完了後: 位置目標を現在位置に取り直す（宙返り中の水平位置のずれは追わない）

### 3.7 フェイルセーフとの整合

角速度異常判定（800 °/s）は「墜落・衝突の検出」が目的であり、宙返りの回転はこれに該当しない。
INV-3（検出と判断の分離）に従い、**検出器（`ImuAnomalyDetector`）は変えず、判断側（`StateManager`）が
「マニューバ窓の中の GYRO_ANOMALY は墜落ではない」と扱う**。窓の開始・終了は制御器が
`controller_status.maneuver_active` で publish し、StateManager が購読する。衝撃判定（3 G）は窓の中でも
有効のまま残す（ぶつかったときの停止手段）。代案（検出器のしきい値を窓内だけ 1900 dps に上げる）は
検出器に「マニューバ」の概念を持ち込むので採らない。

Boost 中の加速度計のノルムは推力 1.85 G ＋ 回転中心からのずれによる遠心加速度（IMU が中心から 1 cm なら 0.7 G）で
3 G に近づく可能性がある。実機ログで確認し、必要なら窓内だけ衝撃しきい値を 4 G にする（§9）。

## 4. アーキテクチャへの組み込み

### 4.1 配置

| 要素 | 置き場所 | 役割 |
|------|---------|------|
| `FlipSequencer` クラス | `firmware/vehicle/components/sf_controller_pid/flip_sequencer.{hpp,cpp}` | 実行条件の判定、フェーズ進行、回転角の積算、レート設定点・集合推力・姿勢ループの要否を毎周期返す。`start(direction)`, `update(gyro, quat, height, v_z, dt)`, `abort()`, `status()` |
| `PidController::compute()` | 既存 | 列生成器が有効なら、その出力を設定点として既存のレート PID／姿勢 PID／ミキサーに流す。既存の同定励振（`excite_active_`）と同じ差し込み位置 |
| `ControllerCmd::Flip{dir}` | `sf_types` | StateManager → 制御器への指示（Takeoff/TakeoffComplete と同じ経路） |
| `controller_status` 拡張 | `sf_types` | `maneuver_active`、`flip_seq`（完了ごとに +1）、`flip_result`（Ok/Rejected(reason)/Aborted(reason)） |
| `ApiCmd::Flip{dir}` | `sf_types` | API → StateManager |
| `cmdFlip()` | `api_task.cpp` | 事前判定（command モード・FLYING）→ `ApiCmd::Flip` 発行 → `flip_seq` の変化を `waitUntil` で待つ → `ok` / `error flip: <理由>` |
| StateManager | `sf_state` | 規範表のセル（§4.3）に従って `ControllerCmd::Flip` を発行。窓内の GYRO_ANOMALY を無視。窓内のモード切替・誘導目標を保留 |
| state_task | `tasks/state_task.cpp` | `CTRL_FLAG_FLIP` の立ち上がりで `ApiCmd::Flip` 相当の要求（Phase 3） |

列生成器を制御器コンポーネントの中に置く理由: 400 Hz のジャイロ積算と、レート PID の積分項リセット・飽和挙動に密に結びつくため。
別コンポーネント化してトピック経由にすると 1 周期（2.5 ms）の遅れと状態の二重管理が生じる。
L1 アプリフック（`sf::app::controller()`）で制御器全体を差し替えた場合、Flip はその制御器の責務になる（文書に明記）。

### 4.2 不変条件（INV）との照合

| INV | 照合結果 |
|-----|---------|
| INV-1 単一パイプライン | 姿勢 PID・レート PID・ミキサーは変更しない。列生成器は **設定点の出所** を有限時間だけ切り替えるだけで、並列の姿勢則は作らない（ACRO が姿勢ループを通らないのと同じ経路）。鉛直チャネルの上書き（T_hi/T_lo）は「フェーズが変えてよいのは鉛直チャネル」の範囲内。`VerticalPhase` は増やさず、`Airborne` 内の一時上書きとする |
| INV-2 パイロットの姿勢操縦 | **例外条項の追加が必要。** 宙返りは操縦者（API またはボタン）が明示的に起動する有限時間（≤ 0.6 s）のマニューバであり、その間の姿勢設定点は列生成器が担う。終了は単一の判定（P4 完了または打ち切り）で通常則に戻る。リンク途絶の例外と同様に `architecture.md` の INV-2 に「操縦者が起動した時限マニューバ」を明記する（§9 の未決事項 1） |
| INV-3 検出と判断 | 回転角到達・打ち切り条件の「検出」は列生成器、GYRO_ANOMALY の扱い・モード切替の保留の「判断」は StateManager。検出器（failsafe）は変更しない |
| INV-4 規範表 | 事象「Flip 要求」「マニューバ中の各事象」の列を規範表に追加してから実装する（§4.3） |
| INV-5 ミキサー入力 | 列生成器の出力は推力[N]・トルク[N·m]（レート PID 経由）。ミキサーは無変更 |

### 4.3 状態機械の規範表への追加（案）

`detailed_design.md` §3.1 に事象「Flip 要求（API/ボタン）」を追加する:

| 状態 | Flip 要求 |
|------|----------|
| INIT / IDLE_GROUND / IDLE_HELD / ARMED_GROUND / TAKEOFF / LANDING | 拒否（`error flip: not flying`） |
| FLYING（マニューバ中でない） | 制御器へ `ControllerCmd::Flip`。制御器が実行条件 C2〜C8 を判定し、不成立なら `flip_result = Rejected` |
| FLYING（マニューバ中） | 拒否（`error flip: busy`） |

マニューバ中（`maneuver_active`）の既存事象:

| 事象 | 処置 |
|------|------|
| モードスイッチのエッジ | 保留（エッジ持続、完了後に適用。TAKEOFF 中の扱いと同じ） |
| API 誘導目標（move/rotate/rc） | 保留（完了後に適用） |
| DISARM 操作・`emergency`・IMPACT | 即時適用（既存どおり） |
| GYRO_ANOMALY | 無視（記録のみ） |
| LOW_BATTERY（緊急） | 完了後に LANDING |
| リンク途絶 | 完了後に通常則 |

### 4.4 API

- `flip <l/r/f/b>` → `cmdFlip()`。事前判定は command モードと FLYING。制御器の判定結果を待って応答する
- エラー語彙: `error flip: not flying` / `error flip: busy` / `error flip: too low` / `error flip: battery low` / `error flip: not steady` / `error flip: aborted` / `error flip: timeout`
- Python SDK（`tools/stampfly_py/stampfly.py`）に `flip(direction)` を追加。djitellopy の `flip_left()` 等は無改変で通る（`flip l` を送るだけ）

### 4.5 リップル確認（前提が変わる既存箇所）

| 箇所 | 変更 |
|------|------|
| `api_task.cpp:991-996` | 拒否コードを `cmdFlip()` に置換 |
| `docs/architecture/tello-api-reference.md:89, :128, :141, :238, :277, :290` | 「非対応」→ 仕様と実行条件・エラー語彙 |
| `firmware/vehicle/docs/feature_status.md:84` | 残課題の更新 |
| `firmware/vehicle/docs/requirements.md` §7 安全要件 | 角速度異常判定の例外（マニューバ窓）と Flip の実行条件を追記 |
| `firmware/vehicle/docs/architecture.md` INV-2 | 例外条項 |
| `firmware/vehicle/docs/detailed_design.md` §3.1 | 規範表の列追加 |
| `firmware/vehicle/docs/operation_manual.md` | 天井余裕・h_min・電池の注意 |
| `sf_failsafe` | 変更なし（判断は StateManager） |
| `sf_estimator_eskf` | ToF 観測の傾き判定（無ければ追加） |
| `simulator/sils` | ジャイロレンジ飽和、新メトリクス、シナリオ |
| `lib/sfcli/commands/sils.py` | メトリクス `flip_angle_deg` / `alt_drop_max` / `flip_recovery_s` |
| `tools/stampfly_py` | `flip()` |
| `protocol/spec/messages.yaml` | 変更なし（FLIP ビットは予約済み） |

## 5. 数値シミュレーション計画

### 5.1 平面 2 自由度モデル（設計段階、Phase 0）

鉛直位置 z とロール角 φ の 2 自由度に、モータ電気機械 ODE（SILS と同式）と B⁻¹ ミキサーの飽和を入れた
モデルで §3.2 のフェーズをそのまま実装し、以下を掃引する: ω_flip、T_lo、φ_a、減速方式（レート指令 0 で減速 ／ 姿勢ループへ直接切替）、
ミキサー飽和方針（集合推力優先／トルク優先）、トルク上限（5.2e-3 N·m／幾何上限）、推力上限（0.672 N／0.8 N）、
電源電圧（3.7 V／3.5 V）、慣性（Ixx／Iyy）、t_boost。出力は所要時間・最大角速度・**最大高度損失**・回復時の高度・
姿勢残差・モータ飽和の割合・失敗の有無。

結果（追記予定）: §5.3。

### 5.2 SILS（Phase 1〜2）

- プラント改修: ジャイロ出力を ±2000 dps で飽和させる（実機 BMI270 と同じ）。加速度計も設定レンジで飽和させる
- 新メトリクス: `flip_angle_deg`（真値の機体角速度の軸成分を窓内で積分）、`alt_drop_max`（開始高度 − 最低高度）、`flip_recovery_s`（完了から姿勢 ±10°・高度誤差 0.1 m 以内に入るまで）
- シナリオ `api_flip.scn`: POS_HOLD で 1.2 m にホバリング → `api "flip r"` → `ok` を確認 → 4 方向を順に → 着陸。合格: `flip_angle_deg` が 340〜380、`alt_drop_max` ≤ 見積り + 0.1 m、完了後 1.5 s 以降の `tilt_max` < 15°、`duty_max` ≤ 1.0
- 摂動族: `--thrust-eff 0.6`、`--motor-delay 10`、`--noise n2`、`--param` で電圧低下、`--torque-authority` 低下。すべて PASS が合格
- 規律: SILS 単独での最適化はしない。実機ログ（Phase 4）でモデル一致を確認してからパラメータを確定する

### 5.3 結果

（Phase 0 の平面モデルの結果をここに追記する）

## 6. オリジナルファーム（工場出荷版）との比較

（著者案を書き終えた後にオリジナル実装を読み、同じ点・違う点・優劣を追記する）

## 7. 最終提案

（§5・§6 を踏まえて追記する）

## 8. 実装フェーズと合格基準

| Phase | 内容 | 合格基準 |
|-------|------|---------|
| 0 設計確定 | 本文書、平面モデルの掃引、オリジナルとの比較、最終提案 | 推奨パラメータと高度損失の見積りが数値で示され、オーナーが §9 の未決事項を決定 |
| 1 設計文書と SILS 準備 | INV-2 例外条項、規範表の列追加、要件 §9 追記、SILS ジャイロ飽和、新メトリクス | 既存の SILS 再確認試験が全 PASS（改修による既存動作の破壊なし） |
| 2 ファーム実装 | `FlipSequencer`、`ControllerCmd::Flip`、`controller_status` 拡張、StateManager のセル、API `cmdFlip`、ToF 傾き判定、パラメータ定義 | 単体テスト（フェーズ進行・打ち切り・角度積算）PASS。SILS `api_flip` 4 方向 PASS。摂動族 PASS。既存シナリオ全 PASS |
| 3 コントローラボタン | `CTRL_FLAG_FLIP` エッジ → Flip 要求、方向決定則 | SILS `rc_flip` PASS |
| 4 実機検証 | ネット・高天井・1 方向ずつ・h = 1.5 m から。400 Hz ログ取得。SILS とのモデル一致確認 | 4 方向 × 3 回成功。高度損失 ≤ 見積り + 0.2 m。完了後 1 s 以内に ±10°。角速度異常の誤検出なし |
| 5 文書・SDK | API 参照、feature_status、操作手引き、Python SDK `flip()`、djitellopy 動作確認、Blockly ブロック（任意） | djitellopy の `flip_*()` が無改変で `ok` を受ける |

## 9. 未決事項（オーナー判断）

| # | 事項 | 著者の推奨 |
|---|------|-----------|
| 1 | INV-2 の例外条項の文言（「操縦者が起動した時限マニューバ」） | 追加する。時限（≤ 0.6 s）と単一の終了判定を条件に書く |
| 2 | 角速度異常判定の扱い | 窓内は StateManager が無視（検出器は無変更）。衝撃判定は維持 |
| 3 | 対象モード | API は ALT_HOLD/POS_HOLD（ホバリング前提）。ボタンは全 FLYING モード（ACRO/STABILIZE では回復後の高度はスティック） |
| 4 | h_min と天井余裕 | h_min はシミュレーションの高度損失 + 0.5 m。天井は操縦者の責任として手引きに明記 |
| 5 | 電池電圧下限 | 3.6 V（負荷時）。実機ログで見直す |
| 6 | ボタン押下時の方向 | スティックが中立なら後方（`b`）、傾いていればその方向（オリジナル実装を確認してから決める） |
| 7 | 衝撃しきい値（3 G）の窓内緩和 | まず緩和せず実機ログで確認 |
