# vehicle_new コードレビュー報告 — 2026-06-13

> 本書は vehicle_new ファームウェア全体（第一者コード約 27,500 行・90 ファイル、ベンダ VL53L3CX ドライバ本体を除く）を、6 設計文書（requirements / architecture / detailed_design / coding_and_education / development_roadmap / hardware_init）の規範と照合してレビューした結果のスナップショットである。レビューは **27 ユニット（24 サブシステム ＋ 3 横断観点）の並列レビュー → 各指摘を独立した懐疑的レビュアーが実コードを読んで敵対的に検証** する 2 段方式で実施した（誤検知の排除）。

## 1. 概要

### このドキュメントについて

- **対象:** `firmware/vehicle_new/` の全コンポーネント・全タスク・コア基盤・BSP
- **方法:** マルチエージェント・レビュー（レビュー 41 候補 → 敵対的検証）。意図的な設計保留（校正 NVS 永続化保留・前方 ToF 未配線・`eskf.use_mag` 既定 off・baro 鉛直融合廃止・WebSocket 廃止・POS_HOLD 実機未検証 等）はバグから除外した。
- **総括:** **致命的（critical）な飛行安全バグはゼロ。** ESKF の核心数学（F 行列符号・Joseph 等価更新・クォータニオン誤差状態・NED `[0,0,−g]` 規約）、PID 離散化（trapezoidal 積分・bilinear 微分・D-on-M）、B⁻¹ ミキサーの符号・配分、モード調停表 §3.1、ペアリング、フェイルセーフ検出器、通信パケット解析はいずれも設計規範および飛行実績のある旧 vehicle 実装と一致しており、成熟度は総じて高い。発見された 41 件は **HIGH 1・MEDIUM 8・LOW 24・INFO 8** で、大半は非既定経路（相補フィルタ・autotune・前方 ToF 等）／コメント・ドキュメント不一致／死にコード／実機 POS_HOLD ブリングアップ前に塞ぐべきロバスト性に集中する。

### 指摘件数（敵対的検証後）

| 重要度 | 件数 | 性質 |
|--------|------|------|
| **CRITICAL** | 0 | — |
| **HIGH** | 1 | autotune のライブ適用ゲートに GM 下限が無い |
| **MEDIUM** | 8 | 安全要件§9 の地上 DISARM 欠落、飛行中 wifi NVS ストール、API のパイロット権限奪取、PWM 定数重複、ToF グリッチ、LED 輝度無効 ほか |
| **LOW** | 24 | コメント/挙動不一致、死にコード、相補/mag/flow のロバスト性、並行性の軽微競合 |
| **INFO** | 8 | ドキュメント不整合、予防的ハードニング、用途未定義 API |
| 誤検知（検証で棄却） | 3 | §6 参照 |

### 最重要 3 件（安全観点で優先確認を推奨）

1. **【MEDIUM/safety】衝撃・異常角速度の自動 DISARM が地上（ARMED_GROUND）で作動しない** — 安全要件§9 と実装の乖離。
2. **【MEDIUM/safety】飛行中（armed）に CLI `wifi ssid/pass` を実行すると NVS フラッシュ消去で 400Hz ループが >10ms 停止し墜落しうる** — `param save`/`magcal` は同じ理由で armed 拒否済みだが wifi だけ抜けている。
3. **【HIGH】onboard autotune のライブ適用ゲートにゲイン余裕（GM）下限チェックが無い** — PM は満たすが GM が薄いゲインが飛行中に無確認で適用されうる。

---

## 2. サブシステム別 現状

成熟度: ◎=高（設計適合・検証済み）／○=中／△=要整備。検証: 実=実機確認、SIL=シミュレーション回帰、host=ホスト単体テスト。

| # | サブシステム | 成熟度 | 検証 | 現状の要点 |
|---|-------------|:------:|------|-----------|
| 1 | **ESKF 状態推定** (`sf_estimator_eskf`) | ◎ | SIL+実 | 15 状態・疎構造化 predict・active_mask P 隔離・χ²ゲート・右摂動クォータニオン。核心数学は内部整合かつ SIL 座標規約と一致。`accel_att_noise=0.8`／バイアスクランプ±0.03 維持。重大バグなし |
| 2 | **相補フィルタ** (`sf_estimator_complementary`) | ○ | SIL | Mahony 型・差替可能な 2 つ目の IEstimator。ゼロ除算/NaN ガード良好。ただし baro 融合が ToF-only 方針に反して無条件に効く（既定 ESKF ゆえ休眠） |
| 3 | **PID カスケード制御** (`sf_controller_pid`) | ◎ | SIL+実 | ACRO/STAB/ALT/POS＋自動離着陸＋ヘディングホールド＋誘導。離散化・単位・符号・鉛直フェーズが規範と 1:1。致命的バグなし。指摘はコメント不一致・死にコードのみ |
| 4 | **アクチュエータ/ミキサー** (`sf_actuator`,`sf_hal_motor`) | ◎ | SIL+実 | B⁻¹ X-quad ミキサー・モータ曲線逆変換・電池電圧補償が SIL プラントと厳密逆（Model Identity）。ARM 二重ゲート・disarm リトライ・watchdog 完備 |
| 5 | **状態管理** (`sf_state`) | ◎ | SIL+実 | 唯一の遷移実行者。モード調停表§3.1 に厳密適合。ARM 前ゲート群・通信断タイマ正しい。soft-landing 未配線と pairing IDLE_HELD 許容の乖離あり |
| 6 | **状態タスク（調停）** (`state_task`) | ◎ | SIL | 表§3.1 全セルを忠実に符号化。エッジ持続・不一致再適用・接地リセット確認済み |
| 7 | **IMU タスク（推定＋クラスB）** (`imu_task`) | ◎ | SIL+実 | predict→観測 update→鉛直ハンドオフ→発行→通知の中核。SPSC 消費・校正連携・loud halt 完備。重大バグなし |
| 8 | **制御タスク** (`control_task`) | ◎ | SIL+実 | IMU 同期・通知 watchdog・motor_test 時計ラップ対策。重大バグなし |
| 9 | **フェイルセーフ＋電源** (`sf_failsafe`,`power_task`,`sf_hal_power`) | ◎ | SIL+実※ | §9 検出器（衝撃/ジャイロ/通信断/低電圧）実装、検出と判断を分離。INA3221 ch1 修正済（※電池接続確認待ち）。地上 DISARM 欠落の乖離あり |
| 10 | **離着陸＋ToF** (`sf_takeoff_landing`,`tof_task`) | ○ | SIL | 着陸鎖は SIL 検証済。ただし離陸確認タイマーが死にコード・ToF 生値にグリッチ除去なしが実機リスク |
| 11 | **通信（ESP-NOW/ペアリング）** (`sf_comm`,`sf_command`) | ◎ | SIL+実 | 14B ControlPacket 解析が送信機と 1:1。相互 MAC 学習・混信フィルタ・NVS・チャネル追従。再ペア時 MAC の軽微競合のみ |
| 12 | **Tello風API** (`api_task`,`sf_api`) | ○ | SIL | UDP テキスト解析・誘導シーク・権限分担。座標/符号正しい。誘導解除の API 非同期と emergency ゲート順の穴あり。実機未検証 |
| 13 | **キャリブレーション** (`sf_calibration`) | ◎ | SIL+実 | 静止ゲート・分散チェック・やり直し・fail-close。符号規約整合。level_offset 算出するも未使用（傾き校正で bias 汚染） |
| 14 | **オンボード自動チューン** (`sf_autotune`) | ○ | host | I/Q 掃引→Nelder-Mead 同定→PID 設計。ホストと同型。GM 下限ゲート欠落・GM 未検出時 0dB 返しが要対応。実機未検証 |
| 15 | **HAL: IMU+Flow** (`bmi270`,`pmw3901`) | ◎ | SIL+実 | LSB→物理量・軸変換（FRD）・SPI 共有保護が整合。Flow の SQUAL がドライバまでで ESKF 側ゲート未使用 |
| 16 | **HAL: 地磁気** (`bmm150`,`mag_calibration`) | ◎ | 実※ | 温度補償・軸リマップが旧 vehicle と 1:1＋NaN ガード。低品質校正でも valid=true（既定 off ゆえ休眠） |
| 17 | **HAL: 気圧+ToF** (`bmp280`,`vl53l3cx_wrapper`) | ◎ | SIL+実 | Bosch 補償式と完全一致。BMP280 が絶対標高固定基準（相補選択時に矛盾） |
| 18 | **HAL: LED/ブザー/ボタン** (`led`,`buzzer`,`button`) | ○ | 実 | LEDC/GPIO 競合なし。setColor が brightness_ 未適用（輝度コマンド無効）・blocking playTone・死蔵パターン |
| 19 | **通知** (`sf_notify`) | ○ | 実 | 優先度オーバーレイ・2 チャネル LED・最新の点滅反転は正しい。LED 輝度無効と blocking ブザーは #18 と同根 |
| 20 | **テレメトリ+Data Stream+ログ** (`sf_telemetry`,`sf_logger`) | ◎ | 実 | 旧 vehicle 電文互換・UDP 非ブロッキング・SPIFFS グレースフル。Blackbox の容量見積りコメントが 100Hz と不一致 |
| 21 | **コア基盤** (`sf_core`: params/topic/data_types) | ◎ | SIL+実 | params SSOT（手書き table[]）・Topic テンプレート・lock-free ring。overflow_count 過大計上の軽微バグ・updated() 死にAPI |
| 22 | **BSP** (`sf_board`) | ◎ | 実 | 共有 HW 唯一所有（R1）・失敗 3 分類・直列 init。PWM タイマ定数の重複定義と netif 起動順序 doc 乖離 |
| 23 | **CLI** (`cli_task`) | ◎ | 実 | esp_console レジストリ・USB+TCP:23・motor_test 安全・param save armed 拒否。wifi NVS の armed ガード欠落 |
| 24 | **小センサタスク** (flow/mag/baro/button) | ◎ | SIL+実 | 周期/優先度/コア割当が§8 と一致。BaroTask の DRDY ゲート欠落（重複発行） |
| 横 | **並行性** (横断) | ◎ | — | Latest/RingBuffer/atomic/xTaskNotify が概ね健全。重大レースなし。軽微競合（再ペア MAC・motor_test）は disarmed 限定 |
| 横 | **単位・符号・frame** (横断) | ◎ | — | IMU→推定→制御→ミキサーの端から端まで NED/FRD・rad・符号が一貫。コードは正、**規範文書側の加速度符号記述が逆**（doc 修正対象） |
| 横 | **設計適合** (横断) | ◎ | — | R1〜R16・調停表§3.1・クラスA/B・SSOT に高度に適合。例外は地上 DISARM 欠落（§9 乖離） |

---

## 3. バグ・指摘の影響度別整理と対策

各指摘は **［状態］確定=実コードで再現確認 / 一部確定=機構は本物だが影響が誇張 / 不確定=コードだけでは断定不可** を付す。重複して複数レビュアーが独立に挙げた指摘は信頼度が高い。

### 3.1 HIGH（1 件）

#### H-1. onboard autotune のライブ適用ゲートにゲイン余裕（GM）の下限が無い ［確定］
- **場所:** `tasks/api_task.cpp:480-508`、`components/sf_autotune/autotune.cpp:242-276`
- **内容:** `cmdAutotune` は FLYING 中に同定残差<0.3・達成 PM が目標±5°・Kp が現ゲインの 1/4〜4 倍・param テーブル範囲の 4 条件でゲインを**飛行中ライブ適用**するが、ゲイン余裕（`tune.gm_db`）の下限を一切チェックしない。ホスト自己テスト（`test_main.cpp:413`）は `gm_db > 6.0` を健全 GM として必須にしているのに、実機ゲートには反映されていない。
- **影響:** PM は満たすが GM が薄い（数 dB 以下）ゲインが飛行中に無確認適用され、モデル誤差・トルク効きの非線形でゲイン側不安定（発振）に至りうる。auto-memory にも実績 yaw ゲインが PM22°/GM3.8dB と薄かった記録があり、GM 支配の軸が現実に存在する。
- **対策:** ① `autotune::tunePid` が GM を有効検出できたか（位相 −180° 交差を掃引域内で見つけたか）を `gm_valid` フラグで返すよう拡張（**H-1 は L-15 と必ずセットで直す** — 下記）。② `api_task.cpp:482` の受理判定直後に `if (tune.gm_valid && tune.gm_db < kMinGmDb) { reply("error gain margin too low"); return; }` を追加。`kMinGmDb` は config 定数（既定 6.0dB、yaw は 8.0dB 等軸別）。マジックナンバー禁止規約に従い config 化。SIL/host で GM 未検出ケースが誤って弾かれないことを回帰固定。

### 3.2 MEDIUM（8 件）

#### M-1. 衝撃・異常角速度の自動 DISARM が地上（ARMED_GROUND）で作動しない ［確定・safety］
- **場所:** `components/sf_state/state_manager.cpp:296-304`
- **内容:** §9 は「衝撃 3.0G×2／異常角速度 800deg/s×2 → 自動 DISARM」を**状態無条件**で規定。検出層（`failsafe.cpp`/`imu_task.cpp`）は ARMED_GROUND でも検出してアラートを発報するのに、消費層 `handleAlert()` の IMPACT/GYRO_ANOMALY 分岐が `if (isAirborne(state_))`（=TAKEOFF/FLYING/LANDING のみ）でゲートされ、ARMED_GROUND が抜けている。検出層コメントは「armed では地上でも保護する」と明言しており、消費層だけが非対称。
- **影響:** ARM 済みで地上に置いた機体（教室で生徒が誤って ARM 後に倒す/叩く）が衝撃・暴れを検出してもモータが止まらない。アイドル回転プロペラの接触リスク。要件と実装の安全境界が食い違う。
- **対策:** `state_manager.cpp:300` のゲートを `if (isAirborne(state_))` → `if (sf::isArmed(state_))` に変更（ARMED_GROUND を含める）。`transition(IDLE_GROUND)` は ARMED_GROUND からでも DISARM 経路と同義で動く。298-299 行のコメント「Crash → immediate DISARM」が変更後に実装と一致。COMM_LOST/LOW_BATTERY を airborne 限定に保つのは正しいので変更不要。

#### M-2. 飛行中（armed）の CLI `wifi ssid/pass` が NVS フラッシュ消去で 400Hz ループを停止させる ［確定・safety］
- **場所:** `tasks/cli_task.cpp:480-497`
- **内容:** `cmd_wifi` の ssid/pass 分岐は `nvs_open→nvs_set_str→nvs_commit` を armed 判定なしで無条件実行する。`param save`（`cli_task.cpp:114-131`）と `magcal save/clear`（`mag_task.cpp:117-131`）は「NVS commit のフラッシュセクタ消去でキャッシュが止まり 400Hz IMU ループが >10ms ストール、armed では飛行中モータがゼロになる」ため armed 拒否済み。wifi だけ抜けている。TCP CLI は `esp_console_run` を本タスク内同期実行するため、飛行中に WiFi 経由で投入すると同じストール経路になる。
- **影響:** 飛行中に wifi ssid/pass を投入すると 400Hz 制御ループが >10ms 停止し、その間モータ duty がゼロ→姿勢を崩す/墜落しうる（`param save` で実機ウォッチドッグ発火が確認済みの事象を再現）。
- **対策:** ssid/pass 書込み分岐の先頭（`cli_task.cpp:482` の nvs_open より前）に姉妹コマンドと同一の armed ガードを 1 行追加。`wifi mode` 分岐は RAM のみ変更で NVS commit しないためガード不要。

#### M-3. API の誘導解除がパイロット介入と同期せず、stop/move/rotate がパイロット操作を黙って奪い返す ［確定・safety］
- **場所:** `tasks/api_task.cpp:336,348`、`components/sf_controller_pid/pid_controller.cpp:149,732`
- **内容:** 制御器はパイロットのスティック動作（`pid_controller.cpp:142-153`「pilot always wins」）やモード切替で自発的に `guidance_active_=false` にし現在位置保持へ戻すが、API 側の `g_target_valid` はこの解除を一切通知されない。その後 `cmdStop/cmdMove/cmdRotate` が呼ばれると `g_target_valid==true` のまま `publishGuidance()` を再送し、制御器で再び `guidance_active_=true` が立つ。
- **影響:** 誘導中にパイロットが介入して制御を取り戻した直後、自動デモが次の move/rotate を投げると機体が再び API 目標へ走り出す。パイロット優先（architecture §2）の安全前提を破る。※ cmdStop の再奪取は現在位置の再保持にとどまり実害小、危険なのは cmdMove/cmdRotate の新目標追従。
- **対策:** 制御器が誘導を自発解除した事実（`guidance_active_` の true→false）を `system_status` 等の Latest トピックに 1bit publish し、ApiTask が立下りエッジで `g_target_valid=false` を落とす。その上で move/stop/rotate を `g_target_valid==false` なら拒否し、再開には明示 takeoff/go を要求。`cmdStop` にも欠けている `g_target_valid` ガードを追加。

#### M-4. LED 輝度コマンド（`UiCmd::LedBrightness`）が実機で完全に無効 ［確定］（notify ＋ hal_io が独立に検出）
- **場所:** `components/sf_hal_led/led.cpp:86-98`、`components/sf_notify/notify.cpp:288-289,383-391`
- **内容:** `LED::setColor()` は生 RGB をそのまま `led_strip_set_pixel` に渡し `brightness_` を掛けない。`brightness_` を適用するのは `LED::update()` のみだが、Notify の実描画経路（`setLeds→setColor`）は `update()` を一切呼ばない（`update()` は board.cpp の FATAL 停止表示専用インスタンスからのみ）。CLI `led <n>` で輝度を設定し NVS 保存しても二度と参照されず、本体 LED は常にフル輝度（既定 32=12% 想定が無視）。
- **影響:** 教室で 30 機が常時フル輝度点灯（眩しさ・消費電力増）。輝度調整機能が「設定できるが効かない」状態。飛行安全への直接影響はない。
- **対策:** `Notify::setLeds` でパック直前に `led.getBrightness()` を掛けてから `setColor` に渡す（最小修正）。または HAL に `setColorScaled()` を追加して責務を HAL に閉じる。

#### M-5. モータ PWM タイマ定数が 3〜4 箇所に独立定義され、分解能乖離で推力が静かに激減しうる ［確定］
- **場所:** `components/sf_board/board.cpp:93-96`、`main/config.hpp:212-213`、`components/sf_actuator/actuator.cpp:66-67`、`components/sf_hal_motor/motor_driver.cpp:30-31,163`
- **内容:** LEDC モータタイマの周波数・分解能・タイマ番号・スピードモードが sf_board／config.hpp／actuator.cpp（ローカルミラー）／motor_driver.cpp の **4 箇所**に独立定義。sf_board は唯一所有者（R1）として getter を公開するが、それを呼ぶ箇所はゼロ（死にコード）。`skip_timer_init=true` 経路では実タイマの分解能（sf_board 定義）と duty 計算が使う分解能（actuator 由来）が別ソースで、コンパイル時の結合（static_assert 等）が皆無。
- **影響:** 将来 sf_board 側の分解能だけ 10bit に上げて config を直し忘れると、タイマは 10bit（max 1023）で走るのに duty は 8bit max（255）で計算され、**全モータが意図の約 1/4 duty しか出ず離陸不能/制御喪失**。安全クリティカルな静かな退行になりうる。
- **対策:** 唯一の出所を `config.hpp` に定め、(1) actuator のローカルミラーを削除し config を直接 include、(2) board.cpp に config.hpp を include して `static_assert(kMotorPwmFreqHz == MOTOR_PWM_FREQ_HZ)` と分解能一致の static_assert を追加。これだけで片側編集をコンパイルエラーで止められる。

#### M-6. ToF 生値に時間的グリッチ除去がなく、単発外乱で接地/空中判定が即反転しうる ［確定］
- **場所:** `components/sf_takeoff_landing/takeoff_landing.cpp:91`、`components/sf_hal_vl53l3cx/src/vl53l3cx_wrapper.cpp:282`、`tasks/tof_task.cpp:70`
- **内容:** `evaluateToF/evaluateHeld` は閾値ヒステリシス（0.05/0.15m）のみでジャンプ/外れ値の時間フィルタが無い。上流 `getDistance()` は「地面は最遠」として最遠ターゲットを選ぶだけ。コンポーネント内に `vl53lx_outlier_filter`（500mm/サンプルのレート制限＋カルマン）が存在するのに底面読み出し経路に未配線。家具・手・ゴースト反射が status==0 で混ざるとその 1 サンプルで `on_ground_` が反転。
- **影響:** 離着陸付近で airborne フラグがちらつく。L-9（離陸確認タイマー死にコード）と組み合わさると、地上で 1 サンプルの遠方ゴーストが出ただけで FLYING へ早期遷移しうる（不安定な低高度で飛行モード突入）。※着陸検出は 1000ms 持続要求があり誤墜落には直結しない。
- **対策:** ① 既存 `vl53lx_outlier_filter` を `tof_task.cpp` に配線（最小コスト最大効果、自己回復も実装済み）。② または接地↔空中エッジに 2〜3 サンプル連続一致の確認カウンタを追加（30Hz で 60〜100ms）。③ さらに `state_task.cpp:552` の TAKEOFF→FLYING を raw airborne でなく確認済みフラグで駆動（L-9 と整合）。①＋③ の併用を推奨。

#### M-7. BMP280 高度が絶対標高（MSL）固定基準で、相補推定器が対地 ToF と矛盾ブレンド ［一部確定］（hal_baro_tof ＋ complementary が独立に検出）
- **場所:** `components/sf_hal_bmp280/bmp280.cpp:197-201`、`components/sf_estimator_complementary/complementary_estimator.cpp:180-189`、`tasks/imu_task.cpp:299`
- **内容:** `calculateAltitude()` は `P0=101325Pa` 固定の海面基準で絶対標高を返す。`setSeaLevelPressure()` は存在するが HAL 外から一度も呼ばれず、離陸地点でゼロ点合わせされない。一方 `imu_task.cpp:299` は推定器を問わず無条件に `updateBaro()` を呼び、相補推定器は同一 `altitude_` 状態へ ToF（対地≈0m）と baro（絶対標高、例:金沢で数十m）を同ゲイン（0.30）でブレンド → 引っ張り合いで平衡高度が中間に大きくバイアスする。
- **影響:** **相補推定器を選択した場合**、ALT_HOLD が成立しない/誤高度で制御。既定 ESKF は `eskf.use_baro` 既定 off ゆえ顕在化しないが、二重アンカーの矛盾がコードに残存し、相補を有効化した瞬間に飛行が破綻する潜在バグ。「推定器を差し替えても同じ挙動」という RESET_PLAN P2 の主張が実機 baro 環境で崩れる。
- **対策（推奨 A）:** 設計方針「鉛直は ToF-only、baro 鉛直融合は廃止」を相補推定器にも適用（`updateBaro` を no-op 化、または `kBaroAltGain/VelGain=0`）。**対策 B:** どうしても baro を使うなら BaroTask 起動時に静止平均気圧で `setSeaLevelPressure()` を呼び対地基準にゼロ点合わせ。加えて `BaroData.altitude` が MSL か対地かをフィールド名/コメントで明示。

#### M-8. autotune の `phi_needed` の wrap がホスト実装と非一致（同期契約からの逸脱）［一部確定］
- **場所:** `components/sf_autotune/autotune.cpp:208-209,224-226`
- **内容:** firmware は必要コントローラ位相を `wrapAngle` で (−π,π] に折り返してから二分法に使うが、ホスト `rate_sysid.py` は折り返さずに比較する。ヘッダは「`rate_sysid.py` と同一アルゴリズム — 双方を同期保守」と明記しており、この差は同期契約からの逸脱。
- **影響:** 通常運用（wc 1〜100, pm 20〜80）では `phi_needed` が ±180° 内に収まり結果一致。極端な動作点でのみ firmware が不正設計を「成功」扱いしうるが、後段の達成 PM ゲート（±5°）が必ず弾くため危険ゲインはモータに届かない。実害は「同一アルゴリズム」表記の不整合と無駄計算に留まる。
- **対策:** firmware から `wrapAngle` を外してホストと完全一致させる（`needed = phi_needed`）。PID で実現可能なリード位相は η による上限で 90° 未満ゆえ wrap 無しでも正しく判定できる。

### 3.3 LOW（24 件） — 影響度別グループ

実害は限定的だが、教材コードの正確性・保守性・実機 POS_HOLD ブリングアップ前の備えとして対応推奨。

#### A. 実機 POS_HOLD ブリングアップ前に塞ぐべきロバスト性
- **L-1. オプティカルフローの SQUAL（表面品質）が ESKF 更新でゲートに使われていない** ［確定］ `hal_imu_flow`。FlowData に詰めているが `EskfEstimator::updateFlow` が SQUAL を参照せず、低品質路面・暗所・高高度の偽フローが ESKF に注入されうる。→ `updateFlowRaw` に SQUAL を渡し `eskf.gate.flow_squal` でゲート、または χ²ゲート導入。閾値は実機 SQUAL 分布で決定。
- **L-2. ToF 距離が機体傾き補正（cosθ）を受けず接地/空中判定に使われる** ［一部確定］ `takeoff_landing`。判別器は粗く許容（精密高度の ESKF 側は既に cosθ 補正＋傾きゲート実装済み）。→ コメント明記＋必要なら判別器にも傾きゲート。

#### B. 相補フィルタ・mag（非既定経路）のロバスト性
- **L-3. 相補フィルタの `updateBaro` が ToF-only 方針に反して無条件ブレンド** ［一部確定］ `complementary`。M-7 と同根（相補推定器に有効化ゲートが無い）。→ M-7 の対策 A で同時解決。
- **L-4. 低品質な磁気校正でも `valid=true` となりヨー融合に使われる（品質ゲート不在）** ［一部確定］ `hal_mag`。coverage<0.5/fitness<0.5 でも警告のみで `valid=true`、`isValid()` も実質常に true。`eskf.use_mag` 既定 off ゆえ休眠。→ `isValid()` に coverage/fitness 下限を追加。use_mag を将来 on にする前に必須。
- **L-5. 未校正 Mag の融合除外が `ReloadParams` で覆される** ［確定］ `imu_task`。起動時に未校正と判定して切った mag が、任意の `eskf.*` ライブ変更/autotune で `cfg_.use_mag` 上書きにより再有効化。→ params 由来 `cfg_.use_mag` とは独立の `mag_calib_gate_` を EskfCore に持たせ、`setConfig` で触らない。

#### C. 状態機械・ペアリングの規範乖離
- **L-6. ソフトランディング（FLYING→ARMED_GROUND）が未配線・リセット動作も欠落** ［確定］ `state_manager`。`notifySoftLanding()` が @design [OK] 付きで実装されているが呼び出し元ゼロ、onEnter/onExit も未登録。→ touch-and-go 不要なら削除して仕様から外す、必要なら onEnter/onExit を配線。
- **L-7. `requestPairing` が IDLE_HELD でも受理（規範は IDLE_GROUND のみ）** ［確定］ `state_manager`。`state_manager.cpp:389` のガードと `state_task.cpp:603` の自動突入条件から IDLE_HELD を除外。
- **L-8. 長押し 3 秒再ペアが空中/ARM 中も呼ばれ得る（ガードは StateManager 側のみ）** ［不確定→INFO 級］ `state_task`。Click 側は state_task で地上限定するのに長押し側は非対称。→ §3.4 に詳述。

#### D. 離着陸・死にコード
- **L-9. 離陸完了の確認タイマーが死にコード — TAKEOFF→FLYING が単発 ToF 読みで即発火** ［確定］ `takeoff_landing`。`detectTakeoff/takeoff_hold_ms/isTakeoffDetected`（500ms 確認）が誰にも消費されず、実配線は単一 ToF 読みが空中閾値超で即発火。→ 死にコードを削除して挙動を文書化、または確認済みフラグで TAKEOFF→FLYING を駆動（M-6 ③ と整合）。

#### E. コメント・挙動・ドキュメント不一致
- **L-10. `computeLanding` のコメント「方位保持」と実装（ヨーレートゼロのみ）が不一致** ［確定］ `pid`。自動着陸中はヨー外乱で機首がゆっくり回頭しうる（降下短時間で衝突リスク小）。→ コメント修正（教材精度）。
- **L-11. 加速度 Z 符号規約の文書食い違い（コードは正、コメント/仕様が逆）** ［確定］ `imu_task`／`x_units` が独立検出。コードは全層 −g 規約で一貫・正しい。`noise_and_vibration_model.md:161-172` と一部規範文が +g 規約＋実在しない +2g バイアス機構を記述。→ ドキュメントをコードに合わせて修正（§3.5 で詳述、新規実装者の符号取り違え防止）。
- **L-12. Blackbox の記録レート/容量/flush 損失量のコメントが実装（100Hz）と不一致** ［確定］ `telemetry_log`。コメントは 50Hz 前提（2.7分/4分）だが実 100Hz で半分（1.3分/2分）。容量計算自体はレート非依存で正しい。→ コメントを 100Hz 基準に修正。
- **L-13. autotune の慣性モーメント定数が config 非参照のハードコード（SSOT 逸脱）** ［一部確定→INFO 級］ `api`。`kSpecInertia` は Nelder-Mead の初期種のみで飛行ゲイン非関与。→ 単位/出所コメント追記で十分。

#### F. 校正・推定の軽微
- **L-14. 傾いた面で校正すると重力投影が `accel_bias` に混入し level_offset は破棄される** ［確定］ `calibration`。水平床では真のオフセットで意図どおり。傾き校正時は飛行中オンライン再推定で収束するが離陸過渡で姿勢が偏る。feature_status.md に既知記載あり。→ `computeLevelOffset` の傾きを ESKF 初期姿勢へ与え、bias には残差のみ。当面は運用注意コメント＋デッドコード解消。
- **L-15. autotune `tunePid`: ゲイン余裕が見つからない時 `gm_db` を 0dB として返す** ［確定］ `autotune`。位相が −180° を横切らない（=GM 実質無限大で最も安全）場合に 0dB（=境界安定）と誤表示。**H-1 で GM 下限ゲートを素朴に入れると最も安全な設計を逆に弾く**。→ `gm_valid` フラグ/NaN 番兵で未検出と 0dB を区別（H-1 と必ずセット）。
- **L-16. ESKF `reset()` が `freeze_accel_bias_` をクリアするが `recomputeActiveMask()` を呼ばずマスクが古いまま** ［確定→休眠］ `eskf`。フリーズ機構は現在「broken」として無効化中ゆえ休眠（到達不能）。→ 再配線時の備えとして `reset()` 末尾に `recomputeActiveMask()` を 1 行追加。
- **L-17. 同一周期の ToF/Flow/Mag/Baro 補正が発行 state に反映されず 1 周期（2.5ms）遅れる** ［一部確定→低/INFO］ `eskf`。`update*` 群は `core_` のみ更新し `cached_state_` を再キャッシュしない（hold/reset 群は更新するため混在）。鉛直は handoff が別途書くため発散せず、空中定常で一定 2.5ms 位相遅れのみ（数Hz 帯域で無視可）。→ `getState()` を遅延評価（毎回 `convertState`）にして混在を構造的に解消。

#### G. 並行性（軽微・disarmed/地上限定）
- **L-18. 再ペア時の `controller_mac_` への非アトミック書き換えが RX コールバックの混信フィルタと競合** ［確定］ `comm`。TOCTOU 窓は存在するが MAC は全ゼロ方向にしか動かず、不正パケットを通す安全侵害には至らない（再ペア中の一過性パケット欠落のみ、地上 disarmed 限定）。→ `servicePairing()` で `paired_.store(false)` を `memset` より前に移して窓を構造的に排除。
- **L-19. ブロッキング `playTone` が 30Hz NotifyTask で LED 更新と notify_command drain を最大 ~0.5s 停止** ［確定/一部確定］ `notify`／`hal_io` が独立検出。トーン再生中に LED 点滅が固まって見える。制御権なしゆえ飛行安全には無影響。既存 `playToneAsync` は呼び出し元ゼロの未配線。→ NotifyTask を非ブロッキング・ステートマシン化（`playToneAsync`＋esp_timer 期限）、または 1 周期 1 イベント処理で点滅を毎周期確実に走らせる。

#### H. コア基盤・センサタスクの軽微
- **L-20. RingBuffer `publish`: `overflow_count` がドロップ未発生でも加算される（R14 精度バグ）** ［確定］ `core`。満杯時の tail 前進 CAS が失敗（=consumer が消費済み、喪失ゼロ）でも `fetch_add(1)` が無条件実行。コメント自身が「consumer が勝てば喪失ではない」と述べるのに計上。→ CAS 成功時のみカウント（`if (compare_exchange_strong(...)) overflow_count_.fetch_add(1)`）。
- **L-21. BaroTask が DRDY/data_ready ゲートなしで 50Hz 無条件発行（重複サンプル）** ［確定］ `sensor_tasks`。BMP280 実更新は ~13-16Hz で、同一変換値が複数回発行。MagTask が `data_ready` を厳格ゲートするのと非対称。baro は ESKF 鉛直に使わないため飛行影響なし。→ コメント是正（「タスク50Hz/実効~13-16Hz」）、必要なら data_ready ゲート化または standby 短縮。

### 3.4 INFO（8 件） — ドキュメント整合・予防的ハードニング・用途未定義

- **I-1. 長押し 3 秒再ペアリングが空中/ARM 中も呼ばれ得る（ガードは StateManager 側のみ）** ［一部確定］ `state_task`。現状 StateManager 内ガードで安全だが Click 側との防御非対称。→ state_task 側でも地上限定にするか「StateManager が唯一のガード所有者」とコメント明記。
- **I-2. VL53L3CX 底面初期化失敗を Critical 扱いせず `vTaskDelete` で握りつぶす** ［一部確定］ `takeoff_landing`。設計の失敗 3 分類との整合確認事項。→ Optional/Critical 分類の明確化。
- **I-3. `TopicLatest::updated()` は read-and-clear で単一 consumer 専用だが用途未定義（死にAPI）** ［確定］ `core`。呼び出し元ゼロ。複数 consumer で使うと取りこぼし。→ 単一 consumer 専用のコメント追記、または削除。
- **I-4. netif 生成の起動順序が設計 Level 構造と乖離（doc 内部矛盾）** ［一部確定］ `board`。機能影響なし（netif はセンサ非依存）。設計本文・hpp doc・実装の 3 者で netif の段が食い違う。→ ドキュメント一本化。
- **I-5. USB REPL タスクと TCP サーバが同一コマンドハンドラへロックなしで並走しうる** ［一部確定］ `cli`。param への同時書き込み。スカラ単発書き込みは Xtensa でアトミックゆえ現状実害なし、ReloadParams 経路はミューテックス保護済み。→ 将来 param が複合構造化する際の予防として CLI ディスパッチをミューテックス直列化。
- **I-6. `sensor_imu` RingBuffer が「SPSC・全サンプル保持」の設計意図と運用が乖離** ［一部確定］ `x_concurrency`。`read()` 消費者がゼロで全 consumer が `latest()`。飛行影響なし（latest は ABA 安全）。→ ドキュメントを実態（latest 専用共有）に合わせ、R14 監視配線時に sensor_imu を除外。
- **I-7. motor_test 失効後のワンショットクリアが publish 競合で取りこぼされうる** ［確定］ `x_concurrency`。armed 中は本分岐に来ない（disarmed 限定）、ベンチで稀に 1 周期無視される程度。→ ControlTask が motor_test を書き換えない設計（producer を CLI のみに）に変更。
- **I-8. 設計規範の加速度符号（「静止時 +9.81 の反力」）が実装（−9.81）と矛盾** ［確定］ `x_units`。L-11 と同根。コードは内部完全整合・physics_smoke でガード済み。→ §3.5 参照。

### 3.5 ドキュメント修正の集約（L-11 / I-8）

加速度符号は **コードが正典**（全層 −g 規約: ドライバ段 `body.z=−chip.z` → 水平静止で機体 Z accel ≈ −9.81）。以下を実装に合わせて訂正:
1. `docs/noise_and_vibration_model.md:161-172` の「生加速度計は水平静止で [0,0,+9.81]（+g 規約）」「ba_z≈+2g で内部変換」を「静止で [0,0,−9.81]（−g 規約）、ba_z≈0」へ。+2g バイアス機構は旧 vehicle 由来で新ファームに存在しない。
2. detailed_design 等の規範文の「静止時 +9.81 の反力」を「静止時 body z ≈ −9.81」へ。
3. `simulator/sil/docs/coordinate_frames.md` は既に −g で正しく、変更不要。符号の SSOT を `eskf_core.cpp:172,854` / `calibration.cpp:262` に集約する旨を明記。

---

## 4. 推奨対応順序

| 優先 | 項目 | 理由 |
|:----:|------|------|
| 1 | **M-1**（地上 DISARM）、**M-2**（wifi NVS armed ガード） | safety・1〜数行の確実な修正・回帰リスク極小 |
| 2 | **M-5**（PWM 定数 static_assert）、**H-1＋L-15**（autotune GM ゲート） | 静かな致命退行/不安定ゲインの予防。M-5 はコンパイル時ガード追加のみ |
| 3 | **M-3**（API 誘導権限）、**M-6**（ToF グリッチ＋L-9） | 安全前提の回復・実機飛行前のロバスト性 |
| 4 | **M-4**（LED 輝度）、**M-7＋L-3**（相補 baro）、**M-8**（phi wrap） | 機能正常化・推定器差替の整合（POS_HOLD/相補実機化前） |
| 5 | **L-1**（Flow SQUAL）、**L-5**（mag 保護）、**L-20**（overflow_count）、**L-21**（baro DRDY） | 実機 POS_HOLD/use_mag 有効化前・診断精度 |
| 6 | LOW/INFO のコメント・死にコード・ドキュメント整合（L-6〜L-19, I-*, §3.5） | 教材コードとしての模範性・保守性 |

各修正は **(1) 該当 SIL シナリオで回帰固定（修正後は必ず `sf sil build`）→(2) host 単体テスト→(3) 実機ベンチ確認** の順で検証する。安全関連（M-1〜M-3, M-6）は SIL に専用シナリオを 1 本ずつ追加して恒久化することを推奨。

---

## 5. 総評

vehicle_new は「スパゲッティ化の解消」「責務分離」「設計段階での未定義動作排除」という再構築目標を高い水準で達成しており、**致命的な飛行安全バグは検出されなかった**。制御・推定・ミキサー・状態機械・通信という安全の中核は設計規範および飛行実績コードと厳密に一致している。発見された 41 件は、(a) 安全境界の数件の取りこぼし（地上 DISARM・飛行中 NVS・API 権限）、(b) 非既定経路（相補フィルタ・autotune・mag・前方 ToF）のロバスト性、(c) 教材コードとしての精度（コメント/ドキュメント不一致・死にコード）に大別され、いずれも局所的かつ対策が明確である。優先度 1〜3 の 6 件（うち 4 件は数行修正）を先に処理すれば、安全境界の穴は塞がる。

---

## 6. 付録: 敵対的検証で棄却した誤検知（3 件）

レビュー段で挙がったが、検証段で実コードを精査して**棄却**した指摘。

1. **`Telemetry::setDestination` の非同期更新競合** — `setDestination()` の呼び出し元がファーム・SIL・tools のどこにも存在せず（定義のみ）、競合経路が成立しない。
2. **ESP-NOW 受信コールバックが Latest トピックの mutex を `portMAX_DELAY` で取得し 400Hz 制御ループと競合** — 経路は事実だが正しい設計（コールバックは WiFi タスク文脈の非 ISR、ミューテックスは短時間保持、優先度逆転は起きない）。指摘者自身も「実害なし」と認めた。
3. **`ApiCmd::Takeoff` が FLYING 中に `requestModeChange` を許し調停表に反する** — コード断片は実在するが、API Takeoff の唯一の発行元が地上限定で、FLYING 中のシナリオは到達不能。調停表の「FLYING/API設定=誘導目標のみ」は正しく実装されている。

---

## 7. レビュー方法（再現用）

- **規模:** 第一者コード約 27,500 行・90 ファイル（ベンダ VL53L3CX ドライバ本体は対象外、wrapper のみ）。
- **方式:** 27 ユニット（24 サブシステム ＋ 横断 3: 並行性／単位・符号・frame／設計適合）の並列レビュー。各レビュアーに設計規範（モード調停表§3.1・R1〜R16・クラスA/B reset・安全要件§9・ESKF/PID/ミキサー仕様）を与えてコードと照合。
- **検証:** 各指摘（severity medium 以上 ＋ correctness/safety/concurrency/units/design 系）を独立した懐疑的レビュアーが実コードを Read して敵対的に検証（confirmed / partially-confirmed / false-positive / uncertain）。意図的設計保留は false-positive として除外。
- **結果:** 96 候補 → 確定/一部確定 41、誤検知 3、低優先未検証で素通し 52。本書は確定/一部確定/不確定の 41 件を収録。
