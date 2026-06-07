# 次セッション指示書 — Phase 6 の残り（Notify/Button/sensor_health/CLI）から再開

最終更新: 2026-06-07（**Phase 0〜5 完了 ＋ Phase 6 の Logger・mag χ² 完了。Phase 6 残=Notify/Button/sensor_health/CLI。割り込み候補=χ²ラッチアップ調査・デッドバンド復活**）

> **Phase 6 進行中（未実装機能の配線）:**
> - **✅ Logger（commit c773bac）**: log_task→sf_logger 配線。armed 中 100Hz で Blackbox(SPIFFS)記録、disarm で flush。Logger の per-cycle ログ氾濫バグ（open失敗時 100Hz で fopen エラー）を blackbox_failed_ ラッチで修正。実機は partitions.csv に storage パーティション確保で記録開始。
> - **✅ mag χ²（commit 6681c69）**: vectorUpdate3 を chi2_gate 引数化。mag→mag_chi2_gate、accel→accel_chi2_gate。mag が accel ゲート定数を流用していた TODO を解消（mag 既定OFFゆえ挙動不変）。
> - **⬜ 残り（次バッチ）:**
>   - **Notify（中）**: notify_task→sf_notify（40%）。LED(sf_hal_led 100%完成)/buzzer(sf_hal_buzzer 100%完成) を駆動。notify.cpp の applyLedPattern/playTone の HAL 呼び出し TODO を埋める。**Phase 4 の Critical-fail LED もここで実装**（board::fatal フック）。
>   - **Button（中）**: button_task→sf_hal_button（100%完成）。**注意: pilot_request は comm が毎パケット所有ゆえ共有不可**。新規 button_event トピック＋state_task consumer が必要（R5）。クリック→ARM/DISARM toggle。
>   - **sensor_health 1Hz（中）**: SensorHealth 型あり・publisher 無し。present_mask（sensor_present 集約）＋healthy_mask（各 sensor topic の鮮度）を 1Hz publish。新タスク or 既存タスク統合。
>   - **CLI（大）**: cli_task→esp_console（host shim あり）。params get/set/save、status、reboot。sf_console コンポーネント新設 or 旧 vehicle/ 参考移植。flight 非必須。

> **Phase 5 完了（起動シーケンス R3 ＋ params SSOT）:**
> - **5a（commit 0246dd2）**: main.cpp を宣言的 Phase 0-4（NVS/BSP/topics/params/tasks）に。14タスク生成を `sf::tasks::start_all()`（tasks.cpp 新設）へ集約。extern TaskHandle 排除（ControlTask が xTaskGetCurrentTaskHandle で自己登録→`sf::tasks::control_handle()`、死蔵 g_state_task_handle 削除）。**`params::init()` を Phase 3 に配線（重大: これまで未呼出＝NVS保存値が起動時に読まれていなかった）**。SIL smoke 3本の手動 handle 配線も撤去。
> - **5b（本コミット）**: 非機能で値もずれた `params.def` と params.hpp の壊れた X-macro スタブを**削除**。`params.cpp`（param_vars + table[]）を正式 SSOT と文書化（development_roadmap 原則2 / architecture / detailed_design §6 / hardware_init §4 を是正）。X-macro 復活はしない（明示テーブルが模範コードとして良いと判断）。
> - 検証: 全 metric が Phase 4 と完全一致（pos_roll 1.4575 等）、ESP-IDF 954.3KB。

> **Phase 4 完了（HW所有の一元化, R1/R2/R4）:** BMI270/PMW3901 の SPI は `skip_bus_init`、motor の LEDC は `skip_timer_init` で sf_board 借用に統一（二重初期化の「握り潰し」を所有権明示の省略へ）。`sensor_present()` を実装（Mag/Flow/Baro/Power の各タスクが init 成否を board に報告、atomic）。Critical 失敗（I2C/SPI/LEDC バス＝board::fatal、IMU＝imu_task、Motor＝actuator）を **halt 統一**（vTaskDelay ループ・esp_restart しない、§5）。**重要なスコープ判断: LED エラーパターン表示は Phase 6 に繰延**（LED/notify の所有が未確立＝notify_task スタブのため。board に sf_hal_led 依存を先行追加すると実機ビルド直前のリスク）。board::fatal に1行フックを残置。検証=11シナリオ＋hover_espnow＋hover_smoke G2/G3＋plant_smoke 全PASS、ESP-IDF 実機ビルド 953.5KB。
>
> **実装上の設計逸脱（記録）:** 計画は「imu_task が board::imu_spi() を渡す」だったが、imu_task/control_task/actuator は **emu_vehicle_new と部分試験(hover_smoke/rtos_smoke/rate_tune)の両方でコンパイルされ、後者は sf_board をリンクしない**。よって共有タスクは board を呼ばず plain bool（skip_bus_init/skip_timer_init）で借用を表現した（spi_host は SPI2_HOST=board::imu_spi() に一致）。flow/mag/baro/power/tof は emu 専用なので board 直呼び出し可。

> このセッションは★ロバスト再飛行 readiness の実装に着手したが、その基盤の状態機械が
> 設計違反の場当たりコードだらけと判明し、ユーザー指示で「場当たりコードを全廃して
> あるべき姿にリファクタリング」する大方針へ発展した。承認済み計画 `valiant-frolicking-sun.md`
> の Phase 0〜8 のうち **Phase 0 / 1 / 2a / 2b / 3 を完了**。次は **Phase 4（HW所有の一元化）**。
>
> **Phase 3 の重大発見（必読）:** sf_command への正規化集約は完全に挙動保存。しかし副次の
> 「V_BATT 実電圧化」と「デッドバンド復活」が、限界安定の毎軸 POSITION_HOLD/STABILIZE を
> 崩すことが SIL で判明。真因は **ESKF accel-attitude の χ² ゲート(7.8)がマニューバ中 66% 棄却し、
> 推定が一度発散すると回復補正も棄却して固着する「ラッチアップ」**。0.1% の推力変化や
> デッドバンドがこの崖を越えさせる。→ **SIL に 1S LiPo 電池サグモデルを実装し、実電圧を live で
> 追従させることで Model Identity を保ち χ² ラッチアップを非発火にして V_BATT 実電圧化を解除**
> （commit b8fd27e）。デッドバンドは機構配線・既定0 で保留。詳細は本書 §6。
>
> **Phase 2b の発見（参考）:** 「ARM時 ESKF 全リセット」「bias freeze/unfreeze」は POS_HOLD を
> 姿勢発散させる。ARM時は「姿勢共分散のみ膨張」（`InflateCov(Attitude)`）を採用。detailed_design §3 注1〜3。

---

## ★最初にやること（未決の判断 — ユーザーに確認）

前セッション末で**順序が未決**。次の3つを、まずユーザーに順序確認してから着手する：

1. **χ² ラッチアップ調査**（推定器ロバスト性・実機にも関わる。`docs/chi2_latchup_finding.md` が起点）
2. **デッドバンド復活**（χ² 改善後に 0.05 を有効化。現状は機構配線・既定0）
3. **Phase 4（HW所有一元化）**（計画 `valiant-frolicking-sun.md` の本来の次フェーズ）

推奨は **1 → 2 → 3**（χ² を直せば 2 もそのまま通る見込み）だが、Phase 4 を先にする選択もある。
**この判断を最初にユーザーへ。** χ² は当座 SIL 電池サグの動的ディザで非発火にしてあるだけで根治していない。

---

## 0. 着手前に必ず読む（順番に）

1. **`/Users/kouhei/.claude/plans/valiant-frolicking-sun.md`** — 承認済みリファクタ計画。Phase 0〜8 の全体像と各フェーズの目的・検証。**これが本指示書の親文書**。
2. **`architecture.md §4「リセット処理の2層分類」**`（クラスA / クラスB の境界条件）— 今回のリファクタの中核原則。Phase 2b 以降の reset 配線はこの分類に従う。
3. 直近コミットログ（Phase 2b = `00468d0` まで）— 各 Phase のコミットメッセージに Next steps と設計判断。
4. CLAUDE.md vehicle_new 6文書（特に architecture.md / detailed_design.md §3 状態遷移テーブル）。
5. auto-memory: `reference_params_ssot`（params の罠。Phase 5 で重要）、`feedback_plain_japanese_terms`（用語）。

---

## 1. 背景（なぜこのリファクタをしているか）

- 元は★ロバスト再飛行 readiness（墜落・通信断後に置き直せば再飛行できる状態に戻す）の実装に着手。
- その基盤の状態機械が **設計違反の場当たりコード**（onEnter/onExit コールバック機構が未使用のデッドコード、reset が各タスクのエッジ検出に散在 ＝ 旧 vehicle/ のスパゲッティ再現）と判明。
- ユーザー指示: **「場当たりコードを全廃し、あるべき姿にリファクタリング」**。
- 3体のレビューエージェントによる全コード監査で約18件の高重大度＋多数を検出（詳細はこのセッションのレビュー結果）。
- スコープ: **全範囲**（状態機械reset集約・責務分離・HW所有/起動骨格・未実装機能配線・品質仕上げ・ロバスト再飛行 SIL検証）をユーザー承認済み。

---

## 2. 完了したフェーズ（コミット済み）

| Phase | 内容 | コミット |
|-------|------|---------|
| 0 | 基盤トピック・型（estimator/controller/notify_command, sensor_health, overflow_count R14, topic_reference R9同期, SystemMode.timestamp 修正） | `0cae0dc` |
| 1 | **reset処理を StateManager の onEnter/onExit/onModeChange に集約**（場当たり排除の核心）。imu_task/control_task のエッジ検出を全廃しトピック経由（R5）に。freezeBias/unfreezeBias 追加。 | `d6b4478` |
| docs | **reset の2層分類（クラスA/B）原則**を architecture §4 / detailed_design §3 に明記 | `ab92271` |
| 2a | **IDLE_HELD検出・着陸完了の状態モデル配線**。TakeoffLandingMgr に isHeld()、detectLanding を velocity 注入の純粋関数化（estimate_state 直読み撤廃）＋レベルフラグ化。held/landing を system_status 経由で state_task が消費し notify 駆動。IDLE_HELD→IDLE_GROUND onEnter で再校正。 | `21a7727` |
| 2b | **ロバスト再飛行 order2-3 ＋ COMM_LOST ＋ 地上→飛行 共分散ハンドオフ**。①接地復帰reset。②COMM_LOST 3秒ホバー遅延。③**ARM時=姿勢共分散のみ膨張**（`InflateCov(Attitude)`）。 | `00468d0` |
| 3 | **責務分離の是正（コア）**。sf_command に正規化・デッドバンド・flags デコードを集約（死蔵コード解消）。正規化を実証パス（2048中央, throttle clamp[0,1]）に統一。sf_comm は生パケットを RawControlInput として渡すだけ＋RawInputSink 注入で comm_task が配線（層逆転回避）。motor_driver 旧HAL残骸（setMixerOutput=二重ミキサー/testMotor/stats）削除。挙動完全保存（ドリフト値ベースライン一致）。 | `95f9418` |
| 3+ | **SIL 1S LiPo 電池サグモデル＋V_BATT 実電圧化**。plant に動的端子電圧 v_batt=OCV(SoC)−I·R_int（電流=モータ電気モデル＋アビオ、クーロンカウント、vpython OCV曲線移植）。emu で有効化、actuator を live(sensor_power)へ。χ²ラッチアップ非発火で全PASS。 | `b8fd27e` |
| 4 | **HW所有の一元化（R1/R2/R4）**。BMI270/PMW3901 SPI を `skip_bus_init`、motor LEDC を `skip_timer_init` で sf_board 借用に統一（二重init握り潰し→所有明示の省略、board::flow_spi() 追加）。`sensor_present()` 実装（atomic, Mag/Flow/Baro/Power の init 成否を board へ報告）。Critical 失敗を halt 統一（board::fatal＋imu_task/actuator、esp_restart せず §5）。**LED 表示は Phase 6 繰延**（LED 所有未確立）。共有タスクは smoke 非リンク制約で board 直呼びせず plain bool で借用。 | (本コミット) |

**全フェーズで検証スイート全PASS**（vehicle_new 11シナリオ + legacy hover_espnow + hover_smoke G2+G3 + plant_smoke）。**Phase 4 で ESP-IDF 実機ビルド確認済み（953.5KB）。**

---

## 3. 次にやること: Phase 6（未実装機能の配線）— ただし χ²/deadband・LED が割り込み候補

Phase 5（起動シーケンス＋params SSOT）まで完了。次は計画 `valiant-frolicking-sun.md` の **Phase 6**。

**割り込み候補（ユーザーと相談して順序決定）:**
- **χ² ラッチアップ調査（推定器ロバスト性、実機にも関わる）**: `eskf_core.cpp` の accel χ²ゲート(7.8≒χ²3自由度95%点)がマニューバ中66%棄却し、推定が発散すると回復補正も棄却して固着する。電池モデルの動的ディザで今は非発火だが潜在。ゲート緩和(11.3≒99%点)/回復ロジック/適応Rを SIL で数値検証。**これを直せばデッドバンド復活も通る見込み。**
- **デッドバンド復活**: χ² 改善後に 0.05 を有効化（現状は機構配線・既定0）。
- **Phase 4 LED 繰延分**: Critical 失敗時の LED エラーパターン表示。Phase 6（notify/LED 所有確立）でやるのが本来＝Phase 6 と一緒に片付くはず。board::fatal にフック有り。

**Phase 6 本体（未実装機能の配線）:**
- `tasks/log_task.cpp`: `sf_logger::Logger` を配線（Blackbox/データストリーム）。
- `tasks/notify_task.cpp`: `sf_notify::Notify` を配線（`notify_command` 消費、LED/ブザー実HAL駆動）。**ここで Phase 4 の Critical-fail LED も実装可**（board::fatal フック）。
- `tasks/button_task.cpp`: ボタン→`button_event` or ARM/DISARM。
- `tasks/cli_task.cpp`: コマンドレジストリ（R6）＋ params get/set/save。
- `sf_board` or 各task: `sensor_health` を 1Hz publish（R15）。sensor_present() は実装済（Phase 4）。
- `eskf`: mag 観測 χ²ゲート（TODO解消）。

### Phase 5 でやったこと（完了・参考）
- 5a: main.cpp 宣言的 Phase 0-4、start_all() 集約（tasks.cpp）、extern TaskHandle 排除（control 自己登録）、params::init() 配線（未呼出だった）、smoke 3本の手動 handle 撤去。
- 5b: params.def 撤去＋X-macro スタブ撤去、params.cpp を SSOT 文書化、4設計文書是正。
- **共有タスク制約（再掲・重要）**: imu_task/control_task/actuator は emu と smoke 両方でコンパイル＆smoke は sf_board 非リンク。control_handle() は control_task.cpp に定義（両方コンパイル）、宣言は tasks.hpp（smoke の include に VN/tasks 追加）。start_all() は tasks.cpp（smoke 非コンパイル）。

### Phase 4 でやったこと（完了・参考）
- BMI270/PMW3901 SPI 借用（skip_bus_init）、motor LEDC 借用（skip_timer_init）、board::flow_spi() 追加。
- sensor_present() 実装（atomic、Mag/Flow/Baro/Power の init 成否報告）。
- Critical 失敗 halt 統一（board::fatal＋imu_task/actuator）。LED は Phase 6 繰延。

### Phase 2b でやったこと（完了・参考）

1. **接地復帰 ESKF reset**（state_task onEnter, IDLE_GROUND の `else` 分岐）: `isAirborne(from)` ゲートで FLYING/TAKEOFF/LANDING→IDLE_GROUND の時のみ `estimator_command(Reset)`。ARMED_GROUND→IDLE_GROUND（飛ばずDISARM）はResetなし。再飛行 readiness 要件①。
2. **COMM_LOST 3秒ホバー遅延**: `StateManager::update(now_us)`＋`comm_lost_pending_`/`comm_lost_time_us_`。handleAlert(COMM_LOST) は FLYING 中にタイマ起動（`alert.timestamp` 基準）、update() が `kCommLossHoverUs`(3s) 経過で LANDING、FLYING を外れたらキャンセル。state_task が毎周期 `update()` 駆動。failsafe は立ち上がりエッジ1回発報ゆえ周期ポーリング必須。無条件着陸（復帰でキャンセルしない＝要件§9）。
3. **ARM時 ESKF処理＝姿勢共分散のみ膨張**（§6 参照で確定）: `EstimatorCmd::InflateCov`＋`CovScope`＋`EstimatorCommand.arg` 新設、ESKF `inflateCovariance(mask)`（推定値 x 保持・指定状態の P 対角を init へ・クロス共分散ゼロ化）。state_task ARM で `InflateCov(Attitude)`。
4. **見送り**: bias freeze/unfreeze（ESKF 凍結機構が恒久隔離用でトグル非互換、detailed_design §3 注3）、校正完了時 freezeBias。freezeBias/unfreezeBias メソッドは capability として残置（estimator.hpp に再配線禁止の caution）。

---

## 4. Phase 3 以降（計画 valiant-frolicking-sun.md 参照）

| Phase | 概要 |
|-------|------|
| 3 | 責務分離: sf_command に正規化集約（sf_comm 死蔵解消・デッドバンド復活）、sf_actuator のパラメータ複製解消（V_BATT固定→実電圧）、motor_driver 旧HAL残骸削除 |
| 4 | HW所有一元化（R1/R2/R4）: IMU/Flow/Motor を board の SPI/LEDC 借用に、二重初期化解消、sensor_present()実装、Critical失敗の abort+LED統一 |
| 5 | 起動シーケンス（R3）: main.cpp の Phase 番号整合・start_all()集約・extern TaskHandle排除・NVS params load 配線、params.def を機能化 or 撤去（reference_params_ssot 参照） |
| 6 | 未実装機能配線: Logger/Notify/CLI/Button タスク、sensor_health 1Hz publish、mag χ²ゲート |
| 7 | 品質仕上げ: @designタグ全[OK]化、R13 @publisher/@subscriber、残TODO、マジックナンバー集約（重力9.80665統一・相補ゲインparams化・pidリミットconfig化）、コメントドリフト修正、esp_netif STA所有の設計文書矛盾解決 |
| 8 | ★ロバスト再飛行 SIL検証（order4-6）: emu に物理ハンドリング機構（持上げ→反転→運搬→設置のキネマティック軌道、IMU/ToF合成、teleport厳禁）、crash_refly.scn / modeswitch.scn でゲート化 |

---

## 5. 検証スイート（各フェーズ末で必ず全PASS → /commit）

```bash
source setup_env.sh
sf sil build
for s in pos_roll pos_pitch pos_flight pos_yaw alt_flight stab_flight acro_flight disturb commloss calib prearm; do
  sf sil scenario simulator/sil/scenarios/$s.scn --target vehicle_new
done
sf sil scenario simulator/sil/scenarios/hover_espnow.scn --target vehicle   # legacy が壊れていないか
simulator/sil/build/hover_smoke simulator/sil/models/stampfly.xml           # G2+G3 物理真値ゲート
sf build vehicle_new   # ESP-IDF 実機ビルド（ファーム変更時は必須）
```

合否は各シナリオの `[PASS]/[FAIL]` で判定。全PASS確認後に `/commit`（Next steps 必須、heredoc は長すぎると稀に malformed になるので適度な長さに）。

---

## 6. 重要な設計判断・教訓（このセッションで確立）

1. **reset の2層分類（architecture §4 に明文化済み）**:
   - **クラスA（遷移リセット）**: 状態遷移に紐づく離散イベント（PID積分器クリア・ESKF全状態リセット・通知）。タイミングは±20msで可。→ StateManager の onEnter/onExit に集約。
   - **クラスB（センサ同期リセット）**: センサ事象に紐づき1サンプル精度を要する推定器内部処理（鉛直ハンドオフ）。→ そのセンサを観測するタスク（imu_task）が所有。
   - クラスBを許す3境界条件（センサ事象トリガ / サンプル精度を数値実証 / 推定器内部連続状態）と越権防止制約は architecture §4 参照。**例外を恣意的に増やさず、境界条件で機械的に振り分ける**ことでスパゲッティ化を防ぐ。

2. **鉛直ハンドオフ（resetPositionVelocity/holdPositionVelocity）は imu_task に残す**: ToF と密結合で1サンプル精度が必要。onEnter(FLYING) に移すと state_task の20msポーリング遅延が乗り、α-βトラッカ（運動加速度補償）の初期化が遅れて POS_HOLD 姿勢が劣化（att_rmse 3.1°→12.8° で実証→差し戻し）。**FlightState が FLYING になるのは20ms遅れてよいが、推定器リセットは ToF 検知と同時でなければならない。**

3. **hover_smoke は StateManager をバイパス**して system_mode を直接注入する部分試験プログラム。インターフェースを変えたら近道している側（hover_smoke）も補填が要る（Phase 1 で controller_command(ModeChange) を補填）。

4. **params.cpp の手書き table[] が真の SSOT、params.def は非機能**（auto-memory reference_params_ssot）。新 param は params.cpp の param_vars + table[] 両方に追加。Phase 5 で扱う。

5. **地上→飛行の共分散ハンドオフは「姿勢のみ膨張」が最良（Phase 2b, SIL 掃引で確定）**:
   - 設計の「ARM時 ESKF 全リセット」「bias freeze/unfreeze」は **POS_HOLD を姿勢発散・墜落させる**（pos_roll/pitch/flight、各々単独でも壊す）。根本原因＝離陸直前/離陸時に **BA・全状態の共分散が初期の大きな値へ再膨張**し、離陸スラスト加速度を加速度計が拾い、それをバイアス/姿勢誤差と誤推定（accel-attitude 観測モデルが推力寄与 a_body=−T/m を含まない構造限界＝detailed_design「線形化バイアス（既知の構造的限界）」節）。
   - **掃引（8方策×飛行スイート）で全PASS は2つだけ**: 何もしない（code 0）と **ARMで姿勢共分散のみ膨張**（code 5）。姿勢は離陸前に地上で重力から再収束するので膨張が害にならない。位置/速度/バイアスの膨張は再収束の機会が無く離陸まで残り発散。タイミングは ARM が最良（地上で再収束する余地）、TAKEOFF/FLYING は最悪。
   - **教訓**: 「地上収束は飛行を代表しない＝出鱈目」は **姿勢には当てはまらない**（重力は地上もホバーも同じ）。位置/速度の「地上ゼロ」非代表性は**既に離陸エッジの resetPositionVelocity（クラスB）が正確に処理済み**。定性推測でなく **掃引で最良策を数値選定**したのが要点（CLAUDE.md 原則）。
   - **ESKF 凍結機構の限界**: `active_mask`＋`enforceCovarianceConstraints` は「センサ恒久不在」隔離用で、凍結状態の共分散を毎周期 init へ戻す。地上↔飛行トグルには非互換（解除で巨大共分散復活）。トグル運用には「共分散を init に戻さない soft-freeze」の別設計が要る（将来課題）。
   - **Phase 8（crash_refly）への申し送り**: 接地復帰reset後すぐ再離陸すると同じ共分散再膨張で発散しうる。地上再収束の時間確保 or soft-freeze が要る見込み。

6. **χ² ゲートのラッチアップ＝過敏性の真因（Phase 3, データで特定）**:
   - 毎軸 POSITION_HOLD/STABILIZE は限界安定。0.1% の推力変化やデッドバンドで pos_yaw が 1.08m→7.40m に発散する「崖」があった。
   - **真因**: ESKF の accel-attitude 3次元更新の χ² ゲート（`eskf_core.cpp::vectorUpdate3`, `accel_chi2_gate=7.8`≒χ²3自由度95%点）。マニューバ中は運動加速度で innovation が大きく **常時66%が棄却**。さらにゲートは**ハードカットオフ**（d²>7.8 で更新を全 return）なので、推定が一度発散すると正しい補正も外れ値に見えて棄却され、d² が 100〜900 に跳ね永久に固着（**latch-up**）。回復補正をゲート自身が弾く。
   - **計測**: FAIL時 roll_est が真値から数百度ズレて固着・att_rmse 巨大。PASS時は att_rmse 2〜3°で推定健全。棄却率は PASS/FAIL でほぼ同じ(66%)＝平均でなく臨界の瞬間に閾値を跨ぐか否か。
   - **位置づけ**: 実機にも関わる**推定器ロバスト性の構造的弱点**（突風・ノイズスパイクでも臨界に当たれば発火しうる）。ただし顕在化は**激しい毎軸POS_HOLD捕捉**のみ。disturb（横風＋モータ故障）・N1/N2・通常飛行は全PASS。
   - **当座の回避（Phase 3+）**: SIL 電池モデルで電圧を動的にし live で追従 → 量子化が一定バイアスでなく微小ディザになりラッチを跨がない。**根治ではない**。
   - **根治の候補（別タスク）**: χ²ゲート緩和(7.8→11.3等)/ラッチ回復ロジック/適応R強化。直せばデッドバンド0.05も通る見込み。CLAUDE.md 原則で SIL 数値検証必須。

7. **SIL 電池サグモデル（Phase 3+, commit b8fd27e）**:
   - プラントは定電圧電池をモデル化していなかった（v_batt固定3.7）。実電圧化はサグ対象が無く量子化の害だけ→保留していた。
   - 動的端子電圧 v_batt=OCV(SoC)−I·R_int を実装（電流=各モータ I_i=(V_motor−Km·ω)/Rm＋アビオ、クーロンカウント、OCV曲線は vpython 移植）。**Config::batt_model_enable 既定OFF**（物理smokeは定電圧）、emu のみON。
   - 実測: 満充電4.2V起動、飛行中 端子3.36〜4.19V・電流~7.5A・SoC 100→90%。低電圧FS(3.3V)誤発報なし。
   - **知見**: ホバー電流~7.5A は 300mAh で~2.5分相当（文書4分より高め）。測定 Rm/Km と内部整合・高C率コアレスでは妥当だが、`thrust_efficiency=1/1.12` と明示サグの二重計上の切り分けは将来の追検証。

---

## 7. 用語の注意（2026-06-07 ユーザー指摘）

- **「回帰（テスト/スイート）」は使わない。** 日本語の「回帰」は線形回帰など統計用語が主で誤解を招く。→ **「検証スイート」「テストを回す」「既存の動作が壊れていないこと」** を使う（auto-memory `feedback_plain_japanese_terms`）。
- ソフト用語は平易な日本語を主に（ハーネス→試験プログラム 等）。

---

## メモ

- 各フェーズ後は**必ず検証スイート全PASS → `/commit`（Next steps 必須）**。
- 制御/ESKFパラメータを変える提案は**必ず SIL の数値シミュレーションで裏付けてから**（CLAUDE.md 原則）。
- 安全機能は実機で命に関わるので、スタブを「動いたつもり」にしない。emu で実経路を発火させて確認。
- 設計矛盾を新たに発見したら実装を止めて報告（CLAUDE.md vehicle_new 原則）。
