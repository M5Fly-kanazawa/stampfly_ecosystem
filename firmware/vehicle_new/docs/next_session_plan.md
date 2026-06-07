# 次セッション指示書 — 設計準拠リファクタリング Phase 3 から再開

最終更新: 2026-06-07（**場当たりコード全廃リファクタ進行中。Phase 0 / 1 / 2a / 2b 完了、Phase 3 から再開**）

> このセッションは★ロバスト再飛行 readiness の実装に着手したが、その基盤の状態機械が
> 設計違反の場当たりコードだらけと判明し、ユーザー指示で「場当たりコードを全廃して
> あるべき姿にリファクタリング」する大方針へ発展した。承認済み計画 `valiant-frolicking-sun.md`
> の Phase 0〜8 のうち **Phase 0 / 1 / 2a / 2b を完了**。次は **Phase 3（責務分離の是正）**。
>
> **Phase 2b の重大発見（必読）:** 設計（detailed_design §3）の「ARM時 ESKF 全リセット」と
> 「bias freeze/unfreeze」を素直に配線すると POS_HOLD が姿勢発散・墜落することが SIL で判明。
> リセットタイミング掃引（8方策×飛行スイート）で最良策を数値選定し、**ARM時は「姿勢共分散のみ膨張」**
> （`InflateCov(Attitude)`）を採用。詳細は本書 §6 と detailed_design §3 注1〜3。

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
| 2b | **ロバスト再飛行 order2-3 ＋ COMM_LOST ＋ 地上→飛行 共分散ハンドオフ**。①接地復帰reset（FLYING/LANDING→IDLE_GROUND で isAirborne(from) ゲートの ESKF Reset、再飛行 readiness 要件①）。②COMM_LOST 3秒ホバー遅延（StateManager に `update()`＋pending timer、state_task が毎周期駆動、無条件着陸＝要件§9）。③**ARM時の ESKF 処理を SIL 掃引で確定**＝姿勢共分散のみ膨張（`InflateCov(Attitude)`）。ESKF に `inflateCovariance(mask)` 追加。 | `00468d0` |

**全フェーズで検証スイート全PASS**（vehicle_new 11シナリオ + legacy hover_espnow + hover_smoke G2+G3）。**Phase 2b で ESP-IDF 実機ビルド確認済み（952.7KB）。**

---

## 3. 次にやること: Phase 3（責務分離の是正）

Phase 2b まで完了。次は計画 `valiant-frolicking-sun.md` の **Phase 3**。

- `sf_command` に正規化を集約: `sf_comm` は生パケットを事実として渡し、ADC正規化・デッドバンド・調停を `sf_command`（責務#8）へ。退行したデッドバンドを復活。ADC中央値の不一致（2047.5 vs 2048）を統一。
- `sf_actuator/actuator.cpp`: config.hpp/params の複製（GPIO/PWM/ARM_D/KAPPA/モータ曲線/V_BATT）を解消。`V_BATT` 固定を実電圧（`sensor_power`）参照に。
- `motor_driver.cpp`: 旧HAL残骸（`setMixerOutput`/`testMotor`/stats系）を削除、二重ミキサー解消。
- 検証: 全検証スイート＋スティック→duty が期待通り。

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
