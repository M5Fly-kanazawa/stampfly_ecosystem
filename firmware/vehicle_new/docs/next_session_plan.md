# 次セッション指示書 — 設計準拠リファクタリング Phase 2b から再開

最終更新: 2026-06-07（**場当たりコード全廃リファクタ進行中。Phase 0 / 1 / 2a 完了、Phase 2b から再開**）

> このセッションは★ロバスト再飛行 readiness の実装に着手したが、その基盤の状態機械が
> 設計違反の場当たりコードだらけと判明し、ユーザー指示で「場当たりコードを全廃して
> あるべき姿にリファクタリング」する大方針へ発展した。承認済み計画 `valiant-frolicking-sun.md`
> の Phase 0〜8 のうち **Phase 0 / 1 / 2a を完了**。次は **Phase 2b**。

---

## 0. 着手前に必ず読む（順番に）

1. **`/Users/kouhei/.claude/plans/valiant-frolicking-sun.md`** — 承認済みリファクタ計画。Phase 0〜8 の全体像と各フェーズの目的・検証。**これが本指示書の親文書**。
2. **`architecture.md §4「リセット処理の2層分類」**`（クラスA / クラスB の境界条件）— 今回のリファクタの中核原則。Phase 2b 以降の reset 配線はこの分類に従う。
3. 直近コミットログ（`21a7727` まで）— 各 Phase のコミットメッセージに Next steps と設計判断。
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

**全フェーズで検証スイート全PASS**（vehicle_new 11シナリオ + legacy hover_espnow + hover_smoke G2+G3）。Phase 0/1 は ESP-IDF 実機ビルドも確認済み（974KB）。**Phase 2a の ESP-IDF ビルドは未確認（Phase 2b 完了後にまとめて行う）。**

---

## 3. 次にやること: Phase 2b（ロバスト再飛行 order2-3 ＋ COMM_LOST）

Phase 1 で「ARM時/接地復帰の ESKF full reset」と「bias freeze/unfreeze の意味づけ」を、校正再注入と一体で行うため Phase 2b に委譲していた。それを配線する。**挙動変更が大きいので検証スイートで慎重に確認すること。**

### 3.1 state_task の onEnter 拡張（クラスA = 遷移リセット、tasks/state_task.cpp の registerStateCallbacks）

detailed_design §3 状態遷移テーブルに従い、以下を追加（既存の Phase 1/2a 配線に上乗せ）:

| 遷移 (from→to) | 追加する発行 | 意図 |
|---|---|---|
| IDLE_GROUND→ARMED_GROUND（ARM） | `estimator_command(Reset)` ＋ `estimator_command(FreezeBias)` | 設計表「ARM時 ESKFリセット」。reset後 reseedCalibration（imu_task）が校正bias再注入、地上ゆえ freeze |
| TAKEOFF→FLYING（離陸完了） | `estimator_command(UnfreezeBias)` | 飛行レジームで bias 推定再開。※pos/vel reset はクラスB（imu_task）のまま触らない |
| FLYING/LANDING→IDLE_GROUND（接地復帰） | `estimator_command(Reset)` | 墜落・着陸・緊急DISARM 後に推定器を全状態リセット（再飛行 readiness の要件①） |
| LANDING→IDLE_GROUND（着陸完了） | ＋ `estimator_command(FreezeBias)` | 地上レジームで bias 凍結 |

- **校正再注入は imu_task 側で既に実装済み**（`processEstimatorCommands` の Reset ケースが `reset()` → `reseedCalibration()` を実行、`g_calib_applied`/`g_applied_*_bias` 保持済み）。state_task は Reset を発行するだけ。
- **bias の地上/飛行レジームの一貫性に注意**: `EskfCore::reset()` は `freeze_accel_bias_=false` にする（estimator.hpp の設計者ノート参照）。よって地上で Reset を出したら必ず FreezeBias も出して「地上=frozen」を保つ（上表で ARM・着陸復帰に FreezeBias を併記しているのはこのため）。校正完了時（imu_task `feedBootCalibration` 末尾）にも `freezeBias()` を1回追加して初期も frozen にする。

### 3.2 imu_task（components 側）

- `feedBootCalibration()` の完了時（applyCalibration の後）に `g_estimator->freezeBias()` を追加（校正後は地上 frozen）。
- `processEstimatorCommands()` の Reset/FreezeBias/UnfreezeBias ケースは Phase 1 で実装済み（追加不要、発行が増えるだけ）。

### 3.3 COMM_LOST の3秒ホバー遅延（components/sf_state/state_manager.cpp:264 付近の TODO）

- 現状: `handleAlert(COMM_LOST)` が即 `transition(LANDING)`。
- 設計（architecture §4）: 通信途絶 → ホバー維持 3秒 → LANDING。
- 実装案: COMM_LOST を受けたら時刻を記録し、3秒経過後に LANDING へ（state_task の周期ポーリングで経過判定、or タイマ）。`commloss.scn` の `.expect` の順序チェックに影響しうるので閾値・窓を再確認。

### 3.4 検証

- 下記「検証スイート」を全PASS。特に **ARM時 ESKF reset が att_rmse / 離陸過渡を悪化させていないか**（pos_flight/pos_yaw/alt_flight）。
- 崩れたら: reset→reseed→freeze の順序、または ARM時 reset の要否を再検討（Phase 1 で「ARM時 reset は校正消失リスク」と判断し見送った経緯。reseedCalibration で解消する前提だが、離陸過渡への影響を数値で確認すること）。
- ESP-IDF 実機ビルド `sf build vehicle_new` を Phase 2b 完了時に確認。

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
