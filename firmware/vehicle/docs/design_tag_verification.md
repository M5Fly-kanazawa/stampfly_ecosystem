# @design タグ検証レポート（Phase 7）

> 作成: 2026-06-08 / 対象: `firmware/vehicle/`（components + tasks + main）
> 本レポートは、コードに付与された `@design` トレーサビリティ注釈のうち**未検証（`[--]`）**だったものを、設計文書と突き合わせて検証した結果の記録です。

---

## 1. 概要

vehicle のソースには、各クラス・インターフェース・状態遷移・データ型に
`@design <文書>.md §X — <説明> [判定]` 形式の注釈が付いている。判定は次の3値：

| 記号 | 意味 |
|------|------|
| `[OK]` | コードが参照先設計節を正しく実装していると検証済み |
| `[NG]` | コードが設計節と矛盾／未達（理由付き） |
| `[--]` | 未検証 |

検証前の状態は **総数 243 件中 [OK] 106 / [NG] 0 / [--] 137**。本検証では **[--] の 137 件**（実際に表へ起こせたのは重複行を1件にまとめた 133 行）を対象に、コードと設計文書の両方を読んで判定した。

**結論（先に要点）:**
- **コード実装はほぼ全て設計通り**（111 件 OK）。
- **設計文書側の参照ズレ（STALE）が 16 件** — detailed_design は §8 まで／architecture は §7 までしか存在しないのに、コードが §8/§10/§11/§12 を参照している等。**コードのバグではなく注釈の参照先・文言の陳腐化**。
- **真の不一致（NG）は 6 件**。うち 3 件は要ユーザー判断（安全閾値・設計矛盾）、3 件は注釈の文言修正で解消。

---

## 2. 検証方法

各 `[--]` タグについて、以下を厳密に実施した（部品クラスタ別に7並列の監査サブエージェントが担当し、各判定に**根拠（設計要求の要約＋それを満たす/満たさないコード箇所）**を必須化。推測での `[OK]` を禁止）。

1. **コードを読む** — タグが付いたコード要素（クラス／関数／状態遷移／データ構造）が実際に何を実装しているかを確認。
2. **設計節を読む** — タグが参照する設計文書（`requirements.md §2` 等）の該当節（§番号＋見出し）を実際に開いて読む。**節が存在するかも確認**。
3. **判定** —
   - `OK`: コードが当該設計節の要求を正しく実装。
   - `NG`: コードが設計節と矛盾／要求未達。
   - `STALE`: 参照先の節番号・見出しが設計文書に存在しない、または対応しない（トレーサビリティの陳腐化）。
4. **根拠を記録** — 設計が何を要求し、どのコードが満たす/満たさないかを1〜2文で明記。

検証後、影響の大きい指摘（failsafe 安全閾値・netif 所有・文書の節番号上限）は**メイン側で実コード・実文書をスポット再検証**し、全件で監査結果と一致することを確認した（過信防止）。

### クラスタ分担

| クラスタ | 対象ファイル | 主参照文書 |
|---------|------------|-----------|
| sf_state | state_manager.cpp/hpp, flight_state.hpp | requirements §2/§4/§9, detailed_design §3, architecture §2/§4 |
| 推定器 | estimator.hpp, eskf_estimator.cpp/hpp, eskf_core.cpp/hpp, complementary_estimator.hpp | detailed_design §5, requirements §4/§10, architecture §2/§3/§4 |
| sf_core | params.cpp/hpp, topics.hpp, topic.hpp, data_types.hpp | detailed_design §2/§6, architecture §3, coding_and_education §2 |
| 制御/failsafe/math | controller.hpp, pid_controller.cpp/hpp, pid.hpp, failsafe.cpp/hpp, sf_math.hpp | detailed_design §4, requirements §4/§9/§10, detailed_design §7 |
| board/comm/api | board.cpp/hpp, comm.cpp, sf_api.hpp | hardware_init §3/§4/§5, architecture §7/§2.5, topic_reference §4 |
| notify/logger/calib/takeoff | notify.cpp/hpp, logger.hpp, calibration.cpp/hpp, takeoff_landing.cpp/hpp | detailed_design §9/§10/§11/§12, architecture §3/§7/§8 |
| tasks/main | state_task.cpp, control_task.cpp, power_task.cpp, notify_task.cpp, tasks.hpp, main.cpp, config.hpp | architecture §5/§6, detailed_design §3/§7/§8, requirements §2/§3/§8/§9 |

---

## 3. 集計

| クラスタ | OK | NG | STALE | 計 |
|---------|----|----|----|----|
| sf_state | 31 | 0 | 0 | 31 |
| 推定器 | 17 | 2 | 1 | 20 |
| sf_core | 12 | 0 | 1 | 13 |
| 制御/failsafe/math | 11 | 2 | 1 | 14 |
| board/comm/api | 9 | 1 | 0 | 10 |
| notify/logger/calib/takeoff | 5 | 0 | 13 | 18 |
| tasks/main | 26 | 1 | 0 | 27 |
| **合計** | **111** | **6** | **16** | **133** |

> 「コードは設計通り（111/133 ≈ 83% が無条件 OK）。残りは大半が文書側の参照ズレ（STALE 16）で、コードの実バグは限定的（実質 NG はファームを直す案件3件）」が全体像。

---

## 4. 要対応事項

### 4-A. ユーザー判断が必要（2件のテーマ）

#### ① 安全閾値の不一致 — failsafe / 要件 §9（NG 3件）

| 項目 | requirements §9 | 実装（failsafe.hpp） | 差 |
|------|-----------------|----------------------|-----|
| 衝撃 | 3.0G × **連続2回** | `impact_accel_g = 4.0f`（単発） | 閾値+連続回数 |
| 異常角速度 | 800 deg/s × **連続2回** | `gyro_anomaly_dps = 1000.0f`（単発） | 閾値+連続回数 |
| LiPo 低電圧 | **≤3.4V** ブザー警告のみ | `low_battery_v = 3.3f` | 閾値 |
| USB 給電 | ≤3.3V ARM 禁止 | `usb_v = 3.3f`（state_manager）✅ | 一致 |

- 該当タグ: `failsafe.cpp:15`, `failsafe.hpp:21`, `power_task.cpp:26`（いずれも `requirements.md §9`）。
- **判断**: コードを要件§9 の規定値（3.0G／800dps／3.4V＋連続2回）に合わせるか、要件§9 を実装値（4.0G／1000dps／3.3V）に合わせて改訂するか。**安全に直結するためユーザー判断とする。** 閾値変更は実機の挙動・誤発報率に影響するため、変更時は CLAUDE.md 原則に従い根拠を添える。

#### ② esp_netif（WiFi STA）の所有者 — 文書間の矛盾（NG 1件）

- 該当タグ: `comm.cpp:197`（`hardware_init.md §3 R1 — sf_board が共有 HW 資源を所有`）。
- 実装: `comm.cpp:364` が `esp_netif_create_default_wifi_sta()` を**自前で呼び STA netif を生成**。
- 文書の矛盾:
  - `architecture.md §7` / `hardware_init.md §3 R1` … esp_netif は **sf_board 所有**。
  - `hardware_init.md §4` … 「WiFi STA netif の生成（実体は sf_comm が後で…）」と **comm 生成を許容**する記述。
- **判断**: どちらに統一するか。(a) board が STA netif を所有し comm は借用、(b) STA netif は comm 所有として R1 の例外を明文化。**これは Phase 7 計画でも「ユーザーと確認し一方に統一」と指定済みの項目。**（参考: 旧 vehicle や DHCP/再接続の所有との整合も考慮）

### 4-B. 機械的に修正可能（注釈側の修正で解消）

#### STALE 16件 — 参照先の節番号／見出しの修正

detailed_design.md は §8 まで・architecture.md は §7 までしか存在しないのに、存在しない節を参照しているもの。**コードは正しい**ので、タグの参照先を実在する節へ向け直す（または設計文書側に該当節を新設する）。

| ファイル:行 | 誤った参照 | 正しい参照（推奨） |
|------------|-----------|-------------------|
| eskf_core.hpp:21 | architecture.md §3 — Sensor observation switch | detailed_design.md §5 — Sensor observation switch |
| params.cpp:86 | detailed_design.md §6 — Single-macro definition | detailed_design.md §6 — Parameter table (SSOT = params.cpp) ※文言を現設計（手書き table[]）に更新 |
| sf_math.hpp:17 | detailed_design.md §7 — sf_math | architecture.md §2 — sf_math (Math library) |
| notify.cpp:14 / notify.hpp:22 | architecture.md §8 — Notification subsystem | architecture.md §2 責務#14 / §5 データフロー |
| notify.cpp:15,31 / notify.hpp:23 | detailed_design.md §10 — LED pattern table | （文書に LED パターン表を新設 or architecture §2 へ） |
| logger.hpp:22 | architecture.md §7 — Logging subsystem | architecture.md §2 責務#13 / §5 ログフロー / §6 LogTask |
| logger.hpp:23 | detailed_design.md §9 — Blackbox format and DataStream | detailed_design.md §8 — Memory Layout（Blackbox 容量）※DataStream 形式は文書未定義 |
| calibration.cpp:14 / calibration.hpp:24 | architecture.md §3 — Calibration subsystem | architecture.md §2 責務#11 |
| calibration.cpp:15 / calibration.hpp:25 | detailed_design.md §12 — Calibration procedures | （文書にキャリブ手順節を新設 or 削除） |
| takeoff_landing.cpp:15 / takeoff_landing.hpp:21 | detailed_design.md §11 — Takeoff/landing logic | detailed_design.md §3 — 状態遷移テーブル（離着陸行） |

#### NG のうち注釈文言で解消できる2件（bias freeze）

| ファイル:行 | 内容 | 対応 |
|------------|------|------|
| estimator.hpp:203 | `freezeBias()` が `detailed_design §3 onEnter(LANDING→IDLE) バイアスフリーズ` を参照するが、§3 注3 で当該フリーズは**見送り（配線しない）**と確定 | タグを「§3 注3 — bias freeze は capability 残置・未配線」に書き換え、`[OK]`（コードは設計の見送り決定どおり） |
| estimator.hpp:210 | `unfreezeBias()` 同上 | 同上 |

> コメント本文は既に「現在未配線（capability として残置）」と正しく説明している。タグの**説明文と判定**を実態（= 設計の見送り決定に従っている）に合わせるだけ。

---

## 5. 全判定結果（クラスタ別）

> 凡例: 判定列の `OK` = 設計通り実装、`NG` = 不一致、`STALE` = 参照先ズレ。`ファイル:行` は `firmware/vehicle/` からの相対。

### 5-1. sf_state（31 OK / 0 NG / 0 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| sf_state/state_manager.cpp:14 | requirements §4 #3 | OK | `StateManager` がモード遷移と ARM 許可判定を一元実装 |
| sf_state/state_manager.cpp:15 | architecture §2 | OK | private `transition()` のみ `state_` 更新＝唯一の遷移実行者 |
| sf_state/state_manager.cpp:16 | detailed_design §3 | OK | 遷移テーブル全行が request/notify メソッドに対応 |
| sf_state/state_manager.cpp:70 | requirements §2 | OK | `notifyInitComplete()` が INIT のみ IDLE_GROUND へ |
| sf_state/state_manager.cpp:132 | requirements §2 | OK | `requestDisarm()` ARMED_GROUND→IDLE_GROUND |
| sf_state/state_manager.cpp:133 | requirements §2 | OK | `requestDisarm()` は isArmed ゲートで空中 DISARM も IDLE へ |
| sf_state/state_manager.cpp:195 | requirements §2 | OK | `notifySoftLanding()` FLYING→ARMED_GROUND |
| sf_state/state_manager.cpp:205 | requirements §2 | OK | `notifyIdleGroundHeld(bool)` が IDLE_GROUND↔HELD 双方向 |
| sf_state/state_manager.cpp:250 | architecture §4 | OK | `handleAlert()` が alert→判断→遷移（FAILSAFE=イベント） |
| sf_state/state_manager.cpp:251 | requirements §9 | OK | 衝撃/異常角速度/通信断/低電圧/USB/ESKF発散を §9 表どおり分岐 |
| sf_state/state_manager.hpp:24–181（13件） | requirements §2/§4/§9, architecture §2/§4, detailed_design §3 | OK | 上記 .cpp と対応。comm-loss 3秒→LANDING（`update()` kCommLossHoverUs=3s）含め全一致 |
| sf_state/flight_state.hpp:20–129（7件） | requirements §2/§9, architecture §4 | OK | FlightState/FlightMode/AlertType enum が §2 状態遷移図・§9 異常表と一致。ARM/DISARM はアクション（`isArmed()` 派生）で状態に持たない |

### 5-2. 推定器（17 OK / 2 NG / 1 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| estimator.hpp:29–85（7件） | requirements §4/§10, architecture §2, detailed_design §5, coding §2 | OK | `IEstimator` 純粋仮想IF＝差替可能性、predict/update*/getState/reset 一致、観測スイッチ規約、バイリンガル準拠 |
| estimator.hpp:203 | detailed_design §3 onEnter(LANDING→IDLE) bias freeze | **NG** | §3 注3 で bias freeze は見送り確定。`freezeBias()` は capability 残置・未配線。タグ説明が実態と乖離 → 文言修正で OK 化 |
| estimator.hpp:210 | detailed_design §3 onEnter(TAKEOFF→FLYING) bias unfreeze | **NG** | 同上（`unfreezeBias()`） |
| estimator.hpp:223 | architecture §4 ground→flight covariance handoff | OK | `inflateCovariance(mask)` が状態 x を保持し姿勢共分散のみ膨張（SILS掃引確定） |
| eskf_estimator.hpp:14–16 / eskf_estimator.cpp:14–15（5件） | requirements §4, detailed_design §5 | OK | IEstimator 実装・観測スイッチを core へ委譲 |
| eskf_core.hpp:20 | requirements §4 #2 | OK | 15状態 [pos,vel,att_err,bg,ba] の状態推定 |
| eskf_core.hpp:21 | architecture §3 — Sensor observation switch | **STALE** | 「観測スイッチ」は detailed_design §5。architecture §3（インターフェース設計）に該当記述なし |
| eskf_core.cpp:28 | detailed_design §5 IEstimator | OK | χ²ゲート・Adaptive R・線形化バイアスも §5「ESKF実装の特性」と整合 |
| complementary_estimator.hpp:29 | requirements §10 | OK | ESKF と差替可能な2つ目の IEstimator |
| complementary_estimator.hpp:30 | coding_and_education §… 22_custom_estimator | OK | 題材は §3 Examples Plan に実在（節番号 `§…` は §3 へ確定が望ましい） |

### 5-3. sf_core（12 OK / 0 NG / 1 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| params.cpp:14,15 / params.hpp:81 | detailed_design §6, requirements §3 | OK | SSOT=params.cpp（param_vars + table[]）、get/set/save/load・命名3階層・NVS永続化・範囲検証 |
| params.cpp:86 | detailed_design §6 — Single-macro definition | **STALE** | Phase 5b で X-macro 撤去・手書き table[] が正式 SSOT。文言「Single-macro」が陳腐化 |
| topics.hpp:17–19 | architecture §3, detailed_design §2 | OK | トピック一覧網羅、extern宣言/cpp実体、R14 overflow_count、R11予約 |
| topic.hpp:22–41（4件） | architecture §3, detailed_design §2, coding §2 | OK | 軽量Pub-Sub・`Topic<T,Policy,Size>`・3バッファ方式（Latest/RingBuffer/Queue）を正確に実装 |
| data_types.hpp:21–23 | architecture §3, detailed_design §2, coding §2 | OK | 全構造体のフィールド/型/単位/timestamp 一致、バイリンガル準拠 |

### 5-4. 制御 / failsafe / math（11 OK / 2 NG / 1 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| controller.hpp:30–55（6件） | requirements §4/§10, architecture §2, detailed_design §4, coding §2 | OK | `IController::compute/reset/onModeChange`＝差替可能な統一IF。型安全な `onModeChange(FlightMode)` は設計より厳格 |
| pid_controller.cpp:22,23 / pid_controller.hpp:14,15 | requirements §4, detailed_design §4 | OK | `PidController : IController` カスケード実装 |
| pid.hpp:23 | detailed_design §4 — 離散化方式 | OK | 積分=台形・微分=bilinear（α=2ηTd/dt 等）・Kp は filter 外、η=0.125 まで式が完全一致 |
| failsafe.cpp:15 | requirements §9 | **NG** | impact 4.0G(≠3.0G)・gyro 1000dps(≠800)・low 3.3V(≠3.4V)・連続2回判定なし |
| failsafe.hpp:21 | requirements §9 | **NG** | `FailsafeConfig` 既定値が §9 規定値と不一致（同上） |
| sf_math.hpp:17 | detailed_design §7 — sf_math | **STALE** | §7（ディレクトリ構造）に sf_math 記載なし。sf_math は architecture §2 対応表に記述 |

### 5-5. board / comm / api（9 OK / 1 NG / 0 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| board.cpp:14–16 / sf_board.hpp:22–24（6件） | hardware_init §3/§4/§5, architecture §7 | OK | sf_board が I2C/SPI/LEDC/event loop/netif を所有・getter公開、起動順 L0→L3、Critical=fat() halt（esp_restart せず）、Optional=sensor_present |
| comm.cpp:197 | hardware_init §3 R1 — sf_board 所有 | **NG** | `initWifi()`(comm.cpp:364) が `esp_netif_create_default_wifi_sta()` を自前生成。netif 所有=board とする §3 R1/architecture §7 と矛盾（hardware_init §4 とは整合し得る＝文書間矛盾） |
| sf_api.hpp:35–37（3件） | architecture §2.5, coding §2, topic_reference §4 | OK | `sf::api::*` が L1 入口（Topic subscribe/publish）、`sf::internal` 非露出（R8）、Latest/RingBuffer 安全 peek |

> 付帯所見（タグ対象外）: `sf_api.hpp:135` の `current_mode()` doc コメントが FlightState を「ARMED/EMERGENCY」と古い名で列挙（実体は `ARMED_GROUND`、`EMERGENCY` は別 enum）。コメント文言の陳腐化として将来修正候補。

### 5-6. notify / logger / calibration / takeoff_landing（5 OK / 0 NG / 13 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| notify.cpp:14 / notify.hpp:22 | architecture §8 — Notification subsystem | **STALE** | architecture は §7 まで。§8 不在（通知は §2 責務#14 / §5） |
| notify.cpp:15,31 / notify.hpp:23 | detailed_design §10 — LED pattern table | **STALE** | detailed_design は §9 まで。§10 と LED パターン表が不在（`kPatternTable` 自体は妥当） |
| notify.hpp:24 | coding §2 — Bilingual comments | OK | §2 実在・準拠 |
| logger.hpp:22 | architecture §7 — Logging subsystem | **STALE** | §7 は「HW初期化と所有権」。ロガーは §2 責務#13 / §5 ログフロー / §6 LogTask |
| logger.hpp:23 | detailed_design §9 — Blackbox/DataStream | **STALE** | §9 は Guidance/Navigation 予約。Blackbox は §8 メモリ配置に容量のみ、DataStream 形式は未定義 |
| logger.hpp:24 | coding §2 | OK | バイリンガル準拠 |
| calibration.cpp:14 / calibration.hpp:24 | architecture §3 — Calibration subsystem | **STALE** | §3 はインターフェース設計。キャリブは §2 責務#11 |
| calibration.cpp:15 / calibration.hpp:25 | detailed_design §12 — Calibration procedures | **STALE** | §12 不在（detailed_design は §9 まで） |
| calibration.hpp:26 | coding §2 | OK | バイリンガル準拠 |
| takeoff_landing.cpp:14 / takeoff_landing.hpp:20 | architecture §4 — State machine transitions | OK | §4 実在。離着陸/接地/held 検出が StateTask 遷移へ供給（検出と判断の分離） |
| takeoff_landing.cpp:15 / takeoff_landing.hpp:21 | detailed_design §11 — Takeoff/landing logic | **STALE** | §11 不在。離着陸遷移は §3 状態遷移テーブルに記述 |
| takeoff_landing.hpp:22 | coding §2 | OK | バイリンガル準拠 |

### 5-7. tasks / main（26 OK / 1 NG / 0 STALE）

| ファイル:行 | 参照 | 判定 | 根拠（要約） |
|---|---|---|---|
| state_task.cpp:20–368（7件） | architecture §2/§4/§6, detailed_design §8, requirements §2 | OK | 優先度22・イベント駆動＋50Hz RC poll、唯一の遷移実行者、検出/判断分離、FAILSAFE=イベント |
| control_task.cpp:22–125（5件） | architecture §5/§6, detailed_design §8, coding §2 | OK | 400Hz IMU同期・優先度23・8KB、制御＋アクチュエーション統合、1関数1責務 |
| power_task.cpp:26 | requirements §9 | **NG** | wire する failsafe の LiPo 警告が 3.3V（要件§9=3.4V）。USB ARM禁止(3.3V)は一致 |
| notify_task.cpp:22,23 | architecture §2, detailed_design §8 | OK | LED/ブザー直接駆動・30Hz・優先度8・4KB |
| tasks.hpp:20,21 | architecture §6, detailed_design §8 | OK | 14タスク宣言・周期/優先度コメント一致 |
| main.cpp:24,25 | architecture §4, detailed_design §3 | OK | Phase 0-4 宣言・遷移は StateTask、onEnter(IDLE)→Recalibrate |
| config.hpp:27–208（8件） | requirements §2/§3/§8, detailed_design §3/§7/§8, architecture §6 | OK | 固定param=constexpr、優先度14定数一致、RAM合計92KB一致、タイミング要件一致、TAKEOFF閾値 |

---

## 6. 次のアクション

1. **要ユーザー判断（4-A）**: ①failsafe 安全閾値（§9）の整合方針、②esp_netif STA 所有の統一先 — 決定後にコード or 文書を修正。
2. **機械的修正（4-B）**: STALE 16件のタグ参照を実在節へ修正、bias freeze 2件の文言修正 → これらは `[--]`/`[NG]` を `[OK]` 化できる。
3. **`[--]`→`[OK]` 反映**: 上記を反映後、検証済み 111件 ＋ 修正で OK 化する 18件（STALE 16＋bias 2）を `[OK]` に更新。NG 残（failsafe/netif）はユーザー判断で決着後に反映。

> 本レポートは検証の記録であり、ソース注釈の一括更新（`[--]`→`[OK]`）は次ステップ。安全閾値・設計矛盾の決定を待ってから実施する。

---

## 7. 対応結果（2026-06-08 反映済み）

ユーザー判断（4-A）を受け、以下を実施。**@design タグは全て `[OK]`（243件中 242件が状態付き＝全[OK]、残1件は状態を持たない説明用 @design 行）、`[--]`/`[NG]` は 0。**

### 7-A. ユーザー決定の反映（NG 6件 → 解消）

| 項目 | 決定 | 実装 | コミット |
|------|------|------|---------|
| failsafe 安全閾値（§9, NG×3） | **コードを要件に合わせる** | impact 4.0→3.0G、gyro 1000→800dps、LiPo 警告 3.3→3.4V、連続2回デバウンス（consecutive_count=2）を実装。critical 3.0V 緊急着陸は追加安全網として維持 | `10b96d1` |
| esp_netif STA 所有（NG×1） | **board 所有に統一** | board::init() が STA netif を生成し `sta_netif()` で公開、comm は借用に変更（自前生成を撤去）。SILS に esp_wifi_default.h shim 追加 | `a7810a3` |
| bias freeze/unfreeze（NG×2） | 設計§3 注3（見送り）に整合 | タグを「capability retained, NOT wired (dropped)」に書き換え（コードは見送り決定どおり no-op 残置）→ [OK] | （本バッチ） |

### 7-B. STALE 16件の参照修正（→ [OK]）

存在しない節（detailed_design §10/§11/§12、architecture §8 等）や節タイトル不一致の参照を、実在節へ向け直した（コードは元から正しい）。

| ファイル | 修正前 → 修正後 |
|---------|----------------|
| eskf_core.hpp:21 | architecture §3 → detailed_design §5（Sensor observation switch） |
| params.cpp:86 | §6 Single-macro definition → §6 Parameter table (SSOT = params.cpp) |
| sf_math.hpp:17 | detailed_design §7 → architecture §2（sf_math Math library） |
| notify.cpp/hpp（4件） | architecture §8 / detailed_design §10 → architecture §2 #14 / §5（Notify data flow） |
| logger.hpp（2件） | architecture §7 / detailed_design §9 → architecture §2 #13 / detailed_design §8（Memory Layout） |
| calibration.cpp/hpp（4件） | architecture §3 / detailed_design §12 → architecture §2 #11 / detailed_design §3（onEnter(IDLE_GROUND)） |
| takeoff_landing.cpp/hpp（2件） | detailed_design §11 → detailed_design §3（State transition table TAKEOFF/LANDING） |

### 7-C. 残課題（設計文書側の拡充・任意）

トレーサビリティは実在節へ向け直して解消したが、以下は**文書側に詳細節が未作成**（将来の文書拡充候補。コード・タグは正しい）:
- LED パターン表（notify）の詳細仕様節
- DataStream 形式（logger）の定義節
- キャリブレーション手順（calibration）の詳細節
- failsafe の「連続2回」は PowerTask レート(10Hz)で約200ms継続 — 極短の衝撃スパイク取りこぼしの可能性（failsafe.cpp にコメントで明記、レート見直しは将来課題）

検証: 全フェーズで vehicle 11シナリオ＋legacy hover_espnow＋hover_smoke G2+G3＋ESP-IDF 実機ビルド 全PASS。@design タグ flip はコメントのみ（SILS ビルド OK で確認）。
