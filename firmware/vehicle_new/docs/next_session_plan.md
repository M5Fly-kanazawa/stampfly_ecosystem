# 次セッション指示書 — 地固め完了 → 次は実機ブリングアップ

最終更新: 2026-06-06（**地固め P1〜P3 全完了**。次は roadmap Phase 2/3 実機）

> **本指示書の P1〜P3 は全て完了しました（下記）。次セッションは「実機ブリングアップ」または「P2 follow-up」から始めること。** 末尾「次フェーズ」参照。

## 0. 現状サマリ（ここまで達成）

- **vehicle_new は SIL で全4飛行モードが成立**：ACRO / STABILIZE / ALTITUDE_HOLD / POSITION_HOLD。
- **POS_HOLD は全4軸でタイト保持**（運動加速度補償 + `position.vel.kp=0.8`）。詳細 `poshold_accel_compensation.md`。
- **数値ゲート試験スイート整備済**：`simulator/sil/scenarios/TEST_MATRIX.md`。

### 本指示書の進捗（2026-06-06 セッション）
- ✅ **P1 完了** — `sf build vehicle_new` ESP-IDF 実機ビルド検証（944KB、警告のみ・良性）。
- ✅ **P2-1 完了**（commit `b578216`）— INA3221 実配線。emu I2C モデル経由で実電圧 3.70V を `sensor_power` に発行（Code Identity）。
- ✅ **P2-2 完了**（commit `aa3f7c1`）— Failsafe を PowerTask に配線（デッドコードだった）+ `checkCommTimeout()` 実装。`commloss.scn` で COMM_LOST→FLYING→LANDING を E2E ゲート化。
- ✅ **P2-3 完了**（commit `dcb4118`=ファーム, `436e007`=SIL）— 起動ジャイロ/加速度バイアス校正を ImuTask に配線。**対照試験で実証**: 生バイアス(accel 0.12/gyro 0.02)注入下で校正ON=tilt 3.2°有界 / OFF=tilt 176°転倒。
  - 重要発見: **params.cpp の手書き table[] が真の SSOT、params.def は非機能**（auto-memory `reference_params_ssot`）。新 param は params.cpp 両所に追加必須。
  - 校正の設計上の急所: disarmed・地上のみ収集（arm で中止）、settle 過渡破棄、**デッドバンドで無視可能バイアスは適用せず**（収束済み ESKF 上書き＝marginal POS_HOLD 撹乱を回避）。
- ✅ **P2-4 完了**（commit `4109ccf`）— requestArm に ARM 前チェック: ①USB/低電圧禁止（実電圧 ≤ 3.3V で reject）②キャリブ済みゲート（system_status トピックで公開）。`prearm.scn` で「校正中=拒否 / 完了後=受理」を実証。③センサ健全性は M2b 繰延。
- ✅ **P3 完了**（commit `6ce498f`）— state_task の dwell 暫定離陸判定を TakeoffLandingMgr の ToF 検出に統一（system_status.airborne 経由）。TAKEOFF_DWELL_MS 除去。全モードで離陸完了が ToF 高度交差で発火。
- 🎯 **地固め完了 → 次は実機ブリングアップ（roadmap Phase 2/3）。末尾「次フェーズ」参照。**

### 着手前に読む
1. `firmware/vehicle_new/docs/development_roadmap.md`（Phase 2/3、SIL→実機 Code Identity）
2. 直近コミットログ（`436e007` まで）— P2-3 の対照試験と設計判断
3. auto-memory `reference_params_ssot`（params 追加の罠）

### 回帰の回し方（各 P の作業後に必ず実行 — コマンド更新済）
```bash
source setup_env.sh
sf sil build                                              # 注: 旧表記 "build vehicle_new" は誤り。-t は cmake target
for s in pos_roll pos_pitch pos_flight pos_yaw alt_flight stab_flight acro_flight disturb commloss calib; do
  sf sil scenario simulator/sil/scenarios/$s.scn --target vehicle_new
done
sf sil scenario simulator/sil/scenarios/hover_espnow.scn --target vehicle   # legacy 無回帰
simulator/sil/build/hover_smoke simulator/sil/models/stampfly.xml           # G2+G3
sf build vehicle_new                                      # ← ESP-IDF 実機ビルド（ファーム変更時は必須）
```
全 PASS を確認してからコミット（`/commit`、Next steps 必須）。

---

## P1 ＝ 最優先：実機 ESP-IDF ビルド検証

### 目的
`vehicle_new` が **ESP32 ターゲットでビルドできる**ことを確認する。これが通らないと実機の話が全て止まるハードゲート。

### なぜ最優先
host SIL（clang/host）では全ソースが通るが、**ESP-IDF（xtensa-gcc + FreeRTOS + 実ドライバ + リンカ）は別物**。今回追加した α-β コード・params・`ESP_LOGI` は plain C++ なので通る見込みだが**未検証**。短時間で確認でき、コケたら最優先で直す。

### 手順
```bash
source setup_env.sh
sf build vehicle_new          # ← これが本体。idf.py build 相当
```
- エラーが出たら型・ヘッダ・FreeRTOS API・リンカ未解決を1つずつ潰す。
- **SIL 専用コードが firmware に混入していないか確認**：`simulator/sil/` の診断グルー（`emu_vehicle_new_glue.cpp`、`SIL_EMU_ESKF_DIAG`）は SIL 側で firmware ビルドには入らない。firmware 側（`firmware/vehicle_new/`）に `getenv`/`<cstdlib>`/SIL 専用 env が残っていないか grep で確認（クリーンアップ済みのはずだが念のため）：
  ```bash
  grep -rn "getenv\|SIL_EXP\|SIL_EMU\|SIL_TUNE" firmware/vehicle_new/ | grep -v docs
  ```
  （ヒットしないこと。ヒットしたら除去。）

### 合格基準
- `sf build vehicle_new` がエラー0で完了。
- （ハードがあれば）`sf flash vehicle -m` で起動ログ（`ESKF initialized (... accel_comp=1 ...)` 等）が出ること。

### 注意点
- host SIL 通過 ≠ ESP-IDF 通過。警告も確認。
- パラメータ NVS：実機で `eskf.accel_comp.*` / `position.vel.kp=0.8` の既定値が効くか（NVS に古い値が残っていれば上書きされる可能性）。初回は NVS クリアを検討。

---

## P2 ＝ 実機で飛ばす前に必須：flight-readiness（安全機能）配線

### 目的
SIL では飛ぶが、**実機で安全に飛ばすための機能**を配線する。実機飛行（最終マイルストーン）の直接の前提。

### サブタスク（現状を確認 → 配線 → SIL で検証）

| # | 項目 | 状態 | 結果 / やること |
|---|------|------|----------|
| P2-1 | **power INA3221 実配線** | ✅ 完了 `b578216` | power_task が実 INA3221 を呼び `sensor_power` に実電圧。SIL で `Battery: 3.70V`（emu v_batt と一致）。Optional 分類・読み失敗ガード。 |
| P2-2 | **Failsafe 配線** | ✅ 完了 `aa3f7c1` | Failsafe を PowerTask に配線（update() で battery/comm/impact/gyro）。`checkCommTimeout()` を CommandSetpoint 経過時間(R16)で実装。`commloss.scn` で COMM_LOST→LANDING を E2E ゲート化。**低電圧/衝撃の専用ゲートは emu knob 待ち（一部 P2-3a で電池以外のバイアス knob 整備済、電池電圧 knob は未）**。 |
| P2-3 | **CalibrationMgr 起動配線** | ✅ 完了 `dcb4118`+`436e007` | ImuTask に in-loop 校正を配線（disarmed・地上のみ・arm 中止・settle・デッドバンド）。emu に生バイアス注入(`bias` scn イベント)+`SIL_EMU_NO_CALIB`。`calib.scn` 対照: ON=tilt3.2°有界 / OFF=tilt176°転倒。 |
| P2-4 | **ARM 前チェック** | ✅ 完了 `4109ccf` | requestArm に ①USB/低電圧禁止（実電圧 ≤ safety.battery.usb_v で reject、0=不明は許可）②キャリブ済みゲート（`system_status` トピックで ImuTask が公開）を実装。`prearm.scn` で実証。③センサ健全性は M2b（sensor_present）繰延。 |

### 進め方の原則
- **1項目ずつ**：配線 → 回帰（上記スイート）→ コミット。まとめてやらない。
- 可能なら**SIL で発火を確認**：P2-1 は emu の INA3221 で実電圧、P2-2 は `disturb.scn`（横風＋モータ故障）を拡張して低電圧/通信断シナリオを追加し、failsafe 遷移を数値ゲート（`log_contains "EMERGENCY"` 等）で検証。
- **設計矛盾を見つけたら実装を止めて報告**（CLAUDE.md vehicle_new 原則）。

### 合格基準
- P2-1: SIL で実 INA3221 経路の電圧が `sensor_power` に出る（emu の電池電圧と一致）。
- P2-2: 低電圧/通信断/衝撃の各シナリオで failsafe が正しい遷移（着陸 or キル）を起こす（ゲート化）。
- P2-3/P2-4: 起動・ARM 経路に配線され、回帰が全 PASS。

---

## P3 ＝ 技術的負債の解消：離陸判定の統一 ✅ 完了 `6ce498f`

**結果**: state_task の dwell を TakeoffLandingMgr の ToF 検出（`system_status.airborne`）に統一。`TAKEOFF_DWELL_MS` 除去。全モードで離陸完了が ToF 高度交差で発火（全回帰 PASS）。以下は記録（着手時の計画）。

### 目的
state_task の「dwell による暫定離陸判定」を **TakeoffLandingMgr の ToF 検出に統一**する（離陸判定ロジックが2本ある状態を1本化）。

### 現状
- `tasks/state_task.cpp:138` に **`TODO(Phase B): replace the dwell with TakeoffLandingMgr ToF detection`** と明記。
- 現状は TAKEOFF に `config::TAKEOFF_DWELL_MS`（500ms）留まる estimator 非依存 dwell。
- `tasks/imu_task.cpp:77` に `g_takeoff_landing`（TakeoffLandingMgr）が既に存在し、ALT_HOLD の鉛直ハンドオフに配線済（今回の POS_HOLD で整備済）。

### やること
- state_task の TAKEOFF→FLYING 判定を、TakeoffLandingMgr の ToF ベース離陸検出（高度しきい値）に置き換える。
- 2つの離陸判定（dwell と ToF）を1本化し、`TODO(Phase B)` を解消。
- ToF が信頼できる前提（POS_HOLD で鉛直推定は成立済）。

### 合格基準
- stab/acro/alt/pos の全シナリオが、統一後の離陸判定で全 PASS（特に `Takeoff complete` のタイミングが破綻しないこと）。
- dwell 関連の暫定コード（state_task.cpp の dwell 用変数）が除去され、コードが1本化される。

### 注意点
- 離陸検出タイミングが変わると `.expect` の `order` チェック（`Takeoff complete` の前後関係）に影響しうる。閾値・窓を再確認。

---

## P4 以降（別フェーズ・本指示書の対象外だが記録）

| 優先度 | 項目 | 概要 |
|--------|------|------|
| P4 | POS_HOLD 入口過渡の整定 | 残課題1。~10秒の減衰を締める（vel の td / 位置の ti）。低リスクの上積み・脆弱領域注意 |
| P5 | n1/n2 振動処理 | 残課題2。ノッチ、フローノイズモデル精緻化 |
| P6 | データ駆動ノイズ化 | `project_sil_noise_data_driven` |
| P7 | 実機ブリングアップ | roadmap Phase 2→3（HAL接続→ACRO初飛行）= 本丸 |

---

## 次フェーズ（地固め完了後の着手候補）

地固め（P1〜P3）は完了。次セッションは以下のいずれかから始める。

### A. 実機ブリングアップ（本丸 — roadmap Phase 2→3）
- `sf flash vehicle -m` で起動ログ確認（`ESKF initialized (... accel_comp=1 ...)`、`Boot calibration applied: ...`、`INA3221 ready`、`Battery: x.xxV`）。
- **初回は NVS クリア**を検討（古い params が残ると `position.vel.kp=0.8` 等の既定が効かない）。
- 静止して起動 → 起動校正が実バイアスを測るか（`Boot calibration applied` のバイアス値が妥当か）。
- ARM 前チェック実機確認: USB 給電のみで ARM 拒否されるか、校正完了まで ARM 拒否されるか。
- HAL 接続 → ACRO 初飛行（roadmap Phase 4.1）。

### B. P2 follow-up（SIL 忠実度・安全機能の上積み）
- **emu 電池電圧 knob**: P2-3a の `bias` イベントと同様に電池電圧を可変にし、①P2-4 の USB/低電圧 ARM 禁止 ②P2-2 の LOW_BATTERY failsafe を SIL でゲート化（現状は emu v_batt 固定 3.7V で発火不可）。
- **Failsafe が `safety.*` params を読む**: 現状 FailsafeConfig のハードコード既定値を使用。params.cpp に `safety.*` は登録済なので接続するだけ。
- **センサ健全性 ARM ゲート**: M2b の `sf_board::sensor_present()` 実装後に requestApt ゲート3を配線。
- **衝撃 failsafe の SIL 注入**: 4G 衝撃を emu で生成する手段（接触イベント等）。

### C. P4〜P6（別フェーズ・性能/研究）
| 項目 | 概要 |
|------|------|
| POS_HOLD 入口過渡の整定 | ~10秒の減衰を締める（vel td / 位置 ti）。脆弱領域注意（[[reference_params_ssot]] の通り params.cpp で調整） |
| n1/n2 振動処理 | ノッチ、フローノイズモデル精緻化 |
| データ駆動ノイズ化 | `project_sil_noise_data_driven` |

---

## メモ
- 各作業後は**必ず回帰スイート全 PASS を確認 → `/commit`（Next steps 必須）**。回帰コマンドは §0 の更新版を使う（`sf sil build` + commloss/calib/prearm 追加、最後に `sf build vehicle_new`）。
- ESKF/制御パラメータを変える提案は**必ず SIL の数値シミュレーションで裏付けてから**（CLAUDE.md 原則）。
- 安全機能は実機で命に関わるので、**スタブを「動いたつもり」にしない**こと（emu で実経路を発火させて確認）。
- **params 追加は params.cpp の param_vars + table[] 両方に**（params.def は非機能。auto-memory `reference_params_ssot`）。
