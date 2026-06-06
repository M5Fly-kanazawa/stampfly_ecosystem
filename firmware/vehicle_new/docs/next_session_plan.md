# 次セッション指示書 — 実機に向けた地固め（P1 → P2 → P3）

最終更新: 2026-06-06

## 0. 現状サマリ（ここまで達成）

- **vehicle_new は SIL で全4飛行モードが成立**：ACRO / STABILIZE / ALTITUDE_HOLD / POSITION_HOLD。
- **POS_HOLD は全4軸（roll/pitch/斜め/yaw）でタイト保持**（運動加速度補償 + `position.vel.kp=0.8`）。詳細は `poshold_accel_compensation.md`。
- **数値ゲート（G1〜G4）の試験スイートが整備済**：`simulator/sil/scenarios/TEST_MATRIX.md`。
- 次の本来の山は **roadmap Phase 2/3（実機）**。本指示書は**そこへ向けた地固め**を P1→P2→P3 の順で行う。

### 着手前に読む
1. `firmware/vehicle_new/docs/development_roadmap.md`（Phase 2/3、SIL→実機 Code Identity）
2. `firmware/vehicle_new/docs/poshold_accel_compensation.md`（直近の成果と残課題）
3. `simulator/sil/scenarios/TEST_MATRIX.md`（回帰の回し方）
4. 直近コミットログ（`d8b5a37` まで）

### 回帰の回し方（各 P の作業後に必ず実行）
```bash
source setup_env.sh
sf sil build vehicle_new
for s in pos_roll pos_pitch pos_flight pos_yaw alt_flight stab_flight acro_flight disturb; do
  sf sil scenario simulator/sil/scenarios/$s.scn --target vehicle_new
done
sf sil scenario simulator/sil/scenarios/hover_espnow.scn --target vehicle   # legacy 無回帰
simulator/sil/build/hover_smoke simulator/sil/models/stampfly.xml           # G2+G3
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

| # | 項目 | 現状 | やること |
|---|------|------|----------|
| P2-1 | **power INA3221 実配線** | `tasks/power_task.cpp:69` が `data.voltage = 4.2f; // Stub` の固定値。実ドライバ `components/sf_hal_power/power_monitor.cpp` は存在 | power_task で実 INA3221 ドライバを呼び `sensor_power` に実電圧を publish。emu は INA3221 をモデル済み（memory）なので SIL で検証可 |
| P2-2 | **Failsafe 配線** | `components/sf_failsafe/`（failsafe.cpp/.hpp）は実装済。**配線状況を要確認** | failsafe が ①通信断 ②低電圧（P2-1 の実電圧が前提）③衝撃 を検知し EMERGENCY/着陸/キルへ遷移するか確認。未配線なら state_manager/タスクへ接続。`disturb.scn` の fault 注入で検証 |
| P2-3 | **CalibrationMgr 起動配線** | `components/sf_calibration/` は存在。**起動シーケンスで参照されているか要確認**（memory「未参照」） | 起動時にジャイロ/加速度バイアスを取り込む経路を配線（`hardware_init.md` の起動シーケンス参照） |
| P2-4 | **ARM 前チェック** | 要確認 | ARM 受理前にセンサ健全性・電圧・キャリブ済みを確認するゲートを state_manager の requestArm 経路に追加 |

### 進め方の原則
- **1項目ずつ**：配線 → 回帰（上記スイート）→ コミット。まとめてやらない。
- 可能なら**SIL で発火を確認**：P2-1 は emu の INA3221 で実電圧、P2-2 は `disturb.scn`（横風＋モータ故障）を拡張して低電圧/通信断シナリオを追加し、failsafe 遷移を数値ゲート（`log_contains "EMERGENCY"` 等）で検証。
- **設計矛盾を見つけたら実装を止めて報告**（CLAUDE.md vehicle_new 原則）。

### 合格基準
- P2-1: SIL で実 INA3221 経路の電圧が `sensor_power` に出る（emu の電池電圧と一致）。
- P2-2: 低電圧/通信断/衝撃の各シナリオで failsafe が正しい遷移（着陸 or キル）を起こす（ゲート化）。
- P2-3/P2-4: 起動・ARM 経路に配線され、回帰が全 PASS。

---

## P3 ＝ 技術的負債の解消：離陸判定の統一

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

## メモ
- 各 P の作業後は**必ず回帰スイート全 PASS を確認 → `/commit`（Next steps 必須）**。
- ESKF/制御パラメータを変える提案は**必ず SIL の数値シミュレーションで裏付けてから**（CLAUDE.md 原則）。
- P2 の安全機能は実機で命に関わるので、**スタブを「動いたつもり」にしない**こと（emu で実経路を発火させて確認）。
