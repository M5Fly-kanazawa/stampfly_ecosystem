# StampFly 統合SIL 検証レポート — 診断計器ロードマップ（M1〜M11）

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

本レポートは、StampFly の SIL（Software-in-the-Loop）を**単なる動作確認の場から「診断計器」へ昇華させる**取り組みのロードマップ（M1〜M11）の成果と検証結果を1枚にまとめたものである。M1〜M3 のみを扱った旧レポート（`M1-M3_validation_report.md`）を置き換える統合版である。

旧 `firmware/vehicle/` では状態推定（ESKF）と状態遷移管理に起因する不明バグが発生し、これがファームウェア刷新（vehicle_new）の最大の動機となった。本取り組みは、**本体C++コードを無改変で参照コンパイル（Code Identity）**しながら、PC 上で制御則と状態機械を決定論的に検証し、さらに**旧ファームの既知バグを SIL で再現できるか（回帰チャレンジ）**で SIL の検出力そのものを較正する。

### 対象読者

- vehicle_new の開発者（人間 + AI）
- SIL を用いて制御系・状態機械を設計・検証する研究者・学生
- 旧機の不明バグの原因切り分けを追う者

### 達成サマリー

| マイルストーン | 内容 | 状態 |
|--------------|------|:---:|
| **M1** | PC化基盤（ESP-IDF 互換シム + トピック実体） | ✅ |
| **M2** | 状態遷移ユニットテスト 47件 | ✅ |
| **M3** | StateManager 駆動 統合SIL（5シナリオ） | ✅ |
| **M4** | 閉ループ化（`--feedback truth\|eskf`）+ gust 注入 | ✅ |
| **M5** | 診断の梯子（N0〜N2、要因分離） | ✅ |
| **M6** | 制御設計検証（評価指標・A/Bスイープ・`control/` 配置） | ✅ |
| **M11-1** | 旧ESKF（active_mask型）をホストで無改変参照コンパイル | ✅ |
| **M11-2** | 回帰チャレンジ：旧バグ A/C 再現（陽性較正）+ D/E 陰性較正 | ✅ |
| **M11-3** | redesign A/B：新ESKFで A/C が直ることを同一入力で定量証明 | ✅ |
| M7 | 差分診断（実機ログ注入・実機↔SIL残差） | ⬜ 未着手 |
| M8 | Model Fidelity 校正（実機ログ照合） | ⬜ 未着手 |
| M9 | ファーム検討の拡充（起動・failsafe連鎖・遷移カバレッジ） | ⬜ 未着手 |
| M10 | sf CLI 統合（host-SIL バックエンド） | ⬜ 未着手 |

### SIL 設計思想 — 3本柱

| 柱 | 内容 | 本レポートでの実証 |
|----|------|------------------|
| **回帰チャレンジ（北極星）** | 旧ファームの既知バグを SIL で再現・検出できるか＝SILの真価の最終受け入れ基準 | §6（M11） |
| **差分診断** | Code/Parameter Identity ゆえ「SILで再現→ソフト要因／非再現→HW・並行性・通信要因」と切り分けられる | §6.2 陰性較正、§7 |
| **診断の梯子** | 層（L1〜L4 / ノイズ N0〜N4 / HW・並行性境界）を段階的にON/OFFし、どの層で再現するかで要因を特定 | §5（M5/M6） |

## 2. 設計原則 — 参照による Code Identity

本取り組みの中核は **「分離しているのに本体と同じコードが走る」** ことである。

- SIL ハーネスは `simulator/sil/` に独立配置するが、ファーム本体（`eskf_core.cpp` / `state_manager.cpp` / `failsafe.cpp` 等）を **コピーせず相対パス参照でコンパイル**する。
- ESP-IDF 依存（FreeRTOS / esp_timer / esp_log、回帰チャレンジでは旧ESKF用の `esp_err.h`）は `compat/` シムで吸収し、**本体ソースは1行も改変しない**。
- 全工程を通じて `git diff firmware/` が空であることを Code Identity の判定基準とした。**新ファーム（vehicle_new）だけでなく、回帰チャレンジで参照する旧ファーム（vehicle）にも同じ鉄則を適用している。**

```
firmware/vehicle_new/        ← 新ファーム（無改変）
   ├ components/sf_state/state_manager.cpp       ┐
   ├ components/sf_failsafe/failsafe.cpp         │ 相対パス参照でコンパイル
   └ components/sf_estimator_eskf/eskf_core.cpp  ┘
firmware/vehicle/            ← 旧ファーム（無改変、回帰チャレンジ用）
   └ components/sf_algo_eskf/eskf.cpp            ┐ compat/esp_err.h 経由で参照
simulator/sil/               ← SILハーネス（独立ツール環境）
   ├ compat/                 ← ESP-IDF互換シム（新旧本体を無改変で通す）
   ├ sil_topics.cpp          ← トピックextern実体（params.cpp と同期）
   ├ integrated_sil.cpp      ← StateManager駆動 統合ループ（M3〜M6）
   ├ old_eskf_regression.cpp ← 旧/新ESKF 並走 A/B（M11）
   └ scenario.hpp / battery_model.hpp
```

## 3. ロードマップと進捗

最終ゴールは2系統 —（1）開発ファームの検討、（2）制御設計の正しさの検証。これらから逆算した依存関係に沿って着実順で進めている。

```
M3(済)─┬─M4 閉ループ化──M5 診断梯子─┬─M6 制御設計検証(ゴール2)
       │                            └─M7 差分診断──M8 Model Fidelity
       └─M9 ファーム検討(ゴール1, M4と並行可)
                          M5,M7,M8──M10 sf CLI統合
            M5,M7,M9,M10──────────────M11 回帰チャレンジ(北極星)
```

北極星である M11 を先行実施し、**「SILでバグ発見 → 修正 → SILで直ったと確認」のループが旧→新の実例で一周した**。M7〜M10（実機ログを要するフェーズ／ツール化）は残作業（§8）。

## 4. M1〜M3 — 基盤と状態遷移

### M1 — PC化基盤

`compat/` ディレクトリが最小の ESP-IDF 互換ヘッダを提供する。

| ファイル | 役割 |
|---------|------|
| `compat/freertos/FreeRTOS.h` | `TickType_t` / `pdTRUE` / `portMAX_DELAY` 等の基本型・定数 |
| `compat/freertos/semphr.h` | `SemaphoreHandle_t` を `std::mutex` にマップ |
| `compat/freertos/queue.h` | `QueueHandle_t` を byte-copy `std::deque` にマップ |
| `compat/esp_timer.h` | `esp_timer_get_time()` を `std::chrono` で実装 |
| `compat/esp_log.h` | `ESP_LOGx` を stderr へ（CSV stdout を汚染しない） |
| `compat/esp_err.h` | `esp_err_t` / `ESP_OK` 等（M11 で旧ESKF用に追加） |
| `sil_topics.cpp` | トピックextern実体12個 + `topics_init()`（`params.cpp` と同期） |

`make check` で最小 main が無改変の StateManager を `INIT → IDLE_GROUND → ARMED_GROUND` と遷移させる（"M1 OK"）。`git diff firmware/` は空。

### M2 — 状態遷移ユニットテスト

`firmware/vehicle_new/test/test_state_manager.cpp` に決定論ユニットテスト47件を実装。**全47件 PASS**。

| カテゴリ | 件数 | 検証内容 |
|---------|:---:|---------|
| A. 正常遷移 | 11 | INIT→IDLE→ARMED→TAKEOFF→FLYING→LANDING→IDLE 全エッジ + disarm / soft-landing / held↔ |
| B. ガード拒否 | 12 | 不正な現在状態からの遷移要求が `false` / 状態不変（二重ARM、INITからARM 等） |
| C. モード変更 | 3 | FLYING 内 `requestModeChange` 全モード、同一モード冪等 |
| D. 全Alert分岐 | 10 | IMPACT/GYRO_ANOMALY×(空中/地上)、COMM_LOST×(FLYING/非FLYING)、LOW_BATTERY/USB_POWER/ESKF_DIVERGED/NONE |
| E. コールバック | 5 | onExit→onEnter 順序、同一遷移で非発火、onModeChange、複数登録順、MAX_CALLBACKS(8)上限 |
| F. Failsafe閾値 | 5 | impact 4G ラッチ、gyro 1000dps、battery WARNING(3.3V)/EMERGENCY(3.0V)/未接続(≤0.1V) |
| G. エンドツーエンド | 1 | FLYING→高G→`failsafe.update`→`system_alert`→`handleAlert`→IDLE_GROUND |
| **合計** | **47** | **全PASS** |

> **補足:** 既存 `test_main.cpp` の `pid_integral` が1件 FAIL（0.95 vs 1.0）。本作業より前から存在する別件で、状態遷移テストとは無関係。

### M3 — StateManager 駆動 統合SIL

旧 `sil_main.cpp` の偽状態管理（`bool armed` + ハードコード時刻 `T_ARM`）を撤廃し、**無改変の本物の StateManager / Failsafe をループで駆動**する。物理 + ESKF + PID + 状態遷移 + failsafe が一気通貫で動作し、フライト状態がモーター制御ゲートと制御モードの唯一の真実となる。固定シード（`srand(1)`）で決定論的。

| シナリオ | 注入 | 期待挙動 | 遷移数 | 最終状態 |
|---------|------|---------|:---:|---------|
| nominal | なし | 正常飛行（離陸→ホバー→着陸） | 6 | IDLE_GROUND |
| comm_lost | t=8s COMM_LOST | FLYING → LANDING → 着陸 | 6 | IDLE_GROUND |
| impact | t=8s 8G | **緊急 IDLE**（再離陸なし） | 5 | IDLE_GROUND |
| low_battery | 3.3V低下 | 警告のみ（**状態不変**）→ 通常着陸 | 6 | IDLE_GROUND |
| eskf_diverged | t=8s ESKF発散 | reset要求のみ（**状態不変**）→ 通常着陸 | 6 | IDLE_GROUND |

#### 図1: 正常飛行の詳細（高度・姿勢・状態遷移）

![Nominal flight detail](assets/fig1_nominal_detail.png)

真値高度と ESKF 推定高度が良好に追従（離陸 → 0.5m ホバー → 着陸）。姿勢が平坦なのは外乱なしのトリム飛行＋制御に真値姿勢を用いるため（M3 時点では ESKF 姿勢はオープンループ推定。閉ループ化は M4）。下段の状態遷移が高度プロファイルと正しく対応している。

#### 図2: 全シナリオの状態遷移タイムライン

![State timelines](assets/fig2_state_timelines.png)

点線がアラート発生時刻。**異常系（comm_lost / impact）はアラートで即座に状態遷移**し、**警告系（low_battery / eskf_diverged）は状態を変えず飛行を継続**することが一目で対比できる。`handleAlert` の設計（IMPACT/COMM_LOST は遷移、LOW_BATTERY/ESKF_DIVERGED は通知のみ）が正しく機能している証拠である。

#### 図3: 全シナリオの高度プロファイル

![Altitude overlay](assets/fig3_altitude_overlay.png)

impact（赤）が t=8s で急降下（緊急着陸）、comm_lost（橙）が緩やかに自動着陸降下する一方、警告系3シナリオ（nominal / low_battery / eskf_diverged）は飛行軌道が完全に一致して重なる。「警告は飛行に影響しない（状態不変）」ことを軌道レベルで裏付けている。

## 5. M4〜M6 — 閉ループ化と制御設計検証

詳細な制御設計検証の知見は `control/validation/sil_control_validation.md`（設計根拠の受け皿）に格納している。本節はその要点を抜粋する。

### M4 — 閉ループ化

制御に真値姿勢でなく **ESKF 推定姿勢をフィードバックする本来の閉ループ**を実現した。`integrated_sil.cpp` に `--feedback {truth|eskf}` を追加。`truth` は M3 と数値一致（回帰の基準点）、`eskf` が L4 閉ループ。突風外乱（gust）注入機構も移植した。

- **合格基準達成:** `--feedback truth` が M3 とビット一致、`--feedback eskf` の閉ループホバーが安定（発散しない）。

### M5 — 診断の梯子（要因分離）

`--feedback eskf` でノイズ層を段階的に ON/OFF し、姿勢誤差の要因を切り分けた。

#### 図4: 診断の梯子 — ノイズ段別の姿勢誤差

![Diagnostic ladder](assets/fig4_diagnostic_ladder.png)

| ノイズ段 | roll RMS | 結論 |
|---------|:---:|------|
| N0（ノイズなし） | 0.000° | ESKF閉ループは構造的に健全（線形化バイアスは無外乱で出ない） |
| N1（白色＋バイアス） | 0.013° | 白色ノイズはほぼ無影響 |
| N2（フル振動） | 3.321° | **スロットル結合振動が姿勢誤差の主因** |

層を ON/OFF するだけで「ESKF構造の問題か、ノイズの問題か」を定量的に切り分けられた。

### M6 — 制御設計検証（A/Bスイープ）

主因と判明した振動に対し、ESKF系LPF α と加速度観測ノイズ R を決定論的に A/B 検証した（`--feedback eskf --noise-stage N2`）。

#### 図5: 制御パラメータの A/B スイープ

![A/B sweep](assets/fig5_ab_sweep.png)

| パラメータ | 範囲 | roll RMS | 効果 |
|-----------|------|:---:|------|
| LPF α | 0.32 → 0.05 | 3.32 → 2.64° | 強フィルタで ~20% 改善 |
| ESKF R | 0.3 → 10 | 3.32 → 2.93° | ~12% 改善、R≥3で頭打ち |

gust 外乱下でも α=0.05 で roll RMS 3.43→2.74°、roll max 4.43→4.00°（この外乱強度では位相遅れの副作用は顕在化せず）。

**知見と留保:** 振動起因の姿勢誤差は LPF 強化や R 増加で**限定的（~20%）にしか改善せず根治しない**。これは旧機の実機知見（「元ゲインが最良・ノッチNG」）とも整合する。**SILで得た改善が実機に転移するかは Model Fidelity（M8）で照合してから判断する**（SIL設計検証 → Model Fidelity → 実機適用 の順を厳守）。

## 6. M11 — 回帰チャレンジ（北極星）

SIL の真価は「**本物の推定器バグを捕まえられるか**」で決まる。M11 は旧 `firmware/vehicle/` の ESKF（active_mask型）を SIL に載せ、既知の旧バグを再現（陽性較正）し、新 ESKF で直ることを同一入力で定量証明する。

### M11-1 — 旧ESKF の参照コンパイル基盤

旧 ESKF の ESP-IDF 依存は `esp_err.h` のみ（FreeRTOS/NVS非依存）。`compat/esp_err.h` を追加して**無改変で参照コンパイル**に成功（`active_mask=0x06c0`, `pos_var=0.03` で初期化、"M11 link OK"）。`git diff firmware/` は空。

### M11-2 — 陽性較正（旧バグ A/C の再現）と陰性較正（D/E）

`old_eskf_regression.cpp` で、旧 ESKF を quad 物理 + 真値PIDホバリングで**観測専用（オープンループ推定）**に並走駆動した。軌道は真値で決定論的なので、旧/新の比較は厳密な A/B になる。

**陽性較正（再現すべき推定器系バグを SIL で再現）:**

| バグ | モード | 現象 | 結論 |
|------|--------|------|------|
| A / 速度 | `--mode flow` | ホバリング+忠実flowで P(VEL)崩壊 → 正常な flow 更新が χ² 誤棄却 → **ESKF速度が 119.7 m/s へ発散**（真値≈0） | 運用上破滅的な帰結を可視化 |
| A / 高度 | `--mode pcollapse` | 静止ToF長時間観測で P(POS_Z) が 1.0 → 2.4e-5（R_tof=1e-4 以下）へ崩壊 | 共分散レベルで再現 |
| C | `--mode ba --free-ba` | BA再凍結を省くと加速度バイアスが較正値から 0.65 m/s² 漂流（既定の凍結なら 0） | ファームが BA を凍結する理由を実証 |

**判別対照（`--no-flow-gate`）:** χ²ゲートを OFF にすると旧の速度発散が 0.54 m/s に有界化する。これは**発散の原因が χ²ゲート破綻（P-collapseの帰結）であり、合成された人工産物ではない**ことを証明する。

**陰性較正（`--mode negcal`）:** 並行性系バグ D（着陸検出タイミング）・E（arbiter仲裁競合）は、3根拠 —（1）リンク境界（`eskf.cpp` のみリンク、`landing_handler`/`control_arbiter` 非リンク）（2）依存閉包（ESKFは FreeRTOS 非依存、D/E は mutex/状態文脈に依存）（3）決定論性（単一スレッドでレースは定義上発生しない）— により **SIL 射程外**と陳述。**非再現は期待される結果**であり、差分診断の対偶（再現→ソフト要因／非再現→HW・並行性要因）を成す。

### M11-3 — redesign A/B（新ESKFで A/C が直ることを定量証明）

同ハーネスに新 ESKF（`sf::EskfCore`）を旧と**同一センサ列で並走駆動**（両者とも観測専用で軌道不変）。新側はファーム設定（ToF + 適応R + innovクランプ + ToF速度観測）を用いる。

#### 図6: 回帰チャレンジ — 旧 vs 新 ESKF（同一入力）

![Regression challenge](assets/fig6_regression_challenge.png)

| パネル | 旧（赤） | 新（緑） | 判定 |
|--------|---------|---------|------|
| バグA / 速度（対数軸） | 119.7 m/s 発散 | 0.31 m/s 有界（`flow_innov_clamp` + `updateToFVelocity`） | **redesign 修正 ✅** |
| バグC（加速度バイアス漂流） | 0.65 m/s² | 0.0009 m/s² | **redesign 修正 ✅** |
| バグA / 高度（対数軸、P(POS_Z)） | 2.4e-5 へ崩壊 | 1.5e-5 へ崩壊 | **旧新とも崩壊（共通現象）** |

#### 重要な訂正 — P-collapse の正しい理解

M11-2 の理解を M11-3 で精緻化した。**P-collapse「現象」自体はバグではなく正常な Kalman 挙動**（一貫した観測下で共分散は縮む）であり、旧・新の両フィルタで起きる（図6 第3パネル）。ToF/高度チャネルは新旧とも**絶対値**イノベーションゲートを使うので、崩壊しても無害である。

バグ化するのは「**崩壊した P の上に χ²（マハラノビス）ゲートが乗る**」とき＝旧の flow/速度チャネルである。`--no-flow-gate` 対照で旧の発散が消えることが、発散の原因が χ²ゲート破綻であると確定させる。これは旧コード `eskf.hpp:135` のコメント「P-collapseでχ²ゲートが位置センサで機能しない」の運用上の実害そのものである。

→ **`pcollapse` モード＝根本原因の確認（旧新共通）、`flow` モード＝運用バグと修正の差別化点**、と位置づけが明確になった。

### SILループの一周

M11 により「**SILでバグ発見 → 修正 → SILで直ったと確認**」のループが旧→新の実例で一周した。これが診断計器としての SIL の北極星の達成である。

## 7. SIL の限界（射程外の明示）

診断計器の信頼の条件は、**捕まえられないバグのクラスを明示できること**である。本SILは以下を射程外とする。

- **並行性（race condition）** — 単一スレッド決定論実行のため定義上発生しない
- **割り込みジッタ／スケジューリング** — `compat` シムは実時間挙動を再現しない
- **通信物理層／キュー溢れ** — ESP-NOW/WiFi の物理層はモデル化しない

差分診断（M7）は実機↔SIL の「再現/非再現」二値判定を出し、**非再現を「HW・並行性・通信要因」へ積極的に切り分ける**ことを価値とする（M11-2 の D/E 陰性較正がその原型）。

## 8. 残作業（M7〜M10）

| フェーズ | 内容 | 前提 |
|---------|------|------|
| **M7 差分診断** | `--inject-input <csv>` で実機操縦コマンドを SIL に注入、実機↔SIL の軸別RMSE + PSD残差を定量化、再現/非再現の二値判定 | 実機フライトログの入手・形式確認 |
| **M8 Model Fidelity** | 層別 Model Fidelity スコア、`development_roadmap.md` Phase 3.2 の許容差（ACROホバー gyro RMS ±50%、ステップ立上り時定数 ±20%）に対する定量判定 | 実機ログ、M7 |
| **M9 ファーム検討** | 起動シーケンス・failsafe連鎖・状態遷移カバレッジ測定（`scenario.hpp` enum拡張、47件テストとの突合） | M4 |
| **M10 sf CLI統合** | `sf sim` に host-SIL バックエンド、`sf log→注入→差分→tune` のパイプライン化 | M5/M7/M8 |

M7/M8 は実機フライトログを前提とするため、次の着手時はまずログの入手・形式確認から始める。

## 9. 再現手順

```bash
cd simulator/sil

# M1: PC化基盤のリンク確認
make check                                  # → "M1 OK"

# M2: 状態遷移ユニットテスト（別ディレクトリ）
cd ../../firmware/vehicle_new/test && make test   # → 47/47 passed
cd ../../../simulator/sil

# M3: 統合SIL（決定論）
make integrated_sil
./integrated_sil                            # nominal、CSV を stdout に
./integrated_sil --scenario impact 2>&1 >/dev/null | grep Transition

# M5: 診断の梯子
for s in N0 N1 N2; do
  ./integrated_sil --feedback eskf --noise-stage $s 2>&1 >/dev/null | grep "roll :"
done

# M6: A/B スイープ
./integrated_sil --feedback eskf --noise-stage N2 --lpf-alpha 0.05 2>&1 >/dev/null | grep "roll :"
./integrated_sil --feedback eskf --noise-stage N2 --accel-noise 3.0 2>&1 >/dev/null | grep "roll :"

# M11: 回帰チャレンジ（旧/新 ESKF A/B）
make old_eskf_regression
./old_eskf_regression --mode flow                 # バグA/速度: 旧発散 vs 新有界
./old_eskf_regression --mode ba --free-ba         # バグC: 旧漂流 vs 新有界
./old_eskf_regression --mode pcollapse            # バグA/高度: 旧新とも崩壊
./old_eskf_regression --mode negcal               # D/E 陰性較正

# 全グラフ（fig1〜fig6）を再生成
python3 plot_sil_results.py                 # docs/assets/*.png を出力
```

---

<a id="english"></a>

## 1. Overview

### About This Document

This report consolidates the results and verification of the roadmap (M1–M11) that elevates the StampFly SIL **from a mere sanity-check rig into a "diagnostic instrument."** It supersedes the earlier report (`M1-M3_validation_report.md`), which covered only M1–M3.

The legacy `firmware/vehicle/` suffered unidentified bugs rooted in state estimation (ESKF) and state-transition management — the primary motivation for the firmware rewrite (vehicle_new). This effort verifies the control law and state machine deterministically on a PC while **reference-compiling the firmware C++ core unmodified (Code Identity)**, and then calibrates the SIL's own detection power by **reproducing the legacy firmware's known bugs (the regression challenge).**

### Achievement Summary

| Milestone | Content | Status |
|-----------|---------|:---:|
| **M1** | Host build base (ESP-IDF compat shim + topic instances) | ✅ |
| **M2** | State-transition unit tests (47) | ✅ |
| **M3** | StateManager-driven integrated SIL (5 scenarios) | ✅ |
| **M4** | Closed loop (`--feedback truth\|eskf`) + gust injection | ✅ |
| **M5** | Diagnostic ladder (N0–N2, cause isolation) | ✅ |
| **M6** | Control-design validation (metrics, A/B sweeps, `control/`) | ✅ |
| **M11-1** | Legacy ESKF (active_mask type) reference-compiled on host | ✅ |
| **M11-2** | Regression challenge: reproduce legacy bugs A/C; D/E negative-calibrate | ✅ |
| **M11-3** | Redesign A/B: prove A/C fixed in new ESKF on identical inputs | ✅ |
| M7 | Diff diagnosis (real-flight injection, real↔SIL residual) | ⬜ TODO |
| M8 | Model Fidelity calibration (real-flight matching) | ⬜ TODO |
| M9 | Firmware behavior coverage (boot, failsafe chains, transitions) | ⬜ TODO |
| M10 | sf CLI integration (host-SIL backend) | ⬜ TODO |

### SIL Design Philosophy — Three Pillars

1. **Regression challenge (north star):** can the SIL reproduce/detect the legacy firmware's known bugs? — the ultimate acceptance criterion (§6).
2. **Diff diagnosis:** thanks to Code/Parameter Identity, a hardware bug is classified as "reproduced → software cause" vs "not reproduced → HW/concurrency/comm cause" (§6.2 negative calibration, §7).
3. **Diagnostic ladder:** toggle layers (L1–L4 / noise N0–N4 / HW & concurrency boundaries) to localize the cause by which layer reproduces it (§5).

## 2. Design Principle — Code Identity by Reference

The core idea is that **the SIL runs the exact same code as the firmware despite being separated**.

- The SIL harness lives independently in `simulator/sil/` but compiles the firmware core (`eskf_core.cpp` / `state_manager.cpp` / `failsafe.cpp`) **by relative-path reference, not by copy**.
- ESP-IDF dependencies (FreeRTOS / esp_timer / esp_log, and `esp_err.h` for the legacy ESKF) are absorbed by `compat/` shims; the **firmware sources are never edited**.
- An empty `git diff firmware/` is the acceptance criterion for Code Identity throughout — applied to **both** the new firmware (vehicle_new) and the legacy firmware (vehicle) referenced by the regression challenge.

## 3. Roadmap and Progress

The two end goals — (1) firmware behavior review, (2) control-design validation — drive a dependency-faithful order. The north-star M11 was executed early, closing the loop **"find a bug in SIL → fix → confirm fixed in SIL"** on a legacy→new example. M7–M10 (phases needing real-flight logs / tooling) remain (§8).

## 4. M1–M3 — Base and State Transitions

- **M1:** `compat/` provides minimal ESP-IDF headers (FreeRTOS mutex → `std::mutex`, queue → `std::deque`, `esp_timer_get_time` → `std::chrono`, `ESP_LOGx` → stderr, plus `esp_err.h` for M11). `make check` drives the unmodified StateManager `INIT → IDLE_GROUND → ARMED_GROUND`; `git diff firmware/` stays empty.
- **M2:** 47 deterministic unit tests in `test_state_manager.cpp`, **all passing** (11 normal transitions, 12 guard rejections, 3 mode changes, 10 alert branches, 5 callback, 5 failsafe thresholds, 1 end-to-end). An unrelated pre-existing `pid_integral` failure is out of scope.
- **M3:** the legacy `bool armed` + hardcoded `T_ARM` fake state is replaced by the **real, unmodified StateManager / Failsafe driven through the loop**. Five scenarios verified deterministically. See **Figure 1** (nominal altitude/attitude/state), **Figure 2** (per-scenario state timelines — faults transition immediately, warnings keep flying), **Figure 3** (altitude overlay — warning scenarios overlap exactly).

| Scenario | Injection | Expected | Transitions | Final |
|----------|-----------|----------|:---:|-------|
| nominal | none | normal flight | 6 | IDLE_GROUND |
| comm_lost | COMM_LOST @8s | FLYING → LANDING | 6 | IDLE_GROUND |
| impact | 8G @8s | **emergency IDLE** | 5 | IDLE_GROUND |
| low_battery | 3.3V sag | warning only (**no state change**) | 6 | IDLE_GROUND |
| eskf_diverged | divergence @8s | reset only (**no state change**) | 6 | IDLE_GROUND |

## 5. M4–M6 — Closed Loop and Control-Design Validation

Detailed control-design findings live in `control/validation/sil_control_validation.md` (the home for design rationale). Summary:

- **M4 (closed loop):** feed back **ESKF-estimated attitude** instead of truth. `--feedback {truth|eskf}` added; `truth` matches M3 bit-for-bit (regression baseline), `eskf` is the L4 closed loop. Gust injection ported. Closed-loop hover is stable.
- **M5 (diagnostic ladder):** toggling the noise stage isolates the cause (Figure 4): N0 (none) → roll RMS 0.000°, N1 (white+bias) → 0.013°, N2 (full vibration) → **3.321°**. Throttle-coupled vibration is the dominant cause.
- **M6 (A/B sweeps):** (Figure 5) LPF α 0.32→0.05 gives roll RMS 3.32→2.64° (~20%); ESKF R 0.3→10 gives 3.32→2.93° (~12%, saturates at R≥3). Vibration-induced error improves only modestly and is not eliminated — consistent with the legacy finding "stock gains best / notch NG." **Whether this transfers to hardware is judged only after Model Fidelity (M8).**

## 6. M11 — Regression Challenge (North Star)

The SIL's worth is whether it **catches real estimator bugs**. M11 loads the legacy `firmware/vehicle/` ESKF (active_mask type) into the SIL, reproduces known bugs (positive calibration), and proves the new ESKF fixes them on identical inputs.

### 6.1 Positive Calibration (M11-1, M11-2)

- **M11-1:** the legacy ESKF's only ESP-IDF dependency is `esp_err.h`. Adding `compat/esp_err.h` reference-compiles it **unmodified** (`active_mask=0x06c0`); `git diff firmware/` empty.
- **M11-2:** `old_eskf_regression.cpp` drives the legacy ESKF observe-only (open-loop) under quad physics + truth-PID hover, so the legacy/new comparison is a clean A/B.

| Bug | Mode | Phenomenon |
|-----|------|------------|
| A / velocity | `--mode flow` | P(VEL) collapse → valid flow update χ²-rejected → **ESKF velocity diverges to 119.7 m/s** (truth ≈ 0) |
| A / altitude | `--mode pcollapse` | static ToF → P(POS_Z) collapses 1.0 → 2.4e-5 (below R_tof=1e-4) |
| C | `--mode ba --free-ba` | unfrozen accel bias drifts 0.65 m/s² from calibration (0 when frozen) |

The `--no-flow-gate` control bounds the legacy divergence to 0.54 m/s, proving the divergence comes from **χ²-gate breakdown (a consequence of P-collapse), not a synthetic artifact**.

### 6.2 Negative Calibration (D/E)

Concurrency bugs D (landing-detection timing) and E (arbiter contention) are **out of SIL scope** on three grounds: link boundary (only `eskf.cpp` linked), dependency closure (ESKF is FreeRTOS-free; D/E need mutex/state context), and determinism (single-thread → no races by definition). **Non-reproduction is the expected result** and forms the contrapositive of diff diagnosis.

### 6.3 Redesign A/B (M11-3)

The new ESKF (`sf::EskfCore`) is driven alongside the legacy one on the **same sensor stream** (both observe-only, trajectory invariant). See **Figure 6**:

| Panel | Legacy (red) | Redesign (green) | Verdict |
|-------|--------------|------------------|---------|
| Bug A / velocity (log) | 119.7 m/s diverges | 0.31 m/s bounded (`flow_innov_clamp` + `updateToFVelocity`) | **fixed ✅** |
| Bug C (accel-bias drift) | 0.65 m/s² | 0.0009 m/s² | **fixed ✅** |
| Bug A / altitude (log, P(POS_Z)) | collapses to 2.4e-5 | collapses to 1.5e-5 | **both collapse (shared)** |

**Key correction on P-collapse:** the collapse *phenomenon* itself is not a bug but normal Kalman behavior (covariance shrinks under consistent observation) and occurs in both filters. ToF/altitude channels use **absolute-value** innovation gates in both, so collapse is benign. The bug appears only when a **χ² (Mahalanobis) gate sits on top of a collapsed P** — the legacy flow/velocity channel. This is exactly the operational harm of the legacy comment at `eskf.hpp:135`. Thus `pcollapse` confirms the shared root cause; `flow` is the differentiator between the operational bug and its fix.

M11 closes the loop **"find a bug in SIL → fix → confirm fixed in SIL"** on a real legacy→new example.

## 7. SIL Boundaries (Explicit Out-of-Scope)

A diagnostic instrument earns trust by **naming the bug classes it cannot catch**: concurrency (single-thread deterministic → none by definition), interrupt jitter/scheduling (shims do not model real-time), and comm physical layer / queue overflow (ESP-NOW/WiFi PHY not modeled). Diff diagnosis (M7) outputs a real↔SIL reproduce/not-reproduce verdict and **actively classifies non-reproduction as "HW/concurrency/comm cause"** — the D/E negative calibration in M11-2 is its prototype.

## 8. Remaining Work (M7–M10)

| Phase | Content | Prerequisite |
|-------|---------|--------------|
| **M7 Diff diagnosis** | `--inject-input <csv>`; per-axis RMSE + PSD residual; reproduce/not verdict | real-flight logs |
| **M8 Model Fidelity** | layered fidelity scores vs `development_roadmap.md` Phase 3.2 tolerances (ACRO hover gyro RMS ±50%, step time-constant ±20%) | real logs, M7 |
| **M9 Firmware review** | boot sequence, failsafe chains, transition coverage (vs the 47 tests) | M4 |
| **M10 sf CLI** | host-SIL backend for `sf sim`; `sf log→inject→diff→tune` pipeline | M5/M7/M8 |

M7/M8 require real-flight logs, so the next step starts by obtaining and confirming the log format.

## 9. Reproduction

See the command block in the Japanese section (§9).
</content>
</invoke>
