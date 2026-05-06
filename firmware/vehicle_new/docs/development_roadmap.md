# vehicle_new Development Roadmap and SIL→Real Workflow
# vehicle_new 開発ロードマップ・SIL→実機ワークフロー

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 本文書の位置づけ

### このドキュメントについて

vehicle_new の **開発の進め方**（=どの順番で何を作り、何を持って各段階の合格とするか）と、**SIL シミュレーションと実機の関係性** を明文化する。

- 設計文書（requirements/architecture/detailed_design）は「**何を**作るか」を定義する
- コーディング教育文書は「**どう書くか**」を定義する
- 本文書は「**どう開発・検証して完成に至るか**」を定義する

### 対象読者

- vehicle_new の実装に関わる開発者（人間 + AI）
- 既存実装をベースに研究・教育を行う学生・研究者

### 用語の使い分け（重要）

本書および vehicle_new プロジェクト全体で、複数の番号付き概念が併存する。混同しないこと。

| 用語 | 意味 | 出典 |
|------|------|------|
| **Phase 0〜6** | 開発工程の段階。本書 §4 の計画 | 本書 §4 |
| **Layer 1〜4** | 段階的プラント同定の層（ACRO → STAB → ALT → POS） | 本書 §3 |
| **SIL Control Level L1〜L4** | SIL 制御テストの複雑度レベル（true rate / true att / noisy / ESKF）| `sim/flight_scenario_test.cpp` の `enum Level`、`sim/control_test.cpp` の `enum TestLevel` |
| **Noise Model Stage N0〜N4** | センサノイズモデルの複雑度段階（教材） | `noise_and_vibration_model.md` §4 |

`L1〜L4` は **SIL Control Level の意味で使う**。ノイズモデルの段階は `N0〜N4` を使い、両者を区別する。

---

## 2. 開発方針の3原則

vehicle_new の SIL → 実機ワークフローは次の3原則に基づく。

### 原則1: Code Identity（コード一致）

**SIL も実機も同じ ESKF / PID コードを実行する。**

| 要素 | 共有方法 |
|------|---------|
| ESKF 数値ロジック | `components/sf_estimator_eskf/eskf_core.cpp` を SIL 側 Makefile が直接コンパイル |
| PID 数値ロジック | `components/sf_controller_pid/include/pid.hpp` を SIL 側が include |
| ベクトル/行列演算 | `components/sf_math/` を共有 |

これにより、SIL でデバッグした制御ロジックの挙動は実機でも同じになる。

### 原則2: Parameter Identity（パラメータ一致）

**SIL も実機も `params.def` を Single Source of Truth として読む。**

- 実機: `params.def` → コード生成 → NVS 永続化 → ランタイム読み取り
- SIL: `params.def` → コード生成 → デフォルト値 / `--params <file>` でオーバライド

実機側は WiFi/CLI でチューニングした値を NVS に保存し、必要に応じてファイルにエクスポート。SIL で詰めたパラメータと実機で詰めたパラメータが、**同じスキーマで相互流通する** こと。

### 原則3: Model Fidelity（モデル忠実度の閉ループ改善）

**実機の飛行ログを使って SIL の物理・センサ・外乱モデルを継続的に校正する。**

実機 ≠ SIL のずれを観測し、ずれの大きい要素から SIL モデルを改善する。SIL モデルが信頼できる範囲が広がるほど「SIL で詰めた → 実機で飛ぶ」確実性が上がる。

---

## 3. プラント同定の戦略 — ACROレート制御を起点とする

### 中核思想

**人間が操縦する ACRO（角速度制御）モードが、実機プラント同定に最も有効である。**

### 理由

| 観点 | ACRO の優位性 |
|------|-------------|
| **制御構造の単純性** | レート PID 内ループのみ。姿勢推定器・外ループ・カスケード結合なし |
| **計測の直接性** | ジャイロ生値 = 制御対象の状態量（バイアス補正のみで使える） |
| **励振の任意性** | パイロットが任意波形で軸を励振できる。Step / Sine / Doublet を手動で打てる |
| **誤差の即時可観測性** | 機体の挙動 = スティック指令 か否かが、観測者にも操縦者にも瞬時に分かる |
| **被疑成分の少なさ** | 不一致が出たら原因は (a) プラント (Ixx/モータ) か (b) レート PID のどちらか。切り分けが容易 |

### 段階的同定（Layer-by-Layer Identification）

ACRO で土台を確定してから、上位層を1段ずつ積み上げる。各層の完成は次層の前提となる。

```
Layer 1: ACRO（レート制御）           ← プラント + レートPID + ジャイロ を確定
            ↓
Layer 2: STABILIZE（姿勢制御）         ← + ESKF姿勢 + 加速度計 + 姿勢PID
            ↓
Layer 3: ALTITUDE_HOLD                 ← + ToF/Baro + 高度PID + ホバースラスト
            ↓
Layer 4: POSITION_HOLD                 ← + Flow + 位置PID
```

### SIL の L1〜L4 との対応

既存の SIL テスト構造（`sim/flight_scenario_test.cpp` 等の Level 概念）と完全に対応する:

| 実機モード | SIL Level | 共通要素 |
|----------|-----------|---------|
| ACRO（生ジャイロ） | L1（true rate） | レート PID 単独動作 |
| ACRO（ノイズあり） | L3（noisy gyro） | + センサノイズ |
| STABILIZE / ALT / POS | L4（full ESKF） | + ESKF 全機能 |

**SIL L1 と実機 ACRO は構造的に等価**。SIL L1 で通ったレート PID は実機 ACRO でも通るはずであり、通らなければ「プラントモデルかセンサノイズモデルが実機と乖離している」と即座に判定できる。

---

## 4. フェーズ計画

### Phase 0: 現状（達成済みの確認）

- 設計4文書完成
- スケルトン + 全14タスク + 全14コンポーネントスタブ + ESKF/PID 新規実装
- SIL 物理モデル（quad_physics）+ 外乱モデル + per-axis 振動ノイズ
- SIL L1〜L4 段階検証で姿勢2.27° / 高度44mm 達成
- ESKF 線形化バイアス問題の定量化（200s ステップ保持で −2.3°）

**合格基準:** 既に達成済み。

---

### Phase 1: パラメータ一元化（最優先）

**目的:** SIL でチューニングした値が手動コピーなしで実機に反映される構造の確立。

| ID | 作業 | 成果物 |
|----|------|--------|
| 1.1 | `params.def` を SIL からも include して `Params` 構造体を生成（X-macro 展開） | SIL 側 `params_loader.hpp` |
| 1.2 | SIL のハードコード値を `params.def` の defaults に統一 | `flight_scenario_test.cpp` のリテラル削除 |
| 1.3 | SIL に `--params <file>` オプション追加 | `sil_main.cpp` 拡張 |
| 1.4 | 実機 NVS ↔ ファイル の双方向エクスポート/インポート | CLI コマンド `params save/load` |

**合格基準:** SIL で `R=2.0, accel_att_noise=0.06, alt_kp=0.6` 等を変えてフルシナリオ実行 → 同じ値が実機 NVS に焼け、実機ファームが起動時に同じ値を読み込んでいることをログで確認。

---

### Phase 2: HAL 接続（実機が動く）

**目的:** 全タスクファイルの TODO 化を解消し、実機センサ値が ESKF へ、制御出力がモータへ到達する経路を作る。

| ID | 作業 | 依存 |
|----|------|------|
| 2.1 | `imu_task` → BMI270（400Hz、SIL のセンサ値生成インターフェースと等価な signature） | sf_hal_bmi270 |
| 2.2 | `tof_task` → VL53L3CX（30Hz） | sf_hal_vl53l3cx |
| 2.3 | `baro_task` → BMP280（50Hz） | sf_hal_bmp280 |
| 2.4 | `flow_task` → PMW3901（100Hz） | sf_hal_pmw3901 |
| 2.5 | `mag_task` → BMM150（オプション、初期は無効） | sf_hal_bmm150 |
| 2.6 | `sf_actuator` → motor HAL（PWM 出力、4ch） | sf_hal_motor |
| 2.7 | `sf_calibration` → NVS 永続化 | nvs_flash |
| 2.8 | `sf_comm` → ESP-NOW 受信 → `command_setpoint` トピック | esp_now |
| 2.9 | `sf_telemetry` → UDP 送信（既存 vehicle/ の統合パケット形式準拠） | lwIP |
| 2.10 | `sf_logger` → SPIFFS Blackbox | spiffs |

**合格基準:** 機体を手で持って ARM → スロットル中立で全モータが等速回転、スティック動作で1軸ずつモータ duty が予期通り変化、テレメトリで全センサ値が表示される。**まだ飛ばさない。**

---

### Phase 3: 実機初飛行 — ACRO で同定（最重要マイルストーン）

**目的:** SIL L1/L3 で確定したレート PID とプラントモデルが実機で成立することを確認する。

#### 3.0 — 地上テスト（飛ばす前）

| ID | 作業 |
|----|------|
| 3.0.1 | キャリブレーション（ジャイロ・加速度バイアス、レベル基準） |
| 3.0.2 | テスト台に拘束した状態でモータ duty スイープ → 推力測定 → SIL の thrust curve 校正 |
| 3.0.3 | 質量/CG 実測 → SIL `QuadParams` の更新 |

#### 3.1 — ACRO 手動飛行（プラント + レート PID 同定）

| ID | 作業 | 取得データ |
|----|------|----------|
| 3.1.1 | ACRO ホバー（数秒〜10秒） | gyro / motor_duty / cmd_rate |
| 3.1.2 | 各軸ステップ入力（roll/pitch/yaw を1軸ずつ） | step response |
| 3.1.3 | 各軸ダブレット（連続ステップ） | broadband 励振 |
| 3.1.4 | パイロット随意操縦 | 実運用領域カバー |

**全飛行でテレメトリ記録必須**: 生 IMU、モータ duty、操縦コマンド、推定値、PID内部状態。

#### 3.2 — SIL diff 解析

実機ログを SIL に注入してオフライン再現:

1. 実機の操縦コマンド時系列を SIL に注入
2. SIL の出力（gyro 応答、motor duty）と実機ログを比較
3. 残差スペクトル解析:
   - 低周波残差 → プラントモデル誤差（Ixx, motor τ, mixer 係数）
   - 高周波残差 → センサノイズモデル誤差
   - DC オフセット → バイアスキャリブレーション誤差

**合格基準:** ACRO ホバーでの gyro RMS が SIL 予測 ±50% 以内、ステップ応答の立ち上がり時定数が ±20% 以内。

---

### Phase 4: 上位層の段階追加

Phase 3 で土台が確定したら、Layer 2→3→4 の順に同じ手順で実機検証:

| Phase | モード | 追加要素 | SIL Level |
|-------|--------|---------|-----------|
| 4.1 | STABILIZE | ESKF 姿勢 + 加速度計 + 姿勢 PID | L4（姿勢のみ） |
| 4.2 | ALTITUDE_HOLD | ToF/Baro + 高度カスケード PID + ホバースラスト | L4（高度追加） |
| 4.3 | POSITION_HOLD | Flow + 位置 PID | L4（位置追加） |

各段階の合格基準は Phase 3 と同様（実機 vs SIL の許容差規定）。

---

### Phase 5: モデル校正の閉ループ運用

実機ログを使って SIL モデルを継続的に改善する **定常運用フェーズ**。

| ID | 作業 |
|----|------|
| 5.1 | 実モータ duty をテレメトリに追加（throttle 指令だけでなく実 duty） |
| 5.2 | hover01/12/13 を含む複数ログで per-axis 振動 σ∝duty² モデルを再検証 |
| 5.3 | Step 入力ログから慣性テンソル (Ixx/Iyy/Izz) の system identification |
| 5.4 | 地面効果モデル追加（高度依存推力増） |
| 5.5 | ESKF 線形化バイアス対策の SIL 検証 → 実機適用<br>`a_gravity = a_meas + [0,0,T_est/m]` で thrust 寄与を補償 |
| 5.6 | colored noise（モータ高調波）モデルの導入検討 |

**合格基準（継続的）:** Phase 4.1〜4.3 の許容差規定が複数機体・複数ログにわたって維持される。

---

### Phase 6: 教材化（vehicle_new の本来目的）

| ID | 作業 |
|----|------|
| 6.1 | Examples Level 2（09-13）— 推定/制御の基礎 |
| 6.2 | Examples Level 3（14-20）— カスケード/PID 教育 |
| 6.3 | Examples Level 4（21-25）— フルフライト |
| 6.4 | チュートリアル10章。**特に「SIL→実機で同じ結果が出る理由」を1章として書く** |
| 6.5 | ワークショップ用スクリプト：「SIL で PID チューニング → params 出力 → 実機書き込み → ACRO飛行 → STABILIZE飛行」を1セッションで通せる |
| 6.6 | `@design` タグ全 `[OK]` 化（リリース基準） |

---

## 5. ガバナンス

### 各 Phase の進行ルール

- 1つの Phase の **合格基準を満たすまで次の Phase に進まない**
- 合格基準を満たした時点で `implementation_log.md` に記録
- 実機 vs SIL の許容差を超えた場合、**先に SIL モデル校正（Phase 5）に戻る**
- 設計文書との矛盾を発見したら実装を止めて報告（coding_and_education.md §1 のルール）

### パラメータ管理

- SIL でのチューニング結果は params ファイルとして保存し、コミットする
- 実機での再チューニング結果も params ファイルとしてエクスポートし、コミットする
- 機体個体差で値が異なる場合、`params/<machine_id>.yaml` のような形で分離管理

### 実機飛行ログ管理

- 飛行ごとに `logs/<date>_<mode>_<seq>.jsonl` 形式で保存
- 重要な検証飛行（Phase 合格判定に使ったもの）は git にコミット
- 解析スクリプトは `scripts/` または `sf log analyze` 系コマンド経由

---

## 6. 関連文書

| 文書 | 役割 |
|------|------|
| `requirements.md` | 何を作るか |
| `architecture.md` | コンポーネント構造 |
| `detailed_design.md` | インターフェース・状態遷移 |
| `coding_and_education.md` | コーディング規約・教育方針 |
| `noise_and_vibration_model.md` | センサノイズ・振動モデル |
| `implementation_log.md` | 実装の時系列記録 |
| **`development_roadmap.md`（本文書）** | 開発ワークフロー・フェーズ計画 |

---

<a id="english"></a>

## 1. About This Document

### Purpose

This document defines **how vehicle_new is developed** — the order of work, the acceptance criteria for each stage, and the relationship between SIL simulation and real flight.

- Design docs (requirements / architecture / detailed_design) define **what** to build
- The coding/education doc defines **how** to write code
- This doc defines **how to develop and validate** to reach completion

### Target Audience

- Developers working on vehicle_new (humans + AI)
- Students and researchers building research/education on top of the existing implementation

### Terminology

Multiple numbered concepts coexist in this project. Keep them distinct.

| Term | Meaning | Source |
|------|---------|--------|
| **Phase 0–6** | Development stages defined in §4 of this doc | This doc §4 |
| **Layer 1–4** | Layered plant-identification stack (ACRO → STAB → ALT → POS) | This doc §3 |
| **SIL Control Level L1–L4** | SIL control-test complexity levels (true rate / true att / noisy / ESKF) | `sim/flight_scenario_test.cpp` `enum Level`, `sim/control_test.cpp` `enum TestLevel` |
| **Noise Model Stage N0–N4** | Sensor-noise complexity stages (educational) | `noise_and_vibration_model.md` §4 |

`L1–L4` always refers to **SIL Control Levels**. Noise model stages use the `N0–N4` notation to avoid collision.

---

## 2. Three Principles of Development

The SIL → real-flight workflow rests on three principles.

### Principle 1: Code Identity

**SIL and real hardware execute the same ESKF / PID code.**

| Element | Sharing Method |
|---------|---------------|
| ESKF numerics | `components/sf_estimator_eskf/eskf_core.cpp` is compiled directly by the SIL Makefile |
| PID numerics | `components/sf_controller_pid/include/pid.hpp` is included by SIL |
| Vector/matrix math | `components/sf_math/` is shared |

Behavior debugged in SIL therefore matches real hardware.

### Principle 2: Parameter Identity

**SIL and real hardware both read `params.def` as the single source of truth.**

Tuning values flow bidirectionally between SIL files and real-hardware NVS through the same schema.

### Principle 3: Model Fidelity (Closed-Loop Improvement)

**Real flight logs continuously calibrate the SIL physics, sensor, and disturbance models.**

The wider the SIL model's trustworthy envelope, the higher the certainty of "tuned in SIL → flies on hardware."

---

## 3. Plant Identification Strategy — ACRO Rate Control as Foundation

### Core Idea

**Human-piloted ACRO (rate) mode is the most effective way to identify a real-aircraft plant.**

### Why

| Aspect | Why ACRO wins |
|--------|--------------|
| **Simplest control structure** | Inner-loop rate PID only. No estimator, outer loop, or cascade coupling |
| **Direct measurement** | Raw gyro = controlled state. Bias correction is the only post-processing |
| **Arbitrary excitation** | Pilot can hand-inject step / sine / doublet on any axis |
| **Immediate observability** | Mismatch between stick command and aircraft motion is obvious to pilot and observer |
| **Few suspects** | If something is off, it is either (a) plant (Ixx / motor) or (b) rate PID. Easy to localize |

### Layer-by-Layer Identification

ACRO confirms the foundation; higher layers stack one at a time. Each layer's pass is the next layer's prerequisite.

```
Layer 1: ACRO (rate)             ← plant + rate PID + gyro
            ↓
Layer 2: STABILIZE (attitude)    ← + ESKF attitude + accel + attitude PID
            ↓
Layer 3: ALTITUDE_HOLD           ← + ToF/Baro + altitude PID + hover thrust
            ↓
Layer 4: POSITION_HOLD           ← + Flow + position PID
```

### Mapping to SIL L1–L4

| Real Mode | SIL Level | Common Elements |
|-----------|-----------|-----------------|
| ACRO (raw gyro) | L1 (true rate) | Rate PID standalone |
| ACRO (with noise) | L3 (noisy gyro) | + sensor noise |
| STABILIZE / ALT / POS | L4 (full ESKF) | + full ESKF |

**SIL L1 ≡ real ACRO structurally.** A rate PID that passes SIL L1 should pass real ACRO; if it does not, the plant or noise model has diverged from reality.

---

## 4. Phase Plan

(See Japanese section above for the detailed phase table — same structure applies.)

- **Phase 0**: Current state (already achieved)
- **Phase 1**: Parameter unification (highest priority)
- **Phase 2**: HAL connection (hardware comes alive)
- **Phase 3**: First flight — ACRO identification (key milestone)
- **Phase 4**: Stack higher layers (STABILIZE → ALT_HOLD → POS_HOLD)
- **Phase 5**: Continuous model-fidelity calibration loop
- **Phase 6**: Educational productization

---

## 5. Governance

- Each phase must clear its acceptance criteria before the next phase begins
- Real-vs-SIL gaps exceeding tolerance trigger a return to Phase 5 model calibration
- Design-vs-implementation conflicts halt implementation pending discussion (per coding_and_education.md §1)

---

## 6. Related Documents

| Doc | Role |
|-----|------|
| `requirements.md` | What to build |
| `architecture.md` | Component structure |
| `detailed_design.md` | Interfaces and state transitions |
| `coding_and_education.md` | Coding standards and education plan |
| `noise_and_vibration_model.md` | Sensor noise and vibration models |
| `implementation_log.md` | Implementation timeline |
| **`development_roadmap.md` (this doc)** | Development workflow and phase plan |
