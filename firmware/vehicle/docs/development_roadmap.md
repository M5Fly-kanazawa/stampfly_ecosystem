# vehicle Development Roadmap and SIL→Real Workflow
# vehicle 開発ロードマップ・SIL→実機ワークフロー

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。
>
> **SIL ベンチの詳細は `simulator/sil/RESET_PLAN.md` を正とする。** 本書は vehicle 開発**全体**（設計→SIL→実機→教材化）の流れを定義し、SIL ベンチそのものの作り方は RESET_PLAN に委ねる。両者が食い違うときは、まず RESET_PLAN を更新してから本書を直す。

## 1. 本文書の位置づけ

### このドキュメントについて

vehicle の **開発の進め方**（=どの順番で何を作り、何を持って各段階の合格とするか）と、**SIL シミュレーションと実機の関係性** を明文化する。

- 設計文書（requirements/architecture/detailed_design/hardware_init）は「**何を**作るか」を定義する
- コーディング教育文書は「**どう書くか**」を定義する
- 本文書は「**どう開発・検証して完成に至るか**」を定義する
- SIL ベンチの作り方は `simulator/sil/RESET_PLAN.md` が定義する

### 対象読者

- vehicle の実装に関わる開発者（人間 + AI）
- 既存実装をベースに研究・教育を行う学生・研究者

### 用語の使い分け（重要）

本書および vehicle プロジェクト全体で、複数の番号付き概念が併存する。混同しないこと。

| 用語 | 意味 | 出典 |
|------|------|------|
| **Phase 0〜6** | 開発工程の段階。本書 §4 の計画 | 本書 §4 |
| **Layer 1〜4** | 段階的プラント同定の層（ACRO → STAB → ALT → POS） | 本書 §3 |
| **ゲート G1〜G4** | SIL の合格基準（起動・状態遷移／推定の追従／閉ループ安定／アクチュエータ健全） | RESET_PLAN §4 |
| **Noise Model Stage N0〜N4** | センサノイズモデルの複雑度段階（教材） | `noise_and_vibration_model.md` §4 |

> **旧「SIL Control Level L1〜L4」は廃止した。** これは旧 SIL（`sim/flight_scenario_test.cpp` 等）の制御テストレベルを指す概念だったが、旧 SIL の完全削除（RESET_PLAN §12）に伴い消滅した。新 SIL は合否を**物理真値のゲート G1〜G4**（RESET_PLAN §4）で判定する。

---

## 2. 開発方針の3原則

vehicle の SIL → 実機ワークフローは次の3原則に基づく。RESET_PLAN の「2つの基本方針」と同じ思想を、開発工程の言葉で言い直したものである。

### 原則1: Code Identity（コード一致 — ループ全体で）

**SIL は vehicle の本体ソースを書き換えず、そのままコンパイルして走らせる。**

旧 SIL の失敗は、制御ループを自前で組み直し、推定器に物理の真値姿勢を渡していたことだった。新しい SIL は、**実際の Pub-Sub ループ（`imu_task → estimate_state → control_task → actuator_motor`）を丸ごとホストで走らせる**。一致するのは ESKF の数式だけではなく、**ループ全体**である。

| 共有するもの | 方法 |
|------------|------|
| 本体ソース全体（Pub-Sub ループ＋全タスク＋推定器・制御器の実装＋数学） | SIL が ESP-IDF 互換シム（RESET_PLAN §7）の上で**書き換えずに参照コンパイル**して実行する |
| 推定器・制御器の選択 | 実機とまったく同じ `IEstimator`／`IController` を経由する（中身＝ESKF/PID であることに SIL は依存しない） |

これにより、「SIL で OK」が**実機の挙動を意味する**（ループレベルの Code Identity）。SIL は推定器・制御器の中身（アルゴリズム）に依存しないので、ESKF を相補フィルタや状態フィードバックに差し替えても、ベンチは一切変えずに同じ検証ができる（RESET_PLAN 方針2）。

### 原則2: Parameter Identity（パラメータ一致）

**SIL も実機も `params.cpp`（`param_vars` + `table[]`）を Single Source of Truth として読む。**

> 注（Phase 5b, 2026-06-07）: 当初は `params.def` の X-macro コード生成を SSOT とする設計だったが、実体は `params.cpp` の手書き `param_vars` + 明示 `table[]` に収束していた。`params.def` は非機能で値もずれていたため撤去し、`params.cpp` を正式な SSOT とした（[[reference_params_ssot]]）。Parameter Identity の本質（SIL と実機が同一スキーマの同一値を読む）は不変 — `params.cpp` は両ビルドが参照コンパイルする。

- 実機: `params.cpp` の `table[]` → 既定値 → NVS 永続化 → ランタイム読み取り
- SIL: `params.cpp` の `table[]` → 既定値（SIL NVS は空ゆえ既定が残る）

実機側は WiFi/CLI でチューニングした値を NVS に保存し、必要に応じてファイルにエクスポート。SIL で詰めたパラメータと実機で詰めたパラメータが、**同じスキーマで相互流通する** こと。

### 原則3: Model Fidelity（モデル忠実度 — 実機で飛ばした後の後追い）

**SIL の真値（正解の状態）は物理モデルから得る。実機データは不要。**

シミュレーションしている以上、真の姿勢・位置・速度は常に分かっている。だから SIL を**作る・動かす**のに実機データはいらない（RESET_PLAN 方針1）。

Model Fidelity（物理モデルが現実とどれだけ合っているか）を上げる作業は、**実機で初めて飛ばした後**に始まる、後追いの精度向上である（RESET_PLAN §3 の流れ [5]→[2]）。実機ログを使うのはこの場面**だけ**で、SIL の前提ではない。SIL モデルの信頼できる範囲が広がるほど「SIL で詰めた → 実機で飛ぶ」確実性が上がる。

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

### 各層を SIL のゲートで検証する

各層は、まず**物理真値の SIL**（RESET_PLAN）で検証し、合格基準（G1〜G4、RESET_PLAN §4）を満たしてから実機に進む。L1〜L4 のような旧 SIL の制御テストレベルの番号体系には依存しない。

| 層 | SIL で確認すること（物理真値で機械判定） | 実機での確認 |
|----|----------------------------------------|-------------|
| Layer 1 (ACRO) | レート PID 単独でホストの本物ループを回し、角速度が指令に追従し有界か（G3）、モータが飽和しないか（G4） | 実機 ACRO 手動飛行（§4 Phase 3） |
| Layer 2 (STAB) | + ESKF 姿勢が真値に追従するか（G2） | 実機 STABILIZE |
| Layer 3 (ALT) | + 高度推定・高度制御が有界か（G2/G3） | 実機 ALTITUDE_HOLD |
| Layer 4 (POS) | + 位置推定・位置制御が有界か（G2/G3） | 実機 POSITION_HOLD |

**SIL の Layer 1 と実機 ACRO は構造的に等価**。SIL で通ったレート PID は実機 ACRO でも通るはずであり、通らなければ「プラントモデルか合成センサのノイズモデルが実機と乖離している」と即座に判定できる（RESET_PLAN §3 の差分診断）。

---

## 4. フェーズ計画

> vehicle 開発全体の順序は **① SIL を作る → ② vehicle 開発を再開 → ③ SIL 上で飛ばす → ④ 実機テスト**。Phase 0 は更地化（達成済み）、Phase 1 が物理ベース SIL の再構築（最優先）、Phase 2 以降が実機ブリングアップと飛行・教材化である。

### Phase 0: クリーンスレート（達成済みの確認）

- 設計6文書完成（requirements / architecture / detailed_design / coding_and_education / hardware_init / 本書）
- vehicle スケルトン + 全14タスク + 全コンポーネントスタブ + ESKF/PID 新規実装
- **旧 SIL を完全削除（RESET_PLAN §12 / P0）。** M1〜M11 で肥大化した旧 SIL（`quad_physics`／`sil_main.cpp`／`flight_scenario_test.cpp` 等）と、それに紐づく旧実績（L1〜L4 検証、姿勢2.27°／高度44mm など）は、削除した旧 SIL のものなので**現在の実績からは外す**。経緯は git 履歴と `implementation_log.md` に保存。

**合格基準:** 達成済み（更地・workshop 無傷・sf CLI 健全・ビルド可）。

---

### Phase 1: 物理ベース SIL の再構築（最優先）

**本フェーズの作り方の詳細は `simulator/sil/RESET_PLAN.md`（P1〜P4）が正。** ここでは vehicle 開発全体の中での位置づけと合格基準だけを示す。

**目的:** まだ一度も飛んでいない vehicle を、ハードを壊さず PC 上で検証できる、**物理ベース・アルゴリズム非依存**の SIL ベンチを更地から作る。

| ID | 作業 | 対応（RESET_PLAN） |
|----|------|------------------|
| 1.1 | MuJoCo を依存として統合（FetchContent、`THIRD_PARTY_LICENSES` 同梱） | §6 |
| 1.2 | ESP-IDF 互換シムを作り直し、本体の Pub-Sub ループをホストで走らせる | §7 |
| 1.3 | 合成センサ（IMU/ToF/フロー/気圧）・モータ・風モデルを自前実装（`noise_and_vibration_model.md`） | §6 |
| 1.4 | ファーム last-mile: `applyMixer` 実装（`control_task.cpp:52-66`）、モータ出力→物理（`:124`）、推定器/制御器のファクトリ化、`@design` を `[--]`→`[OK]` に | §7 |
| 1.5 | `params.cpp`（`table[]`）を SIL からも参照（Parameter Identity の実装） | 原則2 |
| 1.6 | 基本のレビュー動画書き出し（`sf sil video` 最小版） | §9 |

**合格基準（RESET_PLAN P1〜P2 のゲート）:**
- **P1:** 現行 ESKF + PID ファームが **SIL 上でホバーする**（物理の真値で位置が有界）。その様子のレビュー動画を添える。
- **P2:** 第2の推定器（相補フィルタ、約80行）を `IEstimator` で投入し、**ベンチを一切変えずに**ホバーする＝**アルゴリズム非依存の実証（北極星）**。

---

### Phase 2: HAL 接続（実機が動く）

**目的:** 全タスクファイルの TODO 化を解消し、実機センサ値が推定器へ、制御出力がモータへ到達する経路を作る。SIL でループの健全性を確認した後に行う。

| ID | 作業 | 依存 |
|----|------|------|
| 2.1 | `imu_task` → BMI270（400Hz、SIL の合成センサと等価な signature） | sf_hal_bmi270 |
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

**目的:** SIL で確定したレート PID とプラントモデルが実機で成立することを確認する。

#### 3.0 — 地上テスト（飛ばす前）

| ID | 作業 |
|----|------|
| 3.0.1 | キャリブレーション（ジャイロ・加速度バイアス、レベル基準） |
| 3.0.2 | テスト台に拘束した状態でモータ duty スイープ → 推力測定 → SIL の thrust curve 校正 |
| 3.0.3 | 質量/CG 実測 → SIL の機体パラメータ（MuJoCo モデル＋自作モータモデル）の更新 |

#### 3.1 — ACRO 手動飛行（プラント + レート PID 同定）

| ID | 作業 | 取得データ |
|----|------|----------|
| 3.1.1 | ACRO ホバー（数秒〜10秒） | gyro / motor_duty / cmd_rate |
| 3.1.2 | 各軸ステップ入力（roll/pitch/yaw を1軸ずつ） | step response |
| 3.1.3 | 各軸ダブレット（連続ステップ） | broadband 励振 |
| 3.1.4 | パイロット随意操縦 | 実運用領域カバー |

**全飛行でテレメトリ記録必須**: 生 IMU、モータ duty、操縦コマンド、推定値、PID内部状態。

#### 3.2 — 差分診断（実機 vs SIL）

実機ログを SIL に注入してオフライン再現する（RESET_PLAN §3 の [5] 差分診断）:

1. 実機の操縦コマンド時系列を SIL に注入
2. SIL の出力（gyro 応答、motor duty）と実機ログを比較
3. 残差スペクトル解析:
   - 低周波残差 → プラントモデル誤差（Ixx, motor τ, mixer 係数）
   - 高周波残差 → センサノイズモデル誤差
   - DC オフセット → バイアスキャリブレーション誤差
4. SIL で再現する → ソフトが原因。SIL で再現しない → ハード/タイミング/通信（SIL で扱えない範囲）、または物理モデルが甘い → Phase 5（Model Fidelity 向上）へ

**合格基準:** ACRO ホバーでの gyro RMS が SIL 予測 ±50% 以内、ステップ応答の立ち上がり時定数が ±20% 以内。

---

### Phase 4: 上位層の段階追加

Phase 3 で土台が確定したら、Layer 2→3→4 の順に、各層をまず SIL のゲートで検証してから実機検証する:

| Phase | モード | 追加要素 | SIL ゲート |
|-------|--------|---------|-----------|
| 4.1 | STABILIZE | ESKF 姿勢 + 加速度計 + 姿勢 PID | G2（姿勢追従）+ G3 |
| 4.2 | ALTITUDE_HOLD | ToF/Baro + 高度カスケード PID + ホバースラスト | G2/G3（高度） |
| 4.3 | POSITION_HOLD | Flow + 位置 PID | G2/G3（位置） |

各段階の合格基準は Phase 3 と同様（実機 vs SIL の許容差規定）。

---

### Phase 5: モデル校正の閉ループ運用（Model Fidelity）

実機ログを使って SIL の物理・センサ・外乱モデルを継続的に改善する **定常運用フェーズ**。実機で飛ばした後にだけ始まる、後追いの精度向上である（原則3）。

| ID | 作業 |
|----|------|
| 5.1 | 実モータ duty をテレメトリに追加（throttle 指令だけでなく実 duty） |
| 5.2 | 複数ログで per-axis 振動 σ∝duty² モデルを再検証（`noise_and_vibration_model.md` §4） |
| 5.3 | Step 入力ログから慣性テンソル (Ixx/Iyy/Izz) の system identification |
| 5.4 | 地面効果モデル追加（高度依存推力増） |
| 5.5 | ESKF 線形化バイアス対策の SIL 検証 → 実機適用<br>`a_gravity = a_meas + [0,0,T_est/m]` で thrust 寄与を補償 |
| 5.6 | colored noise（モータ高調波）モデルの導入検討 |

**合格基準（継続的）:** Phase 4.1〜4.3 の許容差規定が複数機体・複数ログにわたって維持される。

---

### Phase 6: 教材化（vehicle の本来目的）

| ID | 作業 |
|----|------|
| 6.1 | Examples Level 2（09-13）— 推定/制御の基礎 |
| 6.2 | Examples Level 3（14-20）— カスケード/PID 教育 |
| 6.3 | Examples Level 4（21-25）— フルフライト |
| 6.4 | チュートリアル10章。**特に「SIL→実機で同じ結果が出る理由（ループレベルの Code Identity）」を1章として書く** |
| 6.5 | ワークショップ用スクリプト：「SIL で PID チューニング → params 出力 → 実機書き込み → ACRO飛行 → STABILIZE飛行」を1セッションで通せる |
| 6.6 | `@design` タグ全 `[OK]` 化（リリース基準） |

---

## 5. ガバナンス

### 各 Phase の進行ルール

- 1つの Phase の **合格基準を満たすまで次の Phase に進まない**
- 各節目（Phase の達成・SIL ゲート G1〜G4 の通過）では、**レビュー動画を必ず作る**（RESET_PLAN §9・§11 の必須ルール）。人間が一目で確認できる成果物とする
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
| `hardware_init.md` | BSP・ハードウェア初期化 |
| `coding_and_education.md` | コーディング規約・教育方針 |
| `noise_and_vibration_model.md` | センサノイズ・振動モデル（SIL の合成センサ仕様） |
| `implementation_log.md` | 実装の時系列記録 |
| `../../../simulator/sil/RESET_PLAN.md` | **SIL ベンチ再構築計画（SIL の作り方の正）** |
| **`development_roadmap.md`（本文書）** | 開発ワークフロー・フェーズ計画（全体） |

---

<a id="english"></a>

## 1. About This Document

### Purpose

This document defines **how vehicle is developed** — the order of work, the acceptance criteria for each stage, and the relationship between SIL simulation and real flight.

- Design docs (requirements / architecture / detailed_design / hardware_init) define **what** to build
- The coding/education doc defines **how** to write code
- This doc defines **how to develop and validate** to reach completion
- **How to build the SIL bench is defined by `simulator/sil/RESET_PLAN.md`**, not here

### Target Audience

- Developers working on vehicle (humans + AI)
- Students and researchers building research/education on top of the existing implementation

### Terminology

Multiple numbered concepts coexist in this project. Keep them distinct.

| Term | Meaning | Source |
|------|---------|--------|
| **Phase 0–6** | Development stages defined in §4 of this doc | This doc §4 |
| **Layer 1–4** | Layered plant-identification stack (ACRO → STAB → ALT → POS) | This doc §3 |
| **Gates G1–G4** | SIL acceptance criteria (boot/state transitions / estimator tracking / closed-loop stability / actuator health) | RESET_PLAN §4 |
| **Noise Model Stage N0–N4** | Sensor-noise complexity stages (educational) | `noise_and_vibration_model.md` §4 |

> **The old "SIL Control Level L1–L4" has been retired.** It referred to the control-test levels of the old SIL (`sim/flight_scenario_test.cpp` etc.); it disappeared when the old SIL was fully removed (RESET_PLAN §12). The new SIL judges pass/fail with the physics-truth gates **G1–G4** (RESET_PLAN §4).

---

## 2. Three Principles of Development

The SIL → real-flight workflow rests on three principles — the same ideas as RESET_PLAN's "two basic policies," restated in workflow terms.

### Principle 1: Code Identity (at the loop level)

**SIL compiles and runs the unmodified vehicle source as-is.**

The old SIL's failure was rebuilding the control loop by hand and feeding the estimator the physics-truth attitude. The new SIL runs the **entire real Pub-Sub loop** (`imu_task → estimate_state → control_task → actuator_motor`) on the host. What matches is not just the ESKF math — it is the **whole loop**.

| Shared | Method |
|--------|--------|
| Entire firmware source (Pub-Sub loop + all tasks + estimator/controller implementations + math) | SIL compiles it **by reference, unmodified**, on an ESP-IDF-compatible shim (RESET_PLAN §7) |
| Choice of estimator/controller | Through the exact same `IEstimator` / `IController` as on hardware — SIL does not depend on the inside being ESKF/PID |

So "passes in SIL" **means the same behavior on hardware** (loop-level Code Identity). Because SIL does not depend on the algorithm inside, swapping ESKF for a complementary filter or state feedback needs **no change to the bench** (RESET_PLAN policy 2).

### Principle 2: Parameter Identity

**SIL and real hardware both read `params.cpp` (`param_vars` + `table[]`) as the single source of truth.** (The original `params.def` X-macro codegen was removed in Phase 5b — it had drifted out of use; `params.cpp`'s explicit table is the SSOT. Parameter Identity is unchanged: both builds reference-compile `params.cpp`.) Tuning values flow between SIL and real-hardware NVS through the same schema.

### Principle 3: Model Fidelity (post-flight, after-the-fact)

**SIL's ground truth comes from the physics model — no real data needed.** Since we are simulating, the true attitude/position/velocity are always known, so building and running the SIL needs no real flight data (RESET_PLAN policy 1).

Raising Model Fidelity (how well the physics matches reality) begins **only after the first real flight** — an after-the-fact refinement (the `[5]→[2]` path in RESET_PLAN §3). Real logs are used **only** there, never as a prerequisite for SIL. The wider the SIL model's trustworthy envelope, the higher the certainty of "tuned in SIL → flies on hardware."

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

### Validating Each Layer via SIL Gates

Each layer is first validated against the **physics-truth SIL** (RESET_PLAN) and must clear gates G1–G4 (RESET_PLAN §4) before going to hardware. There is no dependence on the old SIL's control-test-level numbering like L1–L4.

**SIL Layer 1 ≡ real ACRO structurally.** A rate PID that passes the SIL gates should pass real ACRO; if it does not, the plant model or the synthetic-sensor noise model has diverged from reality (RESET_PLAN §3 differential diagnosis).

---

## 4. Phase Plan

(See the Japanese section above for the detailed phase tables — same structure applies.)

Overall order: **① build the SIL → ② resume vehicle development → ③ fly in SIL → ④ real flight.**

- **Phase 0**: Clean slate (achieved) — design docs + skeleton + ESKF/PID done; **old SIL fully removed**. The old SIL's recorded results (L1–L4, 2.27°/44 mm) belonged to the deleted SIL and are dropped from current results.
- **Phase 1**: Rebuild the physics-based SIL (highest priority) — per RESET_PLAN P1–P4: integrate MuJoCo, rebuild the ESP-IDF shim, run the real loop on the host, finish the firmware last mile (mixer, motor output, estimator/controller factory, `@design` → `[OK]`). Gate: current ESKF+PID hovers in SIL + a complementary filter swaps in with no bench change.
- **Phase 2**: HAL connection (hardware comes alive)
- **Phase 3**: First real flight — ACRO identification (key milestone)
- **Phase 4**: Stack higher layers (STABILIZE → ALT_HOLD → POS_HOLD), each SIL-gated then hardware
- **Phase 5**: Continuous model-fidelity calibration loop (post-flight)
- **Phase 6**: Educational productization

---

## 5. Governance

- Each phase must clear its acceptance criteria before the next phase begins
- At every milestone (a phase completion or a SIL gate G1–G4 pass), **produce the review video** (mandatory per RESET_PLAN §9/§11) — the human-verifiable artifact for that node
- Real-vs-SIL gaps exceeding tolerance trigger a return to Phase 5 model calibration
- Design-vs-implementation conflicts halt implementation pending discussion (per coding_and_education.md §1)

---

## 6. Related Documents

| Doc | Role |
|-----|------|
| `requirements.md` | What to build |
| `architecture.md` | Component structure |
| `detailed_design.md` | Interfaces and state transitions |
| `hardware_init.md` | BSP and hardware initialization |
| `coding_and_education.md` | Coding standards and education plan |
| `noise_and_vibration_model.md` | Sensor noise/vibration models (SIL synthetic-sensor spec) |
| `implementation_log.md` | Implementation timeline |
| `../../../simulator/sil/RESET_PLAN.md` | **SIL bench rebuild plan (source of truth for how to build the SIL)** |
| **`development_roadmap.md` (this doc)** | Development workflow and phase plan (overall) |
