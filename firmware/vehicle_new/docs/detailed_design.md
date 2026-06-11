# vehicle_new Detailed Design
# vehicle_new 詳細設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本文書はvehicle_newの詳細設計を定義する。アーキテクチャ設計書（architecture.md）に基づき、実装レベルの仕様を記述する。

| 項目 | 内容 |
|------|------|
| Pub-Subフレームワーク | トピック定義、バッファ方式、API |
| 状態遷移テーブル | onExit/onEnterコールバック |
| 制御インターフェース定義 | IController |
| 状態推定インターフェース定義 | IEstimator |
| パラメータシステム | マクロ1行定義、バリデーション、コールバック |
| センサ観測スイッチ | パラメータ連動、推定内部で判断 |
| ディレクトリ構造 | ESP-IDFプロジェクト配置 |
| メモリ配置 | パーティション、Blackbox |

## 2. Pub-Subフレームワーク

### 設計方針

- StampFlyの制約に最適化した独自の軽量設計
- 同一MCU内のタスク間通信のみ
- シリアライズ不要（構造体メモリ直接共有）
- 全トピックはコンパイル時に確定
- 内部バッファ方式はデータフローの特性に応じて選択

### トピック定義

トピックの実装方針は **ヘッダで `extern` 宣言、対応する `.cpp` で実体定義** とする。`inline` グローバル変数は ESP-IDF のリンク方針と相性が悪く（複数翻訳単位を跨ぐ初期化順序に依存しやすい）、`extern` + 実体定義のほうが安全。

```cpp
// topics.hpp — 全トピックの宣言（1箇所）
// All topic declarations (single location)
// トピック追加はここに1行追加し、対応する .cpp に定義を1行追加する
// Adding a topic: add one extern line here + one definition line in .cpp

#include "topic.hpp"
#include "data_types.hpp"

namespace sf {

// Topic<DataType, BufferPolicy, BufferSize>
extern Topic<ImuData,         RingBuffer, 8>  sensor_imu;
extern Topic<TofData,         Queue, 2>       sensor_tof;
extern Topic<FlowData,        Queue, 2>       sensor_flow;
extern Topic<MagData,         Queue, 2>       sensor_mag;
extern Topic<BaroData,        Queue, 2>       sensor_baro;
extern Topic<PowerData,       Latest, 1>      sensor_power;
extern Topic<StateEstimate,   Latest, 1>      estimate_state;
extern Topic<CommandSetpoint, Latest, 1>      command_setpoint;
extern Topic<ControlOutput,   Latest, 1>      control_output;
extern Topic<MotorOutput,     Latest, 1>      actuator_motor;
extern Topic<SystemMode,      Latest, 1>      system_mode;
extern Topic<SystemAlert,     Queue, 4>       system_alert;

}  // namespace sf
```

```cpp
// topics.cpp — トピック実体定義
// Topic instance definitions

#include "topics.hpp"

namespace sf {

Topic<ImuData,         RingBuffer, 8>  sensor_imu;
Topic<TofData,         Queue, 2>       sensor_tof;
// ... 以下同様
// ... and so on

}  // namespace sf
```

### バッファ方式（3種）

| 方式 | 特性 | 用途 |
|------|------|------|
| RingBuffer | Lock-free SPSC、全サンプル保持、ISR安全 | IMU→推定、全データ→ログ |
| Queue | FreeRTOS Queue、バッファリング | 低レートセンサ（ToF/Flow/Mag/Baro） |
| Latest | 最新値上書き、共有メモリ | 推定値→制御、推定値→テレメトリ |

### API

```cpp
// Publish (producer side)
// 発行（生産者側）
ImuData data = readSensor();
sensor_imu.publish(data);

// Subscribe with callback (consumer side, at init)
// コールバックで購読（消費者側、初期化時に登録）
sensor_imu.subscribe([this](const ImuData& d) {
    this->onImuData(d);
});

// Poll latest value
// 最新値のポーリング
auto data = estimate_state.latest();
```

### トピック追加手順

1. `data_types.hpp` に構造体を追加（`timestamp` フィールド必須）
2. `topics.hpp` に `extern Topic<>` 宣言を1行追加
3. `topics.cpp` に対応する実体定義を1行追加
4. 発行側で `publish()`、購読側で `subscribe()` または `latest()` / `read()`
5. **`docs/topic_reference.md` §3 の表に 1 行追加**（R9 — Topic SSOT 文書）
6. **publisher / subscriber 側のタスクヘッダに `@publisher` / `@subscriber` アノテーション**を追加（R13）

### Publisher / Subscriber アノテーション規約（R13）

各タスク（`tasks/*.cpp`）の冒頭 Doxygen ブロックに、利用する Topic を明記する。これにより「どの Topic を誰が publish / subscribe するか」がコードを読むだけで分かり、`topic_reference.md` の表との一貫性も担保される。

```cpp
/**
 * @file imu_task.cpp
 * @brief IMU reading and state estimation task (400Hz)
 *
 * @publisher  sensor_imu, estimate_state
 * @subscriber sensor_tof, sensor_flow, sensor_mag, sensor_baro
 *
 * @design architecture.md §6 — ImuTask                                [OK]
 * @design detailed_design.md §8 — ImuTask: 400Hz, priority 24         [OK]
 */
```

将来は lint で「アノテーション ↔ 実コード」の整合検証も視野に入れる。

### 将来予約 Topic（v3 設計、未実装）

v3 設計で 4 つの Topic を予約定義した。実装は後続マイルストーンで行うが、置き場所と入出力契約は今のうちに確定する（R11）。

| Topic | データ型 | バッファ | Publisher | 用途 | 実装予定 |
|-------|---------|--------|-----------|------|---------|
| `sensor_imu_raw` | `ImuRawData` | RingBuffer 8 | ImuTask | キャリブ前生 IMU。L2 学習者向け教材 | M2 |
| `sensor_health` | `SensorHealth` | Latest | sf_board / 各 task | publisher 死活監視（presence / last_update_us / quality） | M2 |
| `command_target` | `GuidanceTarget` | Latest | Guidance / Navigator | 位置 + yaw target、ウェイポイント | M4+ |
| `nav_path` | `NavigationPath` | Queue 4 | Navigator | 経路シーケンス | Phase 6 |

詳細な使用方針・データ型定義・選定根拠は [`topic_reference.md`](topic_reference.md) §3.2 を参照。

## 3. 状態遷移テーブル

### onExit/onEnterコールバック

> **注: このテーブルはクラスA（遷移リセット）のみを定義する。** センサ1サンプル精度を要する推定器内部の reset（クラスB、例: 離陸時の鉛直ハンドオフ `resetPositionVelocity`/`holdPositionVelocity`）はこのテーブルの対象外で、そのセンサを観測するタスク（ImuTask）が所有する。クラスA / クラスB の分類基準・境界条件・越権防止制約は [`architecture.md`](architecture.md) §4「リセット処理の2層分類」を参照。

| 遷移 | onExit（旧状態を出る時） | onEnter（新状態に入る時） |
|------|------------------------|------------------------|
| INIT → IDLE_GROUND | — | キャリブレーション管理を起動 |
| IDLE_GROUND → IDLE_HELD | （リザーブ） | （リザーブ） |
| IDLE_HELD → IDLE_GROUND | （リザーブ） | （リザーブ） |
| IDLE_GROUND → ARMED_GROUND | （リザーブ） | 全PIDリセット、ESKF**姿勢共分散の膨張**（注1）、ブザー(arm音) |
| ARMED_GROUND → TAKEOFF | （リザーブ） | 離着陸MGR: 離陸シーケンス開始、高度目標セット |
| TAKEOFF → FLYING | 離着陸MGR: シーケンス終了 | ESKF位置/速度リセット（注2: クラスB, ImuTask）、~~バイアスフリーズ解除~~（注3で見送り） |
| サブモード切替（地上/FLYING） | 旧サブモードのコントローラリセット | 新サブモードの初期化（高度キャプチャ等）。地上では制御器の鉛直フェーズゲート（Grounded=推力ゼロ）により ALT/POS 選択でもモータは回らない |

### 3.1 モード調停表（規範 / Normative）

**この表が唯一の正であり、コード（state_task のモード調停・StateManager::requestModeChange）は本表を写す。**
表に無い (状態, 事象) の組合せを実装してはならず、表の全セルが実装されていなければならない
（抜けのない設計 — 実機 LED バグ①と TAKEOFF 窓の適用漏れ②は本表の作成によって発見・修正された）。

事象の定義:
- **エッジ**: モードスイッチの位置変化（want ≠ prev_want）。拒否された場合 prev_want を更新せず**持続**（受理される状態に達した時点で適用）
- **不一致**: スイッチ位置と実モードの不一致（want ≠ mode、エッジなし）。接地リセット等で発生
- **API設定**: ApiCmd::Takeoff によるモード設定（同一周期で ARM まで進む）
- **接地リセット**: 空中状態→IDLE_GROUND 突入時の STABILIZE 強制（再飛行安全則）

| 状態 \ 事象 | エッジ | 不一致（レベル） | API設定 | 接地リセット |
|------------|--------|----------------|---------|------------|
| INIT | 拒否（エッジ持続） | 無視 | 拒否 | — |
| IDLE_GROUND | **適用** | **適用（再適用・マーカーログ）** | **適用** | （突入時に発火し、直後に不一致則が再適用） |
| IDLE_HELD | 拒否（持続→設置時に適用） | 無視 | 拒否 | — |
| ARMED_GROUND | **適用** | 無視（エッジのみ） | **適用** | — |
| TAKEOFF | 拒否（持続→FLYING で適用） | 無視 | 拒否 | — |
| FLYING | **適用** | 無視（放置送信機が API を覆せない） | （誘導目標のみ） | — |
| LANDING | 拒否（持続→接地後に適用） | 無視 | 拒否 | — |

検証: SIL `acro_crash_relevel`（①の固定）、`api_flight`（FLYING 不一致の無視＝API保護）、
`modeswitch`/`alt_flight`（FLYING エッジ適用）、`alt_auto_takeoff`（IDLE_GROUND API設定）。
| ARMED_GROUND → TAKEOFF（ALT/POS 選択時） | — | ControllerCmd::Takeoff 発行 → 制御器が自動離陸（固定上昇率＋水平姿勢、POS は発進点保持） |
| TAKEOFF → FLYING | — | ControllerCmd::TakeoffComplete 発行 → 通常モード則を係合（ALT_HOLD は現在高度を捕捉） |
| FLYING → LANDING | （リザーブ） | 離着陸MGR: 着陸シーケンス開始 |
| FLYING → ARMED_GROUND | 高度/位置コントローラリセット | ESKFホールド |
| FLYING → IDLE_GROUND | 高度/位置コントローラリセット | モーター停止、ESKFリセット、ブザー(disarm音) |
| LANDING → IDLE_GROUND | 離着陸MGR: シーケンス終了 | モーター停止、ESKFリセット、~~バイアスフリーズ~~（注3で見送り） |
| ARMED_GROUND → IDLE_GROUND | （リザーブ） | モーター停止、ブザー(disarm音) |

「リザーブ」は実装・テスト時に必要に応じて追加する。

**注1（ARM時の ESKF 処理 — SIL 掃引で確定）:** 当初は「ARM時 ESKF 全リセット」（地上の共分散収束は飛行を代表しない、という根拠）だったが、SIL のリセットタイミング掃引（8方策×飛行スイート）で、全リセット — および位置/速度/バイアスの共分散の膨張 — は離陸過渡を不安定化すると判明した。再膨張した共分散がスラスト汚染された加速度計を過信し、POS_HOLD 姿勢が発散する（pos_roll/pitch/flight 墜落）。飛行スイート全PASS は2方策のみ＝「何もしない」と「**姿勢の共分散だけ膨張**」。後者を採用：設計意図（ARM で姿勢の自信をリセット）を満たしつつ、姿勢は離陸前に地上で重力から再収束するため安定。実装は `EstimatorCmd::InflateCov(CovScope::Attitude)`（推定値 x は保持し姿勢共分散のみ初期値へ）。

**注2（位置/速度リセット = クラスB）:** 「ESKF 位置/速度リセット」はタイミング命の ToF 同期鉛直ハンドオフ（クラスB, `ImuTask::applyVerticalGroundHandoff`）。状態機械の onEnter 経由（~20ms 遅れ）に移すと α-βトラッカ初期化が遅れ POS_HOLD 姿勢が劣化する（[`architecture.md`](architecture.md) §4 クラスA/B 分類）。地上ゼロは飛行を代表しないが、その手当ては空中エッジで ImuTask が正確に行う。

**注3（バイアスフリーズ/解除 = 見送り）:** ESKF の凍結機構（`active_mask`＋`enforceCovarianceConstraints`）は「センサ恒久不在」用の隔離（凍結状態の共分散を毎周期 init 値へ戻す）で、地上↔飛行でトグルする用途とは非互換。トグルすると解除時に巨大な共分散が復活し、注1と同じ離陸発散を起こす（SIL で実証）。よってバイアス推定は飛行中も常時アクティブとし、フリーズ/解除は配線しない。地上振動からの校正バイアス保護が必要になった場合は、共分散を init に戻さない「ソフト凍結」を別途設計する（将来課題）。

### ペアリング状態遷移（PairingState — FlightState と並行）

FlightState とは別の独立状態機械。StateManager が所有する（[`architecture.md`](architecture.md) §4
「ペアリング状態の位置づけ」参照）。上の FlightState onExit/onEnter テーブルとは別物。

| 遷移 | トリガー | アクション |
|------|---------|-----------|
| 起動 → Paired / NotPaired | NVS load | 相手 MAC あり→Paired・相手をユニキャスト peer 登録 ／ なし→NotPaired |
| NotPaired → Pairing | 自動（未ペア起動） | comm: PairingPacket を 500ms 周期で broadcast 開始、notify: LED青速点滅+ブザー |
| 任意 → Pairing（再ペア） | `button_event`=LongPress3s（IDLE_GROUND のみ） | 既存ペア破棄（NVS clear）→ 上記 Pairing 開始 |
| Pairing → Paired | comm 発行の `pairing_complete`（src MAC 学習） | comm: NVS保存・broadcast peer 削除・相手をユニキャスト peer 登録、notify: 点滅解除 |

**ガード:** Pairing 中は `requestArm` を拒否（StateManager）。突入は地上（IDLE_GROUND）のみ。

**トピック:**
- `pairing_state`（Latest, StateManager → comm / notify）: 現在の PairingState を周知。
- `pairing_complete`（Latest, comm → StateManager）: comm の現在のバインド状態（bound + 学習/復元した
  送信機 MAC）。起動時の NVS 復元（restored=true）と Pairing 成立（restored=false）の両方を運ぶ。

## 4. 制御インターフェース定義

PID/MPC/LQRを差替可能にするための統一インターフェース。

```cpp
// Control interface definition
// 制御インターフェース定義

class IController {
public:
    virtual ~IController() = default;

    // Called every control cycle (400Hz)
    // 制御周期ごとに呼ばれる（400Hz）
    virtual ControlOutput compute(
        const StateEstimate& state,      // Current estimated state
        const CommandSetpoint& setpoint, // Target setpoint
        float dt                         // Time step [s]
    ) = 0;

    // Reset internal state (integrators, filters, etc.)
    // 内部状態リセット（積分器、フィルタ等）
    virtual void reset() = 0;

    // Mode change notification
    // モード変更通知
    virtual void onModeChange(uint8_t new_mode) = 0;
};
```

### PID 実装の離散化方式

`sf_controller_pid` の PID 実装は **双線形変換（Tustin 法）** を採用する。後退差分による離散化は α = Td/(η·Td+dt) > 1 となり微分フィルタが発散する（旧 SIL のプロトタイピング時に判明、コミット `06b4cd6`／git 履歴）。これを避けるため bilinear に統一した（コミット `07005fb`、vehicle/ 旧実装と一致）。

| 項 | 離散化 |
|----|--------|
| 積分（trapezoidal）| `integral += (kp/ti) · (error + prev_error) · dt/2` |
| 微分（bilinear）| α = 2·η·Td/dt、a = (α−1)/(α+1)、b = 2·Td/(dt·(α+1))<br>`d[n] = a·d[n−1] + b·(e[n] − e[n−1])`<br>Kp は filter 外で適用: `d_term = kp · d[n]` |

η = 0.125、Td = 0.01、dt = 0.0025 で α = 1.0、a = 0、b = 4.0 の安定動作。定量的な検証は、新しい物理真値ゲート **G1〜G4**（`../../../simulator/sil/RESET_PLAN.md` §4）で再取得する（旧 SIL での L1〜L4 数値は削除済み・git 履歴に保存）。

### ヘディングホールド（STABILIZE 以上）

レートループはヨー「角速度」しか帰還しないため、定在ヨー外乱トルク下では方位が操縦修正の合間にランダムウォークする（2026-06-11 実機で確認: 推力比例 CW モーメント下、手放し方位ずれ平均 12.3°）。これを推定ヨー角（バイアス補正済みジャイロ積分 — 地磁気不要）への外側 P ループで抑える。

| 項目 | 内容 |
|------|------|
| 係合 | ヨースティック中立（\|yaw\| < 0.03）かつ空中（STABILIZE: throttle > 0.25 / ALT_HOLD 以上: VerticalPhase == Airborne）|
| 目標捕捉 | 係合エッジで現在の推定ヨー角を捕捉。スティック入力で即解除（パイロット優先）、次の中立で再捕捉 |
| 制御則 | `rate_ref_yaw = clamp(kp · wrap(ψ_target − ψ), ±rate_max)` — 誘導ヨー則と同形 |
| 除外 | ACRO（純レート制御）、誘導アクティブ中（誘導がヨー所有）、自動離陸中、墜落リセット・モード切替で解除 |
| パラメータ | `attitude.yawhold.kp`（既定 3.0 [1/s]、0 で無効）、`attitude.yawhold.rate_max`（既定 2.0 [rad/s]）|
| 検証 | フライトログ再生で方位ずれ 12.3°→5.7°（平均）。SIL ゲート `yaw_hold.scn`: M1 80% 故障下 8 秒手放しで yaw_band < 1.5°（無効化対照 3.1° 単調流出）|

## 5. 状態推定インターフェース定義

ESKF/EKF/Complementary Filterを差替可能にするための統一インターフェース。

```cpp
// Estimation interface definition
// 状態推定インターフェース定義

class IEstimator {
public:
    virtual ~IEstimator() = default;

    // IMU prediction step (400Hz)
    // IMU予測ステップ（400Hz）
    virtual void predict(const ImuData& imu, float dt) = 0;

    // Sensor observation updates (async, called when data arrives)
    // センサ観測更新（非同期、データ到着時に呼ばれる）
    virtual void updateTof(const TofData& tof) = 0;
    virtual void updateFlow(const FlowData& flow) = 0;
    virtual void updateMag(const MagData& mag) = 0;
    virtual void updateBaro(const BaroData& baro) = 0;

    // Get current state estimate
    // 現在の推定値取得
    virtual StateEstimate getState() const = 0;

    // Reset full state
    // 全状態リセット
    virtual void reset() = 0;

    // Reset position and velocity only
    // 位置・速度のみリセット
    virtual void resetPositionVelocity() = 0;
};
```

### センサ観測スイッチ

センサ観測のON/OFFはパラメータシステムと連携し、推定コンポーネント内部で判断する。

```
sensor.tof → 推定コンポーネントに常に届く
           → 推定内部で eskf.use_tof パラメータを確認
           → false なら観測更新をスキップ
           → ログには常に記録される
```

パラメータ変更時のコールバックで推定コンポーネントの内部マスクを即時更新する。これにより段階的にセンサを有効化しながらデバッグが可能。

### ESKF 実装の特性

`sf_estimator_eskf` には次の機能が実装されている（χ²ゲートは実機 vehicle/ の知見、Adaptive R・線形化バイアスは旧 SIL のプロトタイピング由来＝git 履歴。新 SIL での定量検証は G1〜G4 で行う）。

#### χ²ゲート（外れ値棄却）

各観測更新前にイノベーション e と共分散 S = H·P·Hᵀ + R から χ² = eᵀ·S⁻¹·e を計算し、自由度に応じた閾値を超える観測を棄却する。実機 vehicle/ で「ふらつきが PID ゲイン変更なしでほぼ解消」した実績あり（加速度計の外れ値が `updateAccelAttitude` で姿勢を乱していたのを抑制）。コミット `d9172c4`。

#### Adaptive R（加速度姿勢補正の thrust 汚染対策）

加速度計が観測する加速度は厳密には `[0, 0, −T/m]`（body frame）であり、ホバー以外では `R^T·g` から乖離する。15° ロールでは |a| ≈ g/cos(15°) で、ノルムゲートは通過するが方向は誤りで、姿勢推定が破綻する。

対策として観測ノイズを **R_actual = R_base² × (1 + k_adaptive · |a − g|²)** で動的に膨らませ、g から外れた観測の重みを下げる。`k_adaptive = 50` を default とする。旧 SIL のプロトタイピング（0.3Hz サイン応答、git 履歴）での参考値:

| 指標 | Before | After | 改善 |
|------|--------|-------|------|
| ゲイン | 2.34× | 2.00× | — |
| トラッキング誤差 | 16.5° | 9.1° | 45% |
| ホバー姿勢 RMS | 3.94° | 3.87° | 維持 |

コミット `98839a6`。**根本解（thrust 補償観測モデル）は Phase 5 へ繰越**。

#### 線形化バイアス（既知の構造的限界）

加速度ベース姿勢補正の観測モデル `h(x) = R^T·g` はホバー近傍でのみ妥当な線形化。10° ステップ保持を 200s 実行すると、ESKF 推定値と真値の間に **−2.3° の定常バイアス** が残り、収束しない。これは推力寄与（`a_body = [0, 0, −T/m]`）が観測モデルから抜けているための構造的限界で、ESKF + PID ループのフィードバック構造により ESKF は指令値を追跡し、真値はバイアス分ずれる。

復帰時間は 1〜3s。ホバー定点用には十分だが、姿勢追跡用途には限界あり。コミット `8a2e6ca`。教材として「線形化が破綻する条件」の定量データに位置付ける。

## 6. パラメータシステム

### 設計方針

全パラメータの SSOT は `params.cpp`。型付き変数（`namespace param_vars`）と、名前→変数→
既定/最小/最大/コールバックを結ぶ明示的 `table[]` で、定義・バリデーション・コールバック・
NVS 永続化を完結させる。明示テーブルは意図的（模範コードとして読みやすく、値・範囲・根拠が
見える）。

> 注（Phase 5b, 2026-06-07）: 当初は `params.def` の X-macro コード生成を想定していたが、
> 実体は `params.cpp` の手書きテーブルに収束していた。`params.def` は非機能で値もずれて
> いたため撤去し、`params.cpp` を正式 SSOT とした（[[reference_params_ssot]]）。

### パラメータ定義（`params.cpp`）

```cpp
// 1) namespace param_vars に型付き変数を追加（既定値もここ）
namespace param_vars {
    float rate_roll_kp = 1.83e-4f;   // Ixx/τ_resp（物理ゲイン）
    // ...
}

// 2) table[] に行を追加（名前 → 変数 → 既定 / min / max / コールバック）
static const ParamEntry table[] = {
    //   名前              型               変数ポインタ      既定       min    max     callback
    {"rate.roll.kp", ParamType::FLOAT, &rate_roll_kp, 1.83e-4f, 0.0f, 0.01f, nullptr},
    // ...
};
// パラメータ追加＝この2箇所（同一ファイル）に書く
```

### アクセスAPI

```cpp
// Read parameter value (by name, into an out-param; returns false if not found)
// パラメータ値の読み取り（名前で out 引数へ。未発見なら false）
float kp = 0.0f;
params::get_float("rate.roll.kp", kp);

// Write parameter value (from WiFi/CLI)
// パラメータ値の書き込み（WiFi/CLIから）
// → バリデーション → 値更新 → コールバック実行
params::set_float("rate.roll.kp", 2.0e-3f);

// List all parameters (for CLI)
// 全パラメータ一覧（CLI用）
params::list();

// Init (load from NVS, else table defaults) / Save all to NVS / Load from NVS
// 初期化（NVS から、無ければ table 既定）/ NVS 一括保存・読み込み
params::init();
params::save();
params::load();

// Reset to defaults
// デフォルト値にリセット
params::resetAll();
```

### 命名規則

3階層: `機能.対象.パラメータ`

```
rate.roll.kp              # Rate control, roll axis, proportional gain
rate.pitch.ti             # Rate control, pitch axis, integral time
attitude.roll.kp          # Attitude control, roll axis, proportional gain
altitude.alt.kp           # Altitude control, altitude loop, Kp
altitude.vel.kp           # Altitude control, velocity loop, Kp
position.pos.kp           # Position control, position loop, Kp
eskf.process.gyro_noise   # ESKF, process noise, gyro
eskf.obs.tof_noise        # ESKF, observation noise, ToF
eskf.gate.tof_innov       # ESKF, innovation gate, ToF
eskf.use_tof              # ESKF, sensor enable, ToF
safety.impact.accel_threshold  # Safety, impact detection, accel threshold
```

## 7. ディレクトリ構造

HALコンポーネントはvehicle_new内にコピー（完全独立）。

```
firmware/vehicle_new/
├── CMakeLists.txt                 # ESP-IDF project root
├── partitions.csv
├── sdkconfig.defaults
├── docs/
│   ├── requirements.md            # 要件定義書
│   ├── requirements.tex/pdf       # 要件定義書（PDF版）
│   ├── architecture.md            # アーキテクチャ設計書
│   └── detailed_design.md         # 詳細設計書（本文書）
├── main/
│   ├── CMakeLists.txt
│   ├── main.cpp                   # app_main(), INIT逐次実行
│   └── config.hpp                 # 固定パラメータ（constexpr）
├── components/
│   ├── sf_core/                   # 基盤: Pub-Sub, パラメータ, データ型
│   │   ├── include/
│   │   │   ├── topic.hpp          # Topic<T> テンプレート
│   │   │   ├── topics.hpp         # 全トピック定義
│   │   │   ├── data_types.hpp     # 全構造体定義
│   │   │   └── params.hpp         # パラメータ公開API
│   │   └── params.cpp             # パラメータ SSOT（param_vars + table[]）
│   ├── sf_state/                  # 状態管理
│   │   └── include/
│   │       ├── state_manager.hpp  # 状態遷移、onExit/onEnter
│   │       └── flight_state.hpp   # enum定義
│   ├── sf_estimator/              # 状態推定インターフェース
│   │   └── include/
│   │       └── estimator.hpp      # IEstimator定義
│   ├── sf_estimator_eskf/         # ESKF実装
│   ├── sf_controller/             # 制御インターフェース
│   │   └── include/
│   │       └── controller.hpp     # IController定義
│   ├── sf_controller_pid/         # PID実装
│   ├── sf_actuator/               # ミキサー＋モーター出力
│   ├── sf_command/                # コマンド処理
│   ├── sf_failsafe/               # フェイルセーフ
│   ├── sf_takeoff_landing/        # 離着陸マネージャー
│   ├── sf_logger/                 # データロガー＋Blackbox
│   ├── sf_telemetry/              # テレメトリ
│   ├── sf_notify/                 # LED/ブザー通知
│   ├── sf_calibration/            # キャリブレーション管理
│   ├── sf_comm/                   # 通信（ESP-NOW/UDP/TCP）
│   ├── sf_hal_bmi270/             # IMUドライバ（コピー）
│   ├── sf_hal_bmm150/             # 地磁気ドライバ（コピー）
│   ├── sf_hal_bmp280/             # 気圧ドライバ（コピー）
│   ├── sf_hal_vl53l3cx/           # ToFドライバ（コピー）
│   ├── sf_hal_pmw3901/            # OptFlowドライバ（コピー）
│   ├── sf_hal_motor/              # モータードライバ（コピー）
│   ├── sf_hal_led/                # LEDドライバ（コピー）
│   ├── sf_hal_buzzer/             # ブザードライバ（コピー）
│   ├── sf_hal_button/             # ボタンドライバ（コピー）
│   └── sf_hal_power/              # 電源モニタドライバ（コピー）
├── tasks/
│   ├── tasks.hpp                  # タスク関数宣言
│   ├── imu_task.cpp               # IMU + 状態推定
│   ├── control_task.cpp           # 制御 + アクチュエーション
│   ├── state_task.cpp             # 状態管理（イベント駆動）
│   ├── flow_task.cpp              # OptFlow
│   ├── mag_task.cpp               # 地磁気
│   ├── baro_task.cpp              # 気圧
│   ├── comm_task.cpp              # 通信 + コマンド処理
│   ├── tof_task.cpp               # ToF + 離着陸マネージャー
│   ├── telemetry_task.cpp         # テレメトリ
│   ├── power_task.cpp             # 電源 + フェイルセーフ
│   ├── button_task.cpp            # ボタン入力
│   ├── notify_task.cpp            # LED/ブザー
│   ├── cli_task.cpp               # CLI + パラメータ
│   └── log_task.cpp               # データロガー + Blackbox
└── examples/                      # サンプル集（チュートリアル的コメント）
    ├── blink_led/
    ├── read_imu/
    ├── pid_motor/
    ├── eskf_sim/
    ├── tof_altitude/
    └── espnow_pair/
```

> **注（`examples/espnow_pair/` と本体ペアリング機能の区別）:** `examples/espnow_pair/` は ESP-NOW
> ペアリングの**最小教育例**（単独ビルド可能・PairingPacket の送受信を学ぶ短いコード）。本体の
> ペアリング機能（混信対策・状態統合・NVS 永続化）は `sf_comm` ＋ `sf_state` が担い、本書 §3
> 「ペアリング状態遷移」が正典。両者は別物として扱う。

## 8. メモリ配置

### パーティションテーブル

```
# Name,    Type, SubType, Offset,   Size,     Flags
nvs,       data, nvs,     0x9000,   0x6000,           # 24KB — パラメータ永続化
phy_init,  data, phy,     0xF000,   0x1000,           # 4KB  — WiFi/BT校正
factory,   app,  factory, 0x10000,  0x300000,          # 3MB  — ファームウェア
storage,   data, spiffs,  0x310000, 0x200000,          # 2MB  — Blackbox
# 残り 2MB (0x510000〜0x800000) は未割当（将来用）
```

### Blackbox容量

| 記録レート | 容量 | 記録時間 |
|-----------|------|---------|
| 400Hz / 128B | 50KB/s | 約40秒 |
| 50Hz / 128B | 6.25KB/s | 約5分 |

### Blackboxアクセス

- CLI経由: `sf log download` でUSBシリアル経由転送
- 将来: USBマスストレージも検討可能だが初期実装はCLIで統一

### RAMメモリ（ESP32-S3 512KB）

タスクスタックの合計: 約92KB

| タスク | スタック |
|--------|---------|
| ImuTask | 16KB |
| ControlTask | 8KB |
| StateTask | 4KB |
| FlowTask | 8KB |
| MagTask | 8KB |
| BaroTask | 8KB |
| CommTask | 4KB |
| TofTask | 8KB |
| TelemetryTask | 4KB |
| PowerTask | 4KB |
| ButtonTask | 4KB |
| NotifyTask | 4KB |
| CLITask | 8KB |
| LogTask | 4KB |
| **合計** | **92KB** |

残り約420KBでPub-Subバッファ、ESKF行列、ヒープ等を賄う。

## 9. Guidance / Navigation 層の予約設計（v3）

### 設計の動機

vehicle_new は「学習者がレイヤーを段階的に登れる」ことを目標にしている（[`architecture.md`](architecture.md) §2.5「学習者の入口」）。学習段階の最終形は **ガイダンス（目的地・経路指定）→ ナビゲーション（経路計画）** 層であり、要件 (`requirements.md` §4) でも「ナビゲーター（将来追加）」として位置づけられている。

実装は後続フェーズ（M4 以降、Phase 6）で行うが、**Topic 上の置き場所と入出力契約を v3 で確定**する（R11）。これにより：
- Guidance / Navigation を学びたい学習者が「自分のコードはどこに書くのか」を最初から把握できる
- 既存 Controller / Estimator のインターフェースを変更せずに追加できる構造を保証

### 階層構造

```
┌─────────────────────────────────────────────┐
│  L4 Application: free-style learner code     │
│   "ウェイポイントを設定して飛ばす"            │
├─────────────────────────────────────────────┤
│  Navigator    (`sf_navigator`、将来実装)       │
│   経路計画 → nav_path Topic                    │
├─────────────────────────────────────────────┤
│  Guidance     (`sf_guidance`、将来実装)        │
│   目的地 + nav_path → command_target Topic     │
├─────────────────────────────────────────────┤
│  Controller   (既存 sf_controller_pid)         │
│   command_target / command_setpoint           │
│           → control_output                    │
├─────────────────────────────────────────────┤
│  Estimator    (既存 sf_estimator_eskf)        │
└─────────────────────────────────────────────┘
```

### Topic 入出力契約

| Topic | データ型（予定） | Publisher | Subscriber | Rate |
|-------|-------------|-----------|-----------|------|
| `nav_path` | `NavigationPath` (waypoint sequence) | Navigator | Guidance | 1Hz |
| `command_target` | `GuidanceTarget` (position [m] + yaw [rad]) | Guidance / Navigator | ControlTask, NotifyTask | 10Hz |

`NavigationPath` データ型案:
```cpp
struct Waypoint {
    float x_m, y_m, z_m;
    float yaw_rad;
    float velocity_mps;        // 通過時の希望速度
    float acceptance_radius_m; // 到着判定半径
};

struct NavigationPath {
    uint32_t timestamp;
    uint8_t  num_waypoints;
    Waypoint waypoints[8];     // MVP は最大 8 点
    uint8_t  current_index;    // 現在追跡中のインデックス
};
```

`GuidanceTarget` データ型案:
```cpp
struct GuidanceTarget {
    uint32_t timestamp;
    float    position_target_m[3];  // x, y, z (NED)
    float    yaw_target_rad;
    float    velocity_target_mps[3]; // 速度 feed-forward (optional)
    bool     is_active;             // false なら直接 setpoint を使う
};
```

### Controller 側の優先順位

ControlTask は `command_target`（Guidance 出力）と `command_setpoint`（パイロット直接指令）の両方を subscribe するが、**system.mode により優先順位を決める**：

| system.mode | 入力 | 用途 |
|------------|-----|------|
| `MANUAL` (ACRO / STABILIZE / ALT / POS) | `command_setpoint` | パイロット直接操縦 |
| `AUTO_MISSION` (将来) | `command_target` | ウェイポイント追従 |
| `AUTO_RTH` (将来) | `command_target`（home 座標で固定） | Return To Home |

これにより既存 `command_setpoint` のセマンティクスを壊さずに自律モードを追加できる。

### 学習者から見える形

L1 Topic API ユーザーが「自分の Navigator」「自分の Guidance」を書くときのコード例：

```cpp
// MyGuidance.cpp — 学習者のコード
void MyGuidanceTask(void* pvParameters) {
    while (true) {
        // 経路を読む
        sf::NavigationPath path;
        if (sf::nav_path.read(path)) {
            // 現在の位置を読む
            auto state = sf::estimate_state.latest();

            // 自分のロジックで次の target を計算
            sf::GuidanceTarget target = my_pure_pursuit(state, path);
            sf::command_target.publish(target);   // ← Topic に流すだけ
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // 10Hz
    }
}
```

Controller / Estimator / HW のことは何も触らずに、自分のロジックだけ書けば済む。これが 4 階層アクセスの「L1 だけで完結」の意味。

### 実装フェーズ

| Phase | 作業 |
|-------|------|
| M2 | 必要な Topic データ型 (`GuidanceTarget`, `NavigationPath`) を data_types.hpp に予約定義（実体生成は M4 以降） |
| M4 | `sf_guidance` コンポーネント新設、Pure Pursuit / Carrot Following など基本アルゴリズムの実装 |
| Phase 6 | `sf_navigator` コンポーネント新設、経路計画（A* / RRT などは選択肢として残す） |

実装が進んだ時点で本節を更新し、`@design` ステータスを `[OK]` に更新する。

---

<a id="english"></a>

## 1. Overview

This document defines the detailed design of vehicle_new, based on the architecture design (architecture.md).

## 2. Pub-Sub Framework

### Design Policy

- Lightweight custom design optimized for StampFly constraints
- Intra-MCU task communication only, no serialization
- All topics determined at compile time
- Internal buffer policy selected per data flow characteristics

### Buffer Policies

| Policy | Characteristics | Use Case |
|--------|----------------|----------|
| RingBuffer | Lock-free SPSC, full retention, ISR-safe | IMU→Estimation, All→Logger |
| Queue | FreeRTOS Queue, buffered | Low-rate sensors |
| Latest | Latest-value overwrite, shared memory | Estimate→Control |

### API

```cpp
topic.publish(data);                    // Producer
topic.subscribe(callback);             // Consumer (callback)
auto data = topic.latest();            // Consumer (polling)
```

## 3. State Transition Table

See Japanese section for complete onExit/onEnter callback table. Entries marked (reserved) will be filled during implementation and testing.

## 4. Control Interface Definition

```cpp
class IController {
public:
    virtual ~IController() = default;
    virtual ControlOutput compute(const StateEstimate& state,
                                   const CommandSetpoint& setpoint,
                                   float dt) = 0;
    virtual void reset() = 0;
    virtual void onModeChange(uint8_t new_mode) = 0;
};
```

> **レート内ループの帰還（angular_rate）:** `compute()` は最内ループ（角速度制御）の帰還量を `StateEstimate.angular_rate[3]`（機体 FRD・推定器でバイアス補正済み）から読む。推定器（ESKF 実装では `predict` 内の `gyro_raw − gyro_bias`）がこのフィールドを毎サイクル埋める。算法非依存：相補/Madgwick 等に差し替えても、角速度を出す推定器ならこのフィールド経由で同じレート制御が閉じる。
>
> *Rate inner-loop feedback: `compute()` reads the innermost (angular-rate) feedback from `StateEstimate.angular_rate[3]` (body-FRD, bias-corrected by the estimator). Any estimator that produces a body rate fills this field, so the rate loop closes regardless of the estimation algorithm.*

## 5. Estimation Interface Definition

```cpp
class IEstimator {
public:
    virtual ~IEstimator() = default;
    virtual void predict(const ImuData& imu, float dt) = 0;
    virtual void updateTof(const TofData& tof) = 0;
    virtual void updateFlow(const FlowData& flow) = 0;
    virtual void updateMag(const MagData& mag) = 0;
    virtual void updateBaro(const BaroData& baro) = 0;
    virtual StateEstimate getState() const = 0;
    virtual void reset() = 0;
    virtual void resetPositionVelocity() = 0;
};
```

### Sensor Observation Switch

Sensor observation ON/OFF is controlled via parameters and decided internally by the estimation component. Raw data always reaches the estimator and logger; the estimator skips observation updates when the parameter is false.

## 6. Parameter System

Single-macro definition: one line per parameter covers declaration, validation, callback, and NVS persistence.

```cpp
PARAM_FLOAT("rate.roll.kp", 1.365e-3f, 0.0f, 1.0f, on_pid_changed)
```

## 7. Directory Structure

HAL components are copied into vehicle_new (fully independent). See Japanese section for complete tree.

## 8. Memory Layout

### Partition Table

```
nvs,       data, nvs,     0x9000,   0x6000,    # 24KB
phy_init,  data, phy,     0xF000,   0x1000,    # 4KB
factory,   app,  factory, 0x10000,  0x300000,   # 3MB
storage,   data, spiffs,  0x310000, 0x200000,   # 2MB Blackbox
# Remaining 2MB unallocated (future use)
```

### Task Stack Summary

Total task stacks: ~92KB out of 512KB RAM. Remaining ~420KB for Pub-Sub buffers, ESKF matrices, heap, etc.
