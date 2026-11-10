# vehicle Detailed Design
# vehicle 詳細設計書

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

本文書はvehicleの詳細設計を定義する。アーキテクチャ設計書（architecture.md）に基づき、実装レベルの仕様を記述する。

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
| IDLE_GROUND → ARMED_GROUND | （リザーブ） | 全PIDリセット（以後 `Grounded` の間は姿勢/レート積分器を**保持**＝積分しない、注11）、ESKF**姿勢共分散の膨張**（注1）、ブザー(arm音) |
| ARMED_GROUND → TAKEOFF | （リザーブ） | 離着陸MGR: 離陸シーケンス開始、高度目標セット（ALT/POS は `takeoff_target_alt_`=0.5m、注4） |
| TAKEOFF → FLYING | 離着陸MGR: シーケンス終了 | ESKF位置/速度リセット（注2: クラスB, ImuTask）、~~バイアスフリーズ解除~~（注3で見送り） |
| サブモード切替（地上/FLYING） | 旧サブモードのコントローラリセット | 新サブモードの初期化（高度/位置キャプチャ＋ALT/POS 進入時は**スロットル再センターロック**（中央に戻すまでスロットル入力を無視）、注4）。地上では制御器の鉛直フェーズ判定（Grounded=推力ゼロ）により ALT/POS 選択でもモータは回らない |

### 3.1 モード調停表（規範 / Normative）

**この表を基準とし、コード（state_task のモード調停・StateManager::requestModeChange）は本表を写す。**
表に無い (状態, 事象) の組合せを実装してはならず、表の全セルが実装されていなければならない
（抜けのない設計 — 実機 LED バグ①と TAKEOFF 窓の適用漏れ②は本表の作成によって発見・修正された）。

事象の定義:
- **エッジ**: モードスイッチの位置変化（want ≠ prev_want）。拒否された場合 prev_want を更新せず**持続**（受理される状態に達した時点で適用）
- **不一致**: スイッチ位置と実モードの不一致（want ≠ mode、エッジなし）。接地リセット等で発生
- **API設定**: ApiCmd::Takeoff によるモード設定（同一周期で ARM まで進む）
- **接地リセット**: 空中状態→IDLE_GROUND 突入時の STABILIZE 強制（再飛行安全則）

| 状態 \ 事象 | エッジ | 不一致（レベル） | API設定 | 接地リセット | Flip 要求 |
|------------|--------|----------------|---------|------------|----------|
| INIT | 拒否（エッジ持続） | 無視 | 拒否 | — | 拒否（`error flip: not flying`） |
| IDLE_GROUND | **適用** | **適用（再適用・マーカーログ）** | **適用** | （突入時に作動し、直後に不一致則が再適用） | 拒否（`error flip: not flying`） |
| IDLE_HELD | 拒否（持続→設置時に適用） | 無視 | 拒否 | — | 拒否（`error flip: not flying`） |
| ARMED_GROUND | **適用** | 無視（エッジのみ） | **適用** | — | 拒否（`error flip: not flying`） |
| TAKEOFF | 拒否（持続→FLYING で適用） | 無視 | 拒否 | — | 拒否（`error flip: not flying`） |
| FLYING | **適用** | 無視（放置送信機が API を覆せない） | （誘導目標のみ） | — | **受理**（実行条件は制御器が判定） |
| FLIP | 拒否（持続 → FLYING で適用。TAKEOFF と同じ） | 無視 | 拒否 | — | 拒否（`error flip: busy`） |
| LANDING | 拒否（持続→接地後に適用） | 無視 | 拒否 | — | 拒否（`error flip: not flying`） |

検証: SILS `acro_crash_relevel`（①の固定）、`api_flight`（FLYING 不一致の無視＝API保護）、
`modeswitch`/`alt_flight`（FLYING エッジ適用）、`alt_auto_takeoff`（IDLE_GROUND API設定）。

**下向き ToF 出力停止の列（2026-11、注12。同日にさらに更新）:** 上の表の全セルに共通の追加条件。下向き ToF の出力停止の**エピソード**（新サンプルが来ない／飛行中に無効サンプルだけが続く／サンプルは戻ったが推定器がまだ ToF を取り込み直していない。注12）の間、`requestModeChange` は **ALT_HOLD / POS_HOLD への切り替えを、モードを受理する状態（IDLE_GROUND・ARMED_GROUND・FLYING）でも拒否**する（誰も保証できない高度で飛ぶことになるため）。ACRO / STABILIZE への切り替え（およびそれらの継続）は拒否しない。API `takeoff` は ApiTask が先に `error tof silent - check the sensor` で拒否し、ステート側（ARM 前判定3b）も ARM を拒否する。拒否の理由は `SystemMode.mode_block` に発行し、CLI `status` の `modeblock` 行に出し、ログ `Mode change to POS_HOLD refused: downward ToF is silent (no altitude)` を拒否した要求ごとに 1 回出す。**拒否したスイッチのエッジの扱いは状態で分かれる**（`decideToFModeRefusal`、`tof_silence_policy.hpp`）: **地上**（IDLE_GROUND・ARMED_GROUND）では**持続**する（上の「エッジ」の定義どおり prev_want を更新しない。`ModeBlock::TofSilent`）ので、エピソードが終わると要求どおりのモードに自動で切り替わる（IDLE_GROUND では不一致則が再適用する。設置中のモード変更が最も安全な時）。**飛行中**（FLYING）では**消費**する（StateTask が `isModeRequestConsumed()` を見て prev_want を更新する。`ModeBlock::TofSwitchConsumed`、ログ `IN FLIGHT the switch request is CONSUMED: flip the switch again after the ToF has recovered`）ので、ToF が復帰した瞬間にスイッチを入れたきりの機体が勝手に ALT_HOLD / POS_HOLD に入ることはなく、パイロットがスイッチを入れ直す（CLI `status` の `modeblock` 行にその旨を、着陸して地上に戻るかモード要求が受理されるまで出す）。検証: SILS `tof_silent_mode_refused`（地上）、`tof_silent_stabilize`・`tof_invalid_stabilize`（STABILIZE 飛行中）、`tof_recover_edge_consumed`・`tof_recover_reflip`（消費と入れ直し）。
| ARMED_GROUND → TAKEOFF（ALT/POS 選択時） | — | **ARM 自体がトリガ**（スプールドウェル後, 注4）→ ControllerCmd::Takeoff 発行 → 制御器が自動離陸（高度カスケードで目標 0.5m へ速度制限上昇＝**鉛直のみ自動**。姿勢はパイロット操縦可、ALT_HOLD は roll/pitch=スティック、POS は発進点保持） |
| TAKEOFF → FLYING（ALT/POS） | — | **制御器が目標高度 0.5m を捕捉**（`isTakeoffComplete`→`controller_status.takeoff_reached`。制御器自身の時間切れの備え 3 s を含む。無条件）→ StateTask が `decideTakeoff()`（`takeoff_policy.hpp`、純関数）で判断し、ControllerCmd::TakeoffComplete 発行 → 通常モード則を係合（**目標値 0.5m を保持**、行き過ぎ瞬時高度でなく。注4）。**例外:** 健全な ToF が「機体は床の上にいる」と積極的に示している間（有効な読みが `kTakeoffGroundBandM` = 5 cm 以下で、`kAutoTakeoffFloorRiseM` = 2 cm を超えて増えず、`kAutoTakeoffFloorHoldUs` = 1 s 続く。`TakeoffFloorWatch`）は、制御器が到達と言っても TAKEOFF に留める。この証拠は一度立ったらラッチし（ラッチ = 条件が一度成立したら、解除の条件が来るまで状態を保持する仕組み。`TakeoffFloorWatch::seen()` = `tof_health.floor_seen`）、完了を止め続ける（中止は「今」の証拠 `floor_stays` を 1 s 要するので、完了も同じだけ要るようにした。時間切れの後に帯の外へ外れた 1 つの読みで、床の上で FLYING にならない）。**ラッチは、機体が床を離れた証拠で解ける。** 中止が 1 s の証拠を要するのと対称で、(a) 健全な ToF の有効な読みが帯（`kTakeoffGroundBandM`、5 cm）を超える状態が `kAutoTakeoffFloorHoldUs`（1 s）続く（帯の中の読み・ToF の出力停止・`reset()` で区間は途切れ、無効サンプルは中立）、または (b) `kTakeoffAirborneBandM`（15 cm）を超える有効な読みが 1 つ。5〜15 cm の地面効果に留まる機体は、したがって 1 s 後に完了する（永久には保持しない）。セルの全体は下の「TAKEOFF の完了とラッチのセル」の表 |

| TAKEOFF → IDLE_GROUND（ALT/POS・API の自動離陸が浮けない） | — | **離陸の開始から `kAutoTakeoffNoLiftAbortUs`（5 s、候補値）経っても、健全な ToF が「床の上」の積極的な証拠（上の行）を示し続けている**と、StateTask（`updateTakeoff()` → `decideTakeoff()`）が DISARM して中止する（モータ停止。ログ `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`）。**証拠が「無い」ことは床の上の証拠ではない:** 無効サンプル・出力停止の ToF・帯より上の読み・増えている読み（ゆっくり浮く機体）では中止しない（加速度計に基づく以前の規則は、ゆっくり浮く機体を 0.11〜0.17 m で DISARM し 1.2〜1.6 m/s で落下させた。取り下げた）。ToF が床の上で無効を返す機体（実センサの床、SILS の既定）は、ToF 無効の規則（注12）が LANDING で終わらせる（床の上での ToF なしの LANDING は害が無い: 降下の推力は支持推力未満）。「ARMED_GROUND へ戻す」にしないのは、ALT/POS の ARMED_GROUND がスプールドウェルの後で自動離陸をまた始め、繰り返すため。API `takeoff` は離陸の待ちが中止を見て `error` を返し、機体はそのとき DISARM 済み。手動（ACRO / STABILIZE）の TAKEOFF には適用しない（パイロットが決める）。検証: SILS `weak_spool_alt_floor`（中止）、`weak_spool_alt_slow_lift`（中止しない）、ホスト試験 `test_takeoff_policy`。 |
| TAKEOFF → LANDING（通信途絶・ToF の規則・解けない床の証拠） | 通信途絶の猶予経過（下の FLYING → LANDING 行）、ToF の規則（注12）、ラッチした床の証拠が 5 s たっても解けない（下の「TAKEOFF の完了とラッチのセル」） | **TAKEOFF から LANDING に入る道は、この 3 つだけ**（`TakeoffAction::Land` は `StateManager::notifyTakeoffEndedByLanding()`）。**API `land` は TAKEOFF では LANDING にしない**: `StateManager::notifyLandingRequest()` は FLYING（と、高度モードの ARMED_GROUND の即時 DISARM）だけを受け付け、TAKEOFF では無視する。`cmdLand` は TAKEOFF を空中として受け付けるが、TAKEOFF を抜けるまで要求を保留してから実行する: `takeoff` の待機中に届いた `land` は ApiTask が後回しにして離陸の後に実行し、待機の外で届いた `land`（例: RC の ARM が始めた自動離陸の最中）は `cmdLand` が TAKEOFF を抜けるまで待つ。離陸が FLYING で終われば着陸を発行し、LANDING で終われば（通信途絶・ToF の規則など）その着陸の終わりを待って `ok` を返し、地上で終われば `error not flying` を返す（注13）。床の上で始めた LANDING は機体を浮かせうるので、TAKEOFF から LANDING に入る道を、通信途絶・ToF の規則・離陸の中止の判断だけにした（下記）。以後は FLYING → LANDING 行と同じ降下。**床にいる間に始まった LANDING の推力:** 特別な上限は置かない。離陸の推力（ホバー推力の見積もり + 補正）で浮かなかった機体は、見積もりから始まって降りる着陸の推力でも浮かない（SILS 1200 通りで確認、注8 (3)）。**パイロットの DISARM は TAKEOFF では LANDING にならず、どのモードでも即モータ停止**（注5）: 床の上で DISARM を着陸要求にすると、ESKF が地上に固定されたまま着陸の推力上限が重さを超えうる機体が浮き上がる。この案を SILS（ALT_HOLD の自動離陸の 50〜2000 ms 後の DISARM × ホバー推力の見積もり 0.9〜1.3 倍 × ToF の不感帯あり・なし × ノイズなし・n2 × 加速度計のオフセット 0・±0.4 m/s²、336 通り）で評価すると、LANDING が床の上（高さ 5 cm 以下）で始まった 144 通りのうち 38 通りで LANDING の後の上昇が 0.05 m を超え（最高 0.355 m）、20 通りで LANDING が高さ 0.07〜0.16 m で終わって空中でモータが止まった。システム自身の「浮けない離陸の中止」（上の行）も `requestDisarm()` によるモータ停止。**ToF が一度も有効な高さを読まない機体:** TAKEOFF の ToF 無効の規則（注12）が TAKEOFF を LANDING で終わらせ、そのとき ToF は出力停止と判定済みなので、降下は ToF なしの降下（推力は支持推力の 0.95 倍以下で上昇できない）で、接地は ToF なしの接地の判定（注12）が決める。検証: SILS `api_land_in_takeoff`（浮けない機体への `land`: 後回しになり、中止でモータが止まる）、`api_land_deferred_in_takeoff`（`land` は離陸の完了後に実行される）、`api_land_in_rc_takeoff`（RC が始めた自動離陸の最中の `land`: TAKEOFF を抜けてから実行され、`ok` を返す）、`api_land_tof_never_valid`（ToF が一度も有効でない上昇中の `land`）、`takeoff_disarm_stops`（DISARM）、`commloss_takeoff`（通信途絶）、`landing_low_start`（低い高さで始まる LANDING）、ホスト試験 `test_blind_landing`（床の上の推力の上限）・`test_takeoff_policy`（解けないラッチ）・`test_main`（`land_*`） |
| FLYING → LANDING | （リザーブ） | 制御器が `VerticalPhase::Landing` に切替（注6, INV-1）。鉛直チャネルが `landing_descent_rate_` で降下、姿勢はパイロット（リンク途絶時のみ水平, INV-2）。**ALT/POS でのパイロット DISARM**（注5）or 通信途絶の猶予経過（`kCommLossHoverUs` 3 s）で起動。**通信途絶の猶予が過ぎたら、TAKEOFF からも LANDING にする**（`decideCommLossLanding()`。以前は FLYING だけで、完了しない TAKEOFF の間に通信が途絶えると機体は古い指令のまま上昇し続けた。SILS `commloss_takeoff`）。FLIP 中は FLYING に戻るまで保留 |
| FLYING → ARMED_GROUND | 高度/位置コントローラリセット | ESKFホールド |
| FLYING → IDLE_GROUND | 高度/位置コントローラリセット | モーター停止、ESKFリセット、ブザー(disarm音)。**ACRO/STABILIZE のパイロット DISARM・衝突検知・緊急停止**（注5） |
| FLYING → FLIP | API `flip x` またはコントローラ FLIP ボタンの立ち上がり。制御器は FLYING 中に毎周期、実行条件 C2〜C8 の成立を `controller_status.flip_ready`（不成立理由は `flip_block_reason`）として publish する（検出、INV-3）。最初に判定するのはマスタースイッチ `flip.enable`（既定 0 = 無効、`FlipBlockReason::Disabled`、実機初回の宙返りが失敗した 2026-10-02 以降。FLIP ボタンも API も同じ `flip_ready` を読むため両方拒否される）。StateManager は FLYING かつ `flip_ready` のときだけ遷移し、それ以外は遷移しない（判断）。API は `flip_block_reason` を読んで `error flip: <理由>` を返す | `ControllerCmd::Flip{dir}`（開始高度・ヨー取り込み、レート PID 積分項リセット）、`EstimatorCmd::HoldAttitudeCorrection`、LED 橙。FLIP ボタンの拒否は必ずログ1行＋エラー音（`NotifyEvent::FlipRejected`）。一過性の理由 `NotSteady` だけは即拒否せず `flip.button_wait_ms`（既定 1000 ms、0 で即拒否）保持し、定常の窓が開けば開始する（下の「FLIP シーケンスの終了規則」）。CLI `status` の `flip` 行が現在の阻害理由を示す |
| FLIP → FLYING | 制御器: `flip_done`（完了 or 打ち切り後の回復完了、結果は `flip_result`）を publish → state_task が実行（`TakeoffComplete` と同型） | `ControllerCmd::FlipComplete`（位置目標取り直し、高度目標＝取り込み値、高度・位置積分項リセット）、`EstimatorCmd::ResumeAttitudeCorrection`（観測復帰・ToF 再取り込み） |
| FLIP → IDLE_GROUND | DISARM操作・`emergency`・IMPACT（既存の無条件停止と同じ） | モーター停止、ESKFリセット、ブザー(disarm音)。`EstimatorCmd::ResumeAttitudeCorrection` も発行（FlipComplete は発行しない） |
| LANDING → IDLE_GROUND | 離着陸MGR: シーケンス終了。**この LANDING が有効だった間に出た判定だけを数える**（注7） | モーター停止、ESKFリセット、~~バイアスフリーズ~~（注3で見送り）。接地検出＝本当の DISARM |
| ARMED_GROUND → IDLE_GROUND | （リザーブ）。**API `land`**: **高度モード（ALT_HOLD / POS_HOLD）の**、モータが床の上で回り始めたところ（自動離陸のスプール中）に届いた `land` は、降りる高さが無いので離陸の完了を待たずに即時 DISARM（`StateManager::notifyLandingRequest()` が `requestDisarm()` を呼ぶ。注13）。**手動モード（ACRO / STABILIZE）の ARMED_GROUND では従来どおり拒否する**（`error not flying`）: スロットルをパイロットが握り、ARMED_GROUND からでも浮ける（スロットル 0.499 で足りる）ので、床の上とは限らない。`ApiTask` が ARMED_GROUND を見た直後に状態が TAKEOFF へ移る競合では、要求は StateManager が無視する（TAKEOFF では受け付けない）ので、`ApiTask` は要求を発行した後に状態が IDLE_GROUND になったことを確かめてから `ok` を返す（最長 `kLandConfirmMs` = 0.5 s）。TAKEOFF に移っていたら `ok` を返さず、TAKEOFF を抜けるまで待ってから `land` を発行し直す（待機の中なら後回しの行にして待機を続け、離陸の後に実行する。待っていた `takeoff` は離陸の結果どおりに応答する）。この競合そのもの（窓は ApiTask が状態を読んでから StateTask が要求を処理するまでの数 ms）は、SILS の通常のシナリオでは再現できず未確認。要求の発行を省いて競合に負けた場合を模擬すると、`takeoff` は `ok`、続けて後回しの `land` が LANDING → IDLE_GROUND で `ok` を返すところまで確かめた。**既知の問題（直していない）:** 手動モードの ARMED_GROUND はスロットルだけで浮けるので、この状態にいることは床の上にいることを保証しない。検証: SILS `api_land_in_armed_ground`（高度モード）、`api_land_manual_armed_ground`（手動モードの拒否） | モーター停止、**ESKF全リセットのみ（注10。地上でプロペラが回り推定が振動で外れた可能性があるため、飛ばずに DISARM した場合も行う。再キャリブレーションはしない — 校正は水平前提で、自動経路は水平を判別できない）**、ブザー(disarm音) |
| ARMED_GROUND 滞在中: `system_status.attitude_mismatch`（遷移の入力、注10） | — | StateTask が DISARM を要求（`requestDisarm()`）→ 上行で再水平化。**空中状態（TAKEOFF/FLYING/FLIP/LANDING）では作用しない** |
| IDLE_GROUND 滞在中: `attitude_mismatch` の立ち上がりエッジ（遷移なしのイベント, 注10） | — | ESKF 全リセットのみ（自己回復。再キャリブレーションはしない。**IDLE_GROUND 滞在ごとに最大1回**、IDLE_GROUND を出ると再装填）。判定が通るまで ARM 前判定4 が ARM を拒否 |
| IDLE_GROUND 滞在中: ベンチ用モータテスト終了（遷移なしのイベント, 注9） | — | `MOTOR_TEST_SETTLE_US` 静穏後に ESKF 全リセット＋再キャリブレーション（置き直しと同じ処置） |
| ARMED_GROUND 滞在中: **下向き ToF 出力停止**（`alert: TOF_SILENT`、注12） | — | StateManager が DISARM（→ IDLE_GROUND）。モードを問わない。自動離陸（スプール後）が始まる前に捕まえ、高度を誰も見られないまま離陸しない。上の ARMED_GROUND → IDLE_GROUND 行の onEnter（ESKF Reset、disarm 音）が実行される |
| TAKEOFF 滞在中・モード ALT_HOLD / POS_HOLD: **下向き ToF 出力停止**（`alert: TOF_SILENT`、注12） | — | StateManager が LANDING へ遷移（**上昇を中止**。FLYING には到達しない）。`onEnter(LANDING)` が制御器の着陸降下を起動し、ToF が出力停止なので降下は**鉛直速度の閉ループ**（加速度計に基づく速度推定、注12）。ImuTask は固定されたままの接地フラグを信用せず、位置・速度の固定を解除する。2026-11 の事故（接地フラグが固定されたまま推力を積分して天井に衝突）の対策 |
| TAKEOFF 滞在中・モード ACRO / STABILIZE（手動離陸）: **下向き ToF 出力停止**（注12） | — | 強制着陸しない。警告（LED の黄高速点滅、約 5 s ごとの音、ログ `NO forced landing, fly on`）のみで、パイロットが操縦を続ける。パイロットが着陸（DISARM）した後は、出力停止が続く限り次の ARM を拒否（ARM 前判定3b）。手動の TAKEOFF → FLYING は固定されたままの接地フラグでなく、出力停止中は「ARM 中＝接地していない」として `system_status.airborne` が立つ。**床の上で FLYING 表示になりうるのは許容する:** 床がまだ支えている機体（モータが弱い）でも、ToF が床の上で無効を返すと、TAKEOFF の 2 s + 0.15 s の後に出力停止のエピソードが始まり、FLYING になる（`Ground contact unknown`、`Takeoff complete`）。スロットルはパイロットが握っており、STABILIZE は自分では着陸しない（以前は加速度計に基づく上昇量でこれを防ごうとしたが、ゆっくり浮く機体と床が支える機体を区別できず、取り下げた）。検証: SILS `weak_spool_stabilize`。 |
| FLYING 滞在中・モード ALT_HOLD / POS_HOLD: **下向き ToF 出力停止**（`alert: TOF_SILENT`、注12） | — | StateManager が LANDING へ遷移（降下は鉛直速度の閉ループ）。以後は FLYING → LANDING 行と同じ（姿勢は操縦者 or 水平化、INV-2） |
| FLYING 滞在中・モード ACRO / STABILIZE: **下向き ToF 出力停止**（注12） | — | 強制着陸しない。警告のみ（上の TAKEOFF・手動の行と同じ）。**パイロットから鉛直軸（スロットル）を取り上げない**。ARM 中の警告として本体 LED・MCU LED が黄高速点滅する。出力停止中は ALT_HOLD / POS_HOLD への切り替えを拒否（モード調停表の「ToF 出力停止」の列） |
| FLIP 滞在中: 下向き ToF 出力停止 | — | 何もしない（シーケンス途中の中断はしない）。FLIP → FLYING に戻った「その場で」`notifyFlipComplete()` が継続状態（レベル）を見て上の FLYING 行を適用する（ログ `ToF silence episode during the maneuver: decided now that FLIP is over`）。次に再発報されるアラート（最大約 1 s）は待たない: 待つと推定だけで ALT_HOLD / POS_HOLD が飛び続けた |
| LANDING 滞在中: 下向き ToF 出力停止（ToF なしの着陸） | — | 降下の推力は**常に 支持推力 × `kNoTofThrustCapRatio`（0.95）以下**（重さを下回るので、速度推定が何であれ機体は上昇できない。支持推力 = 機体を支えていた指令推力を、床を離れた後のホバー近傍のサンプルで加速度計により較正した値）で、加速度計に基づく速度推定を信用できる間（最後のリセットから `kVerticalVelocityTrustS` = 2 s）は支持推力のまわりの PI（目標 −0.4 m/s。ただし推力は上限を超えて上げられないので、速度を 0.4 m/s に保つ制御ではなく、上昇を止めることと降下の最初の加速にだけ効く）、それを過ぎたら上限の一定値。どちらの場合も実質は**推力の上限による一定の下向き加速度（約 0.05 g）の降下**で、降りるほど速くなる。エピソードが終わる＝推定器が ToF を取り込み直すまで続ける。サンプルが戻っただけで ToF 生存時の閉ループ着陸に戻さない）。ToF による着地判定（`system_status.landing`）は信用せず、**「衝撃のあとの静止」による接地**（`|加速度計の測定値|` が `kBlindTouchdownAccelMps2`（2 g）を連続 `kBlindTouchdownSamples`（2）サンプル超過した後、加速していない・指令推力が支持推力未満（この降下では常に真）・鉛直速度の推定が ±0.15 m/s 以内が 0.25 s 続く）、**または「床が機体を支えている」**（指令推力が支持推力の 0.7 倍未満で、ろ波した |加速度| が 1 g の ±0.1 g 以内が 0.5 s。空中で推力が重さ未満なら読みは 1 g を下回るので、床の上でしか成り立たない。衝撃は不要）で LANDING → IDLE_GROUND。状態が LANDING に入ってから `kBlindLandingTimeoutUs`（10 s、候補値）経ち、かつ ToF が連続して `kBlindLandingMinSilentUs`（5.5 s、候補値）出力停止していれば、StateManager が着陸完了を宣言する（→ IDLE_GROUND、`touchdown by time`。10 s は LANDING に入った時刻から数え、ToF の喪失と復帰の繰り返しで延びない。5.5 s は、ToF が動作したまま 10 s を超えた LANDING の後に止まったとき、空中で即座にモータを止めないための最低の降下時間で、測った最長の降下 4.7 s を 0.8 s の余裕で覆い、後半の低下が推力をゼロにする 8.33 s より手前）。出力停止中でも衝撃検出（IMPACT、3 G・2 サンプル）は有効で、接触の衝撃が 3 G を超えれば静止の判定より先にそれが働く（SILS の接触は 6〜9 g なので、通常はこちらで終わる。実機の本物の接地ログも最大 3.39 g）が、**ToF 出力停止中の LANDING での IMPACT は非常でなく接地（着陸完了）として扱う**（ログ `Touchdown while the ToF is silent (impact) → landing complete, not an emergency`、`StateManager::lastLandingEnd()` = `ToFSilentImpact`。墜落検出の閾値は変えない。GYRO_ANOMALY は非常のまま）。静止の判定が先に働くのは、接触の衝撃が 3 G 未満の柔らかい接地だけ |
| IDLE_GROUND 滞在中: 下向き ToF 出力停止 | — | 遷移なし。**ARM 前判定3b** が ARM を拒否する（`ArmBlock::TofSilent`、注12）。ALT_HOLD / POS_HOLD への切り替えも拒否（`ModeBlock::TofSilent`。エッジは持続し、エピソードが終わると自動で適用）。ACRO / STABILIZE は受理。**ARM はエピソードが終わる（サンプルが 1 s 以上届き、推定器が ToF を取り込み直した）まで、ACRO を含む全モードで拒否**。サンプルが「届いた」だけでは許可しない |
| IDLE_HELD 滞在中: 下向き ToF 出力停止 | — | 遷移なし（`decideToFSilenceAction` は None）。手持ち中は ARM 要求自体を受け付けない（`requestArm` は IDLE_GROUND のみ）ので 3b も作用しない。置き直して IDLE_GROUND に戻った時点で、上の IDLE_GROUND 行が適用される |
| INIT 滞在中: 下向き ToF 出力停止（起動直後はサンプル未到達で必ず出力停止） | — | 遷移なし（None）。INIT では ARM 要求も受け付けない。INIT → IDLE_GROUND の後は上の IDLE_GROUND 行が適用され、最初の ToF サンプルが来るまで ARM 前判定3b が ARM を拒否する（現状の動作の明文化で、動作の変更ではない） |

「リザーブ」は実装・テスト時に必要に応じて追加する。

#### TAKEOFF の完了とラッチのセル（規範、2026-11-10）

ALT_HOLD / POS_HOLD の自動離陸で、制御器が「到達」と言った後の `decideTakeoff()`（`takeoff_policy.hpp`、純関数。ホスト試験 `test_takeoff_policy` が全セルを検査する）。「今」の証拠 = `floor_stays`、ラッチ = `floor_seen`、「5 s」= `kAutoTakeoffNoLiftAbortUs`（TAKEOFF の開始からの時間。`SystemMode.state_since_us` との差）。

| ラッチ（`floor_seen`） | 今の証拠（`floor_stays`）・読みの状況 | TAKEOFF の開始から 5 s 未満 | 5 s 以上 |
|----------------------|--------------------------------------|---------------------------|----------|
| 立っていない | 証拠なし | 完了（制御器が到達と言ったとき。言わなければ保持） | 同じ |
| 立っていない | 帯の中の読みの後に、無効サンプルだけが保持時間（1 s）続く（実際の床は不感帯で無効を返す。`floor_stays` は時計で真になる） | 保持。**`floor_stays` が真になった時点でラッチが立つ**（`TakeoffFloorWatch` は新しいサンプルごとに、保持時間が過ぎていればラッチする。サンプルが無効でも）ので、次の帯の外の 1 つの読みが区間を終わらせても、完了しない | 中止（`floor_stays` が真のあいだ。ラッチは立っているので、以後は「立っている」の行に従う） |
| 立っている | 帯（5 cm）以下で増えない読みが 1 s 続いている（`floor_stays` 真） | 保持 | 中止（DISARM、モータ停止。`requestDisarm()`） |
| 立っている | 帯を超え 15 cm 以下の有効な読みの区間が進行中（`floor_leaving` 真。機体が床を離れつつある。1 s 続けばラッチが解けて完了する） | 保持 | 保持（ゆっくり浮く機体を、離れつつある途中で着陸させない。1 s 続けばラッチが解けて完了、帯の中へ戻れば下の行） |
| 立っている | 帯の中の読みの区間が 1 s に届かず、帯より上の区間も進行中でない（1 つの外れた読み、帯の縁をまたいで交互に来る読みを含む。`floor_stays` 偽、`floor_leaving` 偽） | 保持（完了も中止もしない） | **LANDING**（`TakeoffAction::Land`、`notifyTakeoffEndedByLanding()`。離陸の推力で浮かなかった機体は着陸の推力でも浮かない。注8 (3)） |
| 立っている | 帯を超え 15 cm 以下の有効な読みが 1 s 以上続く | **ラッチが解け、完了** | 同じ |
| 立っている | 15 cm を超える有効な読みが 1 つ | **ラッチが解け、完了**（意図した判断: 床の上の 1 回の誤反射でそうなって完了しても、その後は通常の高度制御で目標高度へ上がるだけで、浮けない機体の中止を妨げない。浮けない機体の読みは床の上のままで、床の証拠はその読みで立て直される） | 同じ |
| 立っている | 無効サンプルだけ（中立: 区間を終わらせも延ばしもしない。**時計は進む**ので、`floor_stays` は帯の中の読みから 1 s で真になる） | 保持（ToF 無効の規則〔注12〕が TAKEOFF の開始から約 2 s + 0.15 s で LANDING にする） | 直前の `floor_stays` に従う（真なら中止、偽なら LANDING）。ただし `floor_leaving` が真の間は保持（LANDING にしない。直前の読みが帯より上の区間なら、浮きつつある途中） |
| 立っている | ToF が出力停止（`floor_stays` は偽として扱う） | 保持（出力停止の規則〔注12〕が 0.3 s で LANDING にする） | LANDING（同じく注12 が先に働く） |
| 立っている | 帯の中の有効な読みが戻る（帯より上の区間は終わり、時計は最初から） | 保持 | 戻った区間が 1 s に達していれば中止、達していなければ LANDING |
| （手動 ACRO / STABILIZE） | — | ラッチを読まない。ToF による空中の判定だけで完了 | 同じ |
| （どの値でも） | TAKEOFF 中の API `land` | 受け付けない（`notifyLandingRequest()` は TAKEOFF を無視する。ApiTask は TAKEOFF を抜けるまで `land` を保留し、離陸が終わってから実行する: `takeoff` の待機中の `land` は後回しの行、待機の外の `land`〔例: RC が始めた自動離陸の最中〕は `cmdLand` が待つ。この表の判断は変わらない） | 同じ |


### シーケンス状態の既定行

TAKEOFF / LANDING / FLIP はいずれも「有限時間の専用シーケンス状態」であり、モード調停・異常時の扱いに共通の既定がある（[`architecture.md`](architecture.md) §4「シーケンス状態の設計規則」規則3）。共通部分を1行で定義し、各シーケンスは差分だけを書く。

| 項目 | 既定 |
|------|------|
| モードスイッチのエッジ | 保留（拒否、持続→通常状態に戻ってから適用） |
| API 誘導目標（move/rotate/rc 等） | 拒否 |
| DISARM / `emergency` / IMPACT | 即時（IDLE_GROUND へ、既存の無条件停止と同じ） |
| 接地リセット | 無し（空中シーケンスのため対象外） |
| 新規シーケンス要求（TAKEOFF/LANDING/FLIP） | 拒否（busy） |

**シーケンスごとの差分**

| シーケンス | GYRO_ANOMALY | LOW_BATTERY | リンク途絶 |
|-----------|--------------|-------------|-----------|
| TAKEOFF / LANDING | 既定どおり DISARM | 既定どおり（ブザー警告） | 既定どおり（R16 タイムアウトで LANDING 等へ） |
| FLIP | **無視**（記録のみ。状態 FLIP の間の異常角速度は墜落ではないと判断） | FLYING 復帰後に既存則 | FLYING 復帰後に既存則（R16 の単一判定） |

### FLIP シーケンスの終了規則と打ち切り（2026-10-02、実機初回失敗を受けて制定）

INV-2「シーケンスは有限時間で、終了は単一の判定で通常則に戻る」を FLIP の全経路に適用する。実機初回（`logs/flip_test1.sflog.zip`）は、ジャイロ打ち切りが φ = 79° で作動し Brake が回転を φ ≈ 207°（反転）で止めたが、Brake の出口が「φ ≥ 290° かつ |ω| ≤ 300 °/s」「φ ≥ 350°」の 2 つだけで時間切れが無く、Brake に留まったまま推力（0.5·max、反転中は下向き）で床へ衝突した。

| フェーズ | 出口（先に成立したもの） | 次 |
|---------|--------------------------|----|
| Boost | `flip.boost_ms` 経過 | Spin |
| Spin | 計画の減速開始点（指令が掃いた角 + ω_cmd²/(2·`flip.brake_ramp_rps2`) ≥ 2π。計測角が指令角の 0.5 倍未満なら回転が始まっていないとして Recover へ）（通常） / ジャイロ飽和（計測 \|ω\| > `flip.gyro_abort_dps` が 3 周期連続）/ `flip.spin_timeout_ms`（直接 Recover） | Brake / Brake / Recover |
| Brake | **\|ω\| ≤ `flip.handoff_rate_dps`（φ に関係なく）** / φ ≥ `flip.handoff_force_deg`（360°） / **`flip.brake_timeout_ms`** | Recover |
| Recover | **傾き ≤ `flip.recover_boost_tilt_deg` かつ vz ≥ 0** / `flip.recover_timeout_ms` | Done → FLYING |

- **レート指令は「計画 360° プロファイル」（2026-10-02、flip-maneuver-plan.md §5.7）。** 加速ランプ `flip.rate_ramp_rps2`（145 rad/s²）でピーク上限（`flip.rate_roll_dps` 1500 / `flip.rate_pitch_dps` 1400）まで上げ、平坦部、減速ランプ `flip.brake_ramp_rps2`（145 rad/s²）で下げる。減速開始は計測でなく指令の積分角で決めるので面積がちょうど 360° になり、アクチュエータの遅れは波形を時間方向にずらすだけで回転量を変えない。必要トルク I·α（ロール 1.3 mN·m / ピッチ 1.9 mN·m）は上限 7 mN·m よりはるかに低く、レートループは線形のまま（実機の失敗は、300/500 rad/s² の台形がループを上限に張り付かせ 1909 °/s まで行き過ぎたこと）。推力の切替角は計画上の減速開始角を使う。フィードフォワードは比較して採らなかった（§5.7）。レート PID は通常飛行のままで、宙返り中に変わるのはトルク上限（`flip.spin_torque_limit_nm`）だけ（INV-1）。
- **Brake は必ず終了する。** 旧い「φ ≥ 290° 条件」（ピッチ回転でオイラー角が 270° 手前まで 180° 側の枝にいるための待ち）は廃止（`flip.handoff_min_deg` 削除）。Recover の姿勢誤差を、オイラー角でなくクォータニオン由来の傾き誤差（`FlipSequencer::levelError`）にしたため、どの角度でも姿勢ループへ渡せる。
- **Recover は任意の姿勢（反転を含む）から水平に戻す。** 既存の姿勢 PID（INV-1）に、重力方向の最短回転ベクトル（小さな傾きでオイラー roll/pitch と一致、180° で折り返さない）を測定値として与える。出力上限は FLIP 中だけ `flip.recover_rate_limit_dps`（既定 600 °/s）に上げる（通常の 3 rad/s = 172 °/s では 155° の立て直しに約 0.9 s かかる）。ヨー保持は傾き ≤ `recover_boost_tilt_deg` のときだけ（傾いた機体のオイラーヨーは無意味）。
- **Recover の集合推力**は傾きで決める: ほぼ水平は増強、`flip.angle_a_deg`（60°）までは `thrust_spin_hi`（差動トルク余裕）、それを超えたら上向き成分 cos(tilt) に比例して縮め、90° 以上（反転）は `flip.thrust_lo_n`。反転中の推力は下向きのため。姿勢の P 則は遅く（kp = 5 /s）トルクをほとんど要さないので、低推力でも立て直しは遅くならない（SILS `flip_abort_recover` の試行: 反転 168° から 0.2 N でも 0.03 N でも同じ傾き履歴、0.03 N の方が落下が遅い）。
- **ジャイロ飽和の打ち切りは「指令に対する行き過ぎ」ではなく IMU 保護**: BMI270 は ±2000 °/s。上限は 1950 °/s（97.5 %）、3 周期（7.5 ms）連続で超えたときだけ作動する。実機の通常オーバーシュート（指令 1500 に対し 1909 °/s）では作動しない。作動時は Brake（指令→0）: 真のレートがセンサの範囲外で積算 φ と制動角の前提が崩れるため、検証できる動作（ゼロ指令）へ移る。継続（指令を下げてスピンを続ける）は採らない。
- **打ち切り理由は最初の原因を保持**する（`FlipResult`: AbortedSpinTimeout / AbortedGyroLimit / AbortedRecoverTimeout / AbortedBrakeTimeout）。後続の Recover タイムアウトが先の原因を隠さない。
- **救えない領域:** 約 2 m 未満で反転した状態へ打ち切られ自由落下に入ると、170° からの立て直し（約 0.4 s）より先に床に着く（1.3 m からの落下 約 0.5 s）。構造的な安全は「Brake を必ず終わらせる」ことで担保し、この領域は開始高度（`flip.min_height_m`）と、ジャイロ打ち切りを作動させない余裕（スピンのオーバーシュート低減、後続の数値裏付け付き調整）で避ける。

検証: ホスト単体 `test/test_flip_sequencer.cpp`、SILS `flip_abort_recover`（打ち切り→復帰）、`flip_brake_timeout`（Brake タイムアウト経路）、`rc_flip_latch`（ボタン保持）、`flip_disabled`（拒否ログ）。

### 状態追加手順（チェックリスト）

新しい `FlightState` を追加するときの標準手順。FLIP の実装作業（Phase 2）はこの手順そのもの。

| # | 手順 |
|---|------|
| 1 | `flight_state.hpp` に enum 値を追加 |
| 2 | 規範表（§3.1、既定行 + 差分）を先に更新する（INV-4） |
| 3 | StateManager に遷移 + onEnter/onExit を実装 |
| 4 | state_task に遷移実行ロジックを追加 |
| 5 | `ControllerCmd` / `IController` にフックを追加 |
| 6 | `controller_status` に完了フラグ・結果を追加 |
| 7 | failsafe の判断（`StateManager::handleAlert()`）に当該状態の扱いを追加 |
| 8 | notify（LED）/ telemetry に状態表示を追加 |
| 9 | API 側に待ち条件を追加 |
| 10 | SILS シナリオ + メトリクスを追加 |
| 11 | `@design` タグを更新 |

**根拠（対象範囲の実測）:** TAKEOFF/LANDING の特別扱いは 2026-09-16 時点で firmware 全体の 16 箇所・4 ファイル（StateManager 8、state_task 5、離着陸 MGR 2、ImuTask 1）に集中している（`docs/plans/flip-maneuver-plan.md` §4.6 実測）。状態として追加する限り拡張は局所的、という根拠になっている。

**注1（ARM時の ESKF 処理 — SILS 掃引で確定）:** 当初は「ARM時 ESKF 全リセット」（地上の共分散収束は飛行を代表しない、という根拠）だったが、SILS のリセットタイミング掃引（8方策×飛行スイート）で、全リセット — および位置/速度/バイアスの共分散の膨張 — は離陸過渡を不安定化すると判明した。再膨張した共分散がスラスト汚染された加速度計を過信し、POS_HOLD 姿勢が発散する（pos_roll/pitch/flight 墜落）。飛行スイート全PASS は2方策のみ＝「何もしない」と「**姿勢の共分散だけ膨張**」。後者を採用：設計意図（ARM で姿勢の自信をリセット）を満たしつつ、姿勢は離陸前に地上で重力から再収束するため安定。実装は `EstimatorCmd::InflateCov(CovScope::Attitude)`（推定値 x は保持し姿勢共分散のみ初期値へ）。

**注2（位置/速度リセット = クラスB）:** 「ESKF 位置/速度リセット」はタイミング命の ToF 同期鉛直ハンドオフ（クラスB, `ImuTask::applyVerticalGroundHandoff`）。状態機械の onEnter 経由（~20ms 遅れ）に移すと α-βトラッカ初期化が遅れ POS_HOLD 姿勢が劣化する（[`architecture.md`](architecture.md) §4 クラスA/B 分類）。地上ゼロは飛行を代表しないが、その手当ては空中エッジで ImuTask が正確に行う。

**注3（バイアスフリーズ/解除 = 見送り）:** ESKF の凍結機構（`active_mask`＋`enforceCovarianceConstraints`）は「センサ恒久不在」用の隔離（凍結状態の共分散を毎周期 init 値へ戻す）で、地上↔飛行でトグルする用途とは非互換。トグルすると解除時に巨大な共分散が復活し、注1と同じ離陸発散を起こす（SILS で実証）。よってバイアス推定は飛行中も常時アクティブとし、フリーズ/解除は配線しない。地上振動からの校正バイアス保護が必要になった場合は、共分散を init に戻さない「ソフト凍結」を別途設計する（将来課題）。

**注4（ALT/POS 離陸・モード進入の再設計 — 2026-06-14, ユーザー確定）:** 旧仕様「スロットル>0.5 で離陸・ToF 空中検知 0.15m で高度保持」を、パイロットがスティックを戻すタイミングを計れない・0.15m は地面効果で物理的に不安定、という理由で再設計した。確定事項:
- **① ARM 起動離陸:** ALT_HOLD/POS_HOLD では **ARM 自体が離陸トリガ**（スロットル入力不要）。StateTask が ARMED_GROUND 突入後、短いスプール/整定ドウェル（`config::ARMED_GROUND_SPOOL_US`=0.3s, モータはゼロのまま=暴発防止＋ARM 時の姿勢共分散膨張が重力から再収束）を待ってから `notifyTakeoff()`。**ACRO/STABILIZE は従来どおり手動スロットル離陸**（生スロットル＝推力、自動離陸なし。`config::TAKEOFF_THROTTLE_THRESH`）。
- **② 目標高度 0.5m・0.15m の役割分離:** TakeoffClimb は高度カスケードで目標 `takeoff_target_alt_`=0.5m（PidController 定数、地面効果回避）へ速度制限（`takeoff_climb_rate_`=0.3m/s）つき上昇する。Airborne 進入後も `alt_setpoint_`=0.5m を保持するので**目標値で捕捉**される（運動量による行き過ぎ瞬時高度でなく、決定②）。TAKEOFF→FLYING は制御器の `isTakeoffComplete()`→`controller_status.takeoff_reached` で成立するが、その判定は**ロバストな「片側到達」＋タイムアウト・バックストップ**: 高度が目標近傍まで上昇（`altitude >= target - kTakeoffCaptureBandM`、上昇途中で成立）を `kTakeoffSettleCycles` 持続 OR `kTakeoffMaxCycles`(3s) 経過で完了。**旧来の両側バンド+低速整定は実機の定常偏差/速度ノイズで成立せず、機体が TakeoffClimb に留まりロール/ピッチが0固定される実機バグ（2026-06-14）を起こした**ため是正（SILS `alt_arm_rollpitch` でガード）。**ToF 空中検知 0.15m は ESKF 鉛直ハンドオフ専用**（注2, ImuTask）に役割分離し、ALT/POS の離陸完了判定には用いない（ACRO/STABILIZE 手動離陸の TAKEOFF→FLYING のみ ToF airborne で判定）。
- **④ API takeoff 統一:** ApiTask の takeoff も同じ ARM 起動自動離陸経路を通り（StateTask が requestArm 後、上記 ② で離陸）、離陸後の高度をそのまま誘導で保持＝目標 0.5m を継承（旧 0.8m ハードコードを廃止）。再センターロックは手動 RC 限定（API はコマンド操作）。
- **ALT_HOLD/POS_HOLD スロットルは対称（バネ復帰式, ユーザー確定）:** スロットルはバネ中央復帰で、ALT/POS では `CommandSetpoint.throttle_axis = (raw-2048)/2048 ∈ [-1,+1]` を使う。**中央 raw 2048 = 0 = 現在高度ホールド**、上(>2048)=上昇（`altitude.climb_rate`）、下(<2048)=降下（`altitude.descent_rate`、**上昇/降下は別パラメータ**）。スティック上下で目標高度を増減する。これは旧 vehicle `altitude_controller` の `stickToClimbRate`（中央2048=hold）と同型＝飛行実績あり。**STABILIZE/ACRO は別** — `throttle [0,1]`（中央2048=推力0=OFF・上半分のみ・離すと降下）。※ vehicle は当初 ALT_HOLD で STABILIZE 用の `[0,1]` を `(throttle-0.5)` で流用し中立を raw 3072 に誤配置していた（バネ静止 2048 で降下＝実機で操縦不能になる潜在バグ）。本対称化で是正。詳細は [`alt_hold_takeoff_findings.md`](alt_hold_takeoff_findings.md)。
- **スロットル再センターロック（`throttle_recentered_`）:** 離陸後（Case A）・飛行中の ALT/POS 進入（Case B, onModeChange）でロックを掛け、スロットルが一度**中央（バネ静止 raw 2048, ±`stick_deadzone_`）に戻って初めて解除し**高度操作を有効化する（暴発・高度ジャンプ防止）。バネ式は離せば中央に戻り解除（旧 vehicle stick-lock と同型）。タイムアウトなし。誘導/API は高度を歩く設定点で動かしスロットル経路を使わないためロック対象外。
- **⑥ ARM 後モードロックは見送り:** 飛行中モード変更は従来どおり可能（§3.1 モード調停表は現状維持）。コントローラ↔機体の双方向モード同期の改修が前提のため将来再検討。

**注5（DISARM のモード依存ルーティング = 自動着陸 — 2026-06-14, ユーザー確定）:** パイロットの DISARM 操作は機体状態・モードで意味が変わる。
- **ALT_HOLD/POS_HOLD で FLYING 中の DISARM → 自動着陸（FLYING → LANDING）:** 高度制御モードは自力で着陸できるため、空中でモータを切らず**緩降下**（`computeLanding`, `landing_descent_rate_`=0.3m/s）で接地し、**接地検出で初めて本当の DISARM**（LANDING → IDLE_GROUND）。空中での即カットは機体を落とすだけ、というユーザー要望。`StateManager::requestDisarm()` が `state_==FLYING && mode_>=ALT_HOLD` を判定して LANDING へルーティング。
- **ACRO/STABILIZE・地上での DISARM → 即カット（→ IDLE_GROUND）:** 手動推力モードは自動着陸を持たない。地上（ARMED_GROUND）も常に即カット。
- **TAKEOFF 中の DISARM → 即カット（どのモードでも）:** 自動着陸になるのは FLYING の間だけ。床の上（ESKF がまだ地上に固定され、床がまだ機体を支えている間）で DISARM を着陸要求にすると、着陸の推力上限が重さを超えうる機体が浮き上がり、空中でモータが止まった（上の TAKEOFF → LANDING 行の評価）。TAKEOFF から LANDING に入るのは、通信途絶・ToF の規則・解けない床の証拠で、パイロットの DISARM でも API `land` でもない。システム自身の「浮けない離陸の中止」も `requestDisarm()` による同じモータ停止。
- **緊急停止（`requestEmergencyStop()`）:** 任意の armed 状態から無条件で即カット。**API `emergency` verb** と、**自動着陸中の再 DISARM（中断）**に用いる。`requestDisarm()` を LANDING 中に再度押すと `state_!=FLYING` ゆえ即カット経路に落ちる＝2回押しが中断になる。
- **検証:** SILS `alt_disarm_land`（ALT_HOLD で ARM→自動離陸→**DISARM→自動着陸の緩降下→接地→DISARM** の全鎖。降下中 duty>0.5＝モータ稼働、DISARM 0.4s 後も alt>0.2＝自由落下でない、終端 alt<0.05＝着地を assert）。既存の飛行スイート（`alt_flight`/`pos_flight` 等）は DISARM 直前にモードスイッチを落として STABILIZE へ復帰するため即カット経路を通り、本変更の影響を受けない。

**注6（着陸則の統一＝Landing を VerticalPhase 化・操縦可能化 — 2026-06-14, リファクタA, INV-1/INV-2）:** 当初 §注5 で「将来課題」とした「自動着陸の降下中も姿勢をパイロットに」を、設計不変条件（[`architecture.md`](architecture.md) INV-1/INV-2）に基づき本実装で達成した。**背景＝場当たりパッチの是正:** §② の「鉛直のみ自動・姿勢は常にパイロット」原則を離陸（TakeoffClimb）には適用したのに、着陸 `computeLanding()` は古い前提「フェイルセーフ専用・スティック無視」のまま**並列の制御経路**として残り、後付けのパイロット起動着陸でこの前提が崩れ「着陸中ロール/ピッチが効かない」実機バグを生んだ。
- **修正:** `computeLanding()` を**削除**し、Landing を `VerticalPhase`（Grounded/TakeoffClimb/Airborne/**Landing**）の一フェーズとして `compute()` の**単一姿勢＋レートパイプラインに統合**（INV-1）。フェーズが変えるのは鉛直チャネル（`landing_descent_rate_` で降下, モード非依存＝ACRO/STABILIZE からの通信途絶着陸にも対応）と接地条件のみ。姿勢は他フェーズと同じ1本道。
- **操縦/水平の作り分け（INV-2, 単一判定）:** Landing 中、**リンク生存中はパイロットが roll/pitch/yaw を保つ**（パイロット指令の着陸は操縦可）。**リンク途絶（通信途絶フェイルセーフ＝パイロット不在）なら単一の水平判定で roll/pitch/yaw を 0 に強制**。生存判定は設定点の新鮮さ `(state.timestamp − setpoint.timestamp) < kLandingLinkStaleUs(500ms, R16)` で行い、**StateManager からフラグを配線しない**（新鮮さだけでパイロット在否が分かり、降下中にリンクが切れても自動で水平化する）。
- **検証:** 新 SILS 2 本＝`alt_disarm_land_steer`（リンク生存のパイロット着陸: 降下中にロール保持→`tilt_max=11°`＝操縦可。旧 computeLanding なら≈0 で FAIL）/`commloss_land_level`（フェイルセーフ着陸: 猶予中 FLYING は古いロールで `tilt 11.6°` だが LANDING 突入後は水平判定で `tilt 3.6°` に水平化＝INV-2 の敵対ガード）。再確認試験（変更で既存の動作が壊れていないかを自動で確かめる試験）SILS 29/29・host 25/25・実機ビルド OK。

**注7（地面効果フロート対策＝降下停滞による接地検出 — 2026-06-14, リファクタB, INV-3）:** 実機で「ある程度の高度で手動 DISARM しないと着地しない／地面付近は予想より小さい Duty で浮く」現象（パイロット報告）。真因＝**地面効果**（接地近傍でローター揚力が増し、機体が低推力で高度を保つ）で一定速度降下が 5cm 地上閾値の上で停滞し、**ToF のみ（<5cm 必須）の旧着陸検出器が作動しない**。
- **修正（検出層 `sf_takeoff_landing`, INV-3）:** 接地検出に第2経路を追加。**(1) 確実な接地**＝`on_ground_`(ToF<5cm)＋静止を `landing_hold_ms`(1s)。**(2) 降下停滞**＝**着陸降下が指令中**(`in_landing_descent`=FlightState::LANDING, `armed` と同じ状態由来の入力)＋ToF が `near_ground_tof_m`(12cm)以内＋鉛直速度停滞(<`landing_vel_mps`)を `stall_hold_ms`(0.6s)。「降下しようとしているのに地面に止められている＝地面が支えている」をキネマティクスで判定し、**脆い推力閾値を使わない**（低ホバーは降下指令でないので判定条件を満たさず誤検出しない）。検出は検出層、遷移判断は StateTask（検出と判断の分離）。
- **第 3 の経路: 不感帯での接地（2026-11-09、`TakeoffLandingMgr::deadZoneTouchdown()`、INV-3）:** 下向き ToF は約 0.031 m より近いと「目標なし」（無効）を返す（実機ログ 3 本で床の上は無効、有効な距離の最小は 0.031 m）ので、柔らかい接地（3 G 未満）では経路 (1) の `on_ground_`（5 cm 未満の有効な読みが 2 回）も (2) の降下停滞も成り立たない。**(3) 不感帯**＝着陸降下が指令中（`in_landing_descent`）＋近傍帯（`dead_zone_band_m` = 0.10 m、`kLandingDeadZoneBandM`）以下の有効な読みの後に、無効サンプルだけが `dead_zone_hold_ms`（0.2 s、`kLandingDeadZoneHoldMs`）続く。速度の条件は置かない（その高さでは機体自身の速度推定を信用できない）。帯の中の有効な読みで数え直し、帯より上の有効な読み・着陸降下の終了・DISARM・`markAirborne()` で取り消す。帯は `kTofInvalidLandingMinHeightM`（ToF 健全性トラッカーが LANDING の無効サンプルを数え始める高さ）と同じ値: それより下では無効サンプルが正常で、上では異常。3 つの経路はどれも検出層の事実（レベルフラグ）で、遷移の判断は StateTask。根拠と実機ログの再生は下の「未解決」の項目 (1)。
- **LANDING の突入で、以前から残った判定を数えない（2026-11-10、`TakeoffLandingMgr::noteLandingDescent()`・`SystemStatus.verdict_state_since_us`、INV-3）:** 接地フラグ（`on_ground_`）は 0.05〜0.15 m のヒステリシス（ToF が 5 cm 未満で立ち、15 cm 超で下りる）で値を保つので、低いホバーから LANDING に入ると、以前の接地の判定が残っている。これを数えると、LANDING の最初の周期に、床の約 0.1 m 上でモータが止まる。(1) **検出層:** ImuTask は毎周期 `noteLandingDescent(in_landing_descent)` を呼び、LANDING の立ち上がりで着陸のレベルフラグと、経路 (1)（確実な接地）・(2)（降下停滞）の保持タイマーを消して、判定を新たに始める。不感帯の経路 (3) は LANDING の外では始まらない（`trackDeadZone()` が着陸降下の指令中だけ数える）ので、LANDING の前の読みを持ち込まない。(2) **判断層の判定条件:** ImuTask は判定を作った周期に読んでいた `SystemMode.state_since_us` を `SystemStatus.verdict_state_since_us` に刻んで発行し、StateTask は、その刻印が「現在の」`SystemMode.state_since_us` と**等しい**ときだけ `landing` を数える（`landingVerdictCounts()`）。前の状態が有効だった間に作られた判定は、時刻印の大小に関わらず一致しないので、LANDING に入った周期に隙間は残らない（「status の時刻が LANDING の開始より後」という大小の比較では、StateTask が遷移させた後も ImuTask が前の状態を読んでいた 1 周期に、前の状態の判定が通りえる）。この判定条件は純関数 `landingVerdictCounts()`（`takeoff_policy.hpp`）。検証: ホスト試験 `test_main` の `land_notice_*` 5 本（LANDING の立ち上がりで持ち越しの判定が消える。その後の新しいサンプルで接地と判定できる。毎周期の通知は立ち上がりではなく、LANDING の中の判定を消さない。LANDING の外では何も変えない。不感帯の証拠も LANDING の立ち上がりで消える）と、`test_takeoff_policy` の `a_touchdown_verdict_counts_only_with_the_stamp_of_the_present_landing`（刻印が一致しない判定は数えない）。判定のやり直しを外すと、`land_notice_*` の 2 本が不合格になる。SILS `landing_low_start` は、低い高さ（0.117 m）で始まる LANDING の動作の確認（その LANDING の中で出た判定で止まる）で、**判定のやり直しの有無を区別しない**: 以前から残った判定を作る入力（接地フラグが立ち、機体が静止し、0.05〜0.15 m の FLYING または TAKEOFF）は、SILS では見つからなかった。
- **床を離れたことの無い機体の LANDING でも判定は新たにやり直す（評価して採らなかった案）:** やり直しは「ToF が接地の帯（5 cm）以上の有効な読みを見たことがある機体」に限る案を評価した（床の上で正しい接地の判定まで捨てて、LANDING が長引く場合を短くするため）。弱い機体（モータの健全度 0.84〜0.91）の自動離陸を ARM のまま続けて ToF の不感帯あり・なしでそろえた 64 通りのうち、LANDING が床の上で 6.38 s（ToF 無効の規則の後の ToF なしの降下が、床が支えている接地を後半の推力低下の後に検出するまでの時間）かかる 8 通り（不感帯あり・ノイズなし）は、この案を入れても 8 通りとも 6.38 s で変わらなかった（案を入れなくても同じ 6.38 s。残る健全度 0.91 の 1 通りは衝撃のあとの静止の検出で 0.26 s）: その LANDING は ToF 出力停止のエピソードで始まり、`markAirborne()` が LANDING の前に接地フラグを「空中」に倒すため、持ち越される判定は既に無い（証拠が無いことを床にいる証拠にしない）。このため採らない。
- **SILS 地面効果モデル（プラント, Model fidelity）:** `lift ×= 1 + ge_gain·exp(−z/ge_height)`。**既定 OFF**（クリーン経路バイト一致, 既存 29 シナリオ不変）で、`sf sils scenario --ground-effect [gain]`（env `SILS_EMU_GROUND_EFFECT`）でオプトイン（ノイズノブと同流儀）。GE ON で実機同様の「低 Duty フロート」（duty 0.75→0.69, alt~0.05m）を再現。**ただし SILS は真値速度ゆえ速度ループが <5cm まで押し切り cond1 で着地する**（実機の「5cm 上で無限フロート」は近地面の速度推定劣化が原因で、真値 SILS では再現されない）。
- **検証:** **host ユニットテスト 4 本**（`land_firm_ground`/`land_stalled_descent_ground_effect`＝8cm フロートを cond2 が捕捉/`land_low_hover_not_landing`＝低ホバー誤検出なし/`land_disarm_clears`）で cond2 ロジックを直接検証（モッククロック）。SILS では GE ON で着地が `Impact/anomaly` でなく `Landing detected` 経由のクリーン接地になることを確認。再確認試験 SILS 29/29（GE off）・host 29/29・実機ビルド OK。検出器はファーム側で常時有効ゆえ**実機（GE 常在）では常に効く**。

**注8（近地面の着地アシスト＝粘り解消の推力ランプダウン — 2026-06-14, A+B 実機フィードバック反映, INV-1）:** 実機で離着陸が成功したが**「最後の地面効果あたりでなかなか着地せず時間がかかる」**との報告。原因＝注7 の通り定速降下(0.3m/s)が地面効果と釣り合い、検出器(注7)は鉛直速度停滞(≈静止)を待つため、機体が地面効果で這うように降りている間は判定されず粘る。
- **A. 近地面で推力上限ランプダウン（制御器 Landing ブランチ, INV-1=鉛直チャネルのみ整形・姿勢不変）:** 高度推定が `kLandingSettleAltM`(0.15m)未満で、推力の**上限**を hover から `kLandingSettleThrustFrac·hover`(0.70)へ `kLandingSettleRampS`(1.0s)かけて絞る（`landing_settle_t_` で経過時間積算、onLanding/reset でクリア）。速度ループは上限内で動くので**通常降下は上限が効く前に穏やかに接地**、まだ浮いていれば**下がる上限が地面効果揚力を下回り確実に沈む**（有界の粘り・推力閾値の当て推量なし・床は hover 割合で自由落下でない）。
- **B. 検出閾値の緩和（検出層）:** `stall_hold_ms` 600→**400**, `landing_hold_ms` 1000→**700**, `near_ground_tof_m` 0.12→**0.15m**。A で機体が速く静止に達するので接地宣言を早める。
- **検証（SILS, GE ON）:** 着地が `Disarm ~16.0s→~15.0s`（約1秒短縮）、以前の「0.064m へ浮き上がる bounce」が消え一直線に沈む。接地鉛直速度 `vz=-0.42m/s`＝穏やか（落下でない）、duty も接地まで保持。再確認試験 SILS 30/30（GE off）・host 29/29・実機ビルド OK。**実機での粘り改善は GE が常在する実機で本領（SILS は真値速度ゆえ元々あまり浮かない）**。
- **LANDING は上昇しない（2026-11-10、`PidController::landingBaseThrust()`・`brakeRiseInLanding()`、INV-1）:** ToF が動作している間の閉ループ着陸は、速度ループが hover 推力の「見積もり」から始まり、積分が遅いので、見積もりが高い（`hover.thrust_corr` の変更の後の古い見積もり、電圧が下がった電池、離陸の上昇中に始まる LANDING）と、推力が数秒間重さを上回り、LANDING が始まった後も機体が上昇する。(1) **基準の上限:** 速度ループの基準推力は、見積もりと、支持推力の基準（`supportReferenceThrust()`: 較正済みなら加速度計で較正した支持推力、較正前は空中の全サンプルでの早い推定、それも無ければ低くした見積もり）の `kLandingBaseOverSupportMax`（1.08）倍の小さいほう。較正の前にも効く。(2) **上昇ブレーキ:** 機体が上昇している間（上昇率 > 0）、推力を支持推力の基準の割合以下に制限する。割合は上昇率 0 で 1 で、`kLandingRiseBrakeRampMps`（0.2 m/s）で直線的に `kLandingRiseBrakeFrac`（0.5）まで下がる（雑音の周りの入り切りを避けるランプ）。**上昇率は ESKF の値だけで、加速度積分は使わない**: ToF なしの長い区間の後で古くなった積分は、上昇していない機体にブレーキをかけて推力を半分にする。ブレーキを呼ぶのは ToF が動作している閉ループ着陸だけで、ESKF の鉛直状態を信用できない LANDING（ToF 出力停止）は ToF なしの降下（注12。推力は支持推力の 0.95 倍以下で上昇できない）が扱い、地上では鉛直状態が 0 に固定される（上昇なし、ブレーキなし）。普通の着陸（`landing_dead_zone`）は変化しない。
(3) **床にいる間に始まった LANDING には、推力の上限を置かない（評価して採らなかった案）:** 支持推力の窓が機体の離床をまだ見ていない（空中のサンプルが無い）LANDING の推力を、ホバー推力の見積もりの 0.5 倍に抑える案を評価した。TAKEOFF 中の API `land`（TAKEOFF から 50〜150 ms 後）で LANDING が始まる場合には、離陸の上昇が残るために、上限なしで機体が浮いた（72 本のうち 12 本で LANDING の後の上昇が 0.05 m を超え、最高 0.236 m）。しかし API `land` は TAKEOFF では保留する（注12 の (4)）ので、TAKEOFF から床の近くで LANDING に入る道は、通信途絶・ToF の規則・離陸の中止の判断（`TakeoffAction::Land`）だけになった。このうち通信途絶の入り口で、上限なしの LANDING が機体を浮かせるかを SILS で測った（ToF の規則と離陸の中止の判断の入り口では、この評価をしていない）。SILS（弱い機体の自動離陸: ALT_HOLD の ARM 起動、および手動 STABILIZE のスロットル 3100 に ARM の 0.6 / 1.5 / 2.5 s 後の通信途絶。モータの健全度 0.80〜0.95 × `hover.thrust_corr` 0.9 / 1.0 / 1.2 / 1.3 × ToF の不感帯あり・なし × 加速度計のオフセット −0.4 / 0 / +0.4 m/s² × ノイズ n2、960 通り）では、TAKEOFF から高さ 0.15 m 未満で LANDING に入ったのは 90 通り（健全度 0.80 と 0.85 だけ）で、LANDING の後の上昇は、推力の上限が無くても 0.5 倍の上限があっても最大 0.011 m で、952 通りの結果が上限の有無で一致した（空中のモータ停止・終わらない LANDING・床からの再離陸は 0）。健全度 0.86〜0.93 を細かく振った 240 通り（`hover.thrust_corr` 1.0 / 1.2 / 1.3 × 不感帯あり・なし × 通信途絶 ARM の 0.6〜2.5 s 後）では、床の近くで LANDING に入る機体が 0 通り（浮く機体は、通信途絶の猶予 3 s より前に離陸が完了する）。 床の上で TAKEOFF が続いた機体は、離陸の推力（ホバー推力の見積もり + 補正）で浮かなかったので、見積もりから始まって降りる着陸の推力でも浮かない。したがって上限は、必要な入り口が無くなったので置かない。支持推力の窓の「床を離れた」の判定 `SupportThrustFilter::leftFloor()` は、窓を開くための内部の判定として制御器の中だけに残り、安全判断の判定条件には使わない（`blind_landing.hpp`）。
(4) **上昇ブレーキに上昇率の不感帯を入れる案（評価して採らなかった）:** 不感帯 0 / 0.03 / 0.05 m/s を、API 離陸後の `land`（120 本。加速度計のバイアス × `hover.thrust_corr` × モータの効率 × ノイズ）で比べた。LANDING の後の上昇の最大は 0.064 m で 3 値とも同じ。0.05 m を超えた本数は、不感帯 0 / 0.03 / 0.05 m/s で 1 / 1 / 2。降下速度の最大は 0.64 / 0.64 / 0.64 m/s、接地速度の最大は 0.63 / 0.63 / 0.62 m/s。不感帯 0 との 1 本ごとの差は、0.03 m/s では接地速度が −0.010〜+0.022 m/s（2 本の組で +0.017・+0.022）、LANDING の後の上昇が増えたのが 1 本（+0.017 m）、0.05 m/s では接地速度が下がったのが 1 本（−0.026 m/s）、上がったのが 1 本（+0.022 m/s）、上昇が増えたのが 2 本（+0.012・+0.024 m）。「上昇が増えず（最大・本数とも）接地速度が下がる」値は無いので入れない。
純関数: (1)(2) の計算は `blind_landing.hpp` の `landingBaseThrust()`・`brakeRiseThrust()` で、`PidController` の同名のメンバ関数はこれらを呼ぶだけ。検証: SILS `landing_rise_est_high`（較正前の LANDING）。ホスト試験 `test_blind_landing` が 2 つの純関数を検査する（基準の上限: 見積もりが支持推力の基準の 1.08 倍の内側・外側・基準より低い場合。上昇ブレーキ: 上昇率 0 以下は素通し、0.02 / 0.1 / 0.2 / 0.6 m/s の上限 0.95 / 0.75 / 0.5 / 0.5 倍、上昇率に対する連続性）。
- **`landing_descent_rate_` 等のparam化**は今後（現状は config 定数）。
- **検証:** SILS `alt_auto_takeoff`/`pos_auto_takeoff`（ARM 起動・スプール中 duty=0・0.5m 捕捉）、`alt_recenter_gate`（離陸後 Case A: 上げスティック無視→中央通過で有効）、`alt_inflight_switch`（飛行中 Case B: STABILIZE→ALT_HOLD 切替でジャンプなし）、`api_flight`（0.5m 統一）。**実機未検証。**
- **実装中の落とし穴2件（実測図つき解説）:** スロットルの中央は raw 3072（norm 0.5）で 2048 でない／TakeoffClimb の速度クランプは対称（±0.3m/s）でないと地上ToF なしの窓の行き過ぎを捕捉できない。詳細・実 SILS トラジェクトリ図は [`alt_hold_takeoff_findings.md`](alt_hold_takeoff_findings.md) を参照。

**注9（ベンチ用モータテスト後の再水平化 — 状態遷移を伴わないクラスA リセット）:** CLI のモータテスト（`motor test/all/sweep`, disarmed 限定）は機体を IDLE_GROUND に置いたままプロペラを回すため、IMU が揺すられて姿勢推定が accel-attitude の χ² ゲートを超えて外れうる。外れると、それを直す accel 補正自体が棄却され続けて自己復帰しない（IDLE_HELD → IDLE_GROUND の置き直しと同じ latch）。実機で確認: 10% スイープ後、accel は水平なのに推定は roll 16° / pitch −33° でなお増加。このまま ARM すると傾いた推定で離陸する。対策として StateTask が `motor_test` を監視し、テスト終了後に `MOTOR_TEST_SETTLE_US`（1.5 s）静穏が続いた時点で IDLE_GROUND なら `EstimatorCmd::Reset` と `Recalibrate` を発行する（リセットの時期を決めるのは状態機械、という architecture §4 の原則どおり）。静穏時間は `MOTOR_SWEEP_REST_US`（1 s）より長く、`motor sweep` ではモータ間の休止ごとではなく最後に1回だけ発動する。その時点で手持ち（IDLE_HELD）なら破棄する（置き直しが同じ処置を行うため）。**テスト実行中および再水平化の発行前は ARM を拒否する（ARM 前判定3）:** 拒否しないと ARM で IDLE_GROUND を離れて再水平化が破棄され、傾いた推定のまま飛ぶ（Issue #4）。StateTask が `StateManager::setBenchRelevelPending()` で待ち状態を渡し、再水平化の発行時に解除する（以後は校正完了判定＝判定2 がキャリブ完了まで ARM を阻む）。

**注10（地上での姿勢/重力の不一致 — ARM 拒否・自動 DISARM・再水平化。2026-10-02）:** 地上でプロペラが回っている間（ARMED_GROUND の ACRO/STABILIZE でスロットルが `TAKEOFF_THROTTLE_THRESH`=0.5 未満のとき、生スロットルが推力を決め、地上ゲートも時間制限もない）、モータ振動で ESKF の姿勢推定が外れうる。誤差が accel-attitude の χ² ゲートを超えると、直すための accel 更新が棄却され続けて自己復帰しない（latch、Issue #4）。注9 はベンチ用モータテストだけを扱ったが、ARMED_GROUND 滞在中の地上回転にも同じ latch が起こり、(a) 飛ばずに DISARM しても旧仕様では ESKF をリセットしない（旧「推定器は乱れていない」という前提が偽）、(b) スロットルを上げるとその傾いた推定で離陸する、という問題が生じる。対策は検出と判断の分離（INV-3）で構成する。
- **検出（ImuTask）:** 純粋ロジックの `TiltConsistencyMonitor`（`sf_estimator`）が毎 IMU 周期、LPF（時定数 `safety.tilt_check.lpf_s`）した加速度方向と、推定姿勢が示す body 系の「上」（`R^T·[0,0,-1]`、ESKF の期待重力 `R^T·[0,0,-g]` と同じ規約）の成す角を評価する。接地中（ImuTask の `groundContactTrusted()`。ToF が出力停止している間は ARM 中＝接地していない、注12）に角が `safety.tilt_check.max_deg` を `safety.tilt_check.persist_s` 連続で超えたら `system_status.attitude_mismatch` を立て、下回れば下ろす。加速度ノルムが妥当帯（0.5 g〜1.5 g、`config::TILT_CHECK_ACCEL_NORM_MIN/MAX`）を外れるサンプルはグリッチとして無視し状態・タイマを保持する。接地でなければ false＋タイマリセット。推定器の Reset 時にモニタも reset し、LPF を新姿勢で再シードする。
- **判断（StateManager / StateTask）:** ① **ARM 前判定4:** ARM には**肯定の判定** `system_status.attitude_verified`（接地中でモニタが判定済み、かつ角が `max_deg` 以内）が必要。不一致の間は `ARM rejected: attitude estimate disagrees with gravity`、未判定（起動・推定器 Reset の後、約 `lpf_s`）の間は `ARM rejected: attitude check not passed yet` で拒否する。不一致のないことだけでは足りない（未判定窓で未検証の推定を通さないため）。② **IDLE_GROUND 滞在中:** `attitude_mismatch` の立ち上がりエッジで ESKF **Reset のみ**を発行する（自己回復）。**IDLE_GROUND 滞在ごとに最大1回**で、IDLE_GROUND を出る（IDLE_HELD・ARMED_GROUND 等）と再装填される。傾いて横たわる機体では Reset→不一致再検出が約1.5秒周期で繰り返され収束を妨げるため。1回の Reset 後も不一致が続けば、人が持ち上げて置き直す（IDLE_HELD→IDLE_GROUND、再校正を行う）まで ARM は拒否されたままになる。③ **ARMED_GROUND 滞在中:** `attitude_mismatch` で DISARM を要求する（`Disarmed on ground: attitude estimate disagrees with gravity`）。遷移後の onEnter(IDLE_GROUND) が再水平化を行う（上表）。
- **なぜ Reset のみ（Recalibrate しない）か:** 再校正（`sf_calibration`）は機体が水平であることを前提に、加速度平均に重力を Z 軸のみで補正して加速度バイアスを求める。傾いたまま再校正すると重力成分がバイアスに混入する（30°傾斜で約 4.9 m/s²）。静止判定（EMA）は加速度のノルムしか見ないため傾きは検出できないが、**窓の完了時に水平チェックを行う**: 窓平均の加速度と機体 −Z（水平静止での向き）との成す角が `CALIB_LEVEL_MAX_DEG`（5°）を超える窓は `Calibration window rejected: not level (tilt … deg > … deg) — place the craft level; restarting` を出して棄却し、蓄積をやり直す（分散による棄却と同じ扱い）。実機の机上の傾きは 0.75〜1.14°（2026-11-01、機体 1 台を机の上でヨー方向に 4 方向回して計測。機体そのものの傾き約 0.8°＋机の傾き約 0.3°。11〜13° に傾けると全窓が棄却され、水平に戻すと約 3.5 s で完了）で、上限は傾き検査 `safety.tilt_check.max_deg`（10°）より十分小さく保つ（傾き θ で校正して後で水平に置くと姿勢誤差が約 θ 残るため）。これにより、墜落後に横倒しになった機体が IDLE_GROUND→IDLE_HELD→IDLE_GROUND の短い持ち上げ判定で再校正を始めても、横倒しのまま校正が完了して加速度バイアス約 ±9.8 m/s² が推定器に入ることはない（SILS `calib_side_lying`）。また、校正結果が無視可能な大きさ（デッドバンド未満）でも、以前の校正がすでに適用されている場合は新しい結果で置き換える（`Boot calibration: bias negligible — previous seed superseded`）。置き換えないと古い値が `reseedCalibration` により以後の Reset のたびに再適用され続ける。以前の適用が無い場合は従来どおり推定器に触れない。**既知の限界（未解決）:** 静止判定はサンプルごとの |ω| < 0.050 rad/s しか見ないので、0.005〜0.01 rad/s 程度のゆっくりした回転は窓に入る。2026-11-01 の実機確認では、置き直し後の再校正でヨー軸ジャイロのバイアスが −0.0070 / −0.0095 rad/s になった回があった（`reboot` 直後は −0.0013〜−0.0033 rad/s、他の置き直し 2 回は −0.0034 / −0.0043 rad/s）。較正中に機体がゆっくりヨー回転していた可能性があるが、原因は未確認。`eskf.use_mag` 既定 off では地磁気によるヨー補正がないため、この誤差は飛行中のヨー角の流れ（0.006 rad/s で約 20°/分）になりうる。確かめ方: 置き直して較正させた後、触らずにヨー角の推定値を 30 s 見る（流れ続ければ較正中の回転、止まっていればバイアス自体の変化）。自動経路（IDLE_GROUND 自己回復、ARMED_GROUND からの DISARM — 転倒・衝撃後は傾いていうる）は水平を知り得ないので Reset のみとする。Reset は姿勢を水平に再シードし、起動校正済みのバイアスを再適用する（`reseedCalibration`）。再校正は、水平と分かる場合 — 起動、人が置き直す IDLE_HELD→IDLE_GROUND、机上のベンチモータテスト（注9）— に限る。
- **なぜ DISARM か:** ACRO/STABILIZE では状態に関わらず推力がスロットルに追従するため、TAKEOFF 遷移を拒否しても離陸は止められない。止める手段はモータを切ることだけである。ALT_HOLD/POS_HOLD は地上で Grounded（推力ゼロ）だが、同じ規則で統一する。
- **暫定パラメータ:** 既定値 `max_deg`=10.0、`persist_s`=0.5、`lpf_s`=1.0 は**暫定**。根拠は実ログ1本（`analysis/datasets/flightlog/vehicle_hover_20260908T121243.sflog.zip`）の LPF 1.0 s での不一致: 回転前の地上 p99 0.98°、地上回転（0.45 s のみ）最大 5.7°、離陸後2 s 最大 7.3°、ホバー p99 5.0°／最大 5.3°。一方 Issue #4 のドリフトは 16〜33° に達した。長時間の地上回転ログで確認・調整する。
- **ARM 拒否の合図（単一の理由関数、注9・注10 共通）:** ARM 前判定は `StateManager::evaluateArmBlock()` の1か所にあり、最初に落ちた判定を `ArmBlock`（None/Pairing/Battery/Calibrating/BenchRelevel/TiltPending/TiltMismatch/TofSilent。TofSilent は注12）で返す。`requestArm()` はこれに従い、`StateManager::update()` が毎周期評価して**変化時のみ** `system_mode.arm_block` に発行する（NotifyTask と CLI は判定を再計算せずこれを読む。INV: 単一の情報源）。合図: ① StampS3 LED（地上 IDLE_GROUND）— TiltMismatch は**赤の低速点滅**（持ち上げて置き直す）、Calibrating/TiltPending/BenchRelevel は**マゼンタの低速点滅**（動かさず待つ）。Pairing（青高速）・Battery（シアン低速）は従来のパターン、TofSilent は**黄の高速点滅**（センサを確認する、注12。黄は他のどの表示にも使っていない色で、ALT_HOLD・LANDING のオレンジと区別できる）。② `requestArm()` が拒否すると `NotifyEvent::ArmRejected` を発行し、NotifyTask が errorTone を鳴らす（呼び出し元は RC ボタンの押下エッジ・機体ボタンのクリック・API verb で全てエッジ起因のため、1要求=1音でレート制限は不要）。③ CLI `status` の `prearm` 行は発行済みの理由を `ready` / `BLOCKED (...)` で表示する（読み取り専用）。IDLE_GROUND 以外（IDLE_HELD 等）での ARM 要求は判定対象外で無音。
- **注9 との関係:** 注9 は「ベンチ用モータテスト後」を**事象ベース**（`motor_test` の終了＋静穏）で再水平化する。注10 は**状態ベース**（推定が重力と食い違っている事実）で、ARMED_GROUND の地上回転まで含めて検出する。両者は同じ処置（Reset＋Recalibrate）を共有し、ARM 前判定3（注9）と判定4（注10）は独立に ARM を阻む。

**注11（地上 ARM 中は姿勢/レート積分器を保持 — 床拘束による二重巻き上がりの防止。2026-10-02）:** 実機ログ（`ground_spin_stab`、STABILIZE で ARMED_GROUND のまま床の上 34 s、スロットル ≤ 0.13）で、姿勢 PID の積分器が傾き推定の残差（~0.1〜0.3°）を積分し続け（34 s 時点でロールのレート目標 0.237 rad/s）、床に拘束された機体はそのレート目標に追従できないため、レート PID の積分器も巻き上がって**ロールのトルク上限 5.2 mNm に飽和**した（二重巻き上がり）。ミキサーに優先順位はなく、このトルクが小さな集合推力を圧倒して 24〜34 s のモータ duty は FR/RR/RL/FL = 0.009/0.076/0.419/0.467（差 0.458）になり、巻き上がった積分器を持ったまま離床すると離床時にロール数十度（1 軸の粗い見積り）になる。ファームの PID を実ログで再生した検証では、地上で積分を止めると duty 差は 0.026、離床時の積分器は 0 だった。スロットルでのゲート（床値）は 0.25 以上でしか効かず、状態でのゲートと同じ結果になる。
- **修正:** `VerticalPhase::Grounded` の間、姿勢（roll/pitch）とレート（roll/pitch/yaw）の PID の**積分器を保持**する（`PID::integration_enabled=false`。値はそのまま・リセットしない・蓄積しない。P/D は通常どおり）。毎周期 `PidController::applyGroundIntegratorHold()` が `phase_` から設定する。制御則・パイプラインは全フェーズで同一（INV-1）で、変わるのは積分器の更新可否のみ。
- **Grounded の範囲:** ARM（`ControllerCmd::Reset`）から `ControllerCmd::TakeoffComplete`（全モードで TAKEOFF→FLYING に発行）まで。ACRO/STABILIZE は手動離陸で `onTakeoff()` を呼ばないため、**TAKEOFF 状態（ToF 空中検知まで）も Grounded のまま**で、離床の過渡は P/D のみで通過する（離床の ToF 0.15 m まで）。ALT/POS は ARM 後約 0.3 s のスプールを経て `onTakeoff()` が TakeoffClimb へ進めるので保持は約 0.3 s。**空中で Grounded に戻る経路はない**（`reset()` は IDLE_GROUND→ARMED_GROUND のみ、モード切替・FLIP・LANDING・通信途絶着陸は `phase_` を Grounded にしない）。
- **鉛直（ALT/POS）:** `alt_pos_`/`alt_vel_`/`pos_*`/`vel_*` は Grounded の間そもそも `compute()` が呼ばれない（推力ゼロ強制）ため積分しない。変更不要。
- **検証:** host ユニットテスト `pid_integration_hold*`、SILS `stab_ground_windup`（校正後に加速度バイアス 0.07 m/s² を注入し、STABILIZE で床の上・スロットル 0.13 を 25 s 保持 → 離床。修正前: 地上 duty 差 0.586・離床で転倒（傾き 180°）、修正後: 地上 duty 差 0.006・離床時傾き 1.65°）。**実機未検証**（STABILIZE で地上 30 s 低スロットル後に離陸し `sf log wifi` で duty 差を比較する）。

### ペアリング状態遷移（PairingState — FlightState と並行）

FlightState とは別の独立状態機械。StateManager が所有する（[`architecture.md`](architecture.md) §4
「ペアリング状態の位置づけ」参照）。上の FlightState onExit/onEnter テーブルとは別物。

| 遷移 | トリガー | アクション |
|------|---------|-----------|
| 起動 → Paired / NotPaired | NVS load | 相手側の MAC あり→Paired・相手側をユニキャスト peer 登録 ／ なし→NotPaired |
| NotPaired → Pairing | 自動（未ペア起動） | comm: PairingPacket を 500ms 周期で broadcast 開始、notify: LED青速点滅+ブザー |
| 任意 → Pairing（再ペア） | `button_event`=LongPress3s（IDLE_GROUND / IDLE_HELD） | 既存ペア破棄（NVS clear）→ 上記 Pairing 開始 |
| Pairing → Paired | comm 発行の `pairing_complete`（src MAC 学習） | comm: NVS保存・broadcast peer 削除・相手側をユニキャスト peer 登録、notify: 点滅解除 |

**ガード:** Pairing 中は `requestArm` を拒否（StateManager）。突入は IDLE_GROUND / IDLE_HELD
（地上または手持ち）で可能（2026-09-12、手持ちでのボタン操作を許容するため拡張。ARM 自体は
従来どおり IDLE_GROUND 限定なので安全性に影響しない）。

**トピック:**
- `pairing_state`（Latest, StateManager → comm / notify）: 現在の PairingState を周知。
- `pairing_complete`（Latest, comm → StateManager）: comm の現在のバインド状態（bound + 学習/復元した
  送信機 MAC）。起動時の NVS 復元（restored=true）と Pairing 成立（restored=false）の両方を運ぶ。

**注12（下向き ToF の出力停止 — 上昇中止・着陸・ARM 拒否。2026-11-09。コード上の名称は silent / `TofSilent`）:** 事故（自動離陸 API、POS_HOLD）: 下向き ToF が最後のサンプル（status=5）を出した後、新しいサンプルを出さなくなった。接地フラグ（`TakeoffLandingMgr::on_ground_`）は ToF サンプルだけで解除されるため「接地」のまま固定され、ImuTask は毎周期 `holdPositionVelocity()` で位置・速度を 0 に固定し続けた。自動離陸の上昇段（`TakeoffClimb`）は速度誤差を積分して推力を上げ続け、機体は天井に衝突した。`sensor_health`（1 Hz）は異常を検出していたが Failsafe・状態機械には接続されていなかった。対策は **「ToF が出力停止したら見えないまま飛ばさない。ただし手動飛行は取り上げない」** を状態機械の規範表の行にすること（2 段階: 最初の対策＝上昇中止・着陸・ARM 拒否、続く改修＝判定を Failsafe に移し、ToF なしの降下を開ループの推力ランプにし、ACRO・STABILIZE は強制着陸しない）:
- **「出力停止」の定義（「エピソード」に拡張した）:** 次のいずれかで**エピソード**が始まり、**推定器が ToF を取り込み直したときだけ**終わる。(a) 新しいサンプルが `sf::limits::kTofSilenceUs`（**0.3 s、候補値**）以上来ない（時刻印だけで判定）。(b) **空中で無効サンプル（`valid=false`）だけが続く**（ToF が測れない床材など。サンプルは届き続けるので (a) には見えない）。「空中」の意味と継続時間は**フェーズごと**（`classifyInvalidSample()`）: FLYING は `kTofInvalidPersistUs`（0.5 s、候補値）。**TAKEOFF は、加速度積分の推定で `kTofInvalidTakeoffRiseM`（5 cm）上昇した後、または TAKEOFF の開始から `kTofInvalidTakeoffMaxUs`（2 s）経った後だけ数え、数え始めてから `kTofInvalidTakeoffPersistUs`（0.15 s）続いたら成立**（実機の床ではセンサは TAKEOFF に入った後 186〜344 ms 無効だから。下の注 12）。LANDING は、最後の有効な高さが `kTofInvalidLandingMinHeightM`（0.10 m）以上のときだけ数える（床の近くの不感帯は正常）。FLIP は数えない（センサが横を向く）。**地上（IDLE_GROUND・ARMED_GROUND・DISARM 中）の無効サンプル（最小レンジ未満）は正常**で、数えない。単発の範囲外はエピソードではなく、後段で気圧計と一緒に扱う。理由コード `tof_reason`: NoSample・InvalidStuck・Reacquiring・StatusStale。一度もサンプルが来ていない（時刻印 0）場合は出力停止とみなす（最初の 1 個が来るまで ARM 拒否。起動校正は約 3.5 s かかるので、校正中の表示は校正が先に出る）。
- **判定の実装 `sf::SilenceWatch`（`sensor_freshness.hpp`。2026-11-09 の改修で、状態を持たない時刻差から変更）:** 時刻印は uint32 のマイクロ秒で、別タスクが書く。2 つの誤りを避ける。① 読み出し順序の逆転: 時刻印が「現在」より数 µs 新しいと、符号なしの差は約 4.29e9 に桁あふれして正常を出力停止と誤る → 新しいサンプルの経過は**符号付き**で取り、負は「今届いた」とする。② 長時間の出力停止: 状態を持たない 32 bit の差は 2^31 µs（35.8 分）で負に戻り（「現在より新しい」＝健全と誤る）、71.6 分で小さな値に戻る → 出力停止が長引くほど再び「動作」と判定される危険側の誤りで、状態を持たない関数では直せない。そこで毎周期 `observe(現在時刻, 最新の時刻印)` を呼ぶ監視器にし、時刻印が**変化**したときだけ新サンプルとして経過を取り直し、変化しなければ前回の観測からの経過を加算して `UINT32_MAX`（71 分）で**飽和**させる（巻き戻らない）。2 回の観測の間の自前計測の間隔だけがモジュラ差で、35.8 分未満なら正しい（Failsafe は 10 Hz、StateManager は 50 Hz、PowerTask は 1 Hz、制御器は 400 Hz）。ToF の出力停止は Failsafe が `checkTofSilence()` で観測する（10 Hz）ので、他の部品が読む値は直近の Failsafe 更新時点のもの。`failsafe_status` はレベル型トピックで最後の値が読み続けられるので、**読む側は必ず `sf::FailsafeStatusGuard`（`failsafe_status_guard.hpp`）に通す**: 1 s（`kFailsafeStatusStaleUs`、候補値）更新されない状態、または一度も発行されていない状態は「出力停止」（理由 StatusStale）と読む。Failsafe（PowerTask）が停滞・停止しても、最後の「出力停止でない」が効き続けて ToF の保護が全部無効になることはない（読む側は ImuTask・ControlTask・StateManager・NotifyTask・CLI で、それぞれ自分のガードを持つ）。③ **観測の空き（2026-11-09 の再度の改修）:** 呼び出し側が 35.8 分以上観測しないと、2 回の観測の間の経過が小さなモジュラ差でなくなる。そこで、時刻印が変化しない場合に経過が許容（`kClockSkewToleranceUs` = 1 s）を超えて負に見えるとき、および新サンプルの時刻印が now より許容を超えて先に見えるとき（初回観測を含む。40 分前の時刻印は負に巻き戻る）は**飽和（停止）**とする。71.6 分以上の空きは小さな値に戻り検出できないので、**観測は飛行状態（ARM/DISARM）に関わらず毎周期行う**（制御器は `compute()` が ARM 中しか走らないため、制御タスクが ARM の gate より前に `IController::observeLinks()` を毎周期呼ぶ。`Failsafe`・`StateManager` の通信断猶予・`PowerTask` の `sensor_health` も無条件に観測する）。時刻印がちょうど 0 になる瞬間（2^32 µs の巻き戻り）は、0 以外を一度見た後なら新サンプルとして扱う（未発行は「0 以外を一度も見ていない」ときだけ）。
- **同じ監視器を使う他の安全判定（時刻印が別タスクのもので uint32 の差を取っていた箇所）:** `Failsafe::checkCommTimeout`（通信断 COMM_LOST。時刻印が now より新しいと誤発報していた）、`StateManager::update` の通信断ホバー猶予（3 s）、`PidController::compute` のリンク生存（着陸中の水平強制）と rc 誘導の鮮度解除、`PowerTask` の `sensor_health`。ホスト試験 `test_sensor_freshness`（36 分超・80 分の出力停止、36/40/60 分の観測の空き、40 分前の時刻印の初回観測、時刻印 0 の巻き戻り、未受信、未来時刻、通信断 500 ms）、SILS `disarm_long_idle`（38 分無音の DISARM のあとの飛行）と `test_tof_silence`（75 分の出力停止、未受信）で検査する。**未確認:** 実機での長時間（36 分超）の連続動作。
- **しきい値 0.3 s の根拠（候補値）:** ToF は約 30 Hz。飛行ログで測ったサンプル間隔の最大が 67〜99 ms なので 3 倍以上の余裕があり、0.3 m/s の上昇でも約 0.3 m 以内に反応できる。実機での偽検出の頻度は未確認。
- **置き場所（通信断と同じ形、2026-11-09 に判定を Failsafe に移した）:** 検出は **Failsafe**（`Failsafe::checkTofSilence()`、PowerTask の 10 Hz）、判断と遷移は **StateManager**。Failsafe は `sensor_snapshot.tof_timestamp`（ImuTask が最後に消費した ToF サンプルの刻印）を `sf::SilenceWatch` で観測し（R16）、結果を 2 本で渡す: **(1) イベント** `system_alert` の `AlertType::TOF_SILENT`（CRITICAL）— ARM 中だけ、ARM 中の最初の更新で即座に、以後は出力停止の間約 1 s ごとに再発報（COMM_LOST と同じレベル意味論。FLIP 中に消費されても失われない）。DISARM 中はアラートを出さない（ToF が動かない機体で毎秒ログが埋まるため）。**(2) 継続状態（レベル）** `failsafe_status.tof_silent`（エピソード中は真）と理由 `tof_reason` — 毎回発行し、StateManager（ARM 前判定3b・モード切り替えの拒否・ToF なしの着陸の時間切れ）、ImuTask（接地フラグの信用）、ControlTask（制御器への `StateEstimate.tof_silent` の注入）が読む。Failsafe は時刻印の鮮度に加え、**ImuTask が `tof_health`（`ToFHealth`: `altitude_reliable`・`invalid_stuck`）に出す事実**を使う。ImuTask は ToF サンプルの唯一の消費者で、推定器が各サンプルをどう扱ったか（`IEstimator::lastToFObservation()`: 採用されたか・イノベーション）を持っているため、純粋ロジックの `sf::ToFHealthTracker`（`tof_health_tracker.hpp`）で追跡する。判断（何をするか）は純関数 `decideToFSilenceAction(状態, モード, 出力停止, ToF なしの着陸時間)` と `toFSilenceRefusesMode`（`tof_silence_policy.hpp`）に分離し、規範表の各セル（状態 × モード）をホストで検査する（`test/test_tof_silence.cpp`）。遷移の実行は StateManager の `handleAlert(TOF_SILENT)`（`handleToFSilenceAlert`）と `update()`（`handleBlindLandingTimeout`）が行う。`sensor_health`（1 Hz）ではない。
- **上の表の行:** ARMED_GROUND → DISARM（モードを問わない）。TAKEOFF・FLYING は**飛行モードで分ける**: ALT_HOLD・POS_HOLD（と、それらにしかない自動離陸）→ LANDING、**ACRO・STABILIZE → 強制着陸しない**（警告のみ。パイロットが操縦を続け、着陸後は出力停止が続く限り次の ARM を拒否）。LANDING に入って `kBlindLandingTimeoutUs`（10 s）経ち、かつ出力停止が `kBlindLandingMinSilentUs`（5.5 s）連続 → 着陸完了。FLIP・地上状態（INIT・IDLE_HELD）は何もしない。**ARM 前判定3b**（`evaluateArmBlock`）が地上の ARM を拒否し、理由 `ArmBlock::TofSilent`（値 7、ログ上の `arm_block`）を発行する。LED は黄の高速点滅（地上の ARM 阻害）、CLI `status` の `prearm` 行は `BLOCKED (downward ToF silent - check the sensor)`。ARM 中（手動飛行の警告）は本体 LED と MCU LED が黄高速点滅し、警告音（`NotifyEvent::TofSilentWarn`）が最初の警告と以後約 5 s ごとに鳴る。**ALT_HOLD・POS_HOLD への切り替えは拒否**（`ModeBlock::TofSilent`、§3.1 の ToF 出力停止の列）。判定番号は既存の注9/注10（判定3・4）を保つため 3b とし、優先順位は BenchRelevel と傾き判定の間にした。
- **ToF なしの着陸（降下の方式、2026-11-09 の ToF なし降下の閉ループ化: 開ループの時間関数 → 鉛直速度の閉ループ）:** 出力停止中は **ToF による着地判定（`system_status.landing`）も高度推定も信用しない**。
  1. **旧方式の欠陥（SILS の真値で再現した）:** 降下は「開始推力 × (1 − 時間の関数)」の開ループで、開始推力は加速度計で較正した値だった。しかし飛行の最初の 2 s は較正を使わず（床が重さの一部を支える）、代替の 0.95 × hover 推力の見積もりから始まる。自動離陸の上昇中に ToF が出力停止すると、見積もりが高いほど上昇が速い（出力停止の瞬間に 1.3 m/s）うえ、その上昇速度が降下の開始に残った。main（2026-11-08 時点）の再測定（`hover.thrust_corr` 1.0 / 1.1 / 1.2、ノイズなし）で真値の最高は上昇初期の出力停止で 0.46 / 4.0 / 10.8 m、API 離陸で 0.36 / 3.4 / 11.5 m、ノイズ n2（3 seed の最悪）で 9.6 / 25 / 38 m と 11.5 / 28 / 47 m。**空中でのモータ停止は 80 本中 8 本**（高度 1.1〜36 m）。根本: 開ループの時間関数は、見積もり誤差と初速に対して「下降する」ことを保証できない。
  2. **鉛直速度の推定（`VerticalVelocityEstimator`、`vertical_velocity_estimator.hpp`。純粋ロジック。制御器が毎周期進める）:** ESKF の鉛直速度は ToF が無いと高度の観測を失い、接地フラグが「接地」の間（ToF の引き継ぎ高さ 0.15 m より下の離陸の上昇）は 0 に固定されるので使えない。そこで 3 つの規則で専用の値を持つ。**(a) 地上で静止**（DISARM、フェーズ Grounded）: 速度 := 0、加速度計のオフセット B := 生の上向き成分 − g のローパス（静止なら読みは g のはず。ゼロ速度更新）。**(b) ToF が健全で推定器が取り込み済み、かつ ESKF の鉛直状態が自由**（`StateEstimate.airborne` = `system_status.airborne`。ControlTask が注入）が `kVerticalVelocityHandoffSettleS`（1 s）続いた: 2 次の相補オブザーバ（時定数 `kVerticalVelocityEskfTauS` = 4 s）が ESKF の速度に従い、B を学習する。**(c) それ以外**（出力停止、または (b) の整定前）: 速度 += (生の上向き成分 − g − B) × dt、B は固定されたまま。**接触の規則:** 上向きの読みが接地の衝撃の閾値（2 g）を 2 サンプル連続で超えたら床が機体を止めたので、読みが閾値の 0.75 倍を下回った時点で速度を 0 にする（ゼロ速度更新）。無いと、衝撃の積分（加速度計のレンジで頭打ち、ノイズ入り）が地上で +0.2 m/s の誤差を残し、接地検出器の「動いていない」が成り立たない（SILS のノイズ n2 で衝撃カットを無効にした 80 本中 22 本が接地を検出できなかった。規則の追加後は 80 本すべてで検出）。**最初の設計（ToF が健全な間は ESKF の値に毎周期リセット）からの変更と実測の根拠:** 最初の実装は毎周期リセットだったが、SILS（`tof_silent_takeoff_est_high`）で、接地→空中の引き継ぎの 0.4 s 後に真値 1.27 m/s に対し ESKF は 0.56 m/s で、リセットはその誤りを積分に持ち込み、出力停止の瞬間から 0.5 m 余計に上昇した。また ESKF の加速度バイアス推定は降下の最初の 0.6 s で +0.19 m/s² 動き（ノイズなしの SILS で生の加速度計は無バイアス）、ESKF が補正した加速度計の測定値を積分すると 0.6 s で 0.12 m/s ずれた。そこで、生の加速度計（`specific_force + accel_bias`）の上向き成分を使い、オフセット B は自前で持って出力停止中は固定されたままにする。「加速度バイアスは ESKF の推定値を使う」は、ESKF の補正を戻して生の値で積分することに変えた。**積分の誤差は（オフセット誤差）×（最後のリセットからの時間）**（0.1 m/s² で 3 s なら 0.3 m/s）。SILS の n2（ノイズあり）の降下で真値との差は 0.1〜0.2 m/s。気圧計（次の段階）は `fuseExternalVelocity()`（積分が動いている間だけ効く）に速度を渡せる形にしてある（インタフェースのみ。呼び出し側は変わらない）。
  3. **降下の推力（`BlindDescent`、`blind_descent.hpp`。制御器は `state.tof_silent` を見る。ControlTask が `failsafe_status` から注入。2026-11-09 の ToF なし降下の上限の改修で「上昇できない」形に変更）:** 推力は**常に 上限 = `kNoTofThrustCapRatio`（0.95）× 支持推力 以下**で、hover 推力の「見積もり」は上限の基準に使わない。加速度計の積分の速度は、最後のリセット（地上静止、または ToF が健全で ESKF に従った時点）からの時間とともに誤差が増える（実機ログを推定器に通すと、ホバー中の ToF 喪失の後 1 s で −0.21 m/s、3 s で −0.25〜−0.39、5 s で −0.44〜−0.71。flip_test2 は 2 s で −0.34 m/s）ので、推定が「下降しすぎ」側にずれたとき推力を上げて直す PI は機体を上昇させた（main（2026-11-08 時点）、SILS: 加速度計のオフセット 0.1 m/s² で再上昇して 1.43 m/s で接地、0.2 m/s² で 2.69 m・5.06 m/s、床に柔らかく接地した後に PI が推力を上げて自分で再離陸し 1.21 m まで上昇した例も）。新しい形: **速度推定を信用できる間**（`VerticalVelocityEstimator::sinceResetS()` ≦ `kVerticalVelocityTrustS` = 2 s）は 推力 = 支持推力 + PI(目標 −0.4 m/s − 速度) を `[kBlindDescentThrustMinFrac(0.25) × 支持推力, 上限]` に制限（PI が出せるのは推力を下げる向きだけで、降下が目標より速くても上限を超えて止めには行かない）。**信用時間を過ぎたら**（降下の開始時点で既に過ぎている、ToF が止まったまま長く飛んだ手動モードの後も含む）PI も積分も止めて推力を上限の一定値にする（小さな一定の下向き加速度: 0.05 g）。最後に後半の低下の係数を掛ける。**支持推力 = 機体を支えていた指令推力を加速度計で較正した値**（`SupportThrustFilter`、`blind_landing.hpp`）: 機体が床を離れた後（健全な ToF が空中と言う、または上昇量の推定が 0.10 m、またはろ波した読みが g を 1 m/s² 超えた後）の、読みが g の ±5 %（`kSupportHoverBandFrac`）以内のサンプルだけを、`kSupportWindowS` = 2 s 分、推力（作動の遅れ `kSupportThrustLagS` = 0.08 s を通したもの）と上向きの加速度計の測定値の平均を取り、支持推力 = 推力の平均 × g ÷ 読みの平均（見積もりの 0.5〜1.5 倍に制限）。床が重さを支えるのは離陸前だけで、離陸後のホバー近傍のサンプルでは推力は重さに等しいので、商は（推力と指令の曲線が何でも）ホバー推力になる。**旧方式（飛行の最初の 2 s は見積もりに戻す、全サンプルの商）の欠陥:** SILS で、上昇中（+0.2 g）の商は出る推力が指令に対し上に凸なためホバー推力の 11〜12 % 高い。サンプルを選ぶ前の窓 0.3 s では実機のホバーの散らばりが ±4.9 %（0.5 s: ±3.2 %、1 s: ±2.5 %、2 s: ±1.0 %）。**窓が無いとき**（離陸前、またはホバーを飛ぶ前に出力停止が来る）: 支持推力 = min(空中の全サンプルでの商 × `kSupportClimbDerate`(0.85)［0.15 s 以上たまったら］、`kBlindUncalibratedFrac`(0.75) × hover 推力の見積もり)。上限はそれに従い、見積もりが真のホバーの 1 / (0.75 × 0.95) = 1.40 倍未満の間は重さを下回る（SILS の `hover.thrust_corr` 1.2 は n0 で 1.0〜1.2 倍、n2 で 1.35〜1.45 倍）。目標の降下速度 `kBlindDescentRateMps` = 0.4 m/s（ToF が動作している閉ループの着陸は 0.3 m/s）。PI は共通の `PID`（`pid.hpp`、条件付き積分）で、ゲインは hover 推力の見積もりに対する割合: Kp = `kBlindDescentKp` = 1.0、Ti = `kBlindDescentTiS` = 2 s、出力制限 1.0。パラメータ `safety.blind_descent.rate_mps` / `kp` / `ti_s` / `cap_ratio`、`safety.vvel.tau_s` / `settle_s` / `trust_s` は SILS の掃引用（既定は `sf::limits`。`cap_ratio` を 1 以上にすると旧挙動の再現になる）。**代償と実態（実際の挙動）:** この降下は「目標 0.4 m/s の速度ループ」ではない。推力は上限（支持推力の 0.95 倍）を超えて上げられないので、PI が効くのは (a) 上昇している機体の推力を下げて止めることと (b) 降下の最初の加速（推力を支持推力より下げる）だけで、降下が 0.4 m/s を超えても、推力を上限より上げて止めには行かない。その後の降下は**推力の上限が決める一定の下向き加速度（約 0.05 g = 0.49 m/s²）の降下**で、降りるほど速くなる（速度 ≒ √(2 × 0.49 × 高さ)）。1.2 m の天井から約 0.8〜1.2 m/s で接地する（旧方式は 0.4〜0.5 m/s）。支持推力が真のホバーより低く較正されると、その分だけ強く当たる（−2 %: 1.2 m/s、−10 %: 約 1.8 m/s。ホスト試験 `loop_lands_gently_for_a_support_thrust_error_up_to_the_margin`）。支持推力が真のホバーを 3.1 % 超えて高く較正されると上限が重さを上回り、機体はゆっくり上昇しうる: その場合は後半の低下が最後の安全策になる（`loop_with_a_support_thrust_above_the_margin_ends_with_the_late_fall`）。
  4. **最後の安全策（後半の低下、`blindLandingLateFactor`）:** 積分の速度はドリフトしうるので、ループだけに頼らない。ToF なしの降下が（1 回の LANDING の中で通算）`kBlindLandingLateStartS`（5 s）続いたら推力に係数を掛け、係数は `kBlindLandingLateRampPerS`（0.3 /s）で下がって **8.33 s（= 5 + 1/0.3）でゼロ**になる（速度推定が何であれ）。通常の降下（天井 1.2 m から約 2〜3 s）は 5 s より前に終わるので、ここは働かない。支持推力の較正が真のホバーを 10 % 高く外した場合も、機体は時間切れ + 落下時間内に着地する（ホスト試験 `loop_with_a_support_thrust_above_the_margin_ends_with_the_late_fall`）。**時間切れ `kBlindLandingTimeoutUs`（10 s、候補値）は LANDING に入った時刻から数え、かつ ToF の出力停止が連続して `kBlindLandingMinSilentUs`（5.5 s、候補値）続いていること**、推力がゼロに達した後（8.33 s + 1.67 s = 13 m からの落下時間）で、推力は常に重さ未満なので機体は常に沈んでいる。空中でのモータ停止は 0 件（2026-11-09 の ToF なし降下の上限の改修の表）。
  5. **終わりは「衝撃のあとの静止」か「時間切れ」**（`TakeoffLandingMgr::updateBlindTouchdown` → `BlindTouchdownDetector`、`blind_touchdown.hpp`。ImuTask が毎 IMU 周期、LANDING かつ ToF 出力停止のときだけ供給。推力比と鉛直速度は制御器が `controller_status` に出す）。旧規則（`|加速度計の測定値|` が 1.4 g を 4 サンプル連続）は、実機のホバー・飛行の振動に余裕がなかった（実機ログ 3 本で飛行中に 1.4 g × 2 サンプルが 16 / 11 / 44 回、1.4 g × 4 が 1 / 0 / 2 回）。新規則は、衝撃（2 g を 2 サンプル連続）の後に、(i) ろ波した（時定数 0.1 s）`|加速度計の測定値|` が 1 g の ±0.15 g 以内（加速していない）、(ii) 指令推力が支持推力未満（`kBlindTouchdownThrustFrac` = 1.0。降下の推力は常に 0.95 倍以下なので常に真。以前は 0.85 倍）、(iii) 鉛直速度の推定が ±0.15 m/s 以内（動いていない）が 0.25 s 続いたらラッチ。定常の降下は (i) を満たすが (iii) を満たさず、ホバーは (iii) を満たすが (ii) を満たさず、衝撃は飛行中にほぼ無い。**閾値の根拠（実機ログ 3 本）:**

     | 衝撃の閾値 × 連続サンプル数 | front_tof_poshold_yaw360 | front_tof_poshold_yaw4x90 | flip_test2 |
     |------|------|------|------|
     | 1.4 g × 2 | 16 | 11 | 44 |
     | 1.4 g × 4 | 1 | 0 | 2 |
     | 1.6 g × 2 | 1 | 0 | 6 |
     | 1.8 g × 2 | 1 | 0 | 1 |
     | **2.0 g × 2（採用）** | **0** | **0** | **0** |
     | 2.5 g × 2 | 0 | 0 | 0 |

     表は飛行中（TAKEOFF / FLYING。FLIP と最後の墜落を除く）に閾値を連続サンプル数以上超えた回数。yaw360 の 1.6〜1.8 g の 1 回は着陸（LANDING）の本物の接地。飛行中の最大 |加速度計の測定値| は yaw360 のホバーで 1.56 g（99.9 パーセンタイル 1.50 g）、yaw4x90 の離陸で 1.58 g、flip_test2 の激しい飛行で 2.28 g。**本物の接地**（yaw360、ToF 生存中の閉ループ着陸 0.28 m/s）は 2 g 超が 3 サンプル連続、最大 3.39 g で、2.0 g × 2 は余裕をもって見える。静止の条件を含めた検出器を 3 本のログに再生すると、飛行中の誤検出は 0 で、本物の接地は衝撃の 0.45 s 後にラッチする。**SILS の実測:** 3 G の衝撃カットを無効にして（`--param safety.impact.accel_g=10`）80 本を実行すると、80 本すべてが検出器で終わり（接触の 0.38〜0.91 s 後、中央値 0.44 s。空中でのラッチは 0、DISARM 時の真値の高度は最大 0.021 m）、接触の規則が無いと 58 本だった。**ただし通常の設定では、IMPACT（3 G・2 サンプル）の非常 DISARM が静止の判定より先に働く:** 今のファームウェアの 80 本中 79 本が IMPACT で終わった（残り 1 本は柔らかい接地で検出器が先）（SILS の接触は 6〜9 g、実機ログの本物の接地も最大 3.39 g）。DISARM は床の上でモータを止めるので着陸としては同じだが、アラート種別は IMPACT（非常）になる。静止の判定が先に働くのは接触の衝撃が 3 G 未満の柔らかい接地だけで、それ以外では最後に床で止まった状態の確認に回る。この点は未解決の設計課題として残る（出力停止中の LANDING に限り IMPACT の閾値を上げる案は、墜落の検出を弱めるため採用していない）。
  6. **接地フラグ（`on_ground_`）の扱い:** `TakeoffLandingMgr::on_ground_` は ToF サンプルだけで更新され、出力停止中は最後の値のまま固定される。固定された値が「接地」だと ImuTask が位置・速度を 0 に固定し続ける（事故の機序）。そこで **ImuTask は `groundContactTrusted()` を使う**: ToF 出力停止中は「ARM 中 → 接地していない、DISARM 中 → 接地している」とし、鉛直ハンドオフ（`holdPositionVelocity` の固定・離陸エッジのリセット）、傾きモニタ、発行する `system_status.airborne` がこれを使う。生のフラグ（`isOnGround()`）は ToF が動作しているときだけ信用する。制御器が受け取る `StateEstimate.airborne` も `system_status.airborne` で、鉛直速度の推定が ESKF に従ってよいかの判定に使う。
  数値はすべて `sf::limits`（`flight_limits.hpp`）の 1 か所にあり、すべて候補値。実機での検証はまだ。
- **復帰は「サンプルが届いた」でなく「推定器が ToF を取り込み直した」で判断する（2026-11-09）:** 欠陥は 2 つあった。ARM 中に出力停止すると生の接地フラグ `on_ground_` が「接地」のまま固定され、ToF 復帰時に `groundContactTrusted()` がその値を再び信用した。復帰直後の最初のサンプルが範囲外で無効だと `evaluateToF` が読み飛ばして固定されたままが続き、推定の位置・速度が 0 に固定され、ALT_HOLD で 10 m 以上上昇して自動着陸が高所で完了しモータが止まった（SILS `tof_recover_frozen_ground`。修正前の真値 alt_max 10.9 m、修正後 2.5 m）。ESKF は復帰後に ToF を取り込み直さない。イノベーション判定（0.5 m 超を棄却）が働き、外す仕組みは宙返りの終わりだけだった。ToF なしの積分で推定高度が 0.8〜1.5 m ずれると以後の全サンプルが棄却された（SILS `tof_recover_climb_alt`。修正前の alt_rmse 1.19 m、修正後 0.02 m）。対策:
  1. **検出（ImuTask、`sf::ToFHealthTracker`、`tof_health_tracker.hpp`。純粋ロジックでホスト試験 `test_tof_health`）:** サンプルごとと毎 IMU 周期の時計で更新する。喪失（0.3 s の無サンプル。飽和する `SilenceWatch` なので 36・72 分超でも喪失のまま）で信頼を失い、`altitude_reliable` は「**小さなイノベーション（`kTofReacquireInnovationM` = 0.10 m 以下）で採用されたサンプルが `kTofReacquireSamples`（10）個連続**し、かつ**喪失後の最初のサンプルから `kTofAliveMinUs`（1 s）以上 ToF が生存**」のときに限り真（ラッチ。喪失だけが解除する。点滅するセンサは到達できない）。「一致」と数える無効サンプルは地上（IDLE_GROUND・ARMED_GROUND・DISARM 中。最小レンジ未満）のものだけ。LANDING・FLIP 中の無効サンプルは一致と数えない（LANDING で無効サンプルだけが届いて約 1 s で取り込み直し済みと判定される欠陥があった）: 数えられるときは継続の証拠、床の近くの LANDING と FLIP は「無視」（一致でも証拠でもない）。継続判定（`stuck_`）を立てる箇所で、取り込み直しの保留（`reacquire_pending_`）も立てる（そうしないと、継続の後の最初の有効サンプルが取り込み直しなしで採用された）。推定器が ToF を使わない構成（`eskf.use_tof=false`）では「生存」だけを求める。
  2. **推定器:** 喪失後の最初の**有効**サンプルの前に `IEstimator::reacquireAltitude()`（ESKF は `EskfCore::reacquireVertical()`）を呼ぶ。鉛直（POS_Z/VEL_Z）共分散を広げ、次に採用するサンプルだけイノベーション判定を外す。**宙返りの終わり（`holdAttitudeCorrection(false)`）と同じ関数を共有し、並列の実装は作らない**（試験 `test_eskf_flip_hold`）。
  3. **接地フラグ:** ARM 中にエピソードが始まったら、ImuTask が `TakeoffLandingMgr::markAirborne()` で**生のフラグを「空中」側へ倒す**（立ち上がりエッジで 1 回）。「接地」に戻るのは既存の空中→接地デバウンス（`flip_confirm_samples` 回連続の有効な地上読み）だけで、無効サンプルや欠測では戻らない。さらにエピソードの間（サンプルが戻ってから推定器が取り込み直すまでも）`groundContactTrusted()` は「ARM 中＝空中」を返し続ける。
  4. **許可条件:** ARM・高度モード（ALT_HOLD / POS_HOLD）・自動離陸の許可は「新しいサンプルが届いた」でなく**エピソードが終わった**こと（`failsafe_status.tof_silent` が偽）。ARM 前判定3b は ACRO を含む全モードで拒否する（`toFSilenceRefusesArm`）。ToF なしの降下は、エピソードが終わるまで続く: 推力の上限は支持推力の 0.95 倍（重さ未満）、速度推定を信用できる間は速度ループ（PI）、超えたら上限の一定推力、降下 5 s 後からの後半の低下と LANDING に入ってから 10 s の時間切れがあり、接地は加速度計（衝撃のあとの静止、または床が機体を支えている）で判定する。
  5. **無効サンプルの継続（上の定義 (b)）:** 空中で無効サンプルだけが続くと、サンプルが届き続けていても出力停止と同じ行を適用する（ALT_HOLD / POS_HOLD → LANDING、ACRO / STABILIZE → 警告のみ）。フェーズごとの意味は `tof_health_tracker.hpp` の `classifyInvalidSample()`（純関数。全行をホスト試験 `test_tof_health` で検査）。**実機の床:** 実機ログ 3 本（`front_tof_poshold_yaw360`・`front_tof_poshold_yaw4x90`・`flip_test2`）で、床の上のセンサは全ての地上の状態で status 254・距離 0（無効）を返し、TAKEOFF に入った後も 186 ms と 344 ms 無効のまま、距離 0.031〜0.032 m の最初の有効サンプルに至る（有効な距離の最小は 0.031 m、無効サンプルは距離を持たない）。飛行中の無効の最長連続は 298 ms。旧い規則（TAKEOFF 突入から 0.5 s 数える）は、離陸の遅い機体（ホバー推力の見積もり 10 % 低で 0.575 s）で誤って成立し自動離陸を打ち切った。新しい規則は TAKEOFF の数え始めを「上昇 5 cm（加速度積分の推定。`vertical_velocity_estimator.hpp` の `riseM()`、`controller_status.vertical_rise_m`）または突入から 2 s」にし、数え始めてからの継続を 0.15 s に短くした。**短くした理由（SILS で測定）:** ToF が床の上で無効のまま ARM した場合（`tof_invalid_alt_hover` の 二つ目の離陸）、固定された推定が自動離陸を全速の上昇のままにするので、判定までの継続時間が上昇量になる。継続 0.5 s だと真値の最大高度は 0.39 m（ノイズなし）、ノイズ n2 の seed 1〜3 で 0.99・1.06・1.24 m（高度上限の 1.2 m に届く。事故の再現）、継続 0.15 s（採用）では 0.20 m、0.38〜0.41 m。掃引の全表は `flight_limits.hpp`。有効なサンプルが届くと継続判定は解けるが、取り込み直しが済むまでエピソードは続く。DISARM で継続判定は解除する。飛行中に成立した判定は着陸の間も保つ。**地上で ToF が無効のまま ARM は受理される**（地上の無効は正常のため）が、自動離陸は上昇 5 cm の後 0.15 s で打ち切られ、真値の最大高度は 0.20 m（ノイズなし。ノイズ n2 とホバー推力の誤差 ±20 % で 0.15〜0.41 m。SILS `tof_invalid_alt_hover`）。**SILS の不感帯:** SILS の VL53 チップモデルに近距離の不感帯（真の距離 31 mm 未満で反射パルス無し = 無効）を入れた（`vl53_device.cpp` の `MIN_MM`。実機の最初の有効距離 0.031 m による）。SILS の機体モデルは床の 21 mm 上で静止するので、実機と同じく地上で無効を返し、10 mm 上がると有効になる。既定で有効で、シナリオのコマンド `tof_deadzone 0` で切れる。これで「地上の無効を一致と数える」ARM の経路と、TAKEOFF の最初の無効の連続（SILS では TAKEOFF 突入の 395 ms 後に最初の有効サンプル、そのときの上昇量の推定は 0.011〜0.015 m）が全シナリオで通る。
  6. **古い failsafe_status:** 上の判定の実装の項（`FailsafeStatusGuard`）。
  7. **FLIP 中に始まったエピソード:** 上の §3.1 の FLIP 行（`notifyFlipComplete()` がその場で判断する）。
  数値（0.10 m・10 個・1 s・0.5 s・1 s の鮮度、離陸の 5 cm・2 s・0.15 s、着陸の 0.10 m）はすべて `sf::limits`（`flight_limits.hpp`）に名前付きで置いた**候補値**。**未確認:** 実機（無効サンプルの出方、各候補値の妥当性、離陸直後に ToF の不感帯を抜ける時間。SILS の全離陸シナリオで偽検出は無いが、実機のスプール時間・不感帯は別。実機の床での加速度積分の推定の上昇量。FLIP から戻った直後の無効の出方）。
- **ToF なし降下の上限の改修（2026-11-09。実機ログと SILS で確認した欠陥の修正。ToF なしの降下を「上昇できない」形にした）:** 欠陥は 3 つ。(a) 加速度積分の鉛直速度は、最後のリセットからの時間とともにずれる（上の項目 3）。(b) 速度ループの推力上限が 1.3 × max(支持推力, 見積もり) だったので、推定が「下降しすぎ」側にずれると上昇した（SILS: 加速度計のオフセット 0.1 / 0.15 / 0.2 m/s² で再上昇し 1.43 / 3.38（1.56 m まで上昇）/ 5.06 m/s（2.69 m まで上昇）で接地、床に柔らかく接地した後に PI が推力を上げて自分で再離陸し 1.21 m まで上昇した例も）。(c) 手動モード（ACRO / STABILIZE）で ToF が止まったまま長く飛んだ後に通信断・電池の非常で LANDING に入ると、積分が何十秒も続いた推定で速度ループが始まる。**変更:** 1) 推力は常に 支持推力 × `kNoTofThrustCapRatio`（0.95）以下（hover 推力の見積もりは上限の基準に使わない）。2) 支持推力の較正: 離陸の上昇中の「見積もりへ戻す」経路をやめ、床を離れた後のホバー近傍のサンプルの窓で 推力 × g ÷ 読み を求める（窓が無いときだけ、低くした見積もりと空中の全サンプルの商 × 0.85 の小さいほう）。3) 速度推定の信用時間 `kVerticalVelocityTrustS`（2 s）を超えたら PI をやめて上限の一定推力。4) 接地に「床が機体を支えている」経路を足し、ToF 出力停止中の LANDING での IMPACT（3 G）は非常でなく接地（`ToFSilentImpact`）。5) 高度上限の実効化: 自動離陸・ALT_HOLD・POS_HOLD で、推定高度が `kAltitudeCeilingM + kAltitudeCeilingMarginM`（1.35 m）を超えたら上昇の指令を禁止して目標を上限へ戻し（`limitClimbAboveCeiling`）、上昇中は推力を見積もりの 0.5 倍以下にする（`brakeAboveCeiling`）。ToF なし降下から ToF 生存の閉ループ着陸へ戻るとき `alt_vel_` を初期化する（0.92 m → 1.78 m の再上昇の再現）。6) 時間切れは LANDING に入った時刻から数える（`LandingClock`）。**根拠の数値:** 実機ログ 3 本での較正（`front_tof_poshold_yaw360` の 5.6 s のホバー: 2 s 窓の平均は長い平均の ±1.0 %、0.3 s 窓は ±4.9 %。窓の再生で較正済みになるのは離陸の開始の 4.1 s 後で +0.7 %、その後 −1.1〜+1.5 %）、読みの平均 − g は実機 2 本で −0.05 / −0.09 m/s²（支持推力は 0.5〜0.9 % 高い側）、SILS での比の掃引（0.95 / 0.97 / 0.98 / 0.99 / 1.00: 1.00 で 0.05 m 再上昇、ノイズなしの ALT_HOLD 上昇 + 離陸からの +0.2 m/s² で 0.97 が 0.07 m、0.98 が 0.54 m 再上昇）。**評価（`TEST_MATRIX.md` の ToF なし降下の上限の改修）:** 6 場面 × 加速度計バイアス 5 通り（ToF 停止と同時 / 離陸開始から）× `hover.thrust_corr` 0.9 / 1.0 / 1.2 × ノイズ off / n0 / n2（3 seed）の 1134 本のうち、場面が成立した 951 本で、空中でのモータ停止 0、降下が始まった後の再上昇 0（最大 0.002 m）、床からの再離陸 0、接地しなかった 0。**未達:** ノイズなし / n0 の接地速度が 1.2 m ホバーで 0.79〜1.04 m/s、上昇初期で最大 1.22 m/s、ノイズ n2 で最大 2.08 m/s（目標の 0.8 m/s は 0.5 m ホバーの一部とバイアスが正の場合だけ）。上昇初期の 2 場面で LANDING 以後の余分な上昇が 0.10〜0.22 m（制動距離を引いた後。目標 0.1 m）。実機は未確認。 **積分飽和対策の変更後の再測定（2026-11-09）:** `BlindDescent` の PI に、同じ周期で効く片側の積分飽和対策（制限の奥へ押す増分だけをその周期で取り消し、戻る向きの増分は残す。以前は前の周期の推力で次の周期の積分を両方向とも止めていた）を入れた後、同じ 1134 本を測り直した: 場面が成立したのは同じ 951 本、四つのゼロ（空中停止・再上昇・床からの再離陸・未接地）は変わらず（再上昇の最大 0.002 m）、接地速度は最大 +0.04 m/s の範囲で動いた（制限にかかった間も積分が戻れるので、降下がわずかに速い）。既存の SILS 全件では ToF なしの降下を使う 15 本ほどの接地速度の指標（descent_speed_max）が −0.002〜+0.018 m/s 動き、合否は変わらなかった。
- **INV 照合とリップル確認（ToF なし降下の閉ループ化。制御則・離着陸に関わる変更）:** INV-1（変えたのは Landing フェーズの鉛直チャネルの `thrust` だけで、姿勢・レートのパイプラインは不変）、INV-2（リンク途絶時の水平化判定は不変）、INV-3（接地の「検出」は `TakeoffLandingMgr`、「判断」は StateManager と `system_status.landing` のまま）に適合。旧い前提（開ループの時間関数と、1.4 g × 4 サンプルの接地）を埋め込んでいた既存の部品を列挙して確認した: (a) `PidController` の `blind_landing_active_` / `blind_start_thrust_` / `blind_elapsed_s_`（削除して `BlindDescent` に一本化。`reset()`・`onLanding()`・ToF 復帰の経路は `blind_descent_.stop()`）、(b) `blind_landing.hpp` の `blindLandingDropFrac` / `blindLandingThrustAt`（削除。後半の低下の係数 `blindLandingLateFactor` だけ残した。並列の実装は残っていない）、(c) StateManager の時間切れ `decideToFSilenceAction`（定数 `kBlindLandingTimeoutUs` だけを参照。根拠文を「8.33 s でゼロ」に更新）、(d) ImuTask の `feedBlindTouchdown`（引数を |加速度| だけから、推力比・鉛直速度込みに変更）、(e) Failsafe の IMPACT（不変。通常は静止の判定より先に働く、上記）、(f) ホスト試験 `test_main`・`test_tof_silence` の旧規則の試験、(g) `.expect` のログ行と metric。
- **INV 照合とリップル確認（高度上限と無効サンプルの継続の改修）:** 制御則に関わる変更は 2 つ。(1) 高度上限をスティック・rc 誘導の上昇に適用: ALT_HOLD / POS_HOLD の鉛直チャネルの上昇率指令だけを変え、姿勢・レートのパイロット操縦には触れない（INV-1: 上昇率の決定を `altitudeHoldClimbRate()` の 1 か所に集約し、スティックと rc 誘導で並列の経路を持たない）。API の rc の上限判定（`api_task.cpp`）は同じ定数を共用する（二重の定義なし）。(2) ToF 健全性トラッカーの入力を、二値の `flying` からフェーズ（`ToFFlightPhase`）に変えた: 旧い前提（TAKEOFF・FLYING 以外では無効サンプルは正常）を埋め込んでいた既存の部品を列挙して確認した: `ImuTask::feedToFHealth`（唯一の呼び出し側）、ホスト試験 `test_tof_health`（`flying` の bool を使う旧い試験は `Rig::run()` が写す）、`failsafe_status.tof_reason` の `InvalidStuck` の文言（「while flying」のまま。LANDING・TAKEOFF の継続も同じ理由コード）。並列の判定経路は残っていない。
- **未解決（高度上限と無効サンプルの継続の改修で見つけたもの。設計の判断が要る）→ (1) と (3) は 2026-11-09 に解決:** (1) **【解決。不感帯での接地、注7 の第 3 の経路】** 近距離の不感帯を入れた SILS では、柔らかい接地（3 G 未満）の ToF による通常の着陸が終わらなかった（`landing_dead_zone.scn`）。降下の最後の数 cm でセンサが無効になり（読みは真値に遅れ、真の高さ 0.038 m で 0.085 m）、`on_ground_`（5 cm 未満の有効な読みが 2 回）が立たず、鉛直速度の推定がずれて降下停滞の判定（`|vz| < 0.05 m/s`）も成り立たなかった。持ち主の決定に従い、LANDING の降下指令中、近傍帯（`kLandingDeadZoneBandM` = 0.10 m）以下の有効な読みの後に無効サンプルだけが `kLandingDeadZoneHoldMs`（0.2 s）続いたら接地とみなす（`TakeoffLandingMgr::deadZoneTouchdown()`。速度の条件は無い。帯より上の有効な読みと着陸降下の終了で取り消し、帯の中の有効な読みで数え直す）。ToF なし着陸の接地検出器とは別の経路で、ToF が動作している着陸だけに効く。0.2 s の根拠は実機ログ（`front_tof_poshold_yaw360`、0.28 m/s）で、帯の中で有効な読みの間に挟まった途切れが無効 1 サンプル（有効な読みの間隔 66 ms）だけだったことの約 3 倍。結果: `landing_dead_zone` は PASS（最後の有効な読み 0.085 m の 0.2 s 後に検出、そのとき機体は床の上で静止、真の高さ 0.021 m・鉛直速度 0.001 m/s）。`alt_disarm_land.scn` は引き続き不感帯を切って DISARM の振り分けだけを検証する。**実機ログの再生**（`TakeoffLandingMgr` に ToF サンプルと飛行状態を時刻どおり与えた）: `front_tof_poshold_yaw360` の着陸の降下中（277.6〜280.2 s）には作動せず（誤検出なし）、接触の後、最後の有効な読み 0.031 m の 0.23 s 後（280.63 s）に作動する（この実機の着陸は、3 G の衝撃カットが 280.23 s に先に終わらせた）。3 本のログ全体で、TAKEOFF・FLYING・LANDING の間ずっと着陸降下が指令されていたと仮定しても（誤作動の最悪ケース）、不感帯の経路は一度も作動しなかった（`yaw4x90` は ToF が全サンプル無効、`flip_test2` は帯の中の有効な読みの後に無効が続く区間が DISARM の後だけ）。実機での通常の着陸での確認は未。 (2) **【今回の改修の対象外】** 高度上限 1.2 m と宙返りの最低高度（`flip.min_height_m` 既定 1.0 m）が近く、上限のある ALT_HOLD / POS_HOLD で宙返りに十分な余裕のある高さを取れない。宙返りは今回の改修の対象外（持ち主の決定）で、宙返りの開発を再開するときに判断する。`flip.min_height_m` の既定値は変えない。(3) **【解決】** TEST_MATRIX.md の ToF なしの降下の表は、ToF なし降下の上限の改修で不感帯と高度上限を含むビルドで測り直した（表は `TEST_MATRIX.md`）。
- **高度上限（同日、`sf::limits::kAltitudeCeilingM` = 1.2 m、候補値）:** 自動離陸の目標（0.5 m、`static_assert` で上限以下を保証）と API の高度入力（`up`/`go` 等は上限へクランプして `ok` を返す、`rc` の上昇は上限で 0 に落とす）に効く。ToF が有効な距離（約 1.6 m）の中に収め、ToF なしの着陸の時間を抑える。**ALT_HOLD・POS_HOLD のスティック上昇と rc 速度誘導（`guide_vz_`）にも同じ上限を適用した**（高度上限と無効サンプルの継続の改修。`PidController::altitudeHoldClimbRate()`。API の rc と同じ `kAltitudeCeilingM` を共用）: 高度が上限以上なら上昇率は 0、高度目標 `alt_setpoint_` は上限でクランプする（上限を超えた機体は戻される）。**上限の手前では上昇率を「上限までの距離 × 高度ループのゲイン」（0.45 1/s × 距離）に絞る**（単純に切ると、全速の上昇が上限で切れて行き過ぎ、上限のまわりで揺れた。SILS で 1.13〜1.44 m）。結果（SILS `alt_ceiling_stick`、スティックを 9 s 上げ続ける、真値）: 1.17〜1.18 m で止まり、スティックを離すと 1.13〜1.17 m を保ち、スティックを下げれば降下できる。上限が無いと同じスティックで 1.97 m まで上昇した。ノイズ n2（`alt_ceiling_stick_noise`、seed 1〜3）では真値の最大 1.185〜1.187 m、推定の最大 1.21〜1.22 m（保持は 0.95 m まで下がることがある）。高度上限と無効サンプルの継続の改修の指示にあった「n2 の推定の偏りで 1.32 m 程度」は、この方式では再現しなかった（上昇を絞るので行き過ぎない）。ACRO・STABILIZE の手動飛行には上限は無い（高度ループを持たない）。**波及:** 上限の導入で、ALT_HOLD のスティックでは 1.2 m を超えられなくなった。これに頼っていた SILS シナリオを直した（`tof_silent_alt_climb` は上昇の最中に出力停止する形に、`tof_silent_hover_1p2` は上昇に 5 s、`rc_flip`・`rc_flip_latch` は `flip.min_height_m` を 0.8 / 0.75 m にして押下の高さを下げ、`alt_inflight_switch`・`alt_recenter_gate` は下限を下げた。理由は各 `.scn` / `.expect`）。実機の宙返りは 1.41 m・1.70 m で始めたので、ALT_HOLD からは宙返りの最低高度 `flip.min_height_m`（既定 1.0 m）が上限 1.2 m の近くになる: 上限のある ALT_HOLD / POS_HOLD で高く上げて宙返りを始める運用は成り立たない（上限と宙返りの高度の関係は、宙返りの開発を再開するときに判断する。上の「未解決」の (2)）。以前は手動の上昇中の出力停止の最悪ケースを `tof_silent_alt_climb`（1.55 m を 0.57 m/s で上昇中に停止。最大 1.79 m、接地速度 0.47 m/s）で測ったが、上限の導入で ALT_HOLD ではその高さに届かない。今は上昇の最中（約 0.8 m、約 0.3 m/s）で止める: 最大 0.92 m、接地速度 0.40 m/s。
- **較正前の支持推力などの改修（評価は `TEST_MATRIX.md`）:**
  1. **自動離陸の行き過ぎ（真値の最高 1.96 m、ToF の有効範囲外で LANDING、2.51 m/s で接地）:** 再現: `hover.thrust_corr` 1.2・ノイズ n2 で 0.5 m の離陸が 1.96 m まで上がる。原因は 3 つ重なっていた。(i) ESKF の鉛直速度は ToF の引き継ぎ高さ 0.15 m まで 0 に固定され（引き継ぎの後の約 1 s も誤る）、TAKEOFF の速度ループは上がっている機体を見ず、指令推力が重さの 1.27 倍のまま 1.2 m/s まで上昇した。(ii) 速度ループの補正は見積もりのまわりの ±0.15 N に制限されるので、見積もりが真値の 1.45 倍だと推力は重さの 0.95 倍までしか下がらない。(iii) 高度上限の制動は推定高度が 1.35 m を超えてから、しかも見積もりの 0.5 倍の推力で、上限を突き抜けた。**変更（鉛直チャネルのみ、INV-1）:** (a) `PidController::takeoffClimbRate()`: 速度ループの上昇率に、加速度積分（`vertical_velocity_`）が示し ESKF が示さない上昇のうち `kTakeoffVelocityDisagreeMps`（0.6 m/s）を超えた分を足す（連続な超過分で、切り替えではない）。余裕は、普通の離陸（最初の 1 s で積分と ESKF が最大 0.5 m/s 違う）を ESKF の信号のままにするため（余裕なしの版は `alt_recenter_gate` の `alt_band` 0.193 → 0.211 と、接触が 3 G 未満という前提の `landing_dead_zone` を変えた）。(b) `brakeAboveCeiling()`: 制動の開始を「止まる高さの予測」で判定する: 高度 + v × 作動の遅れ（`kSupportThrustLagS` 0.08 s）+ v² ÷ (2 a) が `kAltitudeCeilingM + kAltitudeCeilingBrakeStartM`（1.2 + 0.05 m）を超える間（a = `kAltitudeCeilingBrakeDecelDerate` 0.7 × g × (1 − 0.5)、v = ESKF の上昇率と加速度計に基づく上昇率の大きいほう。**天井ブレーキの改修で変更: 加速度計に基づく上昇率を使うのは自動離陸の上昇と ESKF の鉛直状態を信用できない間だけ。信用できる間は ESKF の上昇率**）。制動の推力は「支持推力」（`supportReferenceThrust()` = 較正済みの支持推力、較正前は早い推定）の `kAltitudeCeilingBrakeFrac`（0.5）倍で、ホバー推力の見積もりの 0.5 倍ではない。厳しい側（`limitClimbAboveCeiling`、1.35 m）は変えない。(c) 推力の上限を設ける案（上昇率が目標を超えたら支持推力の割合で上限を下げる）も実装して測ったが、最高高度は最大 0.03 m しか変わらなかったので採用しなかった。**結果（`hover.thrust_corr` 0.9 / 1.0 / 1.2 × n0 / n2（3 seed）、ALT_HOLD・POS_HOLD の自動離陸と API 離陸）:** 真値の最高は最悪 2.03 m → 1.27 m（全条件で 1.35 m 以下）、n0・corr 1.2 で 1.45 → 1.26 m。
  2. **較正前の ToF なし降下が速すぎる:** 較正前の支持推力は min(早い推定, 見積もり × 0.75) で、実機ログの再生では早い推定が 0.75 × 見積もりより大きく、上限は機体を支える推力の 0.63〜0.71 倍（0.29〜0.37 g で落ちる）。実機ログで較正済みになるのは浮上から 5.75 s（yaw360）、`yaw4x90` は 3.45 s 浮上しても較正されず、SILS は約 2.5 s で較正されるため差が見えにくかった。**変更:** `blindLandingStartThrust()` は、空中のサンプルが `kSupportClimbMinS`（0.15 s）たまったら早い推定をそのまま使い（上は見積もりの 1.5 倍）、見積もりの 0.75 倍で切らない。補正 `kSupportClimbDerate` を 0.85 → 0.92 にした。**根拠（実機ログ 3 本の再生と SILS の掃引。`TEST_MATRIX.md`）:** 較正前の「上限 ÷ 機体を支える推力」は、旧: front_tof_poshold_yaw360 0.69〜0.71、yaw4x90 0.63、flip_test2 0.79〜0.91 → 新（0.92）: 0.87〜0.90、0.86、0.85〜0.98（目標 0.85 以上で、重さ未満）。補正の比較（3 本の最小〜最大）: 0.85: 0.79〜0.91、0.90: 0.83〜0.96、0.92: 0.85〜0.98、0.94: 0.87〜1.00（flip_test2 で重さに届く: 不採用）。SILS（出力停止を ARM の 0.9〜4.5 s 後に置く: 離陸直後の出力停止と短いホバー、`hover.thrust_corr` 0.9 / 1.0 / 1.2、n0 / n2、加速度計のオフセット ±0.2 m/s²）: 上限 ÷ 真の重さは 0.86〜0.97、再上昇は最大 0.007 m、上昇初期の接地速度は 1.93 → 1.43 m/s（n0）、2.67 → 2.01 m/s（n2）。
  3. **弱いスプール（床の上で FLYING・ToF なし着陸になる）【2026-11-10 に改めた: 下の (a)(b)(c) の仕組みは取り下げ、(d) だけ残る。「床の積極的な証拠への切り替え」参照】:** 再現: STABILIZE でモータ健全度 0.9、スロットル 3100（離陸判定は超えるがホバー推力未満）で、TAKEOFF の 2 s + 0.15 s の後に無効サンプルが数えられて ToF 出力停止のエピソードになり、接地フラグが「空中」になって（`Ground contact unknown`）床の上で `Takeoff complete → FLYING`。ALT_HOLD の自動離陸でも、床の上で `TAKEOFF → LANDING` に入る。**変更（検出は ImuTask・制御器、判断は StateTask: INV-3）:** (a) 無効サンプルを数える TAKEOFF の時間の上限（`kTofInvalidTakeoffMaxUs` 2 s）は、機体が床を離れた証拠（`SupportThrustFilter::leftFloor()`: 健全な ToF が空中、上昇量の推定 0.10 m、または上向きの読みが g を 1 m/s² 超）を必須とした（`ControllerStatus.left_floor`、`ToFSampleFacts.left_floor`）。上昇量の推定が 5 cm に達する経路は従来どおり。(b) 手動（ACRO / STABILIZE）の TAKEOFF → FLYING は、ToF 出力停止のエピソード中は上昇量の推定が `kManualTakeoffMinRiseM`（5 cm）以上を条件とする（`manualTakeoffComplete()`）。(c) 自動離陸の時間切れの備え（3 s）は、機体が床を離れた証拠（`craftIsOffTheFloor()`）がなければ離陸完了を宣言しない（浮くまで試し続けるのでなく、離陸の開始から 5 s 証拠が無ければ DISARM して中止する。上の TAKEOFF → IDLE_GROUND 行）。(d) 支持推力の窓と鉛直速度の推定は、手動の TAKEOFF（Grounded フェーズ）でも、指令推力が hover の 0.8 倍以上なら動く（`restingOnGround()`）。新シナリオ `weak_spool_stabilize`・`weak_spool_alt`（`.expect` 付き。main（2026-11-08 時点）では不合格）。**注意:** 手動の TAKEOFF は、浮けないままなら終わらない（パイロットが DISARM する）。自動離陸は 5 s で中止する。以前は 2.15 s で FLYING に入り、出力停止を警告するだけだった。
  4. **軽微:** (a) 時間切れは「LANDING の経過 ≥ 10 s」かつ「ToF の連続した出力停止 ≥ `kBlindLandingMinSilentUs`（5.5 s）」（`LandingClock`）: ToF が動作したまま 10 s を超えた LANDING の後に止まっても、空中で即座にモータを止めない。(b) STABILIZE・ToF 出力停止中の通信断で、推定速度が上向きなら下限推力で減速する案は**不採用**: 21 s の出力停止の後の推定の誤差は −2〜−5 m/s（加速度計のオフセット +0.1〜+0.2 m/s²）で信号（屋内で 1 m/s 前後）より大きく、効くのは誤差が上昇の速さより小さいときだけ。誤差が上向きのとき（約半分）は、ホバー中の機体の接地速度が 1.0 → 2.2 m/s に上がる（1 次元の力学モデル。`TEST_MATRIX.md`）。(c) `operation_manual.md` の英語版に「待機中のコマンド」を追加し、ToF なし降下を「目標 0.4 m/s の速度ループ」でなく「推力の上限による一定の下向き加速度（約 0.05 g）の降下」と書いた（本書の項目 3 も）。(d) パラメータ `safety.blind_touchdown.accel_g`・`floor_s` の変更時は推定器と制御器の両方へ再読込を通知する。(e) 制御器のログの残り信用時間は `blind_gains_.trust_s` から出す。(f) 高度上限の根拠文を現行の降下に合わせた。(g) `TOF_SILENT` のコメント、`tof_silent_stabilize.scn` の LED 色（黄）、`state_manager.cpp` のコメントの語を「最後の値のまま固定され」に直した。(h) `StateManager::transition`・`DataStream::appendEntries`・`processEstimatorCommands` は補助関数（`resetModeOnGround`・`appendAbsoluteSensorEntries`・`inflateEstimatorCovariance`）に出して元の行数以下（78 → 56、162 → 137、68 → 53）に、`arbitrateFlightMode` の入れ子は 2 段にした。(i) 宙返りは今回の対象外（持ち主の決定）: 上の「未解決」(2) を書き換え、`flip.min_height_m` の既定値は変えない。
  5. **INV 照合とリップル確認（`left_floor`・手動 TAKEOFF の上昇量・`craftIsOffTheFloor()` に関する部分は後に取り下げた）:** INV-1: 自動離陸の行き過ぎの抑えは鉛直チャネルの推力だけ（`takeoffClimbRate`・`brakeAboveCeiling`）で、姿勢・レートのパイロット操縦は不変。INV-3: 「床を離れた証拠」の検出は制御器（`SupportThrustFilter`、上昇量は `VerticalVelocityEstimator`）で、`controller_status.left_floor` と `vertical_rise_m`（上向きの加速を示した後の上昇量）に出し、ImuTask の追跡器と StateTask の判断（手動離陸の完了 `manualTakeoffComplete()`、自動離陸の中止 `updateAutoTakeoff()`）が読む。INV-4: ToF 出力停止の規範表の LANDING 行に「出力停止の連続時間」の入力を足した（`decideToFSilenceAction` の引数、ホスト試験 `test_tof_silence`）。旧い前提を埋め込んでいた既存の部品を列挙して確認した: (a) `blindLandingStartThrust` の 0.75 の切り捨て（ホスト試験 `support_without_a_window_is_the_lowered_estimate` と新規 `an_early_estimate_gives_a_ceiling_between_85_percent_and_the_weight` に更新）、(b) `brakeAboveCeiling` の 2 つの呼び出し側（TakeoffClimb・Airborne。どちらも新しい判定）、(c) `system_status.airborne` の読み手（`control_task` の `state.airborne` は不変。手動 TAKEOFF の完了だけが上昇量を要求する）、(d) `LandingClock::update`・`decideToFSilenceAction` の呼び出し側 3 か所（`StateManager` 2、ホスト試験）、(e) `ToFSampleFacts` の位置指定の初期化（`left_floor` は末尾に追加し、`test_tof_health` を更新）、(f) 手動 TAKEOFF で支持推力の窓が早く動き始める影響（較正が離陸の開始から進む。SILS の STABILIZE の場面は変わらない）。
- **天井ブレーキの改修【この改修のうち、天井ブレーキの上昇率と `alignToEskf()` は残る。上向き加速の証拠、加速度計に基づく中止、workshop の `ControllerStatus` 発行は取り下げた。「床の積極的な証拠への切り替え」参照】（評価は `TEST_MATRIX.md`）:** 天井ブレーキが `max(ESKF, 加速度積分)` を上昇率に使ったため、長い ToF 出力停止の後（積分にオフセット誤差が丸ごと残る）に ALT_HOLD へ入ると、上昇していない機体で推力が重さの半分に落ち、床に衝突した（SILS `ceiling_brake_stale_velocity`: オフセット −0.4 m/s² で積分 3.7 m/s、1.7 m/s で衝突。`ceiling_brake_stale_velocity_alt`: 1.238 m で入って 0.59 m まで落下。実機ログでは正常な POS_HOLD 中も積分と ESKF が最大 0.52 m/s 食い違う）。**変更:** (a) 積分をブレーキが使うのは、自動離陸の上昇（静止から数え直した直後）と、ESKF の鉛直状態を信用できない間（ToF 出力停止、または地上で鉛直状態が固定）だけ（`ceilingBrakeClimbRate()`）。(b) ESKF を信用できる間は、従来と同じく ESKF 自身の上昇率で決める。(c) ESKF が ToF を取り込み直した周期に、積分を ESKF の速度へ一度に合わせる（`VerticalVelocityEstimator::alignToEskf()`。エピソードの誤差を捨て、上昇量には触れない）。床を離れた証拠が加速度のオフセット誤差だけで成立した（SILS で床の上に TAKEOFF → LANDING・FLYING。評価表の注入 ±0.3 m/s²、実測の誤差は 0.05〜0.09 m/s²）。**変更:** 上昇量の推定 `riseM()`（支持推力の窓・ToF 健全性トラッカー・手動離陸の完了が使う）は、ろ波した（読み − g − オフセット）が `kVerticalVelocityLiftEvidenceMps2`（0.4 m/s²、パラメータ `safety.vvel.lift_mps2`）に自分の遅い基準値（時定数 `kVerticalVelocityLiftBaselineTauS` 1 s。床の上のオフセット誤差に従う）を超えて達する「上向きの加速」を示した後だけ、その時点から（加速度 − 基準値）の 2 重積分を数える（それまでは 0。証拠が立った後のオフセットのドリフトは上昇量に溜まらない）。0.4 は 2 つの掃引の間で決めた（床の上の誤成立: 以前の規則は 56 本中 16 本、基準値つきの規則は 0.3 で 2 本、0.4 で 1 本、0.5 で 0 本。ToF が使えない自動離陸 `tof_invalid_alt_hover`: 0.4 で 0.32 m で打ち切り、0.5 では認識されず 4.3 m まで上昇）。**両側とも余裕が薄いのが未解決の点（設計上の指摘）:** 床の上の余裕は評価表のオフセット（0.3）に対し 0.1 m/s²、ToF が使えない自動離陸の側の余裕は 0.05 m/s² で、同じ量（加速度計の上向きの加速）を使う限り両立の余地が小さい。ToF が使えない自動離陸が上昇し続けないための、加速度計に依らない独立の備え（気圧計など）が望ましい。証拠が立つまでの遅れは最大 0.4 s 程度（掃引）。代償: 加速度が基準値を 0.4 m/s² 超えない緩やかな上昇（ToF が使えないとき）はこの証拠では認識されない（健全な ToF の空中判定・g を超える上向きの読み 1 m/s² は残る。未較正の支持推力は低い見積もりで安全側）。このため `tof_recover_invalid_first` / `tof_recover_reflip` / `tof_invalid_alt_hover` を、証拠が立つ離陸（開始のスロットルを強める）か、閾値の窓に直した（`TEST_MATRIX.md`）。浮けない自動離陸（ALT/POS/API）が終わらなかった（推力が hover + 補正の上限のまま回り続け、API `takeoff` は 12 s で error を返してもモータは止まらない）。**変更:** TAKEOFF の開始から `kAutoTakeoffNoLiftAbortUs`（5 s）経っても `controller_status.left_floor` が立たなければ、StateTask（`updateAutoTakeoff()`）が DISARM する（上の TAKEOFF → IDLE_GROUND 行）。「ARMED_GROUND へ戻す」にしないのは、ALT/POS の ARMED_GROUND がスプールドウェルの後で自動離陸をまた始め、繰り返すため。手動の TAKEOFF は変えない。新シナリオ `weak_spool_api`（`error takeoff aborted`）、`weak_spool_alt`（期待値を更新）、`weak_spool_alt_offset`・`weak_spool_stabilize_offset`（オフセット付きの変種）。workshop は `controller_status` を誰も発行せず、`left_floor` が常に偽・`vertical_rise_m` が常に 0 だったので、TAKEOFF の無効サンプルの数え始めが閉じ、の中止が ALT/POS の TAKEOFF のたびに働く形になった。**変更:** workshop の制御タスクが最小限の `ControllerStatus`（`left_floor` = 健全な ToF が空中、`vertical_rise_m` = ToF が健全な間の高さ、出力停止中はどちらも安全側の偽・0）を毎周期発行する（`publishControllerStatus()`）。workshop は高度モードを使わず、モータは学習者のものなので、出力停止中の手動 TAKEOFF が FLYING にならない挙動（TAKEOFF のまま）は安全側である。(M) `takeoff_elapsed_cycles_`（uint16、163.8 s で一周）を上限で止めた。`kTakeoffVelocityDisagreeM` を `...Mps` に改名、`kManualTakeoffMinRiseM = kTofInvalidTakeoffRiseM`（定義は 1 つ）。`kAltitudeCeilingBrakeDecelDerate` と `kBlindLandingMinSilentUs` の根拠に SILS の数値を添えた。`imu_task.cpp` の `controller_status.latest()` は 1 回だけ読む。`StateManager::transition`（56 → 33 行、`enterState()` を分離）、`processEstimatorCommands`（53 → 48 行、`reloadEstimatorParams()` を分離）、`DataStream::appendEntries`（137 → 8 行、`append400HzEntries()`・`appendFlightPhase400Entry()`・`appendFlightFlagsEntry()`・`appendPilotAndReferenceEntries()` に分離。パケット内の順序は変えない）を 50 行以内にした。**INV 照合とリップル確認:** INV-1: 天井ブレーキの変更は鉛直チャネルの推力だけ。INV-3: 自動離陸の中止の判断は StateTask、検出（`left_floor`・`riseM()`）は制御器・推定器。INV-4: 規範表に TAKEOFF → IDLE_GROUND の行を足し、手動 TAKEOFF → FLYING の条件を更新した。`riseM()` の読み手 3 か所（`SupportThrustFilter::update`、`feedToFHealth`〔ToF 健全性トラッカー〕、`manualTakeoffComplete`）は同じ定義（上向きの加速の後の上昇量）を共有し、二重化していない。
- **床の積極的な証拠への切り替え（評価は `TEST_MATRIX.md`）:** **根本原因:** 「床を離れた」の判定を加速度計だけ（上向き加速の証拠 0.4 m/s² の後から数える上昇量、`left_floor`）で作り、それを安全判断の判定条件にした。ゆっくり浮く機体では証拠が立たず、逆に床の上でも推力の立ち上がりと同時に入るオフセット（0.6 m/s² 以上）で誤成立する。(1) ToF が使えない自動離陸（健全度 0.9〜0.97、`tof_invalid_alt_hover` 型）で無効サンプルを数えず 3.8〜7.6 m まで上昇し、5 s 後に「床を離れていない」として空中で DISARM した。(2) STABILIZE の手動離陸で ToF 出力停止すると TAKEOFF から抜けず、通信喪失の自動着陸が永久に保留され、STABILIZE の離陸（スロットル 3095）で ToF が止まった機体に通信断を足した SILS では、2.76 m まで上昇して空中でモータが止まり 7.3 m/s で衝突した。(3) 制御器を差し替えると `leftFloor()` が常に偽で、5 s 後に必ず DISARM する。**設計判断の原則: 証拠が「無い」ことを、床にいる証拠として扱わない。加速度計だけの判定を安全判断の判定条件にしない。** **変更:** ToF 健全性トラッカーの TAKEOFF 中の無効サンプル計数の時間の備え（2 s）から `left_floor` の条件を外し、以前の動作に戻した（5 cm 上昇の経路は残る。どちらも判定を「早める」向きにしか働かない）。床の上で ToF なしの LANDING に入っても、降下の推力は支持推力の 0.95 倍以下なので浮かない（以前の弱いスプールの ALT 側の仕組みは取り下げ。SILS `weak_spool_alt`・`weak_spool_api`）。手動（STABILIZE / ACRO）の TAKEOFF → FLYING は従来に戻した（ToF 出力停止で接地が「不明 → 空中」になったら FLYING。床の上で FLYING 表示になりうるのは許容する。`operation_manual.md` に書いた。SILS `weak_spool_stabilize`）。通信喪失の猶予（`kCommLossHoverUs` 3 s）が過ぎたら、TAKEOFF からも LANDING にする（`decideCommLossLanding()`。自動・手動とも。SILS `commloss_takeoff`）。浮けない自動離陸の中止（5 s）で DISARM してよいのは、健全な ToF が「床の近くを示し、増えていない」という積極的な証拠（有効な読みが `kTakeoffGroundBandM` 5 cm 以下で、`kAutoTakeoffFloorRiseM` 2 cm を超えて増えず、`kAutoTakeoffFloorHoldUs` 1 s 続く。`TakeoffFloorWatch`）があるときだけにした。この証拠が立っている間は、制御器が到達と言っても TAKEOFF に留める。ToF の読みが増えている（ゆっくり浮く）機体は中止しない。ToF が使えなければ ToF 無効の規則で LANDING になる（SILS `weak_spool_alt_floor`・`weak_spool_alt_slow_lift`）。加速度計の上向きの証拠（`kVerticalVelocityLiftEvidenceMps2`・`kVerticalVelocityLiftBaselineTauS`・`VerticalVelocityConfig::lift_evidence_mps2`・パラメータ `safety.vvel.lift_mps2`・`VerticalVelocityEstimator::liftSeen()`）、`ControllerStatus::left_floor`、`IController::leftFloor()`、`ToFSampleFacts::left_floor`、workshop の `ControllerStatus` 発行、`kManualTakeoffMinRiseM`、`craftIsOffTheFloor()`、`manualTakeoffComplete()`、`updateAutoTakeoff()` を削除した。`riseM()` は元の意味（地上で最後に静止してからの上昇量）に戻り、残る読み手は 2 つで、どちらも誤っても害の無い向きにだけ使う（ToF 健全性トラッカーは無効サンプルを「早く」数え、支持推力の窓は「早く」開いて支持推力が「低く」しか出ない）。`SupportThrustFilter::leftFloor()` は窓を開く内部の判定として残るが、安全判断の判定条件ではない（コメントに書いた）。`vertical_rise_m` はフライトログ（`tof_silence`、0x4E）の列として残る（プロトコルの互換）。入力を強めた `tof_recover_invalid_first`・`tof_recover_reflip`（3171 → 3250）と `tof_invalid_alt_hover` の閾値（0.30 → 0.40）は元に戻して合格した。TAKEOFF の完了・中止・通信喪失の判断を純関数 `takeoff_policy.hpp`（`decideTakeoff()`・`decideCommLossLanding()`）へ移し、ホスト試験 `test/test_takeoff_policy.cpp`（12 本。手動 × airborne、自動 × 到達 × 床の証拠 × 経過時間、全 `FlightState` × 猶予、`TakeoffFloorWatch`）で規範表の全セルを検査する。床の証拠の検出は ImuTask（`tof_health.floor_stays` として発行）、判断の実行は StateTask・StateManager（INV-3）。`architecture.md` の INV-4 を合わせた。API の `land` の後の再上昇: 原因は、ToF 生存中の着陸の速度ループが hover 推力の「見積もり」から始まり、積分が遅いため、見積もりが高い（`hover.thrust_corr` の変更の後の古い見積もり、電圧が下がった電池）と推力が数秒間重さを上回ることだった（`hover.thrust_corr` 1.05 / 1.1 / 1.15 で 0.08〜0.13 / 0.17〜0.24 / 0.27〜0.32 m（seed 1〜3）。1.2 は離陸の行き過ぎで比較できない）。支持推力が較正されていれば、ループの基準を支持推力の `kLandingBaseOverSupportMax`（1.08）倍までに抑える（`landingBaseThrust()`）。LANDING 以後の上昇は 0.00〜0.09 m（以前は 0.65 m まで）。1.05 倍は `landing_dead_zone` の柔らかい接触（約 0.2 m/s）を 3.2 G の衝撃（0.27 m/s）に変えたので採らない。自動離陸の行き過ぎ: 速度ループに足す積分の超過分の余裕 `kTakeoffVelocityDisagreeMps`（0.6 m/s）を 0.45 / 0.3 / 0.2 / 0.1 に掃引（120 本）しても、全条件の真値の最高は 0.6 / 0.45 / 0.3 / 0.2 / 0.1 で 1.296 / 1.295 / 1.311 / 1.314 / 1.303 m と改善せず（行き過ぎを決めるのは天井ブレーキ）、1.0 の n2 は 1.14 → 1.06 m、0.9 の n2 は 0.82 → 0.77 m と小さな改善だけなので、採らない（0.6 のまま）。`kBlindLandingMinSilentUs` を 4 s → 5.5 s に変えた（測った最長の降下 4.7 s を 0.8 s の余裕で覆い、後半の低下が推力をゼロにする 8.33 s より手前）。天井ブレーキの断続作動: 1.2 m のホバーと離陸の行き過ぎで、ブレーキが深さの半分より深くなった回数が 1 回の実行あたり最大 136 回あった（ノイズで上昇率の符号が変わるたび）ので、止まる高さの予測の超過に応じて上限が直線的に変わる形にした（`kAltitudeCeilingBrakeRampM` 0.1 m。最大 3 回、真値のピークの最悪 1.288 → 1.296 m）。「天井でのホバーには触れない」の文書は、「予測が 1.25 m 未満なら触れない。それより上では比例して働く」に直した。軽微: 説明文と定数の割り込み・「上昇量」の説明の食い違いは、上昇の証拠を消したので解消した。`publishControllerStatus` は 50 行、`operation_manual.md` の言い切りは ToF の返し方による場合分けに直した。**INV 照合とリップル確認:** INV-1: LANDING の再上昇の抑えと天井ブレーキの断続作動の変更は鉛直チャネルの推力だけ。INV-3: 床の証拠の検出は ImuTask、判断は純関数、実行は StateTask・StateManager。INV-4: 規範表の TAKEOFF → FLYING（手動・自動）・TAKEOFF → IDLE_GROUND・通信断の行を更新し、全セルをホスト試験で検査する。`riseM()` の読み手 2 か所（`SupportThrustFilter::update`、`feedToFHealth`）は同じ定義を共有し、二重化していない。
- **ToF なし降下の上限の値、LANDING の再上昇、TAKEOFF の終わり方、LANDING の接地判定（評価は `TEST_MATRIX.md`）:** (1) **較正前の ToF なし降下の上限 `kSupportClimbDerate` を 0.92 → 0.91 にした。** 0.92 では、加速度計のオフセット +0.6 m/s²・`hover.thrust_corr` 1.2〜1.3・ノイズなし・離陸の 0.8〜0.9 s 後の出力停止の組で、早い推定が真の重さの 1.14 倍（上に凸な推力とオフセットが足し合わさる）になり、上限が重さの 0.996 倍になって機体は 5 s に後半の低下が始まるまで 0.6〜0.7 m にホバーし、その後 2.02 m/s で接地した（main（2026-11-08 時点）は 0.89 m/s）。出力停止の 340 本（API 離陸後に出力停止する 140 本、離陸直後に出力停止する 200 本）で値を掃引すると、main（2026-11-08 時点）より悪い組は 0.92 で 4、0.915 で 2、0.91・0.905・0.90 で 0。0.91 は悪い組を全て消す最小の値で、0.91 では、LANDING を経て床に接触した 571 本のうち、床からの再離陸（0.10 m 超）は 0 本（最高 0.047 m。0.92 では 1 本が床から 0.38 m の高さへ再離陸した）。**代償:** 強い接地（1.5 m/s 超）が離陸直後の 200 本で 18 → 33 本、ToF なしの評価表の成立 870 本で 38 → 67 本（増え幅の最大 +0.5 m/s）、実機ログの「上限 ÷ 機体を支えた推力」の最小が 0.851 → 0.842。この増加は気圧計の統合（次の作業）で下げる予定で、値では下げない。`test_blind_landing` の下限は 0.85 → 0.84。推力のゆっくりした低下（開始から毎秒 0.03 / 0.05）を足す案も悪い組を消したが、離陸直後の 200 本の強い接地が 65 / 94 本になり採らなかった。(2) **LANDING の再上昇:** ToF が動作している閉ループ着陸の基準推力を、較正の前も支持推力の基準の 1.08 倍までに抑え（`landingBaseThrust()`）、上昇している間は推力を支持推力の基準の割合以下に制限した（`brakeRiseInLanding()`。上昇率は ESKF の値だけ）。`hover.thrust_corr` 1.2・ノイズ n2 で LANDING が 1.19 m で始まり 1.98 m まで上昇、ToF の範囲外から 2.20 m/s（真値の鉛直速度の、床への最初の接触の瞬間＝高さ 3 cm 未満の値）で接地していたのが、再上昇 0・降下速度の最大 0.38 m/s になった（API 離陸の 6 s 後の `land` の 120 組: 強い接地（1.5 m/s 超）9 → 0 本、再上昇 0.10 m 超 46 → 0 本、終わらない LANDING 21 → 0 本。接地速度の最大 0.70 m/s。比較は main（2026-11-08 時点））。`closedLoopLandingThrust()` から地面効果アシストを `landingSettleCeiling()` に分けた（関数を 50 行以内に）。(3) **床の証拠のラッチと、TAKEOFF の終わり方:** 中止は「今」の 1 s の証拠（`floor_stays`）を要するが、完了は「その瞬間に証拠が無い」だけを要していたので、時間切れの後に帯の外へ外れた 1 つの有効な読みで床の上で FLYING になった。証拠が一度立ったら `seen()`（`tof_health.floor_seen`）にラッチし、機体が床を離れた証拠（5 cm を超える有効な読みが 1 s 続く、または 15 cm を超える有効な読みが 1 つ）が来るまで完了を止める。ラッチが解けず、今の証拠も無い（読みが 5 cm の帯をまたいで交互に来て、どの区間も 1 s に届かない）まま 5 s たったら、TAKEOFF を LANDING で終わらせる（`TakeoffAction::Land`）。ただし帯より上の有効な読みの区間が進行中（`floor_leaving`。ゆっくり浮く機体が離れつつある途中）なら LANDING にしない（`weak_spool_alt_slow_lift` が保つ）。規範表に「TAKEOFF の完了とラッチのセル」を足し、ホスト試験 `test_takeoff_policy` は 24 本（全セル、毎サンプル・5・15・29・31・32 サンプルごとに帯をまたぐ読みの順序を含む）。(4) **TAKEOFF 中の着陸要求:** API `land` は TAKEOFF では LANDING にしない。`StateManager::notifyLandingRequest()` は FLYING（と高度モードの ARMED_GROUND の即時 DISARM）だけを受け付け、TAKEOFF に届いた `land` は ApiTask が離陸の終わりまで保留して、離陸が終わってから実行する（`takeoff` の待機中なら後回しの行、待機の外なら `cmdLand` が待つ。注13）。TAKEOFF から LANDING に入る道は、通信途絶・ToF の規則・離陸の中止の判断（`TakeoffAction::Land`、`StateManager::notifyTakeoffEndedByLanding()`）だけ。**パイロットの DISARM も TAKEOFF では LANDING にせず、どのモードでも即モータ停止**とした（注5）。理由は、床の上から始めた LANDING が機体を浮かせたため（TAKEOFF → LANDING 行の評価）。TAKEOFF 中の `land` をすぐ LANDING にする案も評価して採らなかった: LANDING の接地判定に床の読みを要する条件、加速度計による接地検出器を ToF が動作している LANDING でも働かせる条件、ToF 無効の規則の例外を次々に足す必要が生じ、それでも ToF が動作している LANDING の上空 1.1 m で加速度計の「衝撃の後に静止」が成立してモータが止まる組（SILS 384 本のうち 10 本、接地 4.5 m/s）と、加速度計のオフセットが +1.0 m/s² 以上のとき床から始まる LANDING が終わらない組が残った。(5) **LANDING の突入時に残った着陸の判定:** `TakeoffLandingMgr::noteLandingDescent()` と `SystemStatus.verdict_state_since_us`（注7）。(6) **TAKEOFF の経過の時計** を ImuTask（`takeoffElapsedUs`）と StateTask（`takeoff_since_us`）の二重から、StateManager が状態に入るときに刻む `SystemMode.state_since_us` の 1 つにした。(7) **床の証拠のラッチの抜け:** 帯の中の有効な読みの後に、無効サンプルだけが保持時間（1 s）続くと、`floor_stays` は時計で真になるのにラッチは偽のままで、次の帯の外の 1 つの読みで TAKEOFF が完了した。`TakeoffFloorWatch` は、新しいサンプルごとに（無効でも）、保持時間が過ぎていればラッチする（`latchIfStays()`）。ホスト試験 `test_takeoff_policy` に 1 本足した。(8) **ARMED_GROUND の `land`:** 高度モードだけ即時 DISARM、手動モードは拒否、後回しの行を捨てる（注13）。**INV 照合とリップル確認:** INV-1: LANDING の推力の変更（基準の上限、上昇ブレーキ）は鉛直チャネルだけ。INV-3: 床の証拠の検出（`floor_stays`・`floor_seen`・`floor_leaving`）は ImuTask、判断は純関数 `decideTakeoff()`・`landingVerdictCounts()`、実行は StateTask・StateManager。API は `api_command` の発行と `system_mode` の観測だけ。INV-4: 規範表に TAKEOFF → LANDING の行、ARMED_GROUND → IDLE_GROUND の行（高度モードと手動モード）、TAKEOFF の完了とラッチのセルの表を足し、ホスト試験とシナリオで検査する。旧い前提を埋め込んでいた部品を列挙して確認した: (a) `cmdLand`（TAKEOFF を空中として受け付けるが、TAKEOFF では StateManager が無視する。ARMED_GROUND は高度モードだけ）、(b) `dispatchDuringWait`（待機中の `land` を、高度モードの ARMED_GROUND の間だけ即実行にした。TAKEOFF の間は後回し）、(c) `requestDisarm()` の呼び出し側（ARMED_GROUND・パイロット・API・姿勢不一致・浮けない離陸の中止）は変更なし（TAKEOFF の DISARM は従来どおりモータ停止）、(d) `SystemStatus` の読み手（`state_manager.cpp` の ARM 前判定、`control_task`、`data_stream`、`emu_flightlog_vehicle`）は新しい項目を読まない（`landing` の中身は変わらない）、(e) 「ESKF の鉛直状態が地上に固定されたまま」という前提を埋め込む LANDING 側の部品は `brakeRiseInLanding()`（ESKF の上昇率が 0 に見える）で、床の上からは LANDING に入る道が上の 3 つだけで、離陸の推力で浮かなかった機体は着陸の推力でも浮かない（注8 (3)）。**採らなかった案:** 較正前の上限に推力のゆっくりした低下（開始から毎秒 0.03 / 0.05）を足す案は、悪い組を消したが、離陸直後の 200 本の強い接地が 65 / 94 本になった。判定のやり直しを「床を離れたことのある機体」に限る案は、LANDING の長さが変わらなかった（注7）。上昇ブレーキに上昇率の不感帯を入れる案は、上昇が増えず接地速度が下がる値が無かった（注8 (4)）。床にいる間に始まった LANDING の推力に上限を置く案は、必要な入り口が無くなったので採らなかった（注8 (3)）。**未確認:** 実機はいずれも未確認。LANDING が上昇中（鉛直速度 0.05 m/s 以上）に始まったときの慣性による上昇（最大 0.47 m、SILS）は、TAKEOFF 中の API `land` を後回しにしたので、通信途絶と ToF の規則から入る場合だけが残り、測っていない。0.91 での強い接地の増加が実機でも同じ程度か。`floor_leaving` の実機の ToF での挙動（実機の床は ToF が無効を返すので、この経路そのものが立たない）。
- **既知の取引（直していない。数値は SILS の真値、評価は `TEST_MATRIX.md`）:** (a) **較正前の着陸基準推力の上限**（`landingBaseThrust()` の `kLandingBaseOverSupportMax` × 支持推力の基準）は、加速度計のバイアスが負で大きいとき、接地速度を増やす: バイアス −1.5 m/s² で最大 0.95 m/s（main〔2026-11-08 時点〕は 0.45 m/s）、−0.6 m/s² で 0.65 対 0.44 m/s。`hover.thrust_corr` 1.2 では 0.93 対 3.43 m/s で、今の方が良い。(b) **ToF が 0.05〜0.15 m のヒステリシス帯（上下で別のしきい値を持つ帯）にある間は ESKF の鉛直状態が地上に固定される**ため、床の近くで始まった LANDING は、上昇中でも降下停滞と判定されて 0.09〜0.16 m で止まることがある（main でも同じ種類で、件数は同じ）。対策の方向は、固定されない速度の利用で、気圧計の高度を取り入れる作業（B3）で扱う。
- **検証（高度上限と無効サンプルの継続の改修）:** host ユニットテスト `test_tof_health`（`classifyInvalidSample()` の全フェーズ、TAKEOFF の上昇量・時間の条件と短い継続、LANDING・FLIP の無効サンプルは一致でも継続でもない、継続の後の最初の有効サンプルの取り込み直し、CLI 表示用の鮮度判定）、`test_blind_landing`（上昇量の推定）、SILS 新規 `alt_ceiling_stick`・`alt_ceiling_stick_noise`（上限）、`tof_recover_landing_invalid`、`landing_dead_zone`（高度上限と無効サンプルの継続の改修では既知失敗。2026-11-09 に不感帯での接地で PASS）、既存の `tof_invalid_alt_hover`・`alt_disarm_land`・`rc_flip`・`rc_flip_latch`・`tof_silent_alt_climb`・`tof_silent_hover_1p2`・`alt_inflight_switch`・`alt_recenter_gate` を直した。
- **検証（ToF なし降下の閉ループ化まで）:** host ユニットテスト `test_tof_health`（取り込み直しの判定・無効の継続・古い failsafe_status・飛行中のエッジ消費）と `test_eskf_flip_hold`（`reacquireVertical`）と `test_main`（`markAirborne`）、SILS 新規 `tof_recover_frozen_ground`・`tof_recover_edge_consumed`・`tof_recover_climb_alt`・`tof_recover_reflip`・`tof_recover_invalid_first`・`tof_recover_prearm`・`tof_invalid_alt_hover`・`tof_invalid_stabilize`・`tof_silent_flip_end`（故障注入 `tof_invalid`）。以下は最初の対策（出力停止の判定を状態機械に入れた改修）の検証: host ユニットテスト `test_tof_silence`（しきい値境界・時刻逆転・uint32 巻き戻り・36 分超の出力停止・未到達・範囲外は出力停止でない・全状態 × 全モードのセル・ACRO/STABILIZE は警告のみ・高度モードの切り替え拒否）、`test_main`（ToF なし接地の配線・ラッチの寿命・閾値設定）、`test_blind_landing`（後半の低下、速度推定器の 3 規則・オフセット学習・気圧計のインタフェース、1 次元プラントでの降下ループ、接地検出器）。SILS（すべて `simulator/sils/scenarios/`）: `tof_silent_takeoff`（上昇初期の出力停止）・`tof_silent_api_takeoff`（API `takeoff` の事故の連鎖。`error takeoff aborted`、続く二つ目の `takeoff` は `error tof silent - check the sensor`）・`tof_silent_hover`（0.5 m）・`tof_silent_hover_1p2`（1.2 m）・`tof_silent_alt_climb`（手動 ALT_HOLD で 1.55 m を上昇中）・`tof_silent_hover_timed`（衝撃カットと接地検出器を無効にし時間切れの経路）・`tof_silent_takeoff_est_high`・`tof_silent_api_takeoff_est_high`（見積もりが 28 % 高い上昇初期の出力停止）・`tof_silent_takeoff_noise`（ノイズ n2）・`tof_silent_touchdown_detect`（衝撃カットを無効にして接地検出器だけで終える）・`tof_silent_stabilize`（強制着陸しない・着陸後の ARM 拒否・飛行中の POS 切り替え拒否）・`tof_silent_mode_refused`（地上のモード拒否と自動適用）・`tof_silent_prearm`・`tof_silent_armed_ground`（故障注入 `tof_silent`、チップモデルが「データ準備完了」を返さない）。同じ `tof_silent_takeoff` を対策なしで実行すると真値高度が 42 m まで上昇した（事故の再現）。**SILS の実測（真値、2026-11-09 の ToF なし降下の閉ループ化、80 本: 5 場面 × `hover.thrust_corr` 0.9〜1.2 × ノイズなし / n2 の 3 seed）:** 表は `simulator/sils/scenarios/TEST_MATRIX.md`。要約（今のファームウェア、ノイズなし）: 出力停止時からの上昇増分は 0.5 m / 1.2 m ホバー 0.00 m、手動 ALT_HOLD 上昇 0.22〜0.24 m、上昇初期の出力停止 0.03〜0.60 m（`hover.thrust_corr` 0.9〜1.2。増分の大半は ToF 停止から LANDING までの 0.32 s の間の上昇 = 上昇速度 × 0.32 s で、LANDING 以後の増分は 0.01〜0.27 m）、接地速度は全 20 本で 0.25〜0.49 m/s、出力停止から接地まで 0.5〜4.4 s。ノイズ n2 の最悪: 増分 1.18 m（API 離陸、`hover.thrust_corr` 1.2、LANDING の時点で 2.2 m/s で上昇中）、接地速度 0.96 m/s（1.2 m ホバー、`hover.thrust_corr` 1.2。ノイズで ToF の有効範囲を超えて 2.3 m まで行き過ぎた飛行から）。main（2026-11-08 時点）は同じ 80 本で上昇初期の出力停止が最大 47 m、空中でのモータ停止が 8 本、今のファームウェアは 0 本。比較: ToF が動作している通常の自動着陸（1.15 m から）は接地速度 0.30 m/s で 4.5 s。**既知の限界（未解決）:** 実機未検証。実機の hover 推力の誤差の大きさ・向き、実機の加速度計のオフセットと、振動による読みの偏り（SILS の n2 では標準偏差 0.8 m/s² の雑音、ノイズなしでは 0）、機体の接触の柔らかさ（接地検出器の閾値）、実機の ToF 出力停止の頻度は未確認。上昇初期の出力停止の増分には、ToF 停止から LANDING までの 0.32 s（`kTofSilenceUs` 0.3 s + Failsafe の 10 Hz）の間の上昇が含まれ、この検出遅れはこの改修の対象外（短くすれば増分は比例して減る）。

**注13（ApiTask の待機中の `emergency` / `stop` — 2026-11-09）:** `takeoff`・`land`・移動・`flip` は `waitUntil`（`tasks/api_task.cpp`）で条件成立までタスクをブロックする。従来は待機中にソケットを読まないため、その間に届いた `emergency` が待機の終わりまで（最長で離陸 12 s・着陸 20 s）効かなかった（自動離陸の事故で、まさに待機中に止めたい場面）。対策は **待機ループの中でも同じコマンド源（SILS 注入リング、UDP）を読む** こと:

- 周期: 述語の評価と同じ 50 ms（従来の周期を維持。最悪の待ち時間は 50 ms ＋処理）。待機していないときの主ループは 20 ms。
- `emergency`: `cmdEmergency()`（processLine と同じ関数）で即実行し（モータを止めるのが先）、**後回しの行を全て捨てて**、それぞれに `error cancelled by emergency` を返し、待機を打ち切る。待っていたコマンドは `error interrupted by emergency` を返す。応答の順序は「`emergency` の `ok`、後回しの行のエラー、打ち切られたコマンドのエラー」。捨てる理由: 後回しの行は emergency より前に出されたもので、再生すると emergency の直後に `takeoff` が実行され再離陸しうる。
- `stop`: `cmdStop()`（同じ関数）で即実行する。受理（FLYING かつ誘導目標あり）なら後回しの行を捨てて（`error cancelled by stop`）待機を打ち切り（`error interrupted by stop`）、拒否なら拒否の応答だけ返して待機は続ける（`takeoff` / `land` / `flip` の待機中は拒否される。何も止めていないので後回しの行も残す）。`cmdStop()` は位置に加えて向き（ヨー）も現在値へ置き直す: 回頭中の `stop` で古い目標ヨーが機体を回し続けないため。
- `land`（機体が ARMED_GROUND の間で、**高度モード（ALT_HOLD / POS_HOLD）のとき**。`landOnFloorDuringWait()`）: 待っている `takeoff` はまだ床の上でモータが回り始めたところ（スプール 0.3 s）なので、降りる高さが無い。**即時 DISARM を要求**（`g_target_valid` を落として `ApiCmd::Land` を発行。`StateManager::notifyLandingRequest()` は、ARMED_GROUND の高度モードで `requestDisarm()` を呼ぶ）し、機体が IDLE_GROUND になったことを確かめてから（`waitLandOnFloorConfirmed()`、最長 `kLandConfirmMs` = 0.5 s）待機を打ち切る。待っていた `takeoff` は `error cancelled by land` を返す。`land` の行は後回しにせず、その場で `ok` を返す（`emergency` と同じ。モータが止まってから。後回しにすると、機体は既に DISARM 済みで `error not flying` になるため）。**それまでに後回しにした行は、`emergency` と同じく全て捨てて、それぞれに `error cancelled by land` を返す**: `land` より前に出された行で、後回しの `takeoff` が残ると、DISARM の直後にまた ARM する（SILS で、`takeoff`・`takeoff`・`land` の順に出すと DISARM の後に `takeoff` が再生された）。待機の外の `land` も、高度モードの ARMED_GROUND なら同じく即時 DISARM（`cmdLand` が受け付け、IDLE_GROUND を待って `ok`。状態とモードは `system_mode` の 1 回の読みで判定する）。**手動モード（ACRO / STABILIZE）の ARMED_GROUND の `land` は従来どおり拒否する**（`error not flying`。待機中なら後回しの行になる）: スロットルをパイロットが握り、ARMED_GROUND からでも浮ける（スロットル 0.499 で足りる）ので、床の上とは限らない。高度モードで `ApiTask` が ARMED_GROUND を見た直後に状態が TAKEOFF へ移る競合では、要求は StateManager が無視する（TAKEOFF では受け付けない）。そのため `ApiTask` は発行の後に IDLE_GROUND になったことを確かめてから `ok` を返し、TAKEOFF に移っていたら `ok` を返さず、TAKEOFF を抜けるまで待ってから `land` を発行し直す。待機の中なら、その `land` を後回しの行にして待機を続け、待っていた `takeoff` は離陸の結果どおりに応答し、後回しの `land` は離陸の後に実行する。この競合そのもの（窓は数 ms）は SILS で再現できず未確認（要求の発行を省いて競合に負けた場合を模擬すると、後回しの経路が `takeoff` の `ok`、続けて `land` の LANDING → IDLE_GROUND と `ok` で動くところまでは確かめた）。**既知の問題（直していない）:** 手動モードの ARMED_GROUND は、スロットルだけで浮ける。検証: SILS `api_land_in_armed_ground`（高度モード。後回しの `takeoff`・`height?` を捨てる）、`api_land_manual_armed_ground`（手動モードの拒否）。
- `land`（機体が TAKEOFF の間、または `takeoff` の待機に入った直後で状態がまだ IDLE_GROUND の短い間）: 他の行と同じく**後回し**にし、`takeoff` の応答の後に普通の `land` として実行する。離陸が完了していれば（FLYING）LANDING に入り、接地で `ok` を返す。離陸が中止されていれば、機体はもう飛んでいないので `error not flying` を返す。**待機の外で TAKEOFF に届いた `land`**（例: RC の ARM が始めた自動離陸の最中。待っているコマンドは無く、主ループが読む）は、`cmdLand` が TAKEOFF を抜けるまで（最長 `kTakeoffTimeoutMs` = 12 s）要求を保留してから、同じ規則で扱う: FLYING で終われば LANDING を発行して接地で `ok`、LANDING で終われば（通信途絶・ToF の規則など）その着陸の終わりを待って `ok`、地上で終われば `error not flying`。以前は TAKEOFF のまま `ApiCmd::Land` を発行して無視され、`error land timeout`（20 s 後）になった。**TAKEOFF の間の着陸要求にはしない**: `StateManager::notifyLandingRequest()` は TAKEOFF を無視する（床の上で始めた LANDING は機体を浮かせうる。注5）。検証: SILS `api_land_deferred_in_takeoff`（`land` は離陸の完了後に実行され、離陸の最高高度は変わらない）、`api_land_in_takeoff`（浮けない機体: 後回しの `land` は中止の後に `error not flying`）、`api_land_in_rc_takeoff`（RC が始めた自動離陸の最中の `land`: TAKEOFF を抜けてから LANDING → IDLE_GROUND、`ok`。修正前のビルドでは `error land timeout` で不合格）。
- `rc`: 待機中に届いた `rc` 行は後回しにせず**その場で捨てる**（応答なし）。待機が終わった時点で古い操作入力であり、再生すると古いスティックで機体を操縦する。
- それ以外の行（上の TAKEOFF の間の `land` を除く）: 捨てず後回し（最大 8 行、`kDeferredSlots`）にし、待機していたコマンドの応答後に主ループが到着順に実行する（従来はソケット/注入リングで待っていたのと同じ順序）。満杯なら `error busy`。
- **返信先は行ごと:** 各行の返信は「その行の送信元」へ送る（`ReplyTarget`）。待機中に別の送信元から行が届いても、待機していたコマンドの最終応答はその元の送信元へ届く（従来は最新の送信元に上書きされていた）。後回しの行・取り消した行・待機中に即実行した `emergency` / `stop` も、それぞれ自分の送信元へ返信する。
- 状態機械・Pub-Sub の変更はない（`api_command` の発行と `system_mode` の観測のみ。INV-3: 判断と実行は StateManager）。

検証: SILS `api_takeoff_emergency`・`api_land_emergency`（注入からモータ停止まで 40 ms、修正前は待機の残り 0.76 s / 1.52 s。後回しの `takeoff` / `height?` / `left 100` / `rc` を捨てて再離陸しないこと）・`api_land_stop`・`api_move_stop`（後回しの `left 50` を捨てること）・`api_wait_deferred`。既存の SILS 51 本は結果が変わらない。**未確認:** 実機での遅延。UDP 経路（SILS のソケットシムはデータを届けない）。したがって行ごとの返信先（`ReplyTarget`）は SILS でも試験できていない。**対象外（未対応）:** `waitUntil` を使わない待機 — `sysid` コマンド内の固定待ち（`vTaskDelay`、最大 10 s ＋ 1 s）。`autotune` の掃引中に受理された `stop` は掃引の待機を打ち切る（`error autotune aborted ...`）が、制御器側の励振は設定時間（最大 2.5 s）続く。

## 4. 制御インターフェース定義

PID/MPC/LQRを差し替え可能にするための統一インターフェース。

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

`sf_controller_pid` の PID 実装は **双線形変換（Tustin 法）** を採用する。後退差分による離散化は α = Td/(η·Td+dt) > 1 となり微分フィルタが発散する（旧 SILS のプロトタイピング時に判明、コミット `06b4cd6`／git 履歴）。これを避けるため bilinear に統一した（コミット `07005fb`、vehicle/ 旧実装と一致）。

| 項 | 離散化 |
|----|--------|
| 積分（trapezoidal）| `integral += (kp/ti) · (error + prev_error) · dt/2` |
| 積分の保持 | `integration_enabled=false` で積分値を保持（リセットせず蓄積しない、P/D は通常）。`Grounded` の間の姿勢/レート PID に適用（注11）|
| 微分（bilinear）| α = 2·η·Td/dt、a = (α−1)/(α+1)、b = 2·Td/(dt·(α+1))<br>`d[n] = a·d[n−1] + b·(e[n] − e[n−1])`<br>Kp は filter 外で適用: `d_term = kp · d[n]` |

η = 0.125、Td = 0.01、dt = 0.0025 で α = 1.0、a = 0、b = 4.0 の安定動作。定量的な検証は、新しい物理真値の判定 **G1〜G4**（`../../../simulator/sils/RESET_PLAN.md` §4）で再取得する（旧 SILS での L1〜L4 数値は削除済み・git 履歴に保存）。

### ヘディングホールド（STABILIZE 以上）

レートループはヨー「角速度」しか帰還しないため、定在ヨー外乱トルク下では方位が操縦修正の合間にランダムウォークする（2026-06-11 実機で確認: 推力比例 CW モーメント下、手放し方位ずれ平均 12.3°）。これを推定ヨー角（バイアス補正済みジャイロ積分 — 地磁気不要）への外側 P ループで抑える。

| 項目 | 内容 |
|------|------|
| 係合 | ヨースティック中立（\|yaw\| < 0.03）かつ空中（STABILIZE: throttle > 0.25 / ALT_HOLD 以上: VerticalPhase == Airborne）|
| 目標捕捉 | 係合エッジで現在の推定ヨー角を捕捉。スティック入力で即解除（パイロット優先）、次の中立で再捕捉 |
| 制御則 | `rate_ref_yaw = clamp(kp · wrap(ψ_target − ψ), ±rate_max)` — 誘導ヨー則と同形 |
| 除外 | ACRO（純レート制御）、誘導アクティブ中（誘導がヨー所有）、自動離陸中、墜落リセット・モード切替で解除 |
| パラメータ | `attitude.yawhold.kp`（既定 3.0 [1/s]、0 で無効）、`attitude.yawhold.rate_max`（既定 2.0 [rad/s]）|
| 検証 | フライトログ再生で方位ずれ 12.3°→5.7°（平均）。SILS 判定 `yaw_hold.scn`: M1 80% 故障下 8 秒手放しで yaw_band < 1.5°（無効化対照 3.1° 単調流出）|

## 5. 状態推定インターフェース定義

ESKF/EKF/Complementary Filterを差し替え可能にするための統一インターフェース。

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

`sf_estimator_eskf` には次の機能が実装されている（χ² 判定（カイ二乗判定）は実機 vehicle/ の知見、Adaptive R・線形化バイアスは旧 SILS のプロトタイピング由来＝git 履歴。新 SILS での定量検証は G1〜G4 で行う）。

#### χ² 判定（外れ値棄却）

各観測更新前にイノベーション e と共分散 S = H·P·Hᵀ + R から χ² = eᵀ·S⁻¹·e を計算し、自由度に応じた閾値を超える観測を棄却する。実機 vehicle/ で「姿勢の揺れが PID ゲイン変更なしでほぼ解消」した実績あり（加速度計の外れ値が `updateAccelAttitude` で姿勢を乱していたのを抑制）。コミット `d9172c4`。

#### Adaptive R（加速度姿勢補正の thrust 汚染対策）

加速度計が観測する加速度は厳密には `[0, 0, −T/m]`（body frame）であり、ホバー以外では `R^T·g` から乖離する。15° ロールでは |a| ≈ g/cos(15°) で、ノルム判定は通過するが方向は誤りで、姿勢推定が破綻する。

対策として観測ノイズを **R_actual = R_base² × (1 + k_adaptive · |a − g|²)** で動的に膨らませ、g から外れた観測の重みを下げる。`k_adaptive = 10` を default とする（2026-09-16 修正: 本節は長らく `50` と記載していたが、実装 `sf_estimator_eskf/include/eskf_core.hpp:126` の値は `10` であり、コードと不一致だったため文書側を訂正した）。旧 SILS のプロトタイピング（0.3Hz サイン応答、git 履歴）での参考値:

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

HALコンポーネントはvehicle内にコピー（完全独立）。

```
firmware/vehicle/
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
│   ├── sf_estimator/              # 状態推定インターフェース＋推定器側の検出
│   │   ├── tilt_consistency.cpp   # 傾き整合モニタ（地上で姿勢推定 vs 重力、注10）
│   │   └── include/
│   │       ├── estimator.hpp      # IEstimator定義
│   │       └── tilt_consistency.hpp
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

タスクスタックの合計: 約102KB

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
| ApiTask | 6KB |
| TelloStateTask | 4KB |
| CLITask | 8KB |
| LogTask | 4KB |
| **合計** | **102KB** |

残り約410KBでPub-Subバッファ、ESKF行列、ヒープ等を賄う。

## 9. Guidance / Navigation 層の予約設計（v3）

### 設計の動機

vehicle は「学習者がレイヤーを段階的に登れる」ことを目標にしている（[`architecture.md`](architecture.md) §2.5「学習者の入口」）。学習段階の最終形は **ガイダンス（目的地・経路指定）→ ナビゲーション（経路計画）** 層であり、要件 (`requirements.md` §4) でも「ナビゲーター（将来追加）」として位置づけられている。

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

This document defines the detailed design of vehicle, based on the architecture design (architecture.md).

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

### Cells of the TAKEOFF completion and the latch (normative, 2026-11-10)

For an auto-takeoff in ALT_HOLD / POS_HOLD, after the controller has reported "reached": `decideTakeoff()` (`takeoff_policy.hpp`, a pure function; the host test `test_takeoff_policy` checks every cell). Evidence NOW = `floor_stays`, latch = `floor_seen`, "5 s" = `kAutoTakeoffNoLiftAbortUs` (time since the TAKEOFF began, as a difference from `SystemMode.state_since_us`).

| Latch (`floor_seen`) | Evidence NOW (`floor_stays`) / readings | Less than 5 s since the TAKEOFF began | 5 s or more |
|----------------------|------------------------------------------|---------------------------------------|-------------|
| not set | no evidence | complete (when the controller reports reached; hold otherwise) | the same |
| not set | only invalid samples for the hold time (1 s) after a reading in the band (the real floor reads invalid in the dead zone; `floor_stays` is true by the clock) | hold. **The latch is set the moment `floor_stays` becomes true** (`TakeoffFloorWatch` latches at every new sample, valid or not, once the hold time has passed), so one later reading outside the band cannot complete the takeoff | abort (while `floor_stays` is true; the latch is set, so the "set" rows apply afterwards) |
| set | readings at or below the band (5 cm), not rising, for 1 s (`floor_stays` true) | hold | abort (DISARM, the motors stop; `requestDisarm()`) |
| set | a run of valid readings between 5 cm and 15 cm is in progress (`floor_leaving` true: the craft is leaving the floor; after 1 s of it the latch is released and the takeoff completes) | hold | hold (a craft that lifts slowly is not landed in the middle of leaving the floor) |
| set | the in-band run is shorter than 1 s and no run above the band is in progress (one stray reading, or readings that straddle the edge of the band; `floor_stays` false, `floor_leaving` false) | hold (neither complete nor abort) | **LANDING** (`TakeoffAction::Land`, `StateManager::notifyTakeoffEndedByLanding()`; a craft that the takeoff thrust did not lift is not lifted by the landing thrust, note 8 (3)) |
| set | valid readings above the band (5 cm) for 1 s or more | **the latch is released, complete** | the same |
| set | one valid reading above 15 cm | **the latch is released, complete** (intended: if one stray reflection on the floor does it, the craft is then under the ordinary altitude control, which only climbs toward the target, and that does not stand in the way of ending a craft that cannot lift; its readings stay on the floor and the evidence is built again) | the same |
| set | only invalid samples (neutral: the run is neither ended nor extended; **the clock runs**, so `floor_stays` becomes true 1 s after a reading in the band) | hold (the ToF-invalid rule, note 12, makes a LANDING about 2 s + 0.15 s after the TAKEOFF began) | follows the last `floor_stays` (true: abort; false: LANDING), except that it holds while `floor_leaving` is true |
| set | the ToF is silent (`floor_stays` is treated as false) | hold (the silence rule, note 12, makes a LANDING after 0.3 s) | LANDING (note 12 acts first) |
| set | a valid reading in the band returns (the run above the band ends, the clock starts again) | hold | abort if the returned run has reached 1 s, otherwise LANDING |
| (manual ACRO / STABILIZE) | - | the latch is not read; complete when the ToF-based airborne verdict says so | the same |
| (any value) | an API `land` in TAKEOFF | not accepted (`notifyLandingRequest()` ignores TAKEOFF; ApiTask holds the `land` until the TAKEOFF has been left and runs it after the takeoff has ended: a `land` that arrives while a `takeoff` waits is a parked line, one that arrives outside a wait, e.g. during an auto-takeoff started by the RC ARM, is waited for by `cmdLand`); the decision of this table does not change | the same |

### Note 12: downward ToF silence (climb abort, blind landing, no forced landing in manual flight, ARM refusal; 2026-11-09)

Accident: the downward ToF stopped producing samples. The ground-contact flag is released by ToF samples only, so it froze "on the ground", the estimator kept pinning position/velocity to zero, and the auto-takeoff kept integrating thrust until the craft hit the ceiling. The countermeasure is rows of the normative state table (INV-4): the craft never flies blind in a mode that needs altitude, and a manual pilot is never taken out of the loop. Same form as the comm-loss failsafe: Failsafe DETECTS (R16), StateManager DECIDES and transitions.

| Item | Rule |
|------|------|
| Definition of "silent" (an EPISODE) | An episode begins when (a) no NEW sample arrives for `sf::limits::kTofSilenceUs` (0.3 s, candidate value; judged on the sample timestamp only) or (b) only INVALID samples arrive while the craft is in the air (a floor the ToF cannot range; the samples keep arriving, so (a) cannot see it). What counts, and for how long, depends on the phase (`classifyInvalidSample()`): FLYING `kTofInvalidPersistUs` (0.5 s, candidate value); TAKEOFF only after the craft has risen `kTofInvalidTakeoffRiseM` (5 cm, accelerometer integral) or `kTofInvalidTakeoffMaxUs` (2 s) after the TAKEOFF entry, and then `kTofInvalidTakeoffPersistUs` (0.15 s) of them (a real floor reads invalid for 186-344 ms after the TAKEOFF entry); LANDING only while the last valid height was at least `kTofInvalidLandingMinHeightM` (0.10 m); FLIP never. On the ground (IDLE_GROUND, ARMED_GROUND, DISARM) an invalid sample (below the minimum range) is normal and not counted. It ends ONLY when the estimator has RE-ACQUIRED the ToF (below): a sample that merely arrives proves nothing. A single out-of-range sample is not an episode. A timestamp of 0 (never received) counts as silent. Reason code `tof_reason`: NoSample, InvalidStuck, Reacquiring, StatusStale. |
| Judgement | `sf::SilenceWatch` (`sensor_freshness.hpp`): fed every cycle with (now, latest stamp). The age of a new sample is signed (a stamp a few us newer than now is fresh); the silent time saturates at `UINT32_MAX`, so a silence longer than 35.8 min / 71.6 min cannot turn back into "alive" (a bare 32-bit difference does). A gap of 35.8 min or more between observations, and a stamp that far ahead of now (also on the first observation), read as silent (saturated); gaps of 71.6 min or more cannot be detected, so every user observes every cycle in every flight state (the control task calls `IController::observeLinks()` before the arm gate, because `compute()` runs only while armed). A stamp of exactly 0 after a non-zero one is the 2^32 us wrap, an ordinary sample. The same watch judges COMM_LOST in the failsafe, the comm-loss hover grace in StateManager, the link liveness in the controller and `sensor_health`. |
| Detection and hand-over | `Failsafe::checkTofSilence()` (PowerTask, 10 Hz) publishes the LEVEL `failsafe_status.tof_silent` every update and raises the EVENT `AlertType::TOF_SILENT` (CRITICAL) while ARMED only: at the first armed update, then re-raised about once a second while silent (so an alert consumed during FLIP is not lost). Readers of the level: StateManager (pre-arm gate 3b, mode refusal, blind-landing time limit), ImuTask (ground-contact trust), ControlTask (injects `StateEstimate.tof_silent` into the controller), NotifyTask (LED), CLI. Besides the stamp, Failsafe uses the facts ImuTask publishes on `tof_health` (`altitude_reliable`, `invalid_stuck`), tracked by the pure `sf::ToFHealthTracker`. Every reader passes the level through its own `sf::FailsafeStatusGuard`: a status not refreshed for `kFailsafeStatusStaleUs` (1 s, candidate value) or never published reads as silent (reason StatusStale), so a stalled Failsafe can never leave the last "not silent" in force . |
| ARMED_GROUND | DISARM (never launch blind), any mode. |
| TAKEOFF, FLYING in ALT_HOLD / POS_HOLD (and the auto-takeoff, which exists only there) | LANDING (abort the climb); the descent is blind (below). |
| TAKEOFF in ALT_HOLD / POS_HOLD (auto-takeoff, ARM-triggered or API), `kAutoTakeoffNoLiftAbortUs` (5 s) after the TAKEOFF began, while the healthy ToF POSITIVELY shows the craft on the floor (a valid reading at or below `kTakeoffGroundBandM`, 5 cm, not rising by more than `kAutoTakeoffFloorRiseM`, 2 cm, for `kAutoTakeoffFloorHoldUs`, 1 s) | DISARM (TAKEOFF -> IDLE_GROUND, `Auto-takeoff aborted: the healthy ToF has shown the craft on the floor for 5000 ms`); the TAKEOFF is held until then even if the controller reports the takeoff complete (once the evidence has held it is LATCHED, `TakeoffFloorWatch::seen()` = `tof_health.floor_seen`, and keeps blocking the completion until the craft is shown to leave the floor: the abort needs 1 s of evidence NOW, so the completion needs as much, and one stray reading outside the band after the abort time no longer makes FLYING on the floor. The latch is released by the same weight of evidence the abort needs of staying: valid readings above `kTakeoffGroundBandM` (5 cm) for `kAutoTakeoffFloorHoldUs` (1 s), or one valid reading above `kTakeoffAirborneBandM`, 15 cm; a craft that hangs in the 5-15 cm ground effect therefore completes after 1 s instead of being held for ever; an in-band reading, a silent ToF or a reset ends the run above the band, an invalid sample is neutral). A latch that is not released and has no evidence NOW (the readings straddle the 5 cm band, no run reaches 1 s) ends the TAKEOFF by a LANDING from `kAutoTakeoffNoLiftAbortUs` on (`TakeoffAction::Land`; a craft that the takeoff thrust did not lift is not lifted by the landing thrust, note 8 (3)). The pilot's DISARM during the TAKEOFF is an immediate motor cut in every mode; the API `land` is not accepted in the TAKEOFF (`StateManager::notifyLandingRequest()` ignores TAKEOFF; ApiTask parks a `land` that arrives while a `takeoff` waits, note 13). No evidence is NOT floor evidence: an invalid sample, a silent ToF, a reading above the band or a rising reading (a slow lift) never aborts. A craft whose ToF reads invalid on the floor ends by the ToF-invalid rule (a ToF-silent LANDING, harmless: the descent thrust is below the support thrust). |
| TAKEOFF, FLYING in ACRO / STABILIZE | NO forced landing. Warning only: yellow fast blink on both LEDs, a tone on the first warning and about every 5 s, log `NO forced landing, fly on`. The pilot keeps flying; after landing, the next ARM is refused while the sensor is silent. A manual TAKEOFF becomes FLYING when the ToF-based airborne verdict says so; while an episode is in progress that verdict is "armed = not on the ground", so a craft that the floor still holds (weak motors, a ToF that reads invalid on the floor) may show FLYING on the floor after the 2 s + 0.15 s of the ToF-invalid rule: ACCEPTED, the pilot holds the throttle. |
| LANDING | Blind descent: the thrust is ALWAYS at most `kNoTofThrustCapRatio` (0.95) x the support thrust (the thrust that held the craft, calibrated by the accelerometer over near-hover samples after the craft left the floor), so the craft cannot climb whatever the velocity estimate says; while that estimate (an accelerometer integral) is within `kVerticalVelocityTrustS` (2 s) of its last reset a PI around the support thrust (target -0.4 m/s, but it cannot raise the thrust above the ceiling: it does not hold 0.4 m/s, it only stops a climb and starts the descent) works under that ceiling, after it the thrust is the constant ceiling. Either way the descent is in effect a descent at the constant downward acceleration set by the thrust ceiling (about 0.05 g), faster the longer it falls. The ToF landing detection is not trusted; the end is the touchdown detector: (a) an impact above `kBlindTouchdownAccelMps2` (2 g) for `kBlindTouchdownSamples` (2) consecutive IMU samples, then 0.25 s of stillness (not accelerating, commanded thrust below the support thrust, estimated vertical velocity within 0.15 m/s), OR (b) the floor holding the craft up: commanded thrust below 0.7 x the support thrust with the low-passed |accel| within 0.1 g of 1 g for 0.5 s (in the air that thrust reads below 0.7 g), no impact needed; or the time limit `kBlindLandingTimeoutUs` (10 s, counted from the entry into LANDING, AND the ToF continuously silent for `kBlindLandingMinSilentUs`, 5.5 s (4 s earlier; the longest measured descent is 4.7 s): a ToF that dies after a LANDING that already lasted more than 10 s gets a descent of at least 5.5 s before the motors are cut; the late fall brings the thrust to zero at 8.33 s of the blind descent from ANY velocity estimate, +1.67 s for the fall from 13 m). The 3 G impact detection (2 samples) stays active and normally fires FIRST (SILS contact 6-9 g; the real touchdown log peaks at 3.39 g): in a LANDING with a silent ToF it is the TOUCHDOWN (`Touchdown while the ToF is silent (impact)`, `StateManager::lastLandingEnd()` = `ToFSilentImpact`), not an emergency DISARM; GYRO_ANOMALY stays an emergency. |
| FLIP | Nothing (the maneuver is not interrupted). When FLIP returns to FLYING, `StateManager::notifyFlipComplete()` looks at the standing level AT ONCE and applies the FLYING row (log `ToF silence episode during the maneuver: decided now that FLIP is over`); it does not wait for the next re-raised alert (up to ~1 s of ALT_HOLD / POS_HOLD flown on the estimate). |
| IDLE_GROUND | No transition; pre-arm gate 3b refuses ARM (`ArmBlock::TofSilent`); a switch to ALT_HOLD / POS_HOLD is refused (`ModeBlock::TofSilent`, logged once per refused request, shown by CLI `status`), ACRO / STABILIZE are accepted; on the ground the refused switch edge persists and applies by itself when the episode has ended. ARM is refused in EVERY mode (ACRO included) until the episode has ended (the ToF alive for 1 s and the estimator has re-acquired it), not merely until a sample arrives. |
| IDLE_HELD, INIT | No transition. ARM requests are not accepted there; after IDLE_GROUND is reached the IDLE_GROUND row applies. |
| In every state | During an episode a switch INTO ALT_HOLD / POS_HOLD is refused (the craft would fly on an altitude nobody can vouch for). In FLIGHT the refused switch edge is CONSUMED (`decideToFModeRefusal`, `ModeBlock::TofSwitchConsumed`, log `IN FLIGHT the switch request is CONSUMED: flip the switch again after the ToF has recovered`, shown by CLI `status` until the craft is on the ground again or a mode request is accepted): the mode never switches by itself at the instant the ToF recovers, the pilot flips the switch again . |
| Re-acquisition | `sf::ToFHealthTracker` (ImuTask): `altitude_reliable` = `kTofReacquireSamples` (10) consecutive samples ADOPTED by the estimator with an innovation of at most `kTofReacquireInnovationM` (0.10 m) AND the ToF alive (a continuous stream) for `kTofAliveMinUs` (1 s) since the first sample after the loss (latched; only a loss clears it; all candidate values). Before the first VALID sample after a loss `IEstimator::reacquireAltitude()` (ESKF: `EskfCore::reacquireVertical()`) inflates the vertical covariance and lifts the innovation gate for that sample: the SAME mechanism as the end of a flip, one implementation. When the episode begins while ARMED, `TakeoffLandingMgr::markAirborne()` flips the RAW ground-contact flag to "airborne"; it returns to "ground" only through the existing air-to-ground debounce (consecutive VALID ground readings), and during the whole episode `groundContactTrusted()` answers "armed = airborne". The descent without a ToF continues until the episode ends: thrust ceiling 0.95 x the support thrust (below the weight); a velocity loop (PI) while the velocity estimate is trusted, the constant ceiling thrust after that, the late fall from 5 s into the descent and a 10 s time limit counted from the entry into LANDING; touchdown by the accelerometer (impact then stillness, or the floor holding the craft up). |
| Blind descent | `PidController::blindLandingThrust` -> `sf::BlindDescent` (`blind_descent.hpp`) around `blind_landing.hpp`; the velocity is `sf::VerticalVelocityEstimator` (`vertical_velocity_estimator.hpp`); host test `test_blind_landing`. The previous open-loop time function could not guarantee a descent (SILS truth: auto-takeoff silenced at 1.3 m/s rose to 4.0 m with `hover.thrust_corr` 1.1, 10.8 m with 1.2, up to 38 m with noise n2, 8 of 80 runs stopped the motors in the air). **Velocity estimate:** rule 1 at rest on the ground: v := 0 and the accelerometer offset B := low-passed (raw upward reading - g); rule 2 while the ToF is healthy and consumed and the ESKF vertical state is free (`StateEstimate.airborne`, not pinned by the on-ground hold) for 1 s: a 2nd-order complementary observer (tau 4 s) follows the ESKF velocity and learns B; rule 3 otherwise: v += (raw upward reading - g - B) dt with B frozen, plus a contact rule (the reading above the 2 g impact threshold for 2 samples = the floor stopped the craft: v := 0 when it falls below 0.75 x the threshold; without it the integral of the impact left +0.2 m/s on the ground and the detector missed 22 of 80 noisy runs). Deviation from the first design (reset to the ESKF value every cycle) with evidence: 0.4 s after the ground-to-air handoff the ESKF reads 0.56 m/s while the craft climbs at 1.27 m/s (0.5 m extra climb when copied), and the ESKF accel-bias estimate moved +0.19 m/s^2 within 0.6 s of a descent (noise-free SILS: the raw accelerometer has no bias), so the corrected specific force is un-corrected (specific_force + accel_bias) and B is kept and frozen here. The error of rule 3 is (offset error) x (time since the last reset). `fuseExternalVelocity()` is the barometer interface (next stage). **Thrust (2026-11-09):** thrust <= `kNoTofThrustCapRatio` (0.95) x the support thrust ALWAYS (the hover-thrust estimate is not a reference of the ceiling); while the velocity estimate is within `kVerticalVelocityTrustS` (2 s) of its last reset (`VerticalVelocityEstimator::sinceResetS()`): thrust = support + PI(-0.4 m/s - v) bounded to [0.25 x support, the ceiling] (the PI can only lower the thrust); after that, also when the descent starts with an old estimate (a manual flight with the ToF stopped for tens of seconds): the constant ceiling (0.05 g downward); then the late fall factor. Why: the estimate's error grows with the time since its last reset (real logs through the estimator after a ToF loss in the hover: -0.21 m/s at 1 s, -0.25 .. -0.39 at 3 s, -0.44 .. -0.71 at 5 s; flip_test2 -0.34 at 2 s), and a PI that raised the thrust against a 'too fast' descent made the craft climb again (previous build, SILS: an accelerometer offset of 0.1 m/s^2 rose again and touched down at 1.43 m/s, 0.2 m/s^2: 2.69 m and 5.06 m/s; a soft touchdown followed by a PI-driven re-take-off to 1.21 m). **Support thrust** (`SupportThrustFilter`): means of the commanded thrust (through the actuation lag 0.08 s) and of the upward specific force over the samples whose low-passed reading is within 5 % of g, from the moment the craft has left the floor (healthy ToF says airborne, or the estimated rise is 0.10 m, or the reading exceeds g by 1 m/s^2), calibrated after 2 s of such samples (a real hover: the 2 s mean is within +-1 % of the long mean, 0.3 s: +-4.9 %); support = thrust mean x g / reading mean, bounded to 0.5-1.5 x the estimate. Without a window: min(quotient over ALL airborne samples x 0.85 once 0.15 s are there, 0.75 x the hover estimate) (SILS: the quotient in a +0.2 g climb is 11-12 % above the hover thrust because the delivered thrust is concave in the command); the ceiling is then below the weight while the estimate is below 1.40 x the truth. Parameters `safety.blind_descent.cap_ratio` and `safety.vvel.trust_s` are for SILS sweeps (a ratio of 1 or more reproduces the old behavior). **Price:** once the velocity target is reached the descent speed is set by the ceiling; from the 1.2 m ceiling the craft touches down at 0.9-1.0 m/s (the previous build 0.4-0.5 m/s), and a support thrust calibrated below the weight makes it harder. PI gains are fractions of the hover estimate (Kp 1.0 per m/s, Ti 2 s, output limit 1.0). **Last safety (late fall):** after `kBlindLandingLateStartS` (5 s of the blind descent in this LANDING) the thrust is multiplied by a factor falling at 0.3 /s: ZERO at 8.33 s whatever the estimate says; a normal descent from the 1.2 m ceiling ends in ~3.5 s, so it never acts then. Parameters `safety.blind_descent.rate_mps/kp/ti_s` and `safety.vvel.tau_s/settle_s` exist for SILS sweeps (defaults in `sf::limits`, candidate values). |
| Ground-contact flag | `TakeoffLandingMgr::on_ground_` freezes while the ToF is silent. ImuTask uses `groundContactTrusted()`: episode + armed = NOT on the ground, episode + disarmed = on the ground; used by the vertical handoff / hold, the tilt monitor and `system_status.airborne`. |
| Altitude ceiling | `sf::limits::kAltitudeCeilingM` = 1.2 m (candidate value) bounds the auto-takeoff target (`static_assert`), the API altitude inputs and, since 2026-11-09, the ALT_HOLD / POS_HOLD stick climb and the rc velocity guidance in the controller (`PidController::altitudeHoldClimbRate()`, the same constant as the API's rc): at or above the ceiling a climb command is 0 and the altitude target is clamped to it; below it the climb rate is tapered to 0.45 1/s x the distance to the ceiling (a plain cut of a full-rate climb overshot and chattered: 1.13-1.44 m in SILS). SILS `alt_ceiling_stick`: the stick climb stops at 1.17-1.18 m (1.97 m without the ceiling); with noise n2 (3 seeds) the truth peak is 1.185-1.187 m. ACRO / STABILIZE have no ceiling. |
| Altitude ceiling, brake side (`PidController::brakeAboveCeiling()`) | The hard side above (a climb command above `kAltitudeCeilingM` + `kAltitudeCeilingMarginM` = 1.35 m is forbidden, `limitClimbAboveCeiling()`) cannot stop an overshoot that is already under way (the velocity loop's correction is limited and the hover estimate may be high: SILS 1.96 m with a +20 % estimate and noise n2), so the thrust is also limited while the craft is rising and would stop too high. The stop height is predicted as altitude + v x `kSupportThrustLagS` (80 ms) + v^2 / (2 a), with a = `kAltitudeCeilingBrakeDecelDerate` (0.7) x g x (1 - `kAltitudeCeilingBrakeFrac`) (0.5); the brake starts where that exceeds `kAltitudeCeilingM` + `kAltitudeCeilingBrakeStartM` (1.2 + 0.05 = 1.25 m). Above that the limit on the thrust falls LINEARLY from 1 to `kAltitudeCeilingBrakeFrac` x the SUPPORT thrust (`supportReferenceThrust()`: calibrated, or the early estimate; never the hover estimate) over `kAltitudeCeilingBrakeRampM` (0.1 m) of predicted excess: a hover at the ceiling in noise (the climb-rate sign flips every few samples) sees only a gentle, continuous reduction (the first, hard version switched 20-136 times per run; 0-3 with the ramp; worst true peak 1.288 -> 1.296 m). v is the ESKF climb rate; the accelerometer-integral rate counts only in the auto-takeoff climb and while the ESKF vertical state is not trusted (a stale integral after a long ToF silence made the hard version halve the thrust of a craft that was not climbing: SILS `ceiling_brake_stale_velocity`). A hover at the ceiling is not touched while the prediction stays below 1.25 m, proportional above. Result over `hover.thrust_corr` 0.9 / 1.0 / 1.2 x n0 / n2 (3 seeds), ALT_HOLD / POS_HOLD auto-takeoff and API takeoff: worst true peak 2.03 -> 1.27 m. |
| Open issues found before 2026-11-09; (1) and (3) were resolved on 2026-11-09 | (1) **Resolved: the dead-zone touchdown, the third path of note 7.** With the ToF close-range dead zone modelled in SILS, a soft ToF landing (contact below 3 G) never ended (`landing_dead_zone.scn`): the sensor goes invalid in the last centimetres (its reading lags the truth: 0.085 m at a true 0.038 m), so `on_ground_` (two valid readings below 5 cm) was never set and the stalled-descent test failed. Following the owner's decision, during the LANDING descent a valid reading at or below the near band (`kLandingDeadZoneBandM` = 0.10 m) followed by ONLY invalid samples for `kLandingDeadZoneHoldMs` (0.2 s) means the craft is on the floor (`TakeoffLandingMgr::deadZoneTouchdown()`; no velocity condition; a valid reading above the band or the end of the landing descent cancels it, a valid reading inside the band restarts the count). It is a path separate from the touchdown detector of the descent without a ToF, and acts only in a landing with a live ToF. The 0.2 s is about 3 times the one gap inside the band in the real log (`front_tof_poshold_yaw360`, 0.28 m/s): ONE invalid sample between two valid readings (66 ms). Result: `landing_dead_zone` PASSES (detected 0.2 s after the last valid reading of 0.085 m, with the craft at rest on the floor: true height 0.021 m, vertical speed 0.001 m/s); `alt_disarm_land.scn` still switches the dead zone off to test only the routing of the DISARM. **Replay of the real log** (the ToF samples and flight states fed to `TakeoffLandingMgr` at their recorded times): during the descent of the landing in `front_tof_poshold_yaw360` (277.6-280.2 s) it does not fire (no false detection); after the contact it fires 0.23 s after the last valid reading of 0.031 m (at 280.63 s; in that real landing the 3 G impact cut ended it first, at 280.23 s). Even if a landing descent were assumed throughout TAKEOFF, FLYING and LANDING of all three logs (the worst case for a false detection), the dead-zone path never fired (`yaw4x90`: every ToF sample is invalid; `flip_test2`: the only invalid run after a valid reading in the band comes after the DISARM). NOT confirmed on a real normal landing. (2) **Out of scope of this rework:** the 1.2 m ceiling and the minimum flip height (`flip.min_height_m`, default 1.0 m) are close: ALT_HOLD / POS_HOLD cannot climb to a comfortable flip height. The flip is outside this rework (the owner's decision); it will be decided when the flip development resumes. The default of `flip.min_height_m` is not changed. (3) **Resolved:** the table of the descent without a ToF in TEST_MATRIX.md was re-measured on 2026-11-09 with a build that has the dead zone and the ceiling. |

**Revision of 2026-11-09 (pre-calibration support thrust and the remaining defects; evaluation in `TEST_MATRIX.md`).** The 0.5 m auto-takeoff with `hover.thrust_corr` 1.2 and noise n2 climbed to a true 1.96 m, left the ToF range and landed at 2.51 m/s: the ESKF climb rate is pinned at zero below the ToF handoff height (and wrong for about 1 s after it), so the velocity loop did not see the climb (thrust 1.27 x the weight), its correction is limited to +-0.15 N around the estimate (1.45 x the truth), and the ceiling brake started above 1.35 m at 0.5 x the hover ESTIMATE. Now (vertical channel only, INV-1): `PidController::takeoffClimbRate()` adds the part of the climb that the accelerometer integral shows and the ESKF does not, beyond `kTakeoffVelocityDisagreeMps` (0.6 m/s; a continuous excess, so an ordinary takeoff stays on the ESKF signal: the margin-free version changed `alt_recenter_gate` and `landing_dead_zone`); `brakeAboveCeiling()` starts when the predicted stop height (altitude + v x 80 ms + v^2 / (2 a), a = 0.7 x g x 0.5, v = the larger of the ESKF and accelerometer-based climb rates; **changed later on 2026-11-09: the accelerometer-based rate counts only in the auto-takeoff climb and while the ESKF vertical state is not trusted, otherwise the ESKF rate**) exceeds the ceiling + 0.05 m, with 0.5 x the SUPPORT thrust (calibrated, or the early estimate) instead of 0.5 x the hover estimate. A thrust ceiling that falls with the overspeed was also built and measured: at most 0.03 m, not adopted. True peak over `hover.thrust_corr` 0.9 / 1.0 / 1.2 x n0 / n2 (3 seeds), ALT_HOLD / POS_HOLD auto-takeoff and the API takeoff: worst 2.03 m -> 1.27 m (all conditions at or below 1.35 m). Before the calibration the support thrust was min(early estimate, 0.75 x hover estimate), which made the ceiling 0.63-0.71 x the thrust that holds the craft in the two front_tof logs; the early estimate is now used as it is (bounded by 1.5 x the estimate), x `kSupportClimbDerate` 0.92 (was 0.85). Replay of the 3 real logs, ceiling / thrust that held the craft before the calibration: 0.69-0.71 / 0.63 / 0.79-0.91 -> 0.87-0.90 / 0.86 / 0.85-0.98 (target: 0.85 or more, below the weight; 0.94 reaches 1.00 in flip_test2); SILS (silence 0.9-4.5 s after the ARM, corr 0.9 / 1.0 / 1.2, n0 / n2, offset +-0.2 m/s^2): ceiling / weight 0.86-0.97, re-rise at most 0.007 m, touchdown speed after an early silence 1.93 -> 1.43 m/s (n0), 2.67 -> 2.01 m/s (n2). A TAKEOFF that the floor still holds (motor health 0.9, throttle above the takeoff threshold and below the hover) became FLYING after 2 s + 0.15 s ("Ground contact unknown", "Takeoff complete"); in ALT_HOLD it went TAKEOFF -> LANDING on the floor. Now: the 2 s time limit of the invalid-sample count in TAKEOFF needs evidence that the craft left the floor (`SupportThrustFilter::leftFloor()`, published as `controller_status.left_floor`); the manual TAKEOFF -> FLYING needs an estimated rise of `kManualTakeoffMinRiseM` (5 cm) during a ToF-silence episode; the auto-takeoff time backstop (3 s) needs the craft off the floor (`craftIsOffTheFloor()`), otherwise the climb keeps trying; the support-thrust window and the vertical velocity estimate run in a manual TAKEOFF too once the thrust is at least 0.8 x hover (`restingOnGround()`). New scenarios `weak_spool_stabilize`, `weak_spool_alt` (they fail on the previous build). A TAKEOFF that cannot lift does not end by itself (the pilot disarms). The LANDING time limit needs 10 s of LANDING AND 4 s (`kBlindLandingMinSilentUs`) of continuous silence (a ToF that dies after a long LANDING no longer stops the motors in the air at once). The proposal to brake an upward but untrusted velocity estimate with the floor thrust is NOT adopted: after 21 s of silence the estimate errs by -2 to -5 m/s (offset +0.1 to +0.2 m/s^2), more than a climb indoors (about 1 m/s); when the error is upward (about half of the cases) a hovering craft touches down at 2.2 m/s instead of 1.0 m/s (1-D model, `TEST_MATRIX.md`). The descent without a ToF is described as it behaves: it is NOT a regulated 0.4 m/s but, in effect, a descent at the constant downward acceleration set by the thrust ceiling (about 0.05 g); the velocity estimate only stops a climb and starts the descent. The English operation manual has the "commands that wait" paragraph; `safety.blind_touchdown.accel_g` / `floor_s` notify both the estimator and the controller; the ceiling basis text and the `TOF_SILENT` comment follow the present behaviour; `StateManager::transition`, `DataStream::appendEntries` and `processEstimatorCommands` are back below their previous length (78 -> 56, 162 -> 137, 68 -> 53 lines) and `arbitrateFlightMode` nests two levels. The flip is out of scope (the owner's decision): open issue (2) above is rewritten and the default of `flip.min_height_m` is unchanged.
**Revision of 2026-11-09 (ceiling brake; evaluation in `TEST_MATRIX.md`).** The ceiling brake took max(ESKF, accelerometer integral) as the climb rate: after a long ToF-silence episode the integral carries the whole offset error, so switching to ALT_HOLD cut the thrust of a craft that was not climbing to half the weight and the craft hit the floor (SILS `ceiling_brake_stale_velocity`: offset -0.4 m/s^2, integral 3.7 m/s, impact at 1.7 m/s; `ceiling_brake_stale_velocity_alt`: entered at 1.238 m, fell to 0.59 m; in the real logs the integral and the ESKF differ by up to 0.52 m/s during a normal POS_HOLD). Now: the brake uses the integral only in the auto-takeoff climb and while the ESKF vertical state is not trusted (`ceilingBrakeClimbRate()`); while the ESKF is trusted its own climb rate decides (as before); in the cycle in which the ESKF takes the ToF again the integral is set to the ESKF velocity at once (`VerticalVelocityEstimator::alignToEskf()`; the rise is untouched). The evidence that the craft had left the floor could come from the accelerometer offset error alone (SILS: TAKEOFF -> LANDING and FLYING on the floor; injected +-0.3 m/s^2, measured 0.05-0.09 m/s^2). Now `riseM()` (read by the support-thrust window, the ToF health tracker and the manual-takeoff judgement) counts only after the low-passed (reading - g - offset) has reached `kVerticalVelocityLiftEvidenceMps2` (0.4 m/s^2, parameter `safety.vvel.lift_mps2`) above its own slow baseline (time constant `kVerticalVelocityLiftBaselineTauS`, 1 s, which follows the offset error on the floor), and then as the double integral of (acceleration - baseline) since that moment (0 before; the drift of the offset after the evidence does not build up a rise). 0.4 sits between two sweeps (craft declared off the floor while on it, 56 runs per value: the the pre-calibration-thrust build rule 16, the rule with a baseline 2 at 0.3, 1 at 0.4, 0 at 0.5; the ToF-unusable auto-takeoff `tof_invalid_alt_hover`: cut at 0.32 m with 0.4, not recognised with 0.5 and climbing to 4.3 m). **OPEN POINT (a design remark): the margin is thin on both sides** (0.1 m/s^2 above the matrix' offset of 0.3 on the floor, 0.05 m/s^2 on the takeoff side), because both use the same quantity (the upward acceleration of the accelerometer); an independent backstop for a ToF-unusable auto-takeoff that does not rely on the accelerometer (the barometer) is desirable. The evidence comes about 0.4 s after the lift at most (sweep). The price: a gentle lift that never exceeds the baseline by 0.4 m/s^2 is not recognised while the ToF is unusable (the healthy ToF airborne verdict and the upward reading above g by 1 m/s^2 remain; the uncalibrated support thrust is the lowered estimate, the safe side); `tof_recover_invalid_first`, `tof_recover_reflip` and `tof_invalid_alt_hover` were changed accordingly (`TEST_MATRIX.md`). A weak craft that cannot lift kept an auto-takeoff going for ever (thrust at hover + the correction limit; the API `takeoff` replied `error` after 12 s but the motors did not stop). Now StateTask (`updateAutoTakeoff()`) DISARMS when `controller_status.left_floor` has not stood `kAutoTakeoffNoLiftAbortUs` (5 s) after the TAKEOFF began (the new TAKEOFF -> IDLE_GROUND row); not "back to ARMED_GROUND", because in ALT_HOLD / POS_HOLD that state starts the auto-takeoff again after the spool dwell. The manual TAKEOFF is unchanged. New scenarios `weak_spool_api` (`error takeoff aborted`), `weak_spool_alt` (expectation updated), `weak_spool_alt_offset`, `weak_spool_stabilize_offset`. Nobody published `controller_status` in the workshop firmware, so `left_floor` stayed false and `vertical_rise_m` 0 (the invalid-sample count of a TAKEOFF could never start, and the abort of would have fired in every ALT_HOLD / POS_HOLD TAKEOFF). Now the workshop control task publishes a minimal `ControllerStatus` each cycle (`left_floor` = the healthy ToF says airborne, `vertical_rise_m` = the height while the ToF is healthy; both on the safe side, false and 0, while it is silent: `publishControllerStatus()`); the workshop has no altitude modes and the learner owns the motors, so a manual TAKEOFF that stays TAKEOFF during an episode is on the safe side. (M) `takeoff_elapsed_cycles_` (uint16, wraps at 163.8 s) saturates; `kTakeoffVelocityDisagreeM` is now `...Mps`; `kManualTakeoffMinRiseM = kTofInvalidTakeoffRiseM` (one definition); the basis of `kAltitudeCeilingBrakeDecelDerate` and `kBlindLandingMinSilentUs` carries the numbers of the final build; `controller_status.latest()` is read once in `imu_task.cpp`; `StateManager::transition` (56 -> 33 lines), `processEstimatorCommands` (53 -> 48) and `DataStream::appendEntries` (137 -> 8, four helpers, the order in the packet unchanged) are within 50 lines; the Japanese wording follows the glossary. INV check: INV-1: the ceiling-brake change touches the vertical-channel thrust only; INV-3 the abort is decided in StateTask, the evidence (`left_floor`, `riseM()`) is detected in the controller and the estimator; INV-4 the normative table has the new TAKEOFF -> IDLE_GROUND row and the updated manual TAKEOFF -> FLYING condition; the three readers of `riseM()` (`SupportThrustFilter::update`, `feedToFHealth`, `manualTakeoffComplete`) share one definition.

**Revision of 2026-11-10 (switch to the positive floor evidence; evaluation in `TEST_MATRIX.md`).** **Root cause:** "the craft has left the floor" was built from the accelerometer alone (the rise counted after an upward acceleration of 0.4 m/s^2, `left_floor`) and used as the gate of safety decisions. A craft that lifts slowly never shows that evidence, and on the floor an offset of 0.6 m/s^2 or more that appears together with the thrust spool-up fakes it. (1) A ToF-unusable auto-takeoff (motor health 0.9-0.97, `tof_invalid_alt_hover` type) did not count the invalid samples, climbed to 3.8-7.6 m and was DISARMED in the air after 5 s ("did not leave the floor"). (2) A manual STABILIZE takeoff whose ToF output stops never left TAKEOFF, so the comm-loss landing was held for ever: a STABILIZE takeoff (throttle 3095) whose ToF output stopped, with a lost link, climbed to 2.76 m in SILS, the motors stopped in the air and the craft hit the floor at 7.3 m/s. (3) A replaced controller returned `leftFloor()` = false for ever and was always DISARMED after 5 s. **Principle: a missing proof is never taken as proof that the craft is on the floor; no safety decision is gated on an accelerometer-only judgement.** **Changes:** the 2 s time limit of the invalid-sample count in TAKEOFF no longer asks for `left_floor` (back to the earlier rule; the 5 cm rise path stays; both only bring the verdict EARLIER). A ToF-silent LANDING on the floor is harmless: the thrust of that descent is at most 0.95 x the support thrust (the ALT side of the weak-spool case of the pre-calibration-thrust build is withdrawn; SILS `weak_spool_alt`, `weak_spool_api`). The manual (STABILIZE / ACRO) TAKEOFF -> FLYING is back to the earlier rule (FLYING when a ToF-silence episode turns the ground-contact flag "unknown -> airborne"; FLYING may show on the floor: accepted, written in `operation_manual.md`; SILS `weak_spool_stabilize`). Once the comm-loss grace (`kCommLossHoverUs`, 3 s) has elapsed, TAKEOFF also goes to LANDING (`decideCommLossLanding()`, automatic and manual; SILS `commloss_takeoff`). The abort of an auto-takeoff that cannot lift (5 s) DISARMS only on POSITIVE evidence that a healthy ToF shows the craft near the floor, not rising (a valid reading at or below `kTakeoffGroundBandM`, 5 cm, not rising by more than `kAutoTakeoffFloorRiseM`, 2 cm, for `kAutoTakeoffFloorHoldUs`, 1 s; `TakeoffFloorWatch`); while that evidence stands the TAKEOFF is held even if the controller reports it complete. A craft whose ToF reading rises (a slow lift) is not aborted; a ToF that cannot be used ends in a LANDING by the ToF-invalid rule (SILS `weak_spool_alt_floor`, `weak_spool_alt_slow_lift`). Deleted: the accelerometer lift evidence (`kVerticalVelocityLiftEvidenceMps2`, `kVerticalVelocityLiftBaselineTauS`, `VerticalVelocityConfig::lift_evidence_mps2`, the parameter `safety.vvel.lift_mps2`, `VerticalVelocityEstimator::liftSeen()`), `ControllerStatus::left_floor`, `IController::leftFloor()`, `ToFSampleFacts::left_floor`, the workshop `ControllerStatus` publication, `kManualTakeoffMinRiseM`, `craftIsOffTheFloor()`, `manualTakeoffComplete()`, `updateAutoTakeoff()`. `riseM()` is back to its original meaning (the rise since the craft last rested); its two readers use it only in the direction where an error is harmless (the ToF health tracker counts invalid samples EARLIER; the support-thrust window opens EARLIER and its support thrust can only come out LOWER). `SupportThrustFilter::leftFloor()` stays as the internal verdict that opens the window and is not a safety gate (commented). `vertical_rise_m` stays as a column of the flight log (`tof_silence`, 0x4E; protocol compatibility). The inputs that the the ceiling-brake build strengthened (`tof_recover_invalid_first`, `tof_recover_reflip`: 3171 -> 3250) and the `tof_invalid_alt_hover` bound (0.30 -> 0.40) are back at their earlier values and pass. The TAKEOFF completion, abort and comm-loss decisions are the pure functions of `takeoff_policy.hpp` (`decideTakeoff()`, `decideCommLossLanding()`); the host test `test/test_takeoff_policy.cpp` (12 tests: manual x airborne, auto x reached x floor evidence x elapsed time, every `FlightState` x grace, `TakeoffFloorWatch`) checks every cell of the normative table. The floor evidence is detected in ImuTask (published as `tof_health.floor_stays`) and executed by StateTask / StateManager (INV-3); INV-4 in `architecture.md` is updated. Re-rise after an API `land`: the cause is that the velocity loop of the closed-loop landing (ToF alive) starts from the hover-thrust ESTIMATE and its integral is slow, so an estimate that is too high (stale after a `hover.thrust_corr` change, a sagged battery) keeps the thrust above the weight for seconds (it happens in the build before the change as well: `hover.thrust_corr` 1.05 / 1.1 / 1.15: 0.08-0.13 / 0.17-0.24 / 0.27-0.32 m, seeds 1-3; 1.2 cannot be compared because the takeoff before the change overshoots). Once the support thrust is calibrated, the base of the loop is bounded by `kLandingBaseOverSupportMax` (1.08) x the support thrust (`landingBaseThrust()`): the rise after the LANDING began is 0.00-0.09 m (up to 0.65 m in the ceiling-brake build). A bound of 1.05 turned the soft contact of `landing_dead_zone` (about 0.2 m/s) into a 3.2 G impact (0.27 m/s) and is not used. The auto-takeoff overshoot: sweeping the margin `kTakeoffVelocityDisagreeMps` (0.6 m/s) over 0.45 / 0.3 / 0.2 / 0.1 (120 runs) leaves the worst true peak at 1.296 / 1.295 / 1.311 / 1.314 / 1.303 m for 0.6 / 0.45 / 0.3 / 0.2 / 0.1 (the ceiling brake decides the overshoot) and improves only `hover.thrust_corr` 1.0 with noise n2 (1.14 -> 1.06 m) and 0.9 with n2 (0.82 -> 0.77 m): NOT adopted (0.6 stays). `kBlindLandingMinSilentUs` 4 s -> 5.5 s (the longest measured descent is 4.7 s; 0.8 s of margin; before the 8.33 s at which the late fall brings the thrust to zero). The ceiling brake switched on and off: in the 1.2 m hover and the takeoff overshoot it went deeper than half of its depth up to 136 times per run (every time noise flipped the sign of the climb rate), so its limit now falls linearly with the predicted excess of the stop height (`kAltitudeCeilingBrakeRampM`, 0.1 m: at most 3 times, worst true peak 1.288 -> 1.296 m); the text "a hover at the ceiling is not touched" now reads "not touched while the prediction stays below 1.25 m, proportional above". Minor items: the interleaved comment and the conflicting "rise" description vanished with the lift evidence; `publishControllerStatus` is 50 lines; the flat statement in `operation_manual.md` is now a case split on what the ToF returns. INV check: INV-1: the re-rise brake and the ceiling-brake intermittency change touch the vertical-channel thrust only; INV-3: the floor evidence is detected in ImuTask, decided by a pure function, executed by StateTask / StateManager; INV-4: the TAKEOFF -> FLYING (manual / auto), TAKEOFF -> IDLE_GROUND and comm-loss rows are updated and every cell is checked on the host; the two readers of `riseM()` share one definition.
**The value of the ceiling before the calibration, the re-rise of the LANDING, how a TAKEOFF ends, the touchdown verdict of the LANDING (evaluation in `TEST_MATRIX.md`).** (1) **`kSupportClimbDerate` 0.92 -> 0.91.** With 0.92 an accelerometer offset of +0.6 m/s^2, `hover.thrust_corr` 1.2-1.3, no noise and a silence 0.8-0.9 s after the take-off began make the early estimate read 1.14 x the true weight; the ceiling is 0.996 x the weight, the craft HOVERS at 0.6-0.7 m until the late fall starts at 5 s and then touches down at 2.02 m/s (main as of 2026-11-08: 0.89 m/s). Over the 340 runs that stop the ToF after an API takeoff (140) or right after the takeoff began (200), the cells WORSE than main as of 2026-11-08 are 4 at 0.92, 2 at 0.915, 0 at 0.91 / 0.905 / 0.90. 0.91 is the smallest value with none; with it, 0 of the 571 runs that landed and touched the floor rise above 0.10 m after the contact (highest 0.047 m; with 0.92 one run re-took off from the floor to 0.38 m). Price: strong touchdowns (above 1.5 m/s) 18 -> 33 of the 200 runs and 38 -> 67 of the 870 valid runs of the ToF-silent matrix (at most +0.5 m/s), the lowest real-log ceiling over the thrust that held the craft 0.851 -> 0.842; the increase is to be lowered by integrating the barometer, not by the value. The bound in `test_blind_landing` is 0.85 -> 0.84. (2) **The re-rise of the LANDING:** the base thrust of the closed-loop landing (ToF alive) is bounded by `kLandingBaseOverSupportMax` x the support reference also before the calibration (`landingBaseThrust()`), and the thrust is limited while the craft rises (`brakeRiseInLanding()`, the ESKF climb rate alone). `hover.thrust_corr` 1.2 with noise n2: a LANDING that began at 1.19 m rose to 1.98 m and touched down at 2.20 m/s (the true vertical speed at the first contact with the floor, below 3 cm); now no re-rise, 0.38 m/s at most (120 runs of the API `land` 6 s after an API takeoff: strong touchdowns (above 1.5 m/s) 9 -> 0, re-rise above 0.10 m 46 -> 0, LANDINGs that did not end 21 -> 0, touchdown speed 0.70 m/s at most; compared with main as of 2026-11-08). (3) **The floor evidence and the end of a TAKEOFF:** the abort needs 1 s of evidence NOW (`floor_stays`) while the completion needed only that there was none at this instant, so one valid reading outside the band after the abort time made FLYING on the floor. The evidence is LATCHED (`TakeoffFloorWatch::seen()` = `tof_health.floor_seen`) until the craft is shown to leave the floor (valid readings above 5 cm for 1 s, or one above 15 cm). A latch that is not released with no evidence NOW (the readings straddle the 5 cm band, no run reaches 1 s) ends the TAKEOFF by a LANDING from 5 s on (`TakeoffAction::Land`), except while the craft is lifting off (`floor_leaving`: a run of valid readings above the band is in progress; `weak_spool_alt_slow_lift` keeps its result). `test_takeoff_policy`: 24 tests (every cell; the order of readings that straddle the band every sample, every 5, 15, 29, 31 and 32 samples). (4) **A land request during the TAKEOFF:** the API `land` does not start a LANDING in the TAKEOFF. `StateManager::notifyLandingRequest()` accepts FLYING (and the immediate DISARM in the ARMED_GROUND of an altitude mode) only, and an API `land` that arrives while a `takeoff` waits is parked by ApiTask and runs after the takeoff has ended (note 13). The ways from TAKEOFF into LANDING are the loss of the link, the ToF rules and the decision to end a takeoff that cannot lift (`TakeoffAction::Land`, `StateManager::notifyTakeoffEndedByLanding()`). **The pilot's DISARM is NOT a land request in the TAKEOFF either: it is an immediate motor cut in every mode** (note 5), because a LANDING started on the floor lifted the craft in SILS. Making a `land` in the TAKEOFF start the LANDING at once was evaluated and not adopted: it needed condition after condition in the touchdown verdict of the LANDING (a floor reading that arrived in the LANDING, the accelerometer touchdown detector also with a live ToF, an exception in the ToF-invalid rule) and still left a LANDING of a live ToF whose accelerometer verdict "impact, then still" ended it at 1.1 m (10 of 384 SILS runs, touchdown at 4.5 m/s) and a LANDING that began on the floor and did not end with an accelerometer offset of +1.0 m/s^2 or more. (5) **The landing verdict left over at the start of a LANDING** (`TakeoffLandingMgr::noteLandingDescent()`, `SystemStatus.verdict_state_since_us`, note 7). (6) **One TAKEOFF clock:** `SystemMode.state_since_us`, stamped by `StateManager::enterState()`, replaces the elapsed times that ImuTask and StateTask counted separately. (7) **A hole in the latch of the floor evidence:** when only invalid samples follow a reading in the band for the hold time (1 s), `floor_stays` is true by the clock while the latch stayed false, and the next reading outside the band completed the TAKEOFF. `TakeoffFloorWatch` now latches at every new sample, valid or not, once the hold time has passed (`latchIfStays()`); one more host test in `test_takeoff_policy`. (8) **`land` in ARMED_GROUND:** the immediate DISARM only in an altitude mode (ALT_HOLD / POS_HOLD); refused in a manual mode (ACRO / STABILIZE), where the throttle alone can lift the craft (known issue, not fixed); the lines parked before it are dropped (note 13). **INV check and ripple check:** INV-1: the changes of the LANDING thrust (the base bound, the rise brake) are in the vertical channel only. INV-3: the detection of the floor evidence (`floor_stays`, `floor_seen`, `floor_leaving`) is in ImuTask, the decision in the pure functions `decideTakeoff()` and `landingVerdictCounts()`, the execution in StateTask and StateManager; the API only publishes `api_command` and observes `system_mode`. INV-4: the normative table has the TAKEOFF -> LANDING row, the ARMED_GROUND -> IDLE_GROUND row (altitude mode and manual mode) and the table of the TAKEOFF completion and latch cells, checked by host tests and scenarios. The components that embedded an old premise were listed and checked: (a) `cmdLand` (TAKEOFF counts as airborne, but StateManager ignores the request there; ARMED_GROUND only in an altitude mode), (b) `dispatchDuringWait` (a `land` during a wait runs at once only in the ARMED_GROUND of an altitude mode; in TAKEOFF it is parked), (c) the callers of `requestDisarm()` (ARMED_GROUND, pilot, API, attitude mismatch, the abort of a takeoff that cannot lift) are unchanged (the DISARM in TAKEOFF is still a motor cut), (d) the readers of `SystemStatus` (the pre-arm gate in `state_manager.cpp`, `control_task`, `data_stream`, `emu_flightlog_vehicle`) read no new item (the content of `landing` is unchanged), (e) the LANDING component that embeds "the ESKF vertical state stays pinned on the ground" is `brakeRiseInLanding()` (the ESKF climb rate reads 0); the ways into a LANDING from the floor are the three above only, and a craft that the takeoff thrust did not lift is not lifted by the landing thrust (note 8 (3)). **Not adopted after an evaluation:** a slow decrease of the thrust before the calibration (0.03 / 0.05 per second from the start) removed the worse cells but made the strong touchdowns of the 200 runs right after the takeoff 65 / 94; judging the landing verdict afresh only for a craft that has been seen off the floor (64 weak-craft runs: the eight LANDINGs of 6.38 s on the floor stay 6.38 s; the verdict was already gone because `markAirborne()` flips the ground flag at the ToF-silence episode); a dead band in the climb rate of the rise brake (no value lowered the touchdown speed without raising the climb: note 8 (4)); a thrust ceiling for a LANDING that began on the floor (no entry that needs it is left: note 8 (3)). **Unconfirmed:** everything on the real vehicle; the rise by inertia of a LANDING that began while the craft was already climbing; whether the increase of strong touchdowns with 0.91 is the same on a real vehicle; the behaviour of `floor_leaving` with the real ToF (the real floor reads invalid, so that path is never set).

**Known trade-offs (not fixed; numbers are SILS truth, evaluation in `TEST_MATRIX.md`).** (a) The bound on the base thrust of the closed-loop landing before the calibration (`landingBaseThrust()`: `kLandingBaseOverSupportMax` x the support reference) raises the touchdown speed when the accelerometer bias is large and negative: at most 0.95 m/s at a bias of -1.5 m/s^2 (main as of 2026-11-08: 0.45 m/s), 0.65 against 0.44 m/s at -0.6 m/s^2; with `hover.thrust_corr` 1.2 it is 0.93 against 3.43 m/s, better than main. (b) While the ToF is inside the 0.05-0.15 m hysteresis band (a band with different thresholds for rising and falling) the ESKF vertical state is pinned to the ground, so a LANDING that begins near the floor can be judged a stalled descent although the craft is rising, and stop at 0.09-0.16 m (the same kind on main, the same number of runs). The direction of a remedy, a velocity that is not pinned, belongs to the barometer-altitude work (B3).

Measured in SILS (truth, 2026-11-09; 80 runs: 5 scenes x `hover.thrust_corr` 0.9-1.2 x noise off / n2 with 3 seeds; table in `simulator/sils/scenarios/TEST_MATRIX.md`). Noise off: rise above the altitude at the silence 0.00 m (0.5 m and 1.2 m hover), 0.22-0.24 m (manual ALT_HOLD climb), 0.03-0.60 m (silence early in the auto-takeoff climb; most of it is the 0.32 s between the ToF stopping and LANDING at the climb speed, the rise after LANDING is 0.01-0.27 m), touchdown speed 0.25-0.49 m/s in all 20 runs, silence-to-touchdown 0.5-4.4 s. Noise n2, worst of 3 seeds: rise 1.18 m (API takeoff, `hover.thrust_corr` 1.2, climbing at 2.2 m/s at LANDING), touchdown speed 0.96 m/s (1.2 m hover, `hover.thrust_corr` 1.2, a flight that had overshot to 2.3 m beyond the ToF range). Main as of 2026-11-08 in the same 80 runs: up to 47 m, 8 motor stops in the air; the present firmware: 0. For comparison, the normal landing with a working ToF touches down at 0.30 m/s. Touchdown detection (impact threshold from three real flight logs: in-flight false events at 2.0 g x 2 samples 0 / 0 / 0, 1.4 g x 2: 16 / 11 / 44; the real touchdown peaks at 3.39 g over 3 samples): with the 3 G impact cut disabled the detector ended all 80 SILS runs 0.38-0.91 s after the contact (median 0.44 s, none in the air, 58 of 80 before the contact rule of the velocity estimate was added); in the normal configuration the IMPACT cut fired first in 79 of 80 runs (the detector first in 1). Not met / open: real hardware; the 0.32 s detection delay is outside this change.

Verification (2026-11-09): host tests `test_tof_health` (`classifyInvalidSample()` for every phase, the TAKEOFF rise / time conditions and the short persistence, invalid samples in LANDING near the floor and in FLIP are neither agreement nor evidence, the re-acquisition request before the first valid sample after a stuck verdict, the stateless staleness test for the CLI), `test_blind_landing` (the rise estimate); SILS new `alt_ceiling_stick`, `alt_ceiling_stick_noise`, `tof_recover_landing_invalid`, `landing_dead_zone` (known fail until 2026-11-09; PASS since 2026-11-09 with the dead-zone touchdown; host tests `land_dead_zone_*` in `test_main`); fixed `tof_invalid_alt_hover`, `alt_disarm_land`, `rc_flip`, `rc_flip_latch`, `tof_silent_alt_climb`, `tof_silent_hover_1p2`, `alt_inflight_switch`, `alt_recenter_gate`. Earlier: host tests `test_tof_silence` (every state x mode cell, mode refusal), `test_tof_health` (re-acquisition, invalid run, stale status, in-flight edge consumption), `test_eskf_flip_hold` (`reacquireVertical`), `test_main` (blind touchdown wiring, `markAirborne`), `test_blind_landing` (late fall, velocity estimator, velocity loop on a 1-D plant, touchdown detector), `test_sensor_freshness`; SILS `tof_silent_*` scenarios and, with the fault injection `tof_invalid` added, `tof_recover_frozen_ground` (truth alt_max 10.9 m -> 2.5 m), `tof_recover_climb_alt` (alt RMS 1.19 m -> 0.02 m), `tof_recover_edge_consumed`, `tof_recover_reflip` , `tof_recover_invalid_first` , `tof_recover_prearm`, `tof_invalid_alt_hover`, `tof_invalid_stabilize` , `tof_silent_flip_end` , and for the closed-loop descent `tof_silent_takeoff_est_high`, `tof_silent_api_takeoff_est_high`, `tof_silent_takeoff_noise`, `tof_silent_touchdown_detect`. Not verified: real hardware (hover-thrust error and its sign, softness of the real contact for the touchdown detector, the accelerometer offset and vibration rectification of a real craft for the velocity estimate, how invalid samples look, whether 0.10 m / 10 samples / 1 s / 0.5 s are right, the time a real craft needs to leave the ToF dead zone at takeoff), and continuous operation longer than 36 min. On the ground an invalid ToF is normal, so ARM is accepted with a permanently invalid ToF; the auto-takeoff is then cut 0.15 s after the estimated rise reaches 5 cm (SILS: the craft rises 0.20 m, 0.15-0.41 m over noise n2 and hover-thrust error; with the 0.5 s of FLYING it would reach 0.99-1.24 m with n2).

### Note 13: `emergency` / `stop` while ApiTask waits (2026-11-09)

`takeoff`, `land`, moves and `flip` block in `waitUntil` (`tasks/api_task.cpp`). The wait loop keeps reading the command sources (SILS injection ring, UDP) every 50 ms:

| Line read during a wait | Behaviour |
|-------------------------|-----------|
| `emergency` | Run at once (motors first), DISCARD every parked line answering each `error cancelled by emergency`, end the wait (`error interrupted by emergency`). Replaying a parked `takeoff` after an emergency could take off again. |
| `stop` | Accepted (FLYING with a guidance target): run at once, discard the parked lines (`error cancelled by stop`), end the wait (`error interrupted by stop`). Rejected: only the error reply, parked lines stay, the wait continues. |
| `rc` | Dropped at once, no reply: a stale operator input must not be replayed after the wait. |
| anything else | Parked (up to 8 lines, `kDeferredSlots`), run in arrival order after the blocking command has answered; `error busy` when full. |
| `land` while the craft is in TAKEOFF (or in the short time right after the wait began, while the state is still IDLE_GROUND) | Parked like any other line, and run as an ordinary `land` after the `takeoff` has answered: once the takeoff has completed (FLYING) it starts the LANDING and answers `ok` at the touchdown; if the takeoff was aborted the craft is no longer flying and it answers `error not flying`. A `land` that arrives in TAKEOFF OUTSIDE a wait (e.g. during an auto-takeoff started by the RC ARM: no command is waiting, the main loop reads it) is held by `cmdLand` until the TAKEOFF has been left (at most `kTakeoffTimeoutMs` = 12 s) and then handled by the same rule: FLYING -> the LANDING is issued and `ok` at the touchdown; LANDING -> that landing is waited for and `ok`; on the ground -> `error not flying` (before, `ApiCmd::Land` was published in TAKEOFF, ignored, and answered `error land timeout` after 20 s). SILS `api_land_in_rc_takeoff`. It is NOT a request for a LANDING during the TAKEOFF: a LANDING that begins on the floor can float the craft up (note 5). SILS `api_land_deferred_in_takeoff`, `api_land_in_takeoff`. |
| `land` while the craft is in ARMED_GROUND in an ALTITUDE mode (ALT_HOLD / POS_HOLD) | The motors have only begun to spool on the floor, so there is no height to land from: the request is the immediate DISARM (`notifyLandingRequest()` calls `requestDisarm()`), answered `ok` once the craft is confirmed on the ground (IDLE_GROUND, at most `kLandConfirmMs` = 0.5 s; as `emergency` is, not parked), the wait ends and the waiting `takeoff` answers `error cancelled by land`. The lines parked before it are DROPPED as `emergency` drops them (each answered `error cancelled by land`): a parked `takeoff` would ARM the craft again right after the DISARM. SILS `api_land_in_armed_ground`. |
| `land` while the craft is in ARMED_GROUND in a MANUAL mode (ACRO / STABILIZE) | NOT accepted, as before (`error not flying`; during a wait it is parked): the pilot holds the throttle and the craft can lift off from ARMED_GROUND (a throttle of 0.499 is enough), so it is not known to be on the floor. Known issue, not fixed: a manual ARMED_GROUND can lift off by the throttle alone. In an altitude mode, a `land` that finds the state already moved to TAKEOFF is ignored by StateManager; ApiTask therefore answers `ok` only after IDLE_GROUND is confirmed, and otherwise parks the `land` (the wait goes on, the waiting `takeoff` answers as the takeoff ended, the parked `land` runs after the takeoff). The race itself (a window of a few ms) cannot be produced in SILS and is unconfirmed; with the publish of the request left out to imitate losing the race, the parked path ran as designed (`takeoff` `ok`, then the `land` through LANDING to IDLE_GROUND and `ok`). SILS `api_land_manual_armed_ground`. |

Every line is answered at its OWN sender (`ReplyTarget`): a line read from another sender during a wait no longer redirects the blocking command's final reply. No state machine or Pub-Sub change (INV-3). Verification: SILS `api_takeoff_emergency`, `api_land_emergency`, `api_land_stop`, `api_move_stop`, `api_wait_deferred`. Not verified: real hardware latency and the UDP path (the SILS socket shim delivers no data, so the per-sender reply target is untested).

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

> **レート内ループの帰還（angular_rate）:** `compute()` は最内ループ（角速度制御）の帰還量を `StateEstimate.angular_rate[3]`（機体 FRD・推定器でバイアス補正済み）から読む。推定器（ESKF 実装では `predict` 内の `gyro_raw − gyro_bias`）がこのフィールドを毎サイクル埋める。アルゴリズム非依存：相補/Madgwick 等に差し替えても、角速度を出す推定器ならこのフィールド経由で同じレート制御が閉じる。
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

HAL components are copied into vehicle (fully independent). See Japanese section for complete tree.

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

Total task stacks: ~102KB out of 512KB RAM. Remaining ~410KB for Pub-Sub buffers, ESKF matrices, heap, etc.
