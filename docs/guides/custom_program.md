# ACROモードのPID制御をゼロから組み立てる

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. この記事で作るもの・対象読者・前提

### この記事のゴール

送信機のスティックを倒すと、その方向・その速さで機体が回転する——ACROモード（角速度制御モード）の制御プログラムを、**既存の `PidController`（このプロジェクトに最初から入っているカスケードPID制御器）に一切頼らず**、自分の手で最初の1行から組み立てる。

具体的には、次の状態を自分のコードで作る。

- ロールスティックを倒すと、その分だけ機体がロール方向に回転し続ける（倒し切ると最大角速度、離すと回転が止まる）
- ピッチ・ヨーも同様
- スロットルで推力（浮く強さ）を直接操作する
- 送信機との通信が切れたら、暴走せずに自動でゆっくり降りる

最終的に3軸（ロール・ピッチ・ヨー）すべてにフィードバック制御がかかり、実際に機体を飛ばせる完成形まで、1章ずつ機能を足しながら進む。

この記事は、将来的に vehicle ファームウェア全体を自分ひとりの手で書けるようになるための、最初の一歩でもある。ACROモードのPID制御はこのファームが持つ制御則の中でもっとも単純なものであり、ここで Pub/Sub・トピック・`IController` という仕組みに一度手を動かして慣れておくと、この先どんな制御則（姿勢制御、あるいは13章で触れるもっと違う考え方の制御則）を自作するときも同じ土台の上で進められる。

### 対象読者

- C++の基本文法（`if`/`for`/構造体/クラス程度）はわかる
- 「Pub/Sub」「名前空間（namespace）」「組み込み開発」が何なのかはまだよくわからない
- 制御工学の入門（比例・積分・微分が何をする項か）は聞いたことがある程度で構わない。各概念は本文中で説明する

### 前提

この記事は、リポジトリ直下の `README.md` にある「実際に飛ばしてみよう」まで一度済ませていることを前提にする。つまり、標準ファームウェアを機体に書き込み、送信機で一度は飛ばした経験があるということである。まだの場合は先にそちらを終えてから本記事に進んでほしい。

この記事が扱うのは `firmware/vehicle`（vehicle本体。実機・SILS〈SILS: Software In the Loop Simulation、ファームウェアそのものをPC上で動かす検証方法〉の両方で使われる主力ファームウェア）の **L1 Topic API**（`sf::api::` 名前空間と、差し替え可能な `IController` というインターフェース〈interface: 「この関数さえ実装すれば中身は自由」という取り決め〉）である。センサドライバやハードウェア初期化そのものを書く層（HAL/BSP）はこの記事の範囲外である。

## 2. Pub/Subって何か、名前空間って何か

### 掲示板に例えると

このファームウェアの中で、部品（コンポーネント）同士は関数を直接呼び合わない。代わりに **Topic**（トピック。決まった型のデータを1つだけ保持する箱）を介して、**Pub/Sub**（Publish＝発行、Subscribe＝購読）という方式でやり取りする。

これは「掲示板」に例えるとわかりやすい。

- **`publish()`（発行する）** は、掲示板に新しい紙を貼り直す行為。前に貼ってあった紙をめくって捨て、新しい紙を1枚だけ貼る。貼る人は「誰がこれを読みに来るか」を一切知らない。
- **`latest()`（最新値を見る）** は、通りがかりに掲示板を覗いて「いま貼ってある紙」を写し取る行為。見に行く人は「誰が貼ったか」を知らない。何回覗いても紙は減らない（同じ紙を何度でも読める）。
- **`read()`（一件取り出す）** という取り方もある。こちらは郵便受けから手紙を1通取り出すイメージで、取り出すと無くなる（次に見た人には見えない）。トピックによって「掲示板方式」と「郵便受け方式」のどちらで使うかが決まっている。

センサを読むタスクが「センサ値」という掲示板に値を貼り、推定・制御を行うタスクがそれを覗いて計算し、また別の掲示板（例えば「制御出力」）に結果を貼る。このリレーの中のどこかの一部品（センサ値を覗くだけの監視役、あるいは推定・制御そのもの）が、これから自分で書くコードになる。

### 名前空間（namespace）は「建物の階」

`sf::api::estimate_latest()` のような書き方に出てくる `sf::api::` が **名前空間（namespace）**——「同じ名前の関数が別の場所でも衝突しないようにする、住所のようなラベル」である。難しく考えず、「`sf` という建物の `api` という階にある関数」くらいの理解でよい。この記事で使う関数は基本的に `sf::api::` 階（学習者向けに公開されたTopic API）に置かれている。

### この記事での全体像

この記事で書くコードは、**入力側**（スティック・センサの値を読む部分）は今説明したPub/Subそのもの——`sf::api::何か_latest()` を呼んで掲示板を覗くだけである。一方 **出力側**（推力・トルクを機体に伝える部分）は少し違う仕組みを使う。自分で `publish()` を呼ぶのではなく、`IController` という決まった形の「箱」に計算結果を詰めて返す。その箱を実際に掲示板へ貼りに行くのは、この記事のコードではなく、ファームウェア側の `ControlTask` という別の担当者である。なぜそうなっているかは3章で説明する。

## 3. ACROモードの全体像

ACROモードの制御は、次の一本道でできている。

```
スティック（throttle / roll / pitch / yaw、送信機からの入力）
        │
        ▼
目標角速度 = スティック値（roll/pitch/yaw） × 最大角速度
目標推力   = throttleスティック値 × 最大推力
        │
        ▼
誤差 = 目標角速度 − 実際の角速度（ジャイロが測った値）
        │
        ▼
PID演算（比例・積分・微分）→ 軸ごとのトルク（回転させる力）
        │
        ▼
ControlOutput { thrust（推力）, torque[3]（ロール・ピッチ・ヨーのトルク） }
        │            ← ここまでが、この記事で自分の手で書く部分
        ▼
sf_actuator が自動でミキシング（4本のモータへのduty配分に変換）
        │
        ▼
      4つのモータ
```

**ミキシング**（mixing。全体のトルク・推力の指令を、4つあるモータそれぞれの回転数指令に配分する計算）は `sf_actuator` というコンポーネントが自動でやってくれる。モータが何本あって、どの位置に付いていて、どちら向きに回るか——そういったハードウェア寄りの計算は、この記事では一切考えなくてよい。自分が書くのは「機体全体としてどれだけの推力とトルクが欲しいか」までである。

図の `ControlOutput` から下の部分こそが、2章で説明したPub/Subの実例である。自分の `compute()` が返した `ControlOutput` を、`ControlTask` が `control_output` というトピック（掲示板）に `publish()` する。`sf_actuator` はその `control_output` を購読（subscribe）しており、貼り出されるたびにミキシングして4つのモータのduty（PWMの通電比率）に変換する。自分のコード（`IController`）と `sf_actuator` は、互いの存在を知らないまま `control_output` という1枚の掲示板だけでつながっている。

### なぜ自分で `publish()` しないのか

2章で触れた通り、この記事のコードは値を読むとき（`sf::api::estimate_latest()` 等）は掲示板を直接覗くのに、値を返すとき（`compute()` の戻り値）は自分で掲示板に貼りに行かない。これには理由がある。

- **1枚の掲示板に貼る人は1人に決めてある。** `control_output` という掲示板に実際に紙を貼る役目は、ファームウェア側の `ControlTask` 1つだけに決まっている。もし複数の場所から同じ掲示板に貼りに行けてしまうと、「最後にどちらが貼ったか」で結果が変わってしまう競合が起きうる。`IController` を実装する側（＝この記事の読者）は、あくまで「次に貼る紙の中身」を計算する係であり、実際に貼りに行く係ではない——これが `ControlOutput` を戻り値として返すだけの理由である。
- **貼りに行く前後にも仕事がある。** `ControlTask` は `compute()` を呼んだ結果を掲示板に貼るだけでなく、同じタイミングでログ用の記録（Data Stream）を組み立てたり、システム同定用の結果を回収したりしている。これらの付随作業を毎回自分のコントローラの中に書かずに済むのも、`ControlTask` 側に「貼る」責務を集約しているためである。
- **Pub/Subの仕組みを知らなくても動かせる。** `compute()` はただの関数呼び出しであり、戻り値をどう扱うかは呼び出し側の自由である。実際、単独ベンチ型の例題（`10_custom_controller` 等）では、Pub/Subの仕組みが一切無い環境で `compute()` を合成信号に対して直接呼ぶだけの検証ができる——これは、コントローラの中身がPub/Subに依存しない「決まった形の箱」だからこそ可能になっている。

## 4. 開発環境の準備

自分のプロジェクトを作るところから始める。

```bash
source setup_env.sh
```

```bash
sf app new my_acro
```

`firmware/apps/my_acro/` に、次のファイルが作られる。

| ファイル | 役割 |
|---------|------|
| `app.yaml` | プロジェクトの種別（`type: embedded`＝vehicle本体に組み込まれる形式）を記す設定 |
| `app.cpp` | `sf::app::controller()` 等、この記事の主役になるクラスをファームウェアに登録する“つなぎ目” |
| `app_controller.hpp` / `app_controller.cpp` | 自分の `IController` 実装を書くファイル。**この記事でずっと編集するのはここ** |
| `README.md` | 複製元テンプレートの説明（そのままでよい） |

`type: embedded`（組み込み型）というのは、このプロジェクトのソースがvehicle本体のビルドに直接コンパイルされ、実機でもSILSでも同じソースがそのまま動く、という意味である。新しいコンポーネントを作ったり、ビルド設定に何かを追記したりする作業は一切不要——`sf app` コマンドが面倒を見てくれる。

`app_controller.cpp` の中に `compute()` という関数があり、これが **400Hz（1秒間に400回、2.5ミリ秒に1回）** で呼ばれる。この記事のほぼすべての作業は、この `compute()` の中身を書き換えることである。呼び出す側（`ControlTask`）は、こちらがARM（モータ始動許可）されているかどうかに関わらず、機体が動いている間ずっとこの関数を呼び続ける——モータへの安全策は別の場所（ARM状態の管理）が担当するので、`compute()` 自体は常に呼ばれる前提で書く。

2.5ミリ秒という持ち時間は短い。`compute()` の中では次のことを守る。

- `new`/`malloc` のような**動的メモリ確保**（実行中にその都度メモリを確保する処理）をしない——確保にかかる時間が読めず、周期を超過する原因になる
- `printf` のような重いログ出力をそのまま毎回呼ばない——5章で間引き方を説明する
- 応答を待って止まる**ブロッキング呼び出し**をしない

この記事のコード例はすべてこれらを守っている（固定サイズの構造体と `float` の四則演算だけで完結し、ヒープ確保は一切登場しない）。

以降の章では説明のたびに一から `app_controller.hpp`/`.cpp` を貼らず、直前の章との**差分**を示す。実際に手を動かす際は `sf app edit my_acro` などで開いて書き換えてほしい。

## 5. まずは角速度を覗いてみるだけのプログラム

### なぜこの一歩が必要か

いきなりモータを回す制御を書く前に、まず「センサの値が自分のコードから見えている」ことを確認したい。ここではモータへの出力は一切せず（`ControlOutput` はすべてゼロを返す）、ジャイロが測った角速度をログに出すだけのプログラムを作る。

### 実装

2章で紹介した `sf::api::estimate_latest()` は、状態推定（センサの値を組み合わせて推定した「今の機体の状態」）の最新値を1つ返す関数——「掲示板を覗く」関数である。戻り値の `StateEstimate` 構造体の `angular_rate[3]` フィールドが、機体座標系（FRD: X軸=前方かつロール軸、Y軸=右方かつピッチ軸、Z軸=下方かつヨー軸）での角速度（単位 rad/s）である。添字は `[0]=ロール` `[1]=ピッチ` `[2]=ヨー` の順に格納されている。

ただし `compute()` の中では、この関数を自分で呼ぶ必要はない。`compute()` の第1引数 `state` そのものが `sf::api::estimate_latest()` と全く同じ値だからである——`ControlTask` が毎周期あなたの代わりに「センサ値」の掲示板を覗き（`estimate_state.latest()`）、その結果をそのまま `compute()` の引数として手渡してくれている。だから `compute()` の中では `state.angular_rate` を直接使えばよい。`sf::api::estimate_latest()` が活躍するのは `compute()` の外——`IController`/`IEstimator` を介さず、Topicの値を自分で覗きに行く追加タスクを書くような場面である（この記事では扱わない）。

```cpp
// app_controller.hpp
#pragma once
#include "controller.hpp"

namespace sf::app {

class AppController : public sf::IController {
public:
    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;

private:
    uint32_t cycle_count_ = 0;   // compute() の呼び出し回数 / call counter
};

}  // namespace sf::app
```

```cpp
// app_controller.cpp
#include "app_controller.hpp"
#include "esp_log.h"

namespace sf::app {

namespace {
constexpr const char* kLogTag = "MyAcro";
// compute() is called at 400 Hz; logging every call would overrun the
// control period (coding_and_education.md §7). Divide down to ~1 Hz.
// compute()は400Hzで呼ばれる。毎回ログを出すと制御周期を超過するため
// （coding_and_education.md §7）、約1Hzまで間引く。
constexpr uint32_t kLogEveryNCycles = 400;
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)setpoint;
    (void)dt;

    ++cycle_count_;
    if (cycle_count_ % kLogEveryNCycles == 0) {
        // angular_rate[3] = roll, pitch, yaw [rad/s], body frame (FRD)
        // angular_rate[3] = ロール・ピッチ・ヨー [rad/s]、機体座標系(FRD)
        ESP_LOGI(kLogTag, "rate roll=%.3f pitch=%.3f yaw=%.3f",
                 state.angular_rate[0], state.angular_rate[1], state.angular_rate[2]);
    }

    // Not yet touching thrust/torque — a zero-initialized output keeps the
    // motors off no matter what the sticks say.
    // まだ推力・トルクには触れない——ゼロ初期化した出力を返せば、スティックの
    // 値に関わらずモータは回らない。
    sf::ControlOutput output{};
    output.timestamp = state.timestamp;
    return output;
}

void AppController::reset()
{
    cycle_count_ = 0;
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    (void)new_mode;   // このコントローラはまだ何もしない / not used yet
}

}  // namespace sf::app
```

`compute()`・`reset()`・`onModeChange()` の3つは `IController` の中で「純粋仮想関数」（pure virtual function。実装しないとコンパイルが通らない、必ず埋めるべき欄）に指定されている。中身が空でも構わないので、まずは全部書いてしまう。

`ESP_LOGI` を毎回（400Hzで）呼ぶと、ログ出力の待ち時間だけで2.5ミリ秒の持ち時間を圧迫してしまう。`cycle_count_` で間引いて約1Hzに落としているのはそのためである——このカウンタによる間引きは、以降の章でも作法として引き継ぐ。

### 動かしてみる

SILS（実機を使わずPC上で飛行を模擬する検証環境）で動かし、ログに角速度が流れてくることを確認する。

```bash
sf app sils my_acro
```

`sf log analyze` や、モニタ画面に流れるログから、機体をSILS上で傾けたときに角速度の値が変化する様子が見えるはずである。

## 6. スティック入力を目標角速度に変換する

### なぜこの一歩が必要か

制御とは「目標」と「現在値」の差（誤差）を無くしにいく仕組みである。前章で「現在値」（測定された角速度）は手に入った。次は「目標」——スティックがどれだけ倒されているかから、目標角速度を作る。

### 実装

パイロットの指令（スティック値）の最新値は `sf::api::command_latest()` で取れる——のだが、これも前章の角速度と同じ理由で `compute()` の中では呼ぶ必要がない。`compute()` の第2引数 `setpoint`（`CommandSetpoint` 構造体）が、`ControlTask` が代わりに覗いてきた `sf::api::command_latest()` と同じ値そのものである。`roll`/`pitch`/`yaw` は `-1..1` の範囲（倒し切った状態が ±1、中央が0）、`throttle` は `0..1` の範囲（中央が0＝推力ゼロ、倒し切ると1＝最大推力）である。

ACROモードでは、スティックの倒し量がそのまま「目標角速度」になる。倒し切ったときに機体がどれだけ速く回るかを決める定数が「最大角速度」である。ロール・ピッチは 1.0 rad/s、ヨーは 5.0 rad/s を目安値として使う（実機で飛行実績のある値）。

```cpp
// app_controller.cpp（compute()の中身を置き換え）
namespace {
constexpr const char* kLogTag = "MyAcro";
constexpr uint32_t kLogEveryNCycles = 400;

// Named constants (no magic numbers): how fast the craft spins at full
// stick deflection. Flight-proven values for this frame.
// 名前付き定数（マジックナンバー禁止）: スティックを倒し切ったときの回転速度。
// この機体で飛行実績のある値。
constexpr float kMaxRollPitchRateRadS = 1.0f;   // [rad/s]
constexpr float kMaxYawRateRadS       = 5.0f;   // [rad/s]
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)dt;

    // Stick -> target angular rate. Still not fed into the output below.
    // スティック → 目標角速度。まだ下の出力には使わない。
    const float rate_sp_roll  = setpoint.roll  * kMaxRollPitchRateRadS;
    const float rate_sp_pitch = setpoint.pitch * kMaxRollPitchRateRadS;
    const float rate_sp_yaw   = setpoint.yaw   * kMaxYawRateRadS;

    ++cycle_count_;
    if (cycle_count_ % kLogEveryNCycles == 0) {
        ESP_LOGI(kLogTag,
                 "target roll=%.3f pitch=%.3f yaw=%.3f | measured roll=%.3f pitch=%.3f yaw=%.3f",
                 rate_sp_roll, rate_sp_pitch, rate_sp_yaw,
                 state.angular_rate[0], state.angular_rate[1], state.angular_rate[2]);
    }

    sf::ControlOutput output{};   // still zero output — no motor spin yet
    output.timestamp = state.timestamp;
    return output;
}
```

SILSまたは送信機のスティックを動かし、ログの「target」列が動くことを確認する。「measured」列（実際の角速度）はまだこの目標を追いかけない——追いかけさせるのが次章の仕事である。

## 7. 比例制御(P)だけで動かす

### なぜこの一歩が必要か

「目標」と「現在値」がそろったので、いよいよ両者の差（誤差）を無くす方向にモータへ指令を出す。最も単純な方法が **比例制御（P制御）**——誤差の大きさにそのまま比例した力を返す方法である。ここで初めて `compute()` が中身のある `ControlOutput` を返す。

### 安全上の注意（必ず読むこと）

ここから先は実際にモータへ非ゼロの指令が出る。以下を必ず守ること。

- **プロペラを必ず外すか、機体をしっかり固定してから実機で試すこと。** まだピッチ・ヨー軸は制御されておらず（後の章で追加する）、ロール軸も調整前の値であるため、機体が暴れる可能性がある。
- 確認の順序は必ず **SILS → 実機ベンチ（プロペラを外した状態、または固定した状態）**。いきなり自由飛行させない。
- `docs/guides/safety.md` の緊急停止手段（`sf emergency` または送信機の緊急停止）をいつでも実行できる態勢で臨む。

### 実装（ロール軸のみ）

まずロール軸1つだけに絞ってPID制御を組み立てる。ピッチ・ヨーはまだ0を返しておく（つまりまだ開ループ——後の章で閉じる）。

誤差の定義は「目標角速度 − 測定角速度」。これに比例ゲイン `kp` を掛けたものをそのままトルク指令にする。

```cpp
namespace {
// ... (前章の定数はそのまま) ...

// Roll rate loop, P-only stage. A modest first guess — not yet tuned.
// ロールレートループ、P制御のみの段階。まだ追い込んでいない控えめな初期値。
constexpr float kRollKp = 3.0e-4f;   // [Nm / (rad/s)]

// Physical torque limit of this frame's roll/pitch axis — a safety bound,
// not a tuning knob (see sf_controller_pid's max_roll_pitch_torque_).
// この機体のロール/ピッチ軸トルクの物理上限——チューニング値でなく安全上限
// （sf_controller_pidのmax_roll_pitch_torque_と同じ値）。
constexpr float kMaxRollPitchTorqueNm = 5.2e-3f;   // [Nm]
constexpr float kMaxThrustN           = 0.672f;    // [N] 4 motors combined
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)dt;

    const float rate_sp_roll = setpoint.roll * kMaxRollPitchRateRadS;
    const float error_roll   = rate_sp_roll - state.angular_rate[0];

    float torque_roll = kRollKp * error_roll;
    if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
    if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;

    sf::ControlOutput output{};
    output.timestamp = state.timestamp;
    output.torque[0] = torque_roll;   // roll
    output.torque[1] = 0.0f;          // pitch — still open-loop, closed in §10
    output.torque[2] = 0.0f;          // yaw   — still open-loop, closed in §10
    output.thrust     = setpoint.throttle * kMaxThrustN;
    return output;
}
```

`kMaxRollPitchTorqueNm`（5.2e-3 Nm）は「これ以上トルクを指令しても現実的でない」というこの機体の物理的な上限であり、チューニングパラメータではない——安全のためのクランプ（出力の頭打ち）として最初から入れておく。

### SILS → 実機ベンチの順に確認する

```bash
sf app sils my_acro
```

問題なければ、プロペラを外した実機（または固定した実機）で確認する。

```bash
sf app build my_acro
sf app flash my_acro -m
```

ロールスティックを倒すと、ロールのトルク出力（`sf log wifi` 等で確認できる）がスティックに応じて動くはずである。ただし、目標角速度ぴったりまでは到達せず、わずかな **定常偏差**（steady-state error。十分時間が経っても残り続ける誤差）が残ることに気づくはずである。これは、モータの回転反力や配線・個体差による小さな外乱が常に存在し、P制御だけではその外乱をちょうど打ち消すところで釣り合ってしまうためである（外乱トルクを `d`、比例ゲインを `kp` とすると、定常状態では `kp × 誤差 ≈ d` となる関係で釣り合う——`kp` を大きくすれば誤差は小さくなるが、上げすぎると振動する)。この定常偏差を消すのが、次章の積分項の役目である。

## 8. 積分項(I)を足す

### なぜこの一歩が必要か

P制御は「今の誤差」にしか反応しない。誤差が小さくても残り続ける限り、時間とともにじわじわ効いていく項が欲しい——それが **積分制御（I制御）**である。誤差を時間で積み上げていき、その積み上げ値に比例した力を追加で返す。

積分項が想定通りに効いているかを実機で確認する際も、7章の安全上の注意（プロペラを外すか機体を固定する、SILS→実機ベンチの順で確認する）はそのまま引き続き守ること。

### 実装

積分の強さは「積分時間 `Ti`」という時定数で表す（`Ti` が短いほど積分が素早く効く）。誤差を毎周期 `(kp/Ti) × 誤差 × dt` ずつ足し込んでいく。

```cpp
namespace {
// ...
constexpr float kRollTi = 0.5f;   // [s] integral time — smaller = faster catch-up
}  // namespace
```

```cpp
// app_controller.hpp に追加
private:
    float integral_roll_ = 0.0f;   // roll axis integral accumulator
```

```cpp
// compute() 内、P項の計算に続けて
integral_roll_ += (kRollKp / kRollTi) * error_roll * dt;
// Clamp the integral itself to the output limit — a simple anti-windup.
// 積分値そのものを出力上限でクランプする——素朴なアンチワインドアップ。
if (integral_roll_ >  kMaxRollPitchTorqueNm) integral_roll_ =  kMaxRollPitchTorqueNm;
if (integral_roll_ < -kMaxRollPitchTorqueNm) integral_roll_ = -kMaxRollPitchTorqueNm;

float torque_roll = kRollKp * error_roll + integral_roll_;
if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;
```

`reset()` では積分値も忘れずにゼロへ戻す。

```cpp
void AppController::reset()
{
    cycle_count_ = 0;
    integral_roll_ = 0.0f;
}
```

積分値をリセットし忘れると、前回の飛行で溜まった値が次回のARM直後にいきなり出力され、機体が予期せず傾く。`reset()` はARM時に必ず呼ばれるので、積分器を持つ変数は必ずここで初期化する。

### ワインドアップという落とし穴

素朴に積分を足し続けるだけだと、出力がすでに上限（クランプ）で頭打ちになっている間も積分値だけはどんどん大きくなり続けてしまう——これを**ワインドアップ**（積分の「巻き上がり」）と呼ぶ。誤差の符号が反転しても、巻き上がった積分値がゆっくりとしか戻らないため、出力が反対側に大きく振れて**オーバーシュート**（行き過ぎ）する。

上のコードでは「積分値そのものを出力上限にクランプする」という素朴な方法で対策している。これは完全な対策ではない（出力が上限に張り付いている間も、積分値の上限までは巻き上がってしまう）が、まず動かして効果を体感するには十分である。

**もっと踏み込みたい人へ**: このプロジェクトの実機用コントローラ（`firmware/vehicle/components/sf_controller_pid/include/pid.hpp`）は「条件付き積分」という、出力が飽和方向に押されている間だけ積分の更新を止める、より正確な方式を使っている。素朴な方法との違いを読み比べてみるとよい。

## 9. 微分項(D)を足す

### なぜこの一歩が必要か

PI制御（比例＋積分）は定常偏差を消せるが、目標値が急に変わった瞬間の「行き過ぎ」を抑える働きが弱い。目標に向かって近づく速度そのものにブレーキをかける項——**微分制御（D制御）**を足す。

ここでも実機確認はプロペラを外すか固定した状態、かつSILSで先に確認してから、という7章の注意事項を守る。微分項はセンサのノイズを増幅しやすく、ゲインの選び方によっては高周波の振動が急に出ることがある。

### 微分キックという落とし穴

素直に「誤差の微分」を使うと、目標値（スティック値）が階段状に変化するたびに、誤差も一瞬で大きく変化し、微分が跳ね上がってしまう——これを**微分キック**と呼ぶ。ACROのレート目標はスティックの12bit値からそのまま作られるため、目標値は常に細かく階段状に変化しており、誤差の微分をそのまま使うと常にノイズだらけの指令になってしまう。

対策は単純で、**「誤差」ではなく「測定値」を微分する**（D-on-Measurement）。目標値がどれだけ変化したかではなく、機体の回転が実際にどれだけの速さで変化しているかだけを見る。目標値のステップはこの経路を素通りしないので、微分キックが起きない。

### 実装

```cpp
namespace {
// ...
constexpr float kRollTd = 0.001f;   // [s] derivative time
}  // namespace
```

```cpp
// app_controller.hpp に追加
private:
    float prev_measured_roll_ = 0.0f;
    bool  roll_first_sample_  = true;   // primes the derivative after reset()
```

```cpp
// compute() 内
float d_term_roll = 0.0f;
if (!roll_first_sample_) {
    const float measured_rate_of_change = (state.angular_rate[0] - prev_measured_roll_) / dt;
    d_term_roll = -kRollKp * kRollTd * measured_rate_of_change;
}
prev_measured_roll_ = state.angular_rate[0];
roll_first_sample_  = false;

float torque_roll = kRollKp * error_roll + integral_roll_ + d_term_roll;
if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;
```

`reset()` では `prev_measured_roll_` と `roll_first_sample_` も初期化する。`roll_first_sample_` のガードが無いと、リセット直後は「前回の測定値」が古い（あるいは無意味な初期値の）ままなので、リセット直後の1回だけ不自然な微分キックが出てしまう。

これでP・I・Dの3項がそろった。**もっと踏み込みたい人へ**: 微分項は測定ノイズ（ジャイロの細かい揺らぎ）をそのまま増幅してしまう弱点がある。`pid.hpp` の実装は微分にローパスフィルタ（`eta` という係数で高周波を削る「不完全微分」）を追加しており、実機のノイズ環境ではこちらの方が滑らかに効く。

## 10. 3軸そろえて本物のACROコントローラにする

### なぜこの一歩が必要か

ここまではロール軸だけで練習してきた。ピッチ・ヨーも仕組みは全く同じであり、3軸分のPID状態をコピー&ペーストで持つのではなく、**「1軸分のPID状態」を1つの部品（構造体）にまとめ、3つ作る**方が読みやすく、間違いにくい。あわせて、`reset()`/`onModeChange()` をきちんと実装し、実際に飛行検証されたゲイン値を入れて、本物のACROコントローラに仕上げる。

### 1軸分のPID状態をまとめる

```cpp
// app_controller.hpp
#pragma once
#include "controller.hpp"

namespace sf::app {

/// One axis' rate-loop PID state (P + simple clamped I + measurement D).
/// 1軸分のレートループPID状態（P + 素朴なクランプ付きI + 測定値D）。
struct RateAxisPid {
    float kp = 0.0f;
    float ti = 0.0f;
    float td = 0.0f;
    float output_limit = 0.0f;

    float integral = 0.0f;
    float prev_measurement = 0.0f;
    bool first_sample = true;

    float compute(float setpoint, float measurement, float dt);
    void reset();
};

class AppController : public sf::IController {
public:
    AppController();

    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;
    void onLanding() override;

private:
    RateAxisPid roll_pid_;
    RateAxisPid pitch_pid_;
    RateAxisPid yaw_pid_;

    bool  landing_active_       = false;
    float landing_elapsed_s_    = 0.0f;
    float landing_thrust_start_ = 0.0f;
    float last_thrust_          = 0.0f;
};

}  // namespace sf::app
```

`RateAxisPid::compute()` の中身は、7〜9章で組み立てたロール軸のP・I・D計算をそのまま軸に依存しない形にしたものである。

```cpp
// app_controller.cpp
#include "app_controller.hpp"

namespace sf::app {

namespace {
// Named constants — every value is either a physical limit of this frame
// (thrust/torque caps) or a flight-proven ACRO rate-loop gain.
// 名前付き定数——推力・トルクの上限はこの機体の物理限界、それ以外は
// 飛行実績のあるACROレートループゲイン。
constexpr float kMaxRollPitchRateRadS = 1.0f;
constexpr float kMaxYawRateRadS       = 5.0f;
constexpr float kMaxThrustN           = 0.672f;
constexpr float kMaxRollPitchTorqueNm = 5.2e-3f;
constexpr float kMaxYawTorqueNm       = 1.226e-3f;

constexpr float kRollKp = 1.0e-3f,       kRollTi = 0.7f,  kRollTd = 0.002f;
constexpr float kPitchKp = 1.426432e-3f, kPitchTi = 0.7f, kPitchTd = 0.025f;
constexpr float kYawKp = 8.029796e-4f,   kYawTi = 0.8f,   kYawTd = 0.01f;

constexpr float kLandingDescentS = 3.0f;   // §11 で使う降下時間
}  // namespace

float RateAxisPid::compute(float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    const float p_term = kp * error;

    if (ti > 0.0f) {
        integral += (kp / ti) * error * dt;
        if (integral >  output_limit) integral =  output_limit;
        if (integral < -output_limit) integral = -output_limit;
    }

    float d_term = 0.0f;
    if (td > 0.0f && !first_sample) {
        const float measurement_rate = (measurement - prev_measurement) / dt;
        d_term = -kp * td * measurement_rate;
    }
    prev_measurement = measurement;
    first_sample = false;

    float output = p_term + integral + d_term;
    if (output >  output_limit) output =  output_limit;
    if (output < -output_limit) output = -output_limit;
    return output;
}

void RateAxisPid::reset()
{
    integral = 0.0f;
    prev_measurement = 0.0f;
    first_sample = true;
}

AppController::AppController()
    : roll_pid_{kRollKp,  kRollTi,  kRollTd,  kMaxRollPitchTorqueNm},
      pitch_pid_{kPitchKp, kPitchTi, kPitchTd, kMaxRollPitchTorqueNm},
      yaw_pid_{kYawKp,    kYawTi,   kYawTd,   kMaxYawTorqueNm}
{
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    sf::ControlOutput output{};
    output.timestamp = state.timestamp;

    float rate_sp_roll  = setpoint.roll  * kMaxRollPitchRateRadS;
    float rate_sp_pitch = setpoint.pitch * kMaxRollPitchRateRadS;
    float rate_sp_yaw   = setpoint.yaw   * kMaxYawRateRadS;
    float thrust        = setpoint.throttle * kMaxThrustN;

    if (landing_active_) {
        // See §11 — comm-loss / battery-emergency descent.
        rate_sp_roll = rate_sp_pitch = rate_sp_yaw = 0.0f;
        landing_elapsed_s_ += dt;
        float ramp = 1.0f - (landing_elapsed_s_ / kLandingDescentS);
        if (ramp < 0.0f) ramp = 0.0f;
        thrust = landing_thrust_start_ * ramp;
    }

    output.torque[0] = roll_pid_.compute(rate_sp_roll,  state.angular_rate[0], dt);
    output.torque[1] = pitch_pid_.compute(rate_sp_pitch, state.angular_rate[1], dt);
    output.torque[2] = yaw_pid_.compute(rate_sp_yaw,    state.angular_rate[2], dt);
    output.thrust = thrust;

    // Export the rate targets for later analysis (sf log analyze/viz).
    // ACRO has no angle (attitude) loop, so angle_ref stays at 0.
    // 解析用にレート目標を出力（sf log analyze/viz）。ACROには角度ループが
    // ないので angle_ref は0のまま。
    output.rate_ref[0] = rate_sp_roll;
    output.rate_ref[1] = rate_sp_pitch;
    output.rate_ref[2] = rate_sp_yaw;
    output.angle_ref[0] = 0.0f;
    output.angle_ref[1] = 0.0f;

    last_thrust_ = thrust;
    return output;
}

void AppController::reset()
{
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_pid_.reset();
    landing_active_ = false;
    landing_elapsed_s_ = 0.0f;
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    // This tutorial controller only implements ACRO. A controller covering
    // more modes would reconfigure its cascade here (see PidController).
    // このコントローラはACRO専用。複数モードに対応する制御器は、ここで
    // カスケード構成を再構成する（本物のPidController参照）。
    (void)new_mode;
}

}  // namespace sf::app
```

`onLanding()` の実装は次章でまとめて説明する。

### ゲイン値について

| 軸 | kp | ti [s] | td [s] | 出力上限 |
|----|----|--------|--------|---------|
| ロール | 1.0e-3 | 0.7 | 0.002 | ±5.2e-3 Nm |
| ピッチ | 1.426432e-3 | 0.7 | 0.025 | ±5.2e-3 Nm |
| ヨー | 8.029796e-4 | 0.8 | 0.01 | ±1.226e-3 Nm |

これらは、この機体（StampFly）で実際に飛行検証済みの目安値である。ただし、この記事のPID実装（積分の素朴なクランプ、微分にフィルタなし）は、本物の `PidController`（Tustin法による双一次変換・条件付き積分アンチワインドアップ）とは離散化の方式が異なるため、挙動が完全に一致するわけではない。400Hzという十分に速い制御周期のもとでは実用上の差は小さいが、シビアに詰めたい場合は `pid.hpp` の実装を参考にしてほしい。

3軸すべてが閉じたことで、このコントローラは本当に自由飛行できるだけの力を持つ。実機で試す際は、7章の安全上の注意（プロペラを外すか機体をしっかり固定する、SILS→実機ベンチの順で確認する、緊急停止をいつでも実行できる態勢にする）を、これまで以上に徹底すること。実際に自由飛行させるのは、次の11章・12章で安全機構と飛行前チェックリストを確認してからにする。

## 11. 安全機構と `onLanding()`

### なぜこの一歩が必要か

送信機との通信が途切れたらどうなるか。ファームウェアには、通信途絶を検知してから**3秒間は最後の指令のままホバーを試み、それでも復帰しなければ自動的に「着陸」状態へ遷移する**フェイルセーフが既に組み込まれている（電池電圧が危険域まで下がった場合も同様に着陸へ移行する）。この着陸状態に入ると、ファームウェアは自作コントローラの `onLanding()` を1回呼び出す。

**もしこれを実装しないままにすると**、コントローラは「最後に受け取ったスティック値」をいつまでも目標として使い続けてしまう。通信が切れた瞬間のスティックが例えば「前進しながら回転」だった場合、機体はその指令のまま飛び続け、止まる手段がなくなる。`onLanding()` の実装は、この記事のコントローラで省略してはならない安全機構である。

### 実装方針（正直な単純化）

このコントローラはACRO専用で角速度しか制御しておらず、**姿勢（傾き）を検出・補正する仕組みを持たない**。したがって「機体を水平に戻す」ことは原理的にできない——できるのは「それ以上回転させない」ことと「推力を落として静かに降ろす」ことだけである。この記事では、次の単純で正直な方針を取る。

- 目標角速度をロール・ピッチ・ヨーすべてゼロにする（スティック入力を無視し、それ以上の回転を止める。すでにほぼ水平だった場合は、結果的に水平に近い状態を保ったまま降下できる）
- `onLanding()` が呼ばれた瞬間の推力を基準に、一定時間（この記事では3秒、`kLandingDescentS`）かけて推力を線形にゼロまで下げる

`onLanding()` は着陸開始の合図をもらうだけの関数で、状態（今の推力）を直接は受け取れない。そこで、`compute()` の最後に「直前に指令した推力」を毎回 `last_thrust_` に保存しておき、`onLanding()` はその値を降下の初期値として使う。

```cpp
void AppController::onLanding()
{
    landing_active_ = true;
    landing_elapsed_s_ = 0.0f;
    landing_thrust_start_ = last_thrust_;
}
```

`landing_active_` は次の `reset()`（次回ARM）で解除される——`IController::onLanding()` の設計上の約束事であり、上の `reset()` の実装で `landing_active_ = false` としているのはこのためである。

この単純な実装には限界がある。着陸開始時にすでに大きく傾いていた場合、傾いたまま降下することになる。より踏み込んだ実装（例えば加速度センサから簡易的な傾き推定を足す等)は、この記事の範囲を超えるため扱わない——まずは「何もしないよりずっとまし」な、正直で単純な安全策を必ず入れる、ということを覚えてほしい。

この着陸則も、いきなり実機の自由飛行中に送信機の電源を切って試したりしない。まずSILSの通信途絶シナリオで `onLanding()` が呼ばれ、推力が意図通りに下がっていくことをログで確認する。実機で確かめる場合も、プロペラを外すか機体を固定した状態で、`onLanding()` が呼ばれた瞬間の挙動（回転が止まり推力が落ちていくこと）を確認するところから始める。

## 12. SILSで確認してから実機で飛ばす

### 手順

1. **SILSで確認する。**

   ```bash
   sf app sils my_acro simulator/sils/scenarios/acro_flight.scn
   ```

   終了コード0（PASS）になることを確認する。

2. **実機向けにビルドし、書き込む。**

   ```bash
   sf app build my_acro
   sf app flash my_acro -m
   ```

3. **飛行前チェックリスト**（詳細は `docs/guides/safety.md`）を必ず確認する。

   | # | 確認項目 |
   |---|---------|
   | 1 | プロペラガードに破損・変形がない |
   | 2 | プロペラに損傷がない |
   | 3 | バッテリーが十分に充電されている（30%以上） |
   | 4 | 飛行エリアが確保されている（2m×2m以上、障害物なし） |
   | 5 | 周囲に人がいない |
   | 6 | 緊急停止方法（`sf emergency` または送信機の緊急停止）を確認した |

4. 初回飛行は、想定外の挙動が起きやすい。安定して飛ぶと確認できるまでは、緊急停止をすぐに実行できる態勢で臨む。

5. 飛行後はログを取得して確認する。

   ```bash
   sf log wifi -d 30
   sf log analyze
   ```

   `sf log analyze` はジャイロ統計・入力と応答の相関・振動の周波数解析などを表示してくれる。グラフで見たい場合は `sf log viz` を使う。

## 13. ゲインを自分で調整するには・次のステップ

この記事の `kRollKp` 等の定数をソースコードに埋め込んだままでは、ゲインを変えるたびに再ビルド・再書き込みが必要になる。ファームウェアには **NVS**（Non-Volatile Storage。電源を切っても値が消えない保存領域）を使ったパラメータの仕組みがすでにあり、`param set` コマンドで飛行中にゲインを変えながら試すこと（ライブチューニング）ができる。自分のゲインをこの仕組みに乗せる場合は、`firmware/vehicle/components/sf_core/params.cpp` の `rate.roll.*` 等の登録パターンと、`firmware/vehicle/components/sf_controller_pid/pid_controller.cpp` の `loadParams()`/`reloadParams()` を読み、同じパターンで自分の定数をパラメータ化するとよい。

### この先の道筋

ACROモード（角速度制御）は、vehicle ファームが持つ制御則の中でもっとも単純なものである。この先には、機体の傾き（姿勢）を推定する**姿勢推定**、その推定値をもとに傾きそのものを目標へ追従させる**姿勢制御**、そしてPIDのような古典的な手法とは考え方の異なる**LQR/LQI**（線形二次レギュレータ／線形二次積分型制御）のような制御則が続く見通しである。これらはまだ記事になっていないが、いずれ本記事の続編として整備される予定である。全体の学習の道筋は `firmware/vehicle/docs/coding_and_education.md` の教育ロードマップにまとまっている。

さらに深掘りしたい場合は、以下を参照してほしい。

| 文書 | 内容 |
|------|------|
| [`firmware/vehicle/components/sf_controller_pid/include/pid.hpp`](../../firmware/vehicle/components/sf_controller_pid/include/pid.hpp) | 本物のPID実装（条件付き積分・Tustin法による不完全微分） |
| [`firmware/vehicle/docs/topic_reference.md`](../../firmware/vehicle/docs/topic_reference.md) | トピック一覧のSSOT（Single Source of Truth）、使用パターン |
| [`docs/guides/safety.md`](safety.md) | 飛行安全ガイド |

---

<a id="english"></a>

## 1. What You Are Building, Who This Guide Is For, and Prerequisites

### The Goal of This Guide

Deflect a transmitter stick and the craft rotates in that direction, at that speed — ACRO mode (rate control mode). This guide builds ACRO mode's control program from the very first line, **without relying at all on the existing `PidController`** (the cascade PID controller already built into this project).

Concretely, by the end you will have written code that does the following:

- Deflecting the roll stick makes the craft keep rotating on the roll axis by a matching amount (full deflection reaches the maximum angular rate; releasing the stick stops the rotation)
- Pitch and yaw work the same way
- The throttle stick directly commands thrust (how hard the craft lifts)
- If the link to the transmitter is lost, the craft lands slowly and safely instead of running away

You will build this up one chapter at a time, ending with feedback control closed on all three axes (roll, pitch, yaw) — a controller that can genuinely fly the craft.

This guide is also a first step toward eventually being able to write the entire vehicle firmware yourself. ACRO-mode PID control is the simplest control law this firmware has; getting comfortable here with Pub/Sub, Topics, and `IController` gives you the same foundation for whatever control law you write next — attitude control, or a different family of control law entirely (see the LQR/LQI mentioned in section 13).

### Who This Guide Is For

- You know basic C++ syntax (`if`/`for`/structs/classes)
- You are not yet familiar with "Pub/Sub," "namespaces," or the conventions of embedded development
- A passing familiarity with introductory control theory (what the proportional, integral, and derivative terms do) is enough — each concept is explained in the text as it comes up

### Prerequisites

This guide assumes you have already completed the "Fly the Real Drone" section of the top-level `README.md` — that is, you have flashed the stock firmware and flown it once with the transmitter. If you have not done that yet, finish it before continuing here.

This guide covers the **L1 Topic API** of `firmware/vehicle` (the vehicle firmware — the primary firmware used both on real hardware and in SILS, short for Software In the Loop Simulation, a way to run the firmware itself on a PC): the `sf::api::` namespace, together with the swappable `IController` interface (a contract that says "implement these functions and the internals are up to you"). Writing the sensor-driver / hardware-initialization layer itself (HAL/BSP) is out of scope.

## 2. What Is Pub/Sub? What Is a Namespace?

### Think of It as a Bulletin Board

Inside this firmware, components never call each other's functions directly. Instead, they communicate through a **Topic** (a typed container that holds exactly one value) using a **Pub/Sub** pattern (Publish = post, Subscribe = read).

A bulletin board is a good mental model.

- **`publish()` (post)** re-pins a fresh sheet of paper to the board. It takes down whatever was pinned before and puts up exactly one new sheet. Whoever posts has no idea who, if anyone, will come read it.
- **`latest()` (peek at the newest value)** is walking past the board and copying down whatever is currently pinned. Whoever reads has no idea who posted it. Reading never uses up the sheet — you can peek as many times as you like and always get the same answer until the next post.
- There is also **`read()` (take one item)**, which behaves like pulling a single letter out of a mailbox — once you take it, it is gone, and the next reader will not see it. Each Topic is designed to work one way or the other, "bulletin board" or "mailbox."

A sensor task posts a value to a "sensor reading" board; an estimation/control task peeks at it, computes something, and posts the result to a different board (say, "control output"). The code you are about to write becomes one link in that relay — either a task that only watches a board, or the estimation/control step itself.

### A Namespace Is "Which Floor of the Building"

The `sf::api::` in something like `sf::api::estimate_latest()` is a **namespace** — a label, like a street address, that keeps two functions with the same name from colliding. You don't need to overthink it: think of it as "the function that lives on the `api` floor of the `sf` building." Every function this guide uses lives on that `sf::api::` floor (the Topic API published for learners).

### The Big Picture for This Guide

The code you write here reads input (stick and sensor values) exactly the way just described — calling `sf::api::something_latest()` and peeking at a board. Writing output (thrust and torque commanded to the craft) works a bit differently. Instead of calling `publish()` yourself, you fill in a fixed-shape "box" called `IController` and return it. Something else — a part of the firmware called `ControlTask` — is the one that actually walks that box over and pins it to the board. Section 3 explains why.

## 3. The Big Picture of ACRO Mode

ACRO-mode control is one straight pipeline:

```
Stick (throttle / roll / pitch / yaw, from the transmitter)
        │
        ▼
target angular rate = stick value (roll/pitch/yaw) x max angular rate
target thrust        = throttle stick value x max thrust
        │
        ▼
error = target angular rate − measured angular rate (from the gyro)
        │
        ▼
PID computation (proportional + integral + derivative) -> torque per axis
        │
        ▼
ControlOutput { thrust, torque[3] (roll/pitch/yaw torque) }
        │            <- everything up to here is what you write in this guide
        ▼
sf_actuator automatically mixes it (converts it into duty for the 4 motors)
        │
        ▼
      the 4 motors
```

**Mixing** (the calculation that turns the overall thrust/torque command into per-motor spin commands for the 4 motors) is handled automatically by a component called `sf_actuator`. How many motors there are, where each one sits, which way it spins — none of that hardware-level detail is something you need to think about here. What you write stops at "how much total thrust and torque the craft should produce."

Everything below `ControlOutput` in the diagram is the Pub/Sub example from section 2 in action. `ControlTask` takes the `ControlOutput` your `compute()` returns and `publish()`es it to a Topic named `control_output`. `sf_actuator` subscribes to `control_output`; every time a new value is posted, it mixes it into duty (PWM on-time ratio) for the 4 motors. Your code (the `IController`) and `sf_actuator` never know about each other directly — they are connected only through the single `control_output` board.

### Why Not Call `publish()` Yourself?

As noted in section 2, this guide's code reads values (e.g. `sf::api::estimate_latest()`) by peeking at a board directly, but it does not walk up and pin its own result to a board when returning a value from `compute()`. There is a reason for that split.

- **Exactly one poster per board.** The job of actually pinning a new sheet to the `control_output` board belongs to exactly one place: `ControlTask`. If several places could post to the same board, "whichever posted last wins" would create a race. Implementing `IController` (as you are doing here) makes you the one who computes "what the next sheet should say" — not the one who walks over and pins it. That is the entire reason `compute()` returns a plain `ControlOutput` value instead of calling `publish()`.
- **There is bookkeeping before and after the post.** `ControlTask` does more than pin the board: in the same cycle it also assembles a logging record (the Data Stream) and collects system-identification results. Centralizing the "post" responsibility in `ControlTask` means you never have to duplicate that bookkeeping inside every custom controller.
- **You can use it without knowing anything about Pub/Sub.** `compute()` is just a function call; what the caller does with the return value is entirely up to the caller. In fact, the standalone bench-style examples (such as `10_custom_controller`) call `compute()` directly against a synthetic signal, with no Pub/Sub machinery present at all — possible only because the controller itself is a fixed-shape box that never depends on Pub/Sub.

## 4. Setting Up Your Development Environment

Start by creating your own project.

```bash
source setup_env.sh
```

```bash
sf app new my_acro
```

This creates the following files under `firmware/apps/my_acro/`:

| File | Role |
|------|------|
| `app.yaml` | Project metadata, including the type (`type: embedded` — compiled directly into the vehicle firmware) |
| `app.cpp` | The "seam" that registers the class you'll write (via `sf::app::controller()`, etc.) with the firmware |
| `app_controller.hpp` / `app_controller.cpp` | Where your `IController` implementation lives. **This is the file you keep editing throughout this guide** |
| `README.md` | The source template's own explanation (leave as-is) |

`type: embedded` means this project's source is compiled directly into the vehicle firmware's own build, and the exact same source runs on real hardware and in SILS. You never create a new component or edit any build configuration by hand — the `sf app` command handles all of that.

Inside `app_controller.cpp` there is a function called `compute()`, called at **400 Hz (400 times per second, once every 2.5 milliseconds)**. Nearly everything you do in this guide is rewriting the body of `compute()`. The caller (`ControlTask`) keeps calling it every cycle regardless of whether the craft is currently ARMed (permitted to spin motors) — the actual motor safety gate lives elsewhere (ARM-state management) — so write `compute()` assuming it is always being called.

2.5 milliseconds is not much time. Inside `compute()`, follow these rules:

- No **dynamic memory allocation** (`new`/`malloc`, i.e. allocating memory during execution) — allocation time is unpredictable and can blow the period
- Do not call heavy logging (like `printf`) unconditionally every cycle — section 5 shows how to throttle it
- No **blocking calls** that stall waiting for a response

Every code example in this guide follows these rules (nothing but fixed-size structs and `float` arithmetic — no heap allocation ever appears).

From here on, later chapters show the **diff** against the previous chapter instead of re-pasting the whole `app_controller.hpp`/`.cpp` from scratch. When you actually follow along, open the files with something like `sf app edit my_acro` and edit them there.

## 5. First, Just Peek at the Angular Rate

### Why This Step Matters

Before writing any control law that spins motors, first confirm that "the sensor value is visible from my own code." This step sends no output at all (`ControlOutput` is always zero) and just logs the angular rate measured by the gyro.

### Implementation

`sf::api::estimate_latest()`, introduced in section 2, returns the latest value of the state estimate (the "current state of the craft," inferred by combining sensor readings) — a "peek at the board" function. Its return type, `StateEstimate`, has an `angular_rate[3]` field: the angular rate (in rad/s) in the body frame (FRD: X axis = forward and the roll axis, Y axis = right and the pitch axis, Z axis = down and the yaw axis). The indices are stored in order `[0] = roll`, `[1] = pitch`, `[2] = yaw`.

Inside `compute()`, though, you never need to call this function yourself. The first parameter, `state`, is already exactly the same value as `sf::api::estimate_latest()` — `ControlTask` peeks at the "sensor value" board on your behalf every cycle (`estimate_state.latest()`) and hands you the result as a function argument. So inside `compute()`, just use `state.angular_rate` directly. `sf::api::estimate_latest()` earns its keep outside `compute()` — for example, in an additional task that peeks at Topics on its own, without going through `IController`/`IEstimator` (not covered in this guide).

```cpp
// app_controller.hpp
#pragma once
#include "controller.hpp"

namespace sf::app {

class AppController : public sf::IController {
public:
    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;

private:
    uint32_t cycle_count_ = 0;   // compute() の呼び出し回数 / call counter
};

}  // namespace sf::app
```

```cpp
// app_controller.cpp
#include "app_controller.hpp"
#include "esp_log.h"

namespace sf::app {

namespace {
constexpr const char* kLogTag = "MyAcro";
// compute() is called at 400 Hz; logging every call would overrun the
// control period (coding_and_education.md §7). Divide down to ~1 Hz.
// compute()は400Hzで呼ばれる。毎回ログを出すと制御周期を超過するため
// （coding_and_education.md §7）、約1Hzまで間引く。
constexpr uint32_t kLogEveryNCycles = 400;
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)setpoint;
    (void)dt;

    ++cycle_count_;
    if (cycle_count_ % kLogEveryNCycles == 0) {
        // angular_rate[3] = roll, pitch, yaw [rad/s], body frame (FRD)
        // angular_rate[3] = ロール・ピッチ・ヨー [rad/s]、機体座標系(FRD)
        ESP_LOGI(kLogTag, "rate roll=%.3f pitch=%.3f yaw=%.3f",
                 state.angular_rate[0], state.angular_rate[1], state.angular_rate[2]);
    }

    // Not yet touching thrust/torque — a zero-initialized output keeps the
    // motors off no matter what the sticks say.
    // まだ推力・トルクには触れない——ゼロ初期化した出力を返せば、スティックの
    // 値に関わらずモータは回らない。
    sf::ControlOutput output{};
    output.timestamp = state.timestamp;
    return output;
}

void AppController::reset()
{
    cycle_count_ = 0;
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    (void)new_mode;   // このコントローラはまだ何もしない / not used yet
}

}  // namespace sf::app
```

`compute()`, `reset()`, and `onModeChange()` are all declared as "pure virtual functions" inside `IController` (functions with no default body — you must implement them or the code will not compile). It's fine for the bodies to be empty for now; just write all three.

Calling `ESP_LOGI` unconditionally at 400 Hz would eat into the 2.5 ms budget through logging latency alone. That's why `cycle_count_` throttles it down to roughly 1 Hz — this throttling pattern carries forward into every later chapter.

### Try It

Run it in SILS (a PC-side simulation that models flight without real hardware) and confirm the angular rate shows up in the log.

```bash
sf app sils my_acro
```

Watch `sf log analyze`, or the log scrolling in the monitor, while you tilt the craft in SILS — the angular-rate numbers should change.

## 6. Turning Stick Input Into a Target Angular Rate

### Why This Step Matters

Control is the business of driving the difference (the error) between a "target" and a "current value" to zero. The previous chapter got you the "current value" (the measured angular rate). Next comes the "target" — building a target angular rate from how far the sticks are deflected.

### Implementation

The pilot's command (stick values) is available via `sf::api::command_latest()` — but for the same reason as the angular rate in the previous chapter, you don't need to call it inside `compute()`. The second parameter, `setpoint` (a `CommandSetpoint`), is exactly the same value that `ControlTask` already fetched via `sf::api::command_latest()` on your behalf. `roll`/`pitch`/`yaw` range over `-1..1` (full deflection is ±1, center is 0), and `throttle` ranges over `0..1` (center is 0 = zero thrust, full deflection is 1 = maximum thrust).

In ACRO mode, how far the stick is deflected directly becomes the "target angular rate." The constant that decides how fast the craft spins at full deflection is the "max angular rate." We use 1.0 rad/s for roll/pitch and 5.0 rad/s for yaw as our targets — values with real flight history on this frame.

```cpp
// app_controller.cpp（compute()の中身を置き換え）
namespace {
constexpr const char* kLogTag = "MyAcro";
constexpr uint32_t kLogEveryNCycles = 400;

// Named constants (no magic numbers): how fast the craft spins at full
// stick deflection. Flight-proven values for this frame.
// 名前付き定数（マジックナンバー禁止）: スティックを倒し切ったときの回転速度。
// この機体で飛行実績のある値。
constexpr float kMaxRollPitchRateRadS = 1.0f;   // [rad/s]
constexpr float kMaxYawRateRadS       = 5.0f;   // [rad/s]
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)dt;

    // Stick -> target angular rate. Still not fed into the output below.
    // スティック → 目標角速度。まだ下の出力には使わない。
    const float rate_sp_roll  = setpoint.roll  * kMaxRollPitchRateRadS;
    const float rate_sp_pitch = setpoint.pitch * kMaxRollPitchRateRadS;
    const float rate_sp_yaw   = setpoint.yaw   * kMaxYawRateRadS;

    ++cycle_count_;
    if (cycle_count_ % kLogEveryNCycles == 0) {
        ESP_LOGI(kLogTag,
                 "target roll=%.3f pitch=%.3f yaw=%.3f | measured roll=%.3f pitch=%.3f yaw=%.3f",
                 rate_sp_roll, rate_sp_pitch, rate_sp_yaw,
                 state.angular_rate[0], state.angular_rate[1], state.angular_rate[2]);
    }

    sf::ControlOutput output{};   // still zero output — no motor spin yet
    output.timestamp = state.timestamp;
    return output;
}
```

Move the sticks (in SILS or on the transmitter) and confirm the "target" column in the log moves. The "measured" column (the actual angular rate) does not chase this target yet — making it chase is the next chapter's job.

## 7. Driving It With Proportional (P) Control Alone

### Why This Step Matters

Now that you have both a "target" and a "current value," it's time to issue a motor command that drives the difference (the error) between them to zero. The simplest way is **proportional control (P control)** — return a force directly proportional to the size of the error. This is the first time `compute()` returns a `ControlOutput` with anything nonzero in it.

### Safety Notice (Read Before Proceeding)

From here on, nonzero commands actually reach the motors. Follow these rules without exception.

- **Always remove the propellers, or firmly restrain the craft, before trying this on real hardware.** Pitch and yaw are not yet under control (they're added in later chapters), and even roll uses an untuned value — the craft can behave erratically.
- Always confirm in this order: **SILS -> real-hardware bench (props off, or the craft restrained)**. Never go straight to free flight.
- Be ready to execute the emergency stop (`sf emergency`, or the transmitter's emergency stop — see `docs/guides/safety.md`) at any moment.

### Implementation (Roll Axis Only)

Start by building the PID control for the roll axis alone. Pitch and yaw still return 0 (i.e., they remain open-loop — closed in a later chapter).

Define the error as "target angular rate minus measured angular rate," multiply by a proportional gain `kp`, and use that directly as the torque command.

```cpp
namespace {
// ... (constants from the previous chapter, unchanged) ...

// Roll rate loop, P-only stage. A modest first guess — not yet tuned.
// ロールレートループ、P制御のみの段階。まだ追い込んでいない控えめな初期値。
constexpr float kRollKp = 3.0e-4f;   // [Nm / (rad/s)]

// Physical torque limit of this frame's roll/pitch axis — a safety bound,
// not a tuning knob (see sf_controller_pid's max_roll_pitch_torque_).
// この機体のロール/ピッチ軸トルクの物理上限——チューニング値でなく安全上限
// （sf_controller_pidのmax_roll_pitch_torque_と同じ値）。
constexpr float kMaxRollPitchTorqueNm = 5.2e-3f;   // [Nm]
constexpr float kMaxThrustN           = 0.672f;    // [N] 4 motors combined
}  // namespace

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    (void)dt;

    const float rate_sp_roll = setpoint.roll * kMaxRollPitchRateRadS;
    const float error_roll   = rate_sp_roll - state.angular_rate[0];

    float torque_roll = kRollKp * error_roll;
    if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
    if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;

    sf::ControlOutput output{};
    output.timestamp = state.timestamp;
    output.torque[0] = torque_roll;   // roll
    output.torque[1] = 0.0f;          // pitch — still open-loop, closed in §10
    output.torque[2] = 0.0f;          // yaw   — still open-loop, closed in §10
    output.thrust     = setpoint.throttle * kMaxThrustN;
    return output;
}
```

`kMaxRollPitchTorqueNm` (5.2e-3 Nm) is this frame's physical limit on how much torque it makes sense to command — a safety bound, not a tuning parameter. Build it into the clamp from the very start.

### Confirm in SILS, Then on a Real-Hardware Bench

```bash
sf app sils my_acro
```

Once that looks right, confirm on real hardware with the props off (or the craft restrained).

```bash
sf app build my_acro
sf app flash my_acro -m
```

Deflect the roll stick and the roll torque command (visible via `sf log wifi`, for example) should track it. You'll notice, though, that it never quite reaches the target angular rate — a small **steady-state error** (the error that remains no matter how long you wait) persists. This happens because a small disturbance (motor reaction torque, wiring asymmetry, part-to-part variation) is always present, and P control alone settles at whatever point exactly balances that disturbance (call the disturbance torque `d` and the proportional gain `kp`; at steady state, `kp x error ≈ d` — raising `kp` shrinks the error, but push it too far and you get oscillation). Eliminating this steady-state error is the integral term's job, in the next chapter.

## 8. Adding the Integral Term (I)

### Why This Step Matters

P control reacts only to "the error right now." As long as some error remains, even a small one, you want a term that keeps working on it, little by little, over time — that's **integral control (I control)**. It accumulates the error over time and adds a force proportional to that accumulated value.

The safety notice from chapter 7 (props off or the craft restrained, confirm SILS before real hardware) still applies in full when you check the integral term on real hardware.

### Implementation

The strength of the integral action is expressed as a time constant, the "integral time `Ti`" (a shorter `Ti` means the integral catches up faster). Each cycle, accumulate `(kp/Ti) x error x dt`.

```cpp
namespace {
// ...
constexpr float kRollTi = 0.5f;   // [s] integral time — smaller = faster catch-up
}  // namespace
```

```cpp
// app_controller.hpp に追加
private:
    float integral_roll_ = 0.0f;   // roll axis integral accumulator
```

```cpp
// compute() 内、P項の計算に続けて
integral_roll_ += (kRollKp / kRollTi) * error_roll * dt;
// Clamp the integral itself to the output limit — a simple anti-windup.
// 積分値そのものを出力上限でクランプする——素朴なアンチワインドアップ。
if (integral_roll_ >  kMaxRollPitchTorqueNm) integral_roll_ =  kMaxRollPitchTorqueNm;
if (integral_roll_ < -kMaxRollPitchTorqueNm) integral_roll_ = -kMaxRollPitchTorqueNm;

float torque_roll = kRollKp * error_roll + integral_roll_;
if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;
```

Don't forget to zero the integral in `reset()` as well.

```cpp
void AppController::reset()
{
    cycle_count_ = 0;
    integral_roll_ = 0.0f;
}
```

If you forget to reset the integral, whatever value built up during the previous flight fires the instant the craft ARMs again, tilting it unexpectedly. `reset()` is always called on ARM, so any variable holding integrator state must be zeroed there.

### The Windup Trap

If you naively keep accumulating the integral, it keeps growing even while the output is already pinned at its clamp — this is called **windup** (the integral "winding up"). When the error's sign flips, the wound-up integral only unwinds slowly, so the output swings hard the other way and **overshoots**.

The code above guards against this with a crude method: clamping the integral value itself to the output limit. This is not a complete fix (the integral can still wind up all the way to that limit while the output is pinned), but it is enough to fly and feel the effect for now.

**If you want to go further:** this project's real-hardware controller (`firmware/vehicle/components/sf_controller_pid/include/pid.hpp`) uses a more precise technique called "conditional integration," which stops updating the integral only while the output is being pushed further into saturation. It's worth reading the two implementations side by side.

## 9. Adding the Derivative Term (D)

### Why This Step Matters

PI control (proportional + integral) removes the steady-state error, but it does little to restrain the "overshoot" right after a sudden change in the target. Add a term that brakes the rate of approach toward the target itself — **derivative control (D control)**.

Here too, verify on real hardware only with the props off or the craft restrained, and only after confirming in SILS first, per chapter 7's notice. The derivative term tends to amplify sensor noise, and depending on the gain you pick, high-frequency oscillation can show up abruptly.

### The Derivative-Kick Trap

If you naively differentiate "the error," every time the target (the stick value) steps to a new value, the error also jumps instantaneously, and the derivative spikes — this is called a **derivative kick**. ACRO's rate target is built directly from a 12-bit stick reading, so it is constantly stepping in tiny increments; differentiating the error directly would turn the derivative term into a constant source of noisy commands.

The fix is simple: **differentiate the measurement, not the error** (D-on-Measurement). Instead of looking at how much the target changed, look only at how fast the craft's actual rotation is changing. A step in the target never passes through this path, so no derivative kick occurs.

### Implementation

```cpp
namespace {
// ...
constexpr float kRollTd = 0.001f;   // [s] derivative time
}  // namespace
```

```cpp
// app_controller.hpp に追加
private:
    float prev_measured_roll_ = 0.0f;
    bool  roll_first_sample_  = true;   // primes the derivative after reset()
```

```cpp
// compute() 内
float d_term_roll = 0.0f;
if (!roll_first_sample_) {
    const float measured_rate_of_change = (state.angular_rate[0] - prev_measured_roll_) / dt;
    d_term_roll = -kRollKp * kRollTd * measured_rate_of_change;
}
prev_measured_roll_ = state.angular_rate[0];
roll_first_sample_  = false;

float torque_roll = kRollKp * error_roll + integral_roll_ + d_term_roll;
if (torque_roll >  kMaxRollPitchTorqueNm) torque_roll =  kMaxRollPitchTorqueNm;
if (torque_roll < -kMaxRollPitchTorqueNm) torque_roll = -kMaxRollPitchTorqueNm;
```

`reset()` should also initialize `prev_measured_roll_` and `roll_first_sample_`. Without the `roll_first_sample_` guard, the "previous measurement" right after a reset would be stale (or a meaningless default), producing one unnatural derivative kick right at the moment of reset.

That completes all three PID terms. **If you want to go further:** the derivative term has a weakness — it directly amplifies measurement noise (the gyro's small fluctuations). The `pid.hpp` implementation adds a low-pass filter to the derivative (an "incomplete derivative" that trims high frequencies using a coefficient called `eta`), which behaves more smoothly in a real, noisy environment.

## 10. Bringing All Three Axes Together Into a Real ACRO Controller

### Why This Step Matters

Up to now you've practiced on the roll axis alone. Pitch and yaw work in exactly the same way, and rather than copy-pasting three axes' worth of PID state, it's clearer and less error-prone to **consolidate "one axis' PID state" into a single struct, and instantiate three of them**. At the same time, this chapter properly implements `reset()`/`onModeChange()` and plugs in flight-verified gain values, finishing a genuine ACRO controller.

### Consolidating One Axis' PID State

```cpp
// app_controller.hpp
#pragma once
#include "controller.hpp"

namespace sf::app {

/// One axis' rate-loop PID state (P + simple clamped I + measurement D).
/// 1軸分のレートループPID状態（P + 素朴なクランプ付きI + 測定値D）。
struct RateAxisPid {
    float kp = 0.0f;
    float ti = 0.0f;
    float td = 0.0f;
    float output_limit = 0.0f;

    float integral = 0.0f;
    float prev_measurement = 0.0f;
    bool first_sample = true;

    float compute(float setpoint, float measurement, float dt);
    void reset();
};

class AppController : public sf::IController {
public:
    AppController();

    sf::ControlOutput compute(
        const sf::StateEstimate& state,
        const sf::CommandSetpoint& setpoint,
        float dt) override;

    void reset() override;
    void onModeChange(sf::FlightMode new_mode) override;
    void onLanding() override;

private:
    RateAxisPid roll_pid_;
    RateAxisPid pitch_pid_;
    RateAxisPid yaw_pid_;

    bool  landing_active_       = false;
    float landing_elapsed_s_    = 0.0f;
    float landing_thrust_start_ = 0.0f;
    float last_thrust_          = 0.0f;
};

}  // namespace sf::app
```

`RateAxisPid::compute()` is exactly the roll-axis P/I/D calculation you built in chapters 7-9, rewritten so it no longer depends on which axis it's for.

```cpp
// app_controller.cpp
#include "app_controller.hpp"

namespace sf::app {

namespace {
// Named constants — every value is either a physical limit of this frame
// (thrust/torque caps) or a flight-proven ACRO rate-loop gain.
// 名前付き定数——推力・トルクの上限はこの機体の物理限界、それ以外は
// 飛行実績のあるACROレートループゲイン。
constexpr float kMaxRollPitchRateRadS = 1.0f;
constexpr float kMaxYawRateRadS       = 5.0f;
constexpr float kMaxThrustN           = 0.672f;
constexpr float kMaxRollPitchTorqueNm = 5.2e-3f;
constexpr float kMaxYawTorqueNm       = 1.226e-3f;

constexpr float kRollKp = 1.0e-3f,       kRollTi = 0.7f,  kRollTd = 0.002f;
constexpr float kPitchKp = 1.426432e-3f, kPitchTi = 0.7f, kPitchTd = 0.025f;
constexpr float kYawKp = 8.029796e-4f,   kYawTi = 0.8f,   kYawTd = 0.01f;

constexpr float kLandingDescentS = 3.0f;   // §11 で使う降下時間
}  // namespace

float RateAxisPid::compute(float setpoint, float measurement, float dt)
{
    const float error = setpoint - measurement;

    const float p_term = kp * error;

    if (ti > 0.0f) {
        integral += (kp / ti) * error * dt;
        if (integral >  output_limit) integral =  output_limit;
        if (integral < -output_limit) integral = -output_limit;
    }

    float d_term = 0.0f;
    if (td > 0.0f && !first_sample) {
        const float measurement_rate = (measurement - prev_measurement) / dt;
        d_term = -kp * td * measurement_rate;
    }
    prev_measurement = measurement;
    first_sample = false;

    float output = p_term + integral + d_term;
    if (output >  output_limit) output =  output_limit;
    if (output < -output_limit) output = -output_limit;
    return output;
}

void RateAxisPid::reset()
{
    integral = 0.0f;
    prev_measurement = 0.0f;
    first_sample = true;
}

AppController::AppController()
    : roll_pid_{kRollKp,  kRollTi,  kRollTd,  kMaxRollPitchTorqueNm},
      pitch_pid_{kPitchKp, kPitchTi, kPitchTd, kMaxRollPitchTorqueNm},
      yaw_pid_{kYawKp,    kYawTi,   kYawTd,   kMaxYawTorqueNm}
{
}

sf::ControlOutput AppController::compute(
    const sf::StateEstimate& state,
    const sf::CommandSetpoint& setpoint,
    float dt)
{
    sf::ControlOutput output{};
    output.timestamp = state.timestamp;

    float rate_sp_roll  = setpoint.roll  * kMaxRollPitchRateRadS;
    float rate_sp_pitch = setpoint.pitch * kMaxRollPitchRateRadS;
    float rate_sp_yaw   = setpoint.yaw   * kMaxYawRateRadS;
    float thrust        = setpoint.throttle * kMaxThrustN;

    if (landing_active_) {
        // See §11 — comm-loss / battery-emergency descent.
        rate_sp_roll = rate_sp_pitch = rate_sp_yaw = 0.0f;
        landing_elapsed_s_ += dt;
        float ramp = 1.0f - (landing_elapsed_s_ / kLandingDescentS);
        if (ramp < 0.0f) ramp = 0.0f;
        thrust = landing_thrust_start_ * ramp;
    }

    output.torque[0] = roll_pid_.compute(rate_sp_roll,  state.angular_rate[0], dt);
    output.torque[1] = pitch_pid_.compute(rate_sp_pitch, state.angular_rate[1], dt);
    output.torque[2] = yaw_pid_.compute(rate_sp_yaw,    state.angular_rate[2], dt);
    output.thrust = thrust;

    // Export the rate targets for later analysis (sf log analyze/viz).
    // ACRO has no angle (attitude) loop, so angle_ref stays at 0.
    // 解析用にレート目標を出力（sf log analyze/viz）。ACROには角度ループが
    // ないので angle_ref は0のまま。
    output.rate_ref[0] = rate_sp_roll;
    output.rate_ref[1] = rate_sp_pitch;
    output.rate_ref[2] = rate_sp_yaw;
    output.angle_ref[0] = 0.0f;
    output.angle_ref[1] = 0.0f;

    last_thrust_ = thrust;
    return output;
}

void AppController::reset()
{
    roll_pid_.reset();
    pitch_pid_.reset();
    yaw_pid_.reset();
    landing_active_ = false;
    landing_elapsed_s_ = 0.0f;
}

void AppController::onModeChange(sf::FlightMode new_mode)
{
    // This tutorial controller only implements ACRO. A controller covering
    // more modes would reconfigure its cascade here (see PidController).
    // このコントローラはACRO専用。複数モードに対応する制御器は、ここで
    // カスケード構成を再構成する（本物のPidController参照）。
    (void)new_mode;
}

}  // namespace sf::app
```

`onLanding()`'s implementation is covered as a whole in the next chapter.

### About the Gain Values

| Axis | kp | ti [s] | td [s] | Output Limit |
|------|----|--------|--------|--------------|
| Roll  | 1.0e-3 | 0.7 | 0.002 | ±5.2e-3 Nm |
| Pitch | 1.426432e-3 | 0.7 | 0.025 | ±5.2e-3 Nm |
| Yaw   | 8.029796e-4 | 0.8 | 0.01 | ±1.226e-3 Nm |

These are values with real flight history on this craft (StampFly). That said, this guide's PID implementation (a naive clamped integral, an unfiltered derivative) discretizes things differently from the real `PidController` (bilinear/Tustin transform, conditional-integration anti-windup), so behavior will not match exactly. At a control period as fast as 400 Hz the practical difference is small, but if you want to chase it down precisely, look at the `pid.hpp` implementation.

With all three axes closed, this controller now genuinely has enough authority to fly freely. When you try it on real hardware, hold to chapter 7's safety notice (props off or the craft firmly restrained, SILS before real-hardware bench, ready to execute an emergency stop) even more strictly than before. Save actual free flight for after you've gone through the safety mechanism and pre-flight checklist in chapters 11 and 12.

## 11. The Safety Mechanism and `onLanding()`

### Why This Step Matters

What happens if the link to the transmitter drops? The firmware already has a built-in failsafe: after detecting a comm loss, it **tries to keep hovering on the last command for 3 seconds, and if the link has not come back by then, automatically transitions into a "landing" state** (a critically low battery voltage triggers the same transition). The instant it enters that landing state, the firmware calls your controller's `onLanding()` exactly once.

**If you leave this unimplemented**, the controller keeps chasing "the last stick value it ever received" forever. If the stick happened to be commanding, say, "move forward while rotating" at the exact moment the link dropped, the craft keeps flying that command with no way to stop it. Implementing `onLanding()` is a safety mechanism this guide's controller must not skip.

### Implementation Approach (An Honest Simplification)

This controller only handles ACRO — angular rate — and **has no mechanism to detect or correct attitude (tilt)**. So "returning the craft to level" is simply not possible in principle; all it can do is "stop rotating any further" and "ease off thrust and come down gently." This guide takes the following simple, honest approach:

- Set the target angular rate to zero on all three axes (ignore the sticks and stop any further rotation; if the craft was already close to level, it descends while staying roughly level as a side effect)
- Starting from the thrust that was commanded the instant `onLanding()` is called, linearly ramp thrust down to zero over a fixed duration (3 seconds in this guide, `kLandingDescentS`)

`onLanding()` only receives the signal that landing has begun — it has no direct access to state (the current thrust). So at the end of every `compute()` call, save "the thrust just commanded" into `last_thrust_`, and let `onLanding()` use that saved value as the starting point for the descent.

```cpp
void AppController::onLanding()
{
    landing_active_ = true;
    landing_elapsed_s_ = 0.0f;
    landing_thrust_start_ = last_thrust_;
}
```

`landing_active_` is cleared by the next `reset()` (the next ARM) — that's part of `IController::onLanding()`'s design contract, which is why `reset()` above sets `landing_active_ = false`.

This simple implementation has a real limitation: if the craft was already tilted significantly when landing began, it comes down still tilted. A more sophisticated implementation (adding, say, a simple tilt estimate from the accelerometer) is beyond this guide's scope — the point to take away is simply that an honest, simple safety measure is always far better than nothing.

Don't test this landing law by cutting the transmitter's power mid-flight during real free flight, either. First confirm in a SILS comm-loss scenario that `onLanding()` fires and thrust ramps down as intended, from the log. When you check on real hardware, start with the props off or the craft restrained, and confirm the behavior right at the moment `onLanding()` fires (rotation stops, thrust falls).

## 12. Verify in SILS, Then Fly on Real Hardware

### Steps

1. **Confirm in SILS.**

   ```bash
   sf app sils my_acro simulator/sils/scenarios/acro_flight.scn
   ```

   Confirm the exit code is 0 (PASS).

2. **Build for real hardware and flash it.**

   ```bash
   sf app build my_acro
   sf app flash my_acro -m
   ```

3. **Always run the pre-flight checklist** (see `docs/guides/safety.md` for details).

   | # | Check Item |
   |---|-----------|
   | 1 | Propeller guards undamaged |
   | 2 | No propeller damage |
   | 3 | Battery sufficiently charged (above 30%) |
   | 4 | Flight area secured (at least 2m x 2m, no obstacles) |
   | 5 | No people nearby |
   | 6 | Emergency-stop method confirmed (`sf emergency`, or the transmitter's emergency stop) |

4. A first flight is more likely to behave unexpectedly than usual. Until you've confirmed stable flight, stay ready to execute the emergency stop immediately.

5. After flying, capture and inspect the log.

   ```bash
   sf log wifi -d 30
   sf log analyze
   ```

   `sf log analyze` prints gyro statistics, input/response correlation, and oscillation-frequency analysis. Use `sf log viz` if you'd rather see it as a graph.

## 13. Tuning Your Own Gains, and Next Steps

Leaving constants like `kRollKp` baked into the source means every gain change requires a rebuild and reflash. The firmware already has a parameter system built on **NVS** (Non-Volatile Storage — a storage area that survives power loss) and lets you change gains in flight with the `param set` command (live tuning). If you want to hook your own gains into that system, read the registration pattern for `rate.roll.*` and friends in `firmware/vehicle/components/sf_core/params.cpp`, along with `loadParams()`/`reloadParams()` in `firmware/vehicle/components/sf_controller_pid/pid_controller.cpp`, and parameterize your own constants the same way.

### Where This Goes From Here

ACRO mode (rate control) is the simplest control law the vehicle firmware has. What comes next is **attitude estimation** (estimating the craft's tilt), **attitude control** (tracking a target tilt using that estimate), and eventually control laws built on a different foundation than classical PID entirely, such as **LQR/LQI** (Linear-Quadratic Regulator / Linear-Quadratic-Integral control). None of that exists as a guide yet, but it is expected to arrive as a sequel to this one. The overall learning roadmap is laid out in the educational roadmap section of `firmware/vehicle/docs/coding_and_education.md`.

For further reading:

| Document | Content |
|----------|---------|
| [`firmware/vehicle/components/sf_controller_pid/include/pid.hpp`](../../firmware/vehicle/components/sf_controller_pid/include/pid.hpp) | The real PID implementation (conditional integration, Tustin-transform incomplete derivative) |
| [`firmware/vehicle/docs/topic_reference.md`](../../firmware/vehicle/docs/topic_reference.md) | The SSOT (Single Source of Truth) topic catalog and usage patterns |
| [`docs/guides/safety.md`](safety.md) | The flight safety guide |
