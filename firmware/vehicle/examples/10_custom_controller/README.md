# 10_custom_controller — L1 IController 差替え演習

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 目的

`IController`（`components/sf_controller/include/controller.hpp`）を実装した
薄いラッパークラスを書き、既存の `PidController`（カスケードPID制御器）に
処理を委譲しながら、たった1箇所だけ独自の変更を挿入する方法を学ぶ。
`docs/architecture.md` §2.5 が言う「`IController` を実装して既存と差替え」の
最小の第一歩。

## 2. このサンプルはどこまでやるか（実態に合った範囲）

**本サンプルは実機を飛ばさない。** `IController` には実行時ファクトリが無く、
実ファームは `tasks/control_task.cpp:63` の

```cpp
static sf::PidController controller;
```

という **1個の static インスタンス**でコンパイル時に制御器を決めている。単体
サンプルプロジェクトから REQUIRES して差し込めるコンポーネント単位の口は
存在しない。そこで本サンプルは、`LearnerController`（`IController` 実装）を
**合成入力に対してスタンドアロンで動かすベンチ**として構成した — HAL も
Topic も実ファームも一切使わず、このクラスが単体でビルドでき、
`IController` として正しく振る舞うことを実演する。実機で実際に飛ばす手順は
§8「実機で飛ばすレシピ」を参照（本サンプルはそこまで行わない）。

## 3. 重要な5〜10行

学ぶべきコードは `main/learner_controller.hpp` のクラス宣言と、
`main/learner_controller.cpp` の `compute()` 実装、この2箇所。

```cpp
// learner_controller.hpp — IController の全12メソッドを宣言。compute() 以外は
// 単純転送。
class LearnerController : public IController {
public:
    void init();
    ControlOutput compute(const StateEstimate& state, const CommandSetpoint& setpoint,
                           float dt) override;
    void reset() override;
    // ... 他9メソッドも同様に override（.cpp では1行転送のみ）
private:
    PidController inner_controller_;
};
```

```cpp
// learner_controller.cpp — compute() が唯一の演習フック
ControlOutput LearnerController::compute(const StateEstimate& state,
                                          const CommandSetpoint& setpoint, float dt)
{
    ControlOutput output = inner_controller_.compute(state, setpoint, dt);
    output.torque[2] *= config::kYawTorqueScale;   // ← ここに独自制御則を書く
    return output;
}
```

`compute()` 以外の11メソッド（`reset()`, `onModeChange()`, `onLanding()` 等）は
全て `inner_controller_.同名メソッド(...)` の1行転送。既存のカスケードPIDを
コピペで再実装せず、丸ごと再利用しながら1点だけ変えられるのが「薄いラッパー」の
狙い。

## 4. ビルド・書き込み・実行

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/10_custom_controller
sf flash vehicle/examples/10_custom_controller -m    # 書き込み後にモニタを開く。終了は Ctrl+]
```

StampFly実機は不要（センサ・アクチュエータに一切触れない）。任意のESP32-S3
ボードで実行できる。vehicle 本体ビルドとは別のスタンドアロンプロジェクトだが、
`sf build vehicle/examples/10_custom_controller` を対象として指定すればそのまま
ビルド・書き込みできる。`idf.py` を直接使う場合は例題ディレクトリで
`idf.py build flash monitor`（チップは `sdkconfig.defaults` で ESP32-S3 に固定済み）。

## 5. 期待される出力

```
I (327) custom_controller: === Example 10: L1 IController exercise ===
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0021 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0034 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0041 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
...（torque のピッチ列(2番目)が config.hpp の合成正弦波に追従してゆっくり振動する）
```

- `thrust` はスロットル0.5から計算される一定値、`torque(RPY)` の2番目
  （ピッチ）が `config::kSyntheticPitchAmplitudeRadians` の正弦波外乱に対する
  カスケードPIDの応答として振動する。roll/yawはこのベンチでは常に0（外乱を
  与えていないため）。
- §2で述べた通りプラントモデルが無いため、この振動は「制御が外乱を打ち消す
  様子」ではなく「合成した外乱そのものへのPID応答」を表示しているだけである。

## 6. `config::kYawTorqueScale` を変えて演習する

`main/config.hpp` の `kYawTorqueScale`（既定 1.0）を 2.0 に変えて再ビルドすると、
`torque(RPY)` の3番目（yaw）列だけが倍になり、roll/pitch は変化しない —
同じ内側 `PidController` を使い回しながら、ラッパー側の1行だけが効いている
ことが確認できる。`main.cpp` の「ここを変えてみよう」節に他の演習も用意した。

## 7. よくあるエラーと対処

| 症状 | 対処 |
|------|------|
| ビルド時に `IController` の純粋仮想関数が未実装というエラー | `learner_controller.hpp` の12メソュード全てを `learner_controller.cpp` で実装しているか確認 |
| `torque` が全て0のまま変化しない | `config::kSyntheticPitchAmplitudeRadians` が0になっていないか確認（§8「ここを変えてみよう」3番） |

## 8. 実機で飛ばすレシピ（本サンプルはここまで行わない・新方式）

`LearnerController` を実際の StampFly 飛行制御パイプラインで使うには、
**vehicle 本体を再ビルドする必要がある**（本サンプルは別プロジェクトのため
不可）。かつては新規コンポーネント化・`main/CMakeLists.txt` の `REQUIRES` 追加・
`control_task.cpp` への include 追加・63行目の書き換えという手作業が必要だったが、
現在は `sf app` コマンドが同じことを自動化する。

既定の複製元 `11_app_controller` は、本例題の `LearnerController` と同じ
「`PidController` に委譲する薄いラッパー」パターンを `IController` の
全12メソッドに拡張したもの（差し替え可能な1点は `app_controller.cpp` の
`adjust()`）。以下のコマンドで同じことが行える。

```bash
sf app new my_ctrl
```

```bash
sf app sils my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app flash my_ctrl -m
```

`sf app sils` が SILS（Software In the Loop Simulation）での確認、
`sf app build`/`sf app flash` が実機ビルド・書き込みで、どちらも
`firmware/apps/my_ctrl/*.cpp` を vehicle 本体の main コンポーネントに
組み込んで動かす（Code Identity: 実機と SILS で同じコードが動く）。詳細は
[`examples/11_app_controller/README.md`](../11_app_controller/README.md) と
[独自プログラム開発入門](../../../../docs/guides/custom_program.md) を参照。

## 9. 次のステップ

- **`IEstimator` を差し替える**: 同じ考え方の推定側は `examples/09_topic_api_hello`
  の「次のステップ」を参照。
- **設計文書**: `docs/architecture.md` §2.5「学習者の入口（4階層アクセス）」、
  `docs/coding_and_education.md` §3 の `21_custom_controller` 計画。

---

<a id="english"></a>

## 1. Overview

Learn how to write a thin wrapper class implementing `IController`
(`components/sf_controller/include/controller.hpp`) that delegates to the
existing `PidController` (cascade PID controller) while inserting exactly one
custom change. This is the smallest first step toward what
`docs/architecture.md` §2.5 calls "implement `IController` and swap it in for
the existing one".

## 2. How Far This Example Goes (an honest scope statement)

**This example does not fly real hardware.** `IController` has no runtime
factory — the real firmware picks its controller at COMPILE TIME with **one
static instance** at `tasks/control_task.cpp:63`:

```cpp
static sf::PidController controller;
```

There is no component-level "plug in" point a standalone example project can
REQUIRE into. So this example is built as a **standalone bench that runs
`LearnerController` (an `IController` implementation) against synthetic
inputs** — no HAL, no Topics, no real firmware — demonstrating that the class
builds on its own and behaves correctly as an `IController`. See section 8,
"Recipe to actually fly this" for what real-flight registration requires
(this example does not go that far).

## 3. The 5–10 Lines That Matter

The code to study lives in two places: the class declaration in
`main/learner_controller.hpp`, and the `compute()` implementation in
`main/learner_controller.cpp`.

```cpp
// learner_controller.hpp — declares all 12 IController methods. Everything
// except compute() is a plain forward.
class LearnerController : public IController {
public:
    void init();
    ControlOutput compute(const StateEstimate& state, const CommandSetpoint& setpoint,
                           float dt) override;
    void reset() override;
    // ... 9 more methods, each overridden the same way (one-line forward in the .cpp)
private:
    PidController inner_controller_;
};
```

```cpp
// learner_controller.cpp — compute() is the one exercise hook
ControlOutput LearnerController::compute(const StateEstimate& state,
                                          const CommandSetpoint& setpoint, float dt)
{
    ControlOutput output = inner_controller_.compute(state, setpoint, dt);
    output.torque[2] *= config::kYawTorqueScale;   // <- your custom control law goes here
    return output;
}
```

The other 11 methods (`reset()`, `onModeChange()`, `onLanding()`, etc.) are
each a one-line forward to `inner_controller_.<same method>(...)`. The point
of a thin wrapper is that you reuse the whole existing cascade PID instead of
copy-pasting it, and change exactly one thing.

## 4. Build / Flash / Run

```bash
source setup_env.sh                                 # Windows: setup_env.bat
sf build vehicle/examples/10_custom_controller
sf flash vehicle/examples/10_custom_controller -m    # opens a monitor after flashing; exit with Ctrl+]
```

No StampFly hardware is needed (nothing here touches a sensor or actuator);
any ESP32-S3 board will run it. This is a standalone project (not part of
the vehicle firmware build), but `sf build`/`sf flash` can target it
directly via `vehicle/examples/10_custom_controller`. If you prefer
`idf.py` directly, run `idf.py build flash monitor` from the example
directory (the chip is already pinned to ESP32-S3 via `sdkconfig.defaults`).

## 5. Expected Output

```
I (327) custom_controller: === Example 10: L1 IController exercise ===
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0021 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0034 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
thrust=  4.90N  torque(RPY)=[+0.0000 +0.0041 +0.0000]Nm  angle_ref(RP)=[ +0.00  +0.00]deg
... (torque's pitch column, 2nd value, slowly oscillates, tracking config.hpp's synthetic sine wave)
```

- `thrust` is a constant computed from the 0.5 throttle setpoint;
  `torque(RPY)`'s 2nd (pitch) value oscillates as the cascade PID responds to
  the `config::kSyntheticPitchAmplitudeRadians` disturbance. Roll/yaw stay at
  0 in this bench (no disturbance is applied to them).
- As noted in section 2, there is no plant model, so this oscillation shows
  "the PID's response to a synthetic disturbance", not "control cancelling a
  real disturbance".

## 6. Exercise: Change `config::kYawTorqueScale`

Change `kYawTorqueScale` in `main/config.hpp` (default 1.0) to 2.0 and
rebuild: `torque(RPY)`'s 3rd (yaw) column doubles while roll/pitch stay
unchanged — confirming the same inner `PidController` is doing the cascade
work while only the wrapper's one line takes effect. See `main.cpp`'s "Try
changing!" section for more exercises.

## 7. Common Errors

| Symptom | Fix |
|---|---|
| Build error about unimplemented pure virtual methods of `IController` | Check that all 12 methods declared in `learner_controller.hpp` are implemented in `learner_controller.cpp` |
| `torque` stays at exactly 0 forever | Check `config::kSyntheticPitchAmplitudeRadians` is not 0 (see "Try changing!" #3) |

## 8. Recipe to Actually Fly This (this example does not go this far — new approach)

Using `LearnerController` in the real StampFly flight pipeline requires
**rebuilding the `vehicle` firmware itself** (not possible from this separate
example project). This used to require manual steps — turning the class into
a new component, adding it to `main/CMakeLists.txt`'s `REQUIRES`, adding an
include in `control_task.cpp`, and rewriting line 63 — but the `sf app`
command now automates all of it.

The default clone source, `11_app_controller`, extends this example's
`LearnerController` idea (a thin wrapper delegating to `PidController`) to
all 12 `IController` methods; the one swappable insertion point is
`adjust()` in `app_controller.cpp`. The same result follows from these
commands.

```bash
sf app new my_ctrl
```

```bash
sf app sils my_ctrl
```

```bash
sf app build my_ctrl
```

```bash
sf app flash my_ctrl -m
```

`sf app sils` verifies it in SILS (Software In the Loop Simulation); `sf app
build`/`sf app flash` build and flash real hardware. Both embed
`firmware/apps/my_ctrl/*.cpp` into the vehicle firmware's main component
(Code Identity: the same code runs on hardware and in SILS). See
[`examples/11_app_controller/README.md`](../11_app_controller/README.md) and
the [Custom Program Guide](../../../../docs/guides/custom_program.md) for details.

## 9. Next Steps

- **Swap the `IEstimator`**: the same idea on the estimation side is in
  `examples/09_topic_api_hello`'s "Next Steps".
- **Design docs**: `docs/architecture.md` §2.5 "学習者の入口（4階層アクセス）",
  the planned `21_custom_controller` example in `docs/coding_and_education.md`
  §3.
