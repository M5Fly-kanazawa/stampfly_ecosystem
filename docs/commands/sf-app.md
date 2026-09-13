# sf app

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

自分のドローンファームウェアプロジェクトを作成・編集・ビルド・書き込み・SILS
（Software In the Loop Simulation: ファームウェアそのものを PC 上で動かす試験）検証する。
`firmware/vehicle/examples/<N>` の例題を `firmware/apps/<name>` に複製し、
`sf lesson`（ワークショップ実習）と同じ new → edit → build → flash の流れを
研究・自作制御則の入口として提供する。詳細は `firmware/apps/README.md` も参照。

### 組み込み型とベンチ型

`firmware/apps/<name>` は `app.yaml` の `type` によって2種類に分かれる。

| 種別 | 実体 | SILS | 実機ビルド |
|------|------|------|-----------|
| `embedded`（組み込み型） | `firmware/apps/<name>/*.cpp` が vehicle 本体の main コンポーネントに `SF_APP_DIR` 経由で直接コンパイルされる。自身の `CMakeLists.txt` を持たない | 可（`sf app sils`） | 可（vehicle 本体を `SF_APP_DIR` 付きでビルド） |
| `bench`（ベンチ型） | 独立した ESP-IDF プロジェクト（自身の `CMakeLists.txt` を持つ） | 不可（`sf app sils` は終了コード2で案内） | 可（単独プロジェクトとしてビルド） |

`sf app new` の既定の複製元 `11_app_controller` は組み込み型。`09_topic_api_hello`・
`10_custom_controller` はベンチ型として残っている（`app.yaml` が無い例題は、
ベンチ型として扱われる）。

## 2. 構文

```bash
sf app <subcommand> [args]
```

### サブコマンド

| サブコマンド | 説明 |
|-------------|------|
| `new <name> [--from <example>]` | 例題を複製して新規プロジェクトを作成 |
| `edit <name> [--editor X] [--reuse-window]` | 学習者が書くファイルをエディタで開く |
| `build <name> [-c] [-v]` | プロジェクトをビルド |
| `flash <name> [-p PORT] [-b BAUD] [-m]` | プロジェクトを書き込む |
| `sils <name> [scenario] [--expect F] [--noise N] [--seed S] [--duration D]` | 組み込み型プロジェクトの SILS シナリオを実行 |
| `list` | 自分のプロジェクト一覧と `--from` に指定できる例題一覧を表示 |

### `new` の引数

| 引数 | 説明 | デフォルト |
|------|------|-----------|
| `name` | プロジェクト名（C識別子として有効な文字列） | (必須) |
| `--from <example>` | 複製元の例題（`firmware/vehicle/examples/` 配下のディレクトリ名） | `11_app_controller` |

`name` はC識別子であること（英数字とアンダースコアのみ、数字始まり不可）。
既に `firmware/apps/<name>` が存在する場合、および予約名
（`vehicle`, `controller`, `workshop`, `common`, `apps`）は拒否される。
`--from` に存在しない例題を指定するとエラーになり、利用可能な例題一覧が表示される。

### `build` / `flash` の挙動

- **組み込み型**: `idf.py -B firmware/apps/<name>/build -D SF_APP_DIR=<abs path> build`
  を `cwd=firmware/vehicle` で実行する（vehicle 本体のビルド。app ごとにビルド
  ディレクトリが分かれるため、app を切り替えてもキャッシュを汚さない）。生成物は
  `stampfly_vehicle.bin`。
- **ベンチ型**: 従来通り `sf build apps/<name>` / `sf flash apps/<name>`（独立した
  ESP-IDF プロジェクトとしてのビルド）に委譲する。

### `sils` の引数

| 引数 | 説明 | デフォルト |
|------|------|-----------|
| `name` | プロジェクト名 | (必須) |
| `scenario`（位置引数） | `.scn` シナリオファイルへのパス | `simulator/sils/scenarios/alt_flight.scn` |
| `--expect <file>` | 合否判定ファイル | `<scenario>.expect`（存在する場合） |
| `--noise {off,n0,n1,n2}` | センサノイズレベル | `off` |
| `--seed <int>` | ノイズ乱数シード | `12345` |
| `--duration <int>` | シミュレーション時間（マイクロ秒） | `25000000`（25秒） |

`name` がベンチ型プロジェクトの場合、`sf app sils` は組み込めない理由を表示して
終了コード **2** を返す（`sf app new <name> --from 11_app_controller` での作り直しを案内する）。
内部は `sf sils scenario <scenario> --target apps/<name>` と同じ経路
（`build_app_emulator()` が `simulator/sils/build/apps/<name>` に専用のビルドを作る）。

## 3. 使用例

既定の例題（`11_app_controller` = 組み込み型 `IController` テンプレート）から作成する。

```bash
sf app new my_ctrl
```

学習者が書くファイルをエディタで開く。

```bash
sf app edit my_ctrl
```

SILS で確認する（既定シナリオ `alt_flight.scn`）。

```bash
sf app sils my_ctrl
```

別のシナリオを指定する場合。

```bash
sf app sils my_ctrl simulator/sils/scenarios/acro_flight.scn
```

実機向けにビルドする。

```bash
sf app build my_ctrl
```

書き込む。

```bash
sf app flash my_ctrl -m
```

Topic を読むだけの追加タスクのテンプレートから作成する。

```bash
sf app new my_hello --from 12_app_task_hello
```

ベンチ型（vehicle 全体をビルドせずに API を学ぶ）から作成する。

```bash
sf app new my_bench --from 10_custom_controller
```

自分のプロジェクト一覧と、`--from` に指定できる例題一覧を表示する。

```bash
sf app list
```

## 4. 出力

`sf app new` 成功時（組み込み型）:
```
[INFO] Creating new app project: my_ctrl
  From: /path/to/firmware/vehicle/examples/11_app_controller
  To:   /path/to/firmware/apps/my_ctrl
[OK] Project created: /path/to/firmware/apps/my_ctrl

Next steps:
  sf app sils my_ctrl
  sf app build my_ctrl
  sf app flash my_ctrl -m
```

`sf app list` の列: `Name` / `Type`（`embedded`/`bench`） / `Hardware`（常に `yes`） /
`SILS`（`yes` または `no (bench)`） / `Description`。

## 5. `app.yaml`

`firmware/apps/<name>/app.yaml` はプロジェクトの種別を記録するマニフェストファイル。

| キー | 内容 |
|------|------|
| `type` | `embedded` または `bench` |
| `from` | 複製元の例題名 |
| `sils` | SILS 実行可否（`true`/`false`） |
| `name` | プロジェクト名 |
| `description` | 一覧表示用の説明（複製元の README 見出しから自動設定される場合がある） |

`app.yaml` を持たない例題からの複製（09/10 等）は、ベンチ型として合成される
（`type: bench`, `sils: false`）。

## 6. `sf sils --target apps/<name>` との関係

`sf app sils <name>` は `sf sils scenario <scenario> --target apps/<name>` の薄い
ラッパーである。`sf sils scenario`/`sf sils build` の `--target`/`-t` に
`apps/<name>` を直接指定しても同じ経路が使われる（`name` は組み込み型で
なければならない）。

```bash
sf sils scenario simulator/sils/scenarios/alt_flight.scn --target apps/my_ctrl
```

## 7. 補足

- `sf app edit` は組み込み型では `app_controller.cpp`（無ければ `app.cpp`）を、
  ベンチ型では `main/learner_controller.cpp`（無ければ `main/main.cpp`）を開く
  （エディタ検出は VSCode → vi → vim → Notepad(Windowsのみ) の順）。
- ベンチ型の `sf app build`/`sf app flash` は内部で `sf build apps/<name>`/
  `sf flash apps/<name>` に処理を委譲しているだけなので、`sf build`/`sf flash`
  を直接使っても同じ結果になる。

---

<a id="english"></a>

## 1. Overview

Create, edit, build, flash, and run SILS (Software In the Loop Simulation —
running the firmware itself on a PC) on your own StampFly firmware project. A
new project is cloned from a `firmware/vehicle/examples/<N>` example into
`firmware/apps/<name>`, providing the same new -> edit -> build -> flash
flow as `sf lesson` (the workshop exercises) as an entry point for research
and custom control laws. See also `firmware/apps/README.md`.

### Embedded-Type vs. Bench-Type

`firmware/apps/<name>` splits into two kinds, based on `app.yaml`'s `type`.

| Type | What It Is | SILS | Hardware Build |
|------|-----------|------|-----------------|
| `embedded` | `firmware/apps/<name>/*.cpp` is compiled directly into the vehicle firmware's main component via `SF_APP_DIR`. Has no `CMakeLists.txt` of its own | Yes (`sf app sils`) | Yes (vehicle firmware built with `SF_APP_DIR`) |
| `bench` | A standalone ESP-IDF project (has its own `CMakeLists.txt`) | No (`sf app sils` explains why and exits with code 2) | Yes (built as a standalone project) |

`sf app new`'s default source, `11_app_controller`, is embedded-type.
`09_topic_api_hello` and `10_custom_controller` remain bench-type (an example
with no `app.yaml` is treated as bench-type).

## 2. Syntax

```bash
sf app <subcommand> [args]
```

### Subcommands

| Subcommand | Description |
|-----------|------|
| `new <name> [--from <example>]` | Create a new project cloned from an example |
| `edit <name> [--editor X] [--reuse-window]` | Open the learner-facing source file in an editor |
| `build <name> [-c] [-v]` | Build the project |
| `flash <name> [-p PORT] [-b BAUD] [-m]` | Flash the project |
| `sils <name> [scenario] [--expect F] [--noise N] [--seed S] [--duration D]` | Run an embedded-type project's SILS scenario |
| `list` | List your projects and the examples available for `--from` |

### `new` Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `name` | Project name (must be a valid C identifier) | (required) |
| `--from <example>` | Example to clone (a directory name under `firmware/vehicle/examples/`) | `11_app_controller` |

`name` must be a valid C identifier (letters, digits, underscore; cannot
start with a digit). Rejected if `firmware/apps/<name>` already exists, or
if `name` is a reserved word (`vehicle`, `controller`,
`workshop`, `common`, `apps`). An unknown `--from` example fails with an
error listing the available examples.

### `build` / `flash` Behavior

- **Embedded-type**: runs `idf.py -B firmware/apps/<name>/build -D
  SF_APP_DIR=<abs path> build` with `cwd=firmware/vehicle` (a vehicle
  firmware build; each app gets its own build directory, so switching apps
  never clobbers a cache). Produces `stampfly_vehicle.bin`.
- **Bench-type**: delegates to `sf build apps/<name>` / `sf flash apps/<name>`
  as before (a standalone ESP-IDF project build).

### `sils` Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `name` | Project name | (required) |
| `scenario` (positional) | Path to a `.scn` scenario file | `simulator/sils/scenarios/alt_flight.scn` |
| `--expect <file>` | Assertions file | `<scenario>.expect` (if it exists) |
| `--noise {off,n0,n1,n2}` | Sensor noise level | `off` |
| `--seed <int>` | Noise RNG seed | `12345` |
| `--duration <int>` | Simulation duration in microseconds | `25000000` (25 s) |

If `name` is a bench-type project, `sf app sils` explains why it cannot be
embedded and exits with code **2** (pointing you at `sf app new <name>
--from 11_app_controller`). Internally it takes the same path as `sf sils
scenario <scenario> --target apps/<name>` (`build_app_emulator()` builds a
dedicated emulator under `simulator/sils/build/apps/<name>`).

## 3. Examples

Create from the default example (`11_app_controller` = embedded-type `IController` template).

```bash
sf app new my_ctrl
```

Open the learner-facing file in your editor.

```bash
sf app edit my_ctrl
```

Verify in SILS (default scenario `alt_flight.scn`).

```bash
sf app sils my_ctrl
```

Or specify a different scenario.

```bash
sf app sils my_ctrl simulator/sils/scenarios/acro_flight.scn
```

Build for real hardware.

```bash
sf app build my_ctrl
```

Flash it.

```bash
sf app flash my_ctrl -m
```

Create from the template that only reads topics.

```bash
sf app new my_hello --from 12_app_task_hello
```

Create a bench-type project (learn the API without building the whole vehicle).

```bash
sf app new my_bench --from 10_custom_controller
```

List your projects and the examples available for `--from`.

```bash
sf app list
```

## 4. Output

On `sf app new` success (embedded-type):
```
[INFO] Creating new app project: my_ctrl
  From: /path/to/firmware/vehicle/examples/11_app_controller
  To:   /path/to/firmware/apps/my_ctrl
[OK] Project created: /path/to/firmware/apps/my_ctrl

Next steps:
  sf app sils my_ctrl
  sf app build my_ctrl
  sf app flash my_ctrl -m
```

`sf app list` columns: `Name` / `Type` (`embedded`/`bench`) / `Hardware`
(always `yes`) / `SILS` (`yes` or `no (bench)`) / `Description`.

## 5. `app.yaml`

`firmware/apps/<name>/app.yaml` is the manifest file recording a project's kind.

| Key | Content |
|-----|---------|
| `type` | `embedded` or `bench` |
| `from` | Name of the source example |
| `sils` | Whether SILS can run (`true`/`false`) |
| `name` | Project name |
| `description` | Description shown in listings (sometimes set automatically from the source's README heading) |

A clone of an example with no `app.yaml` (e.g. 09/10) is synthesized as
bench-type (`type: bench`, `sils: false`).

## 6. Relation to `sf sils --target apps/<name>`

`sf app sils <name>` is a thin wrapper around `sf sils scenario <scenario>
--target apps/<name>`. Passing `apps/<name>` directly to `sf sils
scenario`'s / `sf sils build`'s `--target`/`-t` takes the same path (`name`
must be embedded-type).

```bash
sf sils scenario simulator/sils/scenarios/alt_flight.scn --target apps/my_ctrl
```

## 7. Notes

- `sf app edit` opens `app_controller.cpp` (falling back to `app.cpp`) for an
  embedded-type project, or `main/learner_controller.cpp` (falling back to
  `main/main.cpp`) for a bench-type project (editor detection order: VSCode
  -> vi -> vim -> Notepad, Windows only).
- A bench-type project's `sf app build`/`sf app flash` simply delegate to `sf
  build apps/<name>`/`sf flash apps/<name>` internally, so using `sf
  build`/`sf flash` directly gives the same result.
