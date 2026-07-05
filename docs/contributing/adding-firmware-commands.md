# CLI コマンド追加ガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFly の CLI システムは ESP-IDF Console (`esp_console`) を基盤としており、USB-CDC の対話コンソール（シリアル REPL）と WiFi 経由の TCP CLI（`nc <機体IP> 23`）の両方から同じコマンド集合が使えます。

実装は `firmware/vehicle/tasks/cli_task.cpp` の**単一ファイル**に集約されています。`cli_task.cpp` はコンポーネント（`components/sf_*`）ではなく `firmware/vehicle/tasks/` 配下のタスクソースで、`main/CMakeLists.txt` の `idf_component_register` に直接 SRCS として列挙され、`main` の一部としてビルドされます。

> 旧ファーム（`firmware/vehicle_old/`）では `sf_svc_console` コンポーネント配下にカテゴリ別 `commands/cmd_system.cpp` / `cmd_sensor.cpp` / `cmd_motor.cpp` … というファイル分割と、`Console::getInstance()` シングルトンによる出力がありました。この構成は**現行の `firmware/vehicle/` には存在しません**。以降の説明は現行 `firmware/vehicle/` の構成に基づきます。旧ファームのコマンドを触る場合は `firmware/vehicle_old/components/sf_svc_console/` を参照してください（新規開発の対象外）。

### アーキテクチャ

```
┌───────────────────────────────────────────────────────────────┐
│              firmware/vehicle/tasks/cli_task.cpp               │
│  ┌───────────────────────────────────────────────────────┐    │
│  │   コマンドハンドラ（無名 namespace 内、この層に追加）      │    │
│  │   int cmd_param(int argc, char** argv)                 │    │
│  │   int cmd_status(...)                                  │    │
│  │   int cmd_sensor(...)                                  │    │
│  │   int cmd_motor(...)                                    │    │
│  │   ...                                                    │    │
│  └───────────────────────────────────────────────────────┘    │
│                          ↓                                     │
│   const CliCommand kCommands[] = {{name, help, &cmd_x}, ...};  │
│                          ↓ registerCommands() ループ            │
│              esp_console_cmd_register()                        │
└───────────────────────────────────────────────────────────────┘
                    ↑                        ↑
        ┌───────────┴───────┐    ┌───────────┴───────────┐
        │  USB-CDC 対話REPL   │    │  TCP CLI (port 23)     │
        └───────────────────┘    └───────────────────────┘
```

この構成は architecture.md の横断ルール **R6**（CLI コマンドはレジストリパターン `{name, callback}` 配列で登録し、CLI 用の extern グローバルポインタを作らない）に対応します。

## 2. コマンド追加手順

### ステップ1: ハンドラ関数を実装

`cli_task.cpp` の無名 namespace 内、既存ハンドラ（`cmd_version`、`cmd_led` など）の近くに追加します。シグネチャは ESP-IDF Console の規約どおり `int cmd_<name>(int argc, char** argv)`（無名 namespace 内なので `static` は不要、既存コードもつけていません）。

```cpp
/// `mycommand [status|set <value>]` — 一言で機能を説明（英語）。
/// `mycommand [status|set <value>]` — 同じ説明（日本語）。
int cmd_mycommand(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: mycommand [status|set <value>]\n");
        return 0;
    }
    if (std::strcmp(argv[1], "status") == 0) {
        std::printf("status: OK\n");
        return 0;
    }
    if (std::strcmp(argv[1], "set") == 0 && argc >= 3) {
        const int value = std::atoi(argv[2]);
        std::printf("value set to: %d\n", value);
        return 0;
    }
    std::printf("unknown subcommand: %s\n", argv[1]);
    return 1;
}
```

### ステップ2: コマンドテーブルへ登録

同ファイル内の `kCommands[]` 配列にエントリを1行追加するだけです。個別の `register_xxx_commands()` 関数やカテゴリ別ファイルへの追記は不要です — 単一の `registerCommands()` ループが `kCommands[]` を読んで `esp_console_cmd_register()` に登録します。

```cpp
const CliCommand kCommands[] = {
    // ... 既存のコマンド ...
    {"mycommand", "mycommand [status|set <value>] — my custom command", &cmd_mycommand},  // ★ 追加
};
```

### ステップ3: help / Tab補完について

`help` コマンドは ESP-IDF 組み込みの `esp_console_register_help_command()` が `kCommands[]` の `help` 文字列から自動生成するため、手動更新は不要です。同様に、現行実装には旧ファームの `wifi_cli.cpp` のような独立した Tab 補完リストは存在しないため、保守すべき補完リストもありません。

## 3. コード例

現行 `cli_task.cpp` の実例に基づく3パターンです。

### 引数なしのコマンド（`cmd_version` 参考）

```cpp
int cmd_version(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("StampFly vehicle firmware\n");
    std::printf("build : %s %s\n", __DATE__, __TIME__);
    return 0;
}
```

### 数値引数＋Pub-Sub発行のコマンド（`cmd_led` 参考）

他コンポーネントの状態を変えるコマンドは、直接関数を呼ぶのではなく Topic を `publish()` します（後述「4. ベストプラクティス」参照）。

```cpp
int cmd_led(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: led <0-255>\n");
        return 0;
    }
    int b = std::atoi(argv[1]);
    if (b < 0)   b = 0;
    if (b > 255) b = 255;
    sf::UiCommand c{};
    c.command   = static_cast<uint8_t>(sf::UiCmd::LedBrightness);
    c.value     = static_cast<uint8_t>(b);
    c.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::ui_command.publish(c);
    std::printf("led brightness = %d\n", b);
    return 0;
}
```

### Topic を読み取るコマンド（`cmd_sensor` の power 分岐 参考）

```cpp
if (all || std::strcmp(which, "power") == 0) {
    const sf::PowerData d = sf::sensor_power.latest();
    std::printf("power: %.2f V  %.0f mA  %.0f mW\n", d.voltage, d.current, d.power);
}
```

## 4. ベストプラクティス

- **改行は `\n`。** USB-CDC REPL・TCP CLI とも通常の行末で十分です（旧ファームの Telnet 互換 `\r\n` は不要）
- **引数を検証してから処理する。** 不正な引数はエラーメッセージを表示し、`0` 以外の終了コードを返す
- **他タスクの内部に直接アクセスしない。** architecture.md の横断ルール **R5**（コンポーネント間は Pub-Sub Topic 経由で通信し、直接依存しない）に従い、状態の読み取りは Topic の `.latest()`、状態の変更は Topic への `.publish()` のみで行う。パラメータの読み書きは `sf::params::get_*`/`set_*` を使う（`cmd_param` を参照）
- **長時間処理は `vTaskDelay()` で他タスクを飢餓させない。** ブロッキング処理を書く場合も CLITask 以外のタスク（飛行制御系）を止めない
- **全ハンドラにバイリンガルコメント（英語→日本語）を付ける。** 既存ハンドラ（`cmd_pair`、`cmd_motor` など）のスタイルに倣う

---

<a id="english"></a>

## 1. Overview

StampFly's CLI system is built on ESP-IDF Console (`esp_console`). The same command set is reachable from both the USB-CDC interactive console (serial REPL) and a WiFi-based TCP CLI (`nc <vehicle-ip> 23`).

The implementation lives in a **single file**, `firmware/vehicle/tasks/cli_task.cpp`. It is not a component (`components/sf_*`) but a task source under `firmware/vehicle/tasks/`, listed directly as a SRCS entry in `main/CMakeLists.txt`'s `idf_component_register` and built as part of `main`.

> The legacy firmware (`firmware/vehicle_old/`) had a `sf_svc_console` component with category-split files (`commands/cmd_system.cpp`, `cmd_sensor.cpp`, `cmd_motor.cpp`, …) and output through a `Console::getInstance()` singleton. **That structure no longer exists in current `firmware/vehicle/`.** Everything below describes the current `firmware/vehicle/` layout. To touch legacy commands, see `firmware/vehicle_old/components/sf_svc_console/` (not a target for new development).

### Architecture

```
┌───────────────────────────────────────────────────────────────┐
│              firmware/vehicle/tasks/cli_task.cpp               │
│  ┌───────────────────────────────────────────────────────┐    │
│  │   Command handlers (add here, inside the anon.        │    │
│  │   namespace)                                            │    │
│  │   int cmd_param(int argc, char** argv)                 │    │
│  │   int cmd_status(...)                                  │    │
│  │   int cmd_sensor(...)                                  │    │
│  │   int cmd_motor(...)                                    │    │
│  │   ...                                                    │    │
│  └───────────────────────────────────────────────────────┘    │
│                          ↓                                     │
│   const CliCommand kCommands[] = {{name, help, &cmd_x}, ...};  │
│                          ↓ registerCommands() loop              │
│              esp_console_cmd_register()                        │
└───────────────────────────────────────────────────────────────┘
                    ↑                        ↑
        ┌───────────┴───────┐    ┌───────────┴───────────┐
        │ USB-CDC interactive│    │  TCP CLI (port 23)     │
        │       REPL          │    │                        │
        └───────────────────┘    └───────────────────────┘
```

This matches architecture.md's cross-cutting rule **R6** (CLI commands use a registry pattern — a `{name, callback}` array — with no extern global pointer for CLI).

## 2. Adding a Command

### Step 1: Implement the handler

Add it inside the anonymous namespace in `cli_task.cpp`, near existing handlers (`cmd_version`, `cmd_led`, etc.). Signature follows the ESP-IDF Console convention: `int cmd_<name>(int argc, char** argv)` (no `static` needed — the anonymous namespace already gives internal linkage, and existing handlers omit it).

```cpp
/// `mycommand [status|set <value>]` — one-line description.
/// `mycommand [status|set <value>]` — same description in Japanese.
int cmd_mycommand(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: mycommand [status|set <value>]\n");
        return 0;
    }
    if (std::strcmp(argv[1], "status") == 0) {
        std::printf("status: OK\n");
        return 0;
    }
    if (std::strcmp(argv[1], "set") == 0 && argc >= 3) {
        const int value = std::atoi(argv[2]);
        std::printf("value set to: %d\n", value);
        return 0;
    }
    std::printf("unknown subcommand: %s\n", argv[1]);
    return 1;
}
```

### Step 2: Register it in the command table

Add one entry to the `kCommands[]` array in the same file. There is no separate `register_xxx_commands()` function and no category file to edit — a single `registerCommands()` loop reads `kCommands[]` and calls `esp_console_cmd_register()` for each entry.

```cpp
const CliCommand kCommands[] = {
    // ... existing commands ...
    {"mycommand", "mycommand [status|set <value>] — my custom command", &cmd_mycommand},  // ★ added
};
```

### Step 3: help / tab completion

The built-in `help` command (`esp_console_register_help_command()`) generates its listing from each entry's `help` string in `kCommands[]`, so there is nothing to update by hand. Likewise, there is no separate tab-completion list to maintain in the current implementation (unlike the legacy firmware's `wifi_cli.cpp`).

## 3. Code Examples

Three patterns grounded in the current `cli_task.cpp`.

### No-argument command (cf. `cmd_version`)

```cpp
int cmd_version(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("StampFly vehicle firmware\n");
    std::printf("build : %s %s\n", __DATE__, __TIME__);
    return 0;
}
```

### Numeric argument + Pub-Sub publish (cf. `cmd_led`)

A command that changes another component's state publishes a Topic instead of calling a function directly (see "4. Best Practices" below).

```cpp
int cmd_led(int argc, char** argv)
{
    if (argc < 2) {
        std::printf("usage: led <0-255>\n");
        return 0;
    }
    int b = std::atoi(argv[1]);
    if (b < 0)   b = 0;
    if (b > 255) b = 255;
    sf::UiCommand c{};
    c.command   = static_cast<uint8_t>(sf::UiCmd::LedBrightness);
    c.value     = static_cast<uint8_t>(b);
    c.timestamp = static_cast<uint32_t>(esp_timer_get_time());
    sf::ui_command.publish(c);
    std::printf("led brightness = %d\n", b);
    return 0;
}
```

### Reading a Topic (cf. the `power` branch of `cmd_sensor`)

```cpp
if (all || std::strcmp(which, "power") == 0) {
    const sf::PowerData d = sf::sensor_power.latest();
    std::printf("power: %.2f V  %.0f mA  %.0f mW\n", d.voltage, d.current, d.power);
}
```

## 4. Best Practices

- **Use `\n` for line endings.** Plain newlines are fine for both the USB-CDC REPL and the TCP CLI (the legacy firmware's Telnet-compatible `\r\n` is not needed)
- **Validate arguments before processing.** Print an error message and return non-zero on invalid input
- **Never reach into another task's internals.** Follow architecture.md's cross-cutting rule **R5** (components communicate only via Pub-Sub Topics, no direct dependencies): read state via a Topic's `.latest()`, change state via `.publish()`. Read/write parameters via `sf::params::get_*`/`set_*` (see `cmd_param`)
- **Don't starve other tasks with long blocking operations.** Use `vTaskDelay()` if a command must wait, so the flight-control tasks are not starved
- **Add bilingual comments (English then Japanese) to every handler**, following the style of existing handlers (`cmd_pair`, `cmd_motor`, etc.)
