# Component Dependency Map

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

このドキュメントは `firmware/vehicle/components/` 配下の全コンポーネントの依存関係を記録する。
`refactor/platform-layer` ブランチでのリファクタリング（platform/app 分離）の基準点として使用する。

## 2. コンポーネント分類

### 移動先の計画

| 移動先 | コンポーネント数 | 条件 |
|--------|:--------------:|------|
| `firmware/platform/` | 26 | 汎用。vehicle 固有ロジックを含まない |
| `firmware/vehicle/components/` に残留 | 2 | vehicle 固有の飛行ロジック |
| `firmware/common/` 既存 | 1 | プロトコル定義（変更なし） |

## 3. 依存関係一覧

### アルゴリズム層（sf_algo_*） → platform 移動

全て ESP-IDF 非依存。純粋な C++ ライブラリ。

| コンポーネント | REQUIRES | PRIV_REQUIRES |
|-------------|----------|---------------|
| sf_algo_math | — | — |
| sf_algo_filter | — | — |
| sf_algo_pid | — | — |
| sf_algo_control | — | — |
| sf_algo_eskf | sf_algo_math | — |
| sf_algo_fusion | sf_algo_eskf, sf_algo_math | — |

### HAL 層（sf_hal_*） → platform 移動

ESP-IDF driver のみに依存。他コンポーネントへの依存なし。

| コンポーネント | REQUIRES | PRIV_REQUIRES |
|-------------|----------|---------------|
| sf_hal_bmi270 | driver, esp_timer | — |
| sf_hal_bmm150 | driver, esp_timer, nvs_flash | — |
| sf_hal_bmp280 | driver, esp_timer | — |
| sf_hal_button | driver | — |
| sf_hal_buzzer | driver, esp_timer | nvs_flash |
| sf_hal_led | driver, led_strip, esp_timer, nvs_flash | — |
| sf_hal_motor | driver, nvs_flash | — |
| sf_hal_pmw3901 | driver | — |
| sf_hal_power | driver, esp_timer | — |
| sf_hal_vl53l3cx | driver, esp_timer | — |

### ライブラリ層（sf_lib_*） → platform 移動

| コンポーネント | REQUIRES | PRIV_REQUIRES |
|-------------|----------|---------------|
| sf_lib_line_editor | — | — |

### サービス層（sf_svc_*） → platform 移動（一部条件付き）

| コンポーネント | REQUIRES | PRIV_REQUIRES | 備考 |
|-------------|----------|---------------|------|
| sf_svc_state | freertos, sf_algo_filter, **sf_svc_control_arbiter** | esp_timer | **循環依存あり** |
| sf_svc_control_arbiter | freertos, esp_common, log | **sf_svc_state** | **循環依存あり** |
| sf_svc_health | — | — | |
| sf_svc_logger | freertos, esp_timer | — | |
| sf_svc_comm | esp_wifi, esp_event, nvs_flash, freertos, sf_svc_led | — | |
| sf_svc_udp | protocol, lwip, freertos, esp_common, log | — | |
| sf_svc_led | sf_hal_led, sf_svc_state, freertos, esp_timer | — | |
| sf_svc_telemetry | esp_wifi, esp_http_server, esp_event, freertos, sf_svc_state | — | |
| sf_svc_serial_cli | console, freertos, sf_lib_line_editor, sf_svc_console | — | |
| sf_svc_wifi_cli | esp_wifi, lwip, freertos, sf_svc_console, sf_lib_line_editor | — | |

### サービス層 — vehicle 残留

| コンポーネント | REQUIRES | PRIV_REQUIRES | 残留理由 |
|-------------|----------|---------------|---------|
| sf_svc_flight_command | freertos, esp_timer, sf_svc_state, sf_svc_control_arbiter, sf_svc_comm, sf_algo_fusion, sf_algo_pid | — | vehicle 固有の飛行ロジック。vehicle/main/globals.hpp を直接参照 |
| sf_svc_console | console, nvs_flash, freertos, sf_svc_state, sf_hal_motor, sf_hal_buzzer, sf_hal_led, sf_hal_bmm150, sf_svc_led, sf_algo_fusion, sf_svc_logger, sf_svc_comm, sf_svc_udp, sf_svc_control_arbiter, sf_algo_control, sf_algo_pid, sf_svc_flight_command | — | 17個の依存。vehicle 固有 CLI コマンドを含む |

### 共通層（firmware/common/）

| コンポーネント | REQUIRES | PRIV_REQUIRES |
|-------------|----------|---------------|
| protocol | — | — |

## 4. 問題点（リファクタリング対象）

### 循環依存

```
sf_svc_state ──REQUIRES──→ sf_svc_control_arbiter
sf_svc_control_arbiter ──PRIV_REQUIRES──→ sf_svc_state
```

**解決方針:** コールバック化 + ControlSource enum の共有型への切り出し

### vehicle/main 直接参照

```
sf_svc_flight_command/CMakeLists.txt:
  target_include_directories(... PRIVATE ${COMPONENT_DIR}/../../main)
```

**解決方針:** 依存注入（DI）パターンで globals 参照を除去

### sf_svc_console の過剰依存

17 コンポーネントに依存。vehicle 固有の CLI コマンド（motor test, calibration 等）を含むため、
platform 共通部分と vehicle 固有部分の分割が必要。

## 5. 依存関係グラフ（簡略版）

```
                    sf_algo_math
                   ╱            ╲
          sf_algo_eskf      sf_algo_filter
               │                  │
          sf_algo_fusion    sf_svc_state ←──╮ 循環
               │                  │         │
               │           sf_svc_control_arbiter
               │                  │
          sf_svc_flight_command ──┘
               │
          vehicle/main (globals.hpp)

  sf_hal_*  ← ESP-IDF driver のみ（独立）

  sf_svc_comm → sf_svc_led → sf_hal_led
                           → sf_svc_state

  sf_svc_console → (17個に依存 — 最大のハブ)
```

---

<a id="english"></a>

## 1. Overview

This document records the dependency relationships of all components under `firmware/vehicle/components/`.
Used as a baseline for the platform/app separation refactoring on the `refactor/platform-layer` branch.

## 2. Component Classification

| Destination | Count | Criteria |
|-------------|:-----:|----------|
| `firmware/platform/` | 26 | Generic, no vehicle-specific logic |
| Remains in `firmware/vehicle/components/` | 2 | Vehicle-specific flight logic |
| `firmware/common/` (existing) | 1 | Protocol definitions (no change) |

## 3. Issues to Resolve

| Issue | Components | Resolution |
|-------|-----------|------------|
| Circular dependency | sf_svc_state <-> sf_svc_control_arbiter | Callback + shared enum extraction |
| Direct main/ reference | sf_svc_flight_command | Dependency injection |
| Excessive dependencies | sf_svc_console (17 deps) | Split platform/vehicle CLI commands |
