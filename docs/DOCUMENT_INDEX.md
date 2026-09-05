# ドキュメント目録 / Document Index

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. プロジェクトルート

| ファイル | 説明 |
|---------|------|
| [README.md](../README.md) | プロジェクト全体の紹介・クイックスタート |
| [PROJECT_PLAN.md](../PROJECT_PLAN.md) | プロジェクト全体の設計方針・計画 |
| [CLAUDE.md](../CLAUDE.md) | Claude Code 向けプロジェクト設定 |

## 2. ユーザー向けドキュメント (`docs/`)

### 入門・セットアップ

| ファイル | 説明 |
|---------|------|
| [getting-started.md](getting-started.md) | 環境構築〜初フライト（ステップバイステップ） |
| [overview.md](overview.md) | プロジェクト全体の俯瞰図 |
| [setup/README.md](setup/README.md) | セットアップガイド（概要） |
| [setup/macos.md](setup/macos.md) | macOS セットアップ手順 |
| [setup/linux.md](setup/linux.md) | Linux (Ubuntu/Debian) セットアップ手順 |
| [setup/windows.md](setup/windows.md) | Windows (WSL2) セットアップ手順 |

### sf CLI コマンドリファレンス

| ファイル | 説明 |
|---------|------|
| [commands/README.md](commands/README.md) | コマンド一覧・クイックリファレンス |
| [commands/sf-version.md](commands/sf-version.md) | `sf version` - バージョン情報 |
| [commands/sf-doctor.md](commands/sf-doctor.md) | `sf doctor` - 環境診断 |
| [commands/sf-setup.md](commands/sf-setup.md) | `sf setup` - 依存パッケージインストール |
| [commands/sf-build.md](commands/sf-build.md) | `sf build` - ファームウェアビルド |
| [commands/sf-flash.md](commands/sf-flash.md) | `sf flash` - ファームウェア書き込み |
| [commands/sf-monitor.md](commands/sf-monitor.md) | `sf monitor` - シリアルモニタ |
| [commands/sf-log.md](commands/sf-log.md) | `sf log` - ログキャプチャ・解析 |
| [commands/sf-sim.md](commands/sf-sim.md) | `sf sim` - シミュレータ |
| [commands/sf-cal.md](commands/sf-cal.md) | `sf cal` - センサキャリブレーション |

### アーキテクチャ・設計

| ファイル | 説明 |
|---------|------|
| [architecture/control-system.md](architecture/control-system.md) | 制御系設計 |
| [architecture/control-allocation-migration.md](architecture/control-allocation-migration.md) | 制御配分マイグレーション |
| [architecture/coordinate-systems.md](architecture/coordinate-systems.md) | 座標系の定義 |
| [architecture/stampfly-parameters.md](architecture/stampfly-parameters.md) | 物理パラメータリファレンス |
| [architecture/genesis-integration.md](architecture/genesis-integration.md) | Genesis シミュレータ統合 |
| [architecture/simulation-policy.md](architecture/simulation-policy.md) | シミュレーション方針の正 — 3層構造・モデル一致ゲート・SILSプラント改修バックログ |

### ガイド・リファレンス (`guides/`)

| ファイル | 説明 |
|---------|------|
| [guides/safety.md](guides/safety.md) | 安全ガイド |
| [guides/troubleshooting.md](guides/troubleshooting.md) | トラブルシューティング |
| [guides/glossary.md](guides/glossary.md) | 用語集 |
| [guides/tools.md](guides/tools.md) | 開発ツール使用ガイド |
| [guides/flight-log-viz.md](guides/flight-log-viz.md) | フライトログ可視化チュートリアル |
| [guides/block_programming.md](guides/block_programming.md) | ブロックプログラミングガイド（`sf blocks`）|
| [setup/wifi-sta.md](setup/wifi-sta.md) | WiFi STA モードセットアップ |
| [architecture/tello-api-reference.md](architecture/tello-api-reference.md) | Tello 互換 API リファレンス |
| [architecture/ros2-udp-debug.md](architecture/ros2-udp-debug.md) | ROS2 UDP 制御デバッグガイド |

### 開発ガイドライン

| ファイル | 説明 |
|---------|------|
| [README.md](README.md) | docs/ ディレクトリガイド |
| [contributing/style-guide.md](contributing/style-guide.md) | ドキュメント記述スタイル規約 |
| [contributing/commit-guidelines.md](contributing/commit-guidelines.md) | コミットメッセージ規約 |
| [contributing/adding-sf-commands.md](contributing/adding-sf-commands.md) | コマンド追加ガイド |
| [contributing/adding-firmware-commands.md](contributing/adding-firmware-commands.md) | CLI コマンド追加ガイド |
| [contributing/scaffolding-tool.md](contributing/scaffolding-tool.md) | スキャフォールディングツール |
| [contributing/claude-code-practices.md](contributing/claude-code-practices.md) | Claude Code 活用ノウハウ |

### 教育資料

| ファイル | 説明 |
|---------|------|
| [setup/education.md](setup/education.md) | 教育向けセットアップガイド |
| [university/syllabus.md](university/syllabus.md) | 大学講義シラバス |
| [university/assessment_rubric.md](university/assessment_rubric.md) | 評価ルーブリック |
| [events/stampfly_workshop/workshop_guide.md](events/stampfly_workshop/workshop_guide.md) | ワークショップガイド |
| [events/stampfly_workshop/workshop_schedule.md](events/stampfly_workshop/workshop_schedule.md) | ワークショップスケジュール |
| [events/stampfly_workshop/competition_rules.md](events/stampfly_workshop/competition_rules.md) | 競技ルール |
| [events/_shared/README.md](events/_shared/README.md) | スライド資料ガイド |
| [events/sci_tutorial_2026/README.md](events/sci_tutorial_2026/README.md) | SCI/SICE チュートリアル講座 2026: 概要・事前準備・資料索引 |
| [events/sci_tutorial_2026/handson_guide.md](events/sci_tutorial_2026/handson_guide.md) | チュートリアル復習手順ガイド（セッション別） |
| [events/sci_tutorial_2026/cheatsheet.md](events/sci_tutorial_2026/cheatsheet.md) | sf CLI・ws:: API 早見表 |

### 開発計画 (`plans/`)

| ファイル | 説明 |
|---------|------|
| [plans/education-outreach-strategy.md](plans/education-outreach-strategy.md) | 教育普及戦略（階層別展開・3 Horizons） |
| [plans/ecosystem-migration.md](plans/ecosystem-migration.md) | エコシステムマイグレーション計画 |
| [plans/installer-architecture.md](plans/installer-architecture.md) | インストーラ設計 |
| [plans/archive/controller-menu-usb.md](plans/archive/controller-menu-usb.md) | コントローラメニュー・USB 計画（完了） |
| [plans/hardware-test.md](plans/hardware-test.md) | 実機テスト計画 |
| [plans/archive/hils-firmware.md](plans/archive/hils-firmware.md) | HILS ファームウェア計画（未実施） |
| [plans/ros2-integration.md](plans/ros2-integration.md) | ROS2 統合計画 |
| [plans/archive/serial-cli-rebuild.md](plans/archive/serial-cli-rebuild.md) | シリアル CLI 再構築計画（完了） |
| [plans/archive/tello-compat.md](plans/archive/tello-compat.md) | Tello 互換計画 |
| [plans/archive/wifi-comm.md](plans/archive/wifi-comm.md) | WiFi 通信計画 |
| [plans/archive/wifi-command-implementation.md](plans/archive/wifi-command-implementation.md) | WiFi コマンド実装計画 |

## 3. ファームウェアドキュメント

### Vehicle（機体）

| ファイル | 説明 |
|---------|------|
| [firmware/vehicle/README.md](../firmware/vehicle/README.md) | 機体ファームウェア全体ガイド |
| [plans/vehicle-firmware.md](plans/vehicle-firmware.md) | 設計方針 |
| [plans/archive/vehicle-phase3-test.md](plans/archive/vehicle-phase3-test.md) | Phase3 テスト計画 |
| [plans/archive/vehicle-phase3-test-guide.md](plans/archive/vehicle-phase3-test-guide.md) | Phase3 テストガイド |

### Vehicle 設計文書 (`firmware/vehicle/docs/`)

現行ファームウェアの設計文書一式。中核となる6文書：

| ファイル | 説明 |
|---------|------|
| [firmware/vehicle/docs/requirements.md](../firmware/vehicle/docs/requirements.md) | 要件定義書 |
| [firmware/vehicle/docs/architecture.md](../firmware/vehicle/docs/architecture.md) | アーキテクチャ設計書（4階層アクセス・横断ルール R1〜R16・BSP 層） |
| [firmware/vehicle/docs/detailed_design.md](../firmware/vehicle/docs/detailed_design.md) | 詳細設計書 |
| [firmware/vehicle/docs/coding_and_education.md](../firmware/vehicle/docs/coding_and_education.md) | コーディング方針・教育計画 |
| [firmware/vehicle/docs/development_roadmap.md](../firmware/vehicle/docs/development_roadmap.md) | 開発ロードマップ・SILS→実機ワークフロー |
| [firmware/vehicle/docs/hardware_init.md](../firmware/vehicle/docs/hardware_init.md) | ハードウェア初期化設計（BSP・起動シーケンス） |

上記6文書に加え、`firmware/vehicle/docs/` にはフライト調査・不具合診断ノート（`poshold_journey.md`、`feature_status.md`、`operation_manual.md`、`topic_reference.md`、`control_theory_overview.md`、`yaw_axis_model.md` 等、計約28ファイル）が格納されている。

### Vehicle コンポーネントドキュメント

| ファイル | 説明 |
|---------|------|
| [firmware/vehicle/components/sf_hal_bmi270/README.md](../firmware/vehicle/components/sf_hal_bmi270/README.md) | BMI270 IMU ドライバ |
| [firmware/vehicle/components/sf_hal_bmi270/docs/API.md](../firmware/vehicle/components/sf_hal_bmi270/docs/API.md) | BMI270 API リファレンス |
| [firmware/vehicle/components/sf_hal_bmi270/docs/bmi270_doc_ja.md](../firmware/vehicle/components/sf_hal_bmi270/docs/bmi270_doc_ja.md) | BMI270 日本語ドキュメント |
| [firmware/vehicle/components/sf_hal_bmi270/docs/esp_idf_bmi270_spi_guide.md](../firmware/vehicle/components/sf_hal_bmi270/docs/esp_idf_bmi270_spi_guide.md) | ESP-IDF BMI270 SPI ガイド |
| [firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md](../firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md) | M5StampFly ハードウェア仕様 |
| [firmware/vehicle/components/sf_hal_pmw3901/README.md](../firmware/vehicle/components/sf_hal_pmw3901/README.md) | PMW3901 オプティカルフロードライバ |
| [firmware/vehicle/components/sf_hal_pmw3901/docs/how_to_use_pwm3901.md](../firmware/vehicle/components/sf_hal_pmw3901/docs/how_to_use_pwm3901.md) | PMW3901 使用方法 |
| [firmware/vehicle/components/sf_hal_pmw3901/docs/PMW3901_IMPLEMENTATION_REFERENCE.md](../firmware/vehicle/components/sf_hal_pmw3901/docs/PMW3901_IMPLEMENTATION_REFERENCE.md) | PMW3901 実装リファレンス |
| [firmware/vehicle/components/sf_hal_pmw3901/docs/STAMPFLY_HARDWARE_SPEC.md](../firmware/vehicle/components/sf_hal_pmw3901/docs/STAMPFLY_HARDWARE_SPEC.md) | StampFly ハードウェア仕様 |
| [firmware/vehicle/components/sf_hal_vl53l3cx/README.md](../firmware/vehicle/components/sf_hal_vl53l3cx/README.md) | VL53L3CX ToF ドライバ |
| [firmware/vehicle/components/sf_hal_vl53l3cx/docs/API.md](../firmware/vehicle/components/sf_hal_vl53l3cx/docs/API.md) | VL53L3CX API リファレンス |
| [firmware/vehicle/components/sf_hal_vl53l3cx/docs/VL53L3CX_driver_doc.md](../firmware/vehicle/components/sf_hal_vl53l3cx/docs/VL53L3CX_driver_doc.md) | VL53L3CX ドライバドキュメント |
| [firmware/vehicle/components/sf_hal_vl53l3cx/docs/i2c_master_api_analysis.md](../firmware/vehicle/components/sf_hal_vl53l3cx/docs/i2c_master_api_analysis.md) | I2C マスタ API 分析 |

### Vehicle サンプルプロジェクト

| ディレクトリ | 説明 |
|-------------|------|
| [firmware/vehicle/components/sf_hal_bmi270/examples/](../firmware/vehicle/components/sf_hal_bmi270/examples/) | BMI270 使用例（polling, interrupt, FIFO, 開発ステージ） |
| [firmware/vehicle/components/sf_hal_vl53l3cx/examples/](../firmware/vehicle/components/sf_hal_vl53l3cx/examples/) | VL53L3CX 使用例（polling, interrupt, 開発ステージ） |

### Vehicle Old（レガシー機体ファームウェア）

POS_HOLD 位置制御の実機検証を機に `firmware/vehicle_new` が `firmware/vehicle` へ昇格し、旧世代ファームウェアは `firmware/vehicle_old` として凍結された（旧 `sf_hal_*`/`sf_algo_*`/`sf_svc_*` 階層命名構成）。実機87フライトの実績があるが新規開発は行わない。

| ファイル | 説明 |
|---------|------|
| [firmware/vehicle_old/README.md](../firmware/vehicle_old/README.md) | レガシー機体ファームウェア全体ガイド |
| [firmware/vehicle_old/components/sf_algo_fusion/README.md](../firmware/vehicle_old/components/sf_algo_fusion/README.md) | センサーフュージョンアルゴリズム |

### Controller（コントローラ）

| ファイル | 説明 |
|---------|------|
| [firmware/controller/README.md](../firmware/controller/README.md) | コントローラファームウェア全体ガイド |
| [architecture/tdma-usage.md](architecture/tdma-usage.md) | TDMA 通信詳細ガイド |
| [plans/archive/controller-espidf-migration.md](plans/archive/controller-espidf-migration.md) | ESP-IDF マイグレーション計画 |

### Common（共有コード）

| ファイル | 説明 |
|---------|------|
| [firmware/common/README.md](../firmware/common/README.md) | 共有コード（`protocol/` は ESP-NOW 通信プロトコル実装済み・vehicle/vehicle_old/controller が使用、`math/`・`utils/` は未実装のプレースホルダ） |

### Workshop（ワークショップ教材）

| ファイル | 説明 |
|---------|------|
| [firmware/workshop/README.md](../firmware/workshop/README.md) | ワークショップ教材の概要 |
| [firmware/workshop/lessons/lesson_00_setup/](../firmware/workshop/lessons/lesson_00_setup/README.md) | Lesson 0: セットアップ |
| [firmware/workshop/lessons/lesson_01_motor/](../firmware/workshop/lessons/lesson_01_motor/README.md) | Lesson 1: モーター制御 |
| [firmware/workshop/lessons/lesson_02_controller/](../firmware/workshop/lessons/lesson_02_controller/README.md) | Lesson 2: コントローラ |
| [firmware/workshop/lessons/lesson_03_led/](../firmware/workshop/lessons/lesson_03_led/README.md) | Lesson 3: LED 制御 |
| [firmware/workshop/lessons/lesson_04_imu/](../firmware/workshop/lessons/lesson_04_imu/README.md) | Lesson 4: IMU |
| [firmware/workshop/lessons/lesson_05_p_control/](../firmware/workshop/lessons/lesson_05_p_control/README.md) | Lesson 5: P 制御 |
| [firmware/workshop/lessons/lesson_06_modeling/](../firmware/workshop/lessons/lesson_06_modeling/README.md) | Lesson 6: モデリング |
| [firmware/workshop/lessons/lesson_07_sysid/](../firmware/workshop/lessons/lesson_07_sysid/README.md) | Lesson 7: システム同定 |
| [firmware/workshop/lessons/lesson_08_pid/](../firmware/workshop/lessons/lesson_08_pid/README.md) | Lesson 8: PID 制御 |
| [firmware/workshop/lessons/lesson_09_estimation/](../firmware/workshop/lessons/lesson_09_estimation/README.md) | Lesson 9: 状態推定 |
| [firmware/workshop/lessons/lesson_10_api_overview/](../firmware/workshop/lessons/lesson_10_api_overview/README.md) | Lesson 10: API 概要 |
| [firmware/workshop/lessons/lesson_12_python_sdk/](../firmware/workshop/lessons/lesson_12_python_sdk/README.md) | Lesson 12: Python SDK |
| [firmware/workshop/lessons/lesson_13_competition/](../firmware/workshop/lessons/lesson_13_competition/README.md) | Lesson 13: 競技 |

## 4. その他のディレクトリ

| ファイル | 説明 |
|---------|------|
| [protocol/README.md](../protocol/README.md) | 通信プロトコル仕様（構築中） |
| [control/README.md](../control/README.md) | 制御設計資産（構築中） |
| [control/design/loop_shaping_tool/README.md](../control/design/loop_shaping_tool/README.md) | ループシェイピングツール |
| [analysis/README.md](../analysis/README.md) | 実験データ解析（構築中） |
| [analysis/datasets/README.md](../analysis/datasets/README.md) | データセット |
| [analysis/notebooks/README.md](../analysis/notebooks/README.md) | Jupyter ノートブック |
| [analysis/scripts/README.md](../analysis/scripts/README.md) | 解析スクリプト |
| [tools/README.md](../tools/README.md) | 補助ツール（構築中） |
| [tools/calibration/README.md](../tools/calibration/README.md) | キャリブレーションツール |
| [tools/log_analyzer/README.md](../tools/log_analyzer/README.md) | ログ解析ツール |
| [tools/log_capture/README.md](../tools/log_capture/README.md) | ログキャプチャツール |
| [simulator/README.md](../simulator/README.md) | シミュレータ概要 |
| [simulator/genesis/README.md](../simulator/genesis/README.md) | Genesis シミュレータ |
| [simulator/genesis/docs/urdf_mesh_normals.md](../simulator/genesis/docs/urdf_mesh_normals.md) | URDF メッシュ法線ガイド |
| [simulator/sandbox/README.md](../simulator/sandbox/README.md) | サンドボックス |
| [plans/simulator-migration.md](plans/simulator-migration.md) | シミュレータマイグレーション計画 |
| [ros/README.md](../ros/README.md) | ROS 連携（構築中） |
| [examples/README.md](../examples/README.md) | 学習用サンプル |

---

<a id="english"></a>

# Document Index

## 1. Project Root

| File | Description |
|------|-------------|
| [README.md](../README.md) | Project introduction and quick start |
| [PROJECT_PLAN.md](../PROJECT_PLAN.md) | Overall project design policy and plan |
| [CLAUDE.md](../CLAUDE.md) | Claude Code project configuration |

## 2. User Documentation (`docs/`)

### Getting Started & Setup

| File | Description |
|------|-------------|
| [getting-started.md](getting-started.md) | Environment setup to first flight (step-by-step) |
| [overview.md](overview.md) | Project overview |
| [setup/README.md](setup/README.md) | Setup guide (overview) |
| [setup/macos.md](setup/macos.md) | macOS setup instructions |
| [setup/linux.md](setup/linux.md) | Linux (Ubuntu/Debian) setup instructions |
| [setup/windows.md](setup/windows.md) | Windows (WSL2) setup instructions |

### sf CLI Command Reference

| File | Description |
|------|-------------|
| [commands/README.md](commands/README.md) | Command list and quick reference |
| [commands/sf-version.md](commands/sf-version.md) | `sf version` - Version info |
| [commands/sf-doctor.md](commands/sf-doctor.md) | `sf doctor` - Environment diagnostics |
| [commands/sf-setup.md](commands/sf-setup.md) | `sf setup` - Install dependencies |
| [commands/sf-build.md](commands/sf-build.md) | `sf build` - Build firmware |
| [commands/sf-flash.md](commands/sf-flash.md) | `sf flash` - Flash firmware |
| [commands/sf-monitor.md](commands/sf-monitor.md) | `sf monitor` - Serial monitor |
| [commands/sf-log.md](commands/sf-log.md) | `sf log` - Log capture and analysis |
| [commands/sf-sim.md](commands/sf-sim.md) | `sf sim` - Simulator |
| [commands/sf-cal.md](commands/sf-cal.md) | `sf cal` - Sensor calibration |

### Architecture & Design

| File | Description |
|------|-------------|
| [architecture/control-system.md](architecture/control-system.md) | Control system design |
| [architecture/control-allocation-migration.md](architecture/control-allocation-migration.md) | Control allocation migration |
| [architecture/coordinate-systems.md](architecture/coordinate-systems.md) | Coordinate system definitions |
| [architecture/stampfly-parameters.md](architecture/stampfly-parameters.md) | Physical parameters reference |
| [architecture/genesis-integration.md](architecture/genesis-integration.md) | Genesis simulator integration |
| [architecture/simulation-policy.md](architecture/simulation-policy.md) | Authoritative simulation policy — three-layer structure, model-identity gate, SILS plant retrofit backlog |

### Guides & Reference (`guides/`)

| File | Description |
|------|-------------|
| [guides/safety.md](guides/safety.md) | Safety guide |
| [guides/troubleshooting.md](guides/troubleshooting.md) | Troubleshooting |
| [guides/glossary.md](guides/glossary.md) | Glossary |
| [guides/tools.md](guides/tools.md) | Development tools guide |
| [guides/flight-log-viz.md](guides/flight-log-viz.md) | Flight log visualization tutorial |
| [guides/block_programming.md](guides/block_programming.md) | Block programming guide (`sf blocks`) |
| [setup/wifi-sta.md](setup/wifi-sta.md) | WiFi STA mode setup |
| [architecture/tello-api-reference.md](architecture/tello-api-reference.md) | Tello-compatible API reference |
| [architecture/ros2-udp-debug.md](architecture/ros2-udp-debug.md) | ROS2 UDP control debug guide |

### Development Guidelines

| File | Description |
|------|-------------|
| [README.md](README.md) | docs/ directory guide |
| [contributing/style-guide.md](contributing/style-guide.md) | Document writing style guide |
| [contributing/commit-guidelines.md](contributing/commit-guidelines.md) | Commit message guidelines |
| [contributing/adding-sf-commands.md](contributing/adding-sf-commands.md) | Adding commands guide |
| [contributing/adding-firmware-commands.md](contributing/adding-firmware-commands.md) | Adding CLI commands guide |
| [contributing/scaffolding-tool.md](contributing/scaffolding-tool.md) | Scaffolding tool |
| [contributing/claude-code-practices.md](contributing/claude-code-practices.md) | Claude Code Best Practices |

### Educational Materials

| File | Description |
|------|-------------|
| [setup/education.md](setup/education.md) | Education-oriented setup guide |
| [university/syllabus.md](university/syllabus.md) | University course syllabus |
| [university/assessment_rubric.md](university/assessment_rubric.md) | Assessment rubric |
| [events/stampfly_workshop/workshop_guide.md](events/stampfly_workshop/workshop_guide.md) | Workshop guide |
| [events/stampfly_workshop/workshop_schedule.md](events/stampfly_workshop/workshop_schedule.md) | Workshop schedule |
| [events/stampfly_workshop/competition_rules.md](events/stampfly_workshop/competition_rules.md) | Competition rules |
| [events/_shared/README.md](events/_shared/README.md) | Slide materials guide |

### Development Plans (`plans/`)

| File | Description |
|------|-------------|
| [plans/education-outreach-strategy.md](plans/education-outreach-strategy.md) | Education outreach strategy (per-tier rollout, 3 horizons) |
| [plans/ecosystem-migration.md](plans/ecosystem-migration.md) | Ecosystem migration plan |
| [plans/installer-architecture.md](plans/installer-architecture.md) | Installer architecture |
| [plans/archive/controller-menu-usb.md](plans/archive/controller-menu-usb.md) | Controller menu & USB plan (completed) |
| [plans/hardware-test.md](plans/hardware-test.md) | Hardware test plan |
| [plans/archive/hils-firmware.md](plans/archive/hils-firmware.md) | HILS firmware plan (not implemented) |
| [plans/ros2-integration.md](plans/ros2-integration.md) | ROS2 integration plan |
| [plans/archive/serial-cli-rebuild.md](plans/archive/serial-cli-rebuild.md) | Serial CLI rebuild plan (completed) |
| [plans/archive/tello-compat.md](plans/archive/tello-compat.md) | Tello compatibility plan |
| [plans/archive/wifi-comm.md](plans/archive/wifi-comm.md) | WiFi communication plan |
| [plans/archive/wifi-command-implementation.md](plans/archive/wifi-command-implementation.md) | WiFi command implementation plan |

## 3. Firmware Documentation

### Vehicle

| File | Description |
|------|-------------|
| [firmware/vehicle/README.md](../firmware/vehicle/README.md) | Vehicle firmware complete guide |
| [plans/vehicle-firmware.md](plans/vehicle-firmware.md) | Design policy |
| [plans/archive/vehicle-phase3-test.md](plans/archive/vehicle-phase3-test.md) | Phase3 test plan |
| [plans/archive/vehicle-phase3-test-guide.md](plans/archive/vehicle-phase3-test-guide.md) | Phase3 test guide |

### Vehicle Design Documents (`firmware/vehicle/docs/`)

Full design document set for the current firmware. The six canonical documents:

| File | Description |
|------|-------------|
| [firmware/vehicle/docs/requirements.md](../firmware/vehicle/docs/requirements.md) | Requirements definition |
| [firmware/vehicle/docs/architecture.md](../firmware/vehicle/docs/architecture.md) | Architecture design (4-layer access, cross-cutting rules R1-R16, BSP layer) |
| [firmware/vehicle/docs/detailed_design.md](../firmware/vehicle/docs/detailed_design.md) | Detailed design |
| [firmware/vehicle/docs/coding_and_education.md](../firmware/vehicle/docs/coding_and_education.md) | Coding policy and education plan |
| [firmware/vehicle/docs/development_roadmap.md](../firmware/vehicle/docs/development_roadmap.md) | Development roadmap and SILS-to-real-hardware workflow |
| [firmware/vehicle/docs/hardware_init.md](../firmware/vehicle/docs/hardware_init.md) | Hardware initialization design (BSP, boot sequence) |

Beyond these six, `firmware/vehicle/docs/` also holds flight-investigation and debugging notes (`poshold_journey.md`, `feature_status.md`, `operation_manual.md`, `topic_reference.md`, `control_theory_overview.md`, `yaw_axis_model.md`, etc. — about 28 files total).

### Vehicle Component Documentation

| File | Description |
|------|-------------|
| [firmware/vehicle/components/sf_hal_bmi270/README.md](../firmware/vehicle/components/sf_hal_bmi270/README.md) | BMI270 IMU driver |
| [firmware/vehicle/components/sf_hal_bmi270/docs/API.md](../firmware/vehicle/components/sf_hal_bmi270/docs/API.md) | BMI270 API reference |
| [firmware/vehicle/components/sf_hal_pmw3901/README.md](../firmware/vehicle/components/sf_hal_pmw3901/README.md) | PMW3901 optical flow driver |
| [firmware/vehicle/components/sf_hal_vl53l3cx/README.md](../firmware/vehicle/components/sf_hal_vl53l3cx/README.md) | VL53L3CX ToF driver |
| [firmware/vehicle/components/sf_hal_vl53l3cx/docs/API.md](../firmware/vehicle/components/sf_hal_vl53l3cx/docs/API.md) | VL53L3CX API reference |

### Vehicle Examples

| Directory | Description |
|-----------|-------------|
| [firmware/vehicle/components/sf_hal_bmi270/examples/](../firmware/vehicle/components/sf_hal_bmi270/examples/) | BMI270 examples (polling, interrupt, FIFO, dev stages) |
| [firmware/vehicle/components/sf_hal_vl53l3cx/examples/](../firmware/vehicle/components/sf_hal_vl53l3cx/examples/) | VL53L3CX examples (polling, interrupt, dev stages) |

### Vehicle Old (Legacy Vehicle Firmware)

After POS_HOLD position control was validated on real hardware, `firmware/vehicle_new` was promoted to `firmware/vehicle`, and the previous-generation firmware was frozen as `firmware/vehicle_old` (the old `sf_hal_*`/`sf_algo_*`/`sf_svc_*` layered naming convention). It has flown 87 real flights but receives no new development.

| File | Description |
|------|-------------|
| [firmware/vehicle_old/README.md](../firmware/vehicle_old/README.md) | Legacy vehicle firmware complete guide |
| [firmware/vehicle_old/components/sf_algo_fusion/README.md](../firmware/vehicle_old/components/sf_algo_fusion/README.md) | Sensor fusion algorithm |

### Controller

| File | Description |
|------|-------------|
| [firmware/controller/README.md](../firmware/controller/README.md) | Controller firmware complete guide |
| [architecture/tdma-usage.md](architecture/tdma-usage.md) | TDMA communication detailed guide |
| [plans/archive/controller-espidf-migration.md](plans/archive/controller-espidf-migration.md) | ESP-IDF migration plan |

### Common (Shared Code)

| File | Description |
|------|-------------|
| [firmware/common/README.md](../firmware/common/README.md) | Shared code (`protocol/` has a working ESP-NOW protocol implementation used by vehicle/vehicle_old/controller; `math/` and `utils/` remain empty placeholders) |

### Workshop (Educational Firmware)

| File | Description |
|------|-------------|
| [firmware/workshop/README.md](../firmware/workshop/README.md) | Workshop materials overview |
| Lessons 0-13 | Setup, Motor, Controller, LED, IMU, P Control, Modeling, SysID, PID, Estimation, API, Python SDK, Competition |

## 4. Other Directories

| File | Description |
|------|-------------|
| [protocol/README.md](../protocol/README.md) | Communication protocol spec (WIP) |
| [control/README.md](../control/README.md) | Control design assets (WIP) |
| [analysis/README.md](../analysis/README.md) | Experiment data analysis (WIP) |
| [tools/README.md](../tools/README.md) | Utility tools (WIP) |
| [simulator/README.md](../simulator/README.md) | Simulator overview |
| [simulator/genesis/README.md](../simulator/genesis/README.md) | Genesis simulator |
| [ros/README.md](../ros/README.md) | ROS integration (WIP) |
| [examples/README.md](../examples/README.md) | Learning examples |
