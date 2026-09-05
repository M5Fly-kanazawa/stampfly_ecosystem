# vehicle — Next-Generation StampFly Vehicle Firmware

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このプロジェクトについて

vehicle は、StampFly 教育プラットフォームのための次世代機体ファームウェアです。設計は **関心の分離・可読性・教育的価値** を最優先に据え、Workshop 受講者から研究者・ファーム実装者まで、それぞれが自分のテーマに集中できるよう **4 階層アクセス（L0〜L3）** を提供します。

### 対象読者

| 層 | 入口 | 典型ユーザー |
|----|-----|------------|
| **L0: Workshop API** | `ws::*` | Workshop 受講者・初学者 — `setup()` / `loop_400Hz(dt)` で完結 |
| **L1: Topic API** | `sf::api::*` | 推定・制御・ガイダンス学習者 — 自分の ESKF / PID / Navigator を実装 |
| **L2: HAL Direct** | `stampfly::*Wrapper` | HW 学習者 — SPI / I2C / RMT / LEDC を理解 |
| **L3: BSP Internal** | `sf::internal::board` | ファーム実装者・拡張者 — 起動順序や HW 資源管理を変更 |

各層は **並列に共存し**、学習者は自分のレベルに応じて入口を選びます。

### 設計文書

実装・修正の前に、以下を必ず読んでください（`docs/` 配下）。

| 文書 | 内容 |
|------|-----|
| [`docs/requirements.md`](docs/requirements.md) | 要件定義書 |
| [`docs/architecture.md`](docs/architecture.md) | アーキテクチャ設計（v3: 4階層 + R1〜R16） |
| [`docs/detailed_design.md`](docs/detailed_design.md) | Topic / インターフェース / 状態遷移 |
| [`docs/coding_and_education.md`](docs/coding_and_education.md) | コーディング規約・教育計画 |
| [`docs/development_roadmap.md`](docs/development_roadmap.md) | 開発ロードマップ・SILS→実機ワークフロー |
| [`docs/hardware_init.md`](docs/hardware_init.md) | BSP・HW 初期化設計 |

---

## 2. ライセンス

### 本プロジェクトのライセンス

vehicle ファームウェア本体（StampFly Ecosystem の一部として開発された原コード）は **MIT License** で配布されます。プロジェクトルートの [`LICENSE`](../../LICENSE) を参照してください。

```
SPDX-License-Identifier: MIT
Copyright (c) 2026 Kouhei Ito
```

全 .cpp / .hpp ファイルの先頭にも上記 SPDX ヘッダを付与しています。

### 第三者コンポーネントのライセンス

vehicle は複数の第三者ドライバ・フレームワークを利用しています。詳細は [`NOTICE.md`](NOTICE.md) を参照してください。

| 主要な第三者要素 | ライセンス | 備考 |
|-----------------|-----------|-----|
| Bosch Sensortec BMI270/BMM150/BMP280 C driver | BSD-3-Clause | センサ HAL の core |
| STMicroelectronics VL53L3CX driver | GPL-2.0+ OR BSD-3-Clause（**BSD-3-Clause を選択**） | ToF センサ |
| Espressif ESP-IDF + led_strip managed component | Apache 2.0 | フレームワーク |

→ vehicle バイナリ全体は **MIT + BSD-3-Clause + Apache 2.0** の互換ライセンスのみで構成され、GPL コードを含みません。

---

## 3. Credits / Influences

### 起点ファームウェア

vehicle は **M5Stack 社公開の M5StampFly 公式ファームウェア** を起点として、関心の分離・教育性向上・SILS 検証の観点から再設計したものです。

- M5StampFly 公式: https://github.com/m5stack/M5StampFly
- ライセンス: MIT
- Copyright: Kouhei Ito

本プロジェクトの著作者と M5StampFly 公式ファームウェアの主たる著作者は同一（Kouhei Ito）です。

### アーキテクチャ思想の借用元

本プロジェクトは以下の業界標準フレームワークから **アイデア・思想** を借用していますが、コードの直接借用はしていません（idea/expression dichotomy）。

| 借用元 | 借用した思想 | 本プロジェクトでの実装 |
|-------|-----------|------------------|
| **PX4 Autopilot** ([px4.io](https://px4.io/), BSD-3-Clause) | uORB Pub-Sub の思想 | 同一 MCU 内特化のテンプレート + 3 ポリシー (Latest / RingBuffer / Queue) として独自実装 |
| **Zephyr RTOS** ([zephyrproject.org](https://www.zephyrproject.org/), Apache 2.0) | parent-child バス所有モデル、init level による順序明示 | `sf_board` BSP として ESP-IDF ネイティブ機構で実装（devicetree なし） |
| **Crazyflie firmware** ([bitcraze.io](https://www.bitcraze.io/), GPLv3) | 教育プラットフォームとしての枠組み | 同じ「ナノドローン教育」ニッチで、ESP32 ベースの代替として位置づけ |
| **Arduino** ([arduino.cc](https://www.arduino.cc/), LGPL/GPL) | `setup()` / `loop()` の簡潔さ | L0 Workshop API（`ws::*`）として再現、HW 詳細を完全に隠蔽 |

### 学術的位置づけ

本プロジェクトは室内ナノドローン教育プラットフォームのオープンソース系譜に連なります。引用する先行研究：

- Giernacki et al., "Crazyflie 2.0 quadrotor as a platform for research and education in robotics and control engineering" (2017)
- Preiss et al., "Crazyswarm: A large nano-quadcopter swarm" (IROS 2017)
- Silano et al., "CrazyS: a Software-In-The-Loop platform for the Crazyflie 2.0 nano-quadcopter" (MED 2018)

vehicle の独自貢献：
1. **4 階層アクセス（L0〜L3）** — 単一ファームで Workshop 受講者から FC 実装者まで階段的に降りられる構造
2. **`@design` タグ + 判定ステータス [OK]/[NG]/[--]** — 設計→実装のトレーサビリティを inline で保持
3. **横断ルール R1〜R16** — 教育性 + 業界標準 + スパゲッティ対策の組合せ
4. **ACRO 起点の Layer-by-Layer Identification** — 段階的プラント同定戦略

---

## 4. ビルドと実行

```bash
# 環境セットアップ (ESP-IDF + sf CLI)
source setup_env.sh

# vehicle をビルド
cd firmware/vehicle
idf.py build

# 書き込み + シリアルモニタ
idf.py flash monitor
```

詳細は [`docs/development_roadmap.md`](docs/development_roadmap.md) を参照。

---

## 5. 開発状況

現在の開発フェーズ・実装状況は [`docs/implementation_log.md`](docs/implementation_log.md) で時系列に記録されています。

進行中: Phase 2 (HAL 接続) — IMU / Motor / ESP-NOW / UDP 完了、ToF / Baro / Mag / Power / NVS / Logger 残

---

<a id="english"></a>

## 1. About This Project

vehicle is a next-generation flight controller firmware for the StampFly educational platform. It prioritizes **separation of concerns, readability, and educational value**, providing a **4-tier access model (L0–L3)** so that Workshop students, control-engineering learners, hardware-engineering students, and firmware implementers can each focus on their own theme.

### Target Audiences

| Tier | Entry Point | Typical User |
|------|------------|------------|
| **L0: Workshop API** | `ws::*` | Workshop students — write `setup()` / `loop_400Hz(dt)` |
| **L1: Topic API** | `sf::api::*` | Estimation / control / guidance learners — implement own ESKF / PID / Navigator |
| **L2: HAL Direct** | `stampfly::*Wrapper` | Hardware learners — understand SPI / I2C / RMT / LEDC |
| **L3: BSP Internal** | `sf::internal::board` | Firmware implementers — modify boot sequence or HW resource management |

The four tiers coexist; learners pick their entry point based on the level they want to engage at.

## 2. License

vehicle firmware (original code as part of StampFly Ecosystem) is distributed under the **MIT License**. See the project root [`LICENSE`](../../LICENSE).

```
SPDX-License-Identifier: MIT
Copyright (c) 2026 Kouhei Ito
```

For third-party components and their licenses, see [`NOTICE.md`](NOTICE.md). The combined binary contains only **MIT + BSD-3-Clause + Apache 2.0** compatible licenses (no GPL code).

## 3. Credits / Influences

vehicle is a redesign of the **M5StampFly official firmware** (https://github.com/m5stack/M5StampFly, MIT, Copyright (c) Kouhei Ito) — same author. Architectural ideas are borrowed from PX4 (uORB Pub-Sub), Zephyr RTOS (parent-child bus ownership, init levels), Crazyflie firmware (educational nano-drone niche), and Arduino (sketch-style L0 simplicity). No code is copied from these projects.

See the Japanese section for details on academic positioning and prior work.
