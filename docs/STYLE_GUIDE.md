# Documentation Style Guide

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

このドキュメントは、StampFly Ecosystem プロジェクトにおけるドキュメント作成のスタイルガイドです。
プロジェクト参加者は、このガイドに従ってドキュメントを作成してください。

### 見本ドキュメント

以下のドキュメントがスタイルの見本です：

- `docs/getting-started.md` - 入門ガイド
- `firmware/vehicle/README.md` - 機体ファームウェア
- `firmware/controller/README.md` - 送信機ファームウェア

---

## 2. バイリンガル構成

すべての主要ドキュメントは**日本語と英語の両方**で記述します。

### 構成ルール

1. **日本語を先に**記述する
2. 冒頭に英語版の存在を示す**注記**を入れる
3. 英語セクションは `---`（水平線）で区切る
4. 日本語と英語で**同じ章番号**を使う

### テンプレート

```markdown
# ドキュメントタイトル

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

（日本語の内容）

## 2. 詳細

（日本語の内容）

---

<a id="english"></a>

## 1. Overview

（English content）

## 2. Details

（English content）
```

---

## 3. 見出しフォーマット

### 見出しレベル

| レベル | 形式 | 用途 | 例 |
|--------|------|------|-----|
| H1 `#` | `# タイトル` | ドキュメントタイトル（1つだけ） | `# StampFly Vehicle Firmware` |
| H2 `##` | `## N. 章タイトル` | 章（番号付き） | `## 1. 概要` |
| H3 `###` | `### 節タイトル` | 節（番号なし） | `### このプロジェクトについて` |
| H4 `####` | `#### 項タイトル` | 項（番号なし） | `#### パラメータ一覧` |

### 重要なルール

- **章（H2）には番号を付ける**：`## 1.`, `## 2.`, `## 3.`, ...
- **節・項には番号を付けない**：`### 1.1` ではなく `###` のみ
- **日本語と英語で同じ番号体系を使う**

### 良い例

```markdown
## 1. 概要

### このプロジェクトについて

### 対象読者

## 2. 詳細

### 仕様

#### パラメータ一覧
```

### 悪い例

```markdown
# 第1章 概要          ← 「第N章」形式は使わない

## 1.1 このプロジェクト  ← 節に番号を付けない

# Chapter 2           ← 英語でも同じ形式を使う
```

---

## 4. テーブル（表）

構造化された情報は、リスト形式よりも**テーブル**を優先します。

### 良い例

```markdown
| 機能 | 説明 |
|------|------|
| IMU | BMI270（加速度・ジャイロ）400Hz |
| 気圧センサー | BMP280 50Hz |
| ToF | VL53L3CX 30Hz |
```

### 悪い例

```markdown
- IMU: BMI270（加速度・ジャイロ）400Hz
- 気圧センサー: BMP280 50Hz
- ToF: VL53L3CX 30Hz
```

---

## 5. コードブロック

コードブロックには**言語を明示**します。

### 対応言語

| 言語 | 指定子 |
|------|--------|
| C/C++ | `cpp` または `c` |
| Python | `python` |
| Bash/Shell | `bash` |
| Markdown | `markdown` |
| JSON | `json` |
| YAML | `yaml` |

### 例

````markdown
```cpp
// Initialize motor
// モータを初期化
motor_init();
```

```bash
idf.py build flash monitor
```
````

---

## 6. 図表（ダイアグラム）

複雑な構造は **ASCII アート**で表現します。

### モーター配置の例

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

### 状態遷移の例

```
INIT ─────────┐
              │ 初期化完了
              ▼
IDLE ◄────── ARMED
 │            │ ▲
 │ ARM要求    │ │ DISARM要求
 └──────────► │ │
              ▼ │
           FLYING
```

---

## 7. 外部リンク

外部リンクは**テーブル形式**で整理します。

### 例

```markdown
| リポジトリ | 説明 |
|-----------|------|
| [StampFly技術仕様](https://github.com/M5Fly-kanazawa/StampFly_technical_specification) | ハードウェア仕様書 |
| [IMUドライバ](https://github.com/kouhei1970/stampfly_imu) | BMI270 ドライバ |
```

---

## 8. コメント規約（コード内）

ソースコード内のコメントは**英語と日本語を併記**します。

```cpp
// Initialize the motor driver
// モータドライバを初期化
motor_init();

// Set PWM frequency to 20kHz
// PWM周波数を20kHzに設定
pwm_set_frequency(20000);
```

---

---

<a id="english"></a>

## 1. Overview

This document is the style guide for documentation in the StampFly Ecosystem project.
All project participants should follow this guide when creating documentation.

### Reference Documents

The following documents serve as style examples:

- `docs/getting-started.md` - Getting started guide
- `firmware/vehicle/README.md` - Vehicle firmware
- `firmware/controller/README.md` - Controller firmware

---

## 2. Bilingual Structure

All major documents must be written in **both Japanese and English**.

### Structure Rules

1. Write **Japanese first**
2. Add a **note** at the beginning indicating the English version exists
3. Separate English section with `---` (horizontal rule)
4. Use the **same chapter numbers** for both languages

### Template

```markdown
# Document Title

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

(Japanese content)

## 2. 詳細

(Japanese content)

---

<a id="english"></a>

## 1. Overview

(English content)

## 2. Details

(English content)
```

---

## 3. Heading Format

### Heading Levels

| Level | Format | Usage | Example |
|-------|--------|-------|---------|
| H1 `#` | `# Title` | Document title (only one) | `# StampFly Vehicle Firmware` |
| H2 `##` | `## N. Chapter Title` | Chapters (numbered) | `## 1. Overview` |
| H3 `###` | `### Section Title` | Sections (no number) | `### About This Project` |
| H4 `####` | `#### Subsection Title` | Subsections (no number) | `#### Parameter List` |

### Important Rules

- **Chapters (H2) must be numbered**: `## 1.`, `## 2.`, `## 3.`, ...
- **Sections/subsections are NOT numbered**: Use `###` only, not `### 1.1`
- **Use the same numbering scheme for both languages**

### Good Example

```markdown
## 1. Overview

### About This Project

### Target Audience

## 2. Details

### Specifications

#### Parameter List
```

### Bad Example

```markdown
# Chapter 1: Overview     ← Don't use "Chapter N:" format

## 1.1 About This Project ← Don't number sections

# 第2章                   ← Use same format for Japanese
```

---

## 4. Tables

Prefer **tables** over lists for structured information.

### Good Example

```markdown
| Feature | Description |
|---------|-------------|
| IMU | BMI270 (accel/gyro) 400Hz |
| Barometer | BMP280 50Hz |
| ToF | VL53L3CX 30Hz |
```

### Bad Example

```markdown
- IMU: BMI270 (accel/gyro) 400Hz
- Barometer: BMP280 50Hz
- ToF: VL53L3CX 30Hz
```

---

## 5. Code Blocks

Always **specify the language** for code blocks.

### Supported Languages

| Language | Specifier |
|----------|-----------|
| C/C++ | `cpp` or `c` |
| Python | `python` |
| Bash/Shell | `bash` |
| Markdown | `markdown` |
| JSON | `json` |
| YAML | `yaml` |

### Example

````markdown
```cpp
// Initialize motor
// モータを初期化
motor_init();
```

```bash
idf.py build flash monitor
```
````

---

## 6. Diagrams

Use **ASCII art** for complex structures.

### Motor Layout Example

```
               Front
          FL (M4)   FR (M1)
             ╲   ▲   ╱
              ╲  │  ╱
               ╲ │ ╱
                ╲│╱
                 ╳         ← Center
                ╱│╲
               ╱ │ ╲
              ╱  │  ╲
             ╱   │   ╲
          RL (M3)    RR (M2)
                Rear
```

### State Transition Example

```
INIT ─────────┐
              │ Init complete
              ▼
IDLE ◄────── ARMED
 │            │ ▲
 │ ARM req    │ │ DISARM req
 └──────────► │ │
              ▼ │
           FLYING
```

---

## 7. External Links

Organize external links in **table format**.

### Example

```markdown
| Repository | Description |
|-----------|-------------|
| [StampFly Technical Spec](https://github.com/M5Fly-kanazawa/StampFly_technical_specification) | Hardware specifications |
| [IMU Driver](https://github.com/kouhei1970/stampfly_imu) | BMI270 driver |
```

---

## 8. Code Comments

Comments in source code should be written in **both English and Japanese**.

```cpp
// Initialize the motor driver
// モータドライバを初期化
motor_init();

// Set PWM frequency to 20kHz
// PWM周波数を20kHzに設定
pwm_set_frequency(20000);
```
