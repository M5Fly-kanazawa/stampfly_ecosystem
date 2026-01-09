# StampFly Loop Shaping Design Tool

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

StampFlyの角速度制御系をループ整形法で設計するためのWebベースインタラクティブツール。

### 特徴

| 機能 | 説明 |
|------|------|
| リアルタイム更新 | パラメータ変更即座にボード線図・ステップ応答が更新 |
| サーバー不要 | ブラウザのみで動作（HTMLファイルを開くだけ） |
| 設計指標表示 | 位相余裕・ゲイン余裕・ゲイン交差周波数を自動計算 |

## 2. 使い方

### 起動方法

`index.html` をWebブラウザで開く：

```bash
# macOS
open index.html

# Linux
xdg-open index.html

# Windows
start index.html
```

### パラメータ設定

#### プラントパラメータ

| パラメータ | 記号 | 説明 | StampFly標準値 |
|-----------|------|------|----------------|
| 慣性モーメント | I | 回転軸周りの慣性モーメント | 9.16×10⁻⁶ kg·m²（Roll軸） |
| モータ時定数 | τₘ | モータの1次遅れ時定数 | 20 ms |

#### PIDパラメータ

| パラメータ | 記号 | 説明 |
|-----------|------|------|
| 比例ゲイン | Kp | 全体のゲイン調整 |
| 積分時間 | Ti | 積分動作の時定数 |
| 微分時間 | Td | 微分動作の時定数 |
| 微分フィルタ係数 | η | 微分の高周波ロールオフ（典型値: 0.1） |

### 設計目標

| 指標 | 推奨値 | 理由 |
|------|--------|------|
| 位相余裕 PM | 60° | 良好な安定性と応答速度のバランス |
| 帯域幅比 ω_gc/ω_m | 30%以下 | モータ遅れによる位相低下を考慮 |

## 3. 表示内容

### ボード線図

| 線 | 色 | 説明 |
|----|-----|------|
| 開ループ L(s) | 青（実線） | C(s)×G(s)、設計対象 |
| プラント G(s) | 緑（破線） | 制御対象 |
| 制御器 C(s) | 赤（一点鎖線） | PID制御器 |

### 設計結果

| 項目 | 説明 | 警告条件 |
|------|------|----------|
| ゲイン交差周波数 ω_gc | 開ループゲインが0dBとなる周波数 | - |
| 位相余裕 PM | 安定余裕の指標 | < 45°で黄、< 30°で赤 |
| ゲイン余裕 GM | 安定余裕の指標 | < 6dBで黄 |
| 帯域幅比 | ω_gc/ω_m | > 50%で黄 |

## 4. 技術詳細

### プラントモデル

$$
G(s) = \frac{1/I}{s(\tau_m s + 1)}
$$

### PID制御器（不完全微分付き）

$$
C(s) = K_p \left( 1 + \frac{1}{T_i s} + \frac{T_d s}{\eta T_d s + 1} \right)
$$

### 使用ライブラリ

| ライブラリ | バージョン | 用途 |
|-----------|-----------|------|
| Plotly.js | 2.27.0 | インタラクティブグラフ描画 |

---

<a id="english"></a>

## 1. Overview

Web-based interactive tool for designing StampFly angular velocity control system using loop shaping method.

### Features

| Feature | Description |
|---------|-------------|
| Real-time update | Bode plot and step response update immediately on parameter change |
| No server required | Runs in browser only (just open HTML file) |
| Design metrics | Automatically calculates phase margin, gain margin, crossover frequency |

## 2. Usage

### Launch

Open `index.html` in a web browser:

```bash
# macOS
open index.html

# Linux
xdg-open index.html

# Windows
start index.html
```

### Parameter Settings

#### Plant Parameters

| Parameter | Symbol | Description | StampFly Default |
|-----------|--------|-------------|------------------|
| Moment of inertia | I | Inertia about rotation axis | 9.16×10⁻⁶ kg·m² (Roll) |
| Motor time constant | τₘ | Motor first-order lag time constant | 20 ms |

#### PID Parameters

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Proportional gain | Kp | Overall gain adjustment |
| Integral time | Ti | Integral action time constant |
| Derivative time | Td | Derivative action time constant |
| Derivative filter coefficient | η | High-frequency rolloff (typical: 0.1) |

### Design Targets

| Metric | Recommended | Reason |
|--------|-------------|--------|
| Phase margin PM | 60° | Good balance of stability and response |
| Bandwidth ratio ω_gc/ω_m | ≤30% | Account for motor lag phase reduction |

## 3. Display

### Bode Plot

| Line | Color | Description |
|------|-------|-------------|
| Open-loop L(s) | Blue (solid) | C(s)×G(s), design target |
| Plant G(s) | Green (dashed) | Controlled system |
| Controller C(s) | Red (dash-dot) | PID controller |

### Design Results

| Item | Description | Warning Condition |
|------|-------------|-------------------|
| Gain crossover freq ω_gc | Frequency where open-loop gain = 0dB | - |
| Phase margin PM | Stability margin indicator | Yellow if < 45°, Red if < 30° |
| Gain margin GM | Stability margin indicator | Yellow if < 6dB |
| Bandwidth ratio | ω_gc/ω_m | Yellow if > 50% |

## 4. Technical Details

### Plant Model

$$
G(s) = \frac{1/I}{s(\tau_m s + 1)}
$$

### PID Controller (with derivative filter)

$$
C(s) = K_p \left( 1 + \frac{1}{T_i s} + \frac{T_d s}{\eta T_d s + 1} \right)
$$

### Libraries Used

| Library | Version | Purpose |
|---------|---------|---------|
| Plotly.js | 2.27.0 | Interactive plotting |
