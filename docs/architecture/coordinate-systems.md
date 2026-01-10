# Coordinate Systems / 座標系

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

StampFlyエコシステムで使用する座標系の定義と、各コンポーネント間の座標変換について説明します。

### 対象読者

- シミュレータ開発者
- ファームウェア開発者
- 可視化ツール開発者

## 2. 座標系の定義

### 計算用座標系: NED (North-East-Down)

制御計算・シミュレーションで使用する座標系です。

| 軸 | 正方向 | 説明 |
|----|--------|------|
| X | 北 / 前方 | 機体の前進方向 |
| Y | 東 / 右 | 機体の右方向 |
| Z | 下 | 重力方向（地面向き） |

```
        X (前/北)
        ▲
        │
        │
        ●───────▶ Y (右/東)
       ╱
      ╱
     ▼
    Z (下)
```

**使用箇所:**
- ファームウェア（姿勢制御、位置制御）
- シミュレーション（物理演算）
- 制御設計（モデル、伝達関数）

### 表示用座標系: WebGL/Three.js (Y-up)

3D可視化で使用する座標系です。

| 軸 | 正方向 | 説明 |
|----|--------|------|
| X | 右 | 画面右方向 |
| Y | 上 | 画面上方向 |
| Z | 手前 | 画面手前方向 |

```
        Y (上)
        ▲
        │
        │
        ●───────▶ X (右)
       ╱
      ╱
     ▼
    Z (手前)
```

**使用箇所:**
- STLファイル（3Dメッシュデータ）
- WebGLビュワー
- Three.jsベースの可視化ツール

## 3. 座標変換

### NED → WebGL 変換

シミュレーション結果を可視化する際の座標変換です。

| WebGL | = | NED | 説明 |
|-------|---|-----|------|
| x | = | +y | 東/右 → 右 |
| y | = | -z | 下 → 上（反転） |
| z | = | +x | 北/前 → 手前 |

```javascript
// NED座標からWebGL座標への変換
// Convert from NED to WebGL coordinates
function nedToWebGL(ned) {
    return {
        x:  ned.y,   // East  → Right
        y: -ned.z,   // Down  → Up (inverted)
        z:  ned.x    // North → Forward
    };
}
```

### WebGL → NED 変換

3Dモデルの位置をシミュレーションに取り込む際の座標変換です。

| NED | = | WebGL | 説明 |
|-----|---|-------|------|
| x | = | +z | 手前 → 北/前 |
| y | = | +x | 右 → 東/右 |
| z | = | -y | 上 → 下（反転） |

```javascript
// WebGL座標からNED座標への変換
// Convert from WebGL to NED coordinates
function webglToNED(webgl) {
    return {
        x:  webgl.z,   // Forward → North
        y:  webgl.x,   // Right   → East
        z: -webgl.y    // Up      → Down (inverted)
    };
}
```

## 4. 回転の座標変換

### 回転軸の対応

| NED回転軸 | WebGL回転軸 | 備考 |
|-----------|-------------|------|
| X (Roll) | Z | 同じ向き |
| Y (Pitch) | X | 同じ向き |
| Z (Yaw) | -Y | **向き反転** |

### なぜYawだけ符号反転？

```
NED:   +Yaw = 右旋回（上から見て時計回り）
       Z軸が下向き、右手法則で時計回り

WebGL: +Y回転 = 左旋回（上から見て反時計回り）
       Y軸が上向き、右手法則で反時計回り

→ 同じ「右旋回」を表すには符号を反転
```

### NED回転 → WebGL回転 変換

```javascript
// NED姿勢からWebGL回転への変換
// Convert from NED attitude to WebGL rotation
function nedRotationToWebGL(roll, pitch, yaw) {
    return {
        x: pitch,    // Pitch → WebGL X rotation
        y: -yaw,     // Yaw → WebGL Y rotation (sign inverted)
        z: roll      // Roll → WebGL Z rotation
    };
}
```

### Three.jsでの実装

```javascript
// 回転順序の設定（NED ZYX → WebGL YXZ）
// Set rotation order (NED ZYX → WebGL YXZ)
mesh.rotation.order = 'YXZ';

// NED姿勢を適用
// Apply NED attitude to mesh
function applyNEDAttitude(mesh, roll, pitch, yaw) {
    mesh.rotation.set(
        pitch,    // X: Pitch
        -yaw,     // Y: Yaw (inverted)
        roll      // Z: Roll
    );
}
```

### 検証例

| NED姿勢 | WebGL回転 | 見た目 |
|---------|-----------|--------|
| Roll = +30° | rotation.z = +30° | 右翼下げ |
| Pitch = +30° | rotation.x = +30° | 機首上げ |
| Yaw = +30° | rotation.y = -30° | 右旋回 |

## 5. 回転の表現（NED座標系）

### オイラー角

| 角度 | 軸 | 正方向 | 説明 |
|------|-----|--------|------|
| Roll (φ) | X軸 | 右翼下げ | 横揺れ |
| Pitch (θ) | Y軸 | 機首上げ | 縦揺れ |
| Yaw (ψ) | Z軸 | 右旋回 | 偏揺れ |

### 回転順序

ZYX順（Yaw → Pitch → Roll）を使用します。

```
R = Rz(ψ) × Ry(θ) × Rx(φ)
```

## 6. 設計方針

### アセットとロジックの分離

```
┌─────────────────────┐     ┌─────────────────────┐
│     Simulation      │     │    Visualization    │
│    (NED座標系)      │────▶│   (WebGL座標系)     │
│                     │ 変換 │                     │
│  - 姿勢制御         │     │  - Three.js描画     │
│  - 位置制御         │     │  - WebGLレンダリング │
│  - 物理演算         │     │                     │
└─────────────────────┘     └─────────────────────┘
                                    ▲
                                    │ 変換不要
                              ┌─────┴─────┐
                              │ STL Files │
                              │(WebGL座標)│
                              └───────────┘
```

### 理由

| 方針 | 理由 |
|------|------|
| STLはWebGL座標で保存 | 表示時に変換不要、3Dツールとの互換性 |
| 計算はNEDで実行 | 航空工学の標準、制御理論との整合性 |
| 変換は可視化レイヤで | 単一の変換ポイント、保守性向上 |

## 7. 関連ファイル

| ファイル | 説明 |
|----------|------|
| `simulator/sandbox/coord_transformer/` | 座標変換ツール |
| `simulator/sandbox/webgl_viewer/` | STLビュワー |
| `simulator/assets/meshes/parts/` | STLファイル（WebGL座標） |

---

<a id="english"></a>

## 1. Overview

### About This Document

This document defines the coordinate systems used in the StampFly ecosystem and explains coordinate transformations between components.

### Target Audience

- Simulator developers
- Firmware developers
- Visualization tool developers

## 2. Coordinate System Definitions

### Computation Coordinate System: NED (North-East-Down)

Used for control calculations and simulation.

| Axis | Positive Direction | Description |
|------|-------------------|-------------|
| X | North / Forward | Aircraft forward direction |
| Y | East / Right | Aircraft right direction |
| Z | Down | Gravity direction (toward ground) |

```
        X (Forward/North)
        ▲
        │
        │
        ●───────▶ Y (Right/East)
       ╱
      ╱
     ▼
    Z (Down)
```

**Used in:**
- Firmware (attitude control, position control)
- Simulation (physics computation)
- Control design (models, transfer functions)

### Display Coordinate System: WebGL/Three.js (Y-up)

Used for 3D visualization.

| Axis | Positive Direction | Description |
|------|-------------------|-------------|
| X | Right | Screen right |
| Y | Up | Screen up |
| Z | Toward viewer | Screen forward |

```
        Y (Up)
        ▲
        │
        │
        ●───────▶ X (Right)
       ╱
      ╱
     ▼
    Z (Toward viewer)
```

**Used in:**
- STL files (3D mesh data)
- WebGL viewer
- Three.js-based visualization tools

## 3. Coordinate Transformations

### NED → WebGL Transformation

Coordinate transformation for visualizing simulation results.

| WebGL | = | NED | Description |
|-------|---|-----|-------------|
| x | = | +y | East/Right → Right |
| y | = | -z | Down → Up (inverted) |
| z | = | +x | North/Forward → Forward |

```javascript
// Convert from NED to WebGL coordinates
function nedToWebGL(ned) {
    return {
        x:  ned.y,   // East  → Right
        y: -ned.z,   // Down  → Up (inverted)
        z:  ned.x    // North → Forward
    };
}
```

### WebGL → NED Transformation

Coordinate transformation for importing 3D model positions into simulation.

| NED | = | WebGL | Description |
|-----|---|-------|-------------|
| x | = | +z | Forward → North/Forward |
| y | = | +x | Right → East/Right |
| z | = | -y | Up → Down (inverted) |

```javascript
// Convert from WebGL to NED coordinates
function webglToNED(webgl) {
    return {
        x:  webgl.z,   // Forward → North
        y:  webgl.x,   // Right   → East
        z: -webgl.y    // Up      → Down (inverted)
    };
}
```

## 4. Rotation Coordinate Transformation

### Rotation Axis Mapping

| NED Rotation Axis | WebGL Rotation Axis | Note |
|-------------------|---------------------|------|
| X (Roll) | Z | Same direction |
| Y (Pitch) | X | Same direction |
| Z (Yaw) | -Y | **Direction inverted** |

### Why is Yaw Sign Inverted?

```
NED:   +Yaw = Turn right (clockwise from above)
       Z-axis points down, right-hand rule gives clockwise

WebGL: +Y rotation = Turn left (counterclockwise from above)
       Y-axis points up, right-hand rule gives counterclockwise

→ To represent the same "turn right", sign must be inverted
```

### NED Rotation → WebGL Rotation Transformation

```javascript
// Convert from NED attitude to WebGL rotation
function nedRotationToWebGL(roll, pitch, yaw) {
    return {
        x: pitch,    // Pitch → WebGL X rotation
        y: -yaw,     // Yaw → WebGL Y rotation (sign inverted)
        z: roll      // Roll → WebGL Z rotation
    };
}
```

### Three.js Implementation

```javascript
// Set rotation order (NED ZYX → WebGL YXZ)
mesh.rotation.order = 'YXZ';

// Apply NED attitude to mesh
function applyNEDAttitude(mesh, roll, pitch, yaw) {
    mesh.rotation.set(
        pitch,    // X: Pitch
        -yaw,     // Y: Yaw (inverted)
        roll      // Z: Roll
    );
}
```

### Verification Examples

| NED Attitude | WebGL Rotation | Appearance |
|--------------|----------------|------------|
| Roll = +30° | rotation.z = +30° | Right wing down |
| Pitch = +30° | rotation.x = +30° | Nose up |
| Yaw = +30° | rotation.y = -30° | Turn right |

## 5. Rotation Representation (NED Coordinate System)

### Euler Angles

| Angle | Axis | Positive Direction | Description |
|-------|------|-------------------|-------------|
| Roll (φ) | X-axis | Right wing down | Bank angle |
| Pitch (θ) | Y-axis | Nose up | Elevation angle |
| Yaw (ψ) | Z-axis | Turn right | Heading angle |

### Rotation Order

ZYX order (Yaw → Pitch → Roll) is used.

```
R = Rz(ψ) × Ry(θ) × Rx(φ)
```

## 6. Design Principles

### Separation of Assets and Logic

```
┌─────────────────────┐     ┌─────────────────────┐
│     Simulation      │     │    Visualization    │
│  (NED coordinates)  │────▶│ (WebGL coordinates) │
│                     │transform│                  │
│  - Attitude control │     │  - Three.js render  │
│  - Position control │     │  - WebGL rendering  │
│  - Physics engine   │     │                     │
└─────────────────────┘     └─────────────────────┘
                                    ▲
                                    │ No transform needed
                              ┌─────┴─────┐
                              │ STL Files │
                              │ (WebGL)   │
                              └───────────┘
```

### Rationale

| Principle | Reason |
|-----------|--------|
| Store STL in WebGL coordinates | No transform for display, compatibility with 3D tools |
| Compute in NED | Aerospace standard, consistency with control theory |
| Transform in visualization layer | Single transformation point, improved maintainability |

## 7. Related Files

| File | Description |
|------|-------------|
| `simulator/sandbox/coord_transformer/` | Coordinate transformation tool |
| `simulator/sandbox/webgl_viewer/` | STL viewer |
| `simulator/assets/meshes/parts/` | STL files (WebGL coordinates) |
