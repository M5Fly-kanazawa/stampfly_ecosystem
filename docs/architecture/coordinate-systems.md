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

### シミュレーション用座標系: Genesis (Z-up)

Genesis物理エンジンで使用する座標系です。ロボットシミュレーションの標準的なZ-up座標系です。

| 軸 | 正方向 | 説明 |
|----|--------|------|
| X | 水平 | 右方向（慣例） |
| Y | 水平 | 前方向（慣例） |
| Z | 上 | 重力の逆方向 |

```
        Z (上)
        ▲
        │
        │
        │
        ●───────▶ Y (前?)
       ╱
      ╱
     ▼
    X (右?)
```

**使用箇所:**
- Genesis物理シミュレーション
- 強化学習環境

**重力設定:** `gravity=(0, 0, -9.81)` （Z軸負方向）

### 表示用座標系: WebGL/Three.js (Y-up, Z-forward)

3D可視化で使用する座標系です。機体後方からのカメラ視点を基準とします。

| 軸 | 正方向 | 説明 |
|----|--------|------|
| X | 左 | 機体の左方向（右手系維持のため） |
| Y | 上 | 画面上方向 |
| Z | 前 | 機体の前進方向 |

```
        Y (上)
        ▲
        │
        │     Z (前)
        │    ╱
        │   ╱
        │  ╱
  X ◀───●
 (左)

※ 右手系: Y × Z = X （上 × 前 = 左）
```

**使用箇所:**
- STLファイル（3Dメッシュデータ）
- WebGLビュワー
- Three.jsベースの可視化ツール

## 3. 位置の座標変換

### NED → WebGL 変換

シミュレーション結果を可視化する際の座標変換です。右手系を維持します。

| WebGL | = | NED | 説明 |
|-------|---|-----|------|
| x | = | **-y** | 東/右 → 左（符号反転） |
| y | = | -z | 下 → 上（符号反転） |
| z | = | +x | 北/前 → 前 |

```javascript
// NED座標からWebGL座標への変換（右手系維持）
// Convert from NED to WebGL coordinates (preserves handedness)
function nedToWebGL(ned) {
    return {
        x: -ned.y,   // East/Right → Left (sign inverted)
        y: -ned.z,   // Down → Up (sign inverted)
        z:  ned.x    // North/Forward → Forward
    };
}
```

### WebGL → NED 変換

3Dモデルの位置をシミュレーションに取り込む際の座標変換です。

| NED | = | WebGL | 説明 |
|-----|---|-------|------|
| x | = | +z | 前 → 北/前 |
| y | = | **-x** | 左 → 東/右（符号反転） |
| z | = | -y | 上 → 下（符号反転） |

```javascript
// WebGL座標からNED座標への変換
// Convert from WebGL to NED coordinates
function webglToNED(webgl) {
    return {
        x:  webgl.z,   // Forward → North
        y: -webgl.x,   // Left → East/Right (sign inverted)
        z: -webgl.y    // Up → Down (sign inverted)
    };
}
```

### 変換行列

```
        | 0 -1  0 |
T_NED→WebGL = | 0  0 -1 |    det(T) = +1 (右手系維持)
        | 1  0  0 |
```

### NED → Genesis 変換

制御計算結果をGenesisシミュレーションに渡す際の座標変換です。

| Genesis | = | NED | 説明 |
|---------|---|-----|------|
| x | = | +y | 東/右 → X |
| y | = | +x | 北/前 → Y |
| z | = | -z | 下 → 上（符号反転） |

```python
# NED座標からGenesis座標への変換
# Convert from NED to Genesis coordinates
def ned_to_genesis(ned):
    return {
        'x': ned['y'],    # East/Right → X
        'y': ned['x'],    # North/Forward → Y
        'z': -ned['z']    # Down → Up (sign inverted)
    }
```

### Genesis → NED 変換

Genesisシミュレーション結果を制御計算に渡す際の座標変換です。

| NED | = | Genesis | 説明 |
|-----|---|---------|------|
| x | = | +y | Y → 北/前 |
| y | = | +x | X → 東/右 |
| z | = | -z | 上 → 下（符号反転） |

```python
# Genesis座標からNED座標への変換
# Convert from Genesis to NED coordinates
def genesis_to_ned(genesis):
    return {
        'x': genesis['y'],    # Y → North/Forward
        'y': genesis['x'],    # X → East/Right
        'z': -genesis['z']    # Up → Down (sign inverted)
    }
```

### WebGL（STL） → Genesis 変換

STLファイル（WebGL座標）をGenesisシミュレーションに読み込む際の座標変換です。

| Genesis | = | WebGL | 説明 |
|---------|---|-------|------|
| x | = | **-x** | 左 → 右（符号反転） |
| y | = | +z | 前 → 前 |
| z | = | +y | 上 → 上 |

```python
# WebGL（STL）座標からGenesis座標への変換
# Convert from WebGL (STL) to Genesis coordinates
def webgl_to_genesis(webgl):
    return {
        'x': -webgl['x'],   # Left → Right (sign inverted)
        'y':  webgl['z'],   # Forward → Forward
        'z':  webgl['y']    # Up → Up
    }
```

**変換行列:**
```
              | -1  0  0 |
T_WebGL→Genesis = |  0  0  1 |    det(T) = +1 (右手系維持)
              |  0  1  0 |
```

### Genesis → WebGL（STL） 変換

Genesisシミュレーション結果をWebGLで可視化する際の座標変換です。

| WebGL | = | Genesis | 説明 |
|-------|---|---------|------|
| x | = | **-x** | 右 → 左（符号反転） |
| y | = | +z | 上 → 上 |
| z | = | +y | 前 → 前 |

```python
# Genesis座標からWebGL座標への変換
# Convert from Genesis to WebGL coordinates
def genesis_to_webgl(genesis):
    return {
        'x': -genesis['x'],   # Right → Left (sign inverted)
        'y':  genesis['z'],   # Up → Up
        'z':  genesis['y']    # Forward → Forward
    }
```

### GenesisでのSTL読み込み例

```python
import genesis as gs

# STLファイルを読み込む際、Genesisが自動的に座標を解釈
# ただし、軸の対応を明示的に変換する場合：
mesh = scene.add_entity(
    gs.morphs.Mesh(
        file='frame.stl',
        scale=0.001,  # mm → m
        # 必要に応じて回転でWebGL→Genesis座標変換
        # euler=(90, 0, 0) などで調整
    ),
)
```

**注意:** Genesisは多くのSTL形式を自動認識しますが、軸の向きが期待と異なる場合は`euler`パラメータで回転調整が必要です。

## 4. 回転の座標変換

### 回転軸の対応

| NED回転軸 | WebGL回転軸 | 備考 |
|-----------|-------------|------|
| X (Roll) | Z | 同じ向き |
| Y (Pitch) | **-X** | **向き反転** |
| Z (Yaw) | **-Y** | **向き反転** |

### なぜPitchとYawが符号反転？

```
位置の軸対応から:
  NED Y (右) → WebGL -X (左)  → Pitch軸が反転
  NED Z (下) → WebGL -Y (上方向の負) → Yaw軸が反転

NED:   +Pitch = 機首上げ（Y軸右向き、右手法則）
WebGL: +X回転 = 機首下げ（X軸左向き、右手法則）
→ 同じ「機首上げ」には符号反転が必要

NED:   +Yaw = 右旋回（Z軸下向き、右手法則）
WebGL: +Y回転 = 左旋回（Y軸上向き、右手法則）
→ 同じ「右旋回」には符号反転が必要
```

### NED回転 → WebGL回転 変換

```javascript
// NED姿勢からWebGL回転への変換（右手系維持）
// Convert from NED attitude to WebGL rotation (preserves handedness)
function nedRotationToWebGL(roll, pitch, yaw) {
    return {
        x: -pitch,   // Pitch → WebGL X rotation (sign inverted)
        y: -yaw,     // Yaw → WebGL Y rotation (sign inverted)
        z:  roll     // Roll → WebGL Z rotation
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
        -pitch,   // X: Pitch (sign inverted)
        -yaw,     // Y: Yaw (sign inverted)
        roll      // Z: Roll
    );
}
```

### 検証例

| NED姿勢 | WebGL回転 | 見た目 |
|---------|-----------|--------|
| Roll = +30° | rotation.z = +30° | 右翼下げ |
| Pitch = +30° | rotation.x = **-30°** | 機首上げ |
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
| `simulator/sandbox/genesis_sim/` | Genesis物理シミュレータ |
| `simulator/assets/meshes/parts/` | STLファイル（WebGL座標） |

## 8. 座標系比較サマリー

| システム | 右 | 上 | 前 | 重力 |
|----------|-----|-----|-----|------|
| NED | +Y | -Z | +X | +Z |
| Genesis | +X | +Z | +Y | -Z |
| WebGL | -X | +Y | +Z | -Y |

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

### Simulation Coordinate System: Genesis (Z-up)

Coordinate system used by the Genesis physics engine. Standard Z-up coordinate system for robot simulation.

| Axis | Positive Direction | Description |
|------|-------------------|-------------|
| X | Horizontal | Right (convention) |
| Y | Horizontal | Forward (convention) |
| Z | Up | Opposite to gravity |

```
        Z (Up)
        ▲
        │
        │
        │
        ●───────▶ Y (Forward?)
       ╱
      ╱
     ▼
    X (Right?)
```

**Used in:**
- Genesis physics simulation
- Reinforcement learning environments

**Gravity setting:** `gravity=(0, 0, -9.81)` (negative Z direction)

### Display Coordinate System: WebGL/Three.js (Y-up, Z-forward)

Used for 3D visualization. Based on rear-following camera view.

| Axis | Positive Direction | Description |
|------|-------------------|-------------|
| X | Left | Aircraft left (to preserve right-handedness) |
| Y | Up | Screen up |
| Z | Forward | Aircraft forward direction |

```
        Y (Up)
        ▲
        │
        │     Z (Forward)
        │    ╱
        │   ╱
        │  ╱
  X ◀───●
(Left)

※ Right-handed: Y × Z = X (Up × Forward = Left)
```

**Used in:**
- STL files (3D mesh data)
- WebGL viewer
- Three.js-based visualization tools

## 3. Position Coordinate Transformations

### NED → WebGL Transformation

Coordinate transformation for visualizing simulation results. Preserves right-handedness.

| WebGL | = | NED | Description |
|-------|---|-----|-------------|
| x | = | **-y** | East/Right → Left (sign inverted) |
| y | = | -z | Down → Up (sign inverted) |
| z | = | +x | North/Forward → Forward |

```javascript
// Convert from NED to WebGL coordinates (preserves handedness)
function nedToWebGL(ned) {
    return {
        x: -ned.y,   // East/Right → Left (sign inverted)
        y: -ned.z,   // Down → Up (sign inverted)
        z:  ned.x    // North/Forward → Forward
    };
}
```

### WebGL → NED Transformation

Coordinate transformation for importing 3D model positions into simulation.

| NED | = | WebGL | Description |
|-----|---|-------|-------------|
| x | = | +z | Forward → North/Forward |
| y | = | **-x** | Left → East/Right (sign inverted) |
| z | = | -y | Up → Down (sign inverted) |

```javascript
// Convert from WebGL to NED coordinates
function webglToNED(webgl) {
    return {
        x:  webgl.z,   // Forward → North
        y: -webgl.x,   // Left → East/Right (sign inverted)
        z: -webgl.y    // Up → Down (sign inverted)
    };
}
```

### Transformation Matrix

```
            | 0 -1  0 |
T_NED→WebGL = | 0  0 -1 |    det(T) = +1 (preserves handedness)
            | 1  0  0 |
```

### NED → Genesis Transformation

Coordinate transformation for passing control computation results to Genesis simulation.

| Genesis | = | NED | Description |
|---------|---|-----|-------------|
| x | = | +y | East/Right → X |
| y | = | +x | North/Forward → Y |
| z | = | -z | Down → Up (sign inverted) |

```python
# Convert from NED to Genesis coordinates
def ned_to_genesis(ned):
    return {
        'x': ned['y'],    # East/Right → X
        'y': ned['x'],    # North/Forward → Y
        'z': -ned['z']    # Down → Up (sign inverted)
    }
```

### Genesis → NED Transformation

Coordinate transformation for passing Genesis simulation results to control computation.

| NED | = | Genesis | Description |
|-----|---|---------|-------------|
| x | = | +y | Y → North/Forward |
| y | = | +x | X → East/Right |
| z | = | -z | Up → Down (sign inverted) |

```python
# Convert from Genesis to NED coordinates
def genesis_to_ned(genesis):
    return {
        'x': genesis['y'],    # Y → North/Forward
        'y': genesis['x'],    # X → East/Right
        'z': -genesis['z']    # Up → Down (sign inverted)
    }
```

### WebGL (STL) → Genesis Transformation

Coordinate transformation for loading STL files (WebGL coordinates) into Genesis simulation.

| Genesis | = | WebGL | Description |
|---------|---|-------|-------------|
| x | = | **-x** | Left → Right (sign inverted) |
| y | = | +z | Forward → Forward |
| z | = | +y | Up → Up |

```python
# Convert from WebGL (STL) to Genesis coordinates
def webgl_to_genesis(webgl):
    return {
        'x': -webgl['x'],   # Left → Right (sign inverted)
        'y':  webgl['z'],   # Forward → Forward
        'z':  webgl['y']    # Up → Up
    }
```

**Transformation Matrix:**
```
              | -1  0  0 |
T_WebGL→Genesis = |  0  0  1 |    det(T) = +1 (preserves handedness)
              |  0  1  0 |
```

### Genesis → WebGL (STL) Transformation

Coordinate transformation for visualizing Genesis simulation results in WebGL.

| WebGL | = | Genesis | Description |
|-------|---|---------|-------------|
| x | = | **-x** | Right → Left (sign inverted) |
| y | = | +z | Up → Up |
| z | = | +y | Forward → Forward |

```python
# Convert from Genesis to WebGL coordinates
def genesis_to_webgl(genesis):
    return {
        'x': -genesis['x'],   # Right → Left (sign inverted)
        'y':  genesis['z'],   # Up → Up
        'z':  genesis['y']    # Forward → Forward
    }
```

### Loading STL in Genesis

```python
import genesis as gs

# When loading STL files, Genesis automatically interprets coordinates
# However, to explicitly transform axes:
mesh = scene.add_entity(
    gs.morphs.Mesh(
        file='frame.stl',
        scale=0.001,  # mm → m
        # Adjust with rotation for WebGL→Genesis coordinate transform if needed
        # euler=(90, 0, 0) etc.
    ),
)
```

**Note:** Genesis auto-recognizes many STL formats, but if axis orientation differs from expectation, use the `euler` parameter for rotation adjustment.

## 4. Rotation Coordinate Transformation

### Rotation Axis Mapping

| NED Rotation Axis | WebGL Rotation Axis | Note |
|-------------------|---------------------|------|
| X (Roll) | Z | Same direction |
| Y (Pitch) | **-X** | **Direction inverted** |
| Z (Yaw) | **-Y** | **Direction inverted** |

### Why are Pitch and Yaw Sign Inverted?

```
From position axis mapping:
  NED Y (Right) → WebGL -X (Left)  → Pitch axis inverted
  NED Z (Down) → WebGL -Y (negative Up) → Yaw axis inverted

NED:   +Pitch = Nose up (Y-axis points right, right-hand rule)
WebGL: +X rotation = Nose down (X-axis points left, right-hand rule)
→ To get "nose up", sign must be inverted

NED:   +Yaw = Turn right (Z-axis points down, right-hand rule)
WebGL: +Y rotation = Turn left (Y-axis points up, right-hand rule)
→ To get "turn right", sign must be inverted
```

### NED Rotation → WebGL Rotation Transformation

```javascript
// Convert from NED attitude to WebGL rotation (preserves handedness)
function nedRotationToWebGL(roll, pitch, yaw) {
    return {
        x: -pitch,   // Pitch → WebGL X rotation (sign inverted)
        y: -yaw,     // Yaw → WebGL Y rotation (sign inverted)
        z:  roll     // Roll → WebGL Z rotation
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
        -pitch,   // X: Pitch (sign inverted)
        -yaw,     // Y: Yaw (sign inverted)
        roll      // Z: Roll
    );
}
```

### Verification Examples

| NED Attitude | WebGL Rotation | Appearance |
|--------------|----------------|------------|
| Roll = +30° | rotation.z = +30° | Right wing down |
| Pitch = +30° | rotation.x = **-30°** | Nose up |
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
| `simulator/sandbox/genesis_sim/` | Genesis physics simulator |
| `simulator/assets/meshes/parts/` | STL files (WebGL coordinates) |

## 8. Coordinate System Comparison Summary

| System | Right | Up | Forward | Gravity |
|--------|-------|-----|---------|---------|
| NED | +Y | -Z | +X | +Z |
| Genesis | +X | +Z | +Y | -Z |
| WebGL | -X | +Y | +Z | -Y |
