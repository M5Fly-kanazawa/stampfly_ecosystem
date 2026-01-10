# Genesis Integration Guide for NED Users / NED座標系ユーザーのためのGenesis統合ガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

NED座標系で制御設計を行う研究者がGenesis物理シミュレータを使用する際の座標変換と注意点をまとめます。

### 対象読者

- NED座標系で制御設計を行っている研究者
- Genesisを使ってドローンシミュレーションを行う開発者

## 2. 座標系の比較

### 軸の対応関係

| 方向 | NED | Genesis | 変換 |
|------|-----|---------|------|
| 前方 (Forward) | +X | +Y | NED.X → Genesis.Y |
| 右方 (Right) | +Y | +X | NED.Y → Genesis.X |
| 下方 (Down) | +Z | -Z | NED.Z → -Genesis.Z |
| 重力方向 | +Z | -Z | |

### 座標系の図解

```
NED座標系 (制御設計)           Genesis座標系 (シミュレーション)

      X (前/Forward)                    Z (上/Up)
      ▲                                 ▲
      │                                 │
      │                                 │
      ●───────▶ Y (右/Right)            ●───────▶ X (右/Right)
     ╱                                 ╱
    ╱                                 ╱
   ▼                                 ▼
  Z (下/Down)                       Y (前/Forward)

  重力: +Z方向                       重力: -Z方向
```

## 3. オイラー角の変換

### 回転順序の互換性

**重要:** NED と Genesis は**同じ ZYX intrinsic 回転順序**を使用しています。

| システム | 回転順序 | 説明 |
|----------|----------|------|
| NED | ZYX intrinsic | Yaw(Z) → Pitch(Y) → Roll(X) |
| Genesis | ZYX intrinsic | euler=(rx, ry, rz) も同じ順序 |

### オイラー角の対応

軸の意味が異なるため、角度の変換が必要です。

| NED | 軸 | Genesis | 軸 | 変換式 |
|-----|----|---------|----|--------|
| Roll (φ) | X (前) | euler[1] | Y (前) | genesis_ry = ned_roll |
| Pitch (θ) | Y (右) | euler[0] | X (右) | genesis_rx = ned_pitch |
| Yaw (ψ) | Z (下) | -euler[2] | Z (上) | genesis_rz = -ned_yaw |

### 変換式

```python
def ned_euler_to_genesis(roll, pitch, yaw):
    """
    NED ZYX Euler angles to Genesis euler parameter.

    NED: Roll(X前), Pitch(Y右), Yaw(Z下)
    Genesis: euler=(rx右, ry前, rz上)

    Args:
        roll: NED roll angle (rad) - rotation around X (forward)
        pitch: NED pitch angle (rad) - rotation around Y (right)
        yaw: NED yaw angle (rad) - rotation around Z (down)

    Returns:
        tuple: Genesis euler (rx, ry, rz) in degrees
    """
    import math
    genesis_rx = math.degrees(pitch)   # NED Y(右) → Genesis X(右)
    genesis_ry = math.degrees(roll)    # NED X(前) → Genesis Y(前)
    genesis_rz = math.degrees(-yaw)    # NED Z(下) → Genesis Z(上) 符号反転
    return (genesis_rx, genesis_ry, genesis_rz)


def genesis_euler_to_ned(genesis_euler):
    """
    Genesis euler parameter to NED ZYX Euler angles.

    Args:
        genesis_euler: tuple (rx, ry, rz) in degrees

    Returns:
        tuple: (roll, pitch, yaw) in radians
    """
    import math
    rx, ry, rz = genesis_euler
    roll = math.radians(ry)    # Genesis Y(前) → NED X(前)
    pitch = math.radians(rx)   # Genesis X(右) → NED Y(右)
    yaw = math.radians(-rz)    # Genesis Z(上) → NED Z(下) 符号反転
    return (roll, pitch, yaw)
```

## 4. 位置・速度・力の変換

### 位置の変換

```python
def ned_pos_to_genesis(x, y, z):
    """NED position to Genesis position."""
    return (y, x, -z)  # (右, 前, 上)

def genesis_pos_to_ned(gx, gy, gz):
    """Genesis position to NED position."""
    return (gy, gx, -gz)  # (前, 右, 下)
```

### 速度の変換

位置と同じ変換を適用します。

```python
def ned_vel_to_genesis(vx, vy, vz):
    """NED velocity to Genesis velocity."""
    return (vy, vx, -vz)

def genesis_vel_to_ned(gvx, gvy, gvz):
    """Genesis velocity to NED velocity."""
    return (gvy, gvx, -gvz)
```

### 角速度の変換

```python
def ned_omega_to_genesis(p, q, r):
    """
    NED angular velocity to Genesis angular velocity.

    Args:
        p: roll rate (around NED X/forward)
        q: pitch rate (around NED Y/right)
        r: yaw rate (around NED Z/down)

    Returns:
        tuple: Genesis angular velocity (wx, wy, wz)
    """
    return (q, p, -r)  # 軸の対応 + Z軸符号反転

def genesis_omega_to_ned(wx, wy, wz):
    """Genesis angular velocity to NED angular velocity."""
    return (wy, wx, -wz)
```

### 力・トルクの変換

```python
def ned_force_to_genesis(fx, fy, fz):
    """NED force to Genesis force."""
    return (fy, fx, -fz)

def ned_torque_to_genesis(tx, ty, tz):
    """NED torque to Genesis torque."""
    return (ty, tx, -tz)
```

## 5. 制御システム統合のアーキテクチャ

### 推奨構成

```
┌────────────────────────────────────────────────────────────────┐
│                     制御コード (NED座標系)                       │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐        │
│  │ 位置制御器   │ → │ 姿勢制御器   │ → │ レート制御器  │        │
│  │ (x,y,z)_ned │    │ (φ,θ,ψ)    │    │ (p,q,r)     │        │
│  └─────────────┘    └─────────────┘    └─────────────┘        │
│                                              │                 │
│                                              ▼                 │
│                                    ┌─────────────────┐        │
│                                    │ モータミキサー   │        │
│                                    │ thrust, τx,τy,τz│        │
│                                    └────────┬────────┘        │
└─────────────────────────────────────────────┼─────────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │        NED → Genesis 変換層                        │
                    │  ┌──────────────────────┴──────────────────────┐  │
                    │  │ ned_force_to_genesis()                       │  │
                    │  │ ned_torque_to_genesis()                      │  │
                    │  └──────────────────────┬──────────────────────┘  │
                    └─────────────────────────┼─────────────────────────┘
                                              │
                                              ▼
┌────────────────────────────────────────────────────────────────┐
│                    Genesis 物理シミュレーション                   │
│                                                                │
│   入力: 力・トルク (Genesis座標系)                               │
│   出力: 位置・姿勢・速度 (Genesis座標系)                         │
│                                                                │
└────────────────────────────────────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │        Genesis → NED 変換層                        │
                    │  ┌──────────────────────┴──────────────────────┐  │
                    │  │ genesis_pos_to_ned()                         │  │
                    │  │ genesis_euler_to_ned()                       │  │
                    │  │ genesis_vel_to_ned()                         │  │
                    │  │ genesis_omega_to_ned()                       │  │
                    │  └──────────────────────┬──────────────────────┘  │
                    └─────────────────────────┼─────────────────────────┘
                                              │
                                              ▼
                                    ┌─────────────────┐
                                    │ センサフィードバック │
                                    │ (NED座標系)      │
                                    └─────────────────┘
```

### 変換層の実装例

```python
class NEDGenesisInterface:
    """NED座標系の制御コードとGenesisを接続するインターフェース"""

    def __init__(self, drone_entity):
        self.drone = drone_entity

    # === 入力変換 (NED → Genesis) ===

    def apply_force_ned(self, fx, fy, fz):
        """NEDの力をGenesisに適用"""
        gfx, gfy, gfz = ned_force_to_genesis(fx, fy, fz)
        self.drone.set_force([gfx, gfy, gfz])

    def apply_torque_ned(self, tx, ty, tz):
        """NEDのトルクをGenesisに適用"""
        gtx, gty, gtz = ned_torque_to_genesis(tx, ty, tz)
        self.drone.set_torque([gtx, gty, gtz])

    # === 出力変換 (Genesis → NED) ===

    def get_position_ned(self):
        """位置をNED座標で取得"""
        gpos = self.drone.get_pos()
        return genesis_pos_to_ned(gpos[0], gpos[1], gpos[2])

    def get_velocity_ned(self):
        """速度をNED座標で取得"""
        gvel = self.drone.get_vel()
        return genesis_vel_to_ned(gvel[0], gvel[1], gvel[2])

    def get_euler_ned(self):
        """オイラー角をNED (roll, pitch, yaw) で取得"""
        # Genesisからクォータニオンを取得してNEDオイラー角に変換
        quat = self.drone.get_quat()  # [w, x, y, z]
        # ... クォータニオンからNED ZYXオイラー角への変換
        pass

    def get_angular_velocity_ned(self):
        """角速度をNED (p, q, r) で取得"""
        gomega = self.drone.get_ang_vel()
        return genesis_omega_to_ned(gomega[0], gomega[1], gomega[2])
```

## 6. よくある間違いと対処法

| 症状 | 原因 | 対処 |
|------|------|------|
| 機体が逆さまに表示 | Z軸の向きが逆 | 位置のZ成分を符号反転 |
| 前後の動きが逆 | X,Yが入れ替わっていない | ned_pos_to_genesis() を使用 |
| ヨー回転が逆方向 | Z軸符号反転忘れ | yaw → -rz |
| ピッチとロールが入れ替わる | 軸の対応間違い | roll→ry, pitch→rx |
| 高度が負の値になる | NED:下が正, Genesis:上が正 | z → -z |

## 7. 検証用コード

```python
def verify_ned_genesis_conversion():
    """変換の正しさを検証するテストコード"""
    import numpy as np

    # テストケース: 機体が北を向いて、少し右に傾いて、上昇中
    ned_pos = (10, 5, -100)      # 北10m, 東5m, 高度100m
    ned_vel = (2, 0.5, -1)       # 前進2m/s, 右0.5m/s, 上昇1m/s
    ned_euler = (0.1, 0.05, 0)   # roll 5.7°, pitch 2.9°, yaw 0°
    ned_omega = (0.01, 0.02, 0)  # p, q, r

    # NED → Genesis
    gen_pos = ned_pos_to_genesis(*ned_pos)
    gen_vel = ned_vel_to_genesis(*ned_vel)
    gen_euler = ned_euler_to_genesis(*ned_euler)
    gen_omega = ned_omega_to_genesis(*ned_omega)

    print("NED → Genesis 変換結果:")
    print(f"  位置: {ned_pos} → {gen_pos}")
    print(f"  速度: {ned_vel} → {gen_vel}")
    print(f"  姿勢: {ned_euler} rad → {gen_euler} deg")
    print(f"  角速度: {ned_omega} → {gen_omega}")

    # Genesis → NED (往復確認)
    ned_pos_back = genesis_pos_to_ned(*gen_pos)
    ned_vel_back = genesis_vel_to_ned(*gen_vel)
    ned_euler_back = genesis_euler_to_ned(gen_euler)
    ned_omega_back = genesis_omega_to_ned(*gen_omega)

    print("\nGenesis → NED 往復確認:")
    print(f"  位置: {np.allclose(ned_pos, ned_pos_back)}")
    print(f"  速度: {np.allclose(ned_vel, ned_vel_back)}")
    print(f"  姿勢: {np.allclose(ned_euler, ned_euler_back)}")
    print(f"  角速度: {np.allclose(ned_omega, ned_omega_back)}")
```

## 8. まとめ

### 変換のポイント

1. **位置・速度・力**: X↔Y入れ替え + Z符号反転
2. **オイラー角**: Roll↔Pitch入れ替え + Yaw符号反転
3. **角速度**: p↔q入れ替え + r符号反転

### 設計指針

- 制御ロジックはNED座標系で維持
- 変換は境界層（インターフェースクラス）に集約
- 変換関数は十分にテストすること

---

<a id="english"></a>

## 1. Overview

### About This Document

This document summarizes coordinate transformations and considerations for researchers who design control systems in NED coordinates when using the Genesis physics simulator.

### Target Audience

- Researchers designing control systems in NED coordinates
- Developers performing drone simulation with Genesis

## 2. Coordinate System Comparison

### Axis Correspondence

| Direction | NED | Genesis | Conversion |
|-----------|-----|---------|------------|
| Forward | +X | +Y | NED.X → Genesis.Y |
| Right | +Y | +X | NED.Y → Genesis.X |
| Down | +Z | -Z | NED.Z → -Genesis.Z |
| Gravity | +Z | -Z | |

## 3. Euler Angle Conversion

### Rotation Order Compatibility

**Important:** NED and Genesis use the **same ZYX intrinsic rotation order**.

| System | Rotation Order | Description |
|--------|----------------|-------------|
| NED | ZYX intrinsic | Yaw(Z) → Pitch(Y) → Roll(X) |
| Genesis | ZYX intrinsic | euler=(rx, ry, rz) same order |

### Euler Angle Mapping

Since axis meanings differ, angle conversion is required.

| NED | Axis | Genesis | Axis | Conversion |
|-----|------|---------|------|------------|
| Roll (φ) | X (fwd) | euler[1] | Y (fwd) | genesis_ry = ned_roll |
| Pitch (θ) | Y (right) | euler[0] | X (right) | genesis_rx = ned_pitch |
| Yaw (ψ) | Z (down) | -euler[2] | Z (up) | genesis_rz = -ned_yaw |

## 4. Position/Velocity/Force Conversion

### Position Conversion

```python
def ned_pos_to_genesis(x, y, z):
    return (y, x, -z)  # (right, forward, up)

def genesis_pos_to_ned(gx, gy, gz):
    return (gy, gx, -gz)  # (forward, right, down)
```

## 5. Summary

### Conversion Key Points

1. **Position/Velocity/Force**: X↔Y swap + Z sign inversion
2. **Euler angles**: Roll↔Pitch swap + Yaw sign inversion
3. **Angular velocity**: p↔q swap + r sign inversion

### Design Guidelines

- Keep control logic in NED coordinate system
- Centralize conversions in boundary layer (interface class)
- Thoroughly test conversion functions
