# URDF メッシュと法線ベクトルのガイド

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このドキュメントについて

このドキュメントでは、URDFファイルでSTLメッシュを使用する際のルールと、特に法線ベクトルの扱いについて詳しく解説します。Genesis物理シミュレータでドローンモデルを正しく表示するための知識をまとめています。

### 対象読者

- URDFを使ってロボットモデルを作成する人
- STLファイルの表示問題（面が欠ける、裏返る）をデバッグする人
- 3Dメッシュの法線ベクトルについて理解したい人

## 2. URDFとは

### URDFの基本

URDF (Unified Robot Description Format) は、ロボットの構造を記述するためのXMLフォーマットです。ROSエコシステムで広く使用されており、Genesis、Gazebo、MuJoCoなど多くのシミュレータがサポートしています。

### URDFの基本構造

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- マテリアル定義 -->
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <!-- リンク（剛体パーツ） -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="part.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <mesh filename="part.stl" scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
  </link>

  <!-- ジョイント（接続） -->
  <joint name="joint1" type="continuous">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### URDFの主要要素

| 要素 | 説明 |
|------|------|
| `<robot>` | ルート要素。ロボット全体を定義 |
| `<link>` | 剛体パーツ。visual, collision, inertialを持つ |
| `<joint>` | リンク間の接続。回転軸や可動範囲を定義 |
| `<material>` | 色やテクスチャを定義 |
| `<visual>` | 表示用のジオメトリ（見た目） |
| `<collision>` | 衝突判定用のジオメトリ |
| `<inertial>` | 質量と慣性モーメント |

### ジョイントの種類

| 種類 | 説明 |
|------|------|
| `fixed` | 固定（動かない） |
| `revolute` | 回転（角度制限あり） |
| `continuous` | 回転（角度制限なし）※プロペラ向け |
| `prismatic` | 直動（スライド） |
| `floating` | 6自由度 |
| `planar` | 平面上の移動 |

## 3. STLメッシュの基本

### STLフォーマット

STL (Stereolithography) は3Dモデルを三角形の集合で表現するフォーマットです。

```
              頂点2 (v2)
                /\
               /  \
              /    \
             /      \
            /   法線  \
           /    ↑     \
          /____________\
       頂点0 (v0)    頂点1 (v1)
```

### STLの構造（バイナリ形式）

| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0 | 80 bytes | ヘッダ |
| 80 | 4 bytes | 三角形の数 |
| 84 | 50 bytes × n | 三角形データ |

各三角形（50 bytes）:
| オフセット | サイズ | 内容 |
|-----------|--------|------|
| 0 | 12 bytes | 法線ベクトル (nx, ny, nz) |
| 12 | 12 bytes | 頂点0 (x, y, z) |
| 24 | 12 bytes | 頂点1 (x, y, z) |
| 36 | 12 bytes | 頂点2 (x, y, z) |
| 48 | 2 bytes | 属性（通常0） |

## 4. 法線ベクトルの詳細

### 法線ベクトルとは

法線ベクトル（Normal Vector）は、面がどちらを向いているかを示すベクトルです。長さ1に正規化され、面に対して垂直な方向を指します。

```
        法線 (n)
          ↑
          │
    ┌─────┼─────┐
    │     │     │
    │  面 │     │
    │     │     │
    └───────────┘
```

### 法線ベクトルの役割

| 役割 | 説明 |
|------|------|
| ライティング | 光の反射計算に使用 |
| バックフェースカリング | 裏面を描画するか決定 |
| 衝突判定 | 面の内外判定に使用 |
| シェーディング | 面の明暗を計算 |

### 法線ベクトルの決定方法

#### 方法1: STLファイルの明示的な法線

STLファイルには各三角形の法線が明示的に格納されていますが、**多くのレンダラはこれを無視します**。

#### 方法2: 頂点の巻き順（Winding Order）

**これが最も重要なルール**です。三角形の3頂点を順番に見たとき、その順序で法線の向きが決まります。

```
反時計回り（CCW: Counter-Clockwise）= 法線は手前向き

        v2
        /\
       /  \     法線 ●→ 手前（画面から出る方向）
      /    \
     v0────v1

      v0 → v1 → v2 の順で反時計回り
```

```
時計回り（CW: Clockwise）= 法線は奥向き

        v2
        /\
       /  \     法線 ○→ 奥（画面に入る方向）
      /    \
     v1────v0

      v0 → v1 → v2 の順で時計回り
```

### 右手の法則

巻き順から法線方向を求める「右手の法則」：

```
    親指 = 法線方向
      ↑
      │
      │    ┌──→ v1
      │   /
     ✋  v0
      │   \
      │    └──→ v2
      │
   (指の曲がる方向 = 頂点の順序)
```

1. 右手の指を頂点の順序方向に曲げる（v0→v1→v2）
2. 親指が指す方向が法線ベクトルの方向

### 数学的な計算

法線ベクトルは外積（クロス積）で計算できます：

```
edge1 = v1 - v0
edge2 = v2 - v0
normal = normalize(edge1 × edge2)
```

具体的な計算：
```python
def calculate_normal(v0, v1, v2):
    edge1 = (v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2])
    edge2 = (v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2])

    # 外積
    nx = edge1[1]*edge2[2] - edge1[2]*edge2[1]
    ny = edge1[2]*edge2[0] - edge1[0]*edge2[2]
    nz = edge1[0]*edge2[1] - edge1[1]*edge2[0]

    # 正規化
    length = sqrt(nx*nx + ny*ny + nz*nz)
    return (nx/length, ny/length, nz/length)
```

## 5. バックフェースカリング

### バックフェースカリングとは

3Dレンダリングの最適化技術で、カメラに背を向けている面（裏面）を描画しないようにします。

```
正しい法線（外向き）      間違った法線（内向き）
     ┌───┐                    ┌───┐
     │ ↗ │ ← 見える           │ ↙ │ ← 見えない！
     │   │                    │   │
     └───┘                    └───┘
   カメラ ●                  カメラ ●
```

### レンダラごとの挙動

| レンダラ | バックフェースカリング |
|----------|----------------------|
| Genesis | 有効（裏面は非表示） |
| WebGL (Three.js) | デフォルト無効（両面表示） |
| Blender | 設定可能 |
| OpenGL | 設定可能 (glCullFace) |

**重要**: WebGLでは正常に見えたSTLがGenesisで面が欠けて見える場合、法線が逆向きの可能性が高いです。

### Genesisでの法線の確認

Genesisは `convexify=False` と `decimate=False` を指定することで、元のメッシュを保持します：

```python
scene.add_entity(
    gs.morphs.Mesh(
        file="model.stl",
        convexify=False,  # 凸包に変換しない
    ),
)
```

## 6. 法線の修正方法

### trimeshを使った法線反転

```python
import trimesh

# STLを読み込み
mesh = trimesh.load("model.stl")

# 法線を反転（頂点の巻き順を逆にする）
mesh.invert()

# 保存
mesh.export("model_fixed.stl")
```

### 法線の一貫性を確認

```python
import trimesh

mesh = trimesh.load("model.stl")

print(f"頂点数: {len(mesh.vertices)}")
print(f"面数: {len(mesh.faces)}")
print(f"水密性: {mesh.is_watertight}")
print(f"巻き順の一貫性: {mesh.is_winding_consistent}")
print(f"体積: {mesh.volume}")  # 負なら法線が内向き
```

### 法線方向の診断

```python
import trimesh
import numpy as np

mesh = trimesh.load("model.stl")
center = mesh.centroid
normals = mesh.face_normals
face_centers = mesh.triangles_center

outward = 0
inward = 0

for fc, fn in zip(face_centers, normals):
    direction = fc - center
    dot = np.dot(direction, fn)
    if dot > 0:
        outward += 1
    else:
        inward += 1

print(f"外向き法線: {outward}")
print(f"内向き法線: {inward}")
print(f"外向き比率: {outward/(outward+inward)*100:.1f}%")
```

## 7. URDFでのメッシュ使用時の注意点

### スケールの設定

STLファイルがミリメートル単位の場合、URDFではメートル単位に変換：

```xml
<mesh filename="part.stl" scale="0.001 0.001 0.001"/>
```

### 座標変換

STLの座標系とシミュレータの座標系が異なる場合、`<origin>`で調整：

```xml
<visual>
  <origin xyz="0 0 0" rpy="-1.5708 0 0"/>  <!-- X軸周り-90度 -->
  <geometry>
    <mesh filename="part.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

### ファイルパスの指定

| 形式 | 例 |
|------|-----|
| 相対パス | `part.stl`（URDFと同じディレクトリ） |
| package:// | `package://robot_description/meshes/part.stl` |
| file:// | `file:///absolute/path/to/part.stl` |

### 複数パーツの組み合わせ

各パーツのSTLが同じ座標系で作成されている場合、origin は (0,0,0) で配置可能：

```xml
<link name="base_link">
  <!-- パーツA -->
  <visual name="part_a">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="part_a.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>

  <!-- パーツB（同じ座標系なのでoffset不要） -->
  <visual name="part_b">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="part_b.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
</link>
```

## 8. トラブルシューティング

### 症状と対処法

| 症状 | 原因 | 対処法 |
|------|------|--------|
| 面が欠けて見える | 法線が逆向き | `mesh.invert()` で反転 |
| 全体が真っ黒 | 全法線が内向き | `mesh.invert()` で反転 |
| 一部だけ黒い | 法線が不一致 | `trimesh.repair.fix_normals(mesh)` |
| モデルが小さすぎる | スケール未設定 | `scale="0.001 0.001 0.001"` |
| 向きがおかしい | 座標系の違い | `rpy` で回転を調整 |
| 穴が空いている | メッシュが開いている | `trimesh.repair.fill_holes(mesh)` |

### デバッグ用コード

```python
import trimesh

def diagnose_stl(filepath):
    mesh = trimesh.load(filepath)

    print(f"=== {filepath} ===")
    print(f"頂点数: {len(mesh.vertices)}")
    print(f"面数: {len(mesh.faces)}")
    print(f"水密性: {mesh.is_watertight}")
    print(f"巻き順一貫性: {mesh.is_winding_consistent}")
    print(f"体積: {mesh.volume:.6f}")
    print(f"境界ボックス: {mesh.bounds}")

    if mesh.volume < 0:
        print("警告: 体積が負 = 法線が内向きの可能性")

    return mesh
```

---

<a id="english"></a>

## 1. Overview

### About This Document

This document explains the rules for using STL meshes in URDF files, with particular focus on normal vector handling. It summarizes the knowledge needed to correctly display drone models in the Genesis physics simulator.

### Target Audience

- People creating robot models with URDF
- People debugging STL display issues (missing faces, inverted surfaces)
- People wanting to understand 3D mesh normal vectors

## 2. What is URDF

### URDF Basics

URDF (Unified Robot Description Format) is an XML format for describing robot structures. It's widely used in the ROS ecosystem and supported by many simulators including Genesis, Gazebo, and MuJoCo.

### Basic URDF Structure

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Material definition -->
  <material name="red">
    <color rgba="1.0 0.0 0.0 1.0"/>
  </material>

  <!-- Link (rigid body part) -->
  <link name="base_link">
    <visual>
      <geometry>
        <mesh filename="part.stl" scale="0.001 0.001 0.001"/>
      </geometry>
      <material name="red"/>
    </visual>
  </link>

  <!-- Joint (connection) -->
  <joint name="joint1" type="continuous">
    <parent link="base_link"/>
    <child link="child_link"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

### Key URDF Elements

| Element | Description |
|---------|-------------|
| `<robot>` | Root element. Defines the entire robot |
| `<link>` | Rigid body part. Has visual, collision, inertial |
| `<joint>` | Connection between links. Defines rotation axis and limits |
| `<material>` | Defines color or texture |
| `<visual>` | Display geometry (appearance) |
| `<collision>` | Collision detection geometry |
| `<inertial>` | Mass and moments of inertia |

## 3. STL Mesh Basics

### STL Format

STL (Stereolithography) represents 3D models as a collection of triangles.

### STL Structure (Binary)

Each triangle (50 bytes):
| Offset | Size | Content |
|--------|------|---------|
| 0 | 12 bytes | Normal vector (nx, ny, nz) |
| 12 | 12 bytes | Vertex 0 (x, y, z) |
| 24 | 12 bytes | Vertex 1 (x, y, z) |
| 36 | 12 bytes | Vertex 2 (x, y, z) |
| 48 | 2 bytes | Attribute (usually 0) |

## 4. Normal Vector Details

### What is a Normal Vector

A normal vector indicates which direction a face is pointing. It's normalized to length 1 and points perpendicular to the surface.

### Roles of Normal Vectors

| Role | Description |
|------|-------------|
| Lighting | Used for light reflection calculations |
| Backface culling | Determines whether to draw back faces |
| Collision detection | Used for inside/outside determination |
| Shading | Calculates face brightness |

### How Normal Direction is Determined

#### Winding Order (Most Important Rule)

When viewing a triangle's 3 vertices in sequence, the order determines the normal direction.

```
Counter-Clockwise (CCW) = Normal points toward viewer

        v2
        /\
       /  \     Normal points OUT (toward viewer)
      /    \
     v0────v1

      v0 → v1 → v2 is counter-clockwise
```

```
Clockwise (CW) = Normal points away

        v2
        /\
       /  \     Normal points IN (away from viewer)
      /    \
     v1────v0

      v0 → v1 → v2 is clockwise
```

### Right-Hand Rule

To find normal direction from winding order:

1. Curl right hand fingers in vertex order (v0→v1→v2)
2. Thumb points in normal vector direction

### Mathematical Calculation

Normal vector is calculated using cross product:

```
edge1 = v1 - v0
edge2 = v2 - v0
normal = normalize(edge1 × edge2)
```

## 5. Backface Culling

### What is Backface Culling

A 3D rendering optimization that doesn't draw faces pointing away from the camera (back faces).

### Behavior by Renderer

| Renderer | Backface Culling |
|----------|------------------|
| Genesis | Enabled (back faces hidden) |
| WebGL (Three.js) | Disabled by default (both sides shown) |
| Blender | Configurable |
| OpenGL | Configurable (glCullFace) |

**Important**: If an STL looks fine in WebGL but has missing faces in Genesis, the normals are likely inverted.

## 6. Fixing Normals

### Inverting Normals with trimesh

```python
import trimesh

# Load STL
mesh = trimesh.load("model.stl")

# Invert normals (reverse vertex winding order)
mesh.invert()

# Save
mesh.export("model_fixed.stl")
```

### Checking Normal Consistency

```python
import trimesh

mesh = trimesh.load("model.stl")

print(f"Vertices: {len(mesh.vertices)}")
print(f"Faces: {len(mesh.faces)}")
print(f"Watertight: {mesh.is_watertight}")
print(f"Winding consistent: {mesh.is_winding_consistent}")
print(f"Volume: {mesh.volume}")  # Negative = normals pointing inward
```

## 7. URDF Mesh Usage Notes

### Scale Setting

If STL is in millimeters, convert to meters in URDF:

```xml
<mesh filename="part.stl" scale="0.001 0.001 0.001"/>
```

### Coordinate Transformation

When STL coordinate system differs from simulator, adjust with `<origin>`:

```xml
<visual>
  <origin xyz="0 0 0" rpy="-1.5708 0 0"/>  <!-- -90° around X-axis -->
  <geometry>
    <mesh filename="part.stl" scale="0.001 0.001 0.001"/>
  </geometry>
</visual>
```

## 8. Troubleshooting

### Symptoms and Solutions

| Symptom | Cause | Solution |
|---------|-------|----------|
| Missing faces | Inverted normals | `mesh.invert()` |
| All black | All normals inward | `mesh.invert()` |
| Partially black | Inconsistent normals | `trimesh.repair.fix_normals(mesh)` |
| Model too small | Scale not set | `scale="0.001 0.001 0.001"` |
| Wrong orientation | Coordinate system mismatch | Adjust `rpy` rotation |
| Holes in mesh | Open mesh | `trimesh.repair.fill_holes(mesh)` |
