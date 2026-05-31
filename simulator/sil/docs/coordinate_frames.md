# SIL Coordinate Frames — MuJoCo ↔ StampFly
# SIL 座標系 — MuJoCo ↔ StampFly 対応

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。
>
> これは `simulator/sil/frames/` 変換モジュールの**仕様**であり、MuJoCo と StampFly の座標系の**唯一の対応表**です。SIL 内で座標変換を行うのはこの `frames` モジュールだけとし、他のどこでも座標変換をしないこと（旧 SIL が座標系不整合4箇所で崩れた反省）。

## 1. この文書について

### 決定（2026-05-31）

**MuJoCo は自然な座標系（Z 上）のまま使い、座標変換はソフト（`frames` モジュール）で行う。**

- **理由（ユーザー判断）**: 完成後にビューアで新ファームや制御理論を確認するとき、MuJoCo を StampFly に合わせて Z 下にすると、画面で**感覚と逆さまに動いて見えて混乱する**。ビューアの目的（視覚確認・レビュー動画）を守るため、MuJoCo は正立のままにする。
- 代償（座標変換が SIL に入る）は、**この1モジュールに閉じ込めて単体テストで保証**し、本文書で対応を明文化することで管理する。

### 決定（2026-05-31）— センサのドライバ正規化（論点2）

**各センサのチップ軸→機体軸の振り替え・符号合わせは各ドライバ（HAL）の中で行い、ドライバは機体 FRD の正しい物理量を返す。**

- **理由（ユーザー判断）**: StampFly は単一ハードで搭載向きは固定 → 一度正しく決めれば永久に正しい定数。上位（推定器/制御器を書く人）はチップ軸・搭載向き・符号に煩わされず、機体軸の綺麗な物理量だけを見ればよい。例: 加速度計は重力を Z 軸 **−9.8 m/s²** として返す（NED Z 下と整合。旧ファームの生値 +9.8＋`ba_z≈+2g` ハックは新ファームでは不要）。
- **含意**: チップ軸の混乱（右手/左手混在・鏡映・極性/軸性符号）は全部ドライバ内に閉じ込め、各ドライバで一度だけ実機検証。→ **SIL の sim ドライバは機体 FRD の量を直接返す**ので、この `frames` モジュールは**チップ軸の写像を持たず、§3 の世界/機体回転だけ**を担う。
- **旧ファーム不可侵**: `firmware/vehicle`(87飛行) と `firmware/vehicle_new` は別コードベース（別ドライバコピー・相互参照なし）。変更は vehicle_new のみ。

### 対象読者

- `frames` モジュールの実装者
- SIL の合成センサ・物理・真値を扱う者
- MuJoCo のビューアで挙動を確認する研究者・学生

---

## 2. 2つの座標系

### StampFly（ファーム本体が固定。動かせない）

本体ソースから裏取り済み（`eskf_core.cpp` / `imu_task.cpp` / `calibration.cpp`）。

| 区分 | 系 | 軸 |
|------|----|----|
| ワールド | **NED** | X=North, Y=East, **Z=Down**（下が正）。重力 `g_ned = [0,0,+9.81]` |
| 機体 | **FRD** | X=Forward, Y=Right, **Z=Down** |

```
        StampFly NED / FRD（Z 下）
              X(North/Forward)
               ↑
               │
   Y(East/Right)│
        ←───────┼
               ╱│
              ╱ │
   (Z=Down: 紙面の向こう・下向き)
```

- 姿勢 `attitude[w,x,y,z] = q_nb`（**body→NED**、Hamilton, R=to_dcm=R_nb）。
- 高度 = `−pos_z`（NED の z は下が正なので、上空で負）。

### MuJoCo（自然なまま。Z 上）

MuJoCo は右手系・Z 上が標準。本 SIL では MuJoCo ワールドを **ENU**、機体を **FLU** と定義する。

| 区分 | 系 | 軸 |
|------|----|----|
| ワールド | **ENU** | X=East, Y=North, **Z=Up**（上が正）。重力 `<option gravity="0 0 -9.81">` |
| 機体 | **FLU** | X=Forward, Y=Left, **Z=Up** |

```
        MuJoCo ENU / FLU（Z 上）
              Z(Up)
               ↑
               │
               │
               ┼───────→ Y(North/Forward は X… 下記注意)
              ╱
             ╱
          X(East)
```

- MuJoCo の機体姿勢 `framequat[w,x,y,z]` は **body(FLU)→world(ENU)**（Hamilton, body→world）。
- MJCF では機体を**プロペラ上向き（自然）**に記述する。

> **NED も ENU も右手系**なので、両者の間には正規回転（det=+1）が存在する。左手系問題は起きない。

---

## 3. 変換（これが `frames` モジュールの中身）

### 3.1 ワールドのベクトル変換 ENU ↔ NED

```
v_ned = (N, E, D) = ( enu.y,  enu.x,  −enu.z )      # XY 入替 ＋ Z 反転
v_enu = (E, N, U) = ( ned.y,  ned.x,  −ned.z )      # 同形（involution）
```

行列 `M_we = [[0,1,0],[1,0,0],[0,0,−1]]`（det=+1）。これは軸 `(1,1,0)/√2` 周りの **180° 回転**。

### 3.2 機体のベクトル変換 FLU ↔ FRD

```
v_frd = (F, R, D) = ( flu.x, −flu.y, −flu.z )       # Y, Z 反転（body X 周り 180°）
v_flu = (F, L, U) = ( frd.x, −frd.y, −frd.z )       # 同形（involution）
```

行列 `M_bf = [[1,0,0],[0,−1,0],[0,0,−1]]`（det=+1）。body X 軸周りの **180° 回転**。

### 3.3 姿勢クォータニオン MuJoCo framequat → StampFly q_nb

固定クォータニオン（単位）:

```
q_we = [0,  1/√2,  1/√2,  0]     # ENU→NED（§3.1 の M_we）
q_bf = [0,  1,     0,     0]     # FRD↔FLU（§3.2 の M_bf, body X 周り 180°）
```

変換:

```
q_nb = q_we ⊗ q_mj ⊗ q_bf        # ⊗ は Hamilton 積（左が後から作用）
       （q_mj = MuJoCo framequat = body(FLU)→world(ENU)）
```

導出: `v_frd →(M_bf)→ v_flu →(q_mj)→ v_enu →(M_we)→ v_ned` なので `R_nb = M_we · R(q_mj) · M_bf`。

**検算（水平・北向き）:** 機体が水平で北を向くとき、StampFly では body FRD が NED と一致 → `q_nb = 単位元`。このとき MuJoCo は `q_mj = [0.7071,0,0,0.7071]`（ENU で北=+Y を向く 90°ヨー）。上式に代入すると `q_nb = (−1,0,0,0)`＝単位元（`q` と `−q` は同一回転）。✓

> 逆変換が要る場合（StampFly→MuJoCo）は各回転の共役/転置を使う: `q_mj = q_we* ⊗ q_nb ⊗ q_bf*`、`v_enu = M_we^T·v_ned`、`v_flu = M_bf·v_frd`。

---

## 4. センサ・アクチュエータの対応

合成センサは**物理から第一原理で StampFly 系（FRD）で作る**。MuJoCo 内蔵センサ（`<accelerometer>`/`<gyro>`/`<rangefinder>`）は**検算リファレンス**として使う（規約差に注意）。

### 4.1 加速度計（ドライバ正規化後）

- **ドライバ出力（機体 FRD、重力を −9.8 として返す）**: `out_frd = R_bn·(a_world_ned − g_ned)`、`g_ned=[0,0,+9.81]`、`R_bn = inv_rotate(q_nb)`。水平静止で `[0,0,−9.81]`（重力が下軸 −9.8）。
- a_world_ned は機体 CG の運動加速度（MuJoCo の機体加速度を §3.1 で NED へ）。
- **MuJoCo `<accelerometer>` と直接一致**: MuJoCo の加速度計は a−g を site(FLU) 系で返す。FLU site・水平静止で `[0,0,+9.81]`、これを §3.2 で FRD にすると `[0,0,−9.81]` ＝ **ドライバ出力と符号まで一致**（新方針は MuJoCo の加速度計規約と整合 → 検算が素直）。
- 旧ファームの `ba_z≈+2g` 起動ハックは**新ファームでは不要**（ドライバが −9.8 を直接返す）。SIL の sim ドライバも `out_frd` を直接返す。

### 4.2 ジャイロ

- StampFly（body FRD, rad/s, [roll-x, pitch-y, yaw-z]）= MuJoCo `<gyro>`（FLU site の角速度）を §3.2 で FRD へ: `gyro_frd = M_bf·gyro_flu`。符号は RH about FRD で一致。

### 4.3 センサ系 → 機体系（ドライバ正規化, 論点2）

各センサのチップ軸→機体軸の振り替え（remap）・符号合わせは**各ドライバ（HAL）の中**で行い、ドライバは**機体 FRD の正しい物理量**を返す。

- チップ軸は右手/左手混在しうる → 写像は**鏡映（det=−1）にもなり得る**。極性ベクトル（加速度・速度）はそのまま、**軸性ベクトル（ジャイロ・磁気）は鏡映時に符号が1つ余分**（`a' = det(R)·R·a`）。これらは**ドライバ内で一度だけ正しく実装し、実機で検証**する固定値。
- 旧ファームは imu_task で remap していた（`body.x=sensor.y` 等）。**新ファームはこれをドライバへ移す**ので imu_task は薄くなる。
- → **SIL の sim ドライバは機体 FRD の量を直接返す**（チップ系の往復モデルは不要）。この `frames` モジュールはチップ remap を持たない。

### 4.4 ToF / Baro / Flow / Mag

| センサ | 規約 | SIL 合成 |
|--------|------|---------|
| ToF（下向き距離 m） | `pos_z = −height`、`height = distance·cosR·cosP` | `distance = −pos_z/(cosR·cosP)`。MuJoCo rangefinder を body **+Z（FRD 下）**に向けると自然 |
| Baro（高度 m, 上正） | `altitude = −pos_z` | `altitude = −pos_z` ＋ノイズ |
| Flow（生 dx,dy counts） | **body remap なし**。dx=前, dy=右 | `dx ∝ (vx_body/height)·dt/flow_rad_per_pixel`（+ジャイロ誘起）, dy 同様。`flow_rad_per_pixel=0.00222` |
| Mag（既定OFF, body µT） | ref=`{20,0,40}` NED | body 磁場 = `R_bn·mag_ref` |

### 4.5 ミキサー → 物理（モータ力の適用）

- `actuator_motor.duty[4]`（0..1）を読み、per-motor 推力 `T_i = k_thrust·duty_i²`（k_thrust=0.168 N）。
- 推力は機体を**上に押す** = FRD で `−Z` 方向（上＝−Z）。各モータ位置（FRD: ±0.023m, CG上 −0.005m）に力 `[0,0,−T_i]` を、ヨー反トルク `τ_yaw,i = ±κ·T_i`（κ=0.00971, CCW=M1/M3）を与える。
- 機体 FRD の力・トルクを §3.2 で MuJoCo FLU に直して適用: `F_flu = M_bf·F_frd`（推力 −Z_frd → +Z_flu＝上、整合）。

---

## 5. 検証（`frames` 単体テスト・正準ケース）

1. **水平静止 → 加速度計のドライバ出力 `out_frd = [0,0,−9.81]`（重力が下軸 −9.8）、ジャイロ `[0,0,0]`**（最重要）。
2. **姿勢往復**: 水平北向きで `q_nb = 単位元`（±）。任意姿勢で MuJoCo→StampFly→（逆）が元に戻る。
3. **固定回転の性質**: `M_we`, `M_bf` が det=+1・involution。`q_we ⊗ q_we* = 単位`。
4. **軸符号**: 純ロール/ピッチ/ヨー角速度 → 対応軸だけ正符号で立つ。
5. **速度往復**: +X 前進速度(NED) ↔ MuJoCo(ENU) 往復一致。
6. **重力の向き**: 機体を前方へ θ 傾ける → 合成加速度計 X が想定符号で現れる。
7. **MuJoCo 内蔵センサとの一致**: §4.1/4.2 の規約差を補正後、内蔵 `<accelerometer>`/`<gyro>` と合成値が一致。

---

## 6. 実装

- 変換は `simulator/sil/frames/`（`frames.hpp` / `frames.cpp`）に集約。`sf_math`（Vec3/Quat, Hamilton）を共有して本体と同じクォータニオン規約を使う。
- 単体テスト `simulator/sil/frames/frames_test.cpp`（§5 の正準ケース）。
- **SIL の他のどこでも座標変換をしない。** 物理・合成センサ・真値は全てこのモジュール経由。

---

<a id="english"></a>

## 1. About This Document

This is the **specification** for the `simulator/sil/frames/` transform module and the **single authoritative mapping** between MuJoCo and StampFly coordinate systems. **Only** this `frames` module performs coordinate transforms in the SIL — nowhere else (the old SIL broke with 4 scattered frame inconsistencies).

### Decision (2026-05-31)

**Keep MuJoCo in its natural (Z-up) frame; do the coordinate transform in software (`frames` module).**

- **Rationale (user):** when later using the MuJoCo viewer to check new firmware / control theory, aligning MuJoCo to StampFly's Z-down would make the scene move **inverted vs. intuition and cause confusion**. To preserve the viewer's purpose (visual checking, review video), MuJoCo stays upright. The cost (a transform inside the SIL) is contained in this one module, guaranteed by unit tests, and documented here.

## 2. Two Coordinate Systems

- **StampFly (fixed by firmware):** World = **NED** (X-north, Y-east, **Z-down**; g_ned=[0,0,+9.81]); Body = **FRD** (X-fwd, Y-right, Z-down); attitude `q_nb` = body→NED; altitude = −pos_z.
- **MuJoCo (natural, Z-up):** World = **ENU** (X-east, Y-north, **Z-up**; gravity [0,0,−9.81]); Body = **FLU** (X-fwd, Y-left, Z-up); `framequat` = body(FLU)→world(ENU).

Both NED and ENU are right-handed, so a proper rotation (det=+1) exists between them.

## 3. Transforms (the `frames` module)

- **World ENU→NED:** `v_ned = (enu.y, enu.x, −enu.z)` (swap XY, negate Z; 180° about (1,1,0)/√2).
- **Body FLU→FRD:** `v_frd = (flu.x, −flu.y, −flu.z)` (180° about body X).
- **Attitude:** `q_nb = q_we ⊗ q_mj ⊗ q_bf`, with `q_we=[0,1/√2,1/√2,0]`, `q_bf=[0,1,0,0]`. Verified: level/north ⇒ q_nb = (−1,0,0,0) ≡ identity.

## 4. Sensors / Actuator

- **Accelerometer (driver-normalized):** the driver returns body-FRD acceleration with gravity as −9.8: `out_frd = R_bn·(a_world − g_ned)`, `[0,0,−9.81]` at rest. MuJoCo's built-in `<accelerometer>` (a−g, FLU) matches directly after FLU→FRD — same sign. The legacy `ba_z≈+2g` startup hack is no longer needed (per the driver-normalization decision, 論点2).
- **Gyro:** MuJoCo `<gyro>` (FLU) → FRD via `M_bf`.
- **BMI270 sensor frame:** SIL returns sensor-frame data = `(body.y, body.x, −body.z)` so `imu_task`'s fixed remap recovers body FRD (Code Identity).
- **ToF/Baro/Flow/Mag/Mixer:** see Japanese §4.4–4.5. Motor: duty→thrust `k·duty²` up = −Z_frd, applied at motor positions with yaw reaction κ, then FRD→FLU into MuJoCo.

## 5. Verification

Canonical unit tests (Japanese §5): level rest accel=[0,0,−9.81] (driver-normalized); attitude round-trip & level/north=identity; det/involution of fixed rotations; gyro axis signs; velocity round-trip; gravity-tilt sign; agreement with MuJoCo built-in sensors.

## 6. Implementation

`simulator/sil/frames/{frames.hpp,frames.cpp}` (shares `sf_math` Vec3/Quat), tests in `frames_test.cpp`. No coordinate transform anywhere else in the SIL.
