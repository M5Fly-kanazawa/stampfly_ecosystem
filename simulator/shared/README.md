# simulator/shared : シミュレータ共有の機体資産

> **Note:** [English version follows after the Japanese section.](#english) / 日本語の後に英語版があります。

## 1. 概要

### このディレクトリについて

`simulator/shared/` は、SILS（ソフトウェアだけで機体を動かす検証環境）・Genesis・Web の 3 次元表示が共有する機体の 3 次元形状・設定・シナリオを置く。このうち**プロペラの形状**は、実測データから作った 4 枚羽根の形状ファイルである。この文書はその出どころ・作り方・座標・回転方向・未確定の点を書く。

| 場所 | 内容 |
|------|------|
| `assets/meshes/propeller_geometry.json` | プロペラの形状データ（値・単位・区分・出どころ付き）。**プロペラの形状の基準**。 |
| `assets/meshes/propeller_source_photos/` | 平面形と翼断面の値の一次資料の写真 2 枚（prop_sim から内容を変えずに写した）。 |
| `assets/meshes/tools/build_propeller.py` | 形状ファイルを作る開発用の補助スクリプト（`propeller_geometry.json` だけを読む）。 |
| `assets/meshes/tools/verify_against_prop_sim.py` | `propeller_geometry.json` が prop_sim の値と一致することの照合（写しの確認用）。 |
| `assets/meshes/tools/test_propeller_mesh.py` | 形状ファイルと機体の定義の整合を調べる試験（CI で動く）。 |
| `assets/meshes/propeller_derived.json` | スクリプトが出す導出値（三角形数・質量・慣性・先端の高さ）。手で直さない。 |
| `assets/meshes/parts/propeller_{fl,fr,rl,rr}.stl` | 機体の位置に置いた 4 個（表示用）。SILS の `stampfly.xml`・Genesis の URDF が読む。 |
| `assets/meshes/parts/propeller_rotor_{ccw,cw}.stl` | 原点がハブの中心・軸が Y の回転用の形 2 個。Web の 3 次元表示（4 か所）が読んで回す。 |

### データの出どころ

このデータは、開発者が公開リポジトリ `kouhei1970/prop_sim`（取得コミット `ca19d1b630c0edecfb293cf8aa280f56fbf7e76d`、2026-08-03）で行った写真からの実測を、2026-11-23 に写したものである。**以後、プロペラの形状の基準はこのリポジトリのデータとし、測り直した場合は `propeller_geometry.json` を更新する。** prop_sim は出どころである。写したデータと写真は、このリポジトリのライセンス（MIT）のもとに置く（開発者の確認済み）。

測定の要点（prop_sim の `docs/STAMPFLY_1209.md` と `prop_sim/presets.py` から要約）:

| 項目 | 内容 |
|------|------|
| 平面形の写真 | 実機プロペラを真上から撮り、同じ面に置いた金属定規の目盛り（cm 目盛り 3 本の間隔 770 px = 10 mm）で 77.0 px/mm に校正した。色で切り分けた外形から、直径 31.21 mm（外周の 99.5 パーセンタイルの 2 倍、不確かさ ±0.2 mm）、羽根 4 枚、ハブ外径 7.04 mm、半径ごとの翼弦長（円弧幅 × cos(後退角)）、後退角（中心線の接線と動径のなす角）、投影面積 292.9 mm² を読んだ。 |
| 翼断面の写真 | 羽根を切った面を黒く塗って撮り、明度と彩度の閾値で抽出し、前縁から後縁までの弦に回して上面・下面からキャンバと厚みを 24 点の表にした（最大キャンバ 7.1 %、最大厚み 9.5 %）。投影誤差の補正はしていない。写真のコード線の傾き（29.6 度）は撮影の向きで決まっただけで、ピッチではない。 |
| ピッチ | 写真から読めない。型番 1209 の公称値 0.9 inch を、幾何ピッチ一定（ねじり角 = atan(P / (2πr))、上限 48 度）として使う。 |

写真と読み取りの結果の対応は `propeller_geometry.json` の `primary_sources` に書いてある。元のカメラ画像（`IMG_3578.jpeg`、`IMG_3720.jpeg`）、定規の読み、読み取りのスクリプト、中間データは prop_sim にも無く、このリポジトリにも無い。

### 形状の作り方と座標

```bash
python3 simulator/shared/assets/meshes/tools/build_propeller.py
python3 simulator/shared/assets/meshes/tools/build_propeller.py --out fine_ccw.stl --fidelity fine --hand ccw --pitch-offset-deg -4
python3 -m pytest -q simulator/shared/assets/meshes/tools
```

| 項目 | 内容 |
|------|------|
| 位置づけ | 開発用の補助スクリプト（`sf` のコマンドにはしない。利用者に提供する道具ではなく、資産の生成用。`PROJECT_PLAN.md` §8 に記載）。依存は numpy と scipy のみ。 |
| 引数なし | 表示用（約 1,800 三角形、1 個約 90 KB）の 6 ファイルと `propeller_derived.json` をこのリポジトリの所定の場所に書く。 |
| `--out` 付き | 1 個だけ指定の場所へ書く。`--fidelity fine`（約 22,800 三角形、約 1.1 MB。数値流体計算などに使う。リポジトリには置かない）、`--hand ccw/cw`、`--diameter-mm`、`--pitch-inch`、`--pitch-offset-deg`（ねじり角を一様にずらす）、`--place X Y Z`。 |
| 座標 | 機体の STL 座標（単位 mm、X = 左・Y = 上・Z = 前）。回転軸は +Y。回転用の形はハブ中心が原点、機体の位置に置いた形はハブ中心が各モータの軸の位置、高さ Y = 6.535 mm。 |
| 回転方向 | 上（+Y）から見て、M1（前右 `fr`）反時計回り、M2（後右 `rr`）時計回り、M3（後左 `rl`）反時計回り、M4（前左 `fl`）時計回り。時計回り用は反時計回り用の鏡像（X を −X にし、面の向きを反転）。 |
| 羽根の向き | 前縁が回転方向の側にあり、前縁が後縁より高く、凸の面が上（+Y）。推力が上向きになる組み合わせで、試験が確かめる。 |
| 構造 | ハブ 1 個と羽根 4 枚の計 5 個の閉じた殻。根元でハブの内側に 0.25 mm 入り込むので、殻同士は重なる（ブーリアン和にはしていない）。各殻は水密で、法線は外向き。 |
| 翼弦長の読み | 表の値は上から見た幅。円弧幅 = c / cos(後退角)、ねじり角 β で傾いた実コード長 = 円弧幅 / cos β として積層する（円筒断面）。 |

### 機体への取り付け

| 項目 | 内容 |
|------|------|
| 高さ | ハブ中心を Y = 6.535 mm に置く。翼端の高さは Y = 6.31〜6.84 mm（中心 6.575 mm。開発者が実物で確かめた 6.4〜6.75 mm の中央）。モータの軸の先端（Y = 6.814 mm）はハブの穴（半径 0.5 mm、軸の半径 0.4 mm）の中に入り、ハブの上面（Y = 7.535 mm）は軸の先端より約 0.7 mm 高い。 |
| すき間 | モータ・フレームとの最短距離は 0.096 mm（ハブの穴と軸の間）で、干渉は無い。翼端とガードの壁（内径 約 32 mm）は約 0.5 mm。 |
| SILS | `simulator/sils/models/stampfly.xml` は位置の補正なし（以前は円板の底をモータの缶に合わせる −0.0017 m の補正があった）。 |
| Genesis（URDF） | `stampfly.urdf` のプロペラのリンクを 1 枚 0.0002525 kg、慣性（軸まわり 13.71e-9、面内 7.01e-9 kg·m²）にし、`base_link` を 0.037 − 4 × 0.0002525 = 0.03599 kg にした（機体の全体 0.037 kg は不変）。見た目の位置は機体の位置に置いた STL のまま。 |
| Web の 3 次元表示 | `landing/index.html`、`simulator/sils/gui/static/app.js`、`tools/log_analyzer/replay_video/page/replay_view.js`、`lib/sfcli/assets/telemetry_web.html` が回転用の 2 個を読み、各ロータの位置に置いて回す。landing は `landing/assets/model/` に写しを持つ（元との一致を試験が確かめる）。 |

### 質量と慣性

| 量 | 値 | 区分 |
|----|----|------|
| 質量（実測） | 0.2525 g（4 枚で 1.01 g） | 開発者の測定（2026-11-23） |
| 質量（prop_sim の見積もり） | 0.26 g | 見積もり（面積 × 板厚 0.55 mm、ハブ 2.0 mm × ポリカーボネート 1.20 g/cm³） |
| 軸まわりの慣性（この形状、一様密度、質量 0.2525 g） | 13.71e-9 kg·m²（I/m = 54.3 mm²、I/(m R²) = 0.223） | 計算（`propeller_derived.json`） |
| 軸まわりの慣性（prop_sim の見積もり） | 1.67e-8 kg·m²（= 0.264 m R²、質量 0.26 g）。質量 0.2525 g に換算すると 1.62e-8 | 見積もり |
| ベンチ実測のロータの慣性 | 1.54e-8 kg·m²（プロペラとモータの回転子を含む。`control/models/stampfly_physical.yaml` の `J`） | ベンチ測定（2026-10-15） |

- 形状から計算したプロペラ単体の値（1.37e-8）は、ベンチ実測のロータの慣性（1.54e-8）の約 89 %で、残り約 0.17e-8 はモータの回転子の分と読める（推論、確度は中）。ベンチの値は形状と独立の測定なので、この変更では変えない。
- prop_sim の 1.67e-8（係数 0.264）がこの形状の値（係数 0.223）より大きい理由は、記録が無く未確認。推論: 平面形に板厚 0.55 mm を一様に掛けると、質量 0.26 g・係数 0.27 前後が再現できる（同じ翼弦長の表から本スクリプトの環境で概算: 円弧幅を使うと 0.261 g・0.271、上から見た幅を使うと 0.252 g・0.258）。この形状の厚みは断面（厚み比 × 翼弦長）で決まり、先端ほど薄いので、質量が内側に寄って係数が小さくなる。
- 形状の殻は根元で重なるので、体積（質量の配分）は約 0.7 % 過大に数える（影響は慣性の 1 % 未満）。

### 未確定の点

| 項目 | 状態 |
|------|------|
| ピッチ | 写真から読めず、公称 0.9 inch（ずらし 0 度）を仮定。実物では未確認。別の検討（数値流体計算とベンチ測定の比較）では公称が最も近いが確定していない。`--pitch-offset-deg` / `--pitch-inch` で作り直せる。上から見た形はピッチによらない。 |
| 翼断面が全半径で同じ | 切断面は 1 か所だけ撮った。仮定。 |
| ハブの形 | 直径 7.04 mm（実測）、高さ 2.0 mm、中心穴の半径 0.5 mm の円柱は仮定。実物は未確認。 |
| 羽根の表裏と前縁の向き | prop_sim の符号の約束から決めた。実物の羽根の向きは未確認。 |
| 撮影したプロペラの回転方向 | 写真のプロペラが反時計回り用か時計回り用か、写真が上から見たものかは記録が無い。ここでは上から見た写真として反時計回り用の形を決めた。 |
| 取付高さ | 先端の高さ Y = 6.4〜6.75 mm は開発者の 3 次元モデルでの読み。ハブの高さ方向の位置（ハブ上面が軸の先端より高い）は仮定。 |

### 触っていないもの

`parts/*_BAK.stl`、`simulator/sandbox/stl_splitter/parts/` の古いプロペラ円板、および生成済みの図（アイコン・講習のスライドの図）は更新していない。

---

<a id="english"></a>

## 1. Overview

### About this directory

`simulator/shared/` holds the vehicle 3D shapes, settings and scenarios shared by SILS (the software-only vehicle test environment), Genesis and the Web 3D views. The **propeller shape** is a 4-blade shape built from measured data. This document gives its origin, how it is built, the frame, the spin directions and the unconfirmed points. The file table above applies (Japanese section).

### Origin of the data

This data was copied on 2026-11-23 from the developer's own photographic measurements in the public repository `kouhei1970/prop_sim` (commit `ca19d1b630c0edecfb293cf8aa280f56fbf7e76d`, 2026-08-03). **From now on, this repository's data is the reference for the propeller shape; update `propeller_geometry.json` when re-measured.** prop_sim is the origin. The copied data and photographs are placed under this repository's licence (MIT), confirmed by the developer.

Measurement summary (from prop_sim `docs/STAMPFLY_1209.md` and `prop_sim/presets.py`):

| Item | Content |
|------|---------|
| Planform photograph | The real propeller was photographed from directly above and calibrated to 77.0 px/mm with a metal ruler laid on the same plane (three cm marks 770 px = 10 mm apart). From the colour-segmented outline it gives the diameter 31.21 mm (twice the 99.5th percentile of the outline radius, uncertainty +-0.2 mm), 4 blades, hub diameter 7.04 mm, chord per radius (arc width x cos(sweep)), sweep (angle between the centre-line tangent and the radial direction) and projected area 292.9 mm2. |
| Section photograph | A cut through a blade was painted black and photographed, extracted by brightness and saturation thresholds, rotated to the leading-edge-to-trailing-edge chord, and the camber and thickness were tabulated at 24 points (max camber 7.1 %, max thickness 9.5 %). The projection error is not corrected. The chord tilt in the photograph (29.6 deg) only reflects the camera direction, not the pitch. |
| Pitch | Not readable from a photograph. The nominal 0.9 inch of type 1209 is used as a constant geometric pitch (twist = atan(P / (2 pi r)), capped at 48 deg). |

The correspondence between the photographs and the extracted values is in `primary_sources` of `propeller_geometry.json`. The original camera images (`IMG_3578.jpeg`, `IMG_3720.jpeg`), the ruler readings, the extraction script and the intermediate data are neither in prop_sim nor in this repository.

### Building the shape and the frame

The commands are in the Japanese section.

| Item | Content |
|------|---------|
| Role | A development helper script (not an `sf` command: it generates assets and is not a user-facing tool; listed in `PROJECT_PLAN.md` section 8). Depends on numpy and scipy only. |
| No arguments | Writes the 6 display files (about 1,800 triangles, about 90 KB each) and `propeller_derived.json` into the repository. |
| With `--out` | Writes one file wherever you want: `--fidelity fine` (about 22,800 triangles, about 1.1 MB, for computational fluid dynamics etc.; not kept in the repository), `--hand ccw/cw`, `--diameter-mm`, `--pitch-inch`, `--pitch-offset-deg` (uniform shift of the twist), `--place X Y Z`. |
| Frame | STL frame of the vehicle (mm, X = left, Y = up, Z = front). The rotation axis is +Y. The rotor-form files have the hub centre at the origin; the placed files have it on each motor axis at height Y = 6.535 mm. |
| Spin | Seen from above (+Y): M1 (front right `fr`) ccw, M2 (rear right `rr`) cw, M3 (rear left `rl`) ccw, M4 (front left `fl`) cw. The cw shape is the mirror image of the ccw one (X -> -X, winding reversed). |
| Blade orientation | Leading edge on the rotation side, higher than the trailing edge, convex face up (+Y): the combination that gives upward thrust, checked by the test. |
| Structure | 5 closed shells: 1 hub and 4 blades. The blade root enters the hub by 0.25 mm, so the shells overlap (no boolean union). Each shell is watertight with outward normals. |
| Chord reading | The table value is the width seen from above. The loft uses arc width = c / cos(sweep) and a true chord = arc width / cos(beta) for the twist angle beta (cylindrical section). |

### Mounting on the vehicle

| Item | Content |
|------|---------|
| Height | Hub centre at Y = 6.535 mm. The tip height is Y = 6.31 to 6.84 mm (centre 6.575 mm, the middle of the 6.4 to 6.75 mm the developer read on the real propeller). The motor-shaft tip (Y = 6.814 mm) sits inside the hub hole (radius 0.5 mm, shaft radius 0.4 mm); the hub top (Y = 7.535 mm) is about 0.7 mm above the shaft tip. |
| Clearance | The shortest distance to the motors and frame is 0.096 mm (hub hole to shaft); no interference. The tip to the guard wall (inner diameter about 32 mm) is about 0.5 mm. |
| SILS | `simulator/sils/models/stampfly.xml` has no position correction (it used to have a -0.0017 m correction that put the disc bottom on the motor can). |
| Genesis (URDF) | Each propeller link in `stampfly.urdf` is 0.0002525 kg with inertia 13.71e-9 (axis) and 7.01e-9 kg m2 (in plane); `base_link` is 0.037 - 4 x 0.0002525 = 0.03599 kg (vehicle total 0.037 kg unchanged). Visual placement stays that of the placed STL. |
| Web 3D views | `landing/index.html`, `simulator/sils/gui/static/app.js`, `tools/log_analyzer/replay_video/page/replay_view.js` and `lib/sfcli/assets/telemetry_web.html` load the two rotor-form files and spin them at each rotor position. The landing page keeps copies in `landing/assets/model/` (a test checks they equal the originals). |

### Mass and inertia

The table above (Japanese section) lists the measured mass 0.2525 g, the prop_sim estimate 0.26 g, the axis inertia of this shape 13.71e-9 kg m2 (I/m = 54.3 mm2, I/(m R^2) = 0.223), the prop_sim estimate 1.67e-8 kg m2 (0.264 m R^2; 1.62e-8 scaled to 0.2525 g) and the bench-measured rotor inertia 1.54e-8 kg m2 (including the motor rotor).

- The shape value of the single propeller (1.37e-8) is about 89 % of the bench rotor inertia (1.54e-8); the remaining 0.17e-8 reads as the motor rotor (inference, medium confidence). The bench value is an independent measurement and is not changed here.
- Why prop_sim's 1.67e-8 (factor 0.264) exceeds this shape's value (factor 0.223) is not documented: unconfirmed. Inference: multiplying the planform by a uniform 0.55 mm thickness reproduces mass 0.26 g and a factor near 0.27 (rough estimate from the same chord table: 0.261 g and 0.271 with the arc width, 0.252 g and 0.258 with the top-view width). In this shape the thickness follows the section (thickness ratio x chord) and is thinner toward the tip, so mass lies further inside and the factor is smaller.
- The shells overlap at the root, so the volume (hence the mass distribution) is overestimated by about 0.7 % (under 1 % effect on the inertia).

### Unconfirmed points

| Item | State |
|------|-------|
| Pitch | Not readable from a photograph; the nominal 0.9 inch (offset 0 deg) is assumed. Unconfirmed on the real propeller. A separate CFD-versus-bench study found the nominal closest, but it is not settled. Rebuild with `--pitch-offset-deg` / `--pitch-inch`. The top view does not depend on the pitch. |
| One section at all radii | Only one cut was photographed. Assumption. |
| Hub shape | Diameter 7.04 mm (measured); height 2.0 mm and centre hole radius 0.5 mm are assumptions. The real hub is unconfirmed. |
| Blade face and leading edge | Taken from prop_sim's sign conventions; the real blade orientation is unconfirmed. |
| Spin of the photographed propeller | Whether the photographed propeller is the ccw or cw type, and whether the photograph is seen from above, is not recorded. The ccw shape here assumes a top view. |
| Mounting height | The tip height Y = 6.4 to 6.75 mm is the developer's reading in the 3D model. The hub position along the axis (hub top above the shaft tip) is an assumption. |

### Not touched

`parts/*_BAK.stl`, the old propeller discs in `simulator/sandbox/stl_splitter/parts/`, and the generated figures (icons, workshop slide figures) are not updated.
