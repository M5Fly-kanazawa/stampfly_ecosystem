#!/usr/bin/env python3
"""Render the StampFly 3D model as a flat-shaded orthographic PNG that lines
up, pixel-for-pixel, with the FRD (Forward-Right-Down) body axes drawn in
docs/events/_shared/tikz/imu_axes.tex.

StampFly 3Dモデル（STL）を、docs/events/_shared/tikz/imu_axes.tex が描く
FRD（前・右・下）機体座標系の矢印と寸分違わず重なるように、平行投影で
フラットシェーディングしたPNGへレンダリングするスクリプト。

Why this exists / 目的:
  imu_axes.tex used to draw the airframe as a hand-drawn square plate with
  four rotor circles. This script replaces that placeholder with a real
  rendering of the CAD model (landing/assets/model/*.stl), using EXACTLY
  the same camera (elevation/azimuth) and coordinate convention as the
  TikZ figure, so the rendered body and the TikZ axis arrows/labels stay
  aligned when the PNG is placed underneath them.
  以前の imu_axes.tex は機体を手描きの正方形プレート+ローター4円で表現して
  いた。本スクリプトはそれを実物のCADモデル(landing/assets/model/*.stl)の
  レンダリングに置き換える。TikZ図と全く同じカメラ（仰角・方位角）と座標
  規約を使うことで、PNGをTikZの軸矢印/ラベルの下に敷いたときにぴったり
  重なるようにしている。

How the frames relate / 座標系の対応:
  The STL files use the three.js/landing-page convention (Y up, right-
  handed): bounding boxes and part positions (see PROJECT context) show
  forward = +Z_landing, right = -X_landing, up = +Y_landing. The FRD body
  frame used by imu_axes.tex is X=forward, Y=right, Z=down. The mapping
  below (LANDING_TO_FRD) is the unique right-handed (det = +1) rotation
  consistent with those two axis identifications; it is verified in code
  by checking det(LANDING_TO_FRD) == +1 and by comparing the projected
  motor positions against the TikZ script's own (BodyHalf, BodyHalf, 0)
  corner points.
  STLファイルは three.js/ランディングページの規約（Y上・右手系）を使って
  おり、境界ボックスとパーツ位置から forward = +Z_landing、
  right = -X_landing、up = +Y_landing であることが分かる。imu_axes.tex の
  FRD機体座標系は X=前, Y=右, Z=下。下記の LANDING_TO_FRD は、この2つの
  軸対応から一意に定まる右手系（行列式 = +1）の回転であり、
  det(LANDING_TO_FRD) == +1 をコードで検証し、投影したモータ位置を
  TikZ側の (BodyHalf, BodyHalf, 0) コーナー点と比較することでも確認する。
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# Paths / パス
# ---------------------------------------------------------------------------

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[4]  # .../docs/events/_shared/tikz/scripts -> repo root
MODEL_DIR = REPO_ROOT / "landing" / "assets" / "model"
OUT_PNG = SCRIPT_DIR.parent / "stampfly_frd_iso.png"
# Temporary alignment-check output only (instructor fix, round 3, item 1).
# Never referenced by imu_axes.tex; --debug swaps it in over OUT_PNG by
# hand for a one-off visual check, then the normal (no-flag) run restores
# the clean OUT_PNG.
# 一時的な位置合わせ確認専用の出力（講師修正指示・第3弾・項目1）。
# imu_axes.tex から参照されることはない。--debug 実行時のみ生成し、
# 確認のため手動で OUT_PNG に差し替える。確認後は通常実行（フラグ無し）で
# 重ね書きのない OUT_PNG に戻す。
DEBUG_OUT_PNG = SCRIPT_DIR.parent / "stampfly_frd_iso_DEBUG.png"

# Each STL part paired with a flat RGB color (0-1 range) and a human label.
# Colors chosen for readability against a transparent background on both
# light and dark slide themes: light grey frame, dark grey motors, dark
# green PCB, dark blue battery, distinct greys for the remaining small
# parts.
# 各STLパーツと塗り色(0-1のRGB)、ラベル名。透明背景の上で明暗どちらの
# スライドテーマでも読みやすいよう、機体フレームは明るいグレー、モータは
# 濃いグレー、基板は濃い緑、バッテリは濃い青、残りの小部品は別系統の
# グレーにしている。
PART_COLORS: dict[str, tuple[float, float, float]] = {
    "frame": (0.78, 0.78, 0.80),
    "pcb": (0.05, 0.35, 0.15),
    "battery": (0.06, 0.16, 0.45),
    "battery_adapter": (0.55, 0.55, 0.60),
    # M5Stack brand orange (#F27D26), per instructor request, so the
    # M5StampS3 module reads clearly as "the M5Stack part" on the body.
    # M5Stackのブランドオレンジ(#F27D26)。講師指示により、M5StampS3が
    # 「M5Stackの部品」だと一目で分かる色にする。
    "m5stamps3": (0xF2 / 0xFF, 0x7D / 0xFF, 0x26 / 0xFF),
    "motor_fr": (0.22, 0.22, 0.24),
    "motor_fl": (0.22, 0.22, 0.24),
    "motor_rl": (0.22, 0.22, 0.24),
    "motor_rr": (0.22, 0.22, 0.24),
}
PART_NAMES = list(PART_COLORS.keys())
MOTOR_NAMES = ["motor_fr", "motor_fl", "motor_rl", "motor_rr"]

# ---------------------------------------------------------------------------
# Binary STL reader (no external mesh library available/needed)
# バイナリSTL読み込み（外部メッシュライブラリ不要・numpyのみ）
# ---------------------------------------------------------------------------

STL_HEADER_BYTES = 80
STL_COUNT_BYTES = 4
STL_RECORD_BYTES = 50  # 12 (normal) + 3*12 (vertices) + 2 (attribute byte count)


def read_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (triangles, normals): triangles has shape (N, 3, 3) — N
    triangles x 3 vertices x (x, y, z) in the file's native mm units;
    normals has shape (N, 3) as stored in the file (used only as a
    fallback; face normals are recomputed from vertices for shading).
    (三角形配列, 法線配列) を返す。triangles は (N, 3, 3)
    （N個の三角形 x 頂点3つ x xyz、ファイル本来のmm単位）。normals は
    ファイルに格納された値（フォールバック用。シェーディングには頂点から
    再計算した法線を使う）。
    """
    data = path.read_bytes()
    if data[:5] == b"solid":
        raise ValueError(f"{path} looks like an ASCII STL; this reader only handles binary STL")
    (n_tri,) = struct.unpack_from("<I", data, STL_HEADER_BYTES)
    expected = STL_HEADER_BYTES + STL_COUNT_BYTES + n_tri * STL_RECORD_BYTES
    if len(data) < expected:
        raise ValueError(f"{path}: truncated binary STL (expected {expected} bytes, got {len(data)})")

    triangles = np.empty((n_tri, 3, 3), dtype=np.float64)
    normals = np.empty((n_tri, 3), dtype=np.float64)
    offset = STL_HEADER_BYTES + STL_COUNT_BYTES
    for i in range(n_tri):
        normals[i] = struct.unpack_from("<3f", data, offset)
        offset += 12
        for v in range(3):
            triangles[i, v] = struct.unpack_from("<3f", data, offset)
            offset += 12
        offset += 2  # attribute byte count, unused / 属性バイト数（未使用）
    return triangles, normals


# ---------------------------------------------------------------------------
# Landing-page (three.js, Y-up) coordinates -> FRD body coordinates
# ランディングページ座標(three.js, Y上) -> FRD機体座標
# ---------------------------------------------------------------------------

# Axis identification, derived from the STL bounding boxes / part positions
# (forward = +Z_landing, right = -X_landing, up = +Y_landing):
#   X_frd (forward) = +Z_landing
#   Y_frd (right)   = -X_landing
#   Z_frd (down)    = -Y_landing
# 軸の対応関係（STLの境界ボックス・パーツ位置から: forward = +Z_landing,
# right = -X_landing, up = +Y_landing）:
#   X_frd（前） = +Z_landing
#   Y_frd（右） = -X_landing
#   Z_frd（下） = -Y_landing
LANDING_TO_FRD = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ]
)
_det = np.linalg.det(LANDING_TO_FRD)
assert abs(_det - 1.0) < 1e-12, f"LANDING_TO_FRD must be a proper (det=+1) rotation, got det={_det}"


def landing_to_frd(points_mm: np.ndarray) -> np.ndarray:
    """Apply the fixed rotation to an (..., 3) array of landing-frame points
    (mm), returning FRD-frame points (mm) of the same shape.
    landing座標系の点群(mm)、形状(..., 3)に固定回転を適用し、同じ形状の
    FRD座標系の点群(mm)を返す。
    """
    return points_mm @ LANDING_TO_FRD.T


# ---------------------------------------------------------------------------
# Camera: identical elevation/azimuth parametrisation to imu_axes.tex
# カメラ: imu_axes.tex と全く同じ仰角・方位角のパラメータ化
# ---------------------------------------------------------------------------

ELEV_DEG = 30.0  # camera elevation above horizontal, matches \ElevDeg in imu_axes.tex / 仰角
AZIM_DEG = 35.0  # camera azimuth from "directly behind", matches \AzimDeg / 方位角

_el = np.deg2rad(ELEV_DEG)
_az = np.deg2rad(AZIM_DEG)

# Screen-right, screen-up and camera-forward (camera -> scene) unit vectors
# in FRD coordinates, using the exact closed forms documented in
# imu_axes.tex (verified there as P P^T = I to 1e-17).
# FRD座標系での画面右・画面上・カメラ前方（カメラ→対象）単位ベクトル。
# imu_axes.tex に記載の閉形式そのもの（P P^T = I を1e-17精度で検証済み）。
R_HAT = np.array([np.sin(_az), np.cos(_az), 0.0])
U_HAT = np.array([np.cos(_az) * np.sin(_el), -np.sin(_az) * np.sin(_el), -np.cos(_el)])
F_HAT = np.array([np.cos(_el) * np.cos(_az), -np.cos(_el) * np.sin(_az), np.sin(_el)])

_P = np.stack([R_HAT, U_HAT])
assert np.allclose(_P @ _P.T, np.eye(2), atol=1e-10), "camera basis must be orthonormal"

# Light direction: a "headlamp" co-located with the camera (light travels
# camera -> scene, i.e. opposite of F_HAT so the light source sits at the
# camera and shines into the scene). This is the simplest fixed light that
# is guaranteed to lit the faces actually visible to the camera.
# 光源方向: カメラに同軸の「ヘッドランプ」（光はカメラ→対象、つまり
# F_HAT の逆向き。光源がカメラ位置にあり奥へ向けて照らす）。カメラから
# 見えている面が必ず照らされる、最も単純な固定光源。
LIGHT_DIR = -F_HAT
AMBIENT = 0.35  # unlit floor brightness (fraction of base color) / 陰影最小の明るさ（基本色に対する割合）
DIFFUSE = 0.65  # additional brightness from direct light, AMBIENT+DIFFUSE=1 / 直接光による追加分（AMBIENT+DIFFUSEで1）


def project(points_units: np.ndarray) -> np.ndarray:
    """Orthographic projection: (..., 3) FRD points (in TikZ units) ->
    (..., 2) screen points, via the linear map P = [R_HAT; U_HAT].
    平行投影: (..., 3) のFRD点（TikZ単位）を P = [R_HAT; U_HAT] の
    線形写像で (..., 2) の画面座標へ変換する。
    """
    return points_units @ _P.T


# ---------------------------------------------------------------------------
# Output canvas: transparent RGBA PNG, exact [-3, 3] x [-3, 3] TikZ-unit
# extent, origin at the pixel center -- matches \includegraphics[width=6cm]
# anchored at (0,0) in imu_axes.tex.
# 出力キャンバス: 透明背景のRGBA PNG。範囲はちょうど[-3, 3]×[-3, 3]TikZ単位、
# 原点はピクセル中心 -- imu_axes.tex で (0,0) に置く
# \includegraphics[width=6cm] と一致させる。
# ---------------------------------------------------------------------------

HALF_EXTENT_UNITS = 3.0  # matches \includegraphics[width=6cm] centred at origin / width=6cmで中心配置に一致
CANVAS_PIXELS = 1800  # was 6 inch x 300 dpi under the old matplotlib path / 旧matplotlib経路の6インチ×300dpi相当


def data_to_pixel(xy_units: np.ndarray) -> np.ndarray:
    """(..., 2) screen points, in TikZ units over [-3, 3] x [-3, 3], to
    (..., 2) pixel coordinates over the CANVAS_PIXELS x CANVAS_PIXELS
    image (column, row), with row 0 at the TOP (y = +HALF_EXTENT_UNITS)
    matching the usual top-down raster/PIL convention.
    (..., 2) の画面座標（TikZ単位、[-3, 3]×[-3, 3]の範囲）を、
    CANVAS_PIXELS×CANVAS_PIXELSの画像上の(..., 2)ピクセル座標（列, 行）に
    変換する。行0が画像の一番上（y = +HALF_EXTENT_UNITS）になる、通常の
    ラスタ画像/PILの上下方向の規約に合わせる。
    """
    col = (xy_units[..., 0] + HALF_EXTENT_UNITS) / (2.0 * HALF_EXTENT_UNITS) * CANVAS_PIXELS
    row = (HALF_EXTENT_UNITS - xy_units[..., 1]) / (2.0 * HALF_EXTENT_UNITS) * CANVAS_PIXELS
    return np.stack([col, row], axis=-1)


def triangulate_polygon_fan(polygon_points: np.ndarray) -> list[np.ndarray]:
    """Split one closed M-vertex polygon (planar or near-planar, star-
    shaped from its own centroid -- true for the blade paddle and the hub
    cap/side quads used here) into M triangles, fanned from the polygon's
    own centroid rather than from vertex 0, so a slightly concave outline
    (like the blade root) still triangulates correctly.
    閉じたM頂点ポリゴン（平面またはほぼ平面で、自身の重心から見て星形 --
    ここで使う羽根パドルやハブの上面/側面クアッドはいずれも該当）を、
    M枚の三角形に分割する。頂点0からではなくポリゴン自身の重心から扇状に
    分割するため、羽根の根元のようなわずかな凹みがあっても正しく
    三角形分割できる。
    """
    centroid = polygon_points.mean(axis=0)
    m = len(polygon_points)
    return [
        np.stack([centroid, polygon_points[i], polygon_points[(i + 1) % m]], axis=0) for i in range(m)
    ]


def rasterize_triangle(
    screen_px: np.ndarray,
    depth3: np.ndarray,
    color_rgb: np.ndarray,
    alpha: float,
    *,
    depth_buffer: np.ndarray,
    color_buffer: np.ndarray,
    alpha_buffer: np.ndarray,
    depth_test_buffer: np.ndarray,
    write_depth: bool,
) -> None:
    """Rasterize one flat-shaded triangle by per-pixel Z-buffer test
    (instructor fix, round 5): replaces the old whole-triangle painter's
    algorithm (sort-by-centroid-depth, draw far-to-near), which produced
    a visible light/dark striping on parts made of many small, similarly-
    angled triangles (the motor cans) because a handful of triangles'
    CENTROID depth ordering does not always match the correct PER-PIXEL
    depth ordering where those triangles overlap on screen.
    Vectorized over the triangle's own screen-space bounding box only
    (not the whole canvas): edge-function barycentric coordinates decide
    which pixels are inside, a small inclusive tolerance avoids 1px seams
    between adjacent triangles that share an edge, and the 3 vertices'
    depths are barycentrically interpolated per pixel (exact, since depth
    is a linear function of position and the triangle is planar).
    画素単位のZバッファ判定で1枚の三角形をフラットシェーディングして描く
    （講師修正指示・第5弾）: 従来の「三角形全体を重心の奥行きで並べ替えて
    遠い順に塗り重ねる」画家アルゴリズムを置き換える。少数の三角形の
    重心の奥行き順は、それらが画面上で重なり合う領域の画素単位の正しい
    奥行き順と必ずしも一致せず、似た向きの小さな三角形が多いモータ缶で
    明暗の縞となって現れていた。
    その三角形自身の画面上の外接矩形だけをベクトル化して処理する
    （キャンバス全体ではない）: エッジ関数による重心座標判定で内部画素を
    決め、隣接する三角形が共有する辺で1px幅の隙間ができないよう小さな
    包含許容誤差を設け、3頂点の奥行きは画素ごとに重心座標で線形補間する
    （三角形は平面なので奥行きは位置の線形関数であり、この補間は厳密に
    正しい）。
    """
    x = screen_px[:, 0]
    y = screen_px[:, 1]
    x_min = max(int(np.floor(x.min())), 0)
    x_max = min(int(np.ceil(x.max())), CANVAS_PIXELS - 1)
    y_min = max(int(np.floor(y.min())), 0)
    y_max = min(int(np.ceil(y.max())), CANVAS_PIXELS - 1)
    if x_min > x_max or y_min > y_max:
        return  # triangle's bbox is entirely off-canvas / 三角形の外接矩形がキャンバス外

    area2 = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])  # 2x signed screen area / 符号付き画面面積の2倍
    if area2 == 0:
        return  # degenerate (zero screen area) triangle / 画面上で面積ゼロの退化三角形

    px_grid, py_grid = np.meshgrid(
        np.arange(x_min, x_max + 1) + 0.5, np.arange(y_min, y_max + 1) + 0.5
    )

    def edge(ax_, ay_, bx_, by_):
        return (bx_ - ax_) * (py_grid - ay_) - (by_ - ay_) * (px_grid - ax_)

    w0 = edge(x[1], y[1], x[2], y[2])  # barycentric weight of vertex 0 / 頂点0の重心座標分子
    w1 = edge(x[2], y[2], x[0], y[0])  # barycentric weight of vertex 1 / 頂点1の重心座標分子
    w2 = edge(x[0], y[0], x[1], y[1])  # barycentric weight of vertex 2 / 頂点2の重心座標分子

    # Small inclusive tolerance (not a strict >=0 test): guarantees no 1px
    # gap along an edge shared by two adjacent triangles, at the cost of a
    # possible sub-pixel overlap the Z-buffer test resolves anyway.
    # 厳密な>=0判定ではなく小さな包含許容誤差を使う: 隣接する2枚の
    # 三角形が共有する辺に沿った1px幅の隙間を無くせる（代わりに生じ得る
    # サブピクセルの重なりはどのみちZバッファ判定で解消される）。
    eps = 1e-6 * abs(area2)
    if area2 > 0:
        inside = (w0 >= -eps) & (w1 >= -eps) & (w2 >= -eps)
    else:
        inside = (w0 <= eps) & (w1 <= eps) & (w2 <= eps)
    if not inside.any():
        return

    b0 = w0 / area2
    b1 = w1 / area2
    b2 = w2 / area2
    pixel_depth = b0 * depth3[0] + b1 * depth3[1] + b2 * depth3[2]

    sub_test = depth_test_buffer[y_min : y_max + 1, x_min : x_max + 1]
    visible = inside & (pixel_depth < sub_test)
    if not visible.any():
        return

    if write_depth:
        # Opaque pass: straight overwrite (depth test + depth write).
        # 不透明パス: 単純な上書き（深度テスト＋深度書き込み）。
        depth_buffer[y_min : y_max + 1, x_min : x_max + 1][visible] = pixel_depth[visible]
        color_buffer[y_min : y_max + 1, x_min : x_max + 1, :][visible] = color_rgb
        alpha_buffer[y_min : y_max + 1, x_min : x_max + 1][visible] = 1.0
    else:
        # Translucent pass: alpha-over compositing against the existing
        # buffer contents, WITHOUT writing depth -- triangles in this pass
        # are depth-TESTED against the frozen opaque buffer
        # (depth_test_buffer) but never occlude each other via the
        # Z-buffer; the caller sorts this pass's triangles far-to-near
        # beforehand so a single "over" pass, applied in that order, is
        # correct. color_buffer holds PREMULTIPLIED color (color*alpha,
        # not straight color) throughout both passes -- for the opaque
        # pass above that is the same thing as straight color, since
        # alpha=1 there; the standard premultiplied "over" operator below
        # (out_premult = src_rgb*src_a + dst_premult*(1-src_a)) is then
        # exact regardless of how much alpha the destination already has,
        # unlike a straight-alpha blend would be. Converted back to
        # straight alpha once, at final PNG export.
        # 半透明パス: 既存のバッファ内容に対してアルファオーバー合成する。
        # 深度は書き込まない -- このパスの三角形は固定した不透明バッファ
        # (depth_test_buffer)に対して深度テストされるが、Zバッファ同士
        # では絶対に隠し合わない。呼び出し側があらかじめこのパスの三角形を
        # 遠い順に並べておくので、その順に1回「over」合成すれば正しい。
        # color_bufferは両パスを通じて「乗算済み(premultiplied)色」
        # （colorそのものではなくcolor*alpha）を保持する -- 上の不透明パス
        # ではalpha=1なので通常のcolorと同じ値になる。下の標準的な
        # premultiplied版over演算子（out_premult = src_rgb*src_a +
        # dst_premult*(1-src_a)）は、宛先が既にどれだけのalphaを持って
        # いても厳密に正しい（straight alphaでの単純ブレンドでは正しくない）。
        # 最終的なPNG書き出し時に一度だけstraight alphaへ変換する。
        sub_premult = color_buffer[y_min : y_max + 1, x_min : x_max + 1, :]
        sub_alpha = alpha_buffer[y_min : y_max + 1, x_min : x_max + 1]
        dst_premult = sub_premult[visible]
        dst_a = sub_alpha[visible]
        new_premult = np.asarray(color_rgb) * alpha + dst_premult * (1.0 - alpha)
        new_a = alpha + dst_a * (1.0 - alpha)
        sub_premult[visible] = new_premult
        sub_alpha[visible] = new_a


# ---------------------------------------------------------------------------
# Body-frame scale: mm per TikZ unit, derived from the actual motor
# positions so it matches imu_axes.tex's BodyHalf convention exactly.
# 機体座標系のスケール: mm/TikZ単位。実際のモータ位置から算出し、
# imu_axes.tex の BodyHalf 規約と厳密に一致させる。
# ---------------------------------------------------------------------------

# imu_axes.tex places the four rotor circles at world (+-BodyHalf,
# +-BodyHalf, 0) TikZ units -- i.e. it assumes the motor arms are equally
# long along body X and Y. BODY_HALF_UNITS must match \BodyHalf there.
# imu_axes.tex は4つのローター円を世界座標 (±BodyHalf, ±BodyHalf, 0)
# TikZ単位に置く（機体アームがX/Y方向で等しい長さという想定）。
# BODY_HALF_UNITS は同ファイルの \BodyHalf と一致させること。
BODY_HALF_UNITS = 1.15

# Drawn-size shrink (instructor fix, round 3, item 2): the rendered
# body/propellers were overlapping the Roll/Pitch/Yaw arcs, so scale the
# drawn geometry down around the origin by this factor. Only the DRAWING
# shrinks -- the PNG's coordinate extent stays exactly [-3, 3] x [-3, 3]
# units (HALF_EXTENT_UNITS below is unchanged) and mm_per_unit is still
# derived from the true, unscaled geometry, so this must NOT be applied to
# the DEBUG alignment-check axes (those must match the true, unscaled AL
# in imu_axes.tex).
# 描画サイズの縮小（講師修正指示・第3弾・項目2）: レンダリングした機体・
# プロペラがRoll/Pitch/Yaw弧に被っていたため、原点を中心に描画ジオメトリ
# だけをこの倍率で縮小する。PNGの座標範囲は[-3, 3]×[-3, 3]単位のまま
# 変えず（下のHALF_EXTENT_UNITSは不変）、mm_per_unitも実寸の（縮小前の）
# ジオメトリから算出したまま。そのため、DEBUG位置合わせ確認用の軸には
# 適用してはならない（imu_axes.tex の実寸のAL長と一致させる必要がある）。
MODEL_SCALE = 0.7

# Approximate per-part masses in grams, named per the instructor's figures
# (total ~= 37 g), used only to weight each part's own volume centroid --
# NOT as a physically exact mass model.
# 部品ごとの概算質量[g]。講師指示の数値をそのまま名前付き定数にした
# （合計 ≈ 37g）。各部品の体積重心を重み付けするためだけに使い、厳密な
# 質量モデルではない。
MASS_G: dict[str, float] = {
    "frame": 8.0,
    "pcb": 4.0,
    "battery": 9.0,
    "battery_adapter": 1.0,
    "m5stamps3": 3.0,
    "motor_fr": 3.0,
    "motor_fl": 3.0,
    "motor_rl": 3.0,
    "motor_rr": 3.0,
}
TOTAL_MASS_G = sum(MASS_G.values())


@dataclass
class Part:
    name: str
    triangles_mm_landing: np.ndarray  # (N, 3, 3)


def load_parts() -> list[Part]:
    parts = []
    for name in PART_NAMES:
        stl_path = MODEL_DIR / f"{name}.stl"
        triangles_mm, _normals = read_binary_stl(stl_path)
        parts.append(Part(name=name, triangles_mm_landing=triangles_mm))
    return parts


def compute_body_bbox_center_landing(parts: list[Part]) -> np.ndarray:
    """Center of the combined bounding box of all 9 body STL parts, in
    landing coordinates. This is a CRUDE proxy for the center of mass
    (kept only so the report can quote how far it is from the true,
    mass-weighted volume centroid below) -- it ignores part shape and
    mass entirely, e.g. a hollow duct ring and a solid motor can of equal
    bbox contribute equally to it.
    9個の機体STLパーツ全てを合わせた境界ボックスの中心（landing座標系）。
    これは重心のごく粗い代用に過ぎない（真の質量重心とどれだけ離れて
    いるかを報告するためだけに残す）-- 部品の形状・質量を一切考慮しない
    （例えば中空のダクトリングと中身の詰まったモータ缶が同じbboxなら
    等しく寄与してしまう）。
    """
    all_vertices = np.concatenate([p.triangles_mm_landing.reshape(-1, 3) for p in parts], axis=0)
    combined_min = all_vertices.min(axis=0)
    combined_max = all_vertices.max(axis=0)
    return 0.5 * (combined_min + combined_max)


def mesh_volume_and_centroid(triangles_mm: np.ndarray) -> tuple[float, np.ndarray]:
    """Closed-mesh volume and volume centroid via the divergence theorem:
    sum, over all triangles, of the signed tetrahedron (coordinate
    origin, v0, v1, v2). Requires each part's mesh to be closed
    (watertight) with globally consistent vertex winding -- the normal
    STL convention for a single printable solid part. (This is a
    different question from whether an individual triangle's STORED
    normal vector is trustworthy, which this script does not rely on
    anywhere -- winding order, not the stored normal field, is what the
    STL format actually uses to define which side is "outside".)
    The returned centroid is invariant to a globally flipped winding (the
    sign cancels between numerator and denominator), so it stays correct
    even if this particular STL exporter wound every part the other way;
    only a MIX of inconsistent winding within one part would bias it, and
    we have no cheaper way to detect that here.
    発散定理による閉メッシュの体積と体積重心: 全三角形について、四面体
    (座標原点, v0, v1, v2) の符号付き体積を合計する。各パーツのメッシュが
    閉じている（水密）こと、かつ頂点の並びが全体で向きが揃っていることが
    前提（印刷可能な単一の中身入りパーツであれば通常のSTL規約として成立）。
    （これは「個々の三角形に格納された法線ベクトルが信用できるか」とは
    別の話であり、本スクリプトはどこでもそれに頼っていない -- STL形式が
    実際に「外側」を定義するのは格納された法線ではなく頂点の並び順）。
    返す重心は、全体の向きが逆でも（分子・分母で符号が相殺するため）
    不変で正しい。1パーツ内で向きが不揃いに混在している場合のみ誤差が
    乗るが、それを安く検出する方法はここでは用意していない。
    """
    v0 = triangles_mm[:, 0, :]
    v1 = triangles_mm[:, 1, :]
    v2 = triangles_mm[:, 2, :]
    signed_vol6 = np.einsum("ij,ij->i", v0, np.cross(v1, v2))  # 6x signed tetra volume / 符号付き四面体体積の6倍
    total_signed_vol6 = signed_vol6.sum()
    tetra_centroid = (v0 + v1 + v2) / 4.0  # origin apex contributes 0 / 原点頂点の寄与は0
    weighted_centroid_num = (signed_vol6[:, None] * tetra_centroid).sum(axis=0)
    centroid_mm = weighted_centroid_num / total_signed_vol6
    volume_mm3 = abs(total_signed_vol6) / 6.0
    return volume_mm3, centroid_mm


def compute_mass_weighted_center_landing(parts: list[Part]) -> tuple[np.ndarray, dict]:
    """True center-of-mass approximation (instructor fix, round 3): each
    part's own volume centroid (mesh_volume_and_centroid), weighted by
    that part's assigned mass in MASS_G -- the "per-part volume centroid
    weighted by mass" fallback the instructor specified when a full
    volume-centroid x density computation is not available.
    真の重心の近似（講師修正指示・第3弾）: 各パーツ自身の体積重心
    (mesh_volume_and_centroid) を、そのパーツの MASS_G の質量で重み付け
    する。「体積重心×密度が無理なら部品ごとの体積重心を質量で重み付け」
    という講師指定のフォールバックそのもの。
    """
    weighted_sum_mm = np.zeros(3)
    per_part_report: dict[str, tuple[float, np.ndarray, float]] = {}
    for part in parts:
        volume_mm3, centroid_mm = mesh_volume_and_centroid(part.triangles_mm_landing)
        mass_g = MASS_G[part.name]
        weighted_sum_mm += mass_g * centroid_mm
        per_part_report[part.name] = (volume_mm3, centroid_mm, mass_g)
    center_landing = weighted_sum_mm / TOTAL_MASS_G
    return center_landing, per_part_report


def recenter_parts(parts: list[Part], center_landing: np.ndarray) -> None:
    """Shift every part's vertices in place so center_landing becomes the
    new landing-frame origin -- which then maps to FRD (0, 0, 0), the same
    point the TikZ axis arrows originate from.
    全パーツの頂点を、center_landing が landing座標系の新しい原点になる
    ように平行移動する（in place）。この原点はFRD (0, 0, 0) に写り、
    TikZの軸矢印の始点と一致する。
    """
    for part in parts:
        part.triangles_mm_landing = part.triangles_mm_landing - center_landing


def compute_mm_per_unit(parts: list[Part]) -> float:
    """Derive mm-per-TikZ-unit from the actual FRD motor-center positions
    (|X| and |Y| should both equal BODY_HALF_UNITS * mm_per_unit), rather
    than hard-coding a millimetre constant.
    実際のFRDモータ中心位置（|X|と|Y|がどちらも BODY_HALF_UNITS *
    mm_per_unit に等しいはず）から mm/TikZ単位を求める。mm定数の
    ハードコードはしない。
    """
    by_name = {p.name: p for p in parts}
    half_diagonals_mm = []
    for name in MOTOR_NAMES:
        verts_landing = by_name[name].triangles_mm_landing.reshape(-1, 3)
        center_landing = 0.5 * (verts_landing.min(axis=0) + verts_landing.max(axis=0))
        center_frd = landing_to_frd(center_landing[None, :])[0]
        half_diagonals_mm.append(abs(center_frd[0]))  # |X_frd|
        half_diagonals_mm.append(abs(center_frd[1]))  # |Y_frd|
    mean_half_diagonal_mm = float(np.mean(half_diagonals_mm))
    spread = float(np.std(half_diagonals_mm))
    print(
        f"[render_stampfly_iso] motor half-diagonal: mean={mean_half_diagonal_mm:.4f} mm, "
        f"std={spread:.4f} mm (4 motors x |X|,|Y|)"
    )
    return mean_half_diagonal_mm / BODY_HALF_UNITS


def face_normals(triangles: np.ndarray) -> np.ndarray:
    """Per-triangle outward normal from vertex winding (cross product),
    NOT the STL file's stored normal -- this stays correct as long as the
    coordinate transform is a proper rotation (checked above), regardless
    of any inconsistency in the source file's own normals.
    頂点の並び（外積）から求めた三角形ごとの外向き法線。STLファイルに
    格納された法線は使わない。座標変換が真の回転（上で検証済み）である
    限り、元ファイルの法線が多少不整合でも正しく求まる。
    """
    e1 = triangles[:, 1, :] - triangles[:, 0, :]
    e2 = triangles[:, 2, :] - triangles[:, 0, :]
    n = np.cross(e1, e2)
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    return n / norm


# ---------------------------------------------------------------------------
# Propellers: not part of the STL set, so they are drawn procedurally.
# Count / diameter / thickness / mounting height and the blade planform are
# reused verbatim from the real StampFly three.js viewer
# (landing/index.html, the "Real StampFly 3D model" module), which already
# builds true-to-life props because the STL parts have no propeller mesh.
# プロペラ: STL部品には含まれないため、手続き的に描く。枚数・直径・厚み・
# 取付高さ、および羽根の平面形状は、実物のStampFly three.jsビューア
# (landing/index.html の「Real StampFly 3D model」モジュール)の値を
# そのまま流用する（STLにプロペラのメッシュが無いため、同ビューアも
# 手続き的に本物そっくりのプロペラを描いている）。
# ---------------------------------------------------------------------------

# From landing/index.html: `const propRadius = 14.99;` (mm) -- overall
# propeller disk radius, so diameter = 2*PROP_RADIUS_MM ~= 30 mm.
# landing/index.html の `const propRadius = 14.99;` (mm) -- プロペラ円盤の
# 半径。直径 = 2*PROP_RADIUS_MM ≈ 30mm。
PROP_RADIUS_MM = 14.99
BLADE_COUNT = 3  # landing: `for (let i = 0; i < 3; i++)` in makeProp() / 3枚羽根
# landing: `const hubR = propRadius * 0.22;`
HUB_RADIUS_MM = PROP_RADIUS_MM * 0.22
# landing: `const bladeLen = propRadius - hubR * 0.6;`
BLADE_LEN_MM = PROP_RADIUS_MM - HUB_RADIUS_MM * 0.6
# landing: `makeBlade(bladeLen, bladeLen / 2.7, 0.6, 0.38, 0.10)` -- 2nd arg
# "Wm" is the blade's width-scale parameter used throughout its outline.
# landing の `makeBlade(bladeLen, bladeLen / 2.7, 0.6, 0.38, 0.10)` --
# 第2引数 "Wm" は羽根の輪郭全体で使われる幅スケールパラメータ。
BLADE_WIDTH_SCALE_MM = BLADE_LEN_MM / 2.7
# landing: extrusion `depth: thick` = 0.6 mm. Kept only for documentation:
# our render draws each blade as a single flat filled silhouette (matching
# how every other part here is flat-shaded per triangle), so a 0.6 mm
# relief is not visually material at this drawing scale and is not built
# into the 2D geometry.
# landing の押し出し `depth: thick` = 0.6mm。ここでは記録目的でのみ保持。
# 本レンダリングは他の部品と同様に羽根も1枚の塗りつぶしシルエットとして
# 描くため（本図の他パーツもすべて三角形単位のフラットシェーディング）、
# この描画スケールでは0.6mmの厚み方向の凹凸は視覚的に意味を持たず、
# 2D形状には反映しない。
BLADE_THICKNESS_MM = 0.6  # documented only, not drawn / 記録のみ・描画には未使用
# landing: `prop.position.set(pos[0], pos[1] - 1.4, pos[2])` with
# `pos:[..., 7.81, ...]` (same Y for all four propHubs entries) -- i.e. the
# mounting height, in the SAME raw (pre-recentering) landing Y coordinate
# used by the body STL vertices, is 7.81 - 1.4 = 6.41 mm.
# landing の `prop.position.set(pos[0], pos[1] - 1.4, pos[2])`、
# `pos:[..., 7.81, ...]`（4つのpropHubs全てで同じY）-- 取付高さは、機体
# STL頂点と同じ生の（センタリング前の）landing Y座標で 7.81 - 1.4 = 6.41mm。
PROP_MOUNT_Y_LANDING_RAW_MM = 7.81 - 1.4
# landing: `prop.translateX(hubR * 0.6)` in makeProp() -- each blade root
# sits hubR*0.6 out from the hub center, not at the center itself.
# landing の makeProp() 内 `blade.translateX(hubR * 0.6)` -- 各羽根の根元は
# ハブ中心そのものではなく、そこから hubR*0.6 だけ外側にある。
BLADE_ROOT_OFFSET_MM = HUB_RADIUS_MM * 0.6

# Translucent red, per instructor spec (round 2 revision -- originally
# translucent dark grey, changed to red for better contrast against the
# grey motors/ducts).
# 半透明の赤（講師指示・第2弾で変更。当初は半透明濃灰だったが、灰色の
# モータ/ダクトとのコントラストのため赤に変更）。
BLADE_COLOR_RGB = (0.85, 0.15, 0.15)
BLADE_ALPHA = 0.45
BLADE_OUTLINE_SAMPLES = 14  # points per bezier segment / ベジェ曲線1区間あたりの点数

# Hub cylinder (instructor fix, round 4, item 2): the propellers had no
# hub. Dimensions reused verbatim from landing/index.html:
#   `const hubGeo = new THREE.CylinderGeometry(hubR, hubR*0.9, 2.6, 24);`
# (top radius hubR, bottom radius hubR*0.9, height 2.6mm, 24 sides).
# landing also adds a small dome sphere above the cylinder; the
# instructor asked only for "top face and side wall" (a plain cylinder),
# so the dome is not reproduced here.
# ハブ円柱（講師修正指示・第4弾・項目2）: プロペラにハブが無かった。
# 寸法は landing/index.html の
# `const hubGeo = new THREE.CylinderGeometry(hubR, hubR*0.9, 2.6, 24);`
# をそのまま流用（上面半径hubR、底面半径hubR*0.9、高さ2.6mm、24分割）。
# landing は円柱の上にドーム状の半球も追加しているが、講師指示は
# 「上面と側面」（単純な円柱）のみなので、ドームはここでは再現しない。
HUB_TOP_RADIUS_MM = HUB_RADIUS_MM
HUB_BOTTOM_RADIUS_MM = HUB_RADIUS_MM * 0.9
HUB_HEIGHT_MM = 2.6
HUB_SIDES = 24
# Same red family as the blades, but closer to opaque (instructor spec).
# 羽根と同系の赤だが、より不透明寄り（講師指示）。
HUB_COLOR_RGB = BLADE_COLOR_RGB
HUB_ALPHA = 0.9


def _sample_cubic_bezier(p0, p1, p2, p3, n: int) -> np.ndarray:
    """n points along a cubic Bezier curve, from p0 (t=0) to p3 (t=1).
    3次ベジェ曲線上のn点（p0(t=0)からp3(t=1)まで）。"""
    t = np.linspace(0.0, 1.0, n)
    mt = 1.0 - t
    x = mt**3 * p0[0] + 3 * mt**2 * t * p1[0] + 3 * mt * t**2 * p2[0] + t**3 * p3[0]
    y = mt**3 * p0[1] + 3 * mt**2 * t * p1[1] + 3 * mt * t**2 * p2[1] + t**3 * p3[1]
    return np.stack([x, y], axis=1)


def _sample_quadratic_bezier(p0, p1, p2, n: int) -> np.ndarray:
    """n points along a quadratic Bezier curve, from p0 (t=0) to p2 (t=1).
    2次ベジェ曲線上のn点（p0(t=0)からp2(t=1)まで）。"""
    t = np.linspace(0.0, 1.0, n)
    mt = 1.0 - t
    x = mt**2 * p0[0] + 2 * mt * t * p1[0] + t**2 * p2[0]
    y = mt**2 * p0[1] + 2 * mt * t * p1[1] + t**2 * p2[1]
    return np.stack([x, y], axis=1)


def build_blade_outline_mm() -> np.ndarray:
    """Reconstruct the blade planform outline (local span/chord mm, span
    s in [0, BLADE_LEN_MM] along local +u, chord w along local +-v) by
    replaying landing/index.html's makeBlade() path commands exactly
    (moveTo/bezierCurveTo/quadraticCurveTo/closePath), with twist omitted
    -- the instructor confirmed blade rotation direction/orientation does
    not matter for this static image, and a flat (untwisted) silhouette is
    the natural simplification for a flat-shaded 2D drawing. Returns an
    (M, 2) closed-polygon outline in (s, w) mm.
    羽根の平面形状（局所スパン方向s∈[0,BLADE_LEN_MM]、局所コード方向w）を、
    landing/index.html の makeBlade() のパス命令
    （moveTo/bezierCurveTo/quadraticCurveTo/closePath）をそのまま再現して
    復元する。ねじれ(twist)は省略 -- 講師確認により、この静止画では羽根の
    回転方向・向きは問わないため、フラットシェーディングの2D図としては
    ねじれ無し（平面）の輪郭が自然な単純化となる。(s, w) mm の閉多角形
    (M, 2) を返す。
    """
    L = BLADE_LEN_MM
    Wm = BLADE_WIDTH_SCALE_MM
    n = BLADE_OUTLINE_SAMPLES

    p_root = (0.0, 0.20 * Wm)
    p_mid_hi = (0.86 * L, 0.30 * Wm)
    p_tip = (L, 0.0)
    p_mid_lo = (0.86 * L, -0.20 * Wm)
    p_root_back = (0.0, -0.24 * Wm)

    seg1 = _sample_cubic_bezier(p_root, (0.40 * L, 0.50 * Wm), (0.65 * L, 0.50 * Wm), p_mid_hi, n)
    seg2 = _sample_quadratic_bezier(p_mid_hi, (L, 0.16 * Wm), p_tip, n)
    seg3 = _sample_quadratic_bezier(p_tip, (L, -0.12 * Wm), p_mid_lo, n)
    seg4 = _sample_cubic_bezier(p_mid_lo, (0.65 * L, -0.58 * Wm), (0.40 * L, -0.58 * Wm), p_root_back, n)
    # closePath(): straight line back from p_root_back to p_root, left for
    # PolyCollection's implicit polygon closing (no explicit sample needed).
    # closePath(): p_root_back から p_root への直線。PolyCollection が
    # 多角形を自動的に閉じるため、明示的な点は不要。
    return np.concatenate([seg1, seg2, seg3, seg4], axis=0)


def _motor_hub_point_frd_units(part: Part, units_per_mm: float) -> np.ndarray:
    """The propeller mounting point for one motor part: that motor's own
    (X, Z) center in the horizontal FRD plane, at the shared
    PROP_MOUNT_Y_LANDING_RAW_MM height, converted to FRD TikZ units. Used
    by both the blades and the hub cylinder so they share one definition
    of "where this motor's propeller is centered."
    1つのモータ部品に対するプロペラ取付点: そのモータ自身の水平FRD面内
    (X, Z)中心を、共通のPROP_MOUNT_Y_LANDING_RAW_MM高さで、FRD TikZ単位に
    変換したもの。羽根とハブ円柱の両方で使い、「このモータのプロペラは
    どこが中心か」の定義を1箇所にまとめる。
    """
    verts_landing = part.triangles_mm_landing.reshape(-1, 3)
    motor_center_landing = 0.5 * (verts_landing.min(axis=0) + verts_landing.max(axis=0))
    hub_point_landing = np.array(
        [motor_center_landing[0], PROP_MOUNT_Y_LANDING_RAW_MM, motor_center_landing[2]]
    )
    return landing_to_frd(hub_point_landing[None, :])[0] * units_per_mm


def build_propeller_polygons_frd_units(
    parts: list[Part], units_per_mm: float
) -> tuple[list[np.ndarray], list[tuple[float, float, float, float]]]:
    """Build the 12 blade polygons (4 motors x 3 blades), each as an
    (M, 3) array of FRD points in TikZ units, plus a matching translucent
    RGBA color list. Blades lie flat in the body's horizontal (FRD X-Y)
    plane at the propeller's mounting height, fanned out at 3 arbitrary
    120-degree-apart angles per motor (orientation is not meaningful for a
    static image, as confirmed by the instructor).
    12枚の羽根ポリゴン（モータ4個×羽根3枚）を、それぞれFRD点(TikZ単位)の
    (M, 3)配列として構築し、対応する半透明RGBA色のリストも返す。羽根は
    機体の水平面（FRD X-Y平面）内、プロペラ取付高さの位置に平らに置き、
    モータごとに（静止画では向きに意味が無いと講師確認済みの）任意の
    120°間隔3方向へ展開する。
    """
    by_name = {p.name: p for p in parts}
    blade_outline_mm = build_blade_outline_mm()  # (M, 2): (s, w)

    polygons: list[np.ndarray] = []
    colors: list[tuple[float, float, float, float]] = []
    for motor_name in MOTOR_NAMES:
        hub_point_frd_units = _motor_hub_point_frd_units(by_name[motor_name], units_per_mm)

        for blade_index in range(BLADE_COUNT):
            angle = 2.0 * np.pi * blade_index / BLADE_COUNT
            span_dir = np.array([np.cos(angle), np.sin(angle)])  # in FRD (X, Y) plane
            chord_dir = np.array([-np.sin(angle), np.cos(angle)])

            s = blade_outline_mm[:, 0] + BLADE_ROOT_OFFSET_MM  # radial distance from hub center
            w = blade_outline_mm[:, 1]
            offset_xy_mm = s[:, None] * span_dir[None, :] + w[:, None] * chord_dir[None, :]
            offset_xy_units = offset_xy_mm * units_per_mm

            blade_xy_units = hub_point_frd_units[:2][None, :] + offset_xy_units
            blade_z_units = np.full((blade_xy_units.shape[0], 1), hub_point_frd_units[2])
            polygons.append(np.concatenate([blade_xy_units, blade_z_units], axis=1))
            colors.append((*BLADE_COLOR_RGB, BLADE_ALPHA))
    return polygons, colors


def build_hub_polygons_frd_units(
    parts: list[Part], units_per_mm: float
) -> tuple[list[np.ndarray], list[tuple[float, float, float, float]]]:
    """Build the 4 hub cylinders (one per motor: HUB_SIDES side-wall
    quads + 1 top cap n-gon each), as FRD-unit polygons with shaded RGBA
    colors (two-sided |normal . light|, same lighting model as the rest
    of the render, so the hub reads as a solid cylinder rather than a
    flat red disk).
    4個のハブ円柱（モータ1個につきHUB_SIDES枚の側面クアッド + 上面の
    n角形1枚）を、FRD単位のポリゴンとしてシェーディング済みRGBA色
    付きで構築する（両面|法線・光源|、本レンダリングの他部分と同じ
    照明モデルを使うため、ハブは平らな赤い円ではなく立体的な円柱として
    見える）。
    """
    by_name = {p.name: p for p in parts}
    half_height_units = (HUB_HEIGHT_MM / 2.0) * units_per_mm
    top_radius_units = HUB_TOP_RADIUS_MM * units_per_mm
    bottom_radius_units = HUB_BOTTOM_RADIUS_MM * units_per_mm
    angles = np.linspace(0.0, 2.0 * np.pi, HUB_SIDES, endpoint=False)
    cos_a, sin_a = np.cos(angles), np.sin(angles)

    def shade(normal_frd: np.ndarray) -> tuple[float, float, float, float]:
        brightness = float(np.clip(AMBIENT + DIFFUSE * abs(normal_frd @ LIGHT_DIR), 0.0, 1.0))
        rgb = np.clip(np.array(HUB_COLOR_RGB) * brightness, 0.0, 1.0)
        return (*rgb, HUB_ALPHA)

    polygons: list[np.ndarray] = []
    colors: list[tuple[float, float, float, float]] = []
    for motor_name in MOTOR_NAMES:
        hub_center_frd_units = _motor_hub_point_frd_units(by_name[motor_name], units_per_mm)
        # "Up" (toward the top face/dome, away from the motor can) is the
        # -Z_frd direction, since Z_frd = -y_landing (down positive).
        # 「上」（上面・ドーム側、モータ缶と反対側）は -Z_frd 方向
        # （Z_frd = -y_landing、下が正のため）。
        top_z = hub_center_frd_units[2] - half_height_units
        bottom_z = hub_center_frd_units[2] + half_height_units

        top_ring = np.stack(
            [
                hub_center_frd_units[0] + top_radius_units * cos_a,
                hub_center_frd_units[1] + top_radius_units * sin_a,
                np.full(HUB_SIDES, top_z),
            ],
            axis=1,
        )
        bottom_ring = np.stack(
            [
                hub_center_frd_units[0] + bottom_radius_units * cos_a,
                hub_center_frd_units[1] + bottom_radius_units * sin_a,
                np.full(HUB_SIDES, bottom_z),
            ],
            axis=1,
        )

        polygons.append(top_ring)
        colors.append(shade(np.array([0.0, 0.0, -1.0])))  # flat horizontal cap, normal along -Z (up) / 水平な上面、法線は-Z(上)方向

        for i in range(HUB_SIDES):
            j = (i + 1) % HUB_SIDES
            quad = np.stack([top_ring[i], top_ring[j], bottom_ring[j], bottom_ring[i]], axis=0)
            normal = np.cross(quad[1] - quad[0], quad[2] - quad[0])
            norm_len = np.linalg.norm(normal)
            if norm_len > 0:
                normal = normal / norm_len
            polygons.append(quad)
            colors.append(shade(normal))
    return polygons, colors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--debug",
        action="store_true",
        help=(
            "Write a temporary alignment-check PNG (DEBUG_OUT_PNG) with an "
            "origin crosshair and thin, unscaled X/Y/Z axis lines overlaid, "
            "instead of the normal clean OUT_PNG. Never referenced by "
            "imu_axes.tex; for a one-off visual overlay check only. / "
            "通常の重ね書きのないOUT_PNGの代わりに、原点の十字と細い（縮小なし "
            "の）X/Y/Z軸線を重ねた一時的な位置合わせ確認用PNG(DEBUG_OUT_PNG) "
            "を出力する。imu_axes.texからは参照されない。1回限りの目視重ね "
            "合わせ確認専用。"
        ),
    )
    args = parser.parse_args()

    parts = load_parts()

    # ---- Recenter on the mass-weighted volume centroid (instructor fix,
    # round 3): each part's true volume centroid (divergence theorem),
    # weighted by its assigned mass -- NOT the bounding-box center used
    # previously. Done before anything else is derived from these
    # coordinates so mm-per-unit, motor positions, and the propeller
    # mounting point all stay consistent with the new origin.
    # ---- 質量で重み付けした体積重心で原点を取り直す（講師修正指示・
    # 第3弾）: 各パーツの真の体積重心（発散定理）を、割り当てた質量で
    # 重み付けする -- 以前使っていた境界ボックス中心ではない。以降の全て
    # （mm/単位スケール、モータ位置、プロペラ取付点）がこの新しい原点と
    # 整合するよう、他の計算より先に行う。
    body_center_landing, per_part_volume_report = compute_mass_weighted_center_landing(parts)
    bbox_center_landing = compute_body_bbox_center_landing(parts)

    print(f"[render_stampfly_iso] per-part volume centroid (landing frame, mm), TOTAL_MASS_G={TOTAL_MASS_G:.1f}g:")
    for name in PART_NAMES:
        vol_mm3, centroid_mm, mass_g = per_part_volume_report[name]
        print(
            f"  {name:16s} mass={mass_g:4.1f}g  volume={vol_mm3:9.1f}mm^3  "
            f"centroid=({centroid_mm[0]:+8.3f},{centroid_mm[1]:+8.3f},{centroid_mm[2]:+8.3f})"
        )
    print(
        "[render_stampfly_iso] mass-weighted center of mass (landing frame, mm): "
        f"x={body_center_landing[0]:+.4f} y={body_center_landing[1]:+.4f} z={body_center_landing[2]:+.4f}"
    )
    print(
        "[render_stampfly_iso] bbox center for comparison (landing frame, mm): "
        f"x={bbox_center_landing[0]:+.4f} y={bbox_center_landing[1]:+.4f} z={bbox_center_landing[2]:+.4f}"
    )
    # Report the CoM-vs-bbox difference in FRD axes (X=forward, Y=right,
    # Z=down), as requested, not raw landing axes.
    # CoMとbbox中心の差は、指示通りFRD軸（X=前,Y=右,Z=下）で報告する
    # （landingの生の軸ではない）。
    diff_frd_mm = landing_to_frd((body_center_landing - bbox_center_landing)[None, :])[0]
    print(
        "[render_stampfly_iso] CoM - bbox_center, in FRD mm: "
        f"X(fwd)={diff_frd_mm[0]:+.4f} Y(right)={diff_frd_mm[1]:+.4f} Z(down)={diff_frd_mm[2]:+.4f}"
    )

    recenter_parts(parts, body_center_landing)
    # The propeller mounting height (reused from landing/index.html) was
    # defined in the SAME raw, pre-recentering landing Y coordinate as the
    # body STL vertices, so it must be shifted by the same amount to stay
    # consistent with the now-recentered body.
    # プロペラ取付高さ（landing/index.html から流用）は、機体STL頂点と
    # 同じ「センタリング前の生の」landing Y座標で定義されているため、
    # センタリング後の機体と整合させるには同じ量だけシフトする必要がある。
    global PROP_MOUNT_Y_LANDING_RAW_MM
    PROP_MOUNT_Y_LANDING_RAW_MM -= body_center_landing[1]

    mm_per_unit = compute_mm_per_unit(parts)
    units_per_mm = 1.0 / mm_per_unit
    print(f"[render_stampfly_iso] mm_per_unit={mm_per_unit:.4f} (1 TikZ unit = {mm_per_unit:.2f} mm)")

    all_tri_units: list[np.ndarray] = []
    all_colors: list[tuple[float, float, float]] = []
    for part in parts:
        tri_frd_mm = landing_to_frd(part.triangles_mm_landing.reshape(-1, 3)).reshape(-1, 3, 3)
        tri_frd_units = tri_frd_mm * units_per_mm
        all_tri_units.append(tri_frd_units)
        base_color = PART_COLORS[part.name]
        all_colors.extend([base_color] * len(tri_frd_units))

    triangles_units = np.concatenate(all_tri_units, axis=0)
    base_colors = np.array(all_colors)

    normals = face_normals(triangles_units)

    # No backface culling (instructor fix, round 2): the STL normals are
    # not trustworthy for deciding "front vs. back" (some faces were
    # dropping out / showing through as transparent gaps), so instead of
    # trying to classify and discard "back" faces, EVERY triangle is kept
    # and drawn; occlusion is now handled per-pixel by the Z-buffer below
    # (round 5), which does not depend on winding/normal direction at all
    # -- a correctly-wound-or-not triangle wins the Z-test purely on
    # depth, so keeping every triangle is both simpler and harmless.
    # face_normals() above already recomputes each normal from vertex
    # winding (cross product), never from the STL file's own (unreliable)
    # stored normal.
    # 背面カリングを廃止（講師修正指示・第2弾）: STLの法線は「表か裏か」の
    # 判定に使えない（一部の面が抜けて透けて見えていた）。「裏」面を分類・
    # 除外しようとするのではなく、全ての三角形をそのまま残して描画し、
    # 前後関係は下の画素単位Zバッファ（第5弾）だけで処理する -- Zバッファは
    # 法線の向きに一切依存しない（三角形の巻き順が正しくてもそうでなくても
    # 深度だけでZテストに勝敗が決まる）ため、全三角形を残すのは単純かつ
    # 無害。face_normals() は既に頂点の並び（外積）から法線を再計算して
    # おり、STLファイル自身の（信頼できない）法線は使っていない。

    # Two-sided flat shading: use |normal . light| so a triangle is lit
    # the same whether its (arbitrary-orientation) normal points toward
    # or away from the light -- unchanged since round 2, still correct
    # under Z-buffered rendering.
    # 両面フラットシェーディング: |法線・光源方向| を使い、法線が
    # （向きが不定な）どちら向きでも同じ明るさになるようにする（第2弾から
    # 不変。Zバッファ描画になっても引き続き妥当）。
    diffuse_term = np.abs(normals @ LIGHT_DIR)
    brightness = np.clip(AMBIENT + DIFFUSE * diffuse_term, 0.0, 1.0)
    mesh_shaded_colors = np.clip(base_colors * brightness[:, None], 0.0, 1.0)

    # ---- Propellers: 12 flat translucent blade polygons (unchanged
    # geometry/color from round 4).
    # ---- プロペラ: 12枚の平らな半透明羽根ポリゴン（幾何・色は第4弾から
    # 不変）。
    blade_polygons_units, blade_colors_rgba = build_propeller_polygons_frd_units(parts, units_per_mm)

    # ---- Hub cylinders (unchanged geometry/color from round 4).
    # ---- ハブ円柱（幾何・色は第4弾から不変）。
    hub_polygons_units, hub_colors_rgba = build_hub_polygons_frd_units(parts, units_per_mm)

    # ---- Shrink the drawn body/propellers around the origin -- applied
    # to the raw 3D point arrays (mesh triangles, blade polygons, hub
    # polygons) BEFORE screen projection and depth computation, so both
    # stay mutually consistent (a uniform positive scale about the origin
    # preserves relative depth ordering along any fixed direction, so
    # this is safe to do before computing per-vertex Z-buffer depths
    # below). The PNG's coordinate extent stays exactly [-3, 3] x [-3, 3]
    # units regardless (HALF_EXTENT_UNITS unchanged), and this must NOT be
    # applied to the DEBUG alignment-check axes (drawn separately, after
    # compositing, at the true unscaled AL length matching imu_axes.tex).
    # ---- 機体・プロペラの描画を原点中心に縮小する -- 画面投影・深度計算の
    # 前に、生の3D点配列（メッシュ三角形・羽根ポリゴン・ハブポリゴン）に
    # 適用する（原点を中心とした一様な正のスケーリングは、任意の固定方向
    # に沿った相対的な深度順序を保つため、下のZバッファ深度を計算する前に
    # 行っても安全）。PNGの座標範囲は[-3, 3]×[-3, 3]単位のまま変わらない
    # （HALF_EXTENT_UNITSは不変）。DEBUG位置合わせ確認用の軸（合成後に別途
    # 描く、imu_axes.texと一致する実寸のAL長）には適用してはならない。
    triangles_units = triangles_units * MODEL_SCALE
    blade_polygons_units = [poly * MODEL_SCALE for poly in blade_polygons_units]
    hub_polygons_units = [poly * MODEL_SCALE for poly in hub_polygons_units]

    # ---- Triangulate the non-triangular translucent polygons (blade
    # paddles, hub top caps, hub side quads) into actual triangles, fanned
    # from each polygon's own centroid, so the same per-triangle
    # rasterizer handles everything uniformly.
    # ---- 三角形でない半透明ポリゴン（羽根パドル・ハブ上面・ハブ側面
    # クアッド）を、それぞれの重心から扇状に実三角形へ分割する。同じ
    # 三角形単位のラスタライザで統一的に扱うため。
    translucent_triangles: list[np.ndarray] = []
    translucent_colors_rgba: list[tuple[float, float, float, float]] = []
    for poly, color in zip(blade_polygons_units + hub_polygons_units, blade_colors_rgba + hub_colors_rgba):
        for tri in triangulate_polygon_fan(poly):
            translucent_triangles.append(tri)
            translucent_colors_rgba.append(color)

    # ---- Per-pixel Z-buffer rasterization (instructor fix, round 5):
    # replaces the old whole-triangle painter's algorithm, which produced
    # visible light/dark striping on parts made of many small, similarly-
    # angled triangles (the motor cans) -- see rasterize_triangle()'s
    # docstring for why. Pass 1 draws the opaque body mesh with full
    # depth test + depth write. Pass 2 draws the translucent hub+blade
    # triangles, sorted far-to-near for correct alpha-over blending,
    # depth-TESTED against the (now frozen) opaque Z-buffer but not
    # depth-written, so a blade/hub triangle hidden behind the opaque body
    # is correctly skipped without ever occluding another translucent
    # triangle via the Z-buffer.
    # ---- 画素単位Zバッファでのラスタライズ（講師修正指示・第5弾）:
    # 従来の三角形単位の画家アルゴリズムを置き換える（モータ缶のような
    # 似た向きの小さな三角形が多い部分で明暗の縞が出ていた理由は
    # rasterize_triangle()のdocstring参照）。パス1で不透明な機体メッシュを
    # 完全な深度テスト＋深度書き込みで描く。パス2で半透明のハブ＋羽根の
    # 三角形を、正しいアルファオーバー合成のため遠い順にソートしてから、
    # （今や固定された）不透明パスのZバッファに対して深度テストしつつ
    # 深度は書き込まずに描く。これにより、不透明な機体の陰に隠れる羽根/
    # ハブの三角形は正しくスキップされ、かつ半透明三角形同士がZバッファで
    # 互いを隠すことはない。
    t_start = time.perf_counter()

    depth_buffer = np.full((CANVAS_PIXELS, CANVAS_PIXELS), np.inf)
    color_buffer = np.zeros((CANVAS_PIXELS, CANVAS_PIXELS, 3))  # premultiplied color / 乗算済み色
    alpha_buffer = np.zeros((CANVAS_PIXELS, CANVAS_PIXELS))

    n_opaque_tris = len(triangles_units)
    for i in range(n_opaque_tris):
        tri = triangles_units[i]
        screen_px = data_to_pixel(project(tri))
        depth3 = tri @ F_HAT
        rasterize_triangle(
            screen_px,
            depth3,
            mesh_shaded_colors[i],
            1.0,
            depth_buffer=depth_buffer,
            color_buffer=color_buffer,
            alpha_buffer=alpha_buffer,
            depth_test_buffer=depth_buffer,
            write_depth=True,
        )

    # Sort the translucent set far-to-near for correct back-to-front
    # alpha-over compositing (see rasterize_triangle()'s else-branch).
    # 半透明の集合を、正しい遠い順のアルファオーバー合成のために遠い順へ
    # ソートする（rasterize_triangle()のelse節を参照）。
    translucent_depth = np.array([tri.mean(axis=0) @ F_HAT for tri in translucent_triangles])
    translucent_order = np.argsort(-translucent_depth)  # descending depth = far first / 深度降順=遠い順
    n_translucent_tris = len(translucent_triangles)
    for i in translucent_order:
        tri = translucent_triangles[i]
        color_rgba = translucent_colors_rgba[i]
        screen_px = data_to_pixel(project(tri))
        depth3 = tri @ F_HAT
        rasterize_triangle(
            screen_px,
            depth3,
            np.array(color_rgba[:3]),
            color_rgba[3],
            depth_buffer=depth_buffer,
            color_buffer=color_buffer,
            alpha_buffer=alpha_buffer,
            depth_test_buffer=depth_buffer,
            write_depth=False,
        )

    elapsed_s = time.perf_counter() - t_start
    print(
        f"[render_stampfly_iso] rasterized {n_opaque_tris} opaque + {n_translucent_tris} translucent "
        f"triangles into {CANVAS_PIXELS}x{CANVAS_PIXELS}px in {elapsed_s:.1f}s"
    )

    # ---- Un-premultiply and pack into an RGBA uint8 image (PIL, not
    # matplotlib -- instructor fix, round 5, item 1). Fully transparent
    # pixels (alpha=0) get an arbitrary safe RGB (division guarded by
    # ALPHA_EPS) since they are invisible anyway.
    # ---- 乗算済み色を通常色へ戻し、RGBA uint8画像へ詰める（PILを使用、
    # matplotlibは使わない -- 講師修正指示・第5弾・項目1）。完全に透明な
    # 画素（alpha=0）は、どのみち見えないため安全な適当なRGB値になる
    # （ALPHA_EPSで0除算を防ぐ）。
    ALPHA_EPS = 1e-8
    alpha_safe = np.where(alpha_buffer > ALPHA_EPS, alpha_buffer, 1.0)
    straight_rgb = np.clip(color_buffer / alpha_safe[:, :, None], 0.0, 1.0)
    rgba_u8 = np.zeros((CANVAS_PIXELS, CANVAS_PIXELS, 4), dtype=np.uint8)
    rgba_u8[:, :, :3] = np.round(straight_rgb * 255.0).astype(np.uint8)
    rgba_u8[:, :, 3] = np.round(np.clip(alpha_buffer, 0.0, 1.0) * 255.0).astype(np.uint8)
    image = Image.fromarray(rgba_u8, mode="RGBA")

    if args.debug:
        # ---- DEBUG-only alignment-check overlay: origin crosshair + thin
        # projected X/Y/Z axis lines, at the SAME unscaled length
        # AL_MATCH_TIKZ_UNITS as \AL in imu_axes.tex, drawn in bright
        # colors distinct from TikZ's own (thicker) red/green/blue arrows,
        # via plain PIL line drawing on top of the already-composited
        # image (no Z-buffer/depth interaction -- this is a flat 2D
        # overlay). Temporarily copy this file over OUT_PNG and rebuild
        # the standalone imu_axes.pdf: a correct projection/scale/
        # placement makes each thin line run exactly down the centerline
        # of the matching thick TikZ arrow, and the crosshair sit exactly
        # on the arrows' common origin.
        # ---- DEBUG専用の位置合わせ確認オーバーレイ: 原点の十字と、
        # imu_axes.tex の \AL と同じ実寸（縮小なし）の長さ
        # AL_MATCH_TIKZ_UNITS で投影したX/Y/Z軸の細線を、TikZ自身の
        # （より太い）赤緑青の矢印とは異なる明るい色で、合成済み画像の上に
        # PILの単純な線描画で重ねる（Zバッファ/深度とは無関係 -- 平面的な
        # 2Dオーバーレイ）。このファイルを一時的にOUT_PNGへコピーして
        # standaloneのimu_axes.pdfを再ビルドすれば、投影・スケール・配置が
        # 正しければ、各細線は対応する太いTikZ矢印の中心線をちょうど通り、
        # 十字は矢印群の共通原点にちょうど重なるはずである。
        draw = ImageDraw.Draw(image)
        AL_MATCH_TIKZ_UNITS = 4.5  # must equal \AL in imu_axes.tex / imu_axes.tex の \AL と同じ値
        CROSSHAIR_HALF_UNITS = 0.15
        debug_axis_specs = [
            (np.array([1.0, 0.0, 0.0]), (255, 215, 0, 255)),  # X: gold / 金
            (np.array([0.0, 1.0, 0.0]), (255, 0, 255, 255)),  # Y: magenta / マゼンタ
            (np.array([0.0, 0.0, 1.0]), (0, 255, 255, 255)),  # Z: cyan / シアン
        ]
        for axis_dir_frd, line_color in debug_axis_specs:
            tip_units = axis_dir_frd * AL_MATCH_TIKZ_UNITS
            seg_px = data_to_pixel(project(np.stack([np.zeros(3), tip_units])))
            draw.line([tuple(seg_px[0]), tuple(seg_px[1])], fill=line_color, width=2)
        origin_px = data_to_pixel(project(np.zeros((1, 3))))[0]
        half_px = CROSSHAIR_HALF_UNITS / (2.0 * HALF_EXTENT_UNITS) * CANVAS_PIXELS
        draw.line(
            [(origin_px[0] - half_px, origin_px[1]), (origin_px[0] + half_px, origin_px[1])],
            fill=(50, 255, 50, 255),
            width=2,
        )
        draw.line(
            [(origin_px[0], origin_px[1] - half_px), (origin_px[0], origin_px[1] + half_px)],
            fill=(50, 255, 50, 255),
            width=2,
        )
        image.save(DEBUG_OUT_PNG)
        print(f"[render_stampfly_iso] DEBUG: wrote {DEBUG_OUT_PNG} (NOT used by imu_axes.tex)")
    else:
        image.save(OUT_PNG)
        print(f"[render_stampfly_iso] wrote {OUT_PNG}")

    # ---- Sanity check: projected motor centres vs. TikZ's own mapping ----
    # ---- 検証: 投影したモータ中心位置とTikZ側のマッピングとの比較 ----
    by_name = {p.name: p for p in parts}
    print("[render_stampfly_iso] motor center comparison (screen units, x right / y up on page):")
    tikz_signs = {  # (sign_x, sign_y) for TikZ's (+-BodyHalf, +-BodyHalf, 0) corners
        "motor_fr": (+1, +1),
        "motor_fl": (+1, -1),
        "motor_rl": (-1, -1),
        "motor_rr": (-1, +1),
    }
    for name in MOTOR_NAMES:
        verts_landing = by_name[name].triangles_mm_landing.reshape(-1, 3)
        center_landing = 0.5 * (verts_landing.min(axis=0) + verts_landing.max(axis=0))
        center_frd_units = landing_to_frd(center_landing[None, :])[0] * units_per_mm
        screen_actual = project(center_frd_units[None, :])[0]

        sx, sy = tikz_signs[name]
        tikz_point_frd_units = np.array([sx * BODY_HALF_UNITS, sy * BODY_HALF_UNITS, 0.0])
        screen_tikz = project(tikz_point_frd_units[None, :])[0]

        print(
            f"  {name:10s} actual(FRD units)=({center_frd_units[0]:+.3f},{center_frd_units[1]:+.3f},{center_frd_units[2]:+.3f})"
            f"  screen_actual=({screen_actual[0]:+.3f},{screen_actual[1]:+.3f})"
            f"  screen_tikz(z=0 plate)=({screen_tikz[0]:+.3f},{screen_tikz[1]:+.3f})"
        )


if __name__ == "__main__":
    main()
