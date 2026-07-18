#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate the StampFly Flasher app icon assets.
StampFly Flasher アプリアイコン一式を生成する。

Purpose / 目的
--------------
Deterministically (no randomness) draws a 1024px app icon -- a rounded
square, dark navy background with a white "X-quad" drone motif (four
rotor circles connected by an X of arms, a hub, and a lightning bolt on
the hub) -- and exports it as the icon formats each OS packaging step
needs:

    icon_1024.png   Source PNG (also used to derive everything below)
    icon_256.png    Standalone 256px PNG (used by the Linux .desktop entry)
    icon.ico        Windows icon, multi-size (16/24/32/48/64/128/256)
    icon.icns       macOS icon (built via `iconutil`, macOS-only)

決定論的（乱数不使用）に1024px角のアプリアイコン——角丸スクエア、
濃紺背景に白の「Xクアッド」ドローンモチーフ（4つのロータ円をXアームで
結び、中央ハブに稲妻）を描き、各OSのパッケージングが必要とする
アイコン形式で書き出す。

Usage / 使い方
--------------
    pip install Pillow
    python3 tools/flasher_gui/assets/gen_icon.py

Run this script and commit its output (icon_1024.png / icon_256.png /
icon.ico / icon.icns) -- release CI does NOT regenerate these at build
time, it just points PyInstaller's --icon at the committed files.
icon.icns generation requires macOS's `iconutil` and is skipped (with a
warning) on other platforms; regenerate it on a Mac if this script's
icon geometry ever changes.

このスクリプトを実行し、出力(icon_1024.png / icon_256.png / icon.ico /
icon.icns)をコミットすること -- リリースCIはビルド時にこれらを再生成
せず、コミット済みファイルを PyInstaller の --icon に渡すだけである。
icon.icns の生成には macOS の `iconutil` が必要で、他プラットフォームでは
警告付きでスキップされる。アイコンの形状を変更した場合は Mac 上で
再生成すること。
"""

import math
import platform
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw

# ---------------------------------------------------------------------------
# Constants (no magic numbers inlined below this block)
# 定数（この節より下にマジックナンバーを埋め込まない）
# ---------------------------------------------------------------------------

OUTPUT_DIR = Path(__file__).resolve().parent

# Canvas / background geometry.
CANVAS_SIZE_PX = 1024
CENTER_PX = CANVAS_SIZE_PX / 2
# macOS "squircle" icons round their corners by roughly 18% of the
# canvas size; 0.176 lands close to that without a full superellipse.
# macOS の「squircle」アイコンは角丸半径がキャンバスの約18%。完全な
# 超楕円までは実装せず、近い比率(0.176)の単純な角丸四角で近似する。
CORNER_RADIUS_PX = round(CANVAS_SIZE_PX * 0.176)

# Background color: a darkened version of the "sfblue" used throughout
# the StampFly slide decks (RGB 0,120,200 -- see
# docs/workshop/slides/tikz/comp_filter.tex), scaled down for a "濃紺"
# (dark navy) app-icon background rather than the brighter slide accent.
# 背景色: StampFly スライド共通の "sfblue"(RGB 0,120,200 --
# docs/workshop/slides/tikz/comp_filter.tex 参照)を暗くしたもの。
# スライドの明るいアクセント用途ではなく「濃紺」のアプリアイコン背景に
# 合わせて縮小する。
SFBLUE_RGB = (0, 120, 200)
BACKGROUND_DARKEN_FACTOR = 0.35
BACKGROUND_COLOR = tuple(round(c * BACKGROUND_DARKEN_FACTOR) for c in SFBLUE_RGB) + (255,)
MOTIF_COLOR = (255, 255, 255, 255)  # white

# X-quad motif geometry (StampFly is an X-frame quadcopter): four rotor
# circles at the diagonal corners, connected to a center hub by thick
# arms, matching the vehicle's real X layout.
# Xクアッドモチーフの形状（StampFlyはXフレーム機）: 対角4隅にロータ円、
# 太いアームで中央ハブへ接続。実機のX配置に合わせる。
ARM_REACH_PX = CANVAS_SIZE_PX * 0.335  # center-to-rotor-center distance
ARM_WIDTH_PX = round(CANVAS_SIZE_PX * 0.042)
ROTOR_RADIUS_PX = round(CANVAS_SIZE_PX * 0.095)
ROTOR_OUTLINE_WIDTH_PX = round(CANVAS_SIZE_PX * 0.014)
HUB_RADIUS_PX = round(CANVAS_SIZE_PX * 0.135)
HUB_OUTLINE_WIDTH_PX = round(CANVAS_SIZE_PX * 0.012)

# Lightning bolt polygon, expressed as fractions of HUB_RADIUS_PX offsets
# from the canvas center so it scales cleanly with the hub. Points trace
# a classic angular bolt shape (Z-like zigzag).
# 稲妻の多角形。キャンバス中心からの HUB_RADIUS_PX 比のオフセットで表現し、
# ハブとの拡大縮小が一致するようにする。古典的な角ばった稲妻(Z字状)。
BOLT_POINTS_FRACTIONS = [
    (0.16, -0.62),
    (-0.30, 0.02),
    (-0.02, 0.02),
    (-0.16, 0.62),
    (0.30, -0.06),
    (0.02, -0.06),
]

# Raster export sizes.
ICON_256_SIZE_PX = 256
ICO_SIZES_PX = [16, 24, 32, 48, 64, 128, 256]
# macOS .iconset naming: (filename, pixel size). "@2x" entries reuse the
# next size up's pixels (Apple's documented iconset convention).
# macOS .iconset の命名規則: (ファイル名, ピクセルサイズ)。"@2x" は
# 1段階大きいサイズのピクセルを流用する(Appleの公式iconset規約)。
ICONSET_ENTRIES = [
    ("icon_16x16.png", 16),
    ("icon_16x16@2x.png", 32),
    ("icon_32x32.png", 32),
    ("icon_32x32@2x.png", 64),
    ("icon_128x128.png", 128),
    ("icon_128x128@2x.png", 256),
    ("icon_256x256.png", 256),
    ("icon_256x256@2x.png", 512),
    ("icon_512x512.png", 512),
    ("icon_512x512@2x.png", 1024),
]

RESAMPLE_FILTER = Image.LANCZOS


def build_base_image() -> Image.Image:
    """Draw the 1024px master icon image (rounded square + X-quad + bolt).
    1024px角のマスターアイコン画像(角丸スクエア+Xクアッド+稲妻)を描く。"""
    image = Image.new("RGBA", (CANVAS_SIZE_PX, CANVAS_SIZE_PX), (0, 0, 0, 0))
    draw = ImageDraw.Draw(image)

    draw.rounded_rectangle(
        [0, 0, CANVAS_SIZE_PX - 1, CANVAS_SIZE_PX - 1],
        radius=CORNER_RADIUS_PX,
        fill=BACKGROUND_COLOR,
    )

    # Four rotor positions at the diagonal corners (45/135/225/315 deg),
    # i.e. an "X" frame rather than a "+" frame -- matches the real
    # StampFly's motor layout.
    # 対角4隅(45/135/225/315度)のロータ位置。「+」フレームではなく「X」
    # フレーム -- 実機StampFlyのモータ配置と一致させる。
    rotor_angles_deg = (45, 135, 225, 315)
    rotor_centers = [
        (
            CENTER_PX + ARM_REACH_PX * math.cos(math.radians(angle)),
            CENTER_PX + ARM_REACH_PX * math.sin(math.radians(angle)),
        )
        for angle in rotor_angles_deg
    ]

    # Arms: thick white lines from center to each rotor. Drawn before the
    # hub so the hub cleanly covers the arms' converging ends at center.
    # アーム: 中央から各ロータへの太い白線。ハブより先に描き、中央での
    # アーム収束部分をハブできれいに覆う。
    for rotor_center in rotor_centers:
        draw.line([(CENTER_PX, CENTER_PX), rotor_center], fill=MOTIF_COLOR, width=ARM_WIDTH_PX)

    # Rotor circles: filled white discs, matching arm width visually.
    # ロータ円: 白の塗りつぶし円（アーム幅と視覚的に揃える）。
    for rotor_center in rotor_centers:
        _draw_circle(draw, rotor_center, ROTOR_RADIUS_PX, fill=MOTIF_COLOR)

    # Center hub: background-colored disc with a white ring, so the white
    # lightning bolt drawn on top reads clearly against navy, not white.
    # 中央ハブ: 背景色の円+白リング。これにより中央に描く白い稲妻が
    # 白背景ではなく濃紺の上にはっきり見える。
    _draw_circle(draw, (CENTER_PX, CENTER_PX), HUB_RADIUS_PX, fill=BACKGROUND_COLOR)
    _draw_circle(
        draw,
        (CENTER_PX, CENTER_PX),
        HUB_RADIUS_PX,
        outline=MOTIF_COLOR,
        width=HUB_OUTLINE_WIDTH_PX,
    )

    bolt_points = [
        (CENTER_PX + fx * HUB_RADIUS_PX, CENTER_PX + fy * HUB_RADIUS_PX)
        for fx, fy in BOLT_POINTS_FRACTIONS
    ]
    draw.polygon(bolt_points, fill=MOTIF_COLOR)

    return image


def _draw_circle(draw: ImageDraw.ImageDraw, center, radius_px, **kwargs) -> None:
    """Draw a circle given a (cx, cy) center and radius -- Pillow's
    ellipse() wants a bounding box, so this converts once in one place.
    (cx, cy) 中心と半径から円を描く -- Pillow の ellipse() はバウンディング
    ボックス指定のため、変換をここに一元化する。"""
    cx, cy = center
    draw.ellipse(
        [cx - radius_px, cy - radius_px, cx + radius_px, cy + radius_px],
        **kwargs,
    )


def export_png(image: Image.Image, size_px: int, path: Path) -> None:
    """Resize (if needed) and save image as a PNG at path.
    必要に応じてリサイズし、image を PNG として path に保存する。"""
    resized = image if size_px == image.width else image.resize(
        (size_px, size_px), RESAMPLE_FILTER
    )
    resized.save(path, format="PNG")
    print(f"  wrote {_display_path(path)} ({size_px}x{size_px})")


def _display_path(path: Path) -> str:
    """Repo-relative path for a friendlier log line when possible (falls
    back to the absolute path for temp files outside the repo, e.g. the
    throwaway .iconset entries).
    ログ表示用にリポジトリ相対パスにする(リポジトリ外の一時ファイル
    -- 使い捨ての.iconsetエントリ等 -- では絶対パスにフォールバック)。"""
    repo_root = OUTPUT_DIR.parent.parent.parent
    try:
        return str(path.relative_to(repo_root))
    except ValueError:
        return str(path)


def export_ico(image: Image.Image, path: Path) -> None:
    """Save a multi-size Windows .ico built from image.
    image から複数サイズ入りの Windows .ico を書き出す。"""
    image.save(path, format="ICO", sizes=[(size, size) for size in ICO_SIZES_PX])
    print(f"  wrote {_display_path(path)} (sizes: {ICO_SIZES_PX})")


def export_icns(image: Image.Image, path: Path) -> None:
    """Build a macOS .icns via `iconutil`. Only possible on macOS (the
    tool is not available elsewhere); prints a warning and returns
    without writing anything on other platforms.
    `iconutil` で macOS の .icns を作る。macOS以外ではツールが無いため
    実行不可 -- 警告を出し何も書き出さずに戻る。"""
    if platform.system() != "Darwin":
        print(
            f"  SKIPPED {path.name}: iconutil is macOS-only; "
            "regenerate this file on a Mac if the icon design changes."
        )
        return

    with tempfile.TemporaryDirectory(prefix="stampfly_flasher_iconset_") as tmp_dir:
        iconset_dir = Path(tmp_dir) / "icon.iconset"
        iconset_dir.mkdir()
        for filename, size_px in ICONSET_ENTRIES:
            export_png(image, size_px, iconset_dir / filename)

        result = subprocess.run(
            ["iconutil", "-c", "icns", str(iconset_dir), "-o", str(path)],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(f"iconutil failed: {result.stderr.strip()}")
    print(f"  wrote {_display_path(path)}")


def main() -> int:
    print("Generating StampFly Flasher icon assets...")
    image = build_base_image()

    export_png(image, CANVAS_SIZE_PX, OUTPUT_DIR / "icon_1024.png")
    export_png(image, ICON_256_SIZE_PX, OUTPUT_DIR / "icon_256.png")
    export_ico(image, OUTPUT_DIR / "icon.ico")
    export_icns(image, OUTPUT_DIR / "icon.icns")

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
