#!/usr/bin/env python3
"""StampFly Setup (ecosystem GUI installer) app icon generator.

Reuses the 3D render pipeline from tools/flasher_gui/assets/gen_icon_3d.py
(imported as a module — single implementation of the STL renderer) and
composes the Setup identity: deep-teal gradient with an orange download
arrow dropping into the glowing M5 unit (install metaphor), instead of the
flasher's navy + lightning bolt. Small sizes (<=32 px) use a simplified
large-arrow variant, mirroring the flasher's small-size policy.

StampFly Setup（エコシステム GUI インストーラ）のアイコン生成。
3Dレンダラは tools/flasher_gui/assets/gen_icon_3d.py を import して共用
（STL レンダラの実装は一箇所のまま）。構図はフラッシャの紺+稲妻に対し、
Setup は深いティール+オレンジのダウンロード矢印が発光する M5 へ落ちる
（インストールのメタファー）。32px 以下はフラッシャと同方針で矢印を
大きくした簡略版。

Usage: python3 gen_icon_3d.py
"""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter

# Reuse the flasher's renderer + composition primitives (single source).
# フラッシャ側のレンダラ・合成プリミティブを共用（実装は一箇所）。
FLASHER_ASSETS = Path(__file__).resolve().parents[2] / "flasher_gui" / "assets"
sys.path.insert(0, str(FLASHER_ASSETS))
import gen_icon_3d as flasher  # noqa: E402

OUT = Path(__file__).resolve().parent
S = flasher.S
CORNER_R = flasher.CORNER_R

# Setup identity colors / Setup の配色
# The arrow is white (not StampFly orange): an orange arrow melts into the
# orange M5 unit it points at — compared side by side 2026-07-20, white wins.
# 矢印は白（StampFlyオレンジではない）: オレンジ矢印は指し先のオレンジの
# M5 に溶け込む — 2026-07-20 の並置比較で白を採用。
TEAL_INNER = (22, 128, 118)
TEAL_OUTER = (6, 40, 38)
ORANGE = (255, 106, 0)
ARROW_FILL = (250, 250, 252)
GLOW_WARM = (255, 190, 90)


def arrow(size, fill):
    """Download-arrow sprite (shaft + head, pointing down).
    ダウンロード矢印スプライト（軸+矢頭、下向き）。"""
    w = h = size
    im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    shaft_w = 0.30 * w
    head_w = 0.72 * w
    head_top = 0.52 * h
    d.rounded_rectangle([w/2 - shaft_w/2, 0.04*h, w/2 + shaft_w/2, head_top + 0.02*h],
                        radius=0.10*w, fill=fill)
    d.polygon([(w/2 - head_w/2, head_top), (w/2 + head_w/2, head_top),
               (w/2, 0.96*h)], fill=fill)
    return im


def compose_icon(hero_path, out_path):
    """Compose the Setup icon from the shared hero render.
    共用ヒーロー描画から Setup アイコンを合成する。"""
    hero = flasher.with_shadow(flasher.crop_alpha(Image.open(hero_path)))
    bg = flasher.radial_gradient(S, TEAL_INNER, TEAL_OUTER, cy=0.30).convert("RGBA")
    craft_cx, craft_cy, craft_w = S*0.50, S*0.585, 900
    m5x = int(craft_cx + 0.0167*S*(900/800))
    m5y = int(craft_cy - 0.090*S*(900/800))
    a = arrow(380, ARROW_FILL + (255,))
    ax, ay = int(m5x - 0.50*a.width), int(m5y - 0.97*a.height)
    flasher.place(bg, hero, craft_cx, craft_cy, craft_w)
    bg.alpha_composite(flasher.glow(a, ORANGE, 30), (ax, ay))
    bg.alpha_composite(a, (ax, ay))
    imp = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    ImageDraw.Draw(imp).ellipse([m5x-64, m5y-46, m5x+64, m5y+46],
                                fill=(255, 214, 140, 185))
    bg.alpha_composite(imp.filter(ImageFilter.GaussianBlur(26)))
    icon = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(flasher.rounded_mask(S, CORNER_R))
    icon.save(out_path)
    return icon


def compose_small_variant(render_px=256):
    """Tiny-size artwork: teal gradient + one large arrow (same policy as
    the flasher's small-size bolt variant).
    小サイズ用: ティールグラデーション+大きな矢印1本（フラッシャの
    小サイズ簡略化と同方針）。"""
    s = render_px
    bg = flasher.radial_gradient(s, TEAL_INNER, TEAL_OUTER, cy=0.30).convert("RGBA")
    a = arrow(int(s * 0.80), ARROW_FILL + (255,))
    ax = int(s/2 - a.width/2)
    ay = int(s/2 - a.height/2)
    bg.alpha_composite(flasher.glow(a, ORANGE, max(2, s // 20)), (ax, ay))
    bg.alpha_composite(a, (ax, ay))
    icon = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(flasher.rounded_mask(s, max(2, round(CORNER_R * s / S))))
    return icon


def export_all(icon):
    """Same per-size policy and asset set as the flasher icon.
    フラッシャと同じサイズ方針・資産一式を出力する。"""
    small = compose_small_variant()

    def art(size):
        base = icon if size >= 48 else small
        return base.resize((size, size), Image.LANCZOS)

    for s_ in flasher.FULL_ART_SIZES + flasher.SMALL_ART_SIZES:
        art(s_).save(OUT / f"icon_{s_}.png")
    ico_sizes = (256, 128, 64, 48, 32, 24, 16)
    art(ico_sizes[0]).save(OUT / "icon.ico",
                           sizes=[(s_, s_) for s_ in ico_sizes],
                           append_images=[art(s_) for s_ in ico_sizes[1:]])
    if sys.platform == "darwin" and shutil.which("iconutil"):
        slots = [
            ("icon_16x16.png", art(16)), ("icon_16x16@2x.png", art(32)),
            ("icon_32x32.png", art(32)), ("icon_32x32@2x.png", art(64)),
            ("icon_128x128.png", art(128)), ("icon_128x128@2x.png", art(256)),
            ("icon_256x256.png", art(256)), ("icon_256x256@2x.png", art(512)),
            ("icon_512x512.png", art(512)), ("icon_512x512@2x.png", icon),
        ]
        with tempfile.TemporaryDirectory() as td:
            iset = Path(td) / "icon.iconset"
            iset.mkdir()
            for name, im in slots:
                im.save(iset / name)
            subprocess.run(["iconutil", "-c", "icns", str(iset),
                            "-o", str(OUT / "icon.icns")], check=True)
        print("icon.icns generated")
    else:
        print("NOTE: .icns skipped (needs macOS iconutil) — regenerate on a Mac")


if __name__ == "__main__":
    hero = OUT / "_hero_tmp.png"
    flasher.OUT = OUT  # render into this directory / 出力先をこちらへ
    flasher.render_hero(out=hero.name, px=2000, ss=2)
    icon = compose_icon(hero, OUT / "icon_1024.png")
    export_all(icon)
    hero.unlink(missing_ok=True)
    print("all Setup icon assets generated in", OUT)
