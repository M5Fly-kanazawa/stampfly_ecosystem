#!/usr/bin/env python3
"""StampFly Terminal launcher icon generator.

Reuses the 3D render pipeline from tools/flasher_gui/assets/gen_icon_3d.py
(single renderer implementation) and composes the Terminal identity: a
dark-slate gradient with a green shell prompt glyph (">_") beside the
craft — the launcher opens a ready-to-use sf shell, so the prompt IS the
product. Small sizes (<=32 px) use a simplified large-glyph variant,
same policy as the flasher/setup icons.

StampFly Terminal ランチャーのアイコン生成。3Dレンダラは
tools/flasher_gui/assets/gen_icon_3d.py を import して共用（実装は一箇所）。
構図はターミナルのアイデンティティ: ダークスレートのグラデーションに、
機体と緑のシェルプロンプト記号(">_")。ランチャーの本質は「すぐ使える
sf シェルが開くこと」なので、プロンプト記号そのものを主役に添える。
32px 以下はフラッシャ/Setup と同方針でグリフを大きくした簡略版。

Usage: python3 gen_icon_3d.py
"""
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter

FLASHER_ASSETS = Path(__file__).resolve().parents[2] / "flasher_gui" / "assets"
sys.path.insert(0, str(FLASHER_ASSETS))
import gen_icon_3d as flasher  # noqa: E402

OUT = Path(__file__).resolve().parent
S = flasher.S
CORNER_R = flasher.CORNER_R

# Terminal identity colors / Terminal の配色
SLATE_INNER = (44, 52, 64)
SLATE_OUTER = (10, 13, 18)
PROMPT_GREEN = (63, 205, 90)


def prompt_glyph(size, fill):
    """">_" shell-prompt sprite: a chevron plus a cursor underscore.
    「>_」シェルプロンプトのスプライト: 山括弧+カーソルのアンダースコア。"""
    w = h = size
    im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    t = 0.16 * w                       # stroke thickness / 線の太さ
    # chevron ">" as two joined strokes / 2本のストロークで ">"
    d.polygon([(0.06*w, 0.10*h), (0.06*w + t, 0.10*h), (0.50*w + t, 0.50*h),
               (0.06*w + t, 0.90*h), (0.06*w, 0.90*h), (0.50*w, 0.50*h)],
              fill=fill)
    # cursor "_" / カーソル
    d.rectangle([0.58*w, 0.90*h - t, 0.98*w, 0.90*h], fill=fill)
    return im


def compose_icon(hero_path, out_path):
    """Compose the Terminal icon from the shared hero render.
    共用ヒーロー描画から Terminal アイコンを合成する。"""
    hero = flasher.with_shadow(flasher.crop_alpha(Image.open(hero_path)))
    bg = flasher.radial_gradient(S, SLATE_INNER, SLATE_OUTER, cy=0.30).convert("RGBA")
    flasher.place(bg, hero, S*0.56, S*0.44, 720)
    g = prompt_glyph(360, PROMPT_GREEN + (255,))
    gx, gy = int(S*0.07), int(S*0.60)
    bg.alpha_composite(flasher.glow(g, PROMPT_GREEN, 26), (gx, gy))
    bg.alpha_composite(g, (gx, gy))
    icon = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(flasher.rounded_mask(S, CORNER_R))
    icon.save(out_path)
    return icon


def compose_small_variant(render_px=256):
    """Tiny sizes: slate gradient + one large prompt glyph.
    小サイズ用: スレートグラデーション+大きなプロンプト記号1つ。"""
    s = render_px
    bg = flasher.radial_gradient(s, SLATE_INNER, SLATE_OUTER, cy=0.30).convert("RGBA")
    g = prompt_glyph(int(s * 0.72), PROMPT_GREEN + (255,))
    gx = int(s/2 - g.width/2)
    gy = int(s/2 - g.height/2)
    bg.alpha_composite(flasher.glow(g, PROMPT_GREEN, max(2, s // 20)), (gx, gy))
    bg.alpha_composite(g, (gx, gy))
    icon = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(flasher.rounded_mask(s, max(2, round(CORNER_R * s / S))))
    return icon


def export_all(icon):
    """Same per-size policy and asset set as the flasher/setup icons.
    フラッシャ/Setup のアイコンと同じサイズ方針・資産一式を出力する。"""
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
    flasher.OUT = OUT
    flasher.render_hero(out=hero.name, px=2000, ss=2)
    icon = compose_icon(hero, OUT / "icon_1024.png")
    export_all(icon)
    hero.unlink(missing_ok=True)
    print("all Terminal icon assets generated in", OUT)
