#!/usr/bin/env python3
"""StampFly Flasher app icon generator — 3D render pipeline.

Renders the real StampFly from the landing page's 9-part STL model
(landing/assets/model/) plus procedurally generated propellers (ported
from landing/index.html makeBlade/makeProp), through a numpy z-buffer
rasterizer (true hidden-surface removal; translucent blades blended in
a depth-tested second pass), then composes the approved "Navy Classic"
icon: front-45 aerial view, lightning striking down into the glowing
M5 unit. Outputs into this directory: icon_1024.png, per-size PNGs
(icon_16.png ... icon_512.png; <=32 px use a simplified large-bolt
variant), icon.ico and (macOS only, via iconutil) icon.icns.

landing ページの実機9パーツSTLモデル+手続き生成プロペラ
（landing/index.html の makeBlade/makeProp 移植）を numpy Zバッファ
ラスタライザ（真の隠面消去。半透明ブレードは深度テスト付き後段合成）で
描画し、承認済み「Navy Classic」構図（前方45°俯瞰・稲妻が発光する M5 へ
注ぐ）に合成する。出力（本ディレクトリ）: icon_1024.png / サイズ別PNG
（icon_16.png〜icon_512.png。32px以下は稲妻を大きくした簡略版）/
icon.ico /（macOSのみ、iconutil 経由で）icon.icns。

Usage: python3 gen_icon_3d.py
Design approved 2026-07-20 after a 5-candidate selection and iterations
(blade strip triangulation fix, z-buffer renderer, prop-hub flush with
the motor shaft tip).
デザインは 2026-07-20 に5案比較+反復調整（羽根のストリップ三角形化修正・
Zバッファ化・ハブ上面を軸先端と面一）を経て承認済み。
"""
import math
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_DIR = REPO_ROOT / "landing" / "assets" / "model"
OUT = Path(__file__).resolve().parent

PART_COLORS = {
    "frame":           "#ccd2db",
    "pcb":             "#23262e",   # slightly lifted from 0x0e1014 so shading reads
    "m5stamps3":       "#ff6a00",
    "battery":         "#3a2a1c",
    "battery_adapter": "#44413b",
    "motor_fl":        "#b9c1cb",
    "motor_fr":        "#b9c1cb",
    "motor_rl":        "#b9c1cb",
    "motor_rr":        "#b9c1cb",
}

def load_stl(path):
    """Parse a binary STL into (N,3,3) vertices / バイナリSTLを頂点配列に"""
    data = path.read_bytes()
    n = int(np.frombuffer(data[80:84], dtype=np.uint32)[0])
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.uint8).reshape(n, 50)
    tri = rec[:, 12:48].copy().view(np.float32).reshape(n, 3, 3)
    return tri.astype(np.float64)

def hex_to_rgb(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i+2], 16) / 255 for i in (0, 2, 4)])


BLADE_RGBA = np.array([1.0, 0.169, 0.239, 0.62])   # 0xff2b3d, opacity .62
HUB_RGBA   = np.array([0.557, 0.020, 0.071, 1.0])  # 0x8e0512

PROP_R = 14.99
HUB_R = PROP_R * 0.22
BLADE_LEN = PROP_R - HUB_R * 0.6
# Prop hub height: hub TOP is flush with the motor-shaft TIP (y=6.814),
# per user direction 2026-07-20 ("モータシャフト上面とハブ上面を揃えて").
# hub_top = PROP_Y + 1.0 + 2.6/2 -> PROP_Y = 6.814 - 2.3 = 4.514.
# プロペラハブ高さ: ハブ上面をモータ軸先端(y=6.814)と面一にする
# （2026-07-20 指示「モータシャフトの上面とプロペラハブの上面を揃えて」）。
# hub_top = PROP_Y + 1.0 + 2.6/2 より PROP_Y = 6.814 - 2.3 = 4.514。
PROP_Y = 4.514
PROP_HUBS = [((22.80, PROP_Y,  22.80),  1), ((-22.80, PROP_Y,  22.80), -1),
             ((22.80, PROP_Y, -22.81), -1), ((-22.80, PROP_Y, -22.81),  1)]


def bezier(p0, p1, p2, p3, n=8):
    t = np.linspace(0, 1, n)[:, None]
    return ((1-t)**3*p0 + 3*(1-t)**2*t*p1 + 3*(1-t)*t**2*p2 + t**3*p3)


def quad(p0, p1, p2, n=6):
    t = np.linspace(0, 1, n)[:, None]
    return (1-t)**2*p0 + 2*(1-t)*t*p1 + t**2*p2


def blade_edges(L, Wm, n=14):
    """上縁・下縁を根本→先端の同数サンプルで返す / upper & lower edges,
    root->tip, equal sample counts (for gap-free strip triangulation).
    NOTE: 扇形分割(tri_fan)は凹輪郭のこの羽根で根本〜中央に未被覆の
    三角形の穴を作っていた(2026-07-20指摘)。ストリップ縫いで根治。"""
    up = np.vstack([
        bezier(np.array([0, .20*Wm]), np.array([.40*L, .50*Wm]),
               np.array([.65*L, .50*Wm]), np.array([.86*L, .30*Wm]), n)[:-1],
        quad(np.array([.86*L, .30*Wm]), np.array([L, .16*Wm]), np.array([L, 0.]), 5),
    ])
    lo = np.vstack([
        bezier(np.array([0, -.24*Wm]), np.array([.40*L, -.58*Wm]),
               np.array([.65*L, -.58*Wm]), np.array([.86*L, -.20*Wm]), n)[:-1],
        quad(np.array([.86*L, -.20*Wm]), np.array([L, -.12*Wm]), np.array([L, 0.]), 5),
    ])
    return up, lo


def strip_tris(up, lo):
    """2本のポリライン間をストリップ三角形化 / stitch two polylines"""
    m = min(len(up), len(lo))
    tris = []
    for i in range(m - 1):
        tris.append(np.array([up[i], lo[i], lo[i+1]]))
        tris.append(np.array([up[i], lo[i+1], up[i+1]]))
    return np.array(tris)


def make_blade_tris(L, Wm, twist_root, twist_tip):
    up, lo = blade_edges(L, Wm)
    u3 = np.column_stack([up[:, 0], up[:, 1], np.zeros(len(up))])
    l3 = np.column_stack([lo[:, 0], lo[:, 1], np.zeros(len(lo))])
    tris = strip_tris(u3, l3)
    v = tris.reshape(-1, 3)
    # linear twist about x / xに沿った線形ねじれ (rotate about x-axis)
    a = twist_root + (twist_tip - twist_root) * np.clip(v[:, 0]/L, 0, 1)
    y, z = v[:, 1].copy(), v[:, 2].copy()
    v[:, 1] = y*np.cos(a) - z*np.sin(a)
    v[:, 2] = y*np.sin(a) + z*np.cos(a)
    # rotateX(pi/2): (x,y,z) -> (x, -z, y)
    v = np.column_stack([v[:, 0], -v[:, 2], v[:, 1]])
    return v.reshape(-1, 3, 3)


def cylinder_tris(r_top, r_bot, h, n=16):
    th = np.linspace(0, 2*np.pi, n+1)
    tris = []
    for i in range(n):
        a, b = th[i], th[i+1]
        pa_t = [r_top*np.cos(a), h/2, r_top*np.sin(a)]
        pb_t = [r_top*np.cos(b), h/2, r_top*np.sin(b)]
        pa_b = [r_bot*np.cos(a), -h/2, r_bot*np.sin(a)]
        pb_b = [r_bot*np.cos(b), -h/2, r_bot*np.sin(b)]
        tris += [np.array([pa_t, pa_b, pb_b]), np.array([pa_t, pb_b, pb_t]),
                 np.array([[0, h/2, 0], pa_t, pb_t])]      # top cap
    return np.array(tris)


def rot_y(a):
    """Rotation matrix about the y (vertical) axis / y軸(鉛直)回りの回転行列"""
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def build_props(spin=18):
    """4基のプロペラ三角形群+RGBA / triangles+RGBA for the 4 props.
    spin: 各プロペラの位相角オフセット[deg]（見栄え用）"""
    blade = make_blade_tris(BLADE_LEN, BLADE_LEN/2.7, 0.38, 0.10)
    blade = blade + np.array([HUB_R*0.6, 0, 0])
    hub = cylinder_tris(HUB_R, HUB_R*0.9, 2.6)
    tris, rgba = [], []
    for k, ((px, py, pz), d) in enumerate(PROP_HUBS):
        base = np.radians(spin + k*37)
        # d (=CW/CCW) only changes the phase direction of the 3 blades;
        # a static render needs no mirrored blade geometry.
        # d(=回転方向)は3枚羽根の位相の向きにだけ効かせる。静止画なので
        # 羽根形状の鏡像化は不要。
        for i in range(3):
            b = blade.reshape(-1, 3) @ rot_y(base + i*2*np.pi/3 * d).T
            t = b.reshape(-1, 3, 3) + np.array([px, py, pz])
            tris.append(t)
            rgba.append(np.tile(BLADE_RGBA, (len(t), 1)))
        h = hub + np.array([px, py + 1.0, pz])
        tris.append(h)
        rgba.append(np.tile(HUB_RGBA, (len(h), 1)))
    return np.concatenate(tris), np.concatenate(rgba)


def _gather(props=True):
    """Collect all triangles + colors: 9 STL parts + 4 procedural props.
    全三角形+色を収集: STL9パーツ+手続き生成プロペラ4基。"""
    tris, rgba = [], []
    for name, chex in PART_COLORS.items():
        t = load_stl(MODEL_DIR / f"{name}.stl")
        tris.append(t)
        rgba.append(np.tile(np.append(hex_to_rgb(chex), 1.0), (len(t), 1)))
    if props:
        pt, pc = build_props()
        tris.append(pt)
        rgba.append(pc)
    return np.concatenate(tris), np.concatenate(rgba)


def _shade(V, C, light, ambient, diffuse):
    """Two-sided Lambert (|n·L|): no normal flipping, so winding order in
    the source STLs cannot distort the shading.
    両面ランバート(|n·L|): 法線を反転しないため、元STLの巻き方向が
    陰影を歪めることがない。"""
    n = np.cross(V[:, 1] - V[:, 0], V[:, 2] - V[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    ln[ln == 0] = 1
    n = n / ln
    L = np.asarray(light, float)
    L = L / np.linalg.norm(L)
    lam = np.abs(n @ L)
    s = np.clip(ambient + diffuse * lam, 0, 1)
    return np.clip(C[:, :3] * s[:, None], 0, 1)


def _raster_pass(V, rgb, alpha, W, H, zbuf, img, abuf, write_z):
    """Rasterize triangles. write_z=True: opaque pass (depth write).
    write_z=False: translucent pass (depth test only, alpha blend).
    三角形群をラスタライズ。write_z=True=不透明(深度書込)、
    False=半透明(深度テストのみ・アルファブレンド)。"""
    # screen coords / スクリーン座標
    xs = (V[:, :, 0] * 0.5 + 0.5) * (W - 1)
    ys = (0.5 - V[:, :, 1] * 0.5) * (H - 1)
    zs = V[:, :, 2]

    order = range(len(V)) if write_z else np.argsort(zs.mean(1))  # far→near
    for i in order:
        x0, x1 = xs[i].min(), xs[i].max()
        y0, y1 = ys[i].min(), ys[i].max()
        ix0, ix1 = max(int(np.floor(x0)), 0), min(int(np.ceil(x1)) + 1, W)
        iy0, iy1 = max(int(np.floor(y0)), 0), min(int(np.ceil(y1)) + 1, H)
        if ix0 >= ix1 or iy0 >= iy1:
            continue
        gx, gy = np.meshgrid(np.arange(ix0, ix1) + 0.5, np.arange(iy0, iy1) + 0.5)
        ax, ay = xs[i, 0], ys[i, 0]
        bx, by = xs[i, 1], ys[i, 1]
        cx, cy = xs[i, 2], ys[i, 2]
        d = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
        if abs(d) < 1e-9:
            continue
        w0 = ((by - cy) * (gx - cx) + (cx - bx) * (gy - cy)) / d
        w1 = ((cy - ay) * (gx - cx) + (ax - cx) * (gy - cy)) / d
        w2 = 1.0 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        depth = w0 * zs[i, 0] + w1 * zs[i, 1] + w2 * zs[i, 2]
        sub = (slice(iy0, iy1), slice(ix0, ix1))
        if write_z:
            visible = inside & (depth > zbuf[sub])       # +z = viewer side
            if not visible.any():
                continue
            zb = zbuf[sub]
            zb[visible] = depth[visible]
            for c in range(3):
                ch = img[sub + (c,)]
                ch[visible] = rgb[i, c]
                img[sub + (c,)] = ch
            ab = abuf[sub]
            ab[visible] = 1.0
            abuf[sub] = ab
        else:
            visible = inside & (depth > zbuf[sub])       # in front of opaque
            if not visible.any():
                continue
            a = alpha[i]
            for c in range(3):
                ch = img[sub + (c,)]
                ch[visible] = rgb[i, c] * a + ch[visible] * (1 - a)
                img[sub + (c,)] = ch
            ab = abuf[sub]
            ab[visible] = a + ab[visible] * (1 - a)
            abuf[sub] = ab


def render_hero(yaw=45, tilt=35, out="hero_zb.png", px=1600, ss=2,
                light=(-0.30, 0.85, 0.45), ambient=0.42, diffuse=0.62):
    """Render the craft to a transparent PNG (ss x supersampled).
    機体を透過PNGへ描画（ss倍スーパーサンプリング後に縮小）。"""
    T, C = _gather()
    yv, tv = np.radians(yaw), np.radians(tilt)
    Ry = np.array([[np.cos(yv), 0, np.sin(yv)], [0, 1, 0],
                   [-np.sin(yv), 0, np.cos(yv)]])
    Rx = np.array([[1, 0, 0], [0, np.cos(tv), -np.sin(tv)],
                   [0, np.sin(tv), np.cos(tv)]])
    V = T @ (Rx @ Ry).T
    ctr = (V.reshape(-1, 3).max(0) + V.reshape(-1, 3).min(0)) / 2
    V = (V - ctr)
    V = V / (np.abs(V[:, :, :2]).max() * 1.04)

    rgb = _shade(V, C, light, ambient, diffuse)
    W = H = px * ss
    zbuf = np.full((H, W), -1e9)
    img = np.zeros((H, W, 3))
    abuf = np.zeros((H, W))

    opaque = C[:, 3] >= 0.999
    _raster_pass(V[opaque], rgb[opaque], None, W, H, zbuf, img, abuf, True)
    _raster_pass(V[~opaque], rgb[~opaque], C[~opaque, 3], W, H, zbuf, img, abuf, False)

    rgba = np.dstack([img, abuf])
    im = Image.fromarray((np.clip(rgba, 0, 1) * 255).astype(np.uint8), "RGBA")
    im = im.resize((px, px), Image.LANCZOS)
    im.save(OUT / out)
    print("saved", out)


# =============================================================================
# Composition (approved parameters, 2026-07-20)
# 合成（2026-07-20 承認パラメータ）
# =============================================================================
S = 1024
CORNER_R = 190
YELLOW = (255, 205, 40)


def rounded_mask(size, radius):
    m = Image.new("L", (size, size), 0)
    ImageDraw.Draw(m).rounded_rectangle([0, 0, size-1, size-1], radius=radius, fill=255)
    return m


def radial_gradient(size, inner, outer, cx=0.5, cy=0.42):
    img = Image.new("RGB", (size, size))
    px = img.load()
    maxd = math.hypot(max(cx, 1-cx), max(cy, 1-cy)) * size
    for y in range(size):
        for x in range(size):
            t = min(1.0, math.hypot(x - cx*size, y - cy*size) / maxd)
            px[x, y] = tuple(int(i + (o - i)*t) for i, o in zip(inner, outer))
    return img


def crop_alpha(im, pad=30):
    b = im.getchannel("A").getbbox()
    return im.crop((max(b[0]-pad, 0), max(b[1]-pad, 0),
                    min(b[2]+pad, im.width), min(b[3]+pad, im.height)))


def with_shadow(craft, blur=34, dy=34, alpha=110):
    a = craft.getchannel("A").point(lambda v: min(v, alpha))
    sh = Image.new("RGBA", craft.size, (0, 0, 0, 0))
    sh.paste(Image.new("RGBA", craft.size, (0, 0, 10, 255)), (0, 0), a)
    sh = sh.filter(ImageFilter.GaussianBlur(blur))
    outi = Image.new("RGBA", (craft.width, craft.height + dy + blur), (0, 0, 0, 0))
    outi.paste(sh, (0, dy), sh)
    outi.alpha_composite(craft, (0, 0))
    return outi


def bolt(size, fill):
    w = h = size
    pts = [(0.62*w, 0.02*h), (0.24*w, 0.56*h), (0.46*w, 0.56*h),
           (0.36*w, 0.98*h), (0.78*w, 0.40*h), (0.54*w, 0.40*h)]
    im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ImageDraw.Draw(im).polygon(pts, fill=fill)
    return im


def glow(sprite, color, blur=18):
    g = Image.new("RGBA", sprite.size, (0, 0, 0, 0))
    g.paste(Image.new("RGBA", sprite.size, color + (255,)), (0, 0), sprite.getchannel("A"))
    return g.filter(ImageFilter.GaussianBlur(blur))


def place(canvas, im, cx, cy, width):
    r = width / im.width
    im2 = im.resize((int(im.width*r), int(im.height*r)), Image.LANCZOS)
    canvas.alpha_composite(im2, (int(cx - im2.width/2), int(cy - im2.height/2)))


def compose_icon(hero_path, out_path):
    """Compose the approved Navy-Classic icon from the hero render.
    ヒーロー描画から承認済みNavy-Classic構図のアイコンを合成する。"""
    hero = with_shadow(crop_alpha(Image.open(hero_path)))
    bg = radial_gradient(S, (46, 88, 164), (10, 22, 52), cy=0.30).convert("RGBA")
    craft_cx, craft_cy, craft_w = S*0.50, S*0.585, 900
    m5x = int(craft_cx + 0.0167*S*(900/800))
    m5y = int(craft_cy - 0.090*S*(900/800))
    b = bolt(500, YELLOW + (255,))
    bx, by = int(m5x - 0.36*b.width), int(m5y - 0.97*b.height)
    place(bg, hero, craft_cx, craft_cy, craft_w)
    bg.alpha_composite(glow(b, YELLOW, 30), (bx, by))
    bg.alpha_composite(b, (bx, by))
    imp = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    ImageDraw.Draw(imp).ellipse([m5x-64, m5y-46, m5x+64, m5y+46],
                                fill=(255, 235, 140, 185))
    bg.alpha_composite(imp.filter(ImageFilter.GaussianBlur(26)))
    icon = Image.new("RGBA", (S, S), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(rounded_mask(S, CORNER_R))
    icon.save(out_path)
    return icon


def compose_small_variant(render_px=256):
    """Simplified artwork for tiny sizes (16-32 px): the same navy gradient
    with one large lightning bolt. At these sizes the full 3D craft
    collapses into noise, so color + bolt alone carry the icon identity
    (standard small-size icon practice on Windows/macOS).
    小サイズ(16〜32px)用の簡略アートワーク: 同じ紺グラデーションに大きな
    稲妻1本。このサイズでは3D機体が潰れてノイズになるため、配色+稲妻
    だけでアイコンの同一性を担う（Windows/macOSの小サイズ定石）。"""
    s = render_px
    bg = radial_gradient(s, (46, 88, 164), (10, 22, 52), cy=0.30).convert("RGBA")
    b = bolt(int(s * 0.94), YELLOW + (255,))
    # The bolt polygon spans x 0.24-0.78 of its sprite, so its visual
    # center sits at 0.51 of the sprite width (y spans 0.02-0.98 -> 0.50).
    # 稲妻ポリゴンはスプライト幅の0.24〜0.78を占めるため、見た目の中心は
    # 幅の0.51にある（yは0.02〜0.98なので0.50）。
    bx = int(s / 2 - 0.51 * b.width)
    by = int(s / 2 - 0.50 * b.height)
    bg.alpha_composite(glow(b, YELLOW, max(2, s // 20)), (bx, by))
    bg.alpha_composite(b, (bx, by))
    icon = Image.new("RGBA", (s, s), (0, 0, 0, 0))
    icon.paste(bg, (0, 0))
    icon.putalpha(rounded_mask(s, max(2, round(CORNER_R * s / S))))
    return icon


# Per-size artwork policy: >=48 px keeps the full 3D art, <=32 px switches
# to the simplified bolt variant. The size list covers every standard
# on-screen size: macOS Launchpad 128pt (256 px on Retina), GNOME app grid
# 96 px / dash 64 px, Windows desktop 48 px / taskbar 32-24 px / lists 16 px.
# サイズ別アートワーク方針: 48px以上はフル3Dアート、32px以下は簡略版。
# サイズ一覧は各OSの標準表示サイズを網羅する: macOS Launchpad 128pt
# （Retina実256px）、GNOMEアプリ一覧96px/ドック64px、Windowsデスクトップ
# 48px/タスクバー32〜24px/一覧16px。
FULL_ART_SIZES = (512, 256, 128, 96, 64, 48)
SMALL_ART_SIZES = (32, 24, 16)


def export_all(icon):
    """Derive every asset from the 1024 px master: per-size PNGs
    (icon_<N>.png, installed into the Linux hicolor theme), icon.ico with
    per-size artwork embedded, and icon.icns (macOS iconutil).
    1024pxマスターから全資産を派生: サイズ別PNG（icon_<N>.png、Linuxの
    hicolorテーマへ設置される）、サイズ別アートワーク埋め込みのicon.ico、
    icon.icns（macOSのiconutil）。"""
    small = compose_small_variant()

    def art(size):
        base = icon if size >= 48 else small
        return base.resize((size, size), Image.LANCZOS)

    for s_ in FULL_ART_SIZES + SMALL_ART_SIZES:
        art(s_).save(OUT / f"icon_{s_}.png")

    # .ico: Pillow matches append_images frames to `sizes` by pixel size,
    # so each size gets its own artwork instead of one scaled master.
    # .ico: Pillowはappend_imagesの各フレームをピクセルサイズでsizesに
    # 対応付けるため、1枚のマスター縮小ではなくサイズ別アートが入る。
    ico_sizes = (256, 128, 64, 48, 32, 24, 16)
    art(ico_sizes[0]).save(OUT / "icon.ico",
                           sizes=[(s_, s_) for s_ in ico_sizes],
                           append_images=[art(s_) for s_ in ico_sizes[1:]])

    if sys.platform == "darwin" and shutil.which("iconutil"):
        # 16pt/32pt slots (16-32 px renders) carry the small variant; from
        # 32x32@2x (64 px) up the full art is legible again.
        # 16pt/32ptスロット（実16〜32px）は簡略版、32x32@2x(実64px)以上は
        # フルアートが判読できるので切り替える。
        slots = [
            ("icon_16x16.png", art(16)),
            ("icon_16x16@2x.png", art(32)),
            ("icon_32x32.png", art(32)),
            ("icon_32x32@2x.png", art(64)),
            ("icon_128x128.png", art(128)),
            ("icon_128x128@2x.png", art(256)),
            ("icon_256x256.png", art(256)),
            ("icon_256x256@2x.png", art(512)),
            ("icon_512x512.png", art(512)),
            ("icon_512x512@2x.png", icon),
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
    render_hero(out=hero.name, px=2000, ss=2)
    icon = compose_icon(OUT / hero.name, OUT / "icon_1024.png")
    export_all(icon)
    (OUT / hero.name).unlink(missing_ok=True)
    print("all icon assets regenerated in", OUT)
