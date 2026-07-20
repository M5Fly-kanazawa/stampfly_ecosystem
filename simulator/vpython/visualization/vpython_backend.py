# MIT License
# 
# Copyright (c) 2025 Kouhei Ito
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from vpython import *
from stl import mesh
from PIL import Image
import numpy as np
import cv2
import os
import time
import math as _math

# Get the path to the assets directory.
# アセットディレクトリへのパスを取得。
#
# Assets live in simulator/shared/assets, referenced directly. Until
# 2026-07-20 each sim had a simulator/<sim>/assets git symlink pointing here,
# but git symlinks are unreliable on Windows (checked out as a text file, or
# as a reparse point that raises "[Errno 22] Invalid argument" when traversed
# -- which crashed `sf sim run` on a Windows PC), so the symlinks were removed
# and every sim now points at the shared directory itself.
# アセットの実体は simulator/shared/assets にあり、直接参照する。2026-07-20 まで
# 各 sim に simulator/<sim>/assets という git シンボリックリンクがあったが、git
# symlink は Windows で不安定（テキストファイル化、または辿ると "[Errno 22]
# Invalid argument" を出す reparse point 化 -- Windows PC で `sf sim run` が
# クラッシュ）なため symlink を撤去し、各 sim は shared ディレクトリを直接指す。
_VISUALIZATION_DIR = os.path.dirname(os.path.abspath(__file__))
_SIMULATOR_DIR = os.path.dirname(_VISUALIZATION_DIR)  # simulator/vpython
_ASSETS_DIR = os.path.normpath(os.path.join(_SIMULATOR_DIR, '..', 'shared', 'assets'))
_MESHES_DIR = os.path.join(_ASSETS_DIR, 'meshes')
_TEXTURES_DIR = os.path.join(_ASSETS_DIR, 'textures')

# Asset file paths
# アセットファイルパス
STAMPFLY_STL_PATH = os.path.join(_MESHES_DIR, 'stampfly_v1.stl')
CHECKERBOARD_PATH = os.path.join(_TEXTURES_DIR, 'checkerboard.png')

class render():
    def __init__(self, fps, world_type='ringworld', seed=None):
        """
        シミュレータのレンダラー初期化
        Initialize simulator renderer

        Parameters:
            fps: フレームレート
            world_type: ワールドタイプ ('ringworld', 'voxel')
            seed: 乱数シード（None=ランダム、整数=固定）
        """
        self.world_type = world_type

        # 乱数シード設定
        # Set random seed
        if seed is None:
            self.seed = np.random.randint(0, 2**31)
        else:
            self.seed = seed
        print(f"World seed: {self.seed}")

        # 衝突判定用データ
        # Collision detection data
        self.cube_size = 0.5
        self.height_map = {}      # (i, j) -> terrain height (in blocks)
        self.tree_data = []       # [(x, y, ground_h, trunk_height), ...]
        self.collision_detected = False

        # VPythonのシーンを設定
        # VPython scene setup (starts web server + opens browser)
        print("  Starting VPython canvas (launching browser)...", flush=True)
        t_canvas = time.perf_counter()
        height = 550
        width = 1000
        title = f'StampFly Simulation - {world_type.upper()}'
        self.scene = canvas(title=title, width=width, height=height, background=vector(2, 34, 43)/255)
        self.scene.ambient = vec(0.37, 0.37, 0.37)  # 環境光を明るくする
        t_canvas = time.perf_counter() - t_canvas
        print(f"  Canvas ready: {t_canvas:.2f}s", flush=True)

        self.fps = fps
        self.anim_time = 0.0
        self.frame_num = 0
        self.keyname = ''

        # First VPython 3D object creation blocks until browser WebSocket connects.
        # On Windows cold start, this can take 20-30s (browser launch + page load).
        # Subsequent runs with browser already open are instant.
        # 最初のVPython 3Dオブジェクト生成でブラウザWebSocket接続完了までブロック。
        # Windows初回起動時は20-30秒かかる（ブラウザ起動＋ページ読み込み）。
        # ブラウザが既に開いていれば即座に完了。
        print("  Connecting to browser (first launch may take 20-30s)...", flush=True)
        t_browser = time.perf_counter()

        #Cameraの設定
        self.camera_init()

        arrow(pos=vec(0, 0, 0), axis=vec(0.2, 0, 0), shaftwidth=0.005, color=color.red, round=True)
        arrow(pos=vec(0, 0, 0), axis=vec(0, 0.2, 0), shaftwidth=0.005, color=color.green, round=True)
        arrow(pos=vec(0, 0, 0), axis=vec(0, 0, 0.2), shaftwidth=0.005, color=color.blue, round=True)
        t_browser = time.perf_counter() - t_browser
        print(f"  Browser connected: {t_browser:.2f}s", flush=True)

        # Performance: measure world generation time
        # 性能計測: ワールド生成時間
        t0 = time.perf_counter()

        #床面を表示
        self.floor_object()

        # ワールド固有のオブジェクトを表示
        if world_type == 'ringworld':
            self._create_ringworld_objects()

        t_world = time.perf_counter() - t0

        self.scene.bind('keydown', self.key_pressed)

        # Performance: measure STL loading time
        # 性能計測: STL読み込み時間
        t0 = time.perf_counter()

        #StampFly表示
        self.stampfly_object()

        t_stl = time.perf_counter() - t0

        self.timer_text = wtext(text="Elapsed Time: 0.0 s")
        self.scene.append_to_caption('\n')
        self.collision_text = wtext(text="")

        # Performance summary
        # 性能サマリー
        print(f"\n=== Renderer Init Performance ===")
        print(f"  Browser connect:  {t_browser:.2f}s")
        print(f"  World generation: {t_world:.2f}s")
        print(f"  STL loading:     {t_stl:.2f}s")
        print(f"  Total init:      {t_browser + t_world + t_stl:.2f}s")
        print(f"=================================\n")

    def _create_ringworld_objects(self):
        """リングワールド用のリングオブジェクトを配置"""
        #Ringを表示
        sqrt2 = np.sqrt(2)
        ring_z= -1
        position = [(4, 0, ring_z), (6, 0, ring_z), (6+sqrt2, -2+sqrt2, ring_z), (8, -2, ring_z),
                    (6+sqrt2, -2-sqrt2, ring_z),(6, -4, ring_z), (6-sqrt2, -6+sqrt2, ring_z), (4, -6, ring_z),
                    (4, -8, ring_z),(2+sqrt2, -8-sqrt2, ring_z), (2, -10, ring_z),(2-sqrt2, -8-sqrt2, ring_z),
                    (0, -8, ring_z), (0, -6, ring_z),(0, -4, ring_z), (0, -2, ring_z)]

        axis_list = [(1, 0, 0), (1, 0, 0), (-1, 1, 0), (0, 1, 0),
                (1, 1, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
                (0, 1, 0), (1, 1, 0), (1, 0, 0),(-1, 1, 0),
                (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0)]

        ring_s = ring(pos=vec(2, 0, ring_z), axis=vec(1, 0, 0), radius = 0.3, thickness = 0.015, color=color.yellow)
        ring_g = ring(pos=vec(0, 0, 1), axis=vec(0, 0, 1), radius = 0.3, thickness = 0.015, color=color.green)

        for pos, ax in zip(position, axis_list):
            ring(pos=vec(*pos), axis=vec(*ax), radius = 0.3, thickness = 0.015, color=color.purple)

        Ring_Num = 50
        rings=[]
        for i in range(Ring_Num):
            angle=np.random.randint(0,90)
            while True:
                x=np.random.randint(-60, 60)
                y=np.random.randint(-60, 60)
                if not(-1<x<9 and -11<y<1):
                    break
            z= np.random.randint(0, 2)*0.5+ ring_z
            rings.append(self.ring_object(pos=vec(x, y, z), angle=angle))

    def key_pressed(self, evt):  # info about event is stored in evt
            self.keyname = evt.key
            #print('The ' + self.keyname + ' key was pressed.')

    def floor_object(self):
        # ワールドタイプに応じて床を生成
        # Generate floor based on world type
        world_type = getattr(self, 'world_type', 'ringworld')

        if world_type == 'voxel':
            self._create_voxel_world()
        elif world_type == 'minimal':
            self._create_minimal_floor()
        else:
            self._create_ringworld_floor()

    def _create_minimal_floor(self):
        # 最小ワールド: ボックス1個のみ（デバッグ用）
        # Minimal world: single box only (for debugging)
        box(pos=vector(0, 0, 0), size=vector(2, 2, 0.01), color=vector(0.5, 0.5, 0.5))
        print("  Minimal floor: 1 object")

    def _create_ringworld_floor(self):
        # Single ground plane + sparse grid lines for visual reference
        # 1枚の地面 + 疎なグリッド線で視覚的参照
        floor_size = 60  # 60m x 60m
        box(pos=vector(0, 0, 0.001), size=vector(floor_size, floor_size, 0.002),
            color=vector(0.1, 0.6, 0.2))

        # Grid lines every 5m for spatial reference
        # 5mごとのグリッド線（空間把握用）
        grid_count = 0
        grid_spacing = 5
        half = floor_size // 2
        for i in range(-half, half + 1, grid_spacing):
            # Lines along X axis
            box(pos=vector(0, i, 0), size=vector(floor_size, 0.02, 0.003),
                color=vector(0.95, 0.95, 0.95))
            # Lines along Y axis
            box(pos=vector(i, 0, 0), size=vector(0.02, floor_size, 0.003),
                color=vector(0.95, 0.95, 0.95))
            grid_count += 2
        print(f"  Floor: 1 plane + {grid_count} grid lines")

    def _create_voxel_world(self):
        # マインクラフト風ボクセルワールド
        # Minecraft-style voxel world
        cube_size = self.cube_size
        self._voxel_count = 0  # Performance counter / 性能カウンタ

        # シードを設定してランダム地形生成
        # Set seed for random terrain generation
        rng = np.random.RandomState(self.seed)

        # 山の位置と大きさをランダムに生成（平地多め）
        # Generate random mountain positions (mostly flat terrain)
        num_mountains = rng.randint(5, 12)  # 5-12個の山
        mountain_centers = []
        for _ in range(num_mountains):
            mx = rng.uniform(-35, 35)  # 山の中心X
            my = rng.uniform(-35, 35)  # 山の中心Y
            mradius = rng.uniform(3, 10)  # 山の半径
            mheight = rng.uniform(3, 8)  # 山の高さ
            mountain_centers.append((mx, my, mradius, mheight))

        # 色定義
        grass_color = vector(0.2, 0.7, 0.2)
        dirt_color = vector(0.55, 0.35, 0.2)
        stone_color = vector(0.5, 0.5, 0.5)
        wood_color = vector(0.4, 0.25, 0.1)
        leaf_color = vector(0.1, 0.5, 0.1)

        # マップ面積4倍（160x160ブロック = 80m x 80m）
        # Map area 4x larger (160x160 blocks = 80m x 80m)
        world_size = 80
        self.world_size = world_size

        # ノイズ用のランダム位相（ループ外で生成）
        # Random phase for noise (generated outside loop)
        noise_phase = rng.uniform(0, 2 * np.pi)

        for i in range(-world_size, world_size):
            for j in range(-world_size, world_size):
                x = i * cube_size
                y = j * cube_size

                # 基本は平地（高さ1）
                # Base is flat terrain (height 1)
                h = 1.0

                # 山の影響を加算（ガウシアン形状）
                # Add mountain influence (Gaussian shape)
                for mx, my, mradius, mheight in mountain_centers:
                    dist_sq = (x - mx)**2 + (y - my)**2
                    influence = mheight * np.exp(-dist_sq / (2 * mradius**2))
                    h += influence

                # 微小なノイズを追加（自然な見た目のため）
                # Add small noise for natural look
                noise = 0.3 * np.sin(x * 0.5 + noise_phase) * np.cos(y * 0.5 + noise_phase * 0.7)
                h += noise

                height = int(h)
                self.height_map[(i, j)] = max(0, min(height, 12))  # 0-12の範囲に制限

        # ボクセルを配置（表面のみ描画して高速化）
        # Place voxels (render only visible surfaces for performance)
        for i in range(-world_size, world_size):
            for j in range(-world_size, world_size):
                x = (i + 0.5) * cube_size
                y = (j + 0.5) * cube_size
                max_h = self.height_map[(i, j)]

                # 隣接する高さを取得
                neighbors = []
                for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    ni, nj = i + di, j + dj
                    if (ni, nj) in self.height_map:
                        neighbors.append(self.height_map[(ni, nj)])
                    else:
                        neighbors.append(0)
                min_neighbor = min(neighbors)

                # 表面と側面が見える部分のみ描画
                for h in range(min(min_neighbor, max_h), max_h + 1):
                    z = -(h + 0.5) * cube_size

                    # 高さに応じて色を変える
                    if h == max_h:
                        cube_color = grass_color  # 表面は草
                    elif h >= max_h - 2:
                        cube_color = dirt_color   # その下は土
                    else:
                        cube_color = stone_color  # さらに下は石

                    box(pos=vector(x, y, z), size=vector(cube_size, cube_size, cube_size), color=cube_color)
                    self._voxel_count += 1

        # 木を配置（同じrngを使用）
        # Place trees (using same rng)
        num_trees = 30
        tree_positions = []
        for _ in range(num_trees):
            ti = rng.randint(-world_size + 5, world_size - 5)
            tj = rng.randint(-world_size + 5, world_size - 5)
            # 平らな場所に木を置く
            h = self.height_map.get((ti, tj), 0)
            neighbors_h = [self.height_map.get((ti+di, tj+dj), 0) for di in [-1,0,1] for dj in [-1,0,1]]
            if max(neighbors_h) - min(neighbors_h) <= 1:  # 平らな場所
                tree_positions.append((ti, tj, h))

        for ti, tj, ground_h in tree_positions:
            tx = (ti + 0.5) * cube_size
            ty = (tj + 0.5) * cube_size

            # 幹（3-5ブロック）
            trunk_height = rng.randint(3, 6)
            # 衝突判定用に木データを保存
            # Save tree data for collision detection
            self.tree_data.append((tx, ty, ground_h, trunk_height))

            for th in range(trunk_height):
                tz = -(ground_h + th + 1 + 0.5) * cube_size
                box(pos=vector(tx, ty, tz), size=vector(cube_size, cube_size, cube_size), color=wood_color)
                self._voxel_count += 1

            # 葉（幹の上に球状に配置）
            leaf_base = ground_h + trunk_height + 1
            for lx in range(-2, 3):
                for ly in range(-2, 3):
                    for lz in range(-1, 3):
                        # 球状にする
                        dist = np.sqrt(lx**2 + ly**2 + (lz-1)**2)
                        if dist <= 2.2 and rng.random() > 0.2:
                            leaf_px = tx + lx * cube_size
                            leaf_py = ty + ly * cube_size
                            leaf_pz = -(leaf_base + lz + 0.5) * cube_size
                            box(pos=vector(leaf_px, leaf_py, leaf_pz),
                                size=vector(cube_size, cube_size, cube_size), color=leaf_color)
                            self._voxel_count += 1

        print(f"  Voxel objects created: {self._voxel_count}")

    def ring_object(self,pos,angle=0):
        x=cos(radians(angle))
        y=sin(radians(angle))
        rgb = (np.random.rand(3)).tolist()
        return ring(pos=pos, axis=vec(x, y, 0), radius = 0.3, thickness = 0.015, color=vec(*rgb))

    # ========================================================================
    # 衝突判定
    # Collision Detection
    # ========================================================================

    def check_collision(self, x, y, z):
        """
        ドローン位置が障害物と衝突しているか判定
        Check if drone position collides with obstacles

        Parameters:
            x, y, z: ドローンの位置（メートル）。z は下向きが正。

        Returns:
            bool: 衝突していればTrue
        """
        if self.world_type != 'voxel':
            # リングワールドでは地面のみ判定（z=0）
            return z > -0.02  # ドローンの高さ考慮

        # Early exit: if drone is high enough, no collision possible
        # 早期リターン: ドローンが十分高ければ衝突不可
        # Max terrain: 12 blocks × 0.5m + trees ~4 blocks = -8m
        if z < -9.0:
            return False

        # Use pre-computed collision constants
        # 事前計算された衝突判定定数を使用
        cs = self.cube_size
        hm = self.height_map

        drone_hz = 0.02
        drone_bottom = z + drone_hz
        margin_x = 0.05 + cs * 0.5  # drone_hx + cube_size/2
        margin_y = 0.05 + cs * 0.5  # drone_hy + cube_size/2

        # Grid index (use math.floor instead of np.floor — 10x faster for scalars)
        # グリッドインデックス（np.floorの代わりにmath.floor — スカラーでは10倍速）
        inv_cs = 1.0 / cs
        i = _math.floor(x * inv_cs)
        j = _math.floor(y * inv_cs)

        # Terrain collision (3×3 neighborhood)
        # 地形衝突（3×3近傍）
        for di in range(-1, 2):
            ni = i + di
            for dj in range(-1, 2):
                nj = j + dj
                terrain_h = hm.get((ni, nj))
                if terrain_h is None:
                    continue
                terrain_top_z = -(terrain_h + 1) * cs
                if drone_bottom <= terrain_top_z:
                    continue
                voxel_x = (ni + 0.5) * cs
                voxel_y = (nj + 0.5) * cs
                if abs(x - voxel_x) < margin_x and abs(y - voxel_y) < margin_y:
                    return True

        # Tree collision (only nearby trees)
        # 木との衝突（近くの木のみ）
        for tx, ty, ground_h, trunk_height in self.tree_data:
            dx = abs(x - tx)
            if dx > 2.0:
                continue
            dy = abs(y - ty)
            if dy > 2.0:
                continue

            # Trunk check
            if dx < margin_x and dy < margin_y:
                trunk_top_z = -(ground_h + trunk_height + 1) * cs
                trunk_bottom_z = -(ground_h + 1) * cs
                if trunk_top_z < z < trunk_bottom_z:
                    return True

            # Leaf check (sphere, radius ~1.1m)
            leaf_center_z = -(ground_h + trunk_height + 2) * cs
            dist_sq = dx * dx + dy * dy + (z - leaf_center_z) ** 2
            if dist_sq < 1.3225:  # (1.1 + 0.05)² = 1.3225
                return True

        return False

    def get_safe_spawn_position(self, x=0.0, y=0.0, clearance=1.0):
        """
        指定位置の上空で安全なスポーン位置を取得
        Get safe spawn position above specified location

        Parameters:
            x, y: 水平位置（メートル）
            clearance: 地形からの余裕高度（メートル）

        Returns:
            (x, y, z): 安全なスポーン位置。z は下向きが正なので負の値。
        """
        if self.world_type != 'voxel':
            # リングワールドでは固定高度
            return (x, y, -1.0)

        cube_size = self.cube_size
        i = int(np.floor(x / cube_size))
        j = int(np.floor(y / cube_size))

        # 地形高さを取得
        terrain_h = self.height_map.get((i, j), 0)
        terrain_top_z = -(terrain_h + 1) * cube_size

        # 近くの木の高さも考慮
        max_obstacle_z = terrain_top_z
        for tx, ty, ground_h, trunk_height in self.tree_data:
            if abs(x - tx) < 2.0 and abs(y - ty) < 2.0:
                # 葉の頂上
                leaf_top_z = -(ground_h + trunk_height + 4) * cube_size
                if leaf_top_z < max_obstacle_z:
                    max_obstacle_z = leaf_top_z

        # 障害物の上にクリアランスを追加
        safe_z = max_obstacle_z - clearance

        return (x, y, safe_z)

    def show_collision(self, x, y, z):
        """
        衝突をVPython画面に表示
        Display collision on VPython screen

        Parameters:
            x, y, z: 衝突位置
        """
        self.collision_detected = True
        self.collision_text.text = f'<span style="color:red;font-size:20px;font-weight:bold;">💥 COLLISION! ({x:.2f}, {y:.2f}, {z:.2f})</span>'
        # 背景色を赤くフラッシュ
        # Flash background red
        self.scene.background = vector(0.5, 0.1, 0.1)

    def clear_collision(self):
        """
        衝突表示をクリア
        Clear collision display
        """
        self.collision_detected = False
        self.collision_text.text = ""
        self.scene.background = vector(2, 34, 43)/255

    def make_texture(self):
        # 市松模様の画像を生成
        size = 6000 # 画像のサイズ
        N_size = 120  # 市松模様の1辺のマスの数
        tile_size = size // N_size  # 市松模様の1マスのサイズ
        image = Image.new("RGB", (size, size), "white")
        for i in range(N_size):
            for j in range(N_size):
                if (i + j) % 2 == 0:
                    for x in range(tile_size):
                        for y in range(tile_size):
                            image.putpixel((i * tile_size + x, j * tile_size + y), (7, 179, 41, 255))
        # 画像を保存
        image.save("checkerboard.png")


    def stampfly_object(self):
        #STLファイルの構造はStampFlyの前後がx軸、上下がy軸、左右がz軸
        #シミュレーションの座標系は前後（前）がx軸、左右（右）がy軸、上下（下）がz軸
        #STLファイルのYとZのデータを入れ替える.更にZは符号反転
        # STLファイルを読み込む（ファイルパスを指定）
        stl_mesh = mesh.Mesh.from_file(STAMPFLY_STL_PATH)
        total_triangles = len(stl_mesh.vectors)
        print(f"  Loading STL: {total_triangles} triangles...")

        obj=[]
        # STLメッシュデータの頂点をVPython用に変換して表示
        for i in range(total_triangles):
            # Progress every 1000 triangles
            # 1000三角形ごとに進捗表示
            if i % 1000 == 0:
                print(f"    {i}/{total_triangles} ({100*i//total_triangles}%)", flush=True)
            #print(i)
            # 各三角形の頂点を取得
            p0=vector(*stl_mesh.vectors[i][0])/1000
            p0.y = -p0.y
            #dummy = p0.y
            #p0.y = p0.z
            #p0.z = -dummy
            p1=vector(*stl_mesh.vectors[i][1])/1000
            p1.y = -p1.y
            #dummy = p1.y
            #p1.y = p1.z
            #p1.z = -dummy
            p2=vector(*stl_mesh.vectors[i][2])/1000
            p2.y = -p2.y
            #dummy = p2.y
            #p2.y = p2.z
            #p2.z = -dummy
            normal = norm(cross((p1-p0),(p2-p1)))

            if i < 4520:
                #フレーム
                r=0.9
                g=0.9
                b=0.8
                opacity = 1.0
            elif i< 4730:
                #モータ1
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 5450:
                #プロペラ1
                r = 1.0
                g = 0.2
                b = 0.2
                opacity = 0.5
            elif i< 5660:
                #モータ２
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 6050:
                #モータ３　モータ４
                r = 0.8
                g = 1.0
                b = 1.0
                opacity = 1.0
            elif i< 8120:
                #プロペラ２
                r = 1.0
                g = 0.2
                b = 0.2      
                opacity = 0.5      
            elif i< 8411:
                #M5StampS3
                r = 0.9
                g = 0.45
                b = 0.0
                opacity = 1.0
            else:
                r = 0.2
                g = 0.2
                b = 0.2
                opacity = 1.0   
            #print(r,g,b)
            color = vector(r,g,b)
            v0 = vertex(pos=p0, normal=normal, color=color)
            v1 = vertex(pos=p1, normal=normal, color=color)
            v2 = vertex(pos=p2, normal=normal, color=color)
            #print(v0)
            # VPythonのtriangleオブジェクトとして描画
            tri=triangle(
                v0=v0,
                v1=v1,
                v2=v2,
            )
            obj.append(tri)

        self.copter = compound(obj)
        self.copter.pos = vec(0.0, 0.0, 0.0)
        self.copter.axis = vec(1,0,0)
        self.copter.up = vec(0,0,1)
        #sleep(100)

    def camera_init(self):
        # Camera initialization using PRIMARY properties only
        # プライマリプロパティのみでカメラを初期化
        xf = 0.0
        yf = 0.0
        zf = 0.0

        self.xc = xf
        self.yc = yf - 0.2
        self.zc = zf

        fwd_x = xf - self.xc
        fwd_y = yf - self.yc
        fwd_z = zf - self.zc
        d = sqrt(fwd_x**2 + fwd_y**2 + fwd_z**2)
        if d < 1e-6:
            d = 1e-6

        scene_range = 0.2
        self.scene.autoscale = False
        self.scene.center = vector(xf, yf, zf)
        self.scene.forward = vector(fwd_x / d, fwd_y / d, fwd_z / d)
        self.scene.range = scene_range
        self.scene.up = vector(0, 1, 0)
        self.scene.fov = 2 * atan2(scene_range, d)


    def fix_camera_setting(self, drone, t):
        # Fixed camera position using PRIMARY properties only
        # 固定カメラ位置（プライマリプロパティのみ使用）
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]

        self.xc = -2
        self.yc = 0
        self.zc = -5

        fwd_x = xf - self.xc
        fwd_y = yf - self.yc
        fwd_z = zf - self.zc
        d = sqrt(fwd_x**2 + fwd_y**2 + fwd_z**2)
        if d < 1e-6:
            d = 1e-6

        self.scene.autoscale = False
        self.scene.center = vector(xf, yf, zf)
        self.scene.forward = vector(fwd_x / d, fwd_y / d, fwd_z / d)
        self.scene.range = 0.2
        self.scene.up = vector(0, 0, -1)

        if t < 1000.0:
            scene_range = 0.2
        else:
            scene_range = 0.5 + (4.0 * t / 16.0)
        d_ref = sqrt(2**2 + 0**2 + 5**2)
        self.scene.fov = 2 * atan2(scene_range, d_ref)


    def follow_camera_setting(self, drone, t):
        # Camera settings using VPython PRIMARY properties only
        # VPythonのプライマリプロパティのみ使用（camera.pos/axisは派生プロパティ
        # であり、セッター内で相互干渉するため使用しない）
        #
        # Primary: scene.center, scene.forward, scene.range, scene.up, scene.fov
        # Derived (DO NOT USE): scene.camera.pos, scene.camera.axis

        # Drone position and heading
        # ドローンの位置とヘディング
        xf_target = drone.body.position[0][0]
        yf_target = drone.body.position[1][0]
        zf_target = drone.body.position[2][0]
        # Compute yaw from DCM (euler[2][0] is unreliable in step_fast)
        # DCMからyawを計算（step_fastではeuler[2][0]が正しく更新されない場合がある）
        direction = atan2(drone.body.DCM[1,0], drone.body.DCM[0,0])

        # Camera offset behind drone (chase camera)
        # ドローンの後方にカメラを配置（チェイスカメラ）
        cam_distance = 1.0   # distance behind drone / ドローンからの距離
        cam_height = -0.15   # height offset (Z-down) / 高さオフセット
        xc_target = xf_target - cam_distance * cos(direction)
        yc_target = yf_target - cam_distance * sin(direction)
        zc_target = zf_target + cam_height

        # Smooth camera position and look-at point (low-pass filter)
        # カメラ位置と注視点のスムージング（ローパスフィルタ）
        alpha_pos = 0.08   # camera position (smaller = smoother)
        alpha_look = 0.15  # look-at point (faster tracking)
        if not hasattr(self, '_cam_initialized'):
            self.xc = xc_target
            self.yc = yc_target
            self.zc = zc_target
            self._xf = xf_target
            self._yf = yf_target
            self._zf = zf_target
            self._cam_initialized = True
        else:
            self.xc += alpha_pos * (xc_target - self.xc)
            self.yc += alpha_pos * (yc_target - self.yc)
            self.zc += alpha_pos * (zc_target - self.zc)
            self._xf += alpha_look * (xf_target - self._xf)
            self._yf += alpha_look * (yf_target - self._yf)
            self._zf += alpha_look * (zf_target - self._zf)

        # Forward direction: from camera position to look-at point
        # 前方方向: カメラ位置から注視点への方向
        fwd_x = self._xf - self.xc
        fwd_y = self._yf - self.yc
        fwd_z = self._zf - self.zc
        d = sqrt(fwd_x**2 + fwd_y**2 + fwd_z**2)
        if d < 1e-6:
            d = 1e-6

        # Use PRIMARY properties only (no camera.pos/axis setters)
        # プライマリプロパティのみ使用（camera.pos/axisセッターは使わない）
        #
        # Extend center point far ahead to increase depth rendering range.
        # 描画の奥行きを確保するため、centerを前方に延長する。
        # Camera position, FOV, and apparent drone size are unchanged:
        #   cam_to_center = d + depth_ext ≈ 21
        #   range = 0.2 * cam_to_center / d ≈ 4.2
        #   fov = 2*atan2(range, cam_to_center) = 2*atan2(0.2, d) (same)
        #   camera = center - forward * (range/tan(fov/2)) = drone - forward*d
        depth_ext = 20
        fwd_nx, fwd_ny, fwd_nz = fwd_x / d, fwd_y / d, fwd_z / d
        cam_to_center = d + depth_ext
        scene_range = 0.2 * cam_to_center / d

        self.scene.autoscale = False
        self.scene.userspin = False
        self.scene.userzoom = False
        self.scene.center = vector(
            self._xf + fwd_nx * depth_ext,
            self._yf + fwd_ny * depth_ext,
            self._zf + fwd_nz * depth_ext)
        self.scene.forward = vector(fwd_nx, fwd_ny, fwd_nz)
        self.scene.range = scene_range
        self.scene.up = vector(0, 0, -1)
        self.scene.fov = 2 * atan2(scene_range, cam_to_center)


    def fix_human_setting(self, drone, t):
        # Human pilot viewpoint using PRIMARY properties only
        # 操縦者視点（プライマリプロパティのみ使用）
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]

        # Pilot fixed position / 操縦者の固定位置
        self.xc = 0.0
        self.yc = 0.0
        self.zc = -1.5

        # Forward direction: pilot looking at drone
        # 前方方向: 操縦者がドローンを見る方向
        fwd_x = xf - self.xc
        fwd_y = yf - self.yc
        fwd_z = zf - self.zc
        d = sqrt(fwd_x**2 + fwd_y**2 + fwd_z**2)
        if d < 1e-6:
            d = 1e-6

        fov = radians(40)
        self.scene.autoscale = False
        self.scene.center = vector(xf, yf, zf)
        self.scene.forward = vector(fwd_x / d, fwd_y / d, fwd_z / d)
        self.scene.range = d * tan(fov / 2)
        self.scene.up = vector(0, 0, -1)
        self.scene.fov = fov



    def rendering(self, sim_time, drone):
        #3D描画
        if(sim_time >= self.anim_time):
            # Measure rate() blocking time
            # rate()のブロック時間を計測
            t_rate_start = time.perf_counter()
            rate(self.fps)
            t_rate_end = time.perf_counter()
            rate_duration = t_rate_end - t_rate_start

            # Measure rendering overhead (scene update excluding rate())
            # レンダリングオーバーヘッド計測（rate()以外のシーン更新）
            t_render_start = time.perf_counter()
            self.copter.pos = vector(drone.body.position[0][0], drone.body.position[1][0], drone.body.position[2][0])
            axis_x = vector(drone.body.DCM[0,0], drone.body.DCM[1,0], drone.body.DCM[2,0])
            axis_z = vector(drone.body.DCM[0,2], drone.body.DCM[1,2], drone.body.DCM[2,2])
            self.copter.axis = axis_x
            self.copter.up = axis_z
            self.anim_time += 1/self.fps
            self.follow_camera_setting(drone, t=sim_time)  # 後ろから追いかけるカメラ
            #self.fix_camera_setting(drone, t=sim_time)
            #self.fix_human_setting(drone, t=sim_time)  # 操縦者視点の設定を使用
            self.timer_text.text = f"Elapsed Time: {sim_time:.1f} s"  # 表示を更新
            t_render_end = time.perf_counter()
            render_overhead = t_render_end - t_render_start

            # Store timing data for diagnostics
            # 診断用タイミングデータを保存
            self.frame_num += 1
            if not hasattr(self, '_timing_data'):
                self._timing_data = {
                    'rate_durations': [],
                    'render_overheads': [],
                    'physics_steps_per_frame': [],
                }
            self._timing_data['rate_durations'].append(rate_duration)
            self._timing_data['render_overheads'].append(render_overhead)

        return self.keyname

    def record_physics_steps(self, count):
        """Record physics steps executed between frames
        フレーム間に実行された物理ステップ数を記録"""
        if hasattr(self, '_timing_data'):
            self._timing_data['physics_steps_per_frame'].append(count)

    def get_timing_report(self):
        """Generate timing analysis report
        タイミング解析レポートを生成"""
        if not hasattr(self, '_timing_data') or not self._timing_data['rate_durations']:
            return "No timing data collected / タイミングデータなし"

        import statistics
        td = self._timing_data
        rd = td['rate_durations']
        ro = td['render_overheads']
        ps = td['physics_steps_per_frame']

        # Skip first 5 frames (warm-up)
        # 最初の5フレームをスキップ（ウォームアップ）
        if len(rd) > 10:
            rd = rd[5:]
            ro = ro[5:]
        if len(ps) > 10:
            ps = ps[5:]

        report = []
        report.append("=" * 70)
        report.append("FRAME TIMING ANALYSIS REPORT / フレームタイミング解析レポート")
        report.append("=" * 70)
        report.append(f"Total frames analyzed: {len(rd)}")
        report.append(f"Target FPS: {self.fps}")
        report.append(f"Target frame time: {1000.0/self.fps:.2f} ms")
        report.append("")

        # rate() duration statistics
        report.append("--- rate() blocking time / rate()ブロック時間 ---")
        report.append(f"  Average:  {statistics.mean(rd)*1000:.2f} ms")
        report.append(f"  Median:   {statistics.median(rd)*1000:.2f} ms")
        report.append(f"  Min:      {min(rd)*1000:.2f} ms")
        report.append(f"  Max:      {max(rd)*1000:.2f} ms")
        if len(rd) > 1:
            report.append(f"  Stdev:    {statistics.stdev(rd)*1000:.2f} ms")
        report.append(f"  Actual FPS from rate(): {1.0/statistics.mean(rd):.1f}")
        report.append("")

        # Render overhead statistics
        report.append("--- Render overhead (excl. rate()) / レンダリングオーバーヘッド ---")
        report.append(f"  Average:  {statistics.mean(ro)*1000:.3f} ms")
        report.append(f"  Median:   {statistics.median(ro)*1000:.3f} ms")
        report.append(f"  Min:      {min(ro)*1000:.3f} ms")
        report.append(f"  Max:      {max(ro)*1000:.3f} ms")
        report.append("")

        # Physics steps per frame
        if ps:
            report.append("--- Physics steps per frame / フレームあたり物理ステップ数 ---")
            report.append(f"  Average:  {statistics.mean(ps):.1f}")
            report.append(f"  Median:   {statistics.median(ps):.1f}")
            report.append(f"  Min:      {min(ps)}")
            report.append(f"  Max:      {max(ps)}")
            phys_hz = getattr(self, 'physics_hz', 1000)
            report.append(f"  Target ({phys_hz}Hz/{self.fps}FPS): {phys_hz/self.fps:.1f} steps/frame")
            report.append("")

        return "\n".join(report)
