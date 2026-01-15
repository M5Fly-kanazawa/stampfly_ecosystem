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

# Get the path to assets directory
# アセットディレクトリへのパスを取得
_VISUALIZATION_DIR = os.path.dirname(os.path.abspath(__file__))
_SIMULATOR_DIR = os.path.dirname(_VISUALIZATION_DIR)
_ASSETS_DIR = os.path.join(_SIMULATOR_DIR, 'assets')
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
        height = 550
        width = 1000
        title = f'StampFly Simulation - {world_type.upper()}'
        self.scene = canvas(title=title, width=width, height=height, background=vector(2, 34, 43)/255)
        self.scene.ambient = vec(0.37, 0.37, 0.37)  # 環境光を明るくする
        self.fps = fps
        self.anim_time = 0.0
        self.frame_num = 0
        self.keyname = ''

        #Cameraの設定
        self.camera_init()

        arrow(pos=vec(0, 0, 0), axis=vec(0.2, 0, 0), shaftwidth=0.005, color=color.red, round=True)
        arrow(pos=vec(0, 0, 0), axis=vec(0, 0.2, 0), shaftwidth=0.005, color=color.green, round=True)
        arrow(pos=vec(0, 0, 0), axis=vec(0, 0, 0.2), shaftwidth=0.005, color=color.blue, round=True)

        #床面を表示
        self.floor_object()

        # ワールド固有のオブジェクトを表示
        if world_type == 'ringworld':
            self._create_ringworld_objects()

        self.scene.bind('keydown', self.key_pressed)

        #StampFly表示
        self.stampfly_object()

        self.timer_text = wtext(text="Elapsed Time: 0.0 s")
        self.scene.append_to_caption('\n')
        self.collision_text = wtext(text="")

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

        Ring_Num = 500
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
        else:
            self._create_ringworld_floor()

    def _create_ringworld_floor(self):
        # 0.5m四方の白と緑の市松模様タイル（リングワールド用）
        # 0.5m x 0.5m white and green checkerboard tiles (for ring world)
        tile_size = 0.5
        tile_range = 60  # -30m to +30m の範囲（広めに）
        white_color = vector(0.95, 0.95, 0.95)
        green_color = vector(0.1, 0.6, 0.2)

        for i in range(-tile_range, tile_range):
            for j in range(-tile_range, tile_range):
                x = (i + 0.5) * tile_size
                y = (j + 0.5) * tile_size
                if (i + j) % 2 == 0:
                    tile_color = white_color
                else:
                    tile_color = green_color
                box(pos=vector(x, y, 0), size=vector(tile_size, tile_size, 0.001), color=tile_color)

    def _create_voxel_world(self):
        # マインクラフト風ボクセルワールド
        # Minecraft-style voxel world
        cube_size = self.cube_size

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

        cube_size = self.cube_size

        # ドローンのバウンディングボックス（半径）
        # Drone bounding box (half-size)
        drone_hx = 0.05  # ~8cm / 2
        drone_hy = 0.05
        drone_hz = 0.02  # ~3cm / 2

        # グリッドインデックスに変換
        # Convert to grid index
        i = int(np.floor(x / cube_size))
        j = int(np.floor(y / cube_size))

        # 地形との衝突判定
        # Terrain collision check
        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                ni, nj = i + di, j + dj
                if (ni, nj) in self.height_map:
                    terrain_h = self.height_map[(ni, nj)]
                    # 地形の上面の高さ（Z座標、上向きが負）
                    terrain_top_z = -(terrain_h + 1) * cube_size

                    # ドローンの底面
                    drone_bottom = z + drone_hz

                    # AABBによる簡易衝突判定
                    voxel_x = (ni + 0.5) * cube_size
                    voxel_y = (nj + 0.5) * cube_size

                    # X, Y軸でオーバーラップ確認
                    if (abs(x - voxel_x) < drone_hx + cube_size / 2 and
                        abs(y - voxel_y) < drone_hy + cube_size / 2):
                        # Z軸でオーバーラップ確認（地形の上面とドローンの底面）
                        if drone_bottom > terrain_top_z:
                            return True

        # 木との衝突判定
        # Tree collision check
        for tx, ty, ground_h, trunk_height in self.tree_data:
            # 近くの木のみチェック（2m以内）
            if abs(x - tx) > 2.0 or abs(y - ty) > 2.0:
                continue

            # 幹との衝突
            trunk_bottom_z = -(ground_h + 1) * cube_size
            trunk_top_z = -(ground_h + trunk_height + 1) * cube_size

            if (abs(x - tx) < drone_hx + cube_size / 2 and
                abs(y - ty) < drone_hy + cube_size / 2):
                # Z軸（幹の範囲内）
                if trunk_top_z < z < trunk_bottom_z:
                    return True

            # 葉との衝突（球状、半径約1.1m）
            leaf_center_z = -(ground_h + trunk_height + 2) * cube_size
            leaf_radius = 1.1  # 約2.2ブロック * 0.5m
            dist_sq = (x - tx)**2 + (y - ty)**2 + (z - leaf_center_z)**2
            if dist_sq < (leaf_radius + drone_hx)**2:
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

        obj=[]
        # STLメッシュデータの頂点をVPython用に変換して表示
        for i in range(len(stl_mesh.vectors)):
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
        #Cameraの設定
        #カメラの見たい場所
        xf = 0.0
        yf = 0.0
        zf = 0.0
        
        #カメラの位置
        self.xc =  xf - 0.00 #scene.upが(0,0,-1)のためこれがうまく表示されない。(0,1,0)に変更するとうまくいく
        self.yc =  yf - 0.2
        self.zc =  zf - 0.0

        #カメラの向き
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        #見える奥行き範囲を延長するための処理
        axis_x = axis_x
        axis_y = axis_y
        axis_z = axis_z
        xf = self.xc + axis_x
        yf = self.yc + axis_y
        zf = self.zc + axis_z

        self.scene.autoscale = False  # オートスケールを無効
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き
        self.scene.up=vector(0,1,0)
        
        #FOVの設定
        scene_range = 0.2
        self.scene.fov = 2*atan2(scene_range, d)

        
    def fix_camera_setting(self, drone, t):
        #Cameraの設定
        #カメラの見たい場所
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]
        
        #カメラの位置
        self.xc =  -2#xf - 0.00 #scene.upが(0,0,-1)のためこれがうまく表示されない。(0,1,0)に変更するとうまくいく
        self.yc =  0#yf - 0.00
        self.zc =  -5

        #カメラの向き
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        #見える奥行き範囲を延長するための処理
        axis_x = axis_x*4
        axis_y = axis_y*4
        axis_z = axis_z*4
        xf = self.xc + axis_x
        yf = self.yc + axis_y
        zf = self.zc + axis_z

        self.scene.autoscale = False  # オートスケールを無効
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き
        self.scene.up=vector(0,0,-1)

        #FOVの設定
        if t < 1000.0:
            scene_range = 0.2
        else:
            scene_range = 0.5 + (4.0 * t/16.0)
        #if scene_range > 3.0:
        #    scene_range = 3.0
        #    scene_range = 0.3
        d = sqrt(2**2 + 0**2 + 5**2)
        self.scene.fov = 2*atan2(scene_range, d)


    def follow_camera_setting(self, drone, t):
        #Cameraの設定
        #カメラの見たい場所（目標）
        xf_target = drone.body.position[0][0]
        yf_target = drone.body.position[1][0]
        zf_target = drone.body.position[2][0]
        direction = drone.body.euler[2][0]

        #カメラの位置（目標）
        pattern = 0
        if pattern == 0:
            #後ろから追いかける
            xc_target = xf_target - 1*cos(direction)
            yc_target = yf_target - 1*sin(direction)
            zc_target = zf_target - 0.15
        elif pattern == 1:
            #上から追いかける
            xc_target = xf_target - 5
            yc_target = yf_target - 0.00
            zc_target = zf_target - 5

        # カメラ位置と注視点のスムージング（ローパスフィルタ）
        # Smooth camera position and look-at point (low-pass filter)
        alpha_pos = 0.08  # カメラ位置用（小さいほど滑らか）
        alpha_look = 0.15  # 注視点用（少し速く追従）
        if not hasattr(self, '_cam_initialized'):
            self.xc = xc_target
            self.yc = yc_target
            self.zc = zc_target
            self._xf = xf_target
            self._yf = yf_target
            self._zf = zf_target
            self._cam_initialized = True
        else:
            # カメラ位置のスムージング
            self.xc = self.xc + alpha_pos * (xc_target - self.xc)
            self.yc = self.yc + alpha_pos * (yc_target - self.yc)
            self.zc = self.zc + alpha_pos * (zc_target - self.zc)
            # 注視点のスムージング
            self._xf = self._xf + alpha_look * (xf_target - self._xf)
            self._yf = self._yf + alpha_look * (yf_target - self._yf)
            self._zf = self._zf + alpha_look * (zf_target - self._zf)

        # スムージングされた注視点を使用
        xf = self._xf
        yf = self._yf
        zf = self._zf

        #カメラの向き
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        #見える奥行き範囲を延長するための処理
        axis_x = axis_x*20
        axis_y = axis_y*20
        axis_z = axis_z*20
        xf = self.xc + axis_x
        yf = self.yc + axis_y
        zf = self.zc + axis_z

        self.scene.autoscale = False  # オートスケールを無効
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き
        self.scene.up=vector(0,0,-1)

        #FOVの設定
        scene_range = 0.2
        self.scene.fov = 2*atan2(scene_range, d)


    def fix_human_setting(self, drone, t):
        #Cameraの設定
        #カメラの見たい場所（ドローンの位置）
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]
        
        #カメラの位置（操縦者の固定位置）
        self.xc = 0.0  # 操縦者のX座標（固定）
        self.yc = 0.0  # 操縦者のY座標（固定）
        self.zc = -1.5  # 操縦者のZ座標（固定）

        #カメラの向き（操縦者がドローンを見る方向）
        axis_x = xf - self.xc
        axis_y = yf - self.yc
        axis_z = zf - self.zc
        d = sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        
        # 操縦者の体の向きを計算（ドローンの方向に体を向ける）
        # XY平面での角度を計算
        angle_xy = atan2(axis_y, axis_x)
        
        # 体の向きを表すupベクトルを計算
        # 基本的には上向き（Z軸負方向）だが、ドローンの位置によって少し傾ける
        # ドローンが高いところにあれば上を向き、低いところにあれば下を向く
        tilt_factor = 0.0  # 体の傾き具合を調整
        up_x = tilt_factor * sin(angle_xy)
        up_y = -tilt_factor * cos(angle_xy)
        up_z = -1.0  # 基本的には上向き
        
        # 視線の方向を設定
        self.scene.autoscale = False  # オートスケールを無効
        self.scene.camera.pos = vector(self.xc, self.yc, self.zc)  # カメラの位置（操縦者の位置）
        self.scene.camera.axis = vector(axis_x, axis_y, axis_z)  # カメラの向き（操縦者の視線）
        self.scene.center = vector(xf, yf, zf)  # カメラの注視点（ドローンの位置）
        self.scene.up = vector(up_x, up_y, up_z)  # 操縦者の体の向き

        #FOVの設定（固定値）
        # 距離に応じてズームしないように固定のFOV値を使用
        # 人間の視野角に近い値（約60度）を使用
        self.scene.fov = radians(40)  # 60度の固定FOV



    def rendering(self, sim_time, drone):
        #3D描画        
        if(sim_time >= self.anim_time):
            rate(self.fps)
            self.copter.pos = vector(*drone.body.position )
            axis_x = vector(drone.body.DCM[0,0], drone.body.DCM[1,0], drone.body.DCM[2,0])
            axis_z = vector(drone.body.DCM[0,2], drone.body.DCM[1,2], drone.body.DCM[2,2])
            self.copter.axis = axis_x
            self.copter.up = axis_z
            self.anim_time += 1/self.fps
            self.follow_camera_setting(drone, t=sim_time)  # 後ろから追いかけるカメラ
            #self.fix_camera_setting(drone, t=sim_time)
            #self.fix_human_setting(drone, t=sim_time)  # 操縦者視点の設定を使用
            self.timer_text.text = f"Elapsed Time: {sim_time:.1f} s"  # 表示を更新
        return self.keyname
