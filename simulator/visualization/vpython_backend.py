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
    def __init__(self, fps):
        # VPythonのシーンを設定
        height = 550
        width = 1000#int(height*9/16)
        self.scene = canvas(title='StampFly Simulation', width=width, height=height, background=vector(2, 34, 43)/255)
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

        #Ringを表示
        sqrt2 = np.sqrt(2)
        ring_z= -1
        position = [(4, 0, ring_z), (6, 0, ring_z), (6+sqrt2, -2+sqrt2, ring_z), (8, -2, ring_z), 
                    (6+sqrt2, -2-sqrt2, ring_z),(6, -4, ring_z), (6-sqrt2, -6+sqrt2, ring_z), (4, -6, ring_z),
                    (4, -8, ring_z),(2+sqrt2, -8-sqrt2, ring_z), (2, -10, ring_z),(2-sqrt2, -8-sqrt2, ring_z),
                    (0, -8, ring_z), (0, -6, ring_z),(0, -4, ring_z), (0, -2, ring_z)]

        axis = [(1, 0, 0), (1, 0, 0), (-1, 1, 0), (0, 1, 0),
                (1, 1, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
                (0, 1, 0), (1, 1, 0), (1, 0, 0),(-1, 1, 0), 
                (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0)]

        ring_s = ring(pos=vec(2, 0, ring_z), axis=vec(1, 0, 0), radius = 0.3, thickness = 0.015, color=color.yellow)
        ring_g = ring(pos=vec(0, 0, 1), axis=vec(0, 0, 1), radius = 0.3, thickness = 0.015, color=color.green)

        for pos,axis in zip(position,axis):
            ring(pos=vec(*pos), axis=vec(*axis), radius = 0.3, thickness = 0.015, color=color.purple)

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

        self.scene.bind('keydown', self.key_pressed)
    
        #StampFly表示
        self.stampfly_object()

        self.timer_text = wtext(text="Elapsed Time: 0.0 s")

    def key_pressed(self, evt):  # info about event is stored in evt
            self.keyname = evt.key
            #print('The ' + self.keyname + ' key was pressed.')

    def floor_object(self):
        # 背景のボックスにテクスチャを適用
        #self.make_texture()
        background = box(pos=vector(0.0, 0.0, 0.0005), size=vector(0.001, 0.001, 120.0), texture=CHECKERBOARD_PATH)        
        background.pos = vec(0.0, 0.0, 0.0)
        background.axis = vec(0.0, 120.0, 0.0)#物体の回転軸
        angle = 0
        x=sin(radians(angle))
        z=cos(radians(angle))
        background.up = vec(x, 0, z)#axisを回転軸としたときupの向きで回転方向を決める

    def ring_object(self,pos,angle=0):
        x=cos(radians(angle))
        y=sin(radians(angle))
        rgb = (np.random.rand(3)).tolist()
        return ring(pos=pos, axis=vec(x, y, 0), radius = 0.3, thickness = 0.015, color=vec(*rgb))

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
        #カメラの見たい場所
        xf = drone.body.position[0][0]
        yf = drone.body.position[1][0]
        zf = drone.body.position[2][0]
        direction = drone.body.euler[2][0]

        #カメラの位置
        pattern = 0
        if pattern == 0:
            #後ろから追いかける
            self.xc =  xf - 1*cos(direction)
            self.yc =  yf - 1*sin(direction)
            self.zc =  zf - 0.15#0.2
        elif pattern == 1:
            #上から追いかける
            self.xc =  xf - 5
            self.yc =  yf - 0.00
            self.zc =  zf-5

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
            #self.follow_camera_setting(drone, t=sim_time)            
            #self.fix_camera_setting(drone, t=sim_time)
            self.fix_human_setting(drone, t=sim_time)  # 操縦者視点の設定を使用
            self.timer_text.text = f"Elapsed Time: {sim_time:.1f} s"  # 表示を更新
        return self.keyname
