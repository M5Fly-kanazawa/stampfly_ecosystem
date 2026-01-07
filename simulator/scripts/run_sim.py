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

# Add simulator package to path
# simulatorパッケージをパスに追加
import sys
import os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_SIMULATOR_DIR = os.path.dirname(_SCRIPT_DIR)
_ECOSYSTEM_DIR = os.path.dirname(_SIMULATOR_DIR)
if _ECOSYSTEM_DIR not in sys.path:
    sys.path.insert(0, _ECOSYSTEM_DIR)

from simulator.core import dynamics as mc
import numpy as np
import matplotlib.pyplot as plt
from simulator.visualization.vpython_backend import *
from vpython import *
from simulator.control.pid import PID
from simulator.interfaces.joystick import *

def test_template():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    stampfly.set_duturbance(moment=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    T=[]
    PQR=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    while t < 10.0:
        if t<5.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, 0.0, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    if True:
        plt.subplot(4,1,1)
        plt.plot(T, UVW[:,0,0], label='u')
        plt.plot(T, UVW[:,1,0], label='v')
        plt.plot(T, UVW[:,2,0], label='w')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('uvw(m/s)')

        plt.subplot(4,1,2)
        plt.plot(T, PQR[:,0,0], label='P')
        plt.plot(T, PQR[:,1,0], label='Q')
        plt.plot(T, PQR[:,2,0], label='R')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('PQR(rad/s)')

        plt.subplot(4,1,3)
        plt.plot(T, EULER[:,0,0], label='phi')
        plt.plot(T, EULER[:,1,0], label='theta')
        plt.plot(T, EULER[:,2,0], label='psi')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Euler angle(rad)')

        plt.subplot(4,1,4)
        plt.plot(T, POS[:,0,0], label='X')
        plt.plot(T, POS[:,1,0], label='Y')
        plt.plot(T, POS[:,2,0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Position(m)')

        plt.show()

def flight_sim(world_type='ringworld', seed=None):
    """
    フライトシミュレーション実行
    Run flight simulation

    Parameters:
        world_type: 'ringworld' or 'voxel'
        seed: 地形生成シード（None=ランダム）
    """
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])

    # レンダラー初期化（衝突判定用データも生成）
    # Initialize renderer (also generates collision detection data)
    Render=render(60, world_type=world_type, seed=seed)

    # 安全なスポーン位置を取得
    # Get safe spawn position above terrain
    spawn_x, spawn_y, spawn_z = Render.get_safe_spawn_position(x=0.0, y=0.0, clearance=1.0)
    stampfly.body.set_position([[spawn_x],[spawn_y],[spawn_z]])
    print(f"Spawn position: ({spawn_x:.2f}, {spawn_y:.2f}, {spawn_z:.2f})")

    joystick = Joystick()
    joystick.open()
    t =0.0
    h = 0.001
    battery_voltage = 3.7
    delta_voltage = 0.0
    delta_roll = 0.0
    delta_pitch = 0.0
    delta_yaw = 0.0
    roll_ref = 0.0
    pitch_ref = 0.0
    yaw_ref = 0.0

    stampfly.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.set_uvw([[0.0],[0.0],[0.0]])
    stampfly.set_euler([[0],[0],[0]])
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    dist = 0  # 1e-8#1e-6
    stampfly.set_disturbance(moment=[dist, dist, dist], force=[dist, dist, dist])
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    T=[]
    PQR=[]
    PQR_REF=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    PQR_REF.append(np.array([[roll_ref], [pitch_ref], [yaw_ref]]))
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    control_time = 0.0
    control_interval = 1e-2

    roll_pid = PID(5.0, 1.0, 0.0)
    pitch_pid = PID(5.0, 1.0, 0.0)
    yaw_pid = PID(0.5, 0.01, 0.0)
    alt_pid = PID(10.0, 5.0, 5.0)

    roll_rate_pid =PID(0.2,10.0,0.002)
    pitch_rate_pid =PID(0.2,10.0,0.002)
    yaw_rate_pid =PID(1.0,2.0,0.001)


    # Joystick calibration（生の値でオフセット計算）
    # Joystick calibration (compute offset with raw values)
    thrust_offset = 0.0
    roll_offset = 0.0
    pitch_offset = 0.0
    yaw_offset = 0.0

    i = 0
    num = 100
    while i < num:
        joydata = joystick.read()
        if joydata is not None:
            # 生の正規化値（-1〜1）でオフセットを計算
            # Compute offset with raw normalized values (-1 to 1)
            thrust_offset += (joydata[0]-127)/127.0
            roll_offset += (joydata[1]-127)/127.0
            pitch_offset += (joydata[2]-127)/127.0
            yaw_offset += (joydata[3]-127)/127.0
            i += 1
            print(i, )
    thrust_offset /= num
    roll_offset /= num
    pitch_offset /= num
    yaw_offset /= num
    print(f"Calibration offsets: thrust={thrust_offset:.4f}, roll={roll_offset:.4f}, pitch={pitch_offset:.4f}, yaw={yaw_offset:.4f}")

    while t < 6000.0:
        rate_p = stampfly.body.pqr[0][0]
        rate_q = stampfly.body.pqr[1][0]
        rate_r = stampfly.body.pqr[2][0]
        phi = stampfly.body.euler[0][0]
        theta = stampfly.body.euler[1][0]
        psi = stampfly.body.euler[2][0]
        xi = stampfly.body.position[0][0]
        yi = stampfly.body.position[1][0]
        zi = stampfly.body.position[2][0]    
        
        joydata = joystick.read()
        if joydata is not None:
            # HIDレポート形式: [throttle, roll, pitch, yaw, buttons, reserved]
            # HID report format: [throttle, roll, pitch, yaw, buttons, reserved]
            # Note: Controller already inverts throttle (4095 - value)
            # 生の値からオフセットを引いてからスケーリング
            # Subtract offset from raw value, then scale
            # 生の値からオフセット補正
            thrust_raw = (joydata[0]-127)/127.0 - thrust_offset
            roll_raw = (joydata[1]-127)/127.0 - roll_offset
            pitch_raw = (joydata[2]-127)/127.0 - pitch_offset
            yaw_raw = (joydata[3]-127)/127.0 - yaw_offset

            # デッドバンド適用（中心付近の微小値を無視）
            # Apply deadband (ignore small values near center)
            deadband = 0.05
            if abs(thrust_raw) < deadband: thrust_raw = 0.0
            if abs(roll_raw) < deadband: roll_raw = 0.0
            if abs(pitch_raw) < deadband: pitch_raw = 0.0
            if abs(yaw_raw) < deadband: yaw_raw = 0.0

            delta_voltage = 0.5 * thrust_raw
            roll_ref = 0.25 * roll_raw * np.pi    # 最大±45°
            pitch_ref = 0.25 * pitch_raw * np.pi  # 最大±45°
            yaw_ref = 0.3 * yaw_raw * np.pi       # ヨーレート感度
        
        #スタビライズモードは実装ちゅう
        control_on = True
        if t >= control_time and control_on:
            control_time += control_interval
            roll_rate_ref = roll_pid.update(roll_ref, phi, control_interval)
            pitch_rate_ref = pitch_pid.update(pitch_ref, theta, control_interval)
            yaw_rate_ref = yaw_ref
            #yaw_rate_ref = yaw_pid.update(yaw_ref, psi, control_interval)


            delta_roll = roll_rate_pid.update(roll_rate_ref, rate_p, control_interval)
            delta_pitch = pitch_rate_pid.update(pitch_rate_ref, rate_q, control_interval)
            delta_yaw = yaw_rate_pid.update(yaw_rate_ref, rate_r, control_interval)
            #delta_voltage = alt_pid.update(0.0, zi, control_interval)

        voltage = nominal_voltage + delta_voltage
        fr = voltage - delta_roll + delta_pitch + delta_yaw  # - 0.01*np.cos(psi - 10*np.pi/180)
        fl = voltage + delta_roll + delta_pitch - delta_yaw  # - 0.01*np.cos(psi - 10*np.pi/180)
        rr = voltage - delta_roll - delta_pitch - delta_yaw  # + 0.01*np.cos(psi - 10*np.pi/180)
        rl = voltage + delta_roll - delta_pitch + delta_yaw  # + 0.01*np.cos(psi - 10*np.pi/180)
        voltage = [fr, rr, rl, fl]
        #print(voltage)
        stampfly.step(voltage, h)

        # 衝突判定
        # Collision detection
        pos = stampfly.body.position
        if Render.check_collision(pos[0][0], pos[1][0], pos[2][0]):
            print(f"COLLISION at t={t:.2f}s, pos=({pos[0][0]:.2f}, {pos[1][0]:.2f}, {pos[2][0]:.2f})")
            # VPython画面に衝突表示
            # Show collision on VPython screen
            Render.show_collision(pos[0][0], pos[1][0], pos[2][0])
            # シミュレーション停止
            # Stop simulation
            break

        key = Render.rendering(t, stampfly)

        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        PQR_REF.append(np.array([[roll_ref], [pitch_ref], [yaw_ref]]))
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    T = np.array(T)
    EULER = np.array(EULER)
    PQR = np.array(PQR)
    PQR_REF = np.array(PQR_REF)
    UVW = np.array(UVW)
    POS = np.array(POS)

    if True:
        plt.subplot(4,1,1)
        plt.plot(T, UVW[:,0,0], label='u')
        plt.plot(T, UVW[:,1,0], label='v')
        plt.plot(T, UVW[:,2,0], label='w')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('uvw(m/s)')

        plt.subplot(4,1,2)
        plt.plot(T, PQR[:,0,0], label='P')
        plt.plot(T, PQR[:,1,0], label='Q')
        plt.plot(T, PQR[:,2,0], label='R')
        plt.plot(T, PQR_REF[:,0,0], label='P_ref')
        plt.plot(T, PQR_REF[:,1,0], label='Q_ref')
        plt.plot(T, PQR_REF[:,2,0], label='R_ref')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('PQR(rad/s)')

        plt.subplot(4,1,3)
        plt.plot(T, EULER[:,0,0], label='phi')
        plt.plot(T, EULER[:,1,0], label='theta')
        plt.plot(T, EULER[:,2,0], label='psi')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Euler angle(rad)')

        plt.subplot(4,1,4)
        plt.plot(T, POS[:,0,0], label='X')
        plt.plot(T, POS[:,1,0], label='Y')
        plt.plot(T, POS[:,2,0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Position(m)')

        plt.show()


def test_two_rotor():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    dist = 1e-4
    stampfly.set_duturbance(moment=[dist, dist, dist], force=[dist, dist, dist])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    T=[]
    PQR=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    while t < 10.0:
        if t<2.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, 0.0, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    UVW=np.array(UVW)
    PQR=np.array(PQR)
    EULER=np.array(EULER)
    POS=np.array(POS)

    if True:
        plt.subplot(4,1,1)
        plt.plot(T, UVW[:,0,0], label='u')
        plt.plot(T, UVW[:,1,0], label='v')
        plt.plot(T, UVW[:,2,0], label='w')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('uvw(m/s)')

        plt.subplot(4,1,2)
        plt.plot(T, PQR[:,0,0], label='P')
        plt.plot(T, PQR[:,1,0], label='Q')
        plt.plot(T, PQR[:,2,0], label='R')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('PQR(rad/s)')

        plt.subplot(4,1,3)
        plt.plot(T, EULER[:,0,0], label='phi')
        plt.plot(T, EULER[:,1,0], label='theta')
        plt.plot(T, EULER[:,2,0], label='psi')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Euler angle(rad)')

        plt.subplot(4,1,4)
        plt.plot(T, POS[:,0,0], label='X')
        plt.plot(T, POS[:,1,0], label='Y')
        plt.plot(T, POS[:,2,0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Position(m)')

        plt.show()

def test_three_rotor():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.0],[0.0],[0.0]])
    dist = 1e-4
    stampfly.set_duturbance(moment=[dist, dist, dist], force=[dist, dist, dist])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/3)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    T=[]
    PQR=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    while t < 10.0:
        if t<2.0:
            voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        else:
            voltage = [0.0, damage_voltage, damage_voltage, damage_voltage]
        stampfly.step(voltage, h)
        Render.rendering(t, stampfly)
        t += h
        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    UVW=np.array(UVW)
    PQR=np.array(PQR)
    EULER=np.array(EULER)
    POS=np.array(POS)

    if True:
        plt.subplot(4,1,1)
        plt.plot(T, UVW[:,0,0], label='u')
        plt.plot(T, UVW[:,1,0], label='v')
        plt.plot(T, UVW[:,2,0], label='w')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('uvw(m/s)')

        plt.subplot(4,1,2)
        plt.plot(T, PQR[:,0,0], label='P')
        plt.plot(T, PQR[:,1,0], label='Q')
        plt.plot(T, PQR[:,2,0], label='R')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('PQR(rad/s)')

        plt.subplot(4,1,3)
        plt.plot(T, EULER[:,0,0], label='phi')
        plt.plot(T, EULER[:,1,0], label='theta')
        plt.plot(T, EULER[:,2,0], label='psi')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Euler angle(rad)')

        plt.subplot(4,1,4)
        plt.plot(T, POS[:,0,0], label='X')
        plt.plot(T, POS[:,1,0], label='Y')
        plt.plot(T, POS[:,2,0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Position(m)')

        plt.show()

def test_ringworld():
    mass = 0.035
    Weight = mass * 9.81
    stampfly = mc.multicopter(mass= mass, inersia=[[9.16e-6, 0.0, 0.0],[0.0, 13.3e-6, 0.0],[0.0, 0.0, 20.4e-6]])
    Render=render(60)
    t =0.0
    h = 0.001

    stampfly.body.set_pqr([[0.0],[0.0],[0.0]])
    stampfly.body.set_uvw([[0.8],[0.0],[0.0]])
    stampfly.set_duturbance(moment=[0.0, 0.0, 0.0], force=[0.0, 0.0, 0.0])
    battery_voltage = 3.7
    nominal_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/4)
    damage_voltage = stampfly.motor_prop[0].equilibrium_voltage(Weight/2)
    nominal_anguler_velocity = stampfly.motor_prop[0].equilibrium_anguler_velocity(Weight/4)
    stampfly.mp1.omega = nominal_anguler_velocity
    stampfly.mp2.omega = nominal_anguler_velocity
    stampfly.mp3.omega = nominal_anguler_velocity
    stampfly.mp4.omega = nominal_anguler_velocity
    print(nominal_voltage)
    print(damage_voltage)

    T=[]
    PQR=[]
    UVW=[]
    EULER=[]
    POS=[]
    T.append(t)
    PQR.append(stampfly.body.pqr.copy())
    UVW.append(stampfly.body.uvw.copy())
    EULER.append(stampfly.body.euler.copy())
    POS.append(stampfly.body.position.copy())

    
    #Ring座標
    sqrt2 = np.sqrt(2)
    ring_position = [(4, 0, 0), (6, 0, 0), (6+sqrt2, -2+sqrt2, 0), (8, -2, 0), 
                (6+sqrt2, -2-sqrt2, 0),(6, -4, 0), (6-sqrt2, -6+sqrt2, 0), (4, -6, 0),
                (4, -8, 0),(2+sqrt2, -8-sqrt2, 0), (2, -10, 0),(2-sqrt2, -8-sqrt2, 0),
                (0, -8, 0), (0, -6, 0),(0, -4, 0), (0, -2, 0)]
    
    ring_axis = [(1, 0, 0), (1, 0, 0), (-1, 1, 0), (0, 1, 0),
            (1, 1, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0),
            (0, 1, 0), (1, 1, 0), (1, 0, 0),(-1, 1, 0), 
            (0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0)]

    vel_ref = 2.0
    uvw_ref = [[[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]],
               [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]], [[vel_ref],[0.0],[0.0]]]
    
    radius = 2.0
    r_ref = vel_ref/radius
    pqr_ref = [[[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[-r_ref]],
               [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[r_ref]], [[0.0],[0.0],[r_ref]],
               [[0.0],[0.0],[0.0]], [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[-r_ref]],
               [[0.0],[0.0],[-r_ref]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]], [[0.0],[0.0],[0.0]]]
                
    flag = 0
    index = 0
    eps =1e-4
    forward_vec = np.array([1.0, 0.0, 0.0])
    while t < 24.0:
        voltage = [nominal_voltage, nominal_voltage, nominal_voltage, nominal_voltage]
        stampfly.body.set_uvw(uvw_ref[index])
        stampfly.body.set_pqr(pqr_ref[index])
        distance = np.linalg.norm(stampfly.body.position.T[0] - np.array(ring_position[index]))
        #print(stampfly.body.position.T[0],ring_position[index], distance)
        #previous_forward_vec = forward_vec.copy()
        ring_vec = np.array(ring_position[index]) - stampfly.body.position.T[0]
        if forward_vec @ ring_vec < 0:
            index += 1
            if index > 15:
                index = 15
            #print(index)
            forward_vec = np.array(ring_position[index]) - stampfly.body.position.T[0]
        stampfly.step(voltage, h)
        t += h
        if np.isnan(stampfly.body.uvw[0][0]):
            break 

        Render.rendering(t, stampfly)

        T.append(t)
        PQR.append(stampfly.body.pqr.copy())
        UVW.append(stampfly.body.uvw.copy())
        EULER.append(stampfly.body.euler.copy())
        POS.append(stampfly.body.position.copy())

    T=np.array(T)
    EULER=np.array(EULER)
    PQR=np.array(PQR)
    UVW=np.array(UVW)
    POS=np.array(POS)

    if False:
        plt.subplot(4,1,1)
        plt.plot(T, UVW[:,0,0], label='u')
        plt.plot(T, UVW[:,1,0], label='v')
        plt.plot(T, UVW[:,2,0], label='w')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('uvw(m/s)')

        plt.subplot(4,1,2)
        plt.plot(T, PQR[:,0,0], label='P')
        plt.plot(T, PQR[:,1,0], label='Q')
        plt.plot(T, PQR[:,2,0], label='R')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('PQR(rad/s)')

        plt.subplot(4,1,3)
        plt.plot(T, EULER[:,0,0], label='phi')
        plt.plot(T, EULER[:,1,0], label='theta')
        plt.plot(T, EULER[:,2,0], label='psi')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Euler angle(rad)')

        plt.subplot(4,1,4)
        plt.plot(T, POS[:,0,0], label='X')
        plt.plot(T, POS[:,1,0], label='Y')
        plt.plot(T, POS[:,2,0], label='Z')
        plt.legend()
        plt.grid()
        plt.xlabel('Time(s)')
        plt.ylabel('Position(m)')

        plt.show()

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='StampFly Flight Simulator')
    parser.add_argument('--world', '-w', type=str, default='voxel',
                        choices=['ringworld', 'voxel'],
                        help='ワールドタイプ (ringworld, voxel)')
    parser.add_argument('--seed', '-s', type=int, default=None,
                        help='地形生成シード（省略時はランダム）')
    args = parser.parse_args()

    print(f"Starting simulation with world: {args.world}")
    flight_sim(world_type=args.world, seed=args.seed)
