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

import numpy as np

class motor_prop():
    def __init__(self, motor_num=1):
        cw = 1
        ccw = -1
        self.omega = 0.0
        self.e = 0.0
        self.i = 0.0
        self.thrust = 0.0
        dlt = 0.0001
        if motor_num == 1:
            self.rotation_dir = ccw
            self.location = np.array([[0.023], [0.023], [0.005]])
        elif motor_num == 2:
            self.rotation_dir = cw
            self.location = np.array([[-0.023], [0.023], [0.005]])
        elif motor_num == 3:
            self.rotation_dir = ccw
            self.location = np.array([[-0.023], [-0.023], [0.005]])
        elif motor_num == 4:
            self.rotation_dir = cw
            self.location = np.array([[0.023], [-0.023], [0.005]])
        
        # Pre-allocate force/moment buffers (avoids 16000 alloc/s)
        # force/momentバッファを事前割り当て（毎ステップのalloc回避）
        self._force_buf = np.array([[0.0], [0.0], [0.0]])
        self._moment_buf = np.array([[0.0], [0.0], [0.0]])
        # Cache location components for manual cross product
        # 手動外積用にlocation成分をキャッシュ
        self._loc_x = self.location[0][0]
        self._loc_y = self.location[1][0]
        self._loc_z = self.location[2][0]

        #StampFlyのパラメータ
        #回転数と電圧の関係から求めたパラメータ
        self.Am = 5.39e-8
        self.Bm = 6.33e-4
        self.Cm = 1.53e-2
        #LCRメータで測定したパラメータ
        self.Lm = 7.5e-6 #1.0e-6
        # Rm を論文LCR実測 0.593Ω に統一（先生決定、コミット 9a656a9f 2026-07-15）。
        # 旧値 0.63 は vpython 側の旧初期値で更新漏れだった。
        self.Rm = 0.593 #0.63 #0.34
        #回転数と推力・トルク測定実験から求めたパラメータ
        # Ct history: pre-2026-07-15 the value here was 1.00e-8. On 2026-07-15
        # it was switched to a bench thrust-stand measurement (6.7e-9), which
        # made kappa = Cq/Ct = 6.12e-3, matching the flight-validated mixer
        # KAPPA of the time (firmware commit 0ae4dea) and fixing a ~18% hover
        # thrust mismatch that made the sim climb at neutral stick (found on
        # the DXH loaner PCs, 2026-07-19). On 2026-08-03 that thrust-stand
        # value was RETRACTED (no valid simultaneous voltage/RPM/thrust
        # measurement exists for the current propeller) and Ct reverted to a
        # PROVISIONAL 1.00e-8 -- numerically the same as the pre-2026-07-15
        # value, but now justified by (1) past same-diameter-propeller
        # measurement and (2) theoretical-simulation consistency, pending a
        # bench V-omega-T co-measurement. kappa (line below, derived from
        # Cq/Ct) is now 4.10e-3. SSOT: control/models/stampfly_physical.yaml
        # measured_2026_07.Ct.
        # Ct の経緯: 2026-07-15以前はここは 1.00e-8 だった。2026-07-15にベンチ推力測定値
        # （6.7e-9）へ切替し、kappa = Cq/Ct = 6.12e-3 となって当時飛行検証済みのミキサー
        # KAPPA（コミット 0ae4dea）と一致、ホバー時の約18%推力ミスマッチ（スティック中立でも
        # 上昇し続けた、2026-07-19 DXH貸出PCで発見）を解消した。2026-08-03、そのthrust
        # stand値は撤回（現行プロペラでの電圧/回転数/推力の有効な同時計測が存在しないため）
        # され、Ct は暫定値 1.00e-8 に戻った——数値上は2026-07-15以前と同じだが、拠り所は
        # (1) 同径プロペラでの過去の確からしい実測、(2) 理論シミュレーションとの整合であり、
        # ベンチ V-ω-T同時計測で確定予定。以下の kappa（Cq/Ct から導出）は 4.10e-3 になる。
        # 正典: control/models/stampfly_physical.yaml measured_2026_07.Ct。
        self.Ct = 1.0e-8
        # NOTE(2026-07-15): 実測は Ke=5.5e-4, R=0.593, τ_c=9.5e-6, B≈0。
        #  (Km,Rm,Dm,Qf)は自己整合パッケージ — 再フィットとセットで更新のこと。
        #  NOTE(2026-07-24): 上の self.Rm を 0.593 に更新済み。Km/Dm/Qf は Rm から
        #  Cq*Rm/Am 等で導出される計算値のため自動的に追従し、Rm は定常特性の式で
        #  代数的にキャンセルする（数値検証: hover ω・電圧とも変化なし）。
        self.Cq = 4.10e-11  # 2026-07-15実測
        #形状と重量から推定した慣性モーメント
        self.Jmp = 1.375e-8  # 2026-07-15実測

        #推定値
        self.Km = self.Cq*self.Rm/self.Am
        self.Dm = (self.Bm - self.Cq*self.Rm/self.Am)*(self.Cq/self.Am)
        self.Qf = self.Cm*self.Cq/self.Am
        self.kappa = self.Cq/self.Ct

        #self.Km = 6.15e-4
        #self.Lm = 1.0e-6
        #self.Dm = 3.25e-8
        #self.Qf = 2.77e-5
        #self.kappa = self.Cq/self.Ct

        #パラメータの確認
        #print('A=',(self.Cq*self.Rm/self.Km))
        #print('B=',(self.Dm+self.Km**2/self.Rm)/(self.Km/self.Rm))
        #print('C=',(self.Qf*self.Rm/self.Km))

    def equilibrium_anguler_velocity(self, T):
        return np.sqrt(T/self.Ct) 

    def equilibrium_voltage(self, T):
        omega0 = self.equilibrium_anguler_velocity(T)
        return self.Rm * ((self.Dm + self.Km**2/self.Rm) * omega0 + self.Cq * omega0**2 + self.Qf) / self.Km
    
    def omega_dot(self, omega, voltage):
        return ( -(self.Dm + self.Km**2/self.Rm ) * omega - self.Cq * omega**2 - self.Qf + self.Km * voltage/self.Rm)/self.Jmp

    def get_current(self, voltage):
        return (voltage - self.Km * self.omega)/self.Rm
    
    def get_thrust(self):
        return self.Ct * self.omega**2
    
    def get_torque(self):
        return self.Cq * self.omega**2

    def get_force(self):
        # Write into pre-allocated buffer (avoids np.array allocation)
        # 事前割り当てバッファに書き込み（np.array生成を回避）
        thrust = self.get_thrust()
        self._force_buf[0][0] = 0.0
        self._force_buf[1][0] = 0.0
        self._force_buf[2][0] = -thrust
        return self._force_buf

    def get_moment(self):
        # Manual cross product: location × [0, 0, -thrust] (avoids np.cross + np.array)
        # 手動外積: location × [0, 0, -thrust]（np.cross + np.array生成を回避）
        thrust = self.get_thrust()
        neg_thrust = -thrust
        # cross(loc, [0,0,-T]) = [loc_y*(-T) - loc_z*0, loc_z*0 - loc_x*(-T), loc_x*0 - loc_y*0]
        #                       = [-loc_y*T, loc_x*T, 0]
        self._moment_buf[0][0] = self._loc_y * neg_thrust
        self._moment_buf[1][0] = -self._loc_x * neg_thrust
        self._moment_buf[2][0] = -self.rotation_dir * self.get_torque()
        return self._moment_buf

    def get_force_moment(self):
        thrust = self.get_thrust()
        neg_thrust = -thrust
        self._force_buf[0][0] = 0.0
        self._force_buf[1][0] = 0.0
        self._force_buf[2][0] = neg_thrust
        self._moment_buf[0][0] = self._loc_y * neg_thrust
        self._moment_buf[1][0] = -self._loc_x * neg_thrust
        self._moment_buf[2][0] = -self.rotation_dir * self.get_torque()
        return self._force_buf, self._moment_buf

    def set_anguler_velocity(self, omega):
        self.omega = omega

    def set_location(self, x, y, z):
        self.location = np.array([[x], [y], [z]])

    def set_rotation_dir(self, rotation_dir):
        self.rotation_dir = rotation_dir 
    
    def step(self, voltage, dt):
        # Runge-Kutta 4th order
        k1 = self.omega_dot(self.omega, voltage)
        k2 = self.omega_dot(self.omega + k1 * dt / 2.0, voltage)
        k3 = self.omega_dot(self.omega + k2 * dt / 2.0, voltage)
        k4 = self.omega_dot(self.omega + k3 * dt, voltage)
        self.omega += (k1 + 2*k2 + 2*k3 + k4) * dt / 6.0
        self.i = self.get_current(voltage)
        self.thrust = self.get_thrust()