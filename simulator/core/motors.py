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
        
        #StampFlyのパラメータ
        #回転数と電圧の関係から求めたパラメータ
        self.Am = 5.39e-8
        self.Bm = 6.33e-4
        self.Cm = 1.53e-2
        #LCRメータで測定したパラメータ
        self.Lm = 1.0e-6
        self.Rm = 0.34
        #回転数と推力・トルク測定実験から求めたパラメータ
        self.Ct = 1.00e-8
        self.Cq = 9.71e-11
        #形状と重量から推定した慣性モーメント
        self.Jmp = 2.01e-8 

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

        self.armx = 0.025
        self.army = 0.025

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
        return np.array([[0.0], [0.0], [-self.get_thrust()]])

    def get_moment(self):
        moment = np.array([[0.0], [0.0], [-self.rotation_dir * self.get_torque()]])
        moment += np.cross(self.location, np.array([[0.0], [0.0], [-self.get_thrust()]]) , axis=0)
        return moment

    def get_force_moment(self):
        force = np.array([[0.0], [0.0], [-self.get_thrust()]])
        moment = np.array([[0.0], [0.0], [-self.rotation_dir * self.get_torque()]])
        moment += np.cross(self.location, np.array([[0.0], [0.0], [-self.get_thrust()]]) , axis=0)
        return force, moment

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