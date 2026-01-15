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
from . import motors as mp
from . import physics as rb
from . import battery as bt

class multicopter():
    '''
    Rotor configuration:
    4_FL_CW    1_FR_CCW
             X
    3_RL_CCW   2_RR_CW
    '''
    def __init__(self, mass, inersia):
        self.body = rb.rigidbody(mass=mass, inersia=inersia)
        self.mp1 = mp.motor_prop(1)
        self.mp2 = mp.motor_prop(2)
        self.mp3 = mp.motor_prop(3)
        self.mp4 = mp.motor_prop(4)
        self.motor_prop = [self.mp1, self.mp2, self.mp3, self.mp4]
        self.distuerbance_moment = [4.4e-6, 4.4e-6, 4.e-6]
        self.distuerbance_force = [1e-6, 1e-6, 1e-6]
        self.battery = bt.battery()

    def set_disturbance(self, moment, force):
        self.distuerbance_moment = moment
        self.distuerbance_force = force

    def set_pqr(self, pqr):
        pqr = np.array(pqr)
        self.body.pqr = pqr
    
    def set_uvw(self, uvw):
        uvw = np.array(uvw)
        self.body.uvw = uvw

    def set_euler(self, euler):
        euler = np.array(euler)
        self.body.set_euler(euler)
        
    def force_moment(self):
        vel_u = self.body.uvw[0][0]
        vel_v = self.body.uvw[1][0]
        vel_w = self.body.uvw[2][0]
        rate_p = self.body.pqr[0][0]
        rate_q = self.body.pqr[1][0]
        rate_r = self.body.pqr[2][0]
        weight = self.body.mass * 9.81
        gravity = np.array([[0.0], [0.0], [weight]])
        gravity_body= self.body.DCM.T @ gravity
        
        #Moment
        moment = self.mp1.get_moment() + self.mp2.get_moment() + self.mp3.get_moment() + self.mp4.get_moment()
        moment_L = moment[0][0] - 1e-5*np.sign(rate_p)*rate_p**2
        moment_M = moment[1][0] - 1e-5*np.sign(rate_q)*rate_q**2
        moment_N = moment[2][0] - 1e-5*np.sign(rate_r)*rate_r**2 

        #Force
        thrust = self.mp1.get_force() + self.mp2.get_force() + self.mp3.get_force() + self.mp4.get_force()
        fx = thrust[0][0] + gravity_body[0][0] - 0.1e0*np.sign(vel_u)*vel_u**2
        fy = thrust[1][0] + gravity_body[1][0] - 0.1e0*np.sign(vel_v)*vel_v**2
        fz = thrust[2][0] + gravity_body[2][0] - 0.1e0*np.sign(vel_w)*vel_w**2
        
        #Add disturbance
        moment_L += np.random.normal(0, self.distuerbance_moment[0])
        moment_M += np.random.normal(0, self.distuerbance_moment[1])
        moment_N += np.random.normal(0, self.distuerbance_moment[2])
        fx += np.random.normal(0, self.distuerbance_force[0])
        fy += np.random.normal(0, self.distuerbance_force[1])
        fz += np.random.normal(0, self.distuerbance_force[2])

        #Output
        Force = [[fx], [fy], [fz]] 
        Moment = np.array([[moment_L],[moment_M],[moment_N]])
        return Force, Moment

    def step(self,voltage, dt):
        #Body state update
        force, moment = self.force_moment()
        self.body.step(force, moment, dt)
        #Motor and Prop state update
        for index, mp in enumerate(self.motor_prop):
            mp.step(voltage[index], dt)

    #EOF