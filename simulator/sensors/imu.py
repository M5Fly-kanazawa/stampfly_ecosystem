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

def imu(pqr, mot_angle, dist):
    p= pqr[0][0]
    q= pqr[1][0]
    r= pqr[2][0]
    mot1_angle = mot_angle[0][0]
    mot2_angle = mot_angle[1][0]
    mot3_angle = mot_angle[2][0]
    p+=np.random.normal(0, dist) * np.sin(mot1_angle) + np.random.normal(0, dist) * np.sin(mot2_angle) + np.random.normal(0, dist) * np.sin(mot3_angle) +np.random.normal(0, dist) * np.sin(mot4_angle)
    q+=np.random.normal(0, dist) * np.sin(mot1_angle) + np.random.normal(0, dist) * np.sin(mot2_angle) + np.random.normal(0, dist) * np.sin(mot3_angle) +np.random.normal(0, dist) * np.sin(mot4_angle)
    r+=np.random.normal(0, dist) * np.sin(mot1_angle) + np.random.normal(0, dist) * np.sin(mot2_angle) + np.random.normal(0, dist) * np.sin(mot3_angle) +np.random.normal(0, dist) * np.sin(mot4_angle)
    return np.array([[p], [q], [r]])
    