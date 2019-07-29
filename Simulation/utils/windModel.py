# -*- coding: utf-8 -*-

import numpy as np
from numpy import sin, cos, pi
import random as rd
import config

deg2rad = pi/180.0

class Wind:

    def __init__(self):
        
        velW_max = 5.0     # m/s
        velW_min = 0.0      # m/s
        qW1_max  = 259.9    # deg
        qW1_min  = 0.0      # deg
        qW2_max  = 25.0     # deg
        qW2_min  = -25.0    # deg

        # Median values
        self.velW_med = (velW_max - velW_min)*rd.random() + velW_min
        self.qW1_med  = ((qW1_max - qW1_min)*rd.random() + qW1_min)*deg2rad
        self.qW2_med  = ((qW2_max - qW2_min)*rd.random() + qW2_min)*deg2rad

        self.velW_a1 = 2.0  # Wind velocity amplitude 1
        self.velW_f1 = 0.7  # Wind velocity frequency 1
        self.velW_d1 = 0    # Wind velocity delay (offset) 1
        self.velW_a2 = 1.3  # Wind velocity amplitude 2
        self.velW_f2 = 1.2  # Wind velocity frequency 2
        self.velW_d2 = 1.3  # Wind velocity delay (offset) 2
        self.velW_a3 = 1.0  # Wind velocity amplitude 3
        self.velW_f3 = 2.3  # Wind velocity frequency 3
        self.velW_d3 = 2.0  # Wind velocity delay (offset) 3

        self.qW1_a1  = 15.0*deg2rad # Wind heading amplitude 1
        self.qW1_f1  = 0.1          # Wind heading frequency 1
        self.qW1_d1  = 0            # Wind heading delay (offset) 1
        self.qW1_a2  = 3.0*deg2rad  # Wind heading amplitude 2
        self.qW1_f2  = 0.54         # Wind heading frequency 2
        self.qW1_d2  = 0            # Wind heading delay (offset) 2

        self.qW2_a1  = 4.0*deg2rad  # Wind elevation amplitude 1
        self.qW2_f1  = 0.1          # Wind elevation frequency 1
        self.qW2_d1  = 0            # Wind elevation delay (offset) 1
        self.qW2_a2  = 0.8*deg2rad  # Wind elevation amplitude 2
        self.qW2_f2  = 0.54         # Wind elevation frequency 2
        self.qW2_d2  = 0            # Wind elevation delay (offset) 2


    def randomWind(self, t):

        velW = self.velW_med
        qW1  = self.qW1_med
        qW2  = self.qW2_med

        velW = self.velW_a1*sin(self.velW_f1*t - self.velW_d1) + self.velW_a2*sin(self.velW_f2*t - self.velW_d2) + self.velW_a3*sin(self.velW_f3*t - self.velW_d3) + self.velW_med
        qW1  = self.qW1_a1*sin(self.qW1_f1*t - self.qW1_d1) + self.qW1_a2*sin(self.qW1_f2*t - self.qW1_d2) + self.qW1_med
        qW2  = self.qW2_a1*sin(self.qW2_f1*t - self.qW2_d1) + self.qW2_a2*sin(self.qW2_f2*t - self.qW2_d2) + self.qW2_med

        velW = max([0, velW])

        return velW, qW1, qW2