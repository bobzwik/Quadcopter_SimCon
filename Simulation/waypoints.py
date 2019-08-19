# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import config

deg2rad = pi/180.0

def makeWaypoints():
    t_ini = 0
    wp_ini = np.array([0, 0, 0])
    yaw_ini = 0
    v_average = 1.6


    t = np.array([2, 5, 8, 11, 14])
    wp = np.array([[2, 2, 1],
                   [-2, 3, -3],
                   [-2, -1, -3],
                   [3, -2, 1],
                   [0, 0, 0]])
        
    yaw = np.array([20, -90, 120, 45, 0])

    t = np.hstack((t_ini, t)).astype(float)
    wp = np.vstack((wp_ini, wp)).astype(float)
    yaw = np.hstack((yaw_ini, yaw)).astype(float)*deg2rad

    return t, wp, yaw, v_average
