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


def makeWaypoints():
    t_ini = 0
    wp_ini = np.array([0, 0, 0])

    t = np.array([1, 5, 8, 12])

    wp = np.array([
        [2, 2, 1],
        [-2, 4, -3],
        [-2, -1, -6],
        [0, 0, 0]
                   ])

    t = np.hstack((t_ini, t)).astype(float)
    wp = np.vstack((wp_ini, wp)).astype(float)

    return t, wp
