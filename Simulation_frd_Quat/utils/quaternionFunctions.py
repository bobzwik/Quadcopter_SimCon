# -*- coding: utf-8 -*-
import numpy as np
from numpy import sin, cos
from numpy.linalg import norm

def quatNormalize(q):
    return q/norm(q)

def quatMultiply(q, p):
    Q = np.array([[q[0], -q[1], -q[2], -q[3]],
                  [q[1],  q[0], -q[3],  q[2]],
                  [q[2],  q[3],  q[0], -q[1]],
                  [q[3], -q[2],  q[1],  q[0]]])
    return Q@p