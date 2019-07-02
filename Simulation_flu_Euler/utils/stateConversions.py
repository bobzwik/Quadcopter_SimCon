# -*- coding: utf-8 -*-

import numpy as np
from numpy import (sin, cos, tan)

def phiThetaPsiDotToPQR(phi, theta, psi, phidot, thetadot, psidot):
    
    p = -sin(theta)*psidot + phidot
    
    q = sin(phi)*cos(theta)*psidot + cos(phi)*thetadot
    
    r = -sin(phi)*thetadot + cos(phi)*cos(theta)*psidot
    
    return np.array([p, q, r])


def xyzDotToUVW(phi, theta, psi, xdot, ydot, zdot):
    u = xdot*cos(psi)*cos(theta) + ydot*sin(psi)*cos(theta) - zdot*sin(theta)
    
    v = (sin(phi)*sin(psi)*sin(theta) + cos(phi)*cos(psi))*ydot + (sin(phi)*sin(theta)*cos(psi) - sin(psi)*cos(phi))*xdot + zdot*sin(phi)*cos(theta)
    
    w = (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*xdot + (-sin(phi)*cos(psi) + sin(psi)*sin(theta)*cos(phi))*ydot + zdot*cos(phi)*cos(theta)
    
    return np.array([u, v, w])


def xyzDotToUVWflat(phi, theta, psi, xdot, ydot, zdot):
    uFlat = xdot * cos(psi) + ydot * sin(psi)

    vFlat = -xdot * sin(psi) + ydot * cos(psi)

    wFlat = zdot

    return np.array([uFlat, vFlat, wFlat])
