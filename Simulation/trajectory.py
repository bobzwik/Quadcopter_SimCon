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


def desiredState(t, trajType, trajSelect, quad, *args):
    
    desPos     = np.array([0., 0., 0.])    # Associated to state x, y, z
    desEul     = np.array([0., 0., 0.])    # Associated to state phi, theta, psi
    desVel     = np.array([0., 0., 0.])    # Associated to state xdot, ydot, zdot
    desPQR     = np.array([0., 0., 0.])    # Associated to state p, q, r
    desThr     = np.array([0., 0., 0.])    # Desired thrust in N-E-D directions


    def hover():
    
        sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
        
        return sDes  
    
    
    def waypoint_timed():
        
        if not (len(t_wps) == wps.shape[0] or len(t_wps) == len(y_wps)):
            raise Exception("Time array and waypoint array not the same size.")
        elif (np.diff(t_wps) <= 0).any():
            raise Exception("Time array isn't properly ordered.")  
        
        if (t == 0):
            t_idx = 0
        elif (t >= t_wps[-1]):
            t_idx = -1
        else:
            t_idx = np.where(t <= t_wps)[0][0] - 1
        
        desPos = wps[t_idx,:]
        desEul[2] = y_wps[t_idx]
    
        sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
        
        return sDes
    
    
    def waypoint_interp():
        
        if not (len(y_wps) == wps.shape[0]):
            raise Exception("Waypoint array and Yaw waypoint array not the same size.")

        distance_segment = wps[1:] - wps[:-1]
    
        T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/v_wp
        t_wps = np.zeros(len(T_segment) + 1)
        t_wps[1:] = np.cumsum(T_segment)
    
        if (t == 0):
            t_idx = 0
            desPos = wps[0,:]
            desEul[2] = y_wps[0]
        elif (t >= t_wps[-1]):
            t_idx = -1
            desPos = wps[-1,:]
            desEul[2] = y_wps[-1]
        else:
            t_idx = np.where(t <= t_wps)[0][0] - 1
            scale = (t - t_wps[t_idx])/T_segment[t_idx]
            desPos = (1 - scale) * wps[t_idx,:] + scale * wps[t_idx + 1,:]
            if (trajSelect[1] == 1):
                desEul[2] = y_wps[t_idx]
            elif (trajSelect[1] == 2):
                desEul[2] = (1 - scale)*y_wps[t_idx] + scale*y_wps[t_idx + 1]
    
        sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
        
        return sDes


    if trajType == "xyz_vel":
        if trajSelect[0] == 1:
            sDes = testVelControl(t)

    elif trajType == "xy_vel_z_pos":
        if trajSelect[0] == 1:
            sDes = testVelControl(t)
    
    elif trajType == "xyz_pos":
        t_wps = args[0]
        wps   = args[1]
        y_wps = args[2]
        v_wp  = args[3]
        if trajSelect[0] == 0:
            sDes = hover()        
        elif trajSelect[0] == 1:
            sDes = waypoint_timed()
        elif trajSelect[0] == 2:
            sDes = waypoint_interp()
        elif trajSelect[0] == 99:
            sDes = testXYZposition(t)

    return sDes




## Testing scripts

def testXYZposition(t):
    desPos     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    desThr     = np.array([0, 0, 0])
    
    if t >= 1 and t < 4:
        desPos = np.array([2, 2, 1])
    elif t >= 4:
        desPos = np.array([2, -2, -2])
        desEul = np.array([0, 0, pi/3])
    sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)

    return sDes


def testVelControl(t):
    desPos     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    desThr     = np.array([0, 0, 0])

    if t >= 1 and t < 4:
        desVel = np.array([3, 2, 0])
    elif t >= 4:
        desVel = np.array([3, -1, 0])
     
    sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
    
    return sDes
