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
           
    if trajType == "xyz_vel":
        if trajSelect == 1:
            sDes = testVelControl(t)

    if trajType == "xy_vel_z_pos":
        if trajSelect == 1:
            sDes = testVelControl(t)
    
    elif trajType == "xyz_pos":
        if trajSelect == 0:
            sDes = hover(t)
        elif trajSelect == 1:
            sDes = testXYZposition(t)
    
    if trajType == "waypoints":
        t_wp = args[0]
        wp   = args[1]
        y_wp = args[2]
        v_wp = args[3]
        if trajSelect == 0:
            sDes = waypoint_timed(t, t_wp, wp, y_wp)
        elif trajSelect == 1:
            sDes = waypoint_interp(t, quad, wp, y_wp, v_wp)

    return sDes


def hover(t):
    desPos     = np.array([0, 0, 0])    # Associated to state x, y, z
    desEul     = np.array([0, 0, 0])    # Associated to state phi, theta, psi
    desVel     = np.array([0, 0, 0])    # Associated to state xdot, ydot, zdot
    desPQR     = np.array([0, 0, 0])    # Associated to state p, q, r
    desThr     = np.array([0, 0, 0])    # Desired thrust in N-E-D directions
    
    sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
    
    return sDes  


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


def waypoint_timed(t, t_wps, waypoints, y_wps):
    
    desPos     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    desThr     = np.array([0, 0, 0])
    
    if not (len(t_wps) == waypoints.shape[0]):
        raise Exception("Time array and waypoint array not the same size.")
    elif (np.diff(t_wps) <= 0).any():
        raise Exception("Time array isn't properly ordered.")  

    for t_wp, waypoint, y_wp in zip(t_wps, waypoints, y_wps):
        if (t >= t_wp):
            desPos = waypoint
            desEul = [0, 0, y_wp]

    sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
    
    return sDes

def waypoint_interp(t, quad, waypoints, y_wps, v_wp):
    
    desPos     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    desThr     = np.array([0, 0, 0])

    distance_segment = waypoints[:-1] - waypoints[1:]

    T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/v_wp
    
    T_cum = np.zeros(len(T_segment) + 1)
    T_cum[1:] = np.cumsum(T_segment)
    print(T_cum)

    sDes = np.hstack((desPos, desEul, desVel, desPQR, desThr)).astype(float)
    
    return sDes
