# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv
import utils

def sys_params():
    mB = 1.2 # mass (kg)
    g = 9.81 # gravity (m/s/s)
    dxm = 0.16 # arm length (m)
    dym = 0.16 # arm length (m)
    dzm = 0.05 # arm length (m)
    IB = np.array([[0.0123, 0,      0     ],
                   [0,      0.0123, 0     ],
                   [0,      0,      0.0224]]) # Inertial tensor (kg*m^2)
    kTh = 1.076e-5  # thrust coeff (N/(rad/s)^2)  (1.18e-7 N/RPM^2)
    kTo = 1.632e-7 # torque coeff (Nm/(rad/s)^2)  (1.79e-9 Nm/RPM^2)
    motorc1 = 8.49 # w (rad/s) = cmd*c1 + c0 (cmd in %)
    motorc0 = 74.7
    motordeadband = 1
    
    
    params = {}
    params["mB"] = mB
    params["g"] = g
    params["dxm"] = dxm
    params["dym"] = dym
    params["dzm"] = dzm
    params["IB"] = IB
    params["invI"] = inv(IB)
    params["kTh"] = kTh
    params["kTo"] = kTo
    params["motorc1"] = motorc1
    params["motorc0"] = motorc0
    params["motordeadband"] = motordeadband
    params["tau"] = 0.015
    params["Kp"] = 1.0
    params["damp"] = 1.0
    params["minThr"] = 0.1
    params["maxThr"] = 9.18
    params["ifexpo"] = bool(False)
    params["ifYawFix"] = bool(False)
    
    if params["ifexpo"]:
        params["maxCmd"] = 100
        params["minCmd"] = 0.01
    else:
        params["maxCmd"] = 100
        params["minCmd"] = 1
    
    return params

def init_cmd(params):
    mB = params["mB"]
    g = params["g"]
    kTh = params["kTh"]
    kTo = params["kTo"]
    c1 = params["motorc1"]
    c0 = params["motorc0"]
    
    # w = cmd*c1 + c0   and   m*g/4 = kTh*w^2   and   torque = kTo*w^2
    thr_hover = mB*g/4.0
    w_hover   = np.sqrt(thr_hover/kTh)
    tor_hover = kTo*w_hover*w_hover
    cmd_hover = (w_hover-c0)/c1
    cmd_hover = utils.expoCmdInv(params, cmd_hover)
    return [cmd_hover, w_hover, thr_hover, tor_hover]

def init_state(params):
    
    x0     = 0  # m
    y0     = 0  # m
    z0     = 0  # m
    phi0   = 0  # rad
    theta0 = 0 # rad
    psi0   = 0  # rad

    quat = utils.YPRToQuat(psi0, theta0, phi0)
    
    s = np.zeros(21)
    s[0]  = x0       # x
    s[1]  = y0       # y
    s[2]  = z0       # z
    s[3]  = quat[0]  # q0
    s[4]  = quat[1]  # q1
    s[5]  = quat[2]  # q2
    s[6]  = quat[3]  # q3
    s[7]  = 0        # xdot
    s[8]  = 0        # ydot
    s[9]  = 0        # zdot
    s[10] = 0        # p
    s[11] = 0        # q
    s[12] = 0        # r

    w_hover = params["w_hover"] # Hovering motor speed
    wdot_hover = 0              # Hovering motor acc

    s[13] = w_hover
    s[14] = wdot_hover
    s[15] = w_hover
    s[16] = wdot_hover
    s[17] = w_hover
    s[18] = wdot_hover
    s[19] = w_hover
    s[20] = wdot_hover
    
    return s