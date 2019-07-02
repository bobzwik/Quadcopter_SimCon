# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv
# from mixer import (expoCmdInv)

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
    params["maxCmd"] = 100
    params["minCmd"] = 1
    
    # params["ifexpo"] = bool(True)
    
    # if params["ifexpo"]:
    #     params["maxCmd"] = 100
    #     params["minCmd"] = 0.01
    # else:
    #     params["maxCmd"] = 100
    #     params["minCmd"] = 1
    
    return params

def init_cmd(params):
    mB = params["mB"]
    g = params["g"]
    kTh = params["kTh"]
    c1 = params["motorc1"]
    c0 = params["motorc0"]
    
    # w = cmd*c1 + c0   and   m*g = 4*kTh*w^2
    w_des = np.sqrt(mB*g/(4*kTh))
    cmd = (w_des-c0)/c1
    # cmd = expoCmdInv(params, cmd)
    
    return [cmd, w_des]

def init_state(params):
    
    x0 = 0      # m
    y0 = 0      # m
    z0 = 0      # m
    phi0 = 0    # rad
    theta0 = 0  # rad
    psi0 = 0    # rad

    s = np.zeros([20])
    s[0] = x0       # x
    s[1] = y0       # y
    s[2] = z0       # z
    s[3] = phi0     # roll
    s[4] = theta0   # pitch
    s[5] = psi0     # yaw
    s[6] = 0        # xdot
    s[7] = 0        # ydot
    s[8] = 0        # zdot
    s[9] = 0        # p
    s[10] = 0       # q
    s[11] = 0       # r

    w_hover = params["w_hover"] # Hovering motor speed
    wdot_hover = 0              # Hovering motor acc

    s[12] = w_hover
    s[13] = wdot_hover
    s[14] = w_hover
    s[15] = wdot_hover
    s[16] = w_hover
    s[17] = wdot_hover
    s[18] = w_hover
    s[19] = wdot_hover
    
    return s