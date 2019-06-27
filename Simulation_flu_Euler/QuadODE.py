# -*- coding: utf-8 -*-

import numpy as np
import utils
from numpy import (sin, cos, tan)
from scipy.linalg import solve


def dynamics(s, t, cmd, params):

    # Import Params
    # ---------------------------    
    mB   = params["mB"]
    g    = params["g"]
    dxm  = params["dxm"]
    dym  = params["dym"]
    dzm  = params["dzm"]
    IB   = params["IB"]
    IBxx = IB[0,0]
    IByy = IB[1,1]
    IBzz = IB[2,2]
    kTh  = params["kTh"]
    kTo  = params["kTo"]
    c1   = params["motorc1"]
    c0   = params["motorc0"]
    tau  = params["tau"]
    Kp   = params["Kp"]
    damp = params["damp"]
    db   = params["motordeadband"]
    
    # Import State Vector
    # ---------------------------  
    x          = s[0]
    y          = s[1]
    z          = s[2]
    phi        = s[3]
    theta      = s[4]
    psi        = s[5]
    xdot       = s[6]
    ydot       = s[7]
    zdot       = s[8]
    p          = s[9]
    q          = s[10]
    r          = s[11]
    wMotor1    = s[12]
    wdotMotor1 = s[13]
    wMotor2    = s[14]
    wdotMotor2 = s[15]
    wMotor3    = s[16]
    wdotMotor3 = s[17]
    wMotor4    = s[18]
    wdotMotor4 = s[19]

    # Motor Dynamics and Rotor forces (Second Order System: https://apmonitor.com/pdc/index.php/Main/SecondOrderSystems)
    # ---------------------------
    uMotor = cmd*c1 + c0    # Motor speed in relation to cmd
    uMotor[cmd < db] = 0    # Apply motor deadband

    wddotMotor1 = (-2*damp*tau*wdotMotor1 - wMotor1 + Kp*uMotor[0])/(tau**2)
    wddotMotor2 = (-2*damp*tau*wdotMotor2 - wMotor2 + Kp*uMotor[1])/(tau**2)
    wddotMotor3 = (-2*damp*tau*wdotMotor3 - wMotor3 + Kp*uMotor[2])/(tau**2)
    wddotMotor4 = (-2*damp*tau*wdotMotor4 - wMotor4 + Kp*uMotor[3])/(tau**2)
    
    wMotor = np.array([wMotor1, wMotor2, wMotor3, wMotor4])
    thrust = kTh*wMotor*wMotor
    torque = kTo*wMotor*wMotor
    
    ThrM1 = thrust[0]
    ThrM2 = thrust[1]
    ThrM3 = thrust[2]
    ThrM4 = thrust[3]
    TorM1 = torque[0]
    TorM2 = torque[1]
    TorM3 = torque[2]
    TorM4 = torque[3]
    
#    # Mass Matrix (from PyDy)
#    # ---------------------------
#    MM = np.array([
#            [1, 0, 0, 0, 0, 0,  0,  0,  0,    0,    0,    0],
#            [0, 1, 0, 0, 0, 0,  0,  0,  0,    0,    0,    0],
#            [0, 0, 1, 0, 0, 0,  0,  0,  0,    0,    0,    0],
#            [0, 0, 0, 1, 0, 0,  0,  0,  0,    0,    0,    0],
#            [0, 0, 0, 0, 1, 0,  0,  0,  0,    0,    0,    0],
#            [0, 0, 0, 0, 0, 1,  0,  0,  0,    0,    0,    0],
#            [0, 0, 0, 0, 0, 0, mB,  0,  0,    0,    0,    0],
#            [0, 0, 0, 0, 0, 0,  0, mB,  0,    0,    0,    0],
#            [0, 0, 0, 0, 0, 0,  0,  0, mB,    0,    0,    0],
#            [0, 0, 0, 0, 0, 0,  0,  0,  0, IBxx,    0,    0],
#            [0, 0, 0, 0, 0, 0,  0,  0,  0,    0, IByy,    0],
#            [0, 0, 0, 0, 0, 0,  0,  0,  0,    0,    0, IBzz]])
#    
#    # Right Hand Side (from PyDy)
#    # ---------------------------
#    RHS = np.array([
#            [                                                                               xdot],
#            [                                                                               ydot],
#            [                                                                               zdot],
#            [                                  p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)],
#            [                                                            q*cos(phi) - r*sin(phi)],
#            [                                               (q*sin(phi) + r*cos(phi))/cos(theta)],
#            [ (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)],
#            [-(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)],
#            [                         cos(phi)*cos(theta)*(ThrM1 + ThrM2 + ThrM3 + ThrM4) - g*mB],
#            [                IByy*q*r - IBzz*q*r + ThrM1*dym - ThrM2*dym - ThrM3*dym + ThrM4*dym],
#            [               -IBxx*p*r + IBzz*p*r - ThrM1*dxm - ThrM2*dxm + ThrM3*dxm + ThrM4*dxm],
#            [                                IBxx*p*q - IByy*p*q + TorM1 - TorM2 + TorM3 - TorM4]])
    
    # Right Hand Side 2 (from PyDy) This is already the analytically solved vector of MM*x = RHS
    # ---------------------------
    RHS2 = np.array([
            [                                                                                  xdot],
            [                                                                                  ydot],
            [                                                                                  zdot],
            [                                     p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)],
            [                                                               q*cos(phi) - r*sin(phi)],
            [                                                  (q*sin(phi) + r*cos(phi))/cos(theta)],
            [ (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [-(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [                       (cos(phi)*cos(theta)*(ThrM1 + ThrM2 + ThrM3 + ThrM4) - g*mB)/mB],
            [            (IByy*q*r - IBzz*q*r + ThrM1*dym - ThrM2*dym - ThrM3*dym + ThrM4*dym)/IBxx],
            [           (-IBxx*p*r + IBzz*p*r - ThrM1*dxm - ThrM2*dxm + ThrM3*dxm + ThrM4*dxm)/IByy],
            [                            (IBxx*p*q - IByy*p*q + TorM1 - TorM2 + TorM3 - TorM4)/IBzz]])
    
    # Solve Dynamics A*x = B -> MM*x = RHS
    # Or just use RHS2
    # ---------------------------
    DynamicsDot = RHS2

    # State Derivative Vector
    # ---------------------------
    sdot     = np.zeros([20])
    sdot[0]  = DynamicsDot[0]
    sdot[1]  = DynamicsDot[1]
    sdot[2]  = DynamicsDot[2]
    sdot[3]  = DynamicsDot[3]
    sdot[4]  = DynamicsDot[4]
    sdot[5]  = DynamicsDot[5]
    sdot[6]  = DynamicsDot[6]
    sdot[7]  = DynamicsDot[7]
    sdot[8]  = DynamicsDot[8]
    sdot[9]  = DynamicsDot[9]
    sdot[10] = DynamicsDot[10]
    sdot[11] = DynamicsDot[11]
    sdot[12] = wdotMotor1
    sdot[13] = wddotMotor1
    sdot[14] = wdotMotor2
    sdot[15] = wddotMotor2
    sdot[16] = wdotMotor3
    sdot[17] = wddotMotor3
    sdot[18] = wdotMotor4
    sdot[19] = wddotMotor4

    return sdot
