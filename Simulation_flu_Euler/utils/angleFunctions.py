# -*- coding: utf-8 -*-
import numpy as np
from numpy import sin, cos

def RPYtoRot_ZYX(RPY):
    
    phi = RPY[0]
    theta = RPY[1]
    psi = RPY[2]
    
#    R = np.array([[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta),
#                   cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), 
#                   -cos(phi)*sin(theta)],
#                  [-cos(phi)*sin(psi),
#                   cos(phi)*cos(psi),
#                   sin(phi)],
#                  [cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),
#                   sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),
#                   cos(phi)*cos(theta)]])
    
    r1 = psi
    r2 = theta
    r3 = phi
    # Rotation ZYX from page 277 of MotionGenesis book
    R = np.array([[cos(r1)*cos(r2),
                   -sin(r1)*cos(r3) + sin(r2)*sin(r3)*cos(r1), 
                   sin(r1)*sin(r3) + sin(r2)*cos(r1)*cos(r3)],
                  [sin(r1)*cos(r2),
                   cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3),
                   -sin(r3)*cos(r1) + sin(r1)*sin(r2)*cos(r3)],
                  [-sin(r2),
                   sin(r3)*cos(r2),
                   cos(r2)*cos(r3)]])
    
    return R

def RotToQuat(R):
    
    R11 = R[0, 0]
    R12 = R[0, 1]
    R13 = R[0, 2]
    R21 = R[1, 0]
    R22 = R[1, 1]
    R23 = R[1, 2]
    R31 = R[2, 0]
    R32 = R[2, 1]
    R33 = R[2, 2]
    # From page 68 of MotionGenesis book
    tr = R11 + R22 + R33

    if tr > R11 and tr > R22 and tr > R33:
        e0 = 0.5 * np.sqrt(1 + tr)
        r = 0.25 / e0
        e1 = (R32 - R23) * r
        e2 = (R13 - R31) * r
        e3 = (R21 - R12) * r
    elif R11 > R22 and R11 > R33:
        e1 = 0.5 * np.sqrt(1 - tr + 2*R11)
        r = 0.25 / e1
        e0 = (R32 - R23) * r
        e2 = (R12 + R21) * r
        e3 = (R13 + R31) * r
    elif R22 > R33:
        e2 = 0.5 * np.sqrt(1 - tr + 2*R22)
        r = 0.25 / e2
        e0 = (R13 - R31) * r
        e1 = (R12 + R21) * r
        e3 = (R23 + R32) * r
    else:
        e3 = 0.5 * np.sqrt(1 - tr + 2*R33)
        r = 0.25 / e3
        e0 = (R21 - R12) * r
        e1 = (R13 + R31) * r
        e2 = (R23 + R32) * r

    # e0,e1,e2,e3 = qw,qx,qy,qz
    q = np.array([e0,e1,e2,e3])
    q = q*np.sign(e0)
    
    q = q/np.sqrt(np.sum(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2))
    
    return q

def QuatToRPY_ZYX(q):
    # [q0 q1 q2 q3] = [w x y z]
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    
    YPR = threeaxisrot( 2.0*(q1*q2 + q0*q3), \
                        q0**2 + q1**2 - q2**2 - q3**2, \
                        -2.0*(q1*q3 - q0*q2), \
                        2.0*(q2*q3 + q0*q1), \
                        q0**2 - q1**2 - q2**2 + q3**2)

    # YPR = [Yaw, pitch, roll] = [psi, theta, phi]
    return YPR

def threeaxisrot(r11, r12, r21, r31, r32):
    r1 = np.arctan2(r11, r12)
    r2 = np.arcsin(r21)
    r3 = np.arctan2(r31, r32)

    return np.array([r1, r2, r3])

def RPYToQuat(RPY):
    phi = RPY[0]
    theta = RPY[1]
    psi = RPY[2]
    
    thet1 = phi
    thet2 = theta
    thet3 = psi  
    # Equation 8.10 of MotionGenesis book
    e0 = cos(0.5*thet1)*cos(0.5*thet2)*cos(0.5*thet3) - sin(0.5*thet1)*sin(0.5*thet2)*sin(0.5*thet3)
    e1 = sin(0.5*thet1)*cos(0.5*thet2)*cos(0.5*thet3) + sin(0.5*thet2)*sin(0.5*thet3)*cos(0.5*thet1)
    e2 = sin(0.5*thet2)*cos(0.5*thet1)*cos(0.5*thet3) - sin(0.5*thet1)*sin(0.5*thet3)*cos(0.5*thet2)
    e3 = sin(0.5*thet1)*sin(0.5*thet2)*cos(0.5*thet3) + sin(0.5*thet3)*cos(0.5*thet1)*cos(0.5*thet2)
    
    #e0 = cos(0.5*thet1)*cos(0.5*thet2)*cos(0.5*thet3) - sin(0.5*thet1)*cos(0.5*thet2)*sin(0.5*thet3)
    #e1 = cos(0.5*thet1)*sin(0.5*thet2)*sin(0.5*thet3) - sin(0.5*thet1)*sin(0.5*thet2)*cos(0.5*thet3)
    #e2 = cos(0.5*thet1)*sin(0.5*thet2)*cos(0.5*thet3) + sin(0.5*thet1)*sin(0.5*thet2)*sin(0.5*thet3)
    #e3 = sin(0.5*thet1)*cos(0.5*thet2)*cos(0.5*thet3) + cos(0.5*thet1)*cos(0.5*thet2)*sin(0.5*thet3)

    # e0,e1,e2,e3 = qw,qx,qy,qz
    q = np.array([e0,e1,e2,e3])
    q = q*np.sign(e0)
    
    q = q/np.sqrt(np.sum(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2))
    
    return q