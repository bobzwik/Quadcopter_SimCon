# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi

def desiredState(t, trajType, trajSelect):
    if trajType == "altitude":
        if   trajSelect == 1:
            sDes = hover(t)
        elif trajSelect == 2:
            sDes = testZControl(t)
        elif trajSelect == 3:
            sDes = TestYawControl(t)
        elif trajSelect == 4:
            sDes = TestRollThenPitchControl(t)
        elif trajSelect == 5:
            sDes = TestPitchThenYawControl(t)
        
    if trajType == "velocity":
        if   trajSelect == 1:
            sDes = testUVW_FlatControl(t)

    if trajType == "grid_velocity":
        if   trajSelect == 1:
            sDes = testGridVelocityControl(t)   
    
    if trajType == "position":
        if   trajSelect == 1:
            sDes = testXYZposition(t)

    return sDes


def hover(t):
    desPos     = np.array([0, 0, 0])    # Associated to state x, y, z
    desVelGrid = np.array([0, 0, 0])    # Associated to state xdot, ydot, zdot
    desVel     = np.array([0, 0, 0])    # Associated to state uFlat, vFlat, wFlat
    desEul     = np.array([0, 0, 0])    # Associated to state phi, theta, psi
    desPQR     = np.array([0, 0, 0])    # Associated to state p, q, r
    
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes  


def testZControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    
    if t >= 1:
        desPos = np.array([0, 0, 1])
    
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes    
        

def TestYawControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])

    if t >= 1:
        desEul = np.array([0, 0, pi])
       
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes


def TestRollThenPitchControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    
    if t >= 1 and t < 3:
        desEul = np.array([-0.3, 0, 0])
    elif t >= 3:
        desEul = np.array([0.3, -0.3, 0])
     
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes


def TestPitchThenYawControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])
    
    if t >= 1 and t < 3:
        desEul = np.array([0, 0.4, 0])
    elif t >= 3:
        desEul = np.array([0, 0.4, pi])
     
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes


def testUVW_FlatControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, 0])
    desPQR     = np.array([0, 0, 0])

    if t >= 1 and t < 3:
        desVel = np.array([0, 0, 0])
    elif t >= 3:
        desVel = np.array([1, 0, 0])
     
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes

def testGridVelocityControl(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, pi/6])
    desPQR     = np.array([0, 0, 0])

    if t >= 1 and t < 3:
        desVelGrid = np.array([0, 0, 0])
    elif t >= 3:
        desVelGrid = np.array([4, 0, 0])
     
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes


def testXYZposition(t):
    desPos     = np.array([0, 0, 0])
    desVelGrid = np.array([0, 0, 0])
    desVel     = np.array([0, 0, 0])
    desEul     = np.array([0, 0, pi/6])
    desPQR     = np.array([0, 0, 0])
    
    if t >= 1 and t < 3:
        desPos = np.array([2, 0, 0])
    elif t >= 3:
        desPos = np.array([-2, 2, 1])
    
    sDes = makesDes(desPos, desVelGrid, desVel, desEul, desPQR)
    
    return sDes

def makesDes(desPos, desVelGrid, desVel, desEul, desPQR):
    sDes = np.zeros([15, 1])
    sDes[0] = desPos[0]
    sDes[1] = desPos[1]
    sDes[2] = desPos[2]
    sDes[3] = desEul[0]
    sDes[4] = desEul[1]
    sDes[5] = desEul[2]
    sDes[6] = desVelGrid[0]
    sDes[7] = desVelGrid[1]
    sDes[8] = desVelGrid[2]
    sDes[9] = desPQR[0]
    sDes[10] = desPQR[1]
    sDes[11] = desPQR[2]

    sDes[12] = desVel[0]
    sDes[13] = desVel[1]
    sDes[14] = desVel[2]
    
    return sDes