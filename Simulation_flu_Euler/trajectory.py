# -*- coding: utf-8 -*-

import numpy as np

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

    return sDes


def hover(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])
    
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes  


def testZControl(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])
    
    if t >= 1:
        desPos = np.array([0, 0, 1])
    
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes    
        

def TestYawControl(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])

    if t >= 1:
        desEul = np.array([0, 0, np.pi])
       
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes


def TestRollThenPitchControl(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])
    if t >= 1 and t < 3:
        desEul = np.array([0.3, 0, 0])
    elif t >= 3:
        desEul = np.array([0.3, 0.3, 0])
     
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes


def TestPitchThenYawControl(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])
    if t >= 1 and t < 3:
        desEul = np.array([0, 0.4, 0])
    elif t >= 3:
        desEul = np.array([0, 0.4, np.pi])
     
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes

def XYZposition(t):
    desPos = np.array([0, 0, 0])
    desVel = np.array([0, 0, 0])
    desEul = np.array([0, 0, 0])
    desPQR = np.array([0, 0, 0])
    if t >= 1 and t < 3:
        desPos = np.array([2, 2, 2])
    elif t >= 3:
        desPos = np.array([2, 3, 2])
    
    sDes = makesDes(desPos, desVel, desEul, desPQR)
    
    return sDes

def makesDes(desPos, desVel, desEul, desPQR):
    sDes = np.zeros([12, 1])
    sDes[0] = desPos[0]
    sDes[1] = desPos[1]
    sDes[2] = desPos[2]
    sDes[3] = desEul[0]
    sDes[4] = desEul[1]
    sDes[5] = desEul[2]
    sDes[6] = desVel[0]
    sDes[7] = desVel[1]
    sDes[8] = desVel[2]
    sDes[9] = desPQR[0]
    sDes[10] = desPQR[1]
    sDes[11] = desPQR[2]
    
    return sDes