# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt

rad2deg = 180.0/pi
deg2rad = pi/180.0

def fullprint(*args, **kwargs):
    opt = np.get_printoptions()
    np.set_printoptions(threshold=np.inf)
    print(*args, **kwargs)
    np.set_printoptions(opt)

def makeFigures(params, time, states, desiredStates, commands):
    x =     states[:,0]
    y =     states[:,1]
    z =     states[:,2]
    phi =   states[:,3]*rad2deg
    theta = states[:,4]*rad2deg
    psi =   states[:,5]*rad2deg
    xdot =  states[:,6]
    ydot =  states[:,7]
    zdot =  states[:,8]
    p =     states[:,9]*rad2deg
    q =     states[:,10]*rad2deg
    r =     states[:,11]*rad2deg
    w1 =    states[:,12]
    w2 =    states[:,14]
    w3 =    states[:,16]
    w4 =    states[:,18]

    xDes =     desiredStates[:,0]
    yDes =     desiredStates[:,1]
    zDes =     desiredStates[:,2]
    phiDes =   desiredStates[:,3]*rad2deg
    thetaDes = desiredStates[:,4]*rad2deg
    psiDes =   desiredStates[:,5]*rad2deg
    xdotDes =  desiredStates[:,6]
    ydotDes =  desiredStates[:,7]
    zdotDes =  desiredStates[:,8]
    pDes =     desiredStates[:,9]*rad2deg
    qDes =     desiredStates[:,10]*rad2deg
    rDes =     desiredStates[:,11] *rad2deg

    
    #cmdVect_expo = expoCmd(params, cmdVect)
    uMotor = commands*params["motorc1"] + params["motorc0"]    # Motor speed in relation to cmd
    uMotor[commands < params["motordeadband"]] = 0              # Apply motor deadband

    plt.figure()
    plt.plot(time, x, time, y, time, z)
    plt.grid(True)
    plt.legend(['x','y','z'])

    plt.figure()
    plt.plot(time, xdot, time, ydot, time, zdot)
    plt.grid(True)
    plt.legend(['Vx','Vy','Vz'])
    
    plt.figure()
    plt.plot(time, phi, time, theta, time, psi)
    plt.plot(time, phiDes, '--', time, thetaDes, '--', time, psiDes, '--')
    plt.grid(True)
    plt.legend(['roll','pitch','yaw'])
    
    plt.figure()
    plt.plot(time, p, time, q, time, r)
    plt.plot(time, pDes, '--', time, qDes, '--', time, rDes, '--')
    plt.grid(True)
    plt.legend(['p','q','r'])
    
    plt.figure()
    plt.plot(time, w1, time, w2, time, w3, time, w4)
    plt.plot(time, uMotor[:,0], '--', time, uMotor[:,1], '--', time, uMotor[:,2], '--', time, uMotor[:,3], '--')
    plt.grid(True)
    plt.legend(['w1','w2','w3','w4'])
    
    plt.show()
    