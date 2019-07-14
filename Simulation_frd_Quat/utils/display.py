# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import utils

rad2deg = 180.0/pi
deg2rad = pi/180.0

def fullprint(*args, **kwargs):
    opt = np.get_printoptions()
    np.set_printoptions(threshold=np.inf)
    print(*args, **kwargs)
    np.set_printoptions(opt)

def makeFigures(params, time, pos_all, vel_all, quat_all, omega_all, euler_all, commands, wMotor_all, thrust, torque, desiredStates):
    x =     pos_all[:,0]
    y =     pos_all[:,1]
    z =     pos_all[:,2]
    q0 =    quat_all[:,0]
    q1 =    quat_all[:,1]
    q2 =    quat_all[:,2]
    q3 =    quat_all[:,3]
    xdot =  vel_all[:,0]
    ydot =  vel_all[:,1]
    zdot =  vel_all[:,2]
    p =     omega_all[:,0]*rad2deg
    q =     omega_all[:,1]*rad2deg
    r =     omega_all[:,2]*rad2deg
    w1 =    wMotor_all[:,0]
    w2 =    wMotor_all[:,1]
    w3 =    wMotor_all[:,2]
    w4 =    wMotor_all[:,3]

    phi   = euler_all[:,0]*rad2deg
    theta = euler_all[:,1]*rad2deg
    psi   = euler_all[:,2]*rad2deg

    # uFlat = extended_states[:,0]
    # vFlat = extended_states[:,1]
    # wFlat = extended_states[:,2]

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
    rDes =     desiredStates[:,11]*rad2deg

    uFlatDes = desiredStates[:,12]
    vFlatDes = desiredStates[:,13]
    wFlatDes = desiredStates[:,14]

    
    # commands = utils.expoCmd(params, commands)
    uMotor = commands*params["motorc1"] + params["motorc0"]    # Motor speed in relation to cmd
    uMotor[commands < params["motordeadband"]] = 0              # Apply motor deadband

    plt.figure()
    plt.plot(time, x, time, y, time, z)
    plt.grid(True)
    plt.legend(['x','y','z'])

    plt.figure()
    plt.plot(time, xdot, time, ydot, time, zdot)
    plt.plot(time, xdotDes, '--', time, ydotDes, '--', time, zdotDes, '--')
    plt.grid(True)
    plt.legend(['Vx','Vy','Vz'])

    # plt.figure()
    # plt.plot(time, uFlat, time, vFlat, time, wFlat)
    # plt.plot(time, uFlatDes, '--', time, vFlatDes, '--', time, wFlatDes, '--')
    # plt.grid(True)
    # plt.legend(['uFlat','vFlat','wFlat'])
    
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

    plt.figure()
    plt.plot(time, thrust[:,0], time, thrust[:,1], time, thrust[:,2], time, thrust[:,3])
    plt.grid(True)
    plt.legend(['thr1','thr2','thr3','thr4'])

    plt.figure()
    plt.plot(time, torque[:,0], time, torque[:,1], time, torque[:,2], time, torque[:,3])
    plt.grid(True)
    plt.legend(['tor1','tor2','tor3','tor4'])

    plt.figure()
    plt.plot(time, torque[:,0]+torque[:,2]-2*torque[0,0], time, -torque[:,1]-torque[:,3]+2*torque[0,0])
    plt.grid(True)
    plt.legend(['tor1+3 (difference from hover)','tor2+4 (difference from hover)'])
    
    # plt.show()
    