# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from scipy.integrate import odeint
# from mixer import expoCmd
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
# import angleFunctions as af 
# from SimulationAnimation import sameAxisAnimation

def quad_control(quad, ctrl, t, Ts, trajType, trajSelect):
    
    # Trajectory for Desired States
    # ---------------------------
    sDes = tr.desiredState(t, trajType, trajSelect)
    
    # Generate Commands
    # ---------------------------
    ctrl.controller(quad, sDes, Ts, trajType, trajSelect)

def main():

    # Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005
    Tf = 6
    maxIter = Tf/Ts
    trajType = "altitude"
    trajSelect = 2

    # MAKE CLASS FOR CONTROL

    # Initialize Quadcopter, Controller Results Matrix
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)

    cmdVect = np.transpose(ctrl.cmd)
    s_result = quad.state.T
    t_result = Ti
    sDesVector = np.zeros([1, 12])

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 0
    while round(t,3) < Tf:
        # Quadcopter Control
        # ---------------------------
        quad_control(quad, ctrl, t, Ts, trajType, trajSelect)
        cmd = ctrl.cmd
        cmdVect = np.vstack((cmdVect, ctrl.cmd.T))
        sDesVector = np.vstack((sDesVector,ctrl.sDesCalc.T))
        
        # Dynamics
        # ---------------------------
        # inputs = (cmd, params)
        # s = odeint(dynamics,s,[t,t+Ts], args=inputs)
        # s = s[1]
        t += Ts
        
        print("{:.3f}".format(t))
        # s_result = np.vstack((s_result,np.transpose(s)))
        # t_result = np.vstack((t_result,t))
        # i += 1
    print(cmdVect)
    print(sDesVector)

if __name__ == "__main__":
    main()




# """ Run Simulation """
# t = Ti
# i = 0
# while round(t,3) < Tf:
#     # Trajectory for Desired States
#     # ---------------------------
#     if   trajSelect == 0:
#         sDes = tr.hover(t)
#     elif trajSelect == 1:
#         sDes = tr.testZControl(t)
#     elif trajSelect == 2:
#         sDes = tr.TestYawControl(t)
#     elif trajSelect == 3:
#         sDes = tr.TestRollThenPitchControl(t)
#     elif trajSelect == 4:    
#         sDes = tr.TestPitchThenYawControl(t)
#     elif trajSelect == 5:
#         sDes = tr.XYZposition(t)
#     # Generate Commands
#     # ---------------------------
#     if controlNum == 1:
#         cmd = ctrl.control1(s, sDes, params, t, cmd)
#     else:
#         if controlNum == 2:
#             output = ctrl.attitude(s, sDes, prevError, params, Ts)
#         elif controlNum == 3:
#             output = ctrl.position(s, sDes, prevError, params, Ts)
#         cmd = output[0:4]
#         prevError = output[4:16]
#         sDesCalc = output[16:]
#         sDesVector = np.vstack((sDesVector,np.transpose(sDesCalc)))
          
#     cmdVect = np.vstack((cmdVect, np.transpose(cmd)))
#     cmd = expoCmd(params, cmd)       # This applies a Square Root Expo on the command
    
    
    
    
# """ View Results """
# x =     s_result[:,0]
# y =     s_result[:,1]
# z =     s_result[:,2]
# phi =   s_result[:,3]*180/pi
# theta = s_result[:,4]*180/pi
# psi =   s_result[:,5]*180/pi
# xdot =  s_result[:,6]
# ydot =  s_result[:,7]
# zdot =  s_result[:,8]
# p =     s_result[:,9]
# q =     s_result[:,10]
# r =     s_result[:,11]
# w1 =    s_result[:,12]
# w2 =    s_result[:,14]
# w3 =    s_result[:,16]
# w4 =    s_result[:,18]

# xDes =     sDesVector[:,0]
# yDes =     sDesVector[:,1]
# zDes =     sDesVector[:,2]
# phiDes =   sDesVector[:,3]*180/pi
# thetaDes = sDesVector[:,4]*180/pi
# psiDes =   sDesVector[:,5]*180/pi
# xdotDes =  sDesVector[:,6]
# ydotDes =  sDesVector[:,7]
# zdotDes =  sDesVector[:,8]
# pDes =     sDesVector[:,9]
# qDes =     sDesVector[:,10]
# rDes =     sDesVector[:,11] 

   
# cmdVect_expo = expoCmd(params, cmdVect)
# uMotor = cmdVect_expo*params["motorc1"] + params["motorc0"]    # Motor speed in relation to cmd
# uMotor[cmdVect_expo < params["motordeadband"]] = 0              # Apply motor deadband

# plt.figure()
# plt.plot(t_result, x, t_result, y, t_result, z)
# plt.grid(True)
# plt.legend(['x','y','z'])

# plt.figure()
# plt.plot(t_result, xdot, t_result, ydot, t_result, zdot)
# plt.grid(True)
# plt.legend(['Vx','Vy','Vz'])

# plt.figure()
# plt.plot(t_result, phi, t_result, theta, t_result, psi)
# plt.plot(t_result, phiDes, '--', t_result, thetaDes, '--', t_result, psiDes, '--')
# plt.grid(True)
# plt.legend(['roll','pitch','yaw'])

# plt.figure()
# plt.plot(t_result, p, t_result, q, t_result, r)
# plt.plot(t_result, pDes, '--', t_result, qDes, '--', t_result, rDes, '--')
# plt.grid(True)
# plt.legend(['p','q','r'])

# plt.figure()
# plt.plot(t_result, w1, t_result, w2, t_result, w3, t_result, w4)
# #plt.plot(t_result, uMotor[:,0], '--', t_result, uMotor[:,1], '--', t_result, uMotor[:,2], '--', t_result, uMotor[:,3], '--')
# plt.grid(True)
# plt.legend(['w1','w2','w3','w4'])

# ani = sameAxisAnimation(s_result, Ts, params)
