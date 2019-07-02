# -*- coding: utf-8 -*-

import numpy as np
# from mixer import expoCmd
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
import utils
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
    trajSelect = 4

    # Initialize Quadcopter, Controller, Results Matrix
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)

    t_result = Ti
    s_result = quad.state.T
    sDesVector = np.zeros([1, 12])
    cmdVect = ctrl.cmd.T
    

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 0
    while round(t,3) < Tf:
        # Quadcopter Control
        # ---------------------------
        quad_control(quad, ctrl, t, Ts, trajType, trajSelect)
        cmdVect = np.vstack((cmdVect, ctrl.cmd.T))
        sDesVector = np.vstack((sDesVector,ctrl.sDesCalc.T))
        
        # Dynamics
        # ---------------------------
        quad.update(t, Ts, ctrl.cmd)
        t += Ts

        print("{:.3f}".format(t))
        s_result = np.vstack((s_result, quad.state.T))
        t_result = np.vstack((t_result, t))
        i += 1
    
    # utils.fullprint(s_result)
    # utils.fullprint(sDesVector)
    # utils.fullprint(cmdVect)

    # View Results
    # ---------------------------
    utils.makeFigures(quad.params, t_result, s_result, sDesVector, cmdVect)

if __name__ == "__main__":
    main()




# ani = sameAxisAnimation(s_result, Ts, params)
