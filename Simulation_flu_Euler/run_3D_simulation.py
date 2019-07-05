# -*- coding: utf-8 -*-

import numpy as np
# from mixer import expoCmd
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
import utils
# import angleFunctions as af 
# from SimulationAnimation import sameAxisAnimation

trajOptions = ["position", "grid_velocity", "velocity", "altitude"]

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
    Tf = 8
    trajType = trajOptions[1]
    trajSelect = 1

    # Initialize Quadcopter, Controller, Results Matrix
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)

    t_all = Ti
    s_all = quad.state.T
    sDes_all = np.zeros([1, 15])
    cmd_all = ctrl.cmd.T
    thr_all = quad.thr.T
    

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 0
    while round(t,3) < Tf:
        
        # Quadcopter Control
        # ---------------------------
        quad_control(quad, ctrl, t, Ts, trajType, trajSelect)
                
        # Dynamics
        # ---------------------------
        quad.update(t, Ts, ctrl.cmd)
        t += Ts

        print("{:.3f}".format(t))
        t_all    = np.vstack((t_all, t))
        s_all    = np.vstack((s_all, quad.state.T))
        sDes_all = np.vstack((sDes_all, ctrl.sDesCalc.T))
        cmd_all  = np.vstack((cmd_all, ctrl.cmd.T))
        thr_all  = np.vstack((thr_all, quad.thr.T))
        i += 1
    
    # utils.fullprint(s_all)
    # utils.fullprint(sDes_all)
    # utils.fullprint(cmd_all)

    # View Results
    # ---------------------------
    utils.makeFigures(quad.params, t_all, s_all, sDes_all, cmd_all, thr_all)

if __name__ == "__main__":
    main()




# ani = sameAxisAnimation(s_all, Ts, params)
