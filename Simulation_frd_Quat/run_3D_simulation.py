# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
import utils
# import angleFunctions as af 


trajOptions = ["position", "velocity", "altitude", "attitude"]

def quad_control(quad, ctrl, t, Ts, trajType, trajSelect):
    
    # Trajectory for Desired States
    # ---------------------------
    sDes = tr.desiredState(t, trajType, trajSelect, quad)
    
    # Generate Commands
    # ---------------------------
    ctrl.controller(quad, sDes, Ts, trajType, trajSelect)

def main():

    # Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005
    Tf = 6
    trajType = trajOptions[0]
    trajSelect = 1

    # Initialize Quadcopter, Controller, Results Matrix
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)

    t_all     = Ti
    s_all     = quad.state.T
    pos_all   = quad.pos.T
    vel_all   = quad.vel.T
    quat_all  = quad.quat.T
    omega_all = quad.omega.T
    euler_all = quad.euler.T
    sDes_all  = ctrl.sDesCalc.T
    cmd_all   = ctrl.cmd.T
    wMotor_all= quad.wMotor.T
    thr_all   = quad.thr.T
    tor_all   = quad.tor.T
    

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

        # ctrl.cmd = np.array([52,51,52,51])

        quad.update(t, Ts, ctrl.cmd)
        t += Ts

        print("{:.3f}".format(t))
        t_all     = np.vstack((t_all, t))
        s_all     = np.vstack((s_all, quad.state.T))
        pos_all   = np.vstack((pos_all, quad.pos.T))
        vel_all   = np.vstack((vel_all, quad.vel.T))
        quat_all  = np.vstack((quat_all, quad.quat.T))
        omega_all = np.vstack((omega_all, quad.omega.T))
        euler_all = np.vstack((euler_all, quad.euler.T))
        sDes_all  = np.vstack((sDes_all, ctrl.sDesCalc.T))
        cmd_all   = np.vstack((cmd_all, ctrl.cmd.T))
        wMotor_all= np.vstack((wMotor_all, quad.wMotor.T))
        thr_all   = np.vstack((thr_all, quad.thr.T))
        tor_all   = np.vstack((tor_all, quad.tor.T))
        i += 1
    
    # utils.fullprint(s_all)
    # utils.fullprint(sDes_all)
    # utils.fullprint(cmd_all)

    # View Results
    # ---------------------------
    utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, cmd_all, wMotor_all, thr_all, tor_all, sDes_all)
    # ani = utils.sameAxisAnimation(s_all, Ts, quad.params)
    plt.show()

if __name__ == "__main__":
    main()

