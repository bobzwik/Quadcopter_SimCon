# -*- coding: utf-8 -*-

# The issue with this Euler angle controller is that for a given xdot=ydot command, the pitch and roll command are equal. 
# However, according to rotation converters (https://www.andre-gaschler.com/rotationconverter/), the axis of rotation for an
# equal pitch and roll command in the order ZYX (yaw-pitch-roll) is not in the plane x-y, but also has a z component.
    # A pure pitch command would have its axis of rotation along x, and a pure roll command would have its axis of rotation along y.
    # For a equal xdot and ydot command, one would imagine the axis of rotation of the drone to be only in the x-y plane.
# But for a equal pitch and roll command, this is not the case.

import numpy as np
import matplotlib.pyplot as plt
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
import utils
# import angleFunctions as af 


trajOptions = ["position", "grid_velocity", "velocity", "altitude", "attitude"]

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
    Tf = 9
    trajType = trajOptions[0]
    trajSelect = 1

    # Initialize Quadcopter, Controller, Results Matrix
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)

    t_all     = Ti
    s_all     = quad.state.T
    ext_s_all = quad.ext_state.T
    sDes_all  = ctrl.sDesCalc.T
    cmd_all   = ctrl.cmd.T
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
        quad.update(t, Ts, ctrl.cmd)
        t += Ts

        print("{:.3f}".format(t))
        t_all     = np.vstack((t_all, t))
        s_all     = np.vstack((s_all, quad.state.T))
        ext_s_all = np.vstack((ext_s_all, quad.ext_state.T))
        sDes_all  = np.vstack((sDes_all, ctrl.sDesCalc.T))
        cmd_all   = np.vstack((cmd_all, ctrl.cmd.T))
        thr_all   = np.vstack((thr_all, quad.thr.T))
        tor_all   = np.vstack((tor_all, quad.tor.T))
        i += 1
    
    # utils.fullprint(s_all)
    # utils.fullprint(sDes_all)
    # utils.fullprint(cmd_all)

    # View Results
    # ---------------------------
    utils.makeFigures(quad.params, t_all, s_all, ext_s_all, sDes_all, cmd_all, thr_all, tor_all)
    ani = utils.sameAxisAnimation(s_all, Ts, quad.params)
    plt.show()

if __name__ == "__main__":
    main()


