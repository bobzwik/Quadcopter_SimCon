# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import cProfile

from trajectory import Trajectory
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config

ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
trajSelect = np.ones(2)

def quad_sim(t, Ts, quad, ctrl, wind, traj):
    
    # Trajectory for Desired States
    # ---------------------------
    sDes = traj.desiredState(t, quad)        
    
    # Generate Commands
    # ---------------------------
    ctrl.controller(quad, sDes, Ts, traj)

    # Dynamics
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)


def main():
    start_time = time.time()

    # Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005
    Tf = 16
    ctrlType = ctrlOptions[0]
    trajSelect[0] = 2       # Position trajectory selection
    trajSelect[1] = 1       # Yaw trajectory selection
    print("Control type: {}".format(ctrlType))

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(Ti)
    traj = Trajectory(ctrlType, trajSelect)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)

    t_all     = Ti
    s_all     = quad.state.T
    pos_all   = quad.pos.T
    vel_all   = quad.vel.T
    quat_all  = quad.quat.T
    omega_all = quad.omega.T
    euler_all = quad.euler.T
    sDes_all  = ctrl.sDesCalc.T
    w_cmd_all = ctrl.w_cmd.T
    wMotor_all= quad.wMotor.T
    thr_all   = quad.thr.T
    tor_all   = quad.tor.T

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 0
    while round(t,3) < Tf:
        
        quad_sim(t, Ts, quad, ctrl, wind, traj)
        t += Ts

        # print("{:.3f}".format(t))
        t_all     = np.vstack((t_all, t))
        s_all     = np.vstack((s_all, quad.state.T))
        pos_all   = np.vstack((pos_all, quad.pos.T))
        vel_all   = np.vstack((vel_all, quad.vel.T))
        quat_all  = np.vstack((quat_all, quad.quat.T))
        omega_all = np.vstack((omega_all, quad.omega.T))
        euler_all = np.vstack((euler_all, quad.euler.T))
        sDes_all  = np.vstack((sDes_all, ctrl.sDesCalc.T))
        w_cmd_all = np.vstack((w_cmd_all, ctrl.w_cmd.T))
        wMotor_all= np.vstack((wMotor_all, quad.wMotor.T))
        thr_all   = np.vstack((thr_all, quad.thr.T))
        tor_all   = np.vstack((tor_all, quad.tor.T))
        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))

    # View Results
    # ---------------------------
    utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_all)
    ani = utils.sameAxisAnimation(t_all, pos_all, quat_all, Ts, quad.params)
    plt.show()

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):
        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))