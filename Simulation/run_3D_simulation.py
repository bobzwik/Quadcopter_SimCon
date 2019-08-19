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

def quad_sim(t, Ts, quad, ctrl, wind, traj):
    
    # Dynamics (using last timestep's commands)
    # ---------------------------
    quad.update(t, Ts, ctrl.w_cmd, wind)
    t += Ts

    # Trajectory for Desired States 
    # ---------------------------
    traj.desiredState(t, Ts, quad)        

    # Generate Commands (for next iteration)
    # ---------------------------
    ctrl.controller(traj, quad, Ts)

    return t
    

def main():
    start_time = time.time()

    # Simulation Setup
    # --------------------------- 
    Ti = 0
    Ts = 0.005
    Tf = 16
    ifsave = 0

    # Choose trajectory settings
    # --------------------------- 
    ctrlOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"]
    trajSelect = np.zeros(3)

    # Select Control Type             (0: xyz_pos,         1: xy_vel_z_pos,          2: xyz_vel)
    ctrlType = ctrlOptions[0]   
    # Select Position Trajectory Type (0: hover,           1: pos_waypoint_timed,    2: pos_waypoint_interp,    3: minimum velocity
    #                                  4: minimum accel,   5: minimum jerk,          6: minimum snap
    trajSelect[0] = 6          
    # Select Yaw Trajectory Type      (0: none             1: yaw_waypoint_timed,    2: yaw_waypoint_interp)
    trajSelect[1] = 3           
    # Select if waypoint time is used, or if average speed is used to calculate waypoint time   (0: waypoint time,   1: average speed)
    trajSelect[2] = 0           
    print("Control type: {}".format(ctrlType))

    # Initialize Quadcopter, Controller, Wind, Result Matrixes
    # ---------------------------
    quad = Quadcopter(Ti)
    traj = Trajectory(quad, ctrlType, trajSelect)
    ctrl = Control(quad, traj.yawType)
    wind = Wind('None', 2.0, 90, -15)

    # Trajectory for First Desired States
    # ---------------------------
    traj.desiredState(0, Ts, quad)        

    # Generate First Commands
    # ---------------------------
    ctrl.controller(traj, quad, Ts)
    
    # Initialize Result Matrixes
    # ---------------------------
    t_all         = Ti
    s_all         = quad.state.T
    pos_all       = quad.pos.T
    vel_all       = quad.vel.T
    quat_all      = quad.quat.T
    omega_all     = quad.omega.T
    euler_all     = quad.euler.T
    sDes_traj_all = traj.sDes.T
    sDes_calc_all = ctrl.sDesCalc.T
    w_cmd_all     = ctrl.w_cmd.T
    wMotor_all    = quad.wMotor.T
    thr_all       = quad.thr.T
    tor_all       = quad.tor.T

    # Run Simulation
    # ---------------------------
    t = Ti
    i = 0
    while round(t,3) < Tf:
        
        t = quad_sim(t, Ts, quad, ctrl, wind, traj)
        
        # print("{:.3f}".format(t))
        t_all           = np.vstack((t_all, t))
        s_all           = np.vstack((s_all, quad.state.T))
        pos_all         = np.vstack((pos_all, quad.pos.T))
        vel_all         = np.vstack((vel_all, quad.vel.T))
        quat_all        = np.vstack((quat_all, quad.quat.T))
        omega_all       = np.vstack((omega_all, quad.omega.T))
        euler_all       = np.vstack((euler_all, quad.euler.T))
        sDes_traj_all   = np.vstack((sDes_traj_all, traj.sDes.T))
        sDes_calc_all   = np.vstack((sDes_calc_all, ctrl.sDesCalc.T))
        w_cmd_all       = np.vstack((w_cmd_all, ctrl.w_cmd.T))
        wMotor_all      = np.vstack((wMotor_all, quad.wMotor.T))
        thr_all         = np.vstack((thr_all, quad.thr.T))
        tor_all         = np.vstack((tor_all, quad.tor.T))
        i += 1
    
    end_time = time.time()
    print("Simulated {:.2f}s in {:.6f}s.".format(t, end_time - start_time))

    # View Results
    # ---------------------------

    # utils.fullprint(sDes_traj_all[:,3:6])
    utils.makeFigures(quad.params, t_all, pos_all, vel_all, quat_all, omega_all, euler_all, w_cmd_all, wMotor_all, thr_all, tor_all, sDes_traj_all, sDes_calc_all)
    ani = utils.sameAxisAnimation(t_all, traj.wps, pos_all, quat_all, sDes_traj_all, Ts, quad.params, traj.xyzType, traj.yawType, ifsave)
    plt.show()

if __name__ == "__main__":
    if (config.orient == "NED" or config.orient == "ENU"):
        main()
        # cProfile.run('main()')
    else:
        raise Exception("{} is not a valid orientation. Verify config.py file.".format(config.orient))