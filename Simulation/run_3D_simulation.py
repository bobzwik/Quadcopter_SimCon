# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import trajectory as tr
from ctrl import Control
from quadFiles.quad import Quadcopter
from utils.windModel import Wind
import utils
import config
import cProfile

trajOptions = ["xyz_pos", "xy_vel_z_pos", "xyz_vel"] # "altitude", "attitude"]

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
    Tf = 10
    trajType = trajOptions[0]
    trajSelect = 0

    # Initialize Quadcopter, Controller, Result Matrixes
    # ---------------------------
    quad = Quadcopter()
    ctrl = Control(quad)
    wind = Wind()

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
        
        # Quadcopter Control
        # ---------------------------
        quad_control(quad, ctrl, t, Ts, trajType, trajSelect)
                
        # Dynamics
        # ---------------------------
        quad.update(t, Ts, ctrl.w_cmd, wind)
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
        print("{} is not a valid orientation. Verify config.py file.".format(config.orient))