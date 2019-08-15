# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
from waypoints import makeWaypoints
import config

class Trajectory:

    def __init__(self, ctrlType, trajSelect):

        self.ctrlType = ctrlType
        self.xyzType = trajSelect[0]
        self.yawType = trajSelect[1]

        t_wps, wps, y_wps, v_wp = makeWaypoints()
        self.t_wps = t_wps
        self.wps   = wps
        self.y_wps = y_wps
        self.v_wp  = v_wp

        if (self.ctrlType == "xyz_pos"):
            if (self.xyzType == 1 or self.xyzType == 2):
                self.T_segment = np.diff(self.t_wps)

            if (self.xyzType == 3):
                distance_segment = self.wps[1:] - self.wps[:-1]
                self.T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/self.v_wp
                self.t_wps = np.zeros(len(self.T_segment) + 1)
                self.t_wps[1:] = np.cumsum(self.T_segment)


    def desiredState(self, t, quad):
        
        self.desPos = np.array([0., 0., 0.])    # Desired position (x, y, z)
        self.desVel = np.array([0., 0., 0.])    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.array([0., 0., 0.])    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.array([0., 0., 0.])    # Desired thrust in N-E-D directions
        self.desEul = np.array([0., 0., 0.])    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.array([0., 0., 0.])    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.                    # Desired yaw speed
     
        
        def pos_waypoint_timed():
            
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
            elif (np.diff(self.t_wps) <= 0).any():
                raise Exception("Time array isn't properly ordered.")  
            
            if (t == 0):
                self.t_idx = 0
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
            
            self.desPos = self.wps[self.t_idx,:]
                            
        
        def pos_waypoint_interp():
            
            if not (len(self.t_wps) == self.wps.shape[0]):
                raise Exception("Time array and waypoint array not the same size.")
            elif (np.diff(self.t_wps) <= 0).any():
                raise Exception("Time array isn't properly ordered.") 

            if (t == 0):
                self.t_idx = 0
                self.desPos = self.wps[0,:]
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
                self.desPos = self.wps[-1,:]
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desPos = (1 - scale) * self.wps[self.t_idx,:] + scale * self.wps[self.t_idx + 1,:]
                    
            
        def pos_waypoint_interp_speed():
            
            if not (len(self.y_wps) == self.wps.shape[0]):
                raise Exception("Waypoint array and Yaw waypoint array not the same size.")
        
            if (t == 0):
                self.t_idx = 0
                self.desPos = self.wps[0,:]
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
                self.desPos = self.wps[-1,:]
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desPos = (1 - scale) * self.wps[self.t_idx,:] + scale * self.wps[self.t_idx + 1,:]
                    

        def yaw_waypoint_timed():
            
            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")
            
            self.desEul[2] = self.y_wps[self.t_idx]
                    

        def yaw_waypoint_interp():

            if not (len(self.t_wps) == len(self.y_wps)):
                raise Exception("Time array and waypoint array not the same size.")

            if (t == 0) or (t >= self.t_wps[-1]):
                self.desEul[2] = self.y_wps[self.t_idx]
            else:
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                self.desEul[2] = (1 - scale)*self.y_wps[self.t_idx] + scale*self.y_wps[self.t_idx + 1]


        if self.ctrlType == "xyz_vel":
            if self.xyzType == 1:
                sDes = testVelControl(t)
            return sDes

        elif self.ctrlType == "xy_vel_z_pos":
            if self.xyzType == 1:
                sDes = testVelControl(t)
            return sDes

        elif self.ctrlType == "xyz_pos":
            # Hover at [0, 0, 0]
            if self.xyzType == 0:
                pass 
            # For simple testing
            elif self.xyzType == 99:
                sDes = testXYZposition(t)   
            else:    
                # List of possible position trajectories
                # ---------------------------
                # Set desired positions at every t_wps[i]
                if self.xyzType == 1:
                    pos_waypoint_timed()
                # Interpolate position between every waypoint, to arrive at desired position every t_wps[i]
                elif self.xyzType == 2:
                    pos_waypoint_interp()
                # Interpolate position between every waypoint, to arrive at desired position every t_wps[i] (calculated using the average speed provided)
                elif self.xyzType == 3:
                    pos_waypoint_interp_speed()
                
                # List of possible yaw trajectories
                # ---------------------------
                # Set desired yaw at every t_wps[i]
                if self.yawType == 1:
                    yaw_waypoint_timed()
                # Interpolate yaw between every waypoint, to arrive at desired yaw every t_wps[i]
                elif self.yawType == 2:
                    yaw_waypoint_interp()

                sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate)).astype(float)

            return sDes




## Testing scripts

def testXYZposition(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 30.0*pi/180
    
    if t >= 1 and t < 4:
        desPos = np.array([2, 2, 1])
    elif t >= 4:
        desPos = np.array([2, -2, -2])
        desEul = np.array([0, 0, pi/3])
    
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)

    return sDes


def testVelControl(t):
    desPos = np.array([0., 0., 0.])
    desVel = np.array([0., 0., 0.])
    desAcc = np.array([0., 0., 0.])
    desThr = np.array([0., 0., 0.])
    desEul = np.array([0., 0., 0.])
    desPQR = np.array([0., 0., 0.])
    desYawRate = 0.

    if t >= 1 and t < 4:
        desVel = np.array([3, 2, 0])
    elif t >= 4:
        desVel = np.array([3, -1, 0])
     
    sDes = np.hstack((desPos, desVel, desAcc, desThr, desEul, desPQR, desYawRate)).astype(float)
    
    return sDes
