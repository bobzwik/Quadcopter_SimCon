# -*- coding: utf-8 -*-

# Position and Velocity Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/PositionControl.cpp
# Desired Thrust to Desired Attitude based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_pos_control/Utility/ControlMath.cpp
# Attitude Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp
# and https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
# Rate Control based on https://github.com/PX4/Firmware/blob/master/src/modules/mc_att_control/mc_att_control_main.cpp

import numpy as np
from numpy import pi
from numpy import sin, cos, tan, sqrt
from numpy.linalg import norm
import utils
import config

rad2deg = 180.0/pi
deg2rad = pi/180.0

Py    = 1.0
Pxdot = 5.0
Dxdot = 0.5

Px    = Py
Pydot = Pxdot
Dydot = Dxdot

Pz    = 2.0
Pzdot = 6.0
Dzdot = 0.2

pos_P_gain = np.array([Px, Py, Pz])
vel_P_gain = np.array([Pxdot, Pydot, Pzdot])
vel_D_gain = np.array([Dxdot, Dydot, Dzdot])

Pphi = 8.0
Pp = 1.5
Dp = 0.04

Ptheta = Pphi
Pq = Pp
Dq = Dp 

Ppsi = 0.8
Pr = 1.0
Dr = 0.1

att_P_gain = np.array([Pphi, Ptheta, Ppsi])
rate_P_gain = np.array([Pp, Pq, Pr])
rate_D_gain = np.array([Dp, Dq, Dr])

uMax = 3.0
vMax = 3.0
wMax = 3.0

velMax = np.array([uMax, vMax, wMax])

tiltMax = 25.0*deg2rad

pMax = 200.0*deg2rad
qMax = 200.0*deg2rad
rMax = 100.0*deg2rad

rateMax = np.array([pMax, qMax, rMax])

class Control:
    
    def __init__(self, quad):
        self.sDesCalc = np.zeros(16)
        self.w_cmd = np.ones(4)*quad.params["w_hover"]
        self.setYawWeight()

    
    def controller(self, quad, sDes, Ts, trajType, trajSelect):

        # Desired State
        # ---------------------------
        self.pos_sp = sDes[0:3]
        self.eul_sp = sDes[3:6]
        self.vel_sp = sDes[6:9]
        self.thrust_sp = sDes[12:15]


        # Select Controller
        # ---------------------------
        # if trajType == "attitude":
        #     self.attitude_control(quad, Ts)
            # self.rate_control(quad, Ts)
        # elif trajType == "altitude":
        #     self.z_pos_control(quad, Ts)
        #     self.z_vel_control(quad, Ts)
            # self.attitude(quad, Ts)
            # self.rate(quad, Ts)
        if trajType == "velocity":
            self.z_pos_control(quad, Ts)
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude_control(quad, Ts)
            self.rate_control(quad, Ts)
        elif trajType == "position":
            self.z_pos_control(quad, Ts)
            self.xy_pos_control(quad, Ts)    
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude_control(quad, Ts)
            self.rate_control(quad, Ts)

        # Mixer
        # --------------------------- 
        self.w_cmd = utils.mixerFM(quad, norm(self.thrust_sp), self.rateCtrl)

        # Add calculated Desired States
        # ---------------------------         
        self.sDesCalc[0:3] = self.pos_sp
        self.sDesCalc[3:7] = self.qd
        self.sDesCalc[7:10] = self.vel_sp
        self.sDesCalc[10:13] = self.rate_sp
        self.sDesCalc[13:16] = self.thrust_sp


    def z_pos_control(self, quad, Ts):
       
        # Z Position Control
        # --------------------------- 
        pos_z_error = self.pos_sp[2] - quad.pos[2]
        self.vel_sp[2] = pos_P_gain[2]*pos_z_error + self.vel_sp[2]
        
        # Saturate velocity setpoint
        self.vel_sp[2] = np.clip(self.vel_sp[2], -velMax[2], velMax[2])

    
    def xy_pos_control(self, quad, Ts):

        # XY Position Control
        # --------------------------- 
        pos_xy_error = (self.pos_sp[0:2] - quad.pos[0:2])
        self.vel_sp[0:2] = pos_P_gain[0:2]*pos_xy_error + self.vel_sp[0:2]
        
        # Saturate velocity setpoint
        self.vel_sp[0:2] = np.clip(self.vel_sp[0:2], -velMax[0:2], velMax[0:2])
        

    def z_vel_control(self, quad, Ts):
        
        # Z Velocity Control (Thrust in D-direction)
        # ---------------------------
        vel_z_error = self.vel_sp[2] - quad.vel[2]
        if (config.orient == "NED"):
            thrust_z_sp = vel_P_gain[2]*vel_z_error - vel_D_gain[2]*quad.vel_dot[2] - quad.params["mB"]*quad.params["g"]
        elif (config.orient == "ENU"):
            thrust_z_sp = vel_P_gain[2]*vel_z_error - vel_D_gain[2]*quad.vel_dot[2] + quad.params["mB"]*quad.params["g"]
        
        # Saturate thrust setpoint in D-direction
        if (config.orient == "NED"):
            # The Thrust limits are negated and swapped due to NED-frame
            self.thrust_sp[2] = np.clip(thrust_z_sp, -quad.params["maxThr"], -quad.params["minThr"])
        elif (config.orient == "ENU"):
            self.thrust_sp[2] = np.clip(thrust_z_sp,  quad.params["minThr"],  quad.params["maxThr"])

    
    def xy_vel_control(self, quad, Ts):
        
        # XY Velocity Control (Thrust in NE-direction)
        # ---------------------------
        vel_xy_error = self.vel_sp[0:2] - quad.vel[0:2]
        thrust_xy_sp = vel_P_gain[0:2]*vel_xy_error - vel_D_gain[0:2]*quad.vel_dot[0:2]

        # Max allowed thrust in NE based on tilt and excess thrust
        thrust_max_xy_tilt = abs(self.thrust_sp[2])*np.tan(tiltMax)
        thrust_max_xy = sqrt(quad.params["maxThr"]**2 - self.thrust_sp[2]**2)
        thrust_max_xy = min(thrust_max_xy, thrust_max_xy_tilt)

        # Saturate thrust in NE-direction
        self.thrust_sp[0:2] = thrust_xy_sp
        if (np.dot(self.thrust_sp[0:2].T, self.thrust_sp[0:2]) > thrust_max_xy**2):
            mag = norm(self.thrust_sp[0:2])
            self.thrust_sp[0:2] = thrust_xy_sp/mag*thrust_max_xy

    
    def thrustToAttitude(self, quad, Ts):
        
        # Create Full Desired Quaternion Based on Thrust Setpoint and Desired Yaw Angle
        # ---------------------------
        yaw_sp = self.eul_sp[2]

        # Desired body_z axis direction
        body_z = -utils.vectNormalize(self.thrust_sp)
        if (config.orient == "ENU"):
            body_z = -body_z
        
        # Vector of desired Yaw direction in XY plane, rotated by pi/2 (fake body_y axis)
        y_C = np.array([-sin(yaw_sp), cos(yaw_sp), 0.0])
        
        # Desired body_x axis direction
        body_x = np.cross(y_C, body_z)
        body_x = utils.vectNormalize(body_x)
        
        # Desired body_y axis direction
        body_y = np.cross(body_z, body_x)

        # Desired rotation matrix
        R_sp = np.array([body_x, body_y, body_z]).T

        # Full desired quaternion (full because it considers the desired Yaw angle)
        self.qd_full = utils.RotToQuat(R_sp)
        
        
    def attitude_control(self, quad, Ts):

        # Current thrust orientation e_z and desired thrust orientation e_z_d
        e_z = quad.dcm[:,2]
        e_z_d = -utils.vectNormalize(self.thrust_sp)
        if (config.orient == "ENU"):
            e_z_d = -e_z_d

        # Quaternion error between the 2 vectors
        qe_red = np.zeros(4)
        qe_red[0] = np.dot(e_z, e_z_d) + sqrt(norm(e_z)**2 * norm(e_z_d)**2)
        qe_red[1:4] = np.cross(e_z, e_z_d)
        qe_red = utils.vectNormalize(qe_red)
        
        # Reduced desired quaternion (reduced because it doesn't consider the desired Yaw angle)
        self.qd_red = utils.quatMultiply(quad.quat, qe_red)
        
        # Mixed desired quaternion (between reduced and full) and resulting desired quaternion qd
        q_mix = utils.quatMultiply(utils.inverse(self.qd_red), self.qd_full)
        q_mix = q_mix*np.sign(q_mix[0])
        q_mix[0] = np.clip(q_mix[0], -1.0, 1.0)
        q_mix[3] = np.clip(q_mix[3], -1.0, 1.0)
        self.qd = utils.quatMultiply(self.qd_red, np.array([cos(self.yaw_w*np.arccos(q_mix[0])), 0, 0, sin(self.yaw_w*np.arcsin(q_mix[3]))]))

        # Resulting error quaternion
        self.qe = utils.quatMultiply(utils.inverse(quad.quat), self.qd)

        # Create rate setpoint from quaternion error
        self.rate_sp = (2.0*np.sign(self.qe[0])*self.qe[1:4])*att_P_gain
        self.rate_sp = np.clip(self.rate_sp, -rateMax, rateMax)
        # If needed to feed-forward Yaw rate setpoint, this is where i should. See AttitudeControl.cpp from PX4


    def rate_control(self, quad, Ts):
        
        # Rate Control
        # ---------------------------
        rate_error = self.rate_sp - quad.omega
        self.rateCtrl = rate_P_gain*rate_error - rate_D_gain*quad.omega_dot     # Be sure it is right sign for the D part


    def setYawWeight(self):
        
        # Calculate weight of the Yaw control gain
        roll_pitch_gain = 0.5*(att_P_gain[0] + att_P_gain[1])
        self.yaw_w = np.clip(att_P_gain[2]/roll_pitch_gain, 0.0, 1.0)

        att_P_gain[2] = roll_pitch_gain

    