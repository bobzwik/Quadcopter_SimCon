# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
from numpy import sin, cos, tan, sqrt
from numpy.linalg import norm
# import angleFunctions as af
import utils

rad2deg = 180.0/pi
deg2rad = pi/180.0

# Px = 1.6
# Pu = 1.1
# Du = 0.15

# Py = 1.6
# Pv = 1.1
# Dv = 0.15

# Pz = 8.0
# Pw = 15.0
# Dw = 0.15

# Puv = 0.9
# Duv = 0.0001

Py    = 1.0
Pxdot = 1.0
Dxdot = 0.1

Px    = 1.0
Pydot = 1.0
Dydot = 0.1

Pz    = 1.0
Pzdot = 1.0
Dzdot = 0.1

pos_P_gain = np.array([Px, Py, Pz])
vel_P_gain = np.array([Pxdot, Pydot, Pzdot])
vel_D_gain = np.array([Dxdot, Dydot, Dzdot])

Pphi = 8.0
Pp = 4.0
Dp = 0.1

Ptheta = Pphi
Pq = Pp
Dq = Dp 

Ppsi = 6.0
Pr = 15.0
Dr = 0.4

uMax = 3
vMax = 3
wMax = 3

velMax = np.array([uMax, vMax, wMax])

phiMax = 20*deg2rad
thetaMax = 20*deg2rad
tiltMax = 22*deg2rad

pMax = 200*deg2rad
qMax = 200*deg2rad
rMax = 200*deg2rad


class Control:
    
    def __init__(self, quad):
        self.sDesCalc = np.zeros(15)
        self.cmd = np.ones(4)*quad.params["FF"]   # Motor 1 is front left, then clockwise numbering
        self.xPrevE     = 0
        self.yPrevE     = 0
        self.zPrevE     = 0
        self.phiPrevE   = 0
        self.thetaPrevE = 0
        self.psiPrevE   = 0
        self.xdotPrevE  = 0
        self.ydotPrevE  = 0
        self.zdotPrevE  = 0
        self.pPrevE     = 0
        self.qPrevE     = 0
        self.rPrevE     = 0

        self.pos_PrevE  = np.zeros(3)
        self.vel_PrevE  = np.zeros(3)

        # self.uFlatPrevE = 0
        # self.vFlatPrevE = 0
        # self.velFlatPrevE = 0

    
    def controller(self, quad, sDes, Ts, trajType, trajSelect):
        
        # Desired State
        # ---------------------------
        self.xDes     = sDes[0]
        self.yDes     = sDes[1]
        self.zDes     = sDes[2]
        self.phiDes   = sDes[3]
        self.thetaDes = sDes[4]
        self.psiDes   = sDes[5]
        self.xdotDes  = sDes[6]
        self.ydotDes  = sDes[7]
        self.zdotDes  = sDes[8]
        self.pDes     = sDes[9]
        self.qDes     = sDes[10]
        self.rDes     = sDes[11]

        self.pos_sp = sDes[0:3]
        self.eul_sp = sDes[3:6]
        self.vel_sp = sDes[6:9]
        self.thrust_sp = sDes[12:15]


        # Select Controller
        # ---------------------------
        self.zCmd = quad.params["FF"]
        self.pCmd = 0
        self.qCmd = 0
        self.rCmd = 0

        # if trajType == "attitude":
            # self.attitude(quad, Ts)
            # self.rate(quad, Ts)
        if trajType == "altitude":
            self.z_pos_control(quad, Ts)
            self.z_vel_control(quad, Ts)
            # self.attitude(quad, Ts)
            # self.rate(quad, Ts)
        if trajType == "velocity":
            self.z_pos_control(quad, Ts)
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            # self.attitude(quad, Ts)
            # self.rate(quad, Ts)
        if trajType == "position":
            self.z_pos_control(quad, Ts)
            self.xy_pos_control(quad, Ts)    
            self.z_vel_control(quad, Ts)
            self.xy_vel_control(quad, Ts)
            self.thrustToAttitude(quad, Ts)
            self.attitude(quad, Ts)
            # self.rate(quad, Ts)

        # Mixer
        # --------------------------- 
        self.cmd = utils.mixer(self.zCmd, self.pCmd, self.qCmd, self.rCmd, quad)

        # Add Exponential to command
        # ---------------------------
        self.cmd = utils.expoCmd(quad.params, self.cmd)

        # Add calculated Desired States
        # ---------------------------         
        # self.sDesCalc[0] = self.xDes
        # self.sDesCalc[1] = self.yDes
        # self.sDesCalc[2] = self.zDes
        self.sDesCalc[3] = self.phiDes
        self.sDesCalc[4] = self.thetaDes
        self.sDesCalc[5] = self.psiDes
        # self.sDesCalc[6] = self.xdotDes
        # self.sDesCalc[7] = self.ydotDes
        # self.sDesCalc[8] = self.zdotDes
        self.sDesCalc[9] = self.pDes
        self.sDesCalc[10] = self.qDes
        self.sDesCalc[11] = self.rDes

        self.sDesCalc[0:3] = self.pos_sp
        self.sDesCalc[6:9] = self.vel_sp

        self.sDesCalc[12] = self.thrust_sp[0]
        self.sDesCalc[13] = self.thrust_sp[1]
        self.sDesCalc[14] = self.thrust_sp[2]


    def z_pos_control(self, quad, Ts):
       
        # Z Position Control
        # --------------------------- 
        pos_z_error = self.pos_sp[2] - quad.pos[2]
        self.vel_sp[2] = pos_P_gain[2]*pos_z_error + self.vel_sp[2]         # Be careful to reset vel_sp for next iteration
        self.vel_sp[2] = np.clip(self.vel_sp[2], -velMax[2], velMax[2])

        # Replace Previous Error
        self.pos_PrevE[2] = pos_z_error

    
    def xy_pos_control(self, quad, Ts):

        # XY Position Control
        # --------------------------- 
        pos_xy_error = (self.pos_sp[0:2] - quad.pos[0:2])
        self.vel_sp[0:2] = pos_P_gain[0:2]*pos_xy_error + self.vel_sp[0:2]  # Be careful to reset vel_sp for next iteration
        self.vel_sp[0:2] = np.clip(self.vel_sp[0:2], -velMax[0:2], velMax[0:2])

        # Replace Previous Error
        self.pos_PrevE[0:2] = pos_xy_error
        

    def z_vel_control(self, quad, Ts):
        
        # Z Velocity Control (Thrust in D-direction)
        # ---------------------------
        vel_z_error = self.vel_sp[2] - quad.vel[2]
        thrust_z_sp = vel_P_gain[2]*vel_z_error + vel_D_gain[2]*quad.vel_dot[2] - quad.params["thr_hover"]
        
        # The Thrust limits are negated and swapped due to NED-frame
        uMax = -quad.params["minThr"]
        uMin = -quad.params["maxThr"]

        # Saturate thrust setpoint in D-direction
        self.thrust_sp[2] = np.clip(thrust_z_sp, uMin, uMax)

        # Replace Previous Error
        self.vel_PrevE[2] = vel_z_error

    
    def xy_vel_control(self, quad, Ts):
        
        # XY Velocity Control (Thrust in NE-direction)
        # ---------------------------
        vel_xy_error = self.vel_sp[0:2] - quad.vel[0:2]
        thrust_xy_sp = vel_P_gain[0:2]*vel_xy_error + vel_D_gain[0:2]*quad.vel_dot[0:2]

        # Max allowed thrust in NE based on tilt and excess thrust
        thrust_max_xy_tilt = abs(self.thrust_sp[2])*np.tan(tiltMax)
        thrust_max_xy = sqrt(quad.params["maxThr"]**2 - self.thrust_sp[2]**2)
        thrust_max_xy = min(thrust_max_xy, thrust_max_xy_tilt)

        # Saturate thrust in NE-direction
        self.thrust_sp[0:2] = thrust_xy_sp
        if (np.dot(self.thrust_sp[0:2].T, self.thrust_sp[0:2]) > thrust_max_xy**2):
            mag = norm(self.thrust_sp[0:2])
            self.thrust_sp[0:2] = thrust_xy_sp/mag*thrust_max_xy

        # Replace Previous Error
        self.vel_PrevE[0:2] = vel_xy_error

    def thrustToAttitude(self, quad, Ts):

        yaw_sp = self.eul_sp[2]

        # Desired body_z axis
        body_z = -self.thrust_sp/norm(self.thrust_sp)
        print(body_z)
        # Vector of desired Yaw direction in XY plane, rotated by pi/2 (fake body_y axis)
        y_C = np.array([-sin(yaw_sp), cos(yaw_sp), 0.0])
        print(y_C)
        body_x = np.cross(y_C, body_z)
        body_x = body_x/norm(body_x)

        body_y = np.cross(body_z, body_x)

        # Desired rotation matrix
        R_sp = np.array([body_x, body_y, body_z]).T
        print(R_sp)

        
    def attitude(self, quad, Ts):

        # Current thrust orientation e_z and desired thrust orientation e_z_d
        e_z = quad.dcm[:,2]
        e_z_d = -self.thrust_sp/norm(self.thrust_sp)
        print(e_z)
        print(e_z_d)
        # Angle alpha and quaternion error between thrust orientations
        # alpha = np.arccos(np.dot(e_z, e_z_d))
        # print(np.cross(e_z, e_z_d))
        # xyz_qe_red = np.sin(alpha/2) * np.cross(e_z, e_z_d)/norm(np.cross(e_z, e_z_d))
        # qe_red = np.array([np.cos(alpha/2), xyz_qe_red[0], xyz_qe_red[1], xyz_qe_red[2]])
        # print(qe_red)

        qe_red = np.zeros(4)
        qe_red[0] = np.dot(e_z, e_z_d) + sqrt(norm(e_z)**2 * norm(e_z_d)**2)
        qe_red[1:4] = np.cross(e_z, e_z_d)
        qe_red = utils.quatNormalize(qe_red)
        print(qe_red)

        qd_red = utils.quatMultiply(quad.quat, qe_red)
        print(qd_red)

        # qd_ful = 

        # q_mix = utils.quatMultiply(utils.inverse(qd_red), qd_ful)


    # def attitude(self, quad, Ts):
       
    #     # Roll Angle Control
    #     # --------------------------- 
    #     phiError = self.phiDes-quad.phi
    #     self.phidotDes = Pphi*phiError
    #     print(quad.phi)
    #     print(phiError)    
    #     # Pitch Angle Control
    #     # --------------------------- 
    #     thetaError = self.thetaDes-quad.theta
    #     self.thetadotDes = Ptheta*thetaError
        
    #     # Yaw Angle Control
    #     # --------------------------- 
    #     psiError = self.psiDes-quad.psi
    #     # print(quad.psi)
    #     # print(psiError)
    #     self.psidotDes = Ppsi*psiError
        
    #     # print(quad.psi)
    #     # print(self.psiDes)
    #     # print(psiError)
    #     # print(self.psidotDes)

    #     # Replace Previous Error
    #     # --------------------------- 
    #     self.phiPrevE = phiError
    #     self.thetaPrevE = thetaError
    #     self.psiPrevE = psiError

    #     # phidotDes, thetadotDes, psidotDes conversion to pDes, qDes, rDes
    #     # ---------------------------
    #     pqrDes = utils.phiThetaPsiDotToPQR(quad.phi, quad.theta, quad.psi, self.phidotDes, self.thetadotDes, self.psidotDes)
    #     pqrDes = np.clip(pqrDes.T, np.array([-pMax, -qMax, -rMax]), np.array([pMax, qMax, rMax])).T
    #     self.pDes = pqrDes[0]
    #     self.qDes = pqrDes[1]
    #     self.rDes = pqrDes[2]
    #     print(pqrDes)


    def rate(self, quad, Ts):
        
        # Roll Rate Control
        # ---------------------------       
        pError = self.pDes-quad.p
        self.pCmd = Pp*pError + Pp*Dp*(pError-self.pPrevE)/Ts
        print(quad.p)
        print(pError)
        print(self.pCmd)
        # Pitch Rate Control
        # --------------------------- 
        qError = self.qDes-quad.q
        self.qCmd = Pq*qError + Pq*Dq*(qError-self.qPrevE)/Ts
        
        # Yaw Rate Control
        # ---------------------------   
        rError = self.rDes-quad.r
        # print(quad.r)
        # print(rError)
        self.rCmd = Pr*rError + Pr*Dr*(rError-self.rPrevE)/Ts
        # print(self.rCmd)
        
        # print(quad.r)
        # print(rError)
        # print(self.rDes)

        # Replace Previous Error
        # --------------------------- 
        self.pPrevE = pError
        self.qPrevE = qError
        self.rPrevE = rError





    