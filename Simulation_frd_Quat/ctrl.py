# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
from numpy import sin, cos, tan, sqrt
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

# Puv = 0.9
# Duv = 0.0001

Py    = 1.6
Pxdot = 1.1
Dxdot = 0.15

Px = 1.6
Pydot = 1.1
Dydot = 0.15 

Pz    = 8.0
Pzdot = 15.0
Dzdot = 0.15

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
angleMax = 22*deg2rad

pMax = 200*deg2rad
qMax = 200*deg2rad
rMax = 200*deg2rad


class Control:
    
    def __init__(self, qd):
        self.sDesCalc = np.zeros([15, 1])
        self.cmd = np.ones([4,1])*qd.params["FF"]   # Motor 1 is front left, then clockwise numbering
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

        self.pos_PrevE  = np.zeros([3,1])
        self.vel_PrevE  = np.zeros([3,1])

        self.uFlatPrevE = 0
        self.vFlatPrevE = 0
        self.velFlatPrevE = 0

    
    def controller(self, qd, sDes, Ts, trajType, trajSelect):
        
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
        self.vel_sp = sDes[6:9]

        self.uFlatDes = sDes[12]
        self.vFlatDes = sDes[13]
        self.wFlatDes = sDes[14]

        # Select Controller
        # ---------------------------
        if trajType == "attitude":
            self.zCmd = qd.params["FF"]
            self.attitude(qd, Ts)
            self.rate(qd, Ts)
        if trajType == "altitude":
            self.altitude(qd, Ts)
            self.attitude(qd, Ts)
            self.rate(qd, Ts)
        if trajType == "velocity":
            self.horiz_vel(qd, Ts)
            self.altitude(qd, Ts)
            self.attitude(qd, Ts)
            self.rate(qd, Ts)
        if trajType == "grid_velocity":
            self.horiz_vel_grid(qd, Ts)    
            self.horiz_vel(qd, Ts)
            self.altitude(qd, Ts)
            self.attitude(qd, Ts)
            self.rate(qd, Ts)
        if trajType == "position":
            self.position(qd, Ts)
            self.horiz_vel_grid(qd, Ts)    
            self.horiz_vel(qd, Ts)
            self.altitude(qd, Ts)
            self.attitude(qd, Ts)
            self.rate(qd, Ts)

        # Mixer
        # --------------------------- 
        self.cmd = utils.mixer(self.zCmd, self.pCmd, self.qCmd, self.rCmd, qd)

        # Add Exponential to command
        # ---------------------------
        self.cmd = utils.expoCmd(qd.params, self.cmd)

        # Add calculated Desired States
        # ---------------------------         
        self.sDesCalc[0] = self.xDes
        self.sDesCalc[1] = self.yDes
        self.sDesCalc[2] = self.zDes
        self.sDesCalc[3] = self.phiDes
        self.sDesCalc[4] = self.thetaDes
        self.sDesCalc[5] = self.psiDes
        self.sDesCalc[6] = self.xdotDes
        self.sDesCalc[7] = self.ydotDes
        self.sDesCalc[8] = self.zdotDes
        self.sDesCalc[9] = self.pDes
        self.sDesCalc[10] = self.qDes
        self.sDesCalc[11] = self.rDes

        self.sDesCalc[12] = self.uFlatDes
        self.sDesCalc[13] = self.vFlatDes
        self.sDesCalc[14] = self.wFlatDes


    def z_pos_control(self, qd, Ts):
       
        # Z Position Control
        # --------------------------- 
        pos_z_error = self.pos_sp[2] - qd.pos[2]
        self.vel_sp[2] = pos_P_gain[2]*pos_z_error + self.vel_sp[2]         # Be careful to reset vel_sp for next iteration
        self.vel_sp[2] = np.clip(self.vel_sp[2], -velMax[2], velMax[2])

        # Replace Previous Error
        # ---------------------------
        self.pos_PrevE[2] = pos_z_error

    
    def xy_pos_control(self, qd, Ts):

        # XY Position Control
        # --------------------------- 
        pos_xy_error = (self.pos_sp[0:2] - qd.pos[0:2])
        self.vel_sp[0:2] = pos_P_gain[0:2]*pos_xy_error + self.vel_sp[0:2]  # Be careful to reset vel_sp for next iteration
        self.vel_sp[0:2] = np.clip(self.vel_sp[0:2], -velMax[0:2], velMax[0:2])

        # Replace Previous Error
        # ---------------------------
        self.pos_PrevE[0:2] = pos_xy_error
        

    def z_vel_control(self, qd, Ts):
        
        # Z Velocity Control
        # ---------------------------
        vel_z_error = self.vel_sp[2] - qd.vel[2]
        thrust_z_sp = vel_P_gain[2]*vel_z_error + vel_D_gain[2]*qd.vel_dot - qd.params["thr_hover"]
        
        
        self.thrust_sp[2] = np.clip.(thrust_z_sp,)
        self.vel_PrevE[2] = vel_z_error

    
    def xy_vel_control(self, qd, Ts):
        vel_xy_error = self.vel_sp[0:2] - qd.vel[0:2]


    

    def horiz_vel_grid(self, qd, Ts):

        # ydotDes, xdotDes conversion to uFlatDes, vFlatDes
        # ---------------------------
        uvwFlatDes = utils.xyzDotToUVW_Flat_euler(qd.phi, qd.theta, qd.psi, self.xdotDes, self.ydotDes, self.zdotDes)
        uvwFlatDes = np.clip(uvwFlatDes[0:2].T, np.array([-uMax, -vMax]), np.array([uMax, vMax])).T

        self.uFlatDes = uvwFlatDes[0]
        self.vFlatDes = uvwFlatDes[1]

    def horiz_vel(self, qd, Ts):
        if qd.params["ifYawFix"]:
            # uFlatError
            # --------------------------- 
            uFlatError = self.uFlatDes-qd.uFlat

            # vFlatError
            # --------------------------- 
            vFlatError = self.vFlatDes-qd.vFlat
        
            # velFlat Control
            # --------------------------- 
            velFlatError = sqrt(self.uFlatDes**2 + self.vFlatDes**2) - sqrt(qd.uFlat**2 + qd.vFlat**2)
            angleDes = Puv*velFlatError + Puv*Duv*(velFlatError-self.velFlatPrevE)/Ts
            angleDes = np.clip(angleDes, -angleMax, angleMax)
            
            # Determine phiDes, thetaDes, psiDes
            # --------------------------- 
            # quaternion = [cos(angleDes/2), -vError/a, uError/a, 0], 
            # where "a" is a coefficient that divides vError and uError in order to have a normalized quaternion
            if (angleDes==0):
                a = 1
            else:
                a = sqrt((uFlatError**2 + vFlatError**2)/(1-(cos(angleDes/2))**2))
        
            desQuat = np.array([cos(angleDes/2), vFlatError/a, -uFlatError/a, 0])

            YPR = utils.QuatToYPR_ZYX(desQuat)
            self.phiDes   = YPR[2]
            self.thetaDes = YPR[1]
            self.psiDes   = self.psiDes+YPR[0]
            
            # Replace Previous Error
            # ---------------------------
            self.uFlatPrevE = uFlatError
            self.vFlatPrevE = vFlatError
            self.velFlatPrevE = velFlatError
        
        else:
            # uFlat Control
            # --------------------------- 
            uFlatError = self.uFlatDes-qd.uFlat
            thetaDes = -(Pu*uFlatError + Pu*Du*(uFlatError-self.uFlatPrevE)/Ts)
            self.thetaDes = np.clip(thetaDes, -thetaMax, thetaMax)

            # vFlat Control
            # --------------------------- 
            vFlatError = self.vFlatDes-qd.vFlat
            phiDes = (Pv*vFlatError + Pv*Dv*(vFlatError-self.vFlatPrevE)/Ts)
            self.phiDes = np.clip(phiDes, -phiMax, phiMax)

            # Replace Previous Error
            # ---------------------------
            self.uFlatPrevE = uFlatError
            self.vFlatPrevE = vFlatError


    def attitude(self, qd, Ts):
       
        # Roll Angle Control
        # --------------------------- 
        phiError = self.phiDes-qd.phi
        self.phidotDes = Pphi*phiError
        print(qd.phi)
        print(phiError)    
        # Pitch Angle Control
        # --------------------------- 
        thetaError = self.thetaDes-qd.theta
        self.thetadotDes = Ptheta*thetaError
        
        # Yaw Angle Control
        # --------------------------- 
        psiError = self.psiDes-qd.psi
        # print(qd.psi)
        # print(psiError)
        self.psidotDes = Ppsi*psiError
        
        # print(qd.psi)
        # print(self.psiDes)
        # print(psiError)
        # print(self.psidotDes)

        # Replace Previous Error
        # --------------------------- 
        self.phiPrevE = phiError
        self.thetaPrevE = thetaError
        self.psiPrevE = psiError

        # phidotDes, thetadotDes, psidotDes conversion to pDes, qDes, rDes
        # ---------------------------
        pqrDes = utils.phiThetaPsiDotToPQR(qd.phi, qd.theta, qd.psi, self.phidotDes, self.thetadotDes, self.psidotDes)
        pqrDes = np.clip(pqrDes.T, np.array([-pMax, -qMax, -rMax]), np.array([pMax, qMax, rMax])).T
        self.pDes = pqrDes[0]
        self.qDes = pqrDes[1]
        self.rDes = pqrDes[2]
        print(pqrDes)


    def rate(self, qd, Ts):
        
        # Roll Rate Control
        # ---------------------------       
        pError = self.pDes-qd.p
        self.pCmd = Pp*pError + Pp*Dp*(pError-self.pPrevE)/Ts
        print(qd.p)
        print(pError)
        print(self.pCmd)
        # Pitch Rate Control
        # --------------------------- 
        qError = self.qDes-qd.q
        self.qCmd = Pq*qError + Pq*Dq*(qError-self.qPrevE)/Ts
        
        # Yaw Rate Control
        # ---------------------------   
        rError = self.rDes-qd.r
        # print(qd.r)
        # print(rError)
        self.rCmd = Pr*rError + Pr*Dr*(rError-self.rPrevE)/Ts
        # print(self.rCmd)
        
        # print(qd.r)
        # print(rError)
        # print(self.rDes)

        # Replace Previous Error
        # --------------------------- 
        self.pPrevE = pError
        self.qPrevE = qError
        self.rPrevE = rError





    