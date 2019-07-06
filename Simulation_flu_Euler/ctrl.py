# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
from numpy import sin, cos, tan
# import angleFunctions as af
import utils

rad2deg = 180.0/pi
deg2rad = pi/180.0

Px = 2.2
Pu = 1.0
Du = 0.1

Py = 2.2
Pv = 1.0
Dv = 0.1

Pz = 10.0
Pw = 15.0
Dw = 0.1

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
wMax = 3.5

phiMax = 20*deg2rad
thetaMax = 20*deg2rad

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

        self.uFlatPrevE = 0
        self.vFlatPrevE = 0

    
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
            # self.altitude(qd, Ts)
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

    
    def position(self, qd, Ts):

        # X Position Control
        # --------------------------- 
        xError = self.xDes-qd.x
        self.xdotDes = Px*xError
    
        # Y Position Control
        # --------------------------- 
        yError = self.yDes-qd.y
        self.ydotDes = Py*yError

        # Replace Previous Error
        # ---------------------------
        self.xPrevE = xError
        self.yPrevE = yError


    def horiz_vel_grid(self, qd, Ts):

        # ydotDes, xdotDes conversion to uFlatDes, vFlatDes
        # ---------------------------
        uvwFlatDes = utils.xyzDotToUVW_Flat(qd.phi, qd.theta, qd.psi, self.xdotDes, self.ydotDes, self.zdotDes)
        uvwFlatDes = np.clip(uvwFlatDes[0:2].T, np.array([-uMax, -vMax]), np.array([uMax, vMax])).T

        self.uFlatDes = uvwFlatDes[0]
        self.vFlatDes = uvwFlatDes[1]


    def horiz_vel(self, qd, Ts):

        # uFlat Control
        # --------------------------- 
        uFlatError = self.uFlatDes-qd.uFlat
        thetaDes = Pu*uFlatError + Pu*Du*(uFlatError-self.uFlatPrevE)/Ts
        self.thetaDes = np.clip(thetaDes, -thetaMax, thetaMax)

        # vFlat Control
        # --------------------------- 
        vFlatError = self.vFlatDes-qd.vFlat
        phiDes = -(Pv*vFlatError + Pv*Dv*(vFlatError-self.vFlatPrevE)/Ts)
        self.phiDes = np.clip(phiDes, -phiMax, phiMax)

        # Replace Previous Error
        # ---------------------------
        self.uFlatPrevE = uFlatError
        self.vFlatPrevE = vFlatError
    
    def altitude(self, qd, Ts):
       
        # Z Position Control
        # --------------------------- 
        zError = self.zDes-qd.z
        zdotDes = Pz*zError
        self.zdotDes = np.clip(zdotDes, -wMax, wMax)
        
        # Z Velocity Control
        # ---------------------------
        zdotError = self.zdotDes-qd.zdot
        self.zCmd = Pw*zdotError + Pw*Dw*(zdotError-self.zdotPrevE)/Ts + qd.params["FF"]/(cos(qd.phi)*cos(qd.theta))
        
        # Replace Previous Error
        # ---------------------------
        self.zPrevE = zError
        self.zdotPrevE = zdotError


    def attitude(self, qd, Ts):
       
        # Roll Angle Control
        # --------------------------- 
        phiError = self.phiDes-qd.phi
        self.phidotDes = Pphi*phiError
            
        # Pitch Angle Control
        # --------------------------- 
        thetaError = self.thetaDes-qd.theta
        self.thetadotDes = Ptheta*thetaError
        
        # Yaw Angle Control
        # --------------------------- 
        psiError = self.psiDes-qd.psi
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


    def rate(self, qd, Ts):
        
        # Roll Rate Control
        # ---------------------------       
        pError = self.pDes-qd.p
        self.pCmd = Pp*pError + Pp*Dp*(pError-self.pPrevE)/Ts
        
        # Pitch Rate Control
        # --------------------------- 
        qError = self.qDes-qd.q
        self.qCmd = Pq*qError + Pq*Dq*(qError-self.qPrevE)/Ts
        
        # Yaw Rate Control
        # ---------------------------   
        rError = self.rDes-qd.r
        self.rCmd = Pr*rError + Pr*Dr*(rError-self.rPrevE)/Ts
        
        # print(qd.r)
        # print(rError)
        # print(self.rDes)

        # Replace Previous Error
        # --------------------------- 
        self.pPrevE = pError
        self.qPrevE = qError
        self.rPrevE = rError





    