# -*- coding: utf-8 -*-

import numpy as np
from numpy import pi
from numpy import sin, cos, tan
# import angleFunctions as af
# from mixer import mixer
import utils

rad2deg = 180.0/pi
deg2rad = pi/180.0

Px = 1.0
Pu = 2.0
Du = 0.01

Py = 1.0
Pv = 2.0
Dv = 0.01

Pz = 10.0
Pw = 6.0
Dw = 0.4

phiMax = 0.6
phiMin = -phiMax
thetaMax = 0.6
thetaMin = -thetaMax

Pphi = 8.0
Pp = 4.0
Dp = 0.1

Ptheta = Pphi
Pq = Pp
Dq = Dp 

Ppsi = 6.0
Pr = 8.0
Dr = 0.2

pmax = 200*deg2rad
qmax = 200*deg2rad
rmax = 200*deg2rad


class Control:
    
    def __init__(self, qd):
        self.sDesCalc = np.zeros([12, 1])
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
        
        # Select Controller
        # ---------------------------
        if trajType == "altitude":
            self.altitude(qd, Ts)
            self.attitude(qd, Ts)
            self.rate(qd, Ts)
        
        # Mixer
        # --------------------------- 
        self.cmd = utils.mixer(self.zCmd, self.pCmd, self.qCmd, self.rCmd, qd)

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

    def altitude(self, qd, Ts):
       
        # Z Position Control
        # --------------------------- 
        zError = self.zDes-qd.z
        self.zdotDes = Pz*zError
        zdotError = self.zdotDes-qd.zdot
        self.zCmd = Pw*zdotError + Pw*Dw*(zdotError-self.zdotPrevE)/Ts + qd.params["FF"]/(cos(qd.phi)*cos(qd.theta))
        
        self.zPrevE = zError
        self.zdotPrevE = zdotError

    def attitude(self, qd, Ts):
        # Roll Control
        # --------------------------- 
        phiError = self.phiDes-qd.phi
        self.phidotDes = Pphi*phiError
            
        # Pitch Control
        # --------------------------- 
        thetaError = self.thetaDes-qd.theta
        self.thetadotDes = Ptheta*thetaError
        
        # Yaw Control
        # --------------------------- 
        psiError = self.psiDes-qd.psi
        self.psidotDes = Ppsi*psiError
        
        self.phiPrevE = phiError
        self.thetaPrevE = thetaError
        self.psiPrevE = psiError

    def rate(self, qd, Ts):
        
        # phidotDes, thetadotDes, psidotDes conversion to pDes, qDes, rDes
        # ---------------------------
        pqrDes = utils.phiThetaPsiDotToPQR(qd.phi, qd.theta, qd.psi, self.phidotDes, self.thetadotDes, self.psidotDes)
        pqrDes = np.clip(pqrDes.T, np.array([-pmax, -qmax, -rmax]), np.array([pmax, qmax, rmax])).T

        self.pDes = pqrDes[0]
        pError = self.pDes-qd.p
        self.pCmd = Pp*pError + Pp*Dp*(pError-self.pPrevE)/Ts
        
        self.qDes = pqrDes[1]
        qError = self.qDes-qd.q
        self.qCmd = Pq*qError + Pq*Dq*(qError-self.qPrevE)/Ts
        
        self.rDes = pqrDes[2]  
        rError = self.rDes-qd.r
        self.rCmd = Pr*rError + Pr*Dr*(rError-self.rPrevE)/Ts

        self.pPrevE = pError
        self.qPrevE = qError
        self.rPrevE = rError





    