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
    
    def __init__(self, quad):
        self.error = np.zeros([12, 1])
        self.sDesCalc = np.zeros([12, 1])
        self.cmd = np.ones([4,1])*quad.params["FF"]   # Motor 1 is front left, then clockwise numbering
    
    def controller(self, quad, sDes, Ts, trajType, trajSelect):
        if trajType == "altitude":
            self.altitude(quad, sDes, Ts)

    def altitude(self, quad, sDes, Ts):
        # Import Params
        # ---------------------------
        FF     = quad.params["FF"]

        # Current State
        # ---------------------------
        x     = quad.state[0]
        y     = quad.state[1]
        z     = quad.state[2]
        phi   = quad.state[3]
        theta = quad.state[4]
        psi   = quad.state[5]
        xdot  = quad.state[6]
        ydot  = quad.state[7]
        zdot  = quad.state[8]
        p     = quad.state[9]
        q     = quad.state[10]
        r     = quad.state[11]

        # Desired State
        # ---------------------------
        xDes     = sDes[0]
        yDes     = sDes[1]
        zDes     = sDes[2]
        phiDes   = sDes[3]
        thetaDes = sDes[4]
        psiDes   = sDes[5]
        xdotDes  = sDes[6]
        ydotDes  = sDes[7]
        zdotDes  = sDes[8]
        pDes     = sDes[9]
        qDes     = sDes[10]
        rDes     = sDes[11]

        # Previous Error
        # ---------------------------
        xPrevE     = self.error[0]
        yPrevE     = self.error[1]
        zPrevE     = self.error[2]
        phiPrevE   = self.error[3]
        thetaPrevE = self.error[4]
        psiPrevE   = self.error[5]
        xdotPrevE  = self.error[6]
        ydotPrevE  = self.error[7]
        zdotPrevE  = self.error[8]
        pPrevE     = self.error[9]
        qPrevE     = self.error[10]
        rPrevE     = self.error[11]
        
        # Z Position Control
        # --------------------------- 
        zError = zDes-z
        zdotDes = Pz*zError
        zdotError = zdotDes-zdot
        zCmd = Pw*zdotError + Pw*Dw*(zdotError-zdotPrevE)/Ts + FF/(cos(phi)*cos(theta))
        #zCmd = FF
        
        # Roll Control
        # --------------------------- 
        phiError = phiDes-phi
        phidotDes = Pphi*phiError
            
        # Pitch Control
        # --------------------------- 
        thetaError = thetaDes-theta
        thetadotDes = Ptheta*thetaError
        
        # Yaw Control
        # --------------------------- 
        psiError = psiDes-psi
        psidotDes = Ppsi*psiError
        
        # phidotDes, thetadotDes, psidotDes conversion to pDes, qDes, rDes
        # ---------------------------
        pqrDes = utils.phiThetaPsiDotToPQR(phi, theta, psi, phidotDes, thetadotDes, psidotDes)
        pqrDes = np.clip(pqrDes.T, np.array([-pmax, -qmax, -rmax]), np.array([pmax, qmax, rmax])).T

        pDes = pqrDes[0]
        pError = pDes-p
        pCmd = Pp*pError + Pp*Dp*(pError-pPrevE)/Ts
        
        qDes = pqrDes[1]
        qError = qDes-q
        qCmd = Pq*qError + Pq*Dq*(qError-qPrevE)/Ts
        
        rDes = pqrDes[2]  
        rError = rDes-r
        rCmd = Pr*rError + Pr*Dr*(rError-rPrevE)/Ts
            
        # Mixer
        # --------------------------- 
        self.cmd = utils.mixer(zCmd, pCmd, qCmd, rCmd, quad)
        
        # Add calculated Desired States
        # ---------------------------         
        self.sDesCalc[0] = xDes
        self.sDesCalc[1] = yDes
        self.sDesCalc[2] = zDes
        self.sDesCalc[3] = phiDes
        self.sDesCalc[4] = thetaDes
        self.sDesCalc[5] = psiDes
        self.sDesCalc[6] = xdotDes
        self.sDesCalc[7] = ydotDes
        self.sDesCalc[8] = zdotDes
        self.sDesCalc[9] = pDes
        self.sDesCalc[10] = qDes
        self.sDesCalc[11] = rDes
        
        # Error Vector
        # --------------------------- 
        self.error[2] = zError
        self.error[3] = phiError
        self.error[4] = thetaError
        self.error[5] = psiError
        self.error[8] = zdotError
        self.error[9] = pError
        self.error[10] = qError
        self.error[11] = rError



    