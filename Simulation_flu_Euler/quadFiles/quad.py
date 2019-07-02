# -*- coding: utf-8 -*-

import numpy as np
from numpy import (sin, cos, tan)
from scipy.linalg import solve
from .initQuad import (sys_params, init_cmd, init_state)
import utils

class Quadcopter:

    def __init__(self):
        
        # Quad Params
        # ---------------------------
        self.params = sys_params()
        
        # Command for stable hover
        # ---------------------------
        hover_cmd = init_cmd(self.params)
        self.params["FF"] = hover_cmd[0]         # Feed-Forward Command for Hover
        self.params["w_hover"] = hover_cmd[1]    # Motor Speed for Hover

        # Initial State
        # ---------------------------
        self.state = init_state(self.params)

    # def point_position(self):

    # def extended_state(self)

    def state_dot(self, t, cmd):

        # Import Params
        # ---------------------------    
        mB   = self.params["mB"]
        g    = self.params["g"]
        dxm  = self.params["dxm"]
        dym  = self.params["dym"]
        dzm  = self.params["dzm"]
        IB   = self.params["IB"]
        IBxx = IB[0,0]
        IByy = IB[1,1]
        IBzz = IB[2,2]
        kTh  = self.params["kTh"]
        kTo  = self.params["kTo"]
        c1   = self.params["motorc1"]
        c0   = self.params["motorc0"]
        tau  = self.params["tau"]
        Kp   = self.params["Kp"]
        damp = self.params["damp"]
        db   = self.params["motordeadband"]
    
        # Import State Vector
        # ---------------------------  
        x          = self.state[0]
        y          = self.state[1]
        z          = self.state[2]
        phi        = self.state[3]
        theta      = self.state[4]
        psi        = self.state[5]
        xdot       = self.state[6]
        ydot       = self.state[7]
        zdot       = self.state[8]
        p          = self.state[9]
        q          = self.state[10]
        r          = self.state[11]
        wMotor1    = self.state[12]
        wdotMotor1 = self.state[13]
        wMotor2    = self.state[14]
        wdotMotor2 = self.state[15]
        wMotor3    = self.state[16]
        wdotMotor3 = self.state[17]
        wMotor4    = self.state[18]
        wdotMotor4 = self.state[19]

        # Motor Dynamics and Rotor forces (Second Order System: https://apmonitor.com/pdc/index.php/Main/SecondOrderSystems)
        # ---------------------------
        uMotor = cmd*c1 + c0    # Motor speed in relation to cmd
        uMotor[cmd < db] = 0    # Apply motor deadband

        wddotMotor1 = (-2*damp*tau*wdotMotor1 - wMotor1 + Kp*uMotor[0])/(tau**2)
        wddotMotor2 = (-2*damp*tau*wdotMotor2 - wMotor2 + Kp*uMotor[1])/(tau**2)
        wddotMotor3 = (-2*damp*tau*wdotMotor3 - wMotor3 + Kp*uMotor[2])/(tau**2)
        wddotMotor4 = (-2*damp*tau*wdotMotor4 - wMotor4 + Kp*uMotor[3])/(tau**2)
    
        wMotor = np.array([wMotor1, wMotor2, wMotor3, wMotor4])
        thrust = kTh*wMotor*wMotor
        torque = kTo*wMotor*wMotor
    
        ThrM1 = thrust[0]
        ThrM2 = thrust[1]
        ThrM3 = thrust[2]
        ThrM4 = thrust[3]
        TorM1 = torque[0]
        TorM2 = torque[1]
        TorM3 = torque[2]
        TorM4 = torque[3]
    
        # State Derivatives (from PyDy) This is already the analytically solved vector of MM*x = RHS
        # ---------------------------
        DynamicsDot = np.array([
            [                                                                                  xdot],
            [                                                                                  ydot],
            [                                                                                  zdot],
            [                                     p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)],
            [                                                               q*cos(phi) - r*sin(phi)],
            [                                                  (q*sin(phi) + r*cos(phi))/cos(theta)],
            [ (sin(phi)*sin(psi) + sin(theta)*cos(phi)*cos(psi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [-(sin(phi)*cos(psi) - sin(psi)*sin(theta)*cos(phi))*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [                       (cos(phi)*cos(theta)*(ThrM1 + ThrM2 + ThrM3 + ThrM4) - g*mB)/mB],
            [            (IByy*q*r - IBzz*q*r + ThrM1*dym - ThrM2*dym - ThrM3*dym + ThrM4*dym)/IBxx],
            [           (-IBxx*p*r + IBzz*p*r - ThrM1*dxm - ThrM2*dxm + ThrM3*dxm + ThrM4*dxm)/IByy],
            [                            (IBxx*p*q - IByy*p*q + TorM1 - TorM2 + TorM3 - TorM4)/IBzz]])
    
    
        # State Derivative Vector
        # ---------------------------
        sdot     = np.zeros([20])
        sdot[0]  = DynamicsDot[0]
        sdot[1]  = DynamicsDot[1]
        sdot[2]  = DynamicsDot[2]
        sdot[3]  = DynamicsDot[3]
        sdot[4]  = DynamicsDot[4]
        sdot[5]  = DynamicsDot[5]
        sdot[6]  = DynamicsDot[6]
        sdot[7]  = DynamicsDot[7]
        sdot[8]  = DynamicsDot[8]
        sdot[9]  = DynamicsDot[9]
        sdot[10] = DynamicsDot[10]
        sdot[11] = DynamicsDot[11]
        sdot[12] = wdotMotor1
        sdot[13] = wddotMotor1
        sdot[14] = wdotMotor2
        sdot[15] = wddotMotor2
        sdot[16] = wdotMotor3
        sdot[17] = wddotMotor3
        sdot[18] = wdotMotor4
        sdot[19] = wddotMotor4

        return sdot
