# -*- coding: utf-8 -*-

import numpy as np
from numpy import sin, cos, tan
from scipy.linalg import solve
from scipy.integrate import odeint
from quadFiles.initQuad import sys_params, init_cmd, init_state
import utils

class Quadcopter:

    def __init__(self):
        
        # Quad Params
        # ---------------------------
        self.params = sys_params()
        
        # Command for initial stable hover
        # ---------------------------
        ini_hover = init_cmd(self.params)
        self.params["FF"] = ini_hover[0]         # Feed-Forward Command for Hover
        self.params["w_hover"] = ini_hover[1]    # Motor Speed for Hover
        self.params["thr_hover"] = ini_hover[2]  # Motor Thrust for Hover  
        self.thr = np.ones(4)*ini_hover[2]
        self.tor = np.ones(4)*ini_hover[3]

        # Initial State
        # ---------------------------
        self.state = init_state(self.params)

        self.pos   = self.state[0:3]
        self.quat  = self.state[3:7]
        self.vel   = self.state[7:10]
        self.omega = self.state[10:13]
        self.wMotor = np.array([self.state[13], self.state[15], self.state[17], self.state[19]])
        self.vel_dot = np.zeros(3)
        self.omega_dot = np.zeros(3)

        self.extended_state()
        self.forces()


    def extended_state(self):

        # Rotation Matrix of current state (Direct Cosine Matrix)
        self.dcm = utils.quat2Dcm(self.quat)

        # Euler angles of current state
        YPR = utils.quatToYPR_ZYX(self.quat)
        self.euler = YPR[::-1] # flip YPR so that euler state = phi, theta, psi
        self.psi   = YPR[0]
        self.theta = YPR[1]
        self.phi   = YPR[2]


    def forces(self):
        
        # Rotor thrusts and torques
        self.thr = self.params["kTh"]*self.wMotor*self.wMotor
        self.tor = self.params["kTo"]*self.wMotor*self.wMotor


    def state_dot(self, state, t, cmd):

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
        kp   = self.params["kp"]
        damp = self.params["damp"]
        db   = self.params["motordeadband"]
    
        # Import State Vector
        # ---------------------------  
        x          = state[0]
        y          = state[1]
        z          = state[2]
        q0         = state[3]
        q1         = state[4]
        q2         = state[5]
        q3         = state[6]
        xdot       = state[7]
        ydot       = state[8]
        zdot       = state[9]
        p          = state[10]
        q          = state[11]
        r          = state[12]
        wMotor1    = state[13]
        wdotMotor1 = state[14]
        wMotor2    = state[15]
        wdotMotor2 = state[16]
        wMotor3    = state[17]
        wdotMotor3 = state[18]
        wMotor4    = state[19]
        wdotMotor4 = state[20]

        # Motor Dynamics and Rotor forces (Second Order System: https://apmonitor.com/pdc/index.php/Main/SecondOrderSystems)
        # ---------------------------
        
        uMotor = cmd
        wddotMotor1 = (-2.0*damp*tau*wdotMotor1 - wMotor1 + kp*uMotor[0])/(tau**2)
        wddotMotor2 = (-2.0*damp*tau*wdotMotor2 - wMotor2 + kp*uMotor[1])/(tau**2)
        wddotMotor3 = (-2.0*damp*tau*wdotMotor3 - wMotor3 + kp*uMotor[2])/(tau**2)
        wddotMotor4 = (-2.0*damp*tau*wdotMotor4 - wMotor4 + kp*uMotor[3])/(tau**2)
    
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
            [                                                                       xdot],
            [                                                                       ydot],
            [                                                                       zdot],
            [                                            -0.5*p*q1 - 0.5*q*q2 - 0.5*q3*r],
            [                                             0.5*p*q0 - 0.5*q*q3 + 0.5*q2*r],
            [                                             0.5*p*q3 + 0.5*q*q0 - 0.5*q1*r],
            [                                            -0.5*p*q2 + 0.5*q*q1 + 0.5*q0*r],
            [                       2*(q0*q2 + q1*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [                      -2*(q0*q1 - q2*q3)*(ThrM1 + ThrM2 + ThrM3 + ThrM4)/mB],
            [((ThrM1 + ThrM2 + ThrM3 + ThrM4)*(q0**2 - q1**2 - q2**2 + q3**2) - g*mB)/mB],
            [ (IByy*q*r - IBzz*q*r + ThrM1*dym - ThrM2*dym - ThrM3*dym + ThrM4*dym)/IBxx],
            [(-IBxx*p*r + IBzz*p*r - ThrM1*dxm - ThrM2*dxm + ThrM3*dxm + ThrM4*dxm)/IByy],
            [                 (IBxx*p*q - IByy*p*q + TorM1 - TorM2 + TorM3 - TorM4)/IBzz]])
    
    
        # State Derivative Vector
        # ---------------------------
        sdot     = np.zeros([21])
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
        sdot[12] = DynamicsDot[12]
        sdot[13] = wdotMotor1
        sdot[14] = wddotMotor1
        sdot[15] = wdotMotor2
        sdot[16] = wddotMotor2
        sdot[17] = wdotMotor3
        sdot[18] = wddotMotor3
        sdot[19] = wdotMotor4
        sdot[20] = wddotMotor4

        return sdot

    def update(self, t, Ts, cmd):

        prev_vel   = self.vel
        prev_omega = self.omega     
        self.state = odeint(self.state_dot, self.state, [t,t+Ts], args = (cmd,))[1]

        self.pos   = self.state[0:3]
        self.quat  = self.state[3:7]
        self.vel   = self.state[7:10]
        self.omega = self.state[10:13]
        self.wMotor = np.array([self.state[13], self.state[15], self.state[17], self.state[19]])

        self.vel_dot = (self.vel - prev_vel)/Ts
        self.omega_dot = (self.omega - prev_omega)/Ts

        self.extended_state()
        self.forces()