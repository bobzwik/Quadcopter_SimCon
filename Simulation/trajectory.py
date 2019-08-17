# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
# Functions get_poly_cc, minSomethingTraj, pos_waypoint_min are derived from Peter Huang's work:
# https://github.com/hbd730/quadcopter-simulation
# author: Peter Huang
# email: hbd730@gmail.com
# license: BSD
# Please feel free to use and modify this, but keep the above information. Thanks!



import numpy as np
from numpy import pi
from numpy.linalg import norm
from waypoints import makeWaypoints
import config

class Trajectory:

    def __init__(self, ctrlType, trajSelect):

        self.ctrlType = ctrlType
        self.xyzType = trajSelect[0]
        self.yawType = trajSelect[1]
        self.averVel = trajSelect[2]

        t_wps, wps, y_wps, v_wp = makeWaypoints()
        self.t_wps = t_wps
        self.wps   = wps
        self.y_wps = y_wps
        self.v_wp  = v_wp

        if (self.ctrlType == "xyz_pos"):
            self.T_segment = np.diff(self.t_wps)

            if (self.averVel == 1):
                distance_segment = self.wps[1:] - self.wps[:-1]
                self.T_segment = np.sqrt(distance_segment[:,0]**2 + distance_segment[:,1]**2 + distance_segment[:,2]**2)/self.v_wp
                self.t_wps = np.zeros(len(self.T_segment) + 1)
                self.t_wps[1:] = np.cumsum(self.T_segment)
            
            if (self.xyzType >= 3 and not (self.xyzType == 99)):
                self.deriv_order = int(self.xyzType-2)       # Looking to minimize which derivative order (eg: Minimum velocity -> first order)

                # Calculate coefficients
                self.coeff_x = minSomethingTraj(self.wps[:,0], self.deriv_order)
                self.coeff_y = minSomethingTraj(self.wps[:,1], self.deriv_order)
                self.coeff_z = minSomethingTraj(self.wps[:,2], self.deriv_order)

            self.current_heading = np.zeros(2)
        
        # Initialize trajectory setpoint
        self.desPos = np.array([0., 0., 0.])    # Desired position (x, y, z)
        self.desVel = np.array([0., 0., 0.])    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.array([0., 0., 0.])    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.array([0., 0., 0.])    # Desired thrust in N-E-D directions (or E-N-U, if selected)
        self.desEul = np.array([0., 0., 0.])    # Desired orientation in the world frame (phi, theta, psi)
        self.desPQR = np.array([0., 0., 0.])    # Desired angular velocity in the body frame (p, q, r)
        self.desYawRate = 0.                    # Desired yaw speed
        self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate)).astype(float)


    def desiredState(self, t, Ts, quad):
        
        self.desPos = np.array([0., 0., 0.])    # Desired position (x, y, z)
        self.desVel = np.array([0., 0., 0.])    # Desired velocity (xdot, ydot, zdot)
        self.desAcc = np.array([0., 0., 0.])    # Desired acceleration (xdotdot, ydotdot, zdotdot)
        self.desThr = np.array([0., 0., 0.])    # Desired thrust in N-E-D directions (or E-N-U, if selected)
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
        
        def pos_waypoint_min():
            """ The function takes known number of waypoints and time, then generates a
            minimum snap trajectory which goes through each waypoint. The output is
            the desired state associated with the next waypont for the time t.
            waypoints is [N,3] matrix, waypoints = [[x0,y0,z0]...[xn,yn,zn]].
            v is velocity in m/s
            """

            nb_coeff = self.deriv_order*2

            # prepare the next desired state
            if t == 0:
                self.t_idx = 0
                self.desPos = self.wps[0,:]
                t1 = get_poly_cc(nb_coeff, 1, 0)
                # self.desVel = np.array([self.coeff_x[0:nb_coeff].dot(t1), self.coeff_y[0:nb_coeff].dot(t1), self.coeff_z[0:nb_coeff].dot(t1)]) * (1.0 / self.T_segment[0])
                self.current_heading = np.array([self.desVel[0], self.desVel[1]])

            # stay hover at the last waypoint position
            elif (t >= self.t_wps[-1]):
                self.t_idx = -1
                self.desPos = self.wps[-1,:]
            else:
                self.t_idx = np.where(t <= self.t_wps)[0][0] - 1
                # scaled time
                scale = (t - self.t_wps[self.t_idx])/self.T_segment[self.t_idx]
                start = nb_coeff * self.t_idx
                end = nb_coeff * (self.t_idx + 1)
                
                t0 = get_poly_cc(nb_coeff, 0, scale)
                self.desPos = np.array([self.coeff_x[start:end].dot(t0), self.coeff_y[start:end].dot(t0), self.coeff_z[start:end].dot(t0)])

                t1 = get_poly_cc(nb_coeff, 1, scale)
                self.desVel = np.array([self.coeff_x[start:end].dot(t1), self.coeff_y[start:end].dot(t1), self.coeff_z[start:end].dot(t1)]) * (1.0 / self.T_segment[self.t_idx])

                t2 = get_poly_cc(nb_coeff, 2, scale)
                self.desAcc = np.array([self.coeff_x[start:end].dot(t2), self.coeff_y[start:end].dot(t2), self.coeff_z[start:end].dot(t2)]) * (1.0 / self.T_segment[self.t_idx]**2)

                # # calculate desired yaw and yaw rate
                # next_heading = np.array([self.desVel[0], self.desVel[1]])
                # # angle between current vector with the next heading vector
                # print(self.current_heading)
                # print(next_heading)
                # delta_psi = np.arccos(np.dot(self.current_heading, next_heading) / (norm(self.current_heading)*norm(next_heading)))
                # # cross product allow us to determine rotating direction
                # norm_v = np.cross(self.current_heading,next_heading)

                # if norm_v > 0:
                #     self.desEul[2] += delta_psi
                # else:
                #     self.desEul[2] -= delta_psi

                # # dirty hack, quadcopter's yaw range represented by quaternion is [-pi, pi]
                # if self.desEul[2] > np.pi:
                #     self.desEul[2] = self.desEul[2] - 2*pi

                # # print next_heading, current_heading, "yaw", yaw*180/np.pi, 'pos', pos
                # self.current_heading = next_heading
                # self.desYawRate = delta_psi / Ts # dt is control period
                    

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
                self.sDes = testVelControl(t)

        elif self.ctrlType == "xy_vel_z_pos":
            if self.xyzType == 1:
                self.sDes = testVelControl(t)
        
        elif self.ctrlType == "xyz_pos":
            # Hover at [0, 0, 0]
            if self.xyzType == 0:
                pass 
            # For simple testing
            elif self.xyzType == 99:
                self.sDes = testXYZposition(t)   
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
                elif self.xyzType >= 3:
                    pos_waypoint_min()
                
                # List of possible yaw trajectories
                # ---------------------------
                # Set desired yaw at every t_wps[i]
                if self.yawType == 0:
                    pass
                elif self.yawType == 1:
                    yaw_waypoint_timed()
                # Interpolate yaw between every waypoint, to arrive at desired yaw every t_wps[i]
                elif self.yawType == 2:
                    yaw_waypoint_interp()

                self.sDes = np.hstack((self.desPos, self.desVel, self.desAcc, self.desThr, self.desEul, self.desPQR, self.desYawRate)).astype(float)
        
        return self.sDes




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


def get_poly_cc(n, k, t):
    """ This is a helper function to get the coeffitient of coefficient for n-th
        order polynomial with k-th derivative at time t.
    """
    assert (n > 0 and k >= 0), "order and derivative must be positive."

    cc = np.ones(n)
    D  = np.linspace(n-1, 0, n)

    for i in range(n):
        for j in range(k):
            cc[i] = cc[i] * D[i]
            D[i] = D[i] - 1
            if D[i] == -1:
                D[i] = 0

    for i, c in enumerate(cc):
        cc[i] = c * np.power(t, D[i])

    return cc

# Minimum velocity/acceleration/jerk/snap Trajectory
def minSomethingTraj(waypoints, order):
    """ This function takes a list of desired waypoint i.e. [x0, x1, x2...xN] and
    time, returns a [8N,1] coeffitients matrix for the N+1 waypoints.

    1.The Problem
    Generate a full trajectory across N+1 waypoint is made of N polynomial line segment.
    Each segment is defined as 7 order polynomial defined as follow:
    Pi = ai_0 + ai1*t + ai2*t^2 + ai3*t^3 + ai4*t^4 + ai5*t^5 + ai6*t^6 + ai7*t^7

    Each polynomial has 8 unknown coefficients, thus we will have 8*N unknown to
    solve in total, so we need to come up with 8*N constraints.

    2.The constraints
    In general, the constraints is a set of condition which define the initial
    and final state, continuity between each piecewise function. This includes
    specifying continuity in higher derivatives of the trajectory at the
    intermediate waypoints.

    3.Matrix Design
    Since we have 8*N unknown coefficients to solve, and if we are given 8*N
    equations(constraints), then the problem becomes solving a linear equation.

    A * Coeff = B

    Let's look at B matrix first, B matrix is simple because it is just some constants
    on the right hand side of the equation. There are 8xN constraints,
    so B matrix will be [8N, 1].

    Now, how do we determine the dimension of Coeff matrix? Coeff is the final
    output matrix consists of 8*N elements. Since B matrix is only one column,
    thus Coeff matrix must be [8N, 1].

    Coeff.transpose = [a10 a11..a17...aN0 aN1..aN7]

    A matrix is tricky, we then can think of A matrix as a coeffient-coeffient matrix.
    We are no longer looking at a particular polynomial Pi, but rather P1, P2...PN
    as a whole. Since now our Coeff matrix is [8N, 1], and B is [8N, 8N], thus
    A matrix must have the form [8N, 8N].

    A = [A10 A12 ... A17 ... AN0 AN1 ...AN7
         ...
        ]

    Each element in a row represents the coefficient of coeffient aij under
    a certain constraint, where aij is the jth coeffient of Pi with i = 1...N, j = 0...7.
    """

    n = len(waypoints) - 1
    nb_coeff = order*2

    # initialize A, and B matrix
    A = np.zeros([nb_coeff*n, nb_coeff*n])
    B = np.zeros(nb_coeff*n)

    # populate B matrix.
    for i in range(n):
        B[i] = waypoints[i]
        B[i + n] = waypoints[i+1]

    # Constraint 1
    for i in range(n):
        A[i][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, 0)

    # Constraint 2
    for i in range(n):
        A[i+n][nb_coeff*i:nb_coeff*(i+1)] = get_poly_cc(nb_coeff, 0, 1)

    # Constraint 3
    for k in range(1, order):
        A[2*n+k-1][:nb_coeff] = get_poly_cc(nb_coeff, k, 0)

    # Constraint 4
    for k in range(1, order):
        A[2*n+(order-1)+k-1][-nb_coeff:] = get_poly_cc(nb_coeff, k, 1)

    if (order > 1):
        # Constraint 5
        for i in range(n-1):
            for k in range(1, nb_coeff-1):
                A[2*n+2*(order-1) + i*2*(order-1)+k-1][i*nb_coeff : (i*nb_coeff+nb_coeff*2)] = np.concatenate((get_poly_cc(nb_coeff, k, 1), -get_poly_cc(nb_coeff, k, 0)))

    # solve for the coefficients
    Coeff = np.linalg.solve(A, B)
    return Coeff
