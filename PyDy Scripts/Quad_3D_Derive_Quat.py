import numpy as np
from sympy import symbols, Matrix
from sympy.physics.mechanics import *

# Reference frames and Points
# ---------------------------
N = ReferenceFrame('N')  # Inertial Frame
B = ReferenceFrame('B')  # Drone after X (roll) rotation (Final rotation)
C = ReferenceFrame('C')  # Drone after Y (pitch) rotation
D = ReferenceFrame('D')  # Drone after Z (yaw) rotation

No = Point('No')
Bcm = Point('Bcm')  # Drone's center of mass
M1 = Point('M1')    # Motor 1 is front left, then the rest increments CW (Drone is in X configuration, not +)
M2 = Point('M2')
M3 = Point('M3')
M4 = Point('M4')

# Variables
# ---------------------------
# x, y and z are the drone's coordinates in the inertial frame, expressed with the inertial frame
# xdot, ydot and zdot are the drone's speeds in the inertial frame, expressed with the inertial frame
# phi, theta and phi represents the drone's attitude in the inertial frame, expressed with a ZYX Body rotation
# p, q and r are the drone's angular velocities in the inertial frame, expressed with the body frame

x, y, z, xdot, ydot, zdot = dynamicsymbols('x y z xdot ydot zdot')
q0, q1, q2, q3, p, q, r = dynamicsymbols('q0 q1 q2 q3 p q r')

xd, yd, zd, xdotd, ydotd, zdotd = dynamicsymbols('x y z xdot ydot zdot', 1)
q0d, q1d, q2d, q3d, pd, qd, rd = dynamicsymbols('q0 q1 q2 q3 p q r', 1)

# Constants
# ---------------------------
mB, g, dxm, dym, dzm, IBxx, IByy, IBzz = symbols('mB g dxm dym dzm IBxx IByy IBzz')
ThrM1, ThrM2, ThrM3, ThrM4, TorM1, TorM2, TorM3, TorM4 = symbols('ThrM1 ThrM2 ThrM3 ThrM4 TorM1 TorM2 TorM3 TorM4')

# Rotation ZYX Body
# ---------------------------
B.orient(N, 'Quaternion', [q0, q1, q2, q3])

# Origin
# ---------------------------
No.set_vel(N, 0)

# Translation
# ---------------------------
Bcm.set_pos(No, x*N.x + y*N.y + z*N.z)
Bcm.set_vel(N, Bcm.pos_from(No).dt(N)) 

M1.set_pos(Bcm,  dxm*B.x + dym*B.y + dzm*B.z)
M2.set_pos(Bcm,  dxm*B.x - dym*B.y + dzm*B.z)
M3.set_pos(Bcm, -dxm*B.x - dym*B.y + dzm*B.z)
M4.set_pos(Bcm, -dxm*B.x + dym*B.y + dzm*B.z)
M1.v2pt_theory(Bcm, N, B)
M2.v2pt_theory(Bcm, N, B)
M3.v2pt_theory(Bcm, N, B)
M4.v2pt_theory(Bcm, N, B)

# Inertia Dyadic
# ---------------------------
IB = inertia(B, IBxx, IByy, IBzz)

# Create Bodies
# ---------------------------
BodyB = RigidBody('BodyB', Bcm, B, mB, (IB, Bcm))
BodyList = [BodyB]

# Forces and Torques
# ---------------------------
Grav_Force = (Bcm, -mB*g*N.z)
FM1 = (M1, ThrM1*B.z)
FM2 = (M2, ThrM2*B.z)
FM3 = (M3, ThrM3*B.z)
FM4 = (M4, ThrM4*B.z)

TM1 = (B,  TorM1*B.z)
TM2 = (B, -TorM2*B.z)
TM3 = (B,  TorM3*B.z)
TM4 = (B, -TorM4*B.z)
ForceList = [Grav_Force, FM1, FM2, FM3, FM4, TM1, TM2, TM3, TM4]

Equat = Matrix([[-q1,  q0, -q3,  q2],
                [-q2,  q3,  q0, -q1],
                [-q3, -q2,  q1,  q0]])

Gquat = Matrix([[-q1,  q0,  q3, -q2],
                [-q2, -q3,  q0,  q1],
                [-q3,  q2, -q1,  q0]])

rotVel = Matrix([[p],[q],[r]])

mprint(Gquat)
mprint(rotVel)

quat_dot = Gquat.T*rotVel/2
mprint(quat_dot)

# Kinematic Differential Equations
# ---------------------------
kd = [xdot - xd, ydot - yd, zdot - zd, q0d - quat_dot[0], q1d - quat_dot[1], q2d - quat_dot[2], q3d - quat_dot[3]]
mprint(kd)

# Kane's Method
# ---------------------------
KM = KanesMethod(N, q_ind=[x, y, z, q0, q1, q2, q3], u_ind=[xdot, ydot, zdot, p, q, r], kd_eqs=kd)
(fr, frstar) = KM.kanes_equations(BodyList, ForceList)

# Equations of Motion
# ---------------------------
MM = KM.mass_matrix_full
kdd = KM.kindiffdict()
rhs = KM.forcing_full
MM = MM.subs(kdd)
rhs = rhs.subs(kdd)


MM.simplify()
MM = MM.subs(q0**2 + q1**2 + q2**2 + q3**2, 1)
print('Mass Matrix')
print('-----------')
mprint(MM)

rhs.simplify()
print()
print('Right Hand Side')
print('---------------')
mprint(rhs)
print()


rhs = rhs.subs(q0**2 + q1**2 + q2**2 + q3**2, 1)
mprint(rhs)

# So, MM*x = rhs, where x is the State Derivatives
# Solve for x
stateDot = MM.inv()*rhs
print('State Derivatives')
print('-----------------------------------')
mprint(stateDot)
print()

# POST-PROCESSING
# ---------------------------

# Useful Numbers
# ---------------------------
# p, q, r are the drone's angular velocities in its own frame.
# These calculations are not relevant to the ODE, but might be used for control
print('P, Q, R (Angular velocities in drone frame)')
print('-------------------------------------------')
mprint(dot(B.ang_vel_in(N), B.x))
print()
mprint(dot(B.ang_vel_in(N), B.y))
print()
mprint(dot(B.ang_vel_in(N), B.z))
print()

# u, v, w are the drone's speed in its own frame.
# These calculations are not relevant to the ODE, but might be used for control
u = dot(Bcm.vel(N).subs(kdd), B.x)
v = dot(Bcm.vel(N).subs(kdd), B.y)
w = dot(Bcm.vel(N).subs(kdd), B.z)
print('u, v, w (Velocities in drone frame)')
print('-----------------------------------')
mprint(u)
print()
mprint(v)
print()
mprint(w)
print()

# uFlat, vFlat, wFlat are the drone's speed in its frame, only after the YAW rotation. 
# These calculations are not relevant to the ODE, but might be used for control
uFlat = dot(Bcm.vel(N).subs(kdd), D.x)
vFlat = dot(Bcm.vel(N).subs(kdd), D.y)
wFlat = dot(Bcm.vel(N).subs(kdd), D.z)
print('uFlat, vFlat, wFlat (Velocities in drone frame (after Yaw))')
print('-----------------------------------------------------------')
mprint(uFlat)
print()
mprint(vFlat)
print()
mprint(wFlat)
print()



