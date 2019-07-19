# Quadcopter Simulation and Control (Quad_SimCon)

This project serves 2 purposes. The first is to provide a PyDy template for creating quadcopter dynamics and outputing the relevant equations of motion.

The second purpose is to provide a simple working simulation of the quadcopter's dynamics and a simple controller that can handle position control, and eventualy trajectory generation.

## PyDy Quadcopter

[PyDy](https://pypi.org/project/pydy/), short for Python Dynamics, is a tool kit made to enable the study of multibody dynamics. At it's core is the SymPy [mechanics package](https://docs.sympy.org/latest/modules/physics/mechanics/index.html#vector), which provides an API for building models and generating the symbolic equations of motion for complex multibody systems. 

In the *PyDy Scripts* folder, one will find multiple different scripts. Some express the drone's orientation in the NED frame, and some in the ENU frame.

__NED frame__ : The world is oriented in such a way that the *X* direction is **North**, *Y* is **East** and *Z* is **Down**. The drone's orientation in this frame is **front-right-down (frd)**. This is a conventional/classic frame used in aeronautics, and also the frame used for the PX4 multicopter controller.

__ENU frame__ : The world is oriented in such a way that the *X* direction is **East**, *Y* is **North** and *Z* is **Up**. The drone's orientation in this frame is **front-left-up (flu)**. This frame is widely used for its vizualizing simplicity (*z* is up), however it possesses a vizualizing complexity where "pitching up" actually results in a negative pitch angle.

The other difference in the provided scripts is that some use Euler angles *phi* (*&phi;*), *theta* (*&theta;*), *psi* (*&psi;*) (roll, pitch, yaw) to describes the drone's orientation, while the other scripts uses a quaternion.

__Euler angles__ : In the Euler angle scripts, the drone is first rotated about its *Z* axis by a certain yaw angle (heading), then about its new *Y* axis by a certain pitch angle (elevation) and then finaly about its new *X* axis by a certain roll angle (bank). The rotation order is thus a **Body ZYX** rotation. Using Euler angles, the resulting equations of motion possesses many sine and cosine functions, meaning that it requires more time to calculate. One must remember that these equations of motion are to be integrated in order to simulated the quadcopter's motion (using an ODE function for example). This means that the equations of motion are computed many time during a single timestep of the simulation.

__Quaternion__ : The use of a quaternion to describe the drone's rotation significantly decreases computing time, because of the absence of sine and cosine functions in the equations of motion. The quaternion is formed with the angle value first, followed by the 3 axis values, like so : `q = [q0, q1, q2, q3] = [qw, qx, qy, qz]`. While it is sometimes complex to understand the rotation expressed by a quaternion, the quadcopter attitude control provided in this project uses quaternions (sets a desired quaternion, computes a quaternion error, ...).

The quadcopter states are the following : 

* Position (*x*, *y*, *z*)
* Rotation (*&phi;*, *&theta;*, *&psi;*) or (*q0*, *q1*, *q2*, *q3*)
* Linear Velocity (*xdot*, *ydot*, *zdot*)
* Angular Velocity (*p*, *q*, *r*)

The PyDy scripts use the Kane Method to derive the system's equations and output a Mass Matrix (*MM*) and a right-hand-side vector (*RHS*). 

### PyDy Installation
To be able to run the PyDy scripts of this project, you need to first install PyDy and its dependancies.

If you have the pip package manager installed you can simply type:

`$ pip install pydy` 

Or if you have conda you can type:

`$ conda install -c conda-forge pydy`