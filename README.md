# Quadcopter Simulation and Control (Quad_SimCon)

This project serves 2 purposes. The first is to provide a PyDy template for creating quadcopter dynamics and outputing the relevant equations of motion.

The second purpose is to provide a simple working simulation of the quadcopter's dynamics and a simple controller that can handle position control, and eventualy trajectory generation.

## PyDy Quadcopter

[PyDy](https://pypi.org/project/pydy/), short for Python Dynamics, is a tool kit made to enable the study of multibody dynamics. At it's core is the SymPy [mechanics package](https://docs.sympy.org/latest/modules/physics/mechanics/index.html#vector), which provides an API for building models and generating the symbolic equations of motion for complex multibody systems. 

In the *PyDy Scripts* folder, you will find multiple different scripts. Some express the drone's orientation in the NED frame, and some in the ENU frame.

__NED frame__ : The world is oriented in such a way that the *x* direction is **North**, *y* is **East** and *z* is **Down**. The drone's orientation in this frame is **front-right-down (frd)**. This is a conventional/classic frame used in aeronautics, and also the frame used for the PX4 multicopter controller.

__ENU frame__ : The world is oriented in such a way that the *x* direction is **East**, *y* is **North** and *z* is **Up**. The drone's orientation in this frame is **front-left-up (flu)**. This frame is widely used for its vizualizing simplicity (*z* is up), however it possesses a vizualizing complexity where "pitching up" actually results in a negative pitch angle.

The other difference in the provided scripts is that some use Euler angles (roll, pitch, yaw) ($latex \theta$)

### PyDy Installation
To be able to run the PyDy scripts of this project, you need to first install PyDy and its dependancies.

If you have the pip package manager installed you can simply type:

`$ pip install pydy` 

Or if you have conda you can type:

`$ conda install -c conda-forge pydy`