# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import utils
import config

numFrames = 5


def updateLines(i, t_all, pos_all, quat_all, params, lines, ax):
    
    time = t_all[i*numFrames]
    pos = pos_all[i*numFrames]
    x = pos[0]
    y = pos[1]
    z = pos[2]

    dxm = params["dxm"]
    dym = params["dym"]
    dzm = params["dzm"]
    
    quat = quat_all[i*numFrames]

    if (config.orient == "NED"):
        z = -z
        quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])

    R = utils.quat2Dcm(quat)    
    motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
    motorPoints = np.dot(R, np.transpose(motorPoints))
    motorPoints[0,:] += x 
    motorPoints[1,:] += y 
    motorPoints[2,:] += z 
    
    lines[0][0].set_data(motorPoints[0,0:3], motorPoints[1,0:3])
    lines[0][0].set_3d_properties(motorPoints[2,0:3])
    lines[1][0].set_data(motorPoints[0,3:6], motorPoints[1,3:6])
    lines[1][0].set_3d_properties(motorPoints[2,3:6])
    ax.set_title(u"Time = {:.2f} s".format(time[0]))
    return lines
    

def ini_plot(fig, ax, pos, quat, params):
    
    dxm = params["dxm"]
    dym = params["dym"]
    dzm = params["dzm"]
    
    x = pos[0]
    y = pos[1]
    z = pos[2]

    if (config.orient == "NED"):
        z = -z
        quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])

    R = utils.quat2Dcm(quat)
    motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
    motorPoints = np.dot(R, np.transpose(motorPoints))
    motorPoints[0,:] += x 
    motorPoints[1,:] += y 
    motorPoints[2,:] += z 
    line1 = ax.plot(motorPoints[0,0:3], motorPoints[1,0:3], motorPoints[2,0:3], color='red')
    line2 = ax.plot(motorPoints[0,3:6], motorPoints[1,3:6], motorPoints[2,3:6], color='blue')
    lines = [line1, line2]

    return lines

def sameAxisAnimation(t_all, pos_all, quat_all, Ts, params):
    
    x = pos_all[:,0]
    y = pos_all[:,1]
    z = pos_all[:,2]

    if (config.orient == "NED"):
        z = -z

    fig = plt.figure()
    ax = p3.Axes3D(fig)
    
    # Setting the axes properties
    extraEachSide = 1
    maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + extraEachSide
    mid_x = 0.5*(x.max()+x.min())
    mid_y = 0.5*(y.max()+y.min())
    mid_z = 0.5*(z.max()+z.min())
    
    ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    ax.set_xlabel('X')
    if (config.orient == "NED"):
        ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
    elif (config.orient == "ENU"):
        ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('Y')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Altitude')
    ax.set_title('3D Test')
    
    lines = ini_plot(fig, ax, pos_all[0], quat_all[0], params)
    
    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, updateLines, int(len(x)/numFrames), fargs=(t_all, pos_all, quat_all, params, lines, ax), interval=int(Ts*1000*numFrames), blit=False, repeat=True)
    
    plt.show()
    
    return line_ani

