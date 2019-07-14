# -*- coding: utf-8 -*-
"""
Created on Sun Nov  4 15:51:44 2018

@author: John
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
import math

numFrames = 20


def updateLines(i, s_result, params, lines, ax):
    
    s_result = s_result[i*numFrames]
    x =     s_result[0]
    y =     s_result[1]
    z =     s_result[2]

    dxm = params["dxm"]
    dym = params["dym"]
    dzm = params["dzm"]
    
    R = rotationMatrix(s_result)
    motorPoints = np.array([[dxm, dym, dzm], [0, 0, 0], [dxm, -dym, dzm], [-dxm, -dym, dzm], [0, 0, 0], [-dxm, dym, dzm]])
    motorPoints = np.dot(R, np.transpose(motorPoints))
    motorPoints[0,:] += x 
    motorPoints[1,:] += y 
    motorPoints[2,:] += z 
    
    lines[0][0].set_data(motorPoints[0,0:3], motorPoints[1,0:3])
    lines[0][0].set_3d_properties(motorPoints[2,0:3])
    lines[1][0].set_data(motorPoints[0,3:6], motorPoints[1,3:6])
    lines[1][0].set_3d_properties(motorPoints[2,3:6])
    ax.set_title(u"Time = {}, Angle:".format(i))
    
    return lines
    
def rotationMatrix(s_result):
    
    phi =   s_result[3]
    theta = s_result[4]
    psi =   s_result[5]
    cphi = math.cos(phi)
    cthe = math.cos(theta)
    cpsi = math.cos(psi)
    sphi = math.sin(phi)
    sthe = math.sin(theta)
    spsi = math.sin(psi)
    
    R = np.array([[cthe*cpsi, sphi*sthe*cpsi - cphi*spsi, cphi*sthe*cpsi + sphi*spsi],
                  [cthe*spsi, sphi*sthe*spsi + cphi*cpsi, cphi*sthe*spsi - sphi*cpsi],
                  [    -sthe,                  sphi*cthe,                  cphi*cthe]])
    
    return R

def ini_plot(fig, ax, s_result, params):
    
    dxm = params["dxm"]
    dym = params["dym"]
    dzm = params["dzm"]
    
    x =     s_result[0]
    y =     s_result[1]
    z =     s_result[2]

    R = rotationMatrix(s_result)
    motorPoints = np.array([[dxm, dym, dzm], [0, 0, 0], [dxm, -dym, dzm], [-dxm, -dym, dzm], [0, 0, 0], [-dxm, dym, dzm]])
    motorPoints = np.dot(R, np.transpose(motorPoints))
    motorPoints[0,:] += x 
    motorPoints[1,:] += y 
    motorPoints[2,:] += z 
    line1 = ax.plot(motorPoints[0,0:3], motorPoints[1,0:3], motorPoints[2,0:3], color='red')
    line2 = ax.plot(motorPoints[0,3:6], motorPoints[1,3:6], motorPoints[2,3:6], color='blue')
    lines = [line1, line2]
    
    return lines

def sameAxisAnimation(s_result, Ts, params):
    
    x =     s_result[:,0]
    y =     s_result[:,1]
    z =     s_result[:,2]

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
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    ax.set_ylabel('Y')
    ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    ax.set_zlabel('Z')
    ax.set_title('3D Test')
    
    lines = ini_plot(fig, ax, s_result[0], params)
    
    # Creating the Animation object
    line_ani = animation.FuncAnimation(fig, updateLines, int(len(x)/numFrames), fargs=(s_result, params, lines, ax), interval=int(Ts*1000*numFrames), blit=False, repeat=True)
    
    plt.show()
    
    return line_ani

##
#x = np.linspace(0,0)
#y = np.linspace(0,0)
#z = np.linspace(0,0)
#phi = np.linspace(0,0)
#theta = np.linspace(0,0)
#psi = np.linspace(0,pi)
#
#params = {}
#params["dxm"] = 0.16 
#params["dym"] = 0.16
#params["dzm"] = 0.05
#    
#s_result = np.transpose(np.array([x,y,z,phi,theta,psi]))
#
#line_ani = sameAxisAnimation(s_result, 0.005, params)
#
#fig = plt.figure()
#ax = p3.Axes3D(fig)
#    
## Setting the axes properties
#extraEachSide = 1
#maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + extraEachSide
#mid_x = 0.5*(x.max()+x.min())
#mid_y = 0.5*(y.max()+y.min())
#mid_z = 0.5*(z.max()+z.min())
#    
#ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
#ax.set_xlabel('X')
#ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
#ax.set_ylabel('Y')
#ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
#ax.set_zlabel('Z')
#ax.set_title('3D Test')
#    
#lines = ini_plot(fig, ax, s_result[0], params)
## Creating the Animation object
#line_ani = animation.FuncAnimation(fig, updateLines, len(x), fargs=(s_result, params, lines), interval=50, blit=False, repeat=True)
#    
#plt.show()