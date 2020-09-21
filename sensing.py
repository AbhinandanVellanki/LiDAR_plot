# -*- coding: utf-8 -*-
"""
Created on Wed Oct  2 15:34:26 2019

@author: abhin
"""
import math
import numpy as np
import matplotlib.pyplot as plt

#load LIDAR data in polar form
scan= np.loadtxt('laserscan.dat')
angle = np.linspace(-math.pi/3, math.pi/3, np.shape(scan)[0], endpoint='true')

# Q2 (a): to plot laser end points from LIDAR data:

x=scan*np.cos(angle)#convert LIDAR data into cartesian coordinates
y=scan*np.sin(angle)
points=np.stack((x,y), axis=-1)
plt.gca().set_aspect('equal', adjustable='box')
plt.plot(points[: ,0],points[: ,1], 'o', color='black')
plt.show()

#Q2 (c): to plot centre of robot, lidar and lidar endpoints in world frame

#creating Transformation matrix: world to robot
awr=math.pi/4
pwr=[[1.0],[0.5]] #centre of robot
Rwr=np.matrix([[math.cos(awr),-math.sin(awr)],[math.sin(awr), math.cos(awr)]]) #Rotation matrix

#creating Transformation matrix: robot to sensor
ars=math.pi/2
prs=[[0.2],[0.0]] #centre of sensor
Rrs=np.matrix([[math.cos(ars),-math.sin(ars)],[math.sin(ars), math.cos(ars)]]) #Rotation matrix

print("Centre of the robot in World Coordrinates: ",pwr)
plt.gca().set_aspect('equal', adjustable='box')
plt.plot(pwr[0],pwr[1],'*', color='red')

#converting sensor coordinates to world frame
pws=np.matmul(Rwr,prs)+pwr
plt.plot(pws[0],pws[1],'o', color='blue')
print("Centre of Lidar Sensor in World Coordinates: ",pws)

#converting LIDAR data to robot frame
pointsrf=np.matmul(Rrs,points.T)+prs
#converting LIDAR data to world frame
pointswf=np.matmul(Rwr,pointsrf)+pwr
pointswf=pointswf.T
plt.plot(pointswf[: ,0],pointswf[: ,1], 'o', color='black')
plt.show()




