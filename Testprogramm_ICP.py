# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 15:42:17 2022

Testprogramm für ICP mit verschiedenen Datenübergaben.

@author: Jan
"""

from pylab import show
from  DataManager import DataManager
import poseCalculations as pc
import RobotData as rd
import matplotlib.pyplot as plt
import numpy as np
from iterative_closest_point import *
robot = DataManager()


nsim = 3
f = open("Testlauf_Etage2_GebB_30072022.csv")
convergecount = 0
convergeIDList = []
j = 0

edgeData = []
poseData = [(0,0,0,0)]
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    print('*Datensatz: ', j+1, ' ********************************************************')
    
    sp = l.split(sep = ";")
    robot.SplitDataStep5(sp)
    robot.CreateRobotData(j)
    plt.title("ICP - Explorationsfahrt"+ str(j+1) )
    plt.xlabel("mm", 
       family='serif', 
       color='k', 
       weight='normal', 
       size = 10,
       labelpad = 6)
    plt.ylabel("mm", 
       family='serif', 
       color='k', 
       weight='normal', 
       size = 10,
       labelpad = 6)
    plt.grid()
    
    previous_Points= np.array([])
    current_Points= np.array([])
    px = np.array([])
    py = np.array([])
    cx = np.array([])
    cy = np.array([])

    error = 999
    print("Roboteraktion", robot.motor_positions[j][0]) 
    for tupel in robot.scanDataPoints[j-1]:
    # previous points
        #print(tupel)
        px = np.append(px, tupel[0])
        py = np.append(py,tupel[1])
    previous_Points = np.vstack((px, py))
    #print(previous_points)
    for tupel in robot.scanDataPoints[j]:
    # previous points
        cx = np.append(cx, tupel[0])
        cy = np.append(cy,tupel[1])
    current_Points = np.vstack((cx, cy))
    print('anzahl PP: ',len(previous_Points[0]))
    print('anzahl CP: ',len(current_Points[0]))
    #ICP###################################################################
    for _ in range(nsim):
        converged, error, R, T = icp_matching(previous_Points, current_Points)
        print("R:", R)
        print("T:", T)
        
    if converged:
        convergecount += 1
    j += 1
f.close()

print('Anzahl converged' , convergecount)


