# -*- coding: utf-8 -*-
"""
Created on Thu Aug 25 14:25:54 2022

@author: Jan
"""

# -*- coding: utf-8 -*-
"""
Created on Thu Aug 18 17:43:15 2022

test, da zur Anzeige aufeinanderfolgender Plots, um zu prüfen, ob PrepareICP funktioniert


@author: Jan
"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../utils/")

import copy
import itertools
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from pylab import show
from  DataManager import DataManager, EuclideanDistance
import poseCalculations as pc
import RobotData as rd
import matplotlib.pyplot as plt
#from graphslam.load import load_g2o_se2_JZ
from iterative_closest_point import *

robot = DataManager()
nsim = 3
f = open("Testlauf_Etage2_GebB_30072022.csv")
j = 0
edgeData = []
poseData = [(0,0,0,0)]
LINE = '###################################################################'


for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j < 90:
        print('*Datensatz: ', j+1, ' ******************************************************************************************************')
        sp = l.split(sep = ";")
        robot.SplitDataStep5(sp)
        robot.CreateRobotData(j)
        print('Pose',robot.sensor_pose_data[j])
        '''
        robot.PlotSensorPoseData()
        plt.grid()
        plt.title(j+1)
        show()
        '''
        mode = 'all'
        if mode == 'all':
            robot.PlotScanData(j)
            '''
            plt.plot(line[0],line[1], 'g.')
            plt.plot(robot.sensor_pose_data[j-1][0],robot.sensor_pose_data[j-1][1], 'b*')
            plt.plot(robot.sensor_pose_data[j][0],robot.sensor_pose_data[j][1], 'g*')
            '''
            label = ''
            plt.title("Messung Nr. " + str(j+1)[0:5]+ " - Ultraschalldaten")
            plt.xlabel("Winkel in °", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.ylabel("gemessene Distanz in mm", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.grid()
            show()


            robot.PlotDerivative(j)
            label = ''
            plt.title("Messung Nr. " + str(j+1)[0:5]+ " - Ableitung")
            plt.xlabel("Winkel in °", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.ylabel("Ableitung", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.grid()
            show()
            
            robot.PlotLandmarks(j)
            robot.PlotScanData(j)
            '''
            plt.plot(line[0],line[1], 'g.')
            plt.plot(robot.sensor_pose_data[j-1][0],robot.sensor_pose_data[j-1][1], 'b*')
            plt.plot(robot.sensor_pose_data[j][0],robot.sensor_pose_data[j][1], 'g*')
            '''
            label = ''
            plt.title("Messung Nr. " + str(j+1)[0:5]+ " - Ultraschalldaten mit Lanmarken")
            plt.xlabel("Winkel in °", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.ylabel("gemessene Distanz in mm", 
               family='serif', 
               color='k', 
               weight='normal', 
               size = 10,
               labelpad = 6)
            plt.grid()
            show()
    j += 1
f.close()
print(len(robot.allLandmarks))