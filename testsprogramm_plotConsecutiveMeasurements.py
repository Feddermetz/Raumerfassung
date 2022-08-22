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
        robot.PlotSensorPoseData()
        plt.grid()
        plt.title(j+1)
        show()
        
        if j>0:
            mode = 'no400'
            if mode == 'all':
                for line in robot.scanDataPoints[j-1]:
                    plt.plot(line[0],line[1], 'b.')
                for line in robot.scanDataPoints[j]:
                    plt.plot(line[0],line[1], 'g.')
                plt.plot(robot.sensor_pose_data[j-1][0],robot.sensor_pose_data[j-1][1], 'b*')
                plt.plot(robot.sensor_pose_data[j][0],robot.sensor_pose_data[j][1], 'g*')
                    
            elif mode =='no400':
                for line in robot.scanDataPointsNo400[j-1]:
                    plt.plot(line[0],line[1], 'b.')
                for line in robot.scanDataPointsNo400[j]:
                    plt.plot(line[0],line[1], 'g.')
                plt.plot(robot.sensor_pose_data[j-1][0],robot.sensor_pose_data[j-1][1], 'b*')
                plt.plot(robot.sensor_pose_data[j][0],robot.sensor_pose_data[j][1], 'g*')
                


            label = ''
            plt.title(j+1)
            plt.grid()
            show()

    j += 1
f.close()
