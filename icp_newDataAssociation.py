# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 15:42:17 2022

GraphSlam-Umsetzung mit Testdaten: "Testlauf_Etage2_GebB_30072022.csv"

@author: Jan
"""

from pylab import show
from  DataManager import DataManager
import poseCalculations as pc
import RobotData as rd
import matplotlib.pyplot as plt
from graphslam.load import load_g2o_se2_JZ
import numpy as np
from iterative_closest_point import *
robot = DataManager()


nsim = 3
f = open("Testlauf_Etage2_GebB_30072022.csv")

j = 0

edgeData = []
poseData = [(0,0,0,0)]
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j > 99:
        print('*Datensatz: ', j+1, ' ********************************************************')
        
        sp = l.split(sep = ";")
        robot.SplitDataStep5(sp)
        robot.CreatePoseDataStep(j)
        robot.CreateUssDataPosesStep(j)
        #print('pose', robot.sensor_pose_data[j])
        #print('scanData: ', robot.scanData[j])
        #print('ussDaten', robot.scanDataPoints[j])
        
        
        #robot.PlotSensorPoseData()
        #robot.PlotMotorPoseData()
        #robot.PlotScanDataPoints('no400')
        robot.CreateDerivativesStep(j)
        #print('Derivatives: ',dmRobot1.derivatives[j])
        robot.CreateLandmarksDataStep(j)
        robot.PlotLmInGlobalCoordinates()
        show()
        #if len(robot.landmarkPairs[j]) > 1:
        #print('Landmarkpairs:' ,robot.landmarkPairs[j])
        #robot.PlotScanDerCyl(j)
        #show()
        #print('Valid LM: ' , validLandmarks)
        previousPoints= np.array([])
        currentPoints= np.array([])
        j += 1
'''
    if j >0:
        previousPoints, currentPoints = robot.PrepareICP(j)
        print('anzahl PP: ',len(previousPoints[0]))
        print('anzahl CP: ',len(currentPoints[0]))
        #print(robot.indexOfAngle)
        #ICP###################################################################
        for _ in range(nsim):
            R, T = icp_matching(previousPoints, currentPoints)
            print("R:", R)
            print("T:", T)
'''
#j += 1
f.close()
#print('Objects: ',robot.cylindersInGlobalCoordinates)
validLandmarks = robot.GetValidLandmarks(rd.minObservations)
robot.PlotValidLmInGC()
show()
print('Landmarken gesamt: ', len(robot.allLandmarks))
print('Valid LM: ' , validLandmarks)
'''
g = load_g2o_se2_JZ(poseData,edgeData)
g.plot()
g.calc_chi2()
g.optimize()
print(g._vertices)
g.plot()
'''

