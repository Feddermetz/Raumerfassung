#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 20 16:26:16 2022

@author: janzimon


Test ICP Algorithmus mit Pose-Daten

"""


from pylab import *
from  DataManager import DataManager
import poseCalculations as pc
import RobotData as rd
from iterative_closest_point import *

robot1 = DataManager()


nsim = 3

  # number of simulation

#filename = "ersterScanTestumgebung.csv"
#f = open("ersterScanTestumgebung2.csv")
#f = open("Testlauf_Etage2_GebB_30072022_all.csv")
f = open("Testlauf_Etage2_GebB_30072022.csv")
#f = open("ersterScanTestumgebungSingle.csv")
j = 0
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j < 15:
        print('*Datensatz: ', j, ' ********************************************************')
        sp = l.split(sep = ";")
        robot1.SplitDataStep5(sp)
        robot1.CreatePoseDataStep(j)
        robot1.CreateUssDataPosesStep(j)
        robot1.PlotMotorPoseData()
        robot1.PlotSensorPoseData()
        #print('PoseData: ', robot1.sensor_pose_data)
        px = np.array([])
        py = np.array([])
        cx = np.array([])
        cy = np.array([])
        if j != 0:
            tupel = robot1.sensor_pose_data[j-1]
            
            print('Tupel: ' , tupel)
        # previous point
            px = np.append(px, tupel[0])
            py = np.append(py, tupel[1])
            previous_points = np.vstack((px, py))
            print(previous_points)
            
            tupel = robot1.sensor_pose_data[j]
            # current points
            cx = np.append(cx, tupel[0])
            cy = np.append(cy,tupel[1])
            current_points = np.vstack((cx, cy))
            print(current_points)
            
            '''
            for _ in range(nsim):
                R, T = icp_matching(previous_points, current_points)
                print("R:", R)
                print("T:", T)
        '''
        #dmRobot1.PlotScanDataPoints()
        #dmRobot1.PlotMotorPoseData()
        #dmRobot1.PlotCylInGlobalCoordinates()
        show()
        #print('Letze Pose: ' , robot1.sensor_pose_data[j-1])
        #print('Aktuelle Pose: ' , robot1.sensor_pose_data[j])
        j += 1
f.close()


#print("MotorPosen")
#print(dmRobot1.motor_pose_data)
#print("SensorPosen")
#print(dmRobot1.sensor_pose_data)
#dmRobot1.PlotSensorDistanceData()
#dmRobot1.PrintData()
#dmRobot1.PlotRobotDataPolar()
#dmRobot1.PlotMotorPoseData()
#dmRobot1.PlotSensorPoseData()
#dmRobot1.PlotScanDataPoints()
#dmRobot1.PlotMotorPoseData()
#dmRobot1.PlotCylInGlobalCoordinates()
#show()
#dmRobot1.PlotScanDerCyl()

#dmRobot1.PrintData()
#dmRobot1.PlotMotorPoseData()
'''
print("uss_Data")
print(dmRobot1.uss_Data)
print("scanData")
print(dmRobot1.scanData)
print("scanDataPoints")
print(dmRobot1.scanDataPoints)
'''

'''
#zur analyse der richtung von sensorpunkten
for line in dmRobot1.scanDataPoints:
    for x,y in line:
        plot(x,y, 'go')
        show()
#for tuple in dmRobot1.scanData
'''


'''
print("USS_DATA:")
print(dmRobot1.uss_Data)
print("MotorData:")
print(dmRobot1.motor_positions)
'''
