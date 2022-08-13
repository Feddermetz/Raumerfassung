# -*- coding: utf-8 -*-
"""
Created on Mon Aug  1 15:42:17 2022

GraphSlam-Umsetzung mit Testdaten: "Testlauf_Etage2_GebB_30072022.csv"

@author: Jan
"""


from pylab import *
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
    if j < 2:
        print('*Datensatz: ', j+1, ' ********************************************************')
        
        sp = l.split(sep = ";")
        robot.SplitDataStep5(sp)
        robot.CreatePoseDataStep(j)
        robot.CreateUssDataPosesStep(j)
        #print('pose', robot.sensor_pose_data[j])
        #print('scanData: ', robot.scanData[j])
        #print('ussDaten', robot.scanDataPoints[j])
        
        '''
        robot.PlotMotorPoseData()
        robot.PlotScanDataPoints('no400')
        robot.CreateDerivativesStep(j)
        #print('Derivatives: ',dmRobot1.derivatives[j])
        robot.CreateCylinderDataStep(j)
        robot.PlotCylInGlobalCoordinates()
        show()
        robot.PlotScanDerCyl(j)
        show()
        '''
        # Test: Berechnung des Winkels in der folgemessung, unter der der selbe punkt beobachtet wurde
        #       und Zuweisung dieser Punkte zueinander, dann ICP oder so, 
        #       mal schauen wie sich das dann in einen Vektor umrechnen lässt
        if j >0:
            px, py, cx, cy = robot.PrepareICP(j)
            print('Px',px)
            print('Py',py)
            print('Cx',cx)
            print('Cy',cy)
            
        #ICP###################################################################
        px = np.array([])
        py = np.array([])
        cx = np.array([])
        cy = np.array([])
        mode = 'landmark'
        '''
        if j != 0:
            if mode == 'pose':
                tupel = dmRobot1.sensor_pose_data[j-1]
                px = np.append(px, tupel[0])
                py = np.append(py, tupel[1])
                previous_points = np.vstack((px, py))
                #print(previous_points)
                
                tupel = dmRobot1.sensor_pose_data[j]
                # current points
                cx = np.append(cx, tupel[0])
                cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
            elif mode == 'uss':
                for tupel in dmRobot1.scanDataPoints[j-1]:
                # previous points
                    px = np.append(px, tupel[0])
                    py = np.append(py,tupel[1])
                previous_points = np.vstack((px, py))
                #print(previous_points)
                for tupel in dmRobot1.scanDataPoints[j]:
                # previous points
                    cx = np.append(cx, tupel[0])
                    cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
                #print(current_points)
            elif mode == 'landmark':
                print(dmRobot1.cylindersInGlobalCoordinates[j-1])
                print(dmRobot1.cylindersInGlobalCoordinates[j])
                
                for tupel in dmRobot1.cylindersInGlobalCoordinates[j-1]:
                # previous points
                    px = np.append(px, tupel[0])
                    py = np.append(py,tupel[1])
                previous_points = np.vstack((px, py))
                print('pp: ',previous_points)
                for tupel in dmRobot1.cylindersInGlobalCoordinates[j]:
                # previous points
                    cx = np.append(cx, tupel[0])
                    cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
                print('cp: ',current_points)
                
            for _ in range(nsim):
                R, T = icp_matching(previous_points, current_points)
                print("R:", R)
                print("T:", T)
                
        #######################################################################
        '''
        '''
        #Edge Daten erstellen:
        if dmRobot1.all_Data[j][0] == 1:
            lastPose = (0,0,0)
        else:
            lastPose = dmRobot1.motor_pose_data[j-1]
        aktPose = dmRobot1.motor_pose_data[j]
        poseData.append((j+1, aktPose[0],aktPose[1], aktPose[2]))
        edgeData.append([j, j+1, aktPose[0]-lastPose[0],aktPose[1]-lastPose[1], aktPose[2]-lastPose[2], 100,0,0,100,0,100])
        print('poseData: ',poseData)
        print('edgeData: ', edgeData)
        
        if j >0:
            g = load_g2o_se2_JZ(poseData,edgeData)
            g.plot()
            g.calc_chi2()
            g.optimize()
            print(g._vertices)
            g.plot()
            '''
        j += 1
f.close()
#print('Objects: ',robot.cylindersInGlobalCoordinates)

'''
g = load_g2o_se2_JZ(poseData,edgeData)
g.plot()
g.calc_chi2()
g.optimize()
print(g._vertices)
g.plot()
'''

