# -*- coding: utf-8 -*-
"""
Created on Tue Jun 28 19:42:36 2022

@author: Jan

Erster Versuch mit Testdaten "ersterScanTestumgebung.csv"

Raum:                          190cm
    ###################################################################
    #                                                                 #
    #                                                                 #
    #                                                                 #
    #                                                                 #
    #     >>>>>>>>>>>>>>>>>>-Roboterpfad->>>>>>>>>>>>>>>>>>>>>>>>     #
    #                                                                 #
    #                                                                 # 100cm
    #                                                                 #
    #                          100cm                                  #
    #         ############################################            #
    #         #                                          #            #
    #         #                                          #20cm        #
    #         #                                          #            #
    ###########                                          ##############
"""


from pylab import *
from  DataManager import DataManager
import poseCalculations as pc
import RobotData as rd
import matplotlib.pyplot as plt
dmRobot1 = DataManager()


#f = open("ersterScanTestumgebung2.csv")
#f = open("ersterScanTestumgebungUss50ms.csv")
f = open("Testlauf_Etage2_GebB_30072022.csv")
#f = open("ersterScanTestumgebungUss50ms_tstMDATA.csv")




#f = open("ersterScanTestumgebungSingle.csv")
j = 0
for l in f: #Simuliert die eingehenden Bluetoothdaten 
#if j < 3:
    print('***************************************************************')
    sp = l.split(sep = ";")
    dmRobot1.SplitDataStep5(sp)
    dmRobot1.CreatePoseDataStep(j)
    #print("motor_pose_data Index: ", len(dmRobot1.motor_pose_data))
    #print(dmRobot1.motor_pose_data)
    #print("Sensor_pose_data Index: ", len(dmRobot1.sensor_pose_data))
    #print(dmRobot1.sensor_pose_data)
#if j == 0 or j ==9:
    print(dmRobot1.all_Data[j])
    dmRobot1.CreateUssDataPosesStep(j)
    print("usspunkte: ", len(dmRobot1.scanDataPoints[j]) )
    print(dmRobot1.scanDataPoints[j])
    #print(dmRobot1.scanDataPoints[j])
    #dmRobot1.CreateDerivatives()
    #dmRobot1.CreateCylinderData()
    j += 1
    #dmRobot1.PlotScanDataPoints()
    dmRobot1.PlotScanDataPoints()
    dmRobot1.PlotMotorPoseData()
    
        #dmRobot1.PlotMotorPoseData()
        #dmRobot1.PlotCylInGlobalCoordinates()
    plt.title(sp[0])
    show()
f.close()

'''
dmRobot1.PlotSensorPoseData()
show()
dmRobot1.PlotScanDataPoints()
show()
'''


print("motordaten")
print(dmRobot1.motor_positions)
print("MotorPosen")
print(dmRobot1.motor_pose_data)
print("SensorPosen")
print(dmRobot1.sensor_pose_data)
print("motordaten")
print(dmRobot1.motor_positions)


#dmRobot1.PlotSensorDistanceData()
#dmRobot1.PrintData()
#dmRobot1.PlotRobotDataPolar()
dmRobot1.PlotMotorPoseData()
show()
dmRobot1.PlotSensorPoseData()
#dmRobot1.PlotScanDataPoints()
#dmRobot1.PlotMotorPoseData()
#dmRobot1.PlotCylInGlobalCoordinates()
show()
#dmRobot1.PlotScanDerCyl()
'''
print("uss_Data")
print(dmRobot1.uss_Data)
print("scanData")
print(dmRobot1.scanData)
print("scanDataPoints")
print(dmRobot1.scanDataPoints)
'''

