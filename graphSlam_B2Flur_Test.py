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

dmRobot1 = DataManager()


f = open("Testlauf_Etage2_GebB_30072022.csv")

j = 0
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j < 5:
        print('***************************************************************')
        sp = l.split(sep = ";")
        dmRobot1.SplitDataStep5(sp)
        dmRobot1.CreatePoseDataStep(j)
        dmRobot1.CreateUssDataPosesStep(j)
        
        g = load_g2o_se2_JZ(dmRobot1.motor_pose_data,dmRobot1.uss_Data)
        g.plot()
        g.calc_chi2()
        g.optimize()
        print(g._vertices)
        g.plot()
        j += 1
f.close()


