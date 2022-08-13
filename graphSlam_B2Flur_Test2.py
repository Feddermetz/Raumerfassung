# -*- coding: utf-8 -*-
"""
Created on Thu Aug  4 13:11:10 2022

@author: Jan


GraphSlam-Umsetzung mit Testdaten: "Testlauf_Etage2_GebB_30072022.csv"
Zweiter Anlauff mit anderer Bibliothek - kein ICP, da ICP-Ergebnisse  unbrauchbar

"""


from pylab import *
from  DataManager import DataManager
import poseCalculations as pc
import RobotData as rd
import matplotlib.pyplot as plt
from graphslam.load import load_g2o_se2_JZ
import numpy as np
from iterative_closest_point import *
dmRobot1 = DataManager()

nsim = 3
f = open("Testlauf_Etage2_GebB_30072022.csv")

j = 0
edgeData = []
poseData = [(0,0,0,0)]
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j < 5:
        print('***************************************************************')
        sp = l.split(sep = ";")
        print('Datensatz: ', sp[0])
        dmRobot1.SplitDataStep5(sp)
        dmRobot1.CreatePoseDataStep(j)
        dmRobot1.CreateUssDataPosesStep(j)
        dmRobot1.CreateDerivativesStep(j)
        dmRobot1.CreateCylinderDataStep(j)
        #dmRobot1.PlotDerivative(j)
        dmRobot1.PlotScanDerCyl(j)
        show()
        print('Len Derivatives: ', len(dmRobot1.derivatives[j]))
        print('Cylinders: ', dmRobot1.cylindersInGlobalCoordinates)
        print('allCylinders: ', dmRobot1.allCylinders)
        print('Länge: ', len(dmRobot1.cylindersFromScanData))
        dmRobot1.PlotMotorPoseData()
        dmRobot1.PlotScanDataPoints()
        dmRobot1.PlotCylInGlobalCoordinates()
        show()
    
        j += 1
f.close()

dmRobot1.CreateDerivatives()
dmRobot1.CreateCylinderData()
dmRobot1.PlotAllCylinders()
dmRobot1.PlotMotorPoseData()
show()

'''
g = load_g2o_se2_JZ(poseData,edgeData)
g.plot()
g.calc_chi2()
g.optimize()
print(g._vertices)
g.plot()
'''

