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


from pylab import show
from  DataManager import DataManager
import matplotlib.pyplot as plt
dmRobot1 = DataManager()


#f = open("ersterScanTestumgebung.csv")
#f = open("ersterScanTestumgebungUss50ms.csv")
f = open("ersterScanTestumgebungUss100ms.csv")

j = 0
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    sp = l.split(sep = ";")
    dmRobot1.SplitDataStep5(sp)
    dmRobot1.CreatePoseDataStep(j)
    dmRobot1.CreateUssDataPosesStep(j)
    dmRobot1.PlotScanDataPoints('all')
    dmRobot1.PlotMotorPoseData()
    plt.title("Testscan Ultraschallmessung mit 100ms Delay")
    plt.xlabel("Distanz in mm", 
       family='serif', 
       color='k', 
       weight='normal', 
       size = 10,
       labelpad = 6)
    plt.ylabel("Distanz in mm", 
       family='serif', 
       color='k', 
       weight='normal', 
       size = 10,
       labelpad = 6)
    plt.grid()
    show()
    j += 1
f.close()


