# -*- coding: utf-8 -*-
"""
Created on Tue May 31 13:36:05 2022

@author: Jan
"""

from pylab import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy.interpolate as si

from math import sin, cos, tan, pi, radians
import RobotData as rd


def CalcPivotPose(oldPose, motorData):
    distances = ConvertRevsToDistance(motorData)
    distLeft = distances[0]
    distRight = distances[1]
    thetaOld = oldPose[2]
    if distLeft == distRight : #Geradeausfahrt
        # Siehe Bewegungsmodell 2. Fall
        l = distLeft # = distRight
        thetaNew = thetaOld
        xNew = oldPose[0]+l*cos(thetaOld)
        yNew = oldPose[1]+l*sin(thetaOld)
        return(xNew,yNew,thetaNew)
    else: # Kurvenfahrt
        # Siehe Bewegungsmodell 1. Fall
        alpha = (distRight-distLeft)/rd.WheelBaseMm
        R = distLeft/alpha
        xC = oldPose[0]-(R+rd.WheelBaseMm/2)*sin(thetaOld)
        yC = oldPose[1]-(R+rd.WheelBaseMm/2)*(-1)*cos(thetaOld)
        thetaNew = (thetaOld+alpha)%(2*pi)
        #thetaNew = (thetaOld+alpha)
        xNew = xC+(R+rd.WheelBaseMm/2)*sin(thetaNew)
        yNew = yC+(R+rd.WheelBaseMm/2)*(-1)*cos(thetaNew)
        return (xNew, yNew, thetaNew)

def CalcSensorPose(oldPose, motorData): #Gibt die Pose des Sensors zurück
    thetaOld = oldPose[2]
    xNew = oldPose[0]-rd.sensorOffset*cos(thetaOld)
    yNew = oldPose[1]-rd.sensorOffset*sin(thetaOld)
    return (xNew, yNew, thetaOld)

def CalcPoint(pose, valuePair): # Berechnung eines Punktes (aus Pose und wertepaar (winkel, distanz)) für globales Koordinatensystem
    thetaOld = pose[2]
    #alpha = thetaOld - radians(70) + radians(valuePair[0])
    alpha = thetaOld + radians(valuePair[0])
    thetaNew = alpha%(2*pi)
    xNew = pose[0]+valuePair[1]*cos(thetaNew)
    yNew = pose[1]+valuePair[1]*sin(thetaNew)
    return (xNew, yNew)


'''
def CalcPoint(pose, angle):
    thetaNew = pose[2]+angle
    # Hier neuen Punkt berechnen
    xNew = pose[0]-rd.sensorPivotOffset*cos(thetaNew)
    yNew = pose[1]-rd.sensorOffset*sin(thetaNew)
    point = (1.0,2.0, thetaNew)
    return point
'''

def ConvertRevsToDistance(motorData):
    ''' TODO: Welche Fahrsituationen kann es geben? Das sollte hier noch ausprogrammiert werden!
            Motoren laufen in unterscheidliche Richtungen:
                1. mLinks < 0, mrechts > 0: vorwärts
                2. mLinks > 0, mrechts < 0: rückwärts
            Motoren laufen in gleiche Richtung: 
                3. mlinks > 0, mRechts > 0 : Dregung nach links
                4. mlinks < 0, mRechts < 0 : Drehung nach rechts
    
    '''
    #if mode == 1: # Geradeausfahrt,  Berechnung aus umdrehungen(ungenau, da umdrehungen des motors nicht realen umdreheungen des Rades entsprechen)
    #    distL = (motorData[1]/360) * pi * rd.WheelCircumferenceMm
    #    distR = (motorData[2]/360) * pi * rd.WheelCircumferenceMm
    #else: # Geradeausfahrt, Berechnung über Laufzeit der Motoren und empirisch gemessener Distanz bei 60rmp
    
    '''
    # unser roboter fährt nur geradaus oder dreht sich auf der stelle. Dies ist erkennbar an motordaten und winkel. 
    # überlegen, ob die distL und distR nach berechung über laufzeit noch um die prozentuale abweichung der motordaten 
    # angepasst werden soll, dann würden kurven korrekt berechnet werden (calcPivotPoint)
    '''
    distL =  distR = (motorData[3]/1000)*rd.speed60rpm 
    return (distL, distR)

'''
aktPose = (0,0,0)        # (x,y,theta)
motorData1 = (5,180,180)    # (messungsnummer, motorWinkelLinks, motorWinkelRechts)


newPose = CalcNewPose((0,0,0), (1,360,360))
print(newPose)
newPose = CalcNewPose(newPose, (2,-360,-360))
print(newPose)


#print(CalcNewPose(aktPose, motorData2))
'''























