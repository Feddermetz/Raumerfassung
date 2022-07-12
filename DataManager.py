# -*- coding: utf-8 -*-
"""
Created on Mon May 30 14:33:59 2022

@author: Jan

Datenmodellierung
"""

from pylab import *
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy.interpolate as si
import poseCalculations as pc
import math
import RobotData as rd

def PlotValues(data,index,style):
    x_val = [x[0] for x in data[index]]
    y_val = [x[1] for x in data[index]]
    plt.plot(x_val,y_val, style)
    

def CalcDerivative(scanLine, minDist):
    x_val = [float(x[0]) for x in scanLine]
    y_val = [float(x[1]) for x in scanLine]
    derivatives = []
    derivatives+={(x_val[0],0)}
    
    for i in range(1,len(scanLine) -1):
        if (y_val[i+1] >= rd.minValidSensorData and y_val[i+1] >= rd.minValidSensorData): # TODO: was wird gemacht wenn eine ungültige Distanz gemessen wird?
            derivative = (y_val[i+1]-y_val[i-1])/(2+5*rd.angleCorrection) # Berechnung über Differenzenquotien
            derivatives+=({(x_val[i],derivative)})
        else:
            derivatives+=({(x_val[i],0)})
    derivatives+=({(x_val[(len(scanLine)-1)],0)})
    return derivatives

def FindCylinders(scan, derivative, jump, minDist):
    cylList = []
    onCyl = False
    sumRay, sumDepth, rays = 0.0, 0.0, 0
    for index, tupel in enumerate(derivative):
        if onCyl == False and tupel[1] < -jump: # Wir haben einen Zylinder gefunden
            onCyl = True
            rays = 1
            sumRay = tupel[0]
            distanceVal = scan[index]
            sumDepth = distanceVal[1]
        elif onCyl == True and tupel[1] < -jump:
            #Wir haben einen zweiten Zylinderanfang gefunden,wir setzen alle Werte auf Start
            rays = 1
            sumRay = tupel[0]
            distanceVal = scan[index]
            sumDepth = distanceVal[1]
        elif onCyl == True and tupel[1] > jump: #Zylinderende
            onCyl = False
            cylCenter = sumRay/rays 
            cylDistance = sumDepth/rays 
            cylList += {(cylCenter,cylDistance)}
        elif onCyl == True:
            rays += 1
            sumRay += tupel[0]
            distanceVal += scan[index]
            sumDepth += distanceVal[1]
        else:
            z=0
        #print(onCyl, rays, sumRay, tupel, scan[index])
    #print(cylList)
    return cylList
'''
def ComputeCartesianKoordinates(cylinderList):
    result = []
    for cyl in cylinderList:
        x = 1
    return result
'''
def ComputeAngle():
    return 0

class DataManager(object):
    def __init__(self):
        self.all_Data = [] # Alle Daten
        self.reference_positions = []
        self.uss_Data = []
        self.pole_indices = []
        self.motor_positions = []
        self.filtered_positions = []
        self.landmarks = []
        self.detected_obstacles = []
        self.last_ticks = None
        self.test_data = []
        self.motor_pose_data = []
        self.sensor_pose_data = []
        self.obstacle_points = []
        self.scanDataFromFile = []
        self.scanData = []  #Sensordaten in Grundform pro zeile 2*180/5 float Werte in mm
        self.scanDataPoints = [] #Datenpunkte zu jedem Messwert einer Messung im globalen koordinatensystem (x,y)
        self.cylindersFromScanData = []
        self.cylindersInGlobalCoordinates = []
        self.derivatives = []
    def Split_Data(self, newData):
        self.all_Data.append(newData)
        # Sensordaten extrahieren
        ussData = ((newData[11]),(newData[0]),(newData[1]),(newData[2]),(newData[3]),(newData[4]),(newData[5]),(newData[6]),newData[7], newData[8])
        self.uss_Data.append(ussData)
        # Motordaten extrahieren
        motorData = ((newData[11]),(newData[9]),(newData[10]))
        self.motor_positions += {motorData}
        #self.motor_positions.
    def Read(self, filename): # zum Einlesen von Testdaten
        file = open(filename)
        for line in file:
            self.Split_Data(line)
            
    def ReadStep5(self, filename): # zum Einlesen von Testdaten
        file = open(filename)
        for line in file:
            self.SplitDataStep5(line)
    def ReadScanDataFile(self):
        f = open("scanData.txt")
        for l in f:
            sp = l.split(sep = ";")
            i = 0
            scanDataTuples = []
            for value in sp:
                # buffer = float(value)
                scanDataTuples += {(i*rd.angleCorrection,(float(value)))}  
                #scanDataTuples += {(i,float(value))}
                i += rd.measurementStep
            self.scanData.append(scanDataTuples)
        f.close()
    def SplitDataStep5(self, newData):
        self.all_Data.append(newData)
        # Sensordaten extrahieren
        temp_ussData = []
        temp_motorData = []
        i = 0
        j = 0
        for index,value in enumerate(newData):
            #print("scanData Tupel: ")
            #print(index, value)
            if index > 0 and index < 37: #vorderer Sensor
                #Richtige Winkel zuordnen
                temp_ussData += {(290+i*rd.angleCorrection,(float(value)))}
                #temp_ussData += {(index,(10*float(value)))}
                i += rd.measurementStep
                #print({(290+i*rd.angleCorrection,(float(value)))})
            if index >36 and index < 74: #hinterer Sensor
                #Richtige Winkel zuordnen
                temp_ussData += {(110+j*rd.angleCorrection,(float(value)))}
                #temp_ussData += {(index,(10*float(value)))}
                j += rd.measurementStep
                #print({(110+j*rd.angleCorrection,(float(value)))})
        self.scanData.append(temp_ussData)
        #self.scanData.append(scanDataTuples)
        self.uss_Data.append(temp_ussData)
        # Motordaten extrahieren
        
        mdRight = abs(float(newData[74]))
        mdLeft = abs(float(newData[75]))
        mdTime = float(newData[76])
        
        if (mdRight == 0 and mdLeft != 0):
            mdRight = mdLeft
        elif (mdLeft == 0 and mdRight != 0):
            mdLeft = mdRight
        temp_motorData = ((int(newData[0])),(mdLeft),(mdRight), (mdTime))
        #temp_motorData = ((mdLeft),(mdRight),(mdTime))
       
        self.motor_positions += {temp_motorData}
        #print("Datensatz " , newData[0])
        #print ("ScanData: ", temp_ussData)
        #print ("MotorData: ", temp_motorData)

    def CreateTestData(self): # Testdaten erzeugen(einfachster Fall, kurzer Flur rechteckig mit Datenschwankungen)
        counter = 1
        start0 = 400
        start45 = (30/math.cos(45))*(1+((-1)^np.random.randint(1,3))*np.random.random()*0.1)
        start90 = 30
        start135 = (30/math.cos(45))*(1+((-1)^np.random.randint(1,3))*np.random.random()*0.1)
        start180 = 5
        start225 = (30/math.cos(45))*(1+((-1)^np.random.randint(1,3))*np.random.random()*0.1)
        start270 = 60
        start315 = (30/math.cos(45))*(1+((-1)^np.random.randint(1,3))*np.random.random()*0.1)
        start360 = start0*(1+((-1)^np.random.randint(1,3))*np.random.random()*0.1)
        startmposleft = 0
        startmposright = 0
        self.test_data.append([start0,start45,start90, start135, start180, start225, start270, start315, start360, startmposleft, startmposright, counter])
        
        for line in self.test_data:
            #print(line)
            self.Split_Data(line)
        
    def CreateDerivatives(self):
        for line in self.scanData:
            self.derivatives.append(CalcDerivative(line, rd.minValidSensorData))
    def CreateCylinderData(self):
        for index,line in enumerate(self.scanData):
            self.cylindersFromScanData.append(FindCylinders(line, self.derivatives[index], 50, rd.minValidSensorData))
        for line in self.cylindersFromScanData:
            self.CalcCylinderInGlobalCS()
    def CalcCylinderInGlobalCS(self): #Berechne Cylinderlage ausgehend von zugehöriger Sensorpose
        cylinderLine = []
        for index, pose in enumerate(self.sensor_pose_data):
            for cylinder in self.cylindersFromScanData[index]:
                #print("Cylinder: ", cylinder)
                #print(cylinder)
                globalPoint = pc.CalcPoint(pose, cylinder)
                #print("Cylinder in global CS: " , globalPoint)
                cylinderLine += {globalPoint}
            self.cylindersInGlobalCoordinates.append(cylinderLine)
        #print(self.cylindersInGlobalCoordinates)
    def PlotData(self):
        mdataleft = []
        mdataright = []
        #print(self.motor_positions)
        for data in self.motor_positions:
            mdataleft += {(data[0],data[1])}
            mdataright += {(data[0],data[2])}
        #print(mdataleft)
        for tupel in mdataleft:
            plot([x[0] for x in mdataleft],[x[1] for x in mdataleft], 'bo')
        show()
    
    def PrintData(self):
        print("Testdaten: ")
        print(self.test_data)
        print("MotorPoseData:")
        print(self.motor_pose_data)
        print("SensorPoseData:")
        print(self.sensor_pose_data)
        print("ScanDataPoints:")
        print(self.scanDataPoints)
        print("Motor_Positions:")
        print(self.motor_positions)
        print("Zylinder: ")
        print(self.cylindersFromScanData)
        print("scanData")
        print(self.scanData)
        

    def CreatePoseData(self):
        #pose = (1850.0,1897.0,213.0/180 * pi)
        pose = (0.0,0.0,0.0/180 * pi)
        pose_s = pose
        for mdline in self.motor_positions:
            #print("MotorDaten : " , mdline)
            pose = pc.CalcPivotPose(pose, mdline)
            #print("Pose: ", pose)
            self.motor_pose_data.append(pose)
            pose_s = pc.CalcSensorPose(pose, mdline)
            self.sensor_pose_data.append(pose_s)
    def CreatePoseDataStep(self, counter):
        if counter == 0: 
            pose = (0.0,0.0,0.0/180 * pi)
        else:
            pose = self.motor_pose_data[counter-1]
        pose_s = pose
        pose = pc.CalcPivotPose(pose, self.motor_positions[counter])
        #print("Pose: ", pose)
        self.motor_pose_data.append(pose)
        pose_s = pc.CalcSensorPose(pose, self.motor_positions[counter])
        self.sensor_pose_data.append(pose_s)
        return 0
    def CalcObstaclePoints(self): # Rechnet die ultraschalldistanzen in das globale koordinatensystem um
        for distance in self.uss_data[counter]:
            print(distance)
            
    def CreateUssDataPoses(self):
        #print("START CreateUssDataPoses: ")
        #print(self.sensor_pose_data)
        for index, pose in enumerate(self.sensor_pose_data):
            scanLinePoints = []
            #print("USSDATAPOSES: " , index , pose)
            for tupel in self.scanData[index]:
                #TODO umsetzen der sensorwerte in punkte in globale karte. sensorposen als ausgangswert benutzen
                newPoint = pc.CalcPoint(pose, tupel)
                scanLinePoints += {newPoint}
            self.scanDataPoints.append(scanLinePoints)
            
    def CreateUssDataPosesStep(self, count):
        #print('START', count)
        pose = self.sensor_pose_data[count]
        scanLinePoints = []
        for tupel in self.scanData[count]:
            #TODO umsetzen der sensorwerte in punkte in globale karte. sensorposen als ausgangswert benutzen
            newPoint = pc.CalcPoint(pose, tupel)
            #print(pose, tupel, newPoint)
            scanLinePoints += {newPoint}
        self.scanDataPoints.append(scanLinePoints)
        #print('ENDE')
    '''
    def CreateSensorDistanceData(self):
        i = 0
        for pose in self.sensor_pose_data:
            print(self.uss_Data[i])
            buffer = self.uss_Data[i]
            y = 1
            convertedData = []
            for value in buffer:
                if y != 1:
                    #value = pc.CalcPoint(pose)
                    #print("Value", value)
                    convertedData += {pc.CalcPoint(pose,math.radians(self.angles[y-1]))}
                y += 1
            self.obstacle_points.append(convertedData)
            i += 1
        print(self.obstacle_points)
        '''
    def PlotMotorPoseData(self):
        for pose in self.motor_pose_data:
            plot([x[0] for x in self.motor_pose_data],[x[1] for x in self.motor_pose_data], 'bo')
    def PlotSensorPoseData(self):
        for pose in self.sensor_pose_data:
            plot([x[0] for x in self.sensor_pose_data],[x[1] for x in self.sensor_pose_data], 'ro')
    def PlotScanDataPoints(self):
        for line in self.scanDataPoints:
            for pair in line:
                plot([x[0] for x in line],[x[1] for x in line], 'go')
    def PlotScanData(self):
        PlotValues(self.scanData, 0, 'or')
    def PlotDerivative(self):
        PlotValues(self.derivatives,0, 'bo' )
        
    def PlotCylinders(self):
        for data in self.cylindersFromScanData:
            scatter([c[0] for c in data],[c[1] for c in data],c = 'g', s=200)
        show()
    def PlotCylInGlobalCoordinates(self):
        for cyl in self.cylindersInGlobalCoordinates:
            plot([x[0] for x in cyl],[x[1] for x in cyl], 'r^')
    def PlotScanDerCyl(self):
        self.PlotScanData()
        #show()
        self.PlotDerivative()
        #show()
        self.PlotCylinders()
        show()
                
    def PlotRobotDataPolar(self):
        plt.axes(projection = 'polar') 
        r = 3
        rads = np.arange(0, (2 * np.pi), 1)
        # plotting the circle
        for i in rads:
            plt.polar(i, r, 'g.')
            # display the Polar plot
        plt.show()
        
        for line in self.scanDataPoints:
            for pair in line:
                plot([x[0] for x in line],[x[1] for x in line], 'go')
                


        