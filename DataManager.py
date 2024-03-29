# -*- coding: utf-8 -*-
"""
Created on Mon May 30 14:33:59 2022

@author: Jan

Class for Robotdata

takes measurements and converts/calculates into needed informations
"""

from pylab import *
import numpy as np
import matplotlib.pyplot as plt
import poseCalculations as pc
from math import degrees, pi, radians
import math
import RobotData as rd

gJump = 30 # value for landmarkextraction: the higher gJump the less landmarks are extracted.


def DataSmoothing(data):
    '''
    checks for single outliers in ultrasonic measurment and replaces them with mean of previous and next value
    '''
    treshhold = 50
    smoothData = []
    for index,value in enumerate(data):
        
        if (index > 0) and (index < len(data)-1):
            if ( abs(value[1] - data[index-1][1]) > treshhold ) and ( abs(value[1] - data[index+1][1]) > treshhold):
                smoothData += ({(data[index][0],((data[index-1][1] + data[index+1][1])/2))})
            else:
                smoothData += ({(value[0],value[1])})  
        else:
            smoothData += ({(value[0],value[1])})    
    return smoothData
def EuclideanDistance(x1, y1, x2, y2):
    return ((x2-x1)**2 + (y2-y1)**2)**(1/2)

def CheckValidAngle(angle):
    if angle >= 110 and angle <= 110+35*5*140/180:
        return True
    elif angle >= 0 and angle <= 17*5*140/180:
        return True
    elif angle >= 290 and angle <=17*5*140/180:
        return True
    else:
        return False

def GetIndexFromAngle(data, angle):
    for tupel in data:
        if angle <= tupel[0]:
            return tupel[1]-1
    return 999

def PlotValues(data,index,style):
    x_val = [x[0] for x in data[index]]
    y_val = [x[1] for x in data[index]]
    plt.plot(x_val,y_val, style)
    

def CalcDerivative(dataV, dataH, minDist):
    '''
    Calculates the derivative from scanData for landmarkextraction
    
    04.08.2022: Correction to separate between scanareas (front and back)
    '''
    x_val_v = [float(x[0]) for x in dataV]
    y_val_v = [float(x[1]) for x in dataV]
    x_val_h = [float(x[0]) for x in dataH]
    y_val_h = [float(x[1]) for x in dataH]
    
    derivatives_v = []
    derivatives_v+={(x_val_v[0],0)}
    
    for i in range(1,len(x_val_v) -1):
        if (y_val_v[i+1] >= rd.minValidSensorData and y_val_v[i+1] >= rd.minValidSensorData):
            derivative = (y_val_v[i+1]-y_val_v[i-1])/(2+5*rd.angleCorrection)
            derivatives_v +=({(x_val_v[i],derivative)})
        else:
            derivatives_v +=({(x_val_v[i],0)})
    derivatives_v+=({(x_val_v[(len(dataV)-1)],0)})
    
    derivatives_h = []
    derivatives_h +={(x_val_h[0],0)}

    for i in range(1,len(x_val_h) -1):
        if (y_val_h[i+1] >= rd.minValidSensorData and y_val_h[i+1] >= rd.minValidSensorData):
            derivative = (y_val_h[i+1]-y_val_h[i-1])/(2+5*rd.angleCorrection) 
            derivatives_h+=({(x_val_h[i],derivative)})
        else:
            derivatives_h+=({(x_val_h[i],0)})
    derivatives_h+=({(x_val_h[(len(dataH)-1)],0)})
    
    return derivatives_v, derivatives_h

def FindObjects(scan, derivative, jump):
    objList = []
    onObj = False
    sumRay, sumDepth, rays = 0.0, 0.0, 0
    for index, tupel in enumerate(derivative):
        if onObj == False and tupel[1] < -jump: # Object found
            onObj = True
            rays = 1
            sumRay = tupel[0]
            distanceVal = scan[index]
            sumDepth = distanceVal[1]
        elif onObj == True and tupel[1] < -jump:
            # Second object found, reset all values
            rays = 1
            sumRay = tupel[0]
            distanceVal = scan[index]
            sumDepth = distanceVal[1]
        elif onObj == True and tupel[1] > jump: #end of object
            onObj = False
            cylCenter = sumRay/rays 
            cylDistance = sumDepth/rays 
            objList += {(cylCenter,cylDistance)}
        elif onObj == True:
            rays += 1
            sumRay += tupel[0]
            distanceVal += scan[index]
            sumDepth += distanceVal[1]
    return objList

class DataManager(object):
    def __init__(self):
        self.all_Data = [] # Alle Daten
        self.uss_Data = []
        self.motor_positions = [] 
        self.test_data = []
        self.motor_pose_data = []
        self.sensor_pose_data = []
        self.obstacle_points = []
        self.scanDataFromFile = []
        self.scanData = []  #Sensordaten in Grundform pro zeile 2*180/5 float Werte in mm
        self.scanData_v = [] #Scandaten vorderer Sensor (wird später benötigt für Objektextrahierung)
        self.scanData_h = [] #Scandaten hinterer Sensor (wird später benötigt für Objektextrahierung)
        self.scanDataPoints = [] #Datenpunkte zu jedem Messwert einer Messung im globalen koordinatensystem (x,y)
        self.scanDataPointsNo400 = [] #Datenpunkte ohne 400cm
        self.allPointNo400=[]
        self.landmarksFromScanData = []
        self.allLandmarks = [] # List of Landmarks with ID, xCoord, yCoord
        self.landmarkPairs = [] # Contains related Landmarks. If a landmark is within a range of another landmark, we get a tupel with (current Landmark ID, global Landmark ID)
        self.landmarksInGC = []
        self.validLandmarks = []
        self.derivatives = []
        self.derivatives_v = []
        self.derivatives_h = []
        self.indexOfAngle = []


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
                scanDataTuples += {(i*rd.angleCorrection,(float(value)))}  
                i += rd.measurementStep
            self.scanData.append(scanDataTuples)
        f.close()

    def SplitDataStep5(self, newData):
        '''
        takes the measurements for each step and converts the data into various lists/arrays
        '''
        
        self.all_Data.append(newData)
        # Sensordaten extrahieren
        temp_ussData = []
        temp_ussData_v = []
        temp_ussData_h = []
        temp_motorData = []
        i = 0
        j = 0
        for index,value in enumerate(newData):
            if index > 0 and index < 37: #vorderer Sensor
                #Richtige Winkel zuordnen
                winkel = 290+i*rd.angleCorrection
                
                '''# Dasm musste wieder raus - später rpüfen ob das Probleme bei ICP-Berechnungen (assocoation- modus) erzeugt
                if winkel >360:
                    winkel -= 360
                if winkel == 360:
                    winkel = 0
                '''
                tupel = {(winkel,(float(value)*10))}
                #tupel = {(winkel,(float(value)))}
                temp_ussData += tupel
                temp_ussData_v += tupel
                i += rd.measurementStep
            if index >36 and index < 73: #hinterer Sensor
                #Richtige Winkel zuordnen
                tupel = {(110+j*rd.angleCorrection,(float(value)*10))}
                #tupel = {(110+j*rd.angleCorrection,(float(value)))}
                temp_ussData += tupel
                temp_ussData_h += tupel
                j += rd.measurementStep
        temp_ussData = DataSmoothing(temp_ussData)
        temp_ussData_v = DataSmoothing(temp_ussData_v)
        temp_ussData_h = DataSmoothing(temp_ussData_h)
        self.scanData.append(temp_ussData)
        self.scanData_v.append(temp_ussData_v)
        self.scanData_h.append(temp_ussData_h)
        self.uss_Data.append(temp_ussData)
        
        if len(self.scanData) == 1:
            sortedList = [(value[0],index)for index,value in enumerate(temp_ussData)]
            sortedList.sort()
            self.indexOfAngle.append(sortedList)
        temp_motorData = self.CheckCorrectMotorData(int(newData[73]),float(newData[74]),float(newData[75]),float(newData[76]))
        self.motor_positions += {temp_motorData}
        
    def CreateRobotData(self, index):
        self.CreatePoseDataStep(index)
        self.CreateUssDataPosesStep(index)
        self.CreateDerivativesStep(index)
        self.CreateLandmarksDataStep(index)
    def UpdateDataFromSLAM(self,newPoses):
        '''
        takes newPoses (resulting new poses from SLAM-Algorithm) and creates new Instance of DataManager, 
        where poses and all other Data is updates with respect to new Pposes
        '''
        updateRobot = DataManager()
        newX = newPoses[0]
        newY = newPoses[1]
        newTheta = newPoses[2]
        
        for index,line in enumerate(self.all_Data):
            updateRobot.SplitDataStep5(line)
            pose = (newX[index],newY[index],newTheta[index])
            updateRobot.motor_pose_data.append(pose)
            pose_s = rd.CalcSensorPose(pose, updateRobot.motor_positions[index])
            updateRobot.sensor_pose_data.append(pose_s)        
            updateRobot.CreateUssDataPosesStep(index)
            updateRobot.CreateDerivativesStep(index)
            updateRobot.CreateLandmarksDataStep(index)
        return updateRobot
    
    def CheckCorrectMotorData(self, angle, rotMRight, rotMLeft, duration):
        '''
        check and correct invalid motordata (revs != 0 must be corrected)
        '''
        if (rotMRight == 0  and rotMLeft == 0):
            # calculate motion over duration
            if (angle == 0):
                rotMRight = duration/10000 *360
                rotMLeft = duration/10000 * 360 *(-1)
            elif (angle == 999):
                rotMRight = duration/10000 *360  *(-1)
                rotMLeft = duration/10000 * 360
            else:
                if angle > 0:#turning left
                    rotMRight = rotMLeft = duration/10 *360
                else: #turning right
                    rotMRight = rotMLeft = duration/10 *360 * (-1)
        elif (rotMRight == 0  or rotMLeft == 0):
            if (angle == 0 or angle == 999): # foreward or backward motion
                if rotMRight == 0:
                    rotMRight = rotMLeft * (-1)
                elif rotMLeft == 0:
                    rotMLeft = rotMRight * (-1)
            elif angle > 0:
                if rotMRight == 0:
                    rotMRight = rotMLeft 
                elif rotMLeft == 0:
                    rotMLeft = rotMRight
            elif angle < 0:
                if rotMRight == 0:
                    rotMRight = rotMLeft * (-1) 
                elif rotMLeft == 0:
                    rotMLeft = rotMRight * (-1)
        #return((angle, rotMRight, rotMLeft*0.9965, duration))
        return((angle, rotMRight, rotMLeft, duration))

    def CreateDerivativesStep(self, index):
        '''
        Calculates the derivative for each measurement per sensor
        
        '''
        derivatives_v, derivatives_h = CalcDerivative(self.scanData_v[index],self.scanData_h[index], rd.minValidSensorData)
        self.derivatives_v.append(derivatives_v)
        self.derivatives_h.append(derivatives_h)
        derivatives = []
        for value in self.derivatives_v[index]:
            derivatives.append(value)
        for value in self.derivatives_h[index]:
            derivatives.append(value)
        self.derivatives.append(derivatives)
    
    def CreateDerivatives(self):
        for line in self.scanData:
            self.derivatives.append(CalcDerivative(line, rd.minValidSensorData))
    
    def CreateLandmarksDataStep(self, index):
        '''
        extraction of landmarks
        
        changes:
            06.08.2022 - Anpassungen an Unterscheidung zwischen vorderem und hinterem Sensor um fehlerhafte Landmarken zu vermeiden
            13.08.2022 - hinzufügen von checkroutine für landmarkenzuweisung
        '''
        lineV = self.scanData_v[index]
        lineH = self.scanData_h[index]
        objectList = []
        objectList.append(FindObjects(lineV, self.derivatives_v[index], gJump))
        objectList.append(FindObjects(lineH, self.derivatives_h[index], gJump))
        allObjects = []
        for line in objectList:
            for valuePair in line:
                allObjects.append(valuePair)
        self.landmarksFromScanData.append(allObjects)
        self.CalcLandmarksInGlobalCoordinates(index)
        self.landmarkPairs.append(self.CheckForNearbyLandmarks(index))
    
    def GetValidLandmarks(self, minObservations):
        '''
        Create and return a list of all landmarks, wich were seen at least minObservations-times. These should be "valid"
        '''
        validLandmarks = []
        for landmark in self.allLandmarks:
            counter = 0
            for line in self.landmarkPairs:
                for tupel in line: 
                    if tupel[1] == landmark[0]:
                        counter +=1
            if counter >= minObservations:
                validLandmarks.append((landmark))
            self.validLandmarks.append((validLandmarks))
        return validLandmarks
        
    def CheckForNearbyLandmarks(self, index):
        '''
        Check which known landmarks were found in latest scan and create dataset
        '''
        maxDistanceToNext = rd.landmarkMargin 
        foundPairs = []
        for scanLandmark in self.landmarksInGC[index]:
            for gLandmark in self.allLandmarks: 
                if  EuclideanDistance(scanLandmark[1],scanLandmark[2],gLandmark[1],gLandmark[2]) <= maxDistanceToNext:
                    foundPairs.append((scanLandmark[0],gLandmark[0]))
        return foundPairs
        
    def CreateLandmarkData(self):
        for index,line in enumerate(self.scanData):
            self.landmarksFromScanData.append(FindObjects(line, self.derivatives[index], gJump))
        for line in self.landmarksFromScanData:
            self.CalcLandmarksInGlobal()
    
    def CalcLandmarksInGlobalCoordinates(self, index):
        '''
        Calculates the Landmarkposition in global coordinates based on the sensor pose
        
        changes: 
            13.08.2022: Hinzugefügt: Datensatz pro Scan mit Zuweisungen welche Landmarke gesehen wurde
        '''
        landmarkLine = []
        pose = self.sensor_pose_data[index]
        for count, landmark in enumerate(self.landmarksFromScanData[index]):
            globalPoint = rd.CalcPoint(pose, landmark)
            landmarkLine += {(count+1, globalPoint[0], globalPoint[1])}
            lmID = len(self.allLandmarks)+1
            self.allLandmarks.append((lmID, globalPoint[0], globalPoint[1]))
        self.landmarksInGC.append(landmarkLine)

    def CalcLandmarksInGlobal(self): #Berechne Cylinderlage ausgehend von zugehöriger Sensorpose
        landmarkLine = []
        for index, pose in enumerate(self.sensor_pose_data):
            for landmark in self.landmarksFromScanData[index]:
                globalPoint = rd.CalcPoint(pose, landmark)
                landmarkLine += {globalPoint}
            self.landmarksInGC.append(landmarkLine)
    
    def PlotData(self):
        mdataleft = []
        mdataright = []
        for data in self.motor_positions:
            mdataleft += {(data[0],data[1])}
            mdataright += {(data[0],data[2])}
        for tupel in mdataleft:
            plt.plot([x[0] for x in mdataleft],[x[1] for x in mdataleft], 'bo')
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
        pose = (0.0,0.0,0.0/180 * pi)
        pose_s = pose
        for mdline in self.motor_positions:
            pose = rd.CalcPivotPose(pose, mdline)
            self.motor_pose_data.append(pose)
            pose_s = rd.CalcSensorPose(pose, mdline)
            self.sensor_pose_data.append(pose_s)
            
    def CreatePoseDataStep(self, counter):
        if counter == 0: 
            pose = (0.0,0.0,0.0/180 * pi)
        else:
            pose = self.motor_pose_data[counter-1]
        pose_s = pose
        pose = rd.CalcPivotPose(pose, self.motor_positions[counter])
        self.motor_pose_data.append(pose)
        pose_s = rd.CalcSensorPose(pose, self.motor_positions[counter])
        self.sensor_pose_data.append(pose_s)
        return 0
            
    def CreateUssDataPoses(self):
        for index, pose in enumerate(self.sensor_pose_data):
            scanLinePoints = []
            for tupel in self.scanData[index]:
                newPoint = rd.CalcPoint(pose, tupel)
                scanLinePoints += {newPoint}
            self.scanDataPoints.append(scanLinePoints)
            
    def CreateUssDataPosesStep(self, count):
        pose = self.sensor_pose_data[count]
        scanLinePoints = []
        scanLinePointsNo400 = []
        for tupel in self.scanData[count]:
            newPoint = rd.CalcPoint(pose, tupel)
            scanLinePoints += {newPoint}
            if tupel[1] < 4000:
                scanLinePointsNo400 += {newPoint}
        self.scanDataPoints.append(scanLinePoints)
        self.scanDataPointsNo400.append(scanLinePointsNo400)


    def PlotMotorPoseData(self):
        for pose in self.motor_pose_data:
            plt.plot([x[0] for x in self.motor_pose_data],[x[1] for x in self.motor_pose_data], 'b-',)
            
    def PlotSensorPoseData(self):
        for pose in self.sensor_pose_data:
            plt.plot([x[0] for x in self.sensor_pose_data],[x[1] for x in self.sensor_pose_data], 'r-')
            
    def PlotScanDataPoints(self, mode):
        if mode == 'all':
            for line in self.scanDataPoints:
               plt.plot([x[0] for x in line],[x[1] for x in line], '.k')
        elif mode =='no400':
            for line in self.scanDataPointsNo400:
                plt.plot([x[0] for x in line],[x[1] for x in line], '.k')
                    
    def PlotScanData(self, index):
        PlotValues(self.scanData, index, '.r')
    def PlotDerivative(self, index):
        PlotValues(self.derivatives,index, '.b' )
        
    def PlotLandmarks(self, index):
        plt.plot([x[0] for x in self.landmarksFromScanData[index]],[x[1] for x in self.landmarksFromScanData[index]], 'go') 
        
    def PlotLmInGlobalCoordinates(self):
        for lm in self.landmarksInGC:
            plt.plot([x[1] for x in lm],[x[2] for x in lm], 'r^')
            
    def PlotValidLmInGC(self):
        for lm in self.validLandmarks:
            plt.plot([x[1] for x in lm],[x[2] for x in lm], 'bo')
            
    def PlotAllLandmarks(self):
        for cyl in self.allLandmarks:
            plt.plot(cyl[0],cyl[1], 'r^')
            
    def PlotScanDerLM(self, index):
        self.PlotScanData(index)
        self.PlotDerivative(index)
        self.PlotLandmarks(index)
                
    def FilterSingleObservationPoints(self, index):
        '''
        Idee:   Berechnung des Winkels unter dem ein Messpunkt in der Folgemessung 
                betrachtet werden müsste. danach Zuweisung dieser Punkte zueinander. 
                Zur Verbesserung der übergebenen Landmarken und der ICP- Ergebnisse
        '''
        px = np.array([])
        py = np.array([])
        cx = np.array([])
        cy = np.array([])
        
        pPose = self.sensor_pose_data[index-1]
        cPose = self.sensor_pose_data[index]
        pUssData = self.uss_Data[index-1]
        cUssData = self.uss_Data[index]
        pUssPoints = self.scanDataPoints[index-1]
        cUssPoints = self.scanDataPoints[index-1]
        previous_points = np.array([])
        current_points = np.array([])
        for i, tupel in enumerate(pUssData): # (winkel, gemessene distanz)
            if tupel[1] != 4000: # Grenzbereich wird nicht berücksichtigt
                #Wir berechenn die Beobachtungswinkeldifferenz zwischen den beiden Messungen
                pHeading = pPose[2]
                cHeading = cPose[2]
                x1, y1 = pPose[0] , pPose[1]
                x3, y3 = pUssPoints[i][0], pUssPoints[i][1]
                x2, y2 = cPose[0], cPose[1]
                
                pAlpha = math.asin((y2-y1)/(((x2-x1)**2+(y2-y1)**2)**(1/2)))
                cAlpha = cPose[2] + math.asin((y3-y2)/(((x3-x2)**2+(y3-y2)**2)**(1/2)))
                
                angleInNext = cHeading + radians(tupel[0])-cAlpha
                
                '''
                plt.title(tupel, fontdict=None, loc='center') 
                plt.plot(x1,y1, 'r^' )
                plt.plot(x2,y2, 'go')
                plt.plot(x3,y3, 'bo')
                show()
                '''

                if CheckValidAngle(degrees(angleInNext)): 
                    # Es gibt einen Wert in der nächsten Messung, der auch in der vorherigen Messung gemessen wurde
                    nextIndex = GetIndexFromAngle(self.indexOfAngle[0], degrees(angleInNext))
                    nextScanData =  cUssData[nextIndex]
                    nextPoint = cUssPoints[nextIndex]
                    
                    # fast fertig: nur wenn der korrelierende Messwert kleiner 400 ist, haben wir einen validen Punkt für ICP
                    if nextScanData[1] < 4000:
                        px = np.append(px, x3)
                        py = np.append(py, y3)

                        cx = np.append(cx, nextPoint[0])
                        cy = np.append(cy,nextPoint[1])
                        
                        previous_points = np.vstack((px, py))
                        current_points = np.vstack((cx, cy))
        return previous_points, current_points
    

    