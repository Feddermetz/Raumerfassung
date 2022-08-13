# -*- coding: utf-8 -*-
"""
Created on Mon May 30 14:33:59 2022

@author: Jan

Datenmodellierung
"""

from pylab import *
import numpy as np
#import matplotlib as mpl
import matplotlib.pyplot as plt
import scipy.interpolate as si
import poseCalculations as pc
import math
import RobotData as rd


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
    #print(sortedList)
    for tupel in data:
        if angle <= tupel[0]:
            #print('Winkel: ', angle, ' Index: ', tupel[1]-1)
            return tupel[1]-1 #index
    return 999

def PlotValues(data,index,style):
    x_val = [x[0] for x in data[index]]
    y_val = [x[1] for x in data[index]]
    plt.plot(x_val,y_val, style)
    

def CalcDerivative(dataV, dataH, minDist):
    '''
    04.08.2022: Anpassungen, da zwischen vorderem und hinterem Sensor unterschieden werden muss. 
    Hier wird sonst immer ein Objekt gefunden
    '''
    
    x_val_v = [float(x[0]) for x in dataV]
    y_val_v = [float(x[1]) for x in dataV]
    x_val_h = [float(x[0]) for x in dataH]
    y_val_h = [float(x[1]) for x in dataH]
    
    derivatives_v = []
    derivatives_v+={(x_val_v[0],0)}
    
    for i in range(1,len(x_val_v) -1):
        if (y_val_v[i+1] >= rd.minValidSensorData and y_val_v[i+1] >= rd.minValidSensorData): # TODO: was wird gemacht wenn eine ungültige Distanz gemessen wird?
            derivative = (y_val_v[i+1]-y_val_v[i-1])/(2+5*rd.angleCorrection) # Berechnung über Differenzenquotien
            derivatives_v +=({(x_val_v[i],derivative)})
        else:
            derivatives_v +=({(x_val_v[i],0)})
    derivatives_v+=({(x_val_v[(len(dataV)-1)],0)})
    
    derivatives_h = []
    derivatives_h +={(x_val_h[0],0)}

    for i in range(1,len(x_val_h) -1):
        if (y_val_h[i+1] >= rd.minValidSensorData and y_val_h[i+1] >= rd.minValidSensorData): # TODO: was wird gemacht wenn eine ungültige Distanz gemessen wird?
            derivative = (y_val_h[i+1]-y_val_h[i-1])/(2+5*rd.angleCorrection) # Berechnung über Differenzenquotien
            derivatives_h+=({(x_val_h[i],derivative)})
        else:
            derivatives_h+=({(x_val_h[i],0)})
    derivatives_h+=({(x_val_h[(len(dataH)-1)],0)})
    #derivatives_h+={(x_val_h[0],0)}
    
    return derivatives_v, derivatives_h
    #return 0

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
        self.scanData_v = [] #Scandaten vorderer Sensor (wird später benötigt für Objektextrahierung)
        self.scanData_h = [] #Scandaten hinterer Sensor (wird später benötigt für Objektextrahierung)
        self.scanDataPoints = [] #Datenpunkte zu jedem Messwert einer Messung im globalen koordinatensystem (x,y)
        self.scanDataPointsNo400 = [] #Datenpunkte ohne 400cm
        self.landmarksFromScanData = []
        #self.cylindersFromScanData = []
        #self.cylindersInGlobalCoordinates = []
        self.allLandmarks = [] # List oif Landmarks with ID, xCoord, yCoord
        self.landmarkPairs = [] # Welche
        self.landmarksInGC = []
        self.derivatives = []
        self.derivatives_v = []
        self.derivatives_h = []
        self.indexOfAngle = []
    def Split_Data(self, newData): # nicht angepasst an neue übergabestruktur
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
                temp_ussData += tupel
                temp_ussData_v += tupel
                #temp_ussData += {(index,(10*float(value)))}
                i += rd.measurementStep
                #print({(290+i*rd.angleCorrection,(float(value)))})
            if index >36 and index < 73: #hinterer Sensor
                #Richtige Winkel zuordnen
                tupel = {(110+j*rd.angleCorrection,(float(value)*10))}
                temp_ussData += tupel
                temp_ussData_h += tupel
                #temp_ussData += {(index,(10*float(value)))}
                j += rd.measurementStep
                #print({(110+j*rd.angleCorrection,(float(value)))})
        self.scanData.append(temp_ussData)
        self.scanData_v.append(temp_ussData_v)
        self.scanData_h.append(temp_ussData_h)
        #self.scanData.append(scanDataTuples)
        self.uss_Data.append(temp_ussData)
        
        if len(self.scanData) == 1:
            sortedList = [(value[0],index)for index,value in enumerate(temp_ussData)]
            sortedList.sort()
            self.indexOfAngle.append(sortedList)
            
        # Motordaten extrahieren
        temp_motorData = self.CheckCorrectMotorData(int(newData[73]),float(newData[74]),float(newData[75]),float(newData[76]))
        self.motor_positions += {temp_motorData}

    def CheckCorrectMotorData(self, angle, rotMRight, rotMLeft, duration):
        # check and correct invalid motordata (revs != 0 must be corrected)
        # correction if value 0 occurs
        if (rotMRight == 0  and rotMLeft == 0):
            #print("Correction needed: both values 0")
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
            #print("Correction needed: one value 0")
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
        return((angle, rotMRight, rotMLeft, duration))

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
            self.Split_Data(line)
    
    def CreateDerivativesStep(self, index):
        #line = self.scanData[index]
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
    

    
    def CreateCylinderDataStep(self, index):
        '''
        06.08.2022 - Anpassungen an Unterscheidung zwischen vorderem und hinterem Sensor um fehlerhafte Landmarken zu vermeiden
        13.08.2022 - hinzufügen von checkroutine für landmarkenzuweisung
        '''
        lineV = self.scanData_v[index]
        lineH = self.scanData_h[index]
        objectList = []
        objectList.append(FindCylinders(lineV, self.derivatives_v[index], 50, rd.minValidSensorData))
        objectList.append(FindCylinders(lineH, self.derivatives_h[index], 50, rd.minValidSensorData))
        allObjects = []
        for line in objectList:
            for valuePair in line:
                allObjects.append(valuePair)
        self.landmarksFromScanData.append(allObjects)
        self.CalcCylinderInGlobalStep(index)
        self.landmarkPairs.append(self.CheckForNearbyLandmarks(index))
        
    def CheckForNearbyLandmarks(self, index):
        '''
        Check which known landmarks were found in latest scan and create dataset
        '''
        maxDistanceToNext = 700 
        foundPairs = []
        for scanLandmark in self.landmarksInGC[index]:
            for gLandmark in self.allLandmarks: 
                if  EuclideanDistance(scanLandmark[1],scanLandmark[2],gLandmark[1],gLandmark[2]) <= maxDistanceToNext:
                    foundPairs.append((scanLandmark[0],gLandmark[0]))
        return foundPairs
        
    def CreateCylinderData(self):
        for index,line in enumerate(self.scanData):
            self.landmarksFromScanData.append(FindCylinders(line, self.derivatives[index], 50, rd.minValidSensorData))
        for line in self.landmarksFromScanData:
            self.CalcCylinderInGlobal()
    
    def CalcCylinderInGlobalStep(self, index): #Berechne Cylinderlage ausgehend von zugehöriger Sensorpose
        '''
        13.08.2022: Hinzugefügt: Datensatz pro Scan mit Zuweisungen welche Landmarke gesehen wurde
        '''
        cylinderLine = []
        pose = self.sensor_pose_data[index]
        for count, cylinder in enumerate(self.landmarksFromScanData[index]):
            #print("Cylinder: ", cylinder)
            #print(cylinder)
            globalPoint = pc.CalcPoint(pose, cylinder)
            #print("Cylinder in global CS: " , globalPoint)
            cylinderLine += {(count+1, globalPoint[0], globalPoint[1])}
            lmID = len(self.allLandmarks)+1
            self.allLandmarks.append((lmID, globalPoint[0], globalPoint[1]))
        self.landmarksInGC.append(cylinderLine)

    
    def CalcCylinderInGlobal(self): #Berechne Cylinderlage ausgehend von zugehöriger Sensorpose
        cylinderLine = []
        for index, pose in enumerate(self.sensor_pose_data):
            for cylinder in self.landmarksFromScanData[index]:
                #print("Cylinder: ", cylinder)
                #print(cylinder)
                globalPoint = pc.CalcPoint(pose, cylinder)
                #print("Cylinder in global CS: " , globalPoint)
                cylinderLine += {globalPoint}
            self.landmarksInGC.append(cylinderLine)
    
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
        scanLinePointsNo400 = []
        for tupel in self.scanData[count]:
            #TODO umsetzen der sensorwerte in punkte in globale karte. sensorposen als ausgangswert benutzen
            
            newPoint = pc.CalcPoint(pose, tupel)
            scanLinePoints += {newPoint}
            if tupel[1] < 4000:
                scanLinePointsNo400 += {newPoint}
        self.scanDataPoints.append(scanLinePoints)
        self.scanDataPointsNo400.append(scanLinePointsNo400)
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
            plot([x[0] for x in self.motor_pose_data],[x[1] for x in self.motor_pose_data], 'bo',)
    def PlotSensorPoseData(self):
        for pose in self.sensor_pose_data:
            plot([x[0] for x in self.sensor_pose_data],[x[1] for x in self.sensor_pose_data], 'ro')
    def PlotScanDataPoints(self, mode):
        if mode == 'all':
            for line in self.scanDataPoints:
                for pair in line:
                    plot([x[0] for x in line],[x[1] for x in line], 'go')
        elif mode =='no400':
            for line in self.scanDataPointsNo400:
                for pair in line:
                    plot([x[0] for x in line],[x[1] for x in line], 'go')
                    
    def PlotScanData(self, index):
        PlotValues(self.scanData, index, 'or')
    def PlotDerivative(self, index):
        PlotValues(self.derivatives,index, 'bo' )
        
    def PlotCylinders(self, index):
        #for data in self.cylindersFromScanData:
         #   scatter([c[0] for c in data],[c[1] for c in data],c = 'g', s=200)
        PlotValues(self.cylindersFromScanData,index, 'go')
    def PlotCylInGlobalCoordinates(self):
        for cyl in self.landmarksInGC:
            plot([x[1] for x in cyl],[x[2] for x in cyl], 'r^')
    def PlotAllCylinders(self):
        for cyl in self.allCylinders:
            plot(cyl[0],cyl[1], 'r^')
    def PlotScanDerCyl(self, index):
        self.PlotScanData(index)
        #show()
        self.PlotDerivative(index)
        #show()
        self.PlotCylinders(index)
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
                
    def PrepareICP(self, index):
        '''
        Idee:   Berechnung des Winkels unter dem ein Messpunkt in der Folgemessung 
                betrachtet werden müsste.
                Danach Zuweisung dieser Punkte zueinander, dann ICP oder so, 
                mal schauen wie sich das dann in einen Vektor für die constraints umrechnen lässt
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
        #print('ppose: ', pPose)
        #print(pUssPoints)
        data = []
        counter = 0
        for i, tupel in enumerate(pUssData): # (winkel, gemessene distanz)
            if tupel[1] != 4000: # Grenzbereich wird nicht berücksichtigt (für später: vielleicht doch, falls BEIDE werte 400)
                #Wir berechenn die Beobachtungswinkeldifferenz zwischen den beiden Messungen
                pHeading = pPose[2]
                cHeading = cPose[2]
                x1, y1 = pPose[0] , pPose[1]
                x3, y3 = pUssPoints[i][0], pUssPoints[i][1]
                x2, y2 = cPose[0], cPose[1]
                #print('Tupel: ', tupel)
                #print('P1: (',x1,y1,')', 'P2: (',x2,y2,')', 'P3: (',x3,y3,')', )
                
                pAlpha = math.asin((y2-y1)/(((x2-x1)**2+(y2-y1)**2)**(1/2)))
                cAlpha = cPose[2] + math.asin((y3-y2)/(((x3-x2)**2+(y3-y2)**2)**(1/2)))
                
                #print('Alpha1:',degrees(pAlpha),'Alpha2:',degrees(cAlpha), 'Winkel zwischen Geraden: ', degrees(pAlpha-cAlpha) )
                angleInNext = cHeading + radians(tupel[0])-cAlpha
                
                '''
                plt.title(tupel, fontdict=None, loc='center') 
                plt.plot(x1,y1, 'r^' )
                plt.plot(x2,y2, 'go')
                plt.plot(x3,y3, 'bo')
                show()
                '''
                if degrees(angleInNext) > 360:
                    angleInNext -= radians(360)
                #print('Wert in nächstem Datensatz bei: ' , degrees(angleInNext))
                
                if CheckValidAngle(degrees(angleInNext)): 
                    # Es gibt einen Wert in der nächsten Messung, der auch in der vorherigen Messung gemessen wurde
                    nextIndex = GetIndexFromAngle(self.indexOfAngle[0], degrees(angleInNext))
                    #print('nextIndex: ' , nextIndex)
                    nextScanData =  cUssData[nextIndex]
                    nextPoint = cUssPoints[nextIndex]
                    #print('nextUp: ',nextScanData, nextPoint)
                    
                    # fast fertig: nur wenn der korrelierende Messwert kleiner 400 ist, haben wir einen validen Punkt für ICP
                    if nextScanData[1] < 4000:
                        #print('Supi, wir haben einen Messwert gefunden!')
                        # previous point
                        px = np.append(px, x3)
                        py = np.append(py, y3)

                        # current points
                        cx = np.append(cx, nextPoint[0])
                        cy = np.append(cy,nextPoint[1])
                        
                        previous_points = np.vstack((px, py))
                        current_points = np.vstack((cx, cy))
                        #print(current_points)
        return previous_points, current_points