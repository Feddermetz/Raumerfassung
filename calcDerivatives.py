# -*- coding: utf-8 -*-
"""
Created on Mon Jun  6 11:12:38 2022

@author: Jan
"""
import RobotData as rd
from pylab import *

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
    
    for i in range(1,len(x_val) -1):
        if (y_val_v[i+1] >= rd.minValidSensorData and y_val_v[i+1] >= rd.minValidSensorData): # TODO: was wird gemacht wenn eine ungültige Distanz gemessen wird?
            derivative = (y_val_v[i+1]-y_val_v[i-1])/(2+5*rd.angleCorrection) # Berechnung über Differenzenquotien
            derivatives_v +=({(x_val[i],derivative)})
        else:
            derivatives_v +=({(x_val[i],0)})
    derivatives_v+=({(x_val[(len(scanLine)-1)],0)})
    
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
    
    #return derivatives_v, derivatives_h
    return 0

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
            #print(2)
        elif onCyl == True and tupel[1] > jump: #Zylinderende
            onCyl = False
            cylCenter = sumRay/rays 
            cylDistance = sumDepth/rays 
            cylList += {(cylCenter,cylDistance)}
            #print(3)
        elif onCyl == True:
            rays += 1
            sumRay += tupel[0]
            distanceVal += scan[index]
            sumDepth += distanceVal[1]
            #print(4)
        else:
            z=0
    return cylList
def ComputeCartesianKoordinates(cylinderList):
    result = []
    for cyl in cylinderList:
        x = 1
    return result

def ComputeAngle():
    return 0

'''
if __name__ == '__main__':
    scanData = []
    derivatives = []
    f = open("scanData.txt")
    for l in f:
        sp = l.split(sep = ";")
        i = 0
        scanDataTuples = []
        for value in sp:
            # buffer = float(value)
            scanDataTuples += {(i*rd.angleCorrection,float(value))}  
            #scanDataTuples += {(i,float(value))}
            i += rd.measurementStep
        scanData.append(scanDataTuples)
    f.close()

    #for counter, line in enumerate(scanData):
     #   print(counter, line)
    #print(scanData)
    
    #PlotValues(scanData, 0, 'or')
    PlotValues(scanData, 1, 'or')
    
    derivatives.append(CalcDerivative(scanData[0], rd.minValidSensorData))
    derivatives.append(CalcDerivative(scanData[1], rd.minValidSensorData))
    #print(derivatives[1])
    #PlotValues(derivatives, 0, 'bo' )
    PlotValues(derivatives, 1, 'bo' )
    cylinders = []
    
    
    cylinders.append(FindCylinders(scanData[1], derivatives[1], 5, rd.minValidSensorData))
    for data in cylinders:
        scatter([c[0] for c in data],[c[1] for c in data],c = 'g', s=200)
    show()
    print("Zylinder: ")
    print(cylinders)
    print(scanData[1])
    print(len(scanData))
    
    '''
    
    
    
    
    