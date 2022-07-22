# -*- coding: utf-8 -*-
"""
Created on Tue May 31 14:06:44 2022

@author: Jan
"""

# Globale Daten zu Roboter

WidthMm = 202
LenghtMm = 205

WheelBaseMm = 181
WheelCircumferenceMm = 68.5 
distance360Degrees = 10.0 # Umrechnungsfaktor, da 360° nicht einer realen Umdreheung des Rads entspricht

sensorOffset = 64
sensorPivotOffset = 50

measurementStep = 5
angleCorrection = 140/180 # Umrechnungsfaktor zwischen Servomotorwinkel und realem Winkel
measurements = 72

minValidSensorData = 5.0

speed60rpm = 115.42 # Geschwindigkeit in mm/s bei 60rpm Motordrehzahl

ignoreValue = 400

'''
Überlegung:
    
    Laufzeit der Ultraschallmessungen reduzieren
    



'''