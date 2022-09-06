# -*- coding: utf-8 -*-
"""
Created on Tue May 31 14:06:44 2022
@author: Jan
"""

WidthMm = 202
LenghtMm = 205

WheelBaseMm = 181
WheelCircumferenceMm = 68.5

sensorOffset = 64
sensorPivotOffset = 50

measurementStep = 5
angleCorrection = 140/180 # Umrechnungsfaktor zwischen Servomotorwinkel und realem Winkel
measurements = 72

minValidSensorData = 5.0

speed60rpm = 115.42 # Geschwindigkeit in cm/s bei 60rpm Motordrehzahl, bzw Diastanz pro Umdrehung

ignoreValue = 400

landmarkMargin = 1000
minObservations = 2