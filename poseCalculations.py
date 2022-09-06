# -*- coding: utf-8 -*-
"""
Created on Tue May 31 13:36:05 2022
@author: Jan
"""
from math import sin, cos, pi, radians
import RobotData as rd

def CalcPivotPose(oldPose, motorData):
    '''
    Parameters
    ----------
    oldPose : (x,y,orientation)
        the last pose if the robot.
    motorData : (int, int)
        revolutions of right and left motor

    Returns
    -------
    new robotpose (x,y,theta)
    '''
    turn = ''
    if (motorData[0] == 0):  # forwardmotion
        mRight = abs(motorData[1])
        mLeft = abs(motorData[2])
    elif (motorData[0] == 999): # backwardmotion
        mRight = motorData[1] * (-1)
        mLeft = motorData[2]
    else:  # we need to change sign if robot turns
        if motorData[0] > 0:
            # beide motorenwerte sind positiv. wir brauchen: rechts negativ, links positiv
            turn = 'left'
            mRight = motorData[1] * (-1)
            mLeft = abs(motorData[2])
        else:
            # beide motorenwerte sind negativ wir brauchen: rechts negativ, links positiv
            turn = 'right'
            mRight = motorData[1]
            mLeft = abs(motorData[2])
    distances = ConvertRevsToDistance((motorData[0], mRight, mLeft, motorData[3]))
    distLeft = distances[0]
    distRight = distances[1]
    thetaOld = oldPose[2]

    if distLeft == distRight:  # simpler calculation if actual traveled distances of both wheels are the same.
        l = distLeft
        thetaNew = thetaOld
        xNew = oldPose[0] + l * cos(thetaOld)
        yNew = oldPose[1] + l * sin(thetaOld)
    else:  # calsulation for driven turns
        alpha = (distRight - distLeft) / rd.WheelBaseMm
        R = distLeft / alpha
        xC = oldPose[0] - (R + rd.WheelBaseMm / 2) * sin(thetaOld)
        yC = oldPose[1] - (R + rd.WheelBaseMm / 2) * (-1) * cos(thetaOld)
        if turn == 'left':
            thetaNew = (thetaOld + alpha) % (2 * pi)
        elif turn == 'right':
            thetaNew = (thetaOld - alpha) % (2 * pi)
        else:
            thetaNew = (thetaOld + alpha) % (2 * pi)
        xNew = xC + (R + rd.WheelBaseMm / 2) * sin(thetaNew)
        yNew = yC + (R + rd.WheelBaseMm / 2) * (-1) * cos(thetaNew)
    return (xNew, yNew, thetaNew)


def CalcSensorPose(oldPose, motorData):
    '''
    returns Pose of the sensors (mounting point of the servomotor)

    '''
    thetaOld = oldPose[2]
    xNew = oldPose[0] - rd.sensorOffset * cos(thetaOld)
    yNew = oldPose[1] - rd.sensorOffset * sin(thetaOld)
    return (xNew, yNew, thetaOld)


def CalcPoint(pose, valuePair):
    '''
    Calculates point from pose and scandata
    '''
    thetaOld = pose[2]
    alpha = thetaOld + radians(valuePair[0])
    thetaNew = alpha % (2 * pi)
    xNew = pose[0] + valuePair[1] * cos(thetaNew)
    yNew = pose[1] + valuePair[1] * sin(thetaNew)
    return (xNew, yNew)

def ConvertRevsToDistance(motorData):
    '''
    takes motorData and calculates the driven distance per wheel
    '''
    duration = motorData[3] / 1000
    revsRight = motorData[1] / 360
    revsLeft = motorData[2] / 360
    distR = revsLeft * rd.speed60rpm
    distL = revsRight * rd.speed60rpm

    return (distL, distR)











