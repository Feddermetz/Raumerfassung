from DataManager import DataManager
from mapping import Roommap
from time import sleep

import matplotlib.pyplot as plt

dmRobot1 = DataManager()


def run_no_correction_slam():
    j = 0
    for dataset in Roommap.data_all:
        dmRobot1.SplitDataStep5(dataset)
        dmRobot1.CreatePoseDataStep(j)
        dmRobot1.CreateUssDataPosesStep(j)
        motor_pose_x = dmRobot1.motor_pose_data[j]
        motor_pose_y = dmRobot1.motor_pose_data[j]
        motor_pose_x = motor_pose_x[0]
        motor_pose_y = motor_pose_y[1]
        plt.plot(motor_pose_x, motor_pose_y, 'ro', zorder=2)
        uss_data = dmRobot1.scanDataPointsNo400[j]
        for element in uss_data:
            plt.plot(element[0], element[1], 'bo', zorder=1)

        j += 1

    return plt

"""
    j = 0
    #plt.cla()
    for dataset in Roommap.data_all:
        dmRobot1.SplitDataStep5(dataset)
        dmRobot1.CreatePoseDataStep(j)
        dmRobot1.CreateUssDataPosesStep(j)
        #dmRobot1.PlotScanDataPoints('no400')
        #dmRobot1.PlotMotorPoseData()
        #print("motor_pose_data: ", dmRobot1.motor_pose_data[j])
        motor_pose_x = dmRobot1.motor_pose_data[j]
        motor_pose_y = dmRobot1.motor_pose_data[j]
        motor_pose_x = motor_pose_x[0]
        motor_pose_y = motor_pose_y[1]
        #print(motor_pose_x)
        #print(motor_pose_y)
        #print(motor_pose_x[0])
        plt.plot(motor_pose_x, motor_pose_y, 'ro', zorder=2)
        #print(dmRobot1.scanDataPointsNo400[j])
        uss_data = dmRobot1.scanDataPointsNo400[j]
        for element in uss_data:
            plt.plot(element[0], element[1], 'bo', zorder=1)
        #plt.show()
        #sleep(1.0)
        #plt.close()

        #plt.plot(dmRo, 'bo')
        #plt.show()
        j += 1
        #plt.show()
    #plt.show()
    return plt
    #print("motor_pose: ", dmRobot1.motor_pose_data)
    #print("scanDataPointsNo400: ", dmRobot1.scanDataPointsNo400)
    """