from DataManager import DataManager
from raw_data import Roommap
import matplotlib.pyplot as plt

dmRobot1 = DataManager()


def run_no_correction_slam():
    """
    Reads the data of Datamanager and creates a pyplot from the data, without
    using a SLAM method for calculations.
    """
    plt.cla()
    j = 0  # just a counting index
    motor_pose_x = []
    motor_pose_y = []
    for dataset in Roommap.data_all:
        dmRobot1.SplitDataStep5(dataset)
        dmRobot1.CreatePoseDataStep(j)
        dmRobot1.CreateUssDataPosesStep(j)
        motor_pose_x.append(dmRobot1.motor_pose_data[j][0])
        motor_pose_y.append(dmRobot1.motor_pose_data[j][1])
        uss_data = dmRobot1.scanDataPointsNo400[j]
        print(uss_data)
        for element in uss_data:
            plt.plot(element[0], element[1], 'bo', zorder=1)
        j += 1
    plt.plot(motor_pose_x, motor_pose_y, 'ro', zorder=2, linestyle='solid')
    plt.grid(True)
    return plt