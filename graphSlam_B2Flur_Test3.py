# -*- coding: utf-8 -*-
"""
Created on Fri Aug  7 13:23:31 2022

Test mit neu ermittelten landmarken (siehe einträge imn worddatei vom 06 + 07.08.2022)
@author: Jan
"""
#JZ Start######################################################################
from pylab import *
from  DataManager import DataManager
import matplotlib.pyplot as plt
import numpy as np
#JZ Ende#######################################################################
from iterative_closest_point import *
import copy
import math
import itertools
from graph_based_slam import calc_3d_rotational_matrix, calc_jacobian, cal_observation_sigma, \
                             calc_input, observation, motion_model, Edge, pi_2_pi



#  Simulation parameter
nsim = 3 # Anzahl durchgänge ICP
Qsim = np.diag([0.01, np.deg2rad(0.010)])**2 # error added to range and bearing
Rsim = np.diag([0.1, np.deg2rad(1.0)])**2 # error added to [v, w]

DT = 2.0  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 50000.0  # maximum observation range
STATE_SIZE = 3  # State size [x,y,yaw]

# TODO: Why not use Qsim ?
# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = np.deg2rad(1.0)

MAX_ITR = 20  # Maximum iteration during optimization
timesteps = 3


def DoTheThing():
    # consider only 2 landmarks for simplicity
    # RFID positions [x, y, yaw]
    


    
    RFID1 = np.array([[10.1, -2000.0, 0.0],
#                  [15.0, 10.0, 0.0],
#                 [3.0, 15.0, 0.0],
#                  [-5.0, 20.0, 0.0],
                  [-5.0, 5.0, 0.0]
                 ])
    #print('RFID',RFID)
    
    RFID = np.array([(landmark[0], landmark[1],0) for landmark in robot.allCylinders])
    print('RFID',RFID)
    
    # JZ
    v = 1
    angle = 0
    
    # State Vector [x y yaw v]'
    xTrue = np.zeros((STATE_SIZE, 1))
    #print('xTrue',xTrue)

    print('xTrue',xTrue)
    
    #xDR = np.zeros((STATE_SIZE, v))  # Dead reckoning
    xDR = np.array([[robot.motor_pose_data[j][0]],
                       [robot.motor_pose_data[j][1]],
                       [robot.motor_pose_data[j][2]]
                       ])
    xTrue[2] = np.deg2rad(angle)
    xDR[2] = np.deg2rad(angle)
    
    print('xTrue ', xTrue)
    print('xDR ', xDR)
    
    # history initial values
    hxTrue = xTrue
    hxDR = xTrue
    _, z, _, _ = observation(xTrue, xDR, np.array([[0,0]]).T, RFID)
    hz = [z]
    
    print('hz', hz)
    for i in range(timesteps):
        u = calc_input()
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)
        hxDR = np.hstack((hxDR, xDR))
        hxTrue = np.hstack((hxTrue, xTrue))
        hz.append(z)
    
    # visualize
    graphics_radius = 0.3
    plt.plot(RFID[:, 0], RFID[:, 1], "*k", markersize=20)
    plt.plot(hxDR[0, :], hxDR[1, :], '.', markersize=50, alpha=0.8, label='Odom')
    plt.plot(hxTrue[0, :], hxTrue[1, :], '.', markersize=20, alpha=0.6, label='X_true')
    
    for i in range(hxDR.shape[1]):
        x = hxDR[0, i]
        y = hxDR[1, i]
        yaw = hxDR[2, i]
        plt.plot([x, x + graphics_radius * np.cos(yaw)],
                 [y, y + graphics_radius * np.sin(yaw)], 'r')
        d = hz[i][:, 0]
        angle = hz[i][:, 1]
        plt.plot([x + d * np.cos(angle + yaw)], [y + d * np.sin(angle + yaw)], '.',
                 markersize=20, alpha=0.7)
        plt.legend()
    plt.grid()
    plt.show()
    
    # Copy the data to have a consistent naming with the .py file
    zlist = copy.deepcopy(hz)
    print('zlist: ', zlist)
    x_opt = copy.deepcopy(hxDR)
    xlist = copy.deepcopy(hxDR)
    number_of_nodes = x_opt.shape[1]
    n = number_of_nodes * STATE_SIZE
    
    # get all the possible combination of the different node
    zids = list(itertools.combinations(range(len(zlist)), 2))
    print('zids: ', zids)
    
    print("Node combinations: \n", zids)
    print(range(xlist.shape[1]))
    
    for i in range(xlist.shape[1]):
        print()
        print("Node {} observed landmark with ID {}".format(i, zlist[i][0, 3]))
    
    # Initialize edges list
    edges = []
    
    # Go through all the different combinations
    for (t1, t2) in zids:
        x1, y1, yaw1 = xlist[0, t1], xlist[1, t1], xlist[2, t1]
        x2, y2, yaw2 = xlist[0, t2], xlist[1, t2], xlist[2, t2]
    
        # All nodes have valid observation with ID=0, therefore, no data association condition
        iz1 = 0
        iz2 = 0
    
        d1 = zlist[t1][iz1, 0]
        angle1, phi1 = zlist[t1][iz1, 1], zlist[t1][iz1, 2]
        d2 = zlist[t2][iz2, 0]
        angle2, phi2 = zlist[t2][iz2, 1], zlist[t2][iz2, 2]
    
        # find angle between observation and horizontal
        tangle1 = pi_2_pi(yaw1 + angle1)
        tangle2 = pi_2_pi(yaw2 + angle2)
    
        # project the observations
        tmp1 = d1 * math.cos(tangle1)
        tmp2 = d2 * math.cos(tangle2)
        tmp3 = d1 * math.sin(tangle1)
        tmp4 = d2 * math.sin(tangle2)
    
        edge = Edge()
        print(y1,y2, tmp3, tmp4)
        # calculate the error of the virtual measurement
        # node 1 as seen from node 2 throught the observations 1,2
        edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
        edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
        edge.e[2, 0] = pi_2_pi(yaw2 - yaw1 - tangle1 + tangle2)
    
        edge.d1, edge.d2 = d1, d2
        edge.yaw1, edge.yaw2 = yaw1, yaw2
        edge.angle1, edge.angle2 = angle1, angle2
        edge.id1, edge.id2 = t1, t2
    
        edges.append(edge)
    
        print("For nodes",(t1,t2))
        print("Added edge with errors: \n", edge.e)
    
        # Visualize measurement projections
        plt.plot(RFID[0, 0], RFID[0, 1], "*k", markersize=20)
        plt.plot([x1,x2],[y1,y2], '.', markersize=50, alpha=0.8)
        plt.plot([x1, x1 + graphics_radius * np.cos(yaw1)],
                 [y1, y1 + graphics_radius * np.sin(yaw1)], 'r')
        plt.plot([x2, x2 + graphics_radius * np.cos(yaw2)],
                 [y2, y2 + graphics_radius * np.sin(yaw2)], 'r')
    
        plt.plot([x1,x1+tmp1], [y1,y1], label="obs 1 x")
        plt.plot([x2,x2+tmp2], [y2,y2], label="obs 2 x")
        plt.plot([x1,x1], [y1,y1+tmp3], label="obs 1 y")
        plt.plot([x2,x2], [y2,y2+tmp4], label="obs 2 y")
        plt.plot(x1+tmp1, y1+tmp3, 'o')
        plt.plot(x2+tmp2, y2+tmp4, 'o')
    plt.legend()
    plt.grid()
    plt.show()
    
    # Initialize the system information matrix and information vector
    H = np.zeros((n, n))
    b = np.zeros((n, 1))
    x_opt = copy.deepcopy(hxDR)
    
    for edge in edges:
        id1 = edge.id1 * STATE_SIZE
        id2 = edge.id2 * STATE_SIZE
    
        t1 = edge.yaw1 + edge.angle1
        A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                      [0, -1.0, -edge.d1 * math.cos(t1)],
                      [0, 0, -1.0]])
    
        t2 = edge.yaw2 + edge.angle2
        B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                      [0, 1.0, edge.d2 * math.cos(t2)],
                      [0, 0, 1.0]])
    
        # TODO: use Qsim instead of sigma
        sigma = np.diag([C_SIGMA1, C_SIGMA2, C_SIGMA3])
        Rt1 = calc_3d_rotational_matrix(tangle1)
        Rt2 = calc_3d_rotational_matrix(tangle2)
        edge.omega = np.linalg.inv(Rt1 @ sigma @ Rt1.T + Rt2 @ sigma @ Rt2.T)
    
        # Fill in entries in H and b
        H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T @ edge.omega @ A
        H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T @ edge.omega @ B
        H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T @ edge.omega @ A
        H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T @ edge.omega @ B
    
        b[id1:id1 + STATE_SIZE] += (A.T @ edge.omega @ edge.e)
        b[id2:id2 + STATE_SIZE] += (B.T @ edge.omega @ edge.e)
    
    
    print("The determinant of H: ", np.linalg.det(H))
    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(H, extent=[0, n, 0, n])
    plt.subplot(1,2,2)
    plt.imshow(b, extent=[0, 1, 0, n])
    plt.show()
    
    # Fix the origin, multiply by large number gives same results but better visualization
    H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)
    print("The determinant of H after origin constraint: ", np.linalg.det(H))
    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(H, extent=[0, n, 0, n])
    plt.subplot(1,2,2)
    plt.imshow(b, extent=[0, 1, 0, n])
    plt.show()
    
    # Find the solution (first iteration)
    dx = - np.linalg.inv(H) @ b
    for i in range(number_of_nodes):
        x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]
    print("dx: \n",dx)
    print("ground truth: \n ",hxTrue)
    print("Odom: \n", hxDR)
    print("SLAM: \n", x_opt)
    
    # performance will improve with more iterations, nodes and landmarks.
    print("\ngraphSLAM localization error: ", np.sum((x_opt - hxTrue) ** 2))
    print("Odom localization error: ", np.sum((hxDR - hxTrue) ** 2))
    return 0




#JZ Start######################################################################
'''
Wir lesen mal die Daten ein und schauen wie es so läuft
'''

robot = DataManager()
f = open("Testlauf_Etage2_GebB_30072022.csv")
j = 0
edgeData = []
poseData = [(0,0,0,0)]
for l in f: #Simuliert die eingehenden Bluetoothdaten 
    if j < 5:
        print('*Datensatz: ', j+1, ' ********************************************************')
        sp = l.split(sep = ";")
        robot.SplitDataStep5(sp)
        robot.CreatePoseDataStep(j)
        robot.CreateUssDataPosesStep(j)
        robot.PlotMotorPoseData()
        #robot.PlotScanDataPoints('no400')
        robot.CreateDerivativesStep(j)
        robot.CreateCylinderDataStep(j)
        #robot.PlotCylInGlobalCoordinates()
        #show()
        #robot.PlotScanDerCyl(j)
        show()
        #print('Objects: ',robot.cylindersInGlobalCoordinates[j])
        #ICP###################################################################
        px = np.array([])
        py = np.array([])
        cx = np.array([])
        cy = np.array([])
        mode = 'association'
        if j != 0:
            if mode == 'pose':
                tupel = dmRobot1.sensor_pose_data[j-1]
                px = np.append(px, tupel[0])
                py = np.append(py, tupel[1])
                previous_points = np.vstack((px, py))
                #print(previous_points)
                
                tupel = robot.sensor_pose_data[j]
                # current points
                cx = np.append(cx, tupel[0])
                cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
            elif mode == 'uss':
                for tupel in dmRobot1.scanDataPoints[j-1]:
                # previous points
                    px = np.append(px, tupel[0])
                    py = np.append(py,tupel[1])
                previous_points = np.vstack((px, py))
                #print(previous_points)
                for tupel in dmRobot1.scanDataPoints[j]:
                # previous points
                    cx = np.append(cx, tupel[0])
                    cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
                #print(current_points)
            elif mode == 'landmark':
                print(robot.cylindersInGlobalCoordinates[j-1])
                print(robot.cylindersInGlobalCoordinates[j])
                
                for tupel in robot.cylindersInGlobalCoordinates[j-1]:
                # previous points
                    px = np.append(px, tupel[0])
                    py = np.append(py,tupel[1])
                previous_points = np.vstack((px, py))
                print('pp: ',previous_points)
                for tupel in robot.cylindersInGlobalCoordinates[j]:
                # previous points
                    cx = np.append(cx, tupel[0])
                    cy = np.append(cy,tupel[1])
                current_points = np.vstack((cx, cy))
                print('cp: ',current_points)
            elif mode == 'association':
                previous_points, current_points = robot.PrepareICP(j)
                print('anzahl PP: ',len(previous_points[0]))
                print('anzahl CP: ',len(current_points[0]))
            
            for _ in range(nsim):
                R, T = icp_matching(previous_points, current_points)
                print("R:", R)
                print("T:", T)
                
        #######################################################################
        
# Hier bauen wir den Ganzen kram ein aus dem Beispiel, als Routine verpackt
        DoTheThing()
        j += 1
f.close()
