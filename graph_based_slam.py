"""

Graph based SLAM example

author: Atsushi Sakai (@Atsushi_twi)

Ref

[A Tutorial on Graph-Based SLAM]
(http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf)

"""
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../utils/")

import copy
import itertools
import math

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from pylab import show
from  DataManager import DataManager, EuclideanDistance
import matplotlib.pyplot as plt
#from graphslam.load import load_g2o_se2_JZ
from iterative_closest_point import *
from raw_data import Roommap

#  Simulation parameter
Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2
R_sim = np.diag([0.1, np.deg2rad(10.0)]) ** 2

DT = 1.0  # time tick [s]
SIM_TIME = 100.0  # simulation time [s]
MAX_RANGE = 4000.0  # maximum observation range
STATE_SIZE = 3  # State size [x,y,yaw]

# Covariance parameter of Graph Based SLAM
C_SIGMA1 = 0.1
C_SIGMA2 = 0.1
C_SIGMA3 = np.deg2rad(1.0)

MAX_ITR = 20  # Maximum iteration

show_graph_d_time = 1.0  # [s]
show_animation = True


raw_data_old = []


class Edge:
    def __init__(self):
        self.e = np.zeros((3, 1))
        self.omega = np.zeros((3, 3))  # information matrix
        self.d1 = 0.0
        self.d2 = 0.0
        self.yaw1 = 0.0
        self.yaw2 = 0.0
        self.angle1 = 0.0
        self.angle2 = 0.0
        self.id1 = 0
        self.id2 = 0


def cal_observation_sigma():
    sigma = np.zeros((3, 3))
    sigma[0, 0] = C_SIGMA1 ** 2
    sigma[1, 1] = C_SIGMA2 ** 2
    sigma[2, 2] = C_SIGMA3 ** 2

    return sigma


def calc_3d_rotational_matrix(angle):
    return Rot.from_euler('z', angle).as_matrix()


def calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
              angle1, d2, angle2, t1, t2):
    edge = Edge()

    tangle1 = pi_2_pi(yaw1 + angle1)
    tangle2 = pi_2_pi(yaw2 + angle2)
    tmp1 = d1 * math.cos(tangle1)
    tmp2 = d2 * math.cos(tangle2)
    tmp3 = d1 * math.sin(tangle1)
    tmp4 = d2 * math.sin(tangle2)

    edge.e[0, 0] = x2 - x1 - tmp1 + tmp2
    edge.e[1, 0] = y2 - y1 - tmp3 + tmp4
    edge.e[2, 0] = 0

    Rt1 = calc_3d_rotational_matrix(tangle1)
    Rt2 = calc_3d_rotational_matrix(tangle2)

    sig1 = cal_observation_sigma()
    sig2 = cal_observation_sigma()

    #edge.omega = np.linalg.inv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)
    edge.omega = np.linalg.pinv(Rt1 @ sig1 @ Rt1.T + Rt2 @ sig2 @ Rt2.T)
    
    edge.d1, edge.d2 = d1, d2
    edge.yaw1, edge.yaw2 = yaw1, yaw2
    edge.angle1, edge.angle2 = angle1, angle2
    edge.id1, edge.id2 = t1, t2

    return edge


def calc_edges(x_list, z_list):
    edges = []
    cost = 0.0
    z_ids = list(itertools.combinations(range(len(z_list)), 2))

    for (t1, t2) in z_ids:
        x1, y1, yaw1 = x_list[0, t1], x_list[1, t1], x_list[2, t1]
        x2, y2, yaw2 = x_list[0, t2], x_list[1, t2], x_list[2, t2]

        if z_list[t1] is None or z_list[t2] is None:
            continue  # No observation

        for iz1 in range(len(z_list[t1][:, 0])):
            for iz2 in range(len(z_list[t2][:, 0])):
                if z_list[t1][iz1, 3] == z_list[t2][iz2, 3]:
                    d1 = z_list[t1][iz1, 0]
                    angle1, phi1 = z_list[t1][iz1, 1], z_list[t1][iz1, 2]
                    d2 = z_list[t2][iz2, 0]
                    angle2, phi2 = z_list[t2][iz2, 1], z_list[t2][iz2, 2]

                    edge = calc_edge(x1, y1, yaw1, x2, y2, yaw2, d1,
                                     angle1, d2, angle2, t1, t2)

                    edges.append(edge)
                    cost += (edge.e.T @ edge.omega @ edge.e)[0, 0]

    print("cost:", cost, ",n_edge:", len(edges))
    return edges


def calc_jacobian(edge):
    t1 = edge.yaw1 + edge.angle1
    A = np.array([[-1.0, 0, edge.d1 * math.sin(t1)],
                  [0, -1.0, -edge.d1 * math.cos(t1)],
                  [0, 0, 0]])

    t2 = edge.yaw2 + edge.angle2
    B = np.array([[1.0, 0, -edge.d2 * math.sin(t2)],
                  [0, 1.0, edge.d2 * math.cos(t2)],
                  [0, 0, 0]])

    return A, B


def fill_H_and_b(H, b, edge):
    A, B = calc_jacobian(edge)

    id1 = edge.id1 * STATE_SIZE
    id2 = edge.id2 * STATE_SIZE

    H[id1:id1 + STATE_SIZE, id1:id1 + STATE_SIZE] += A.T @ edge.omega @ A
    H[id1:id1 + STATE_SIZE, id2:id2 + STATE_SIZE] += A.T @ edge.omega @ B
    H[id2:id2 + STATE_SIZE, id1:id1 + STATE_SIZE] += B.T @ edge.omega @ A
    H[id2:id2 + STATE_SIZE, id2:id2 + STATE_SIZE] += B.T @ edge.omega @ B

    b[id1:id1 + STATE_SIZE] += (A.T @ edge.omega @ edge.e)
    b[id2:id2 + STATE_SIZE] += (B.T @ edge.omega @ edge.e)

    return H, b


def graph_based_slam(x_init, hz):
    print("start graph based slam")

    z_list = copy.deepcopy(hz)

    x_opt = copy.deepcopy(x_init)
    nt = x_opt.shape[1]
    n = nt * STATE_SIZE
    
    #print("JZ3: ")
    #print(x_opt)
    #print(z_list)
    
    for itr in range(MAX_ITR):
        
        
        edges = calc_edges(x_opt, z_list)

        H = np.zeros((n, n))
        b = np.zeros((n, 1))

        
        for edge in edges:
            H, b = fill_H_and_b(H, b, edge)
        
        
        # to fix origin
        H[0:STATE_SIZE, 0:STATE_SIZE] += np.identity(STATE_SIZE)

        #dx = - np.linalg.inv(H) @ b
        dx = - np.linalg.pinv(H) @ b
        
        for i in range(nt):
            x_opt[0:3, i] += dx[i * 3:i * 3 + 3, 0]

        diff = dx.T @ dx
        print("iteration: %d, diff: %f" % (itr + 1, diff))
        if diff < 1.0e-5:
            break

    return x_opt


def calc_input():
    v = 1.0  # [m/s]
    yaw_rate = 0.1  # [rad/s]
    u = np.array([[v, yaw_rate]]).T
    return u


def observation(xTrue, xd, u, RFID):
    xTrue = motion_model(xTrue, u)
    # add noise to gps x-y
    z = np.zeros((0, 4))

    for i in range(len(RFID[:, 0])):

        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]
        phi = pi_2_pi(math.atan2(dy, dx))
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
            angle_noise = np.random.randn() * Q_sim[1, 1]
            angle += angle_noise
            phi += angle_noise
            zi = np.array([dn, angle, phi, i])
            z = np.vstack((z, zi))

    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])
    
    x = F @ x + B @ u

    return x

def motion_model_jz(x, u):
    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT]])
    
    x = F @ x + B @ u
    return x

def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def observation_jz(xTrue, xd, u, RFID ):
    
    xTrue = motion_model(xTrue, u)

    # add noise to gps x-y
    z = np.zeros((0, 4))
    
    for i in range(len(RFID[:, 0])):
        dx = RFID[i, 0] - xTrue[0, 0]
        dy = RFID[i, 1] - xTrue[1, 0]
        d = math.hypot(dx, dy)
        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]
        phi = pi_2_pi(math.atan2(dy, dx))
        
        if d <= MAX_RANGE:
            dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
            angle_noise = np.random.randn() * Q_sim[1, 1]
            angle += angle_noise
            phi += angle_noise
            zi = np.array([dn, angle, phi, i])
            z = np.vstack((z, zi))
    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model(xd, ud)
    
    return xTrue, z, xd, ud

def observation_jz2(xTrue, xd, u, RFID, lmPairs, obs):
    
    xTrue = motion_model_jz(xTrue, u)
    
    '''
    todo: wir übernehmen nur aktuelle landmarken und geben als distanzen die sensordaten (winkel, distanz an d und angle/phi)
    falls keine landmarken gefunden wurden wird ICP benutzt
    '''
    # add noise to gps x-y
    z = np.zeros((0, 4))
    
    for i in range(len(RFID[:, 0])):
        print('RFID',RFID[i])
        #now: did we observe the landmark in this scan?
        for lm in lmPairs:
            if lm[1]==RFID[i,3]:
                #Landmarke wurde observiert im Scan
                for lmObs in obs:
                    if lmObs[0] == lm[0]:
                        print('LMOBS: ', lmObs)
                        dx = lmObs[1] - xTrue[0, 0]
                        dy = lmObs[2] - xTrue[1, 0]
                        d = math.hypot(dx, dy)
                        angle = pi_2_pi(math.atan2(dy, dx)) - xTrue[2, 0]
                        phi = pi_2_pi(math.atan2(dy, dx))
                        print('dx ',dx,'dy ',dy,'d ', d, 'angle ',angle,'phi ', phi)
                        
                        dn = d + np.random.randn() * Q_sim[0, 0]  # add noise
                        angle_noise = np.random.randn() * Q_sim[1, 1]
                        angle += angle_noise
                        phi += angle_noise
                        zi = np.array([dn, angle, phi, i])
                        print('dn',dn,'angle',angle,'phi',phi,'i',i)
                        z = np.vstack((z, zi))
                    
    # add noise to input
    ud1 = u[0, 0] + np.random.randn() * R_sim[0, 0]
    ud2 = u[1, 0] + np.random.randn() * R_sim[1, 1]
    ud = np.array([[ud1, ud2]]).T

    xd = motion_model_jz(xd, ud)
    
    return xTrue, z, xd, ud

def main_jz(lmMode):
    robot = DataManager()
    nsim = 3
    f = open("Testlauf_Etage2_GebB_30072022.csv")
    j = 0
    edgeData = []
    poseData = [(0,0,0,0)]
    LINE = '###################################################################'
    # Init Data for Graphslam 
    xTrue = np.zeros((STATE_SIZE, 1))# State Vector [x y yaw v]'
    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning
    # history
    hxTrue = []
    hxDR = []
    hz = []
    d_time = 0.0
    init = False
    time = 0.0
    
    for l in f: #Simuliert die eingehenden Bluetoothdaten 
        if j < 90:
            print('*Datensatz: ', j+1, ' ******************************************************************************************************')
            sp = l.split(sep = ";")
            robot.SplitDataStep5(sp)
            robot.CreateRobotData(j)
            robot.PlotSensorPoseData()
            robot.PlotScanDataPoints('no400')
            plt.title(j+1)
            show()
            # Start SLAM implementation
            
            #ICP###################################################################
            
            if lmMode == 'prepareICP':
                previous_points = np.array([])
                current_points = np.array([])
                if j > 0:
                    previous_points, current_points = robot.PrepareICP(j)
                    
                if len(current_points) != 0:
                    print('anzahl PP: ',len(previous_points[0]))
                    print('pp', previous_points)
                    print('anzahl CP: ',len(current_points[0]))
                    print('cc', current_points)
                    RFID = np.array([(point[0], point[1],0) for point in current_points])
                    
                else:
                    validLandmarks = robot.GetValidLandmarks(2)
                    RFID = np.array([(landmark[1], landmark[2],0) for landmark in validLandmarks])
            elif lmMode == 'allPoints':
                allPoints = []
                for line in robot.scanDataPointsNo400:
                    for tupel in line:
                        allPoints += {tupel}
                RFID = np.array([(landmark[0], landmark[1],0) for landmark in allPoints])
            elif lmMode == 'aktLandmarks':
                RFID = np.array([(landmark[0], landmark[1]) for landmark in robot.landmarksInGC[j]])
            elif lmMode == 'ICP':
                return 0
            elif lmMode == 'validLm1':
                validLandmarks = robot.GetValidLandmarks(1)
                RFID = np.array([(landmark[1], landmark[2],0, landmark[0]) for landmark in validLandmarks])
           
            
            
            xDR = np.array([[robot.sensor_pose_data[j][0]],
                               [robot.sensor_pose_data[j][1]],
                               [robot.sensor_pose_data[j][2]]
                               ])
            #print('xDR: ', xDR)
            print('RFID ', len(RFID))
            if not init:
                hxTrue = xTrue
                hxDR = xTrue
                init = True
            else:
                hxDR = np.hstack((hxDR, xDR))
                hxTrue = np.hstack((hxTrue, xTrue))
            time += DT
            d_time += DT
            #u = calc_input()  #brauchen wir nicht stattdessen:
                    
            if j == 0:
                u = np.array([[0,0]]).T
            else:
                d = EuclideanDistance(robot.sensor_pose_data[j-1][0],
                                      robot.sensor_pose_data[j-1][1],
                                      robot.sensor_pose_data[j][0],
                                      robot.sensor_pose_data[j][1] )

                diffTheta = robot.sensor_pose_data[j][2]-robot.sensor_pose_data[j-1][2]
                u = np.array([[d,diffTheta]]).T
            
            print(LINE)
            
            doIt = False
            if doIt:
                xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)
    
                hz.append(z)
                
                x_opt = graph_based_slam(hxDR, hz)
                print(LINE)
                
                #print('x_opt: ', x_opt)
                d_time = 0.0
    
                if show_animation:  # pragma: no cover
                    plt.cla()
                    # for stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect(
                        'key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                    plt.plot(RFID[:, 0], RFID[:, 1], ".k")
                    plt.plot(hxTrue[0, :].flatten(),
                             hxTrue[1, :].flatten(), "-b")
                    plt.plot(hxDR[0, :].flatten(),
                             hxDR[1, :].flatten(), "-k")
                    plt.plot(x_opt[0, :].flatten(),
                             x_opt[1, :].flatten(), "-r")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Measurement: " + str(j+1)[0:5])
                    plt.pause(1.0)
        j += 1
    f.close()
    return 0

def RunGraphBasedSLAM(lmMode):
    '''
    global raw_data_old
    if raw_data_old == Roommap.data_all:
        return
    '''
    ##############################################################################################################
    robot = DataManager()
    nsim = 3
    j = 0
    edgeData = []
    poseData = [(0,0,0,0)]
    LINE = '###################################################################'
    # Init Data for Graphslam 
    xTrue = np.zeros((STATE_SIZE, 1))# State Vector [x y yaw v]'
    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning
    # history
    hxTrue = []
    hxDR = []
    hz = []
    d_time = 0.0
    init = False
    time = 0.0
    
    print("länge: ", len(Roommap.data_all))
    for l in Roommap.data_all:
        print("Datensatz: ",j , "Inhalt:", l)
        robot.SplitDataStep5(l)
        robot.CreateRobotData(j)

        # Start SLAM implementation
        lmMode = 'validLm1'
       
        if lmMode == 'prepareICP':
            previous_points = np.array([])
            current_points = np.array([])
            if j > 0:
                previous_points, current_points = robot.PrepareICP(j)
                
            if len(current_points) != 0:
                print('anzahl PP: ',len(previous_points[0]))
                print('pp', previous_points)
                print('anzahl CP: ',len(current_points[0]))
                print('cc', current_points)
                RFID = np.array([(point[0], point[1],0) for point in current_points])
               
            else:
                validLandmarks = robot.GetValidLandmarks(2)
                RFID = np.array([(landmark[1], landmark[2],0) for landmark in validLandmarks])
        elif lmMode == 'allPoints':
            allPoints = []
            for line in robot.scanDataPointsNo400:
                for tupel in line:
                    allPoints += {tupel}
            RFID = np.array([(landmark[0], landmark[1],0) for landmark in allPoints])
        elif lmMode == 'aktLandmarks':
            RFID = np.array([(landmark[0], landmark[1]) for landmark in robot.landmarksInGC[j]])
        elif lmMode == 'ICP':
            return 0
        elif lmMode == 'validLm1':
            validLandmarks = robot.GetValidLandmarks(1)
            RFID = np.array([(landmark[1], landmark[2],0, landmark[0]) for landmark in validLandmarks])
        else:
            print("No valid Landmarkmode selected")
            return 0
        xDR = np.array([[robot.sensor_pose_data[j][0]],
                           [robot.sensor_pose_data[j][1]],
                           [robot.sensor_pose_data[j][2]]
                           ])
        if not init:
            hxTrue = xTrue
            hxDR = xTrue
            init = True
        else:
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))
        time += DT
        d_time += DT
        #u = calc_input()  #brauchen wir nicht stattdessen:
               
        if j == 0:
            u = np.array([[0,0]]).T
        else:
            d = EuclideanDistance(robot.sensor_pose_data[j-1][0],
                                  robot.sensor_pose_data[j-1][1],
                                  robot.sensor_pose_data[j][0],
                                  robot.sensor_pose_data[j][1] )

            diffTheta = robot.sensor_pose_data[j][2]-robot.sensor_pose_data[j-1][2]
            u = np.array([[d,diffTheta]]).T
            
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)
        hz.append(z)
        x_opt = graph_based_slam(hxDR, hz)

        d_time = 0.0
        j += 1
    ##############################################################################################################
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(RFID[:, 0], RFID[:, 1], ".k")
    plt.plot(hxTrue[0, :].flatten(),
             hxTrue[1, :].flatten(), "-b")
    plt.plot(hxDR[0, :].flatten(),
             hxDR[1, :].flatten(), "-k")
    plt.plot(x_opt[0, :].flatten(),
             x_opt[1, :].flatten(), "-r")
    plt.axis("equal")
    plt.grid(True)
    plt.title("Graph-Based-SLAM result")
    plt.pause(1.0)
    return plt

def main():
    print(__file__ + " start!!")
    
    time = 0.0

    # RFID positions [x, y, yaw]
    RFID = np.array([[10.0, -2.0, 0.0],
                     [15.0, 10.0, 0.0],
                     [3.0, 15.0, 0.0],
                     [-5.0, 20.0, 0.0],
                     [-5.0, 5.0, 0.0]
                     ])

    # State Vector [x y yaw v]'
    xTrue = np.zeros((STATE_SIZE, 1))
    xDR = np.zeros((STATE_SIZE, 1))  # Dead reckoning

    # history
    hxTrue = []
    hxDR = []
    hz = []
    d_time = 0.0
    init = False
    while SIM_TIME >= time:

        if not init:
            hxTrue = xTrue
            hxDR = xTrue
            init = True
        else:
            hxDR = np.hstack((hxDR, xDR))
            hxTrue = np.hstack((hxTrue, xTrue))

        time += DT
        d_time += DT
        
        print('RFID:', RFID)
        
        print('xTrue1: ', xTrue)
        u = calc_input()
        print('xTrue2: ', xTrue)
        xTrue, z, xDR, ud = observation(xTrue, xDR, u, RFID)

        hz.append(z)

        if d_time >= show_graph_d_time:
            x_opt = graph_based_slam(hxDR, hz)
            d_time = 0.0

            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(RFID[:, 0], RFID[:, 1], "*k")

                plt.plot(hxTrue[0, :].flatten(),
                         hxTrue[1, :].flatten(), "-b")
                plt.plot(hxDR[0, :].flatten(),
                         hxDR[1, :].flatten(), "-k")
                plt.plot(x_opt[0, :].flatten(),
                         x_opt[1, :].flatten(), "-r")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Time" + str(time)[0:5])
                plt.pause(1.0)
                
if __name__ == '__main__':
    #main()
    main_jz('validLm1')