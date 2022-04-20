import struct
from math import sin
from math import cos
from math import pi
from math import radians

class Map:
    data_now = [0] * 14
    data_old = [0] * 14
    data_as_bytes = []

    def __init__(self):
        self.map = [[False]*1000 for i in range(1000)]
        self.data_now = [0] * 14
        self.data_old = [0] * 14
        self.data_as_bytes = []
        self.robot_angle = 0
        self.robot_position_x = 500
        self.robot_position_y = 500

    def save_data_now(self):
        #self.data_old.clear(self)
        self.data_old = self.data_now
        self.data_now = self.convert_byte_to_float(self)
        self.data_as_bytes.clear()
        print(self.data_now)

    def convert_byte_to_float(self):
        bytebuffer = bytearray()
        floatbuffer = []
        j = 0
        print(self.data_as_bytes)
        print(len(self.data_as_bytes))
        for i in range(14):
            while self.data_as_bytes[j] != 10 and j < len(self.data_as_bytes):
                bytebuffer.append(self.data_as_bytes[j])
                j = j + 1
            print(bytebuffer)
            floatbuffer.append(float(bytebuffer))
            print(floatbuffer)
            j = j + 1
            bytebuffer.clear()
        return floatbuffer

    def calculate_robot_position(self):
        self.robot_angle = self.robot_angle + self.data_now[13]
        self.robot_position_x = self.robot_position_x + self.data_now[12] * sin(radians(self.robot_angle))
        self.robot_position_y = self.robot_position_y + self.data_now[12] * cos(radians(self.robot_angle))

    def update_walls(self):
        distances = [0, 0] * 12
        coordinates = [0, 0] * 12

        distances[0] = 0, self.data_now[0]
        distances[1] = sin(radians(30)) * self.data_now[1], cos(radians(30)) * self.data_now[1]
        distances[2] = sin(radians(60)) * self.data_now[2], cos(radians(60)) * self.data_now[2]
        distances[3] = self.data_now[3], 0
        distances[4] = sin(radians(120)) * self.data_now[4], cos(radians(120)) * self.data_now[4]
        distances[5] = sin(radians(150)) * self.data_now[5], cos(radians(150)) * self.data_now[5]
        distances[6] = 0, - self.data_now[6]
        distances[7] = sin(radians(210)) * self.data_now[7], cos(radians(210)) * self.data_now[7]
        distances[8] = sin(radians(240)) * self.data_now[8], cos(radians(240)) * self.data_now[8]
        distances[9] = - self.data_now[9], 0
        distances[10] = sin(radians(300)) * self.data_now[10], cos(radians(300)) * self.data_now[10]
        distances[11] = sin(radians(330)) * self.data_now[11], cos(radians(330)) * self.data_now[11]

        coordinates[0] = round(distances[0[0]] + self.robot_position_x, distances[0[1]] + self.robot_position_y)
        coordinates[1] = round(distances[1[0]] + self.robot_position_x, distances[1[1]] + self.robot_position_y)
        coordinates[2] = round(distances[2[0]] + self.robot_position_x, distances[2[1]] + self.robot_position_y)
        coordinates[3] = round(distances[3[0]] + self.robot_position_x, distances[3[1]] + self.robot_position_y)
        coordinates[4] = round(distances[4[0]] + self.robot_position_x, distances[4[1]] + self.robot_position_y)
        coordinates[5] = round(distances[5[0]] + self.robot_position_x, distances[5[1]] + self.robot_position_y)
        coordinates[6] = round(distances[6[0]] + self.robot_position_x, distances[6[1]] + self.robot_position_y)
        coordinates[7] = round(distances[7[0]] + self.robot_position_x, distances[7[1]] + self.robot_position_y)
        coordinates[8] = round(distances[8[0]] + self.robot_position_x, distances[8[1]] + self.robot_position_y)
        coordinates[9] = round(distances[9[0]] + self.robot_position_x, distances[9[1]] + self.robot_position_y)
        coordinates[10] = round(distances[10[0]] + self.robot_position_x, distances[10[1]] + self.robot_position_y)
        coordinates[11] = round(distances[11[0]] + self.robot_position_x, distances[11[1]] + self.robot_position_y)

        map[coordinates[0[0]]][coordinates[0[1]]] = True
        map[coordinates[1[0]]][coordinates[1[1]]] = True
        map[coordinates[2[0]]][coordinates[2[1]]] = True
        map[coordinates[3[0]]][coordinates[3[1]]] = True
        map[coordinates[4[0]]][coordinates[4[1]]] = True
        map[coordinates[5[0]]][coordinates[5[1]]] = True
        map[coordinates[6[0]]][coordinates[6[1]]] = True
        map[coordinates[7[0]]][coordinates[7[1]]] = True
        map[coordinates[8[0]]][coordinates[8[1]]] = True
        map[coordinates[9[0]]][coordinates[9[1]]] = True
        map[coordinates[10[0]]][coordinates[10[1]]] = True
        map[coordinates[11[0]]][coordinates[11[1]]] = True


