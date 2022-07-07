import struct
from math import sin, cos, radians
from time import sleep
from kivy.graphics import Color, Rectangle


class Map:

    def __init__(self):
        self.map = [[False]*1000 for i in range(1000)]
        self.data_now = [0] * 14
        self.data_old = [0] * 14
        self.data_as_bytes = bytearray()
        self.robot_angle = 0
        self.robot_position_x = 500
        self.robot_position_y = 500
        self.map_to_draw = False
        self.coordinates = []
        self.wall_coordinates = []

    def get_wall_coordinates(self):
        return self.wall_coordinates

    def save_data_now(self):
        self.data_old = self.data_now
        self.data_now = self.convert_byte_to_float()
        print('Länge des empfangenen Pakets: ', len(self.data_now))
        print('Inhalt des empfangenen Pakets: ', self.data_now)
        self.data_as_bytes.clear()

    def convert_byte_to_float(self):
        bytebuffer = bytearray()
        intbuffer = []
        j = 0
        #print('Länge des Bytearrays: ', len(self.data_as_bytes))
        for i in range(77):
            # != 10 because 10 is newline in ASCII-Code and the values are separated by a newline
            while self.data_as_bytes[j] != 10:
                bytebuffer.append(self.data_as_bytes[j])
                j = j + 1
            intbuffer.append(int(bytebuffer))
            j = j + 1
            bytebuffer.clear()
        return intbuffer


    '''
    def convert_byte_to_float(self):
        bytebuffer = bytearray()
        floatbuffer = []
        j = 0
        #print(self.data_as_bytes)
        #print(len(self.data_as_bytes))
        for i in range(77):
            print("i:")
            print(i)
            print("Länge Bytearray:")
            print(len(self.data_as_bytes))
            #while self.data_as_bytes[j] != 10 and j < len(self.data_as_bytes):
            while self.data_as_bytes[j] != 59 and self.data_as_bytes[j] != 10:
                bytebuffer.append(self.data_as_bytes[j])
                #print(self.data_as_bytes[j])
                j = j + 1
            #print(len(bytebuffer))
            #print(float(bytebuffer))
            floatbuffer.append(float(bytebuffer))
            print("Länge floatbuffer:")
            print(len(floatbuffer))
            print(floatbuffer)
            j = j + 1
            bytebuffer.clear()
        return floatbuffer
    '''

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

        for i in range(12):
            coordinates[i] = round(self.robot_position_x + distances[i][0]), round(self.robot_position_y + distances[i][1])

        for i in range(12):
            coordinate_x = int(coordinates[i][0])
            coordinate_y = int(coordinates[i][1])
            self.map[coordinate_x][coordinate_y] = True

        self.wall_coordinates = coordinates

        #print("Koordinaten:", coordinates)
        #print("Länge Koordinaten:", len(coordinates))


Roommap = Map()





