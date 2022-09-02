from math import sin, cos, radians


class RawData:

    def __init__(self):
        self.map = [[False]*1000 for i in range(1000)]
        self.data_now = []
        self.data_old = []
        self.data_all = []
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
        """
        Saves the new incoming data as integers
        """
        self.data_old = self.data_now
        self.data_all.append(self.data_old)
        self.data_now = self.convert_byte_to_int()
        self.data_as_bytes.clear()

    def convert_byte_to_int(self):
        """
        Converts the bytes incoming from bluetooth to integers
        """
        bytebuffer = bytearray()
        intbuffer = []
        j = 0  # Simple counter to go through the bytearray
        for i in range(77):
            # != 10 because 10 is newline in ASCII-Code and the values are separated by a newline
            while self.data_as_bytes[j] != 10:
                bytebuffer.append(self.data_as_bytes[j])
                j = j + 1
            intbuffer.append(int(bytebuffer))
            j = j + 1
            bytebuffer.clear()
        return intbuffer


Roommap = RawData()