"""
@author: Robin Justinger
"""
from DataManager import DataManager


class RawData:

    def __init__(self):
        self.data_now = []
        self.data_old = []
        self.data_all = []
        self.data_as_bytes = bytearray()

    def save_data_now(self):
        """
        Saves the new incoming data as integers and clears the
        bytebuffer "data_as_bytes", so that it is ready to save
        data again.
        """
        self.data_old = self.data_now
        self.data_all.append(self.data_old)
        if self.data_all[0] == [] and len(self.data_all) > 1:
            self.data_all.pop(0)
        self.data_now = self.convert_byte_to_int()
        self.data_as_bytes.clear()

    def convert_byte_to_int(self):
        """
        Converts the bytes that came in from bluetooth to integers and
        puts the integers into a list.
        """
        byte_buffer = bytearray()
        int_buffer = []
        j = 0  # Simple counter to go through the bytearray
        for i in range(77):
            # != 10 because 10 is newline in ASCII-Code and the values are separated by a newline
            while self.data_as_bytes[j] != 10:
                byte_buffer.append(self.data_as_bytes[j])
                j = j + 1
            int_buffer.append(int(byte_buffer))
            j = j + 1
            byte_buffer.clear()
        return int_buffer


Roommap = RawData()
dmRobot1 = DataManager()