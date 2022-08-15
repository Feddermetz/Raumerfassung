'''
@author: Robin Justinger
'''
from kivy.config import Config
# has to be at the beginning of the main file to work for fixed window size
Config.set('graphics', 'width', 1280)
Config.set('graphics', 'height', 720)
Config.set('graphics', 'resizable', False)
import logging
import kivy
import asyncio
import bleak
import platform
import sys
import bluetooth_communication
from bluetooth_communication import eventloop
from bluetooth_communication import BluetoothConnection
from mapping import Roommap
from kivy.app import App
from kivy.core.window import Window
from kivy.graphics import Rectangle, Color
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.clock import Clock
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError
from kivy.properties import ObjectProperty
from threading import Thread

Makeblock_connection = BluetoothConnection()
slam_mode = None

def start_coroutine(routine):
    loop = asyncio.new_event_loop()
    Thread(target=loop.run_forever, daemon=True).start()
    loop.call_soon_threadsafe(asyncio.create_task, routine)


class Mapping(Widget):

    def turn_left_45(self):
        print("Drehe 45 Grad links!")
        Makeblock_connection.direction = b'1'
        Makeblock_connection.send_request = True

    def drive_forward(self):
        print("Fahre vor!")
        Makeblock_connection.direction = b'2'
        Makeblock_connection.send_request = True

    def turn_right_45(self):
        print("Drehe 45 Grad rechts!")
        Makeblock_connection.direction = b'3'
        Makeblock_connection.send_request = True

    def turn_left_90(self):
        print("Drehe 90 Grad links!")
        Makeblock_connection.direction = b'4'
        Makeblock_connection.send_request = True

    def turn_right_90(self):
        print("Drehe 90 Grad rechts!")
        Makeblock_connection.direction = b'6'
        Makeblock_connection.send_request = True

    def turn_left_135(self):
        print("Drehe 90 Grad links!")
        Makeblock_connection.direction = b'7'
        Makeblock_connection.send_request = True

    def drive_backward(self):
        print("Fahre zurück!")
        Makeblock_connection.direction = b'8'
        Makeblock_connection.send_request = True

    def turn_right_135(self):
        print("Drehe 135 Grad rechts!")
        Makeblock_connection.direction = b'9'
        Makeblock_connection.send_request = True

    def manage_bluetooth_connection(self, is_connection_wanted):
        Makeblock_connection.is_connection_wanted = is_connection_wanted
        start_coroutine(Makeblock_connection.manage_bluetooth_connection())
        Clock.schedule_interval(self.draw_data, 5.0)

    def show_connection_status(self, dt):
        if Makeblock_connection.connection_status:
            self.ids.bluetooth_connection_status.source = 'images/bluetooth_connected.png'
            self.ids.connect_bluetooth.disabled = True
            self.ids.disconnect_bluetooth.disabled = False
        else:
            self.ids.bluetooth_connection_status.source = 'images/bluetooth_disconnected.png'
            self.ids.connect_bluetooth.disabled = False
            self.ids.disconnect_bluetooth.disabled = True

    def set_slam_mode(self, mode):
        """
        Sets the SLAM mode to use for the calculations.

        :param mode: name of the SLAM mode to be used
        """
        global slam_mode
        if mode == 'ekf':
            slam_mode = 'ekf'
        elif mode == 'graph':
            slam_mode = 'graph'
        else:
            slam_mode = None

    def draw_data(self, dt):
        #print("Bin in draw!")
        coordinates = Roommap.get_wall_coordinates()
        with self.canvas:
            #print("Bin in canvas")
            Color(1,1,1)
            #print("Länge der Koordinaten:", len(coordinates))
            if len(coordinates) > 0:
                #print("coordinates-Variable:", coordinates[0][1])
                #print("Länge", len(coordinates))
                for i in range(12):
            #    print("Eingegebene Koordinaten: ", coordinates[i][0], coordinates[i][1])
                    Rectangle(pos=(coordinates[i][0], coordinates[i][1]), size=(2,2))
            #for i in range(len(Map.get_coordinates_wall())):
            #    print("Bin in draw_data!")
            #    Color(0.5,0.5,0.5,0.5)
            #    self.rect = Rectangle(pos=(Map.coordinates_wall[i][0], Map.coordinates_wall[i][1]), size=(2,2))

    def create_csv_file(self):
        """
        Creates a .csv file named "ScanDaten.csv" in the project directory,
        where all sensor data send so far by the robot is saved.
        """
        print("erstelle Datei")
        f = open("ScanDaten.csv", "w")
        for line in Roommap.data_all:
            if len(line) < 77:
                line_as_str = ""
            else:
                line_as_str = str(line)
                line_as_str = line_as_str.replace(",", ";")
                line_as_str = line_as_str.replace("[", "")
                line_as_str = line_as_str.replace("]", "")
                print(line_as_str, file=f)
        print("\n", file=f)
        f.close()
        return 0

    def import_csv_file(self):
        """
        Imports a .csv file named "ScanDaten.csv" from the project directory. The data
        is then saved to Roommap.data_all in the same form as the data that is received through
        the bluetooth connection.
        """
        f = open("Testaufnahme_eigenes_Zimmer.csv")
        for line in f:
            row_as_int = []
            row = line.split(sep=";")
            for data in row:
                row_as_int.append(int(data))
            Roommap.data_all.append(row_as_int)
        f.close()
        print("Daten momentan: ", Roommap.data_all)


class MappingApp(App):
    def build(self):
        return Mapping()

    def on_start(self):
        Clock.schedule_interval(self.root.show_connection_status, 2.0)


if __name__ == '__main__':
    MappingApp().run()
