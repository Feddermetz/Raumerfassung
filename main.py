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


def start_coroutine(routine):
    loop = asyncio.new_event_loop()
    Thread(target=loop.run_forever, daemon=True).start()
    loop.call_soon_threadsafe(asyncio.create_task, routine)


class Mapping(Widget):
    # automatikstatusanzeige = ObjectProperty(None)

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

    # automatic driving mode will probably not be used
    '''
    def automatik_einschalten(self):
        print("Automatik wird wieder eingeschaltet!")
        Makeblock_connection.direction = b'a'
        Makeblock_connection.send_request = True
    
    def schrittweise_einschalten(self):
        print("Schrittweiser Betrieb wird wieder eingeschaltet!")
        Makeblock_connection.direction = b's'
        Makeblock_connection.send_request = True
    '''

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


class MappingApp(App):
    def build(self):
        return Mapping()

    def on_start(self):
        Clock.schedule_interval(self.root.show_connection_status, 2.0)


if __name__ == '__main__':
    MappingApp().run()
