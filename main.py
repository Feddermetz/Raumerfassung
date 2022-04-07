import logging
import kivy
import asyncio
import bleak
import platform
import sys
import time
import bluetooth_kommunikation
from bluetooth_kommunikation import eventloop
from bluetooth_kommunikation import Bluetooth_connection
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.uix.anchorlayout import AnchorLayout
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.floatlayout import FloatLayout
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError
from kivy.properties import ObjectProperty
from threading import Thread

Makeblock_connection = Bluetooth_connection()

def start_coroutine(routine):
    Loop = asyncio.new_event_loop()
    Thread(target=Loop.run_forever, daemon=True).start()
    Loop.call_soon_threadsafe(asyncio.create_task, routine)

class Raumerfassung(Widget):
    automatikstatusanzeige = ObjectProperty(None)

    def fahre_vor(self):
        print("Fahre vor!")
        Makeblock_connection.direction = b'f'
        Makeblock_connection.send_request = True

    def fahre_zurueck(self):
        print("Fahre zurück!")
        Makeblock_connection.direction = b'b'
        Makeblock_connection.send_request = True

    def fahre_links(self):
        print("Fahre links!")
        Makeblock_connection.direction = b'l'
        Makeblock_connection.send_request = True

    def fahre_rechts(self):
        print("Fahre rechts!")
        Makeblock_connection.direction = b'r'
        Makeblock_connection.send_request = True

    def verbinde_bluetooth(self):
        start_coroutine(Makeblock_connection.bluetooth_verbinden())
        time.sleep(1)

    def automatik_einschalten(self):
        print("Automatik wird wieder eingeschaltet!")
        Makeblock_connection.direction = b'a'
        Makeblock_connection.send_request = True


class RaumerfassungApp(App):
    def build(self):
        return Raumerfassung()


RaumerfassungApp().run()