import logging
import kivy
import asyncio
import bleak
import platform
import sys
import bluetooth_kommunikation
from bluetooth_kommunikation import eventloop
kivy.require('1.11.1')
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

class Raumerfassung(Widget):
    automatikstatusanzeige = ObjectProperty(None)

    def fahre_vor(self):
        print("Fahre vor!")

    def fahre_zurueck(self):
        print("Fahre zurück!")

    def fahre_links(self):
        print("Fahre links!")

    def fahre_rechts(self):
        print("Fahre rechts!")

    def verbinde_bluetooth(self):
        print("Bluetooth wird verbunden!")
        eventloop(bluetooth_kommunikation.bluetooth_verbinden())

    def automatik_einschalten(self):
        print("Automatik wird wieder eingeschaltet!")


class RaumerfassungApp(App):
    def build(self):
        return Raumerfassung()

RaumerfassungApp().run()
"""
def notification_handler(sender, data):
    #Simple notification handler which prints the data received.
    output_numbers = list(data)
    print(output_numbers)

async def bluetooth():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name =='MAKEBLOCK':
            MAKEBLOCKDevice = d
            print(MAKEBLOCKDevice.metadata)

    async with BleakClient(MAKEBLOCKDevice) as client:
        await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
        await asyncio.sleep(10.0)
        await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")

    return client

Test = asyncio.run(bluetooth())
print(Test)
asyncio.run(Test.connect())
if Test.is_connected:
    print("Client verbunden")
else:
    print("Client nicht verbunden!")

Servic = asyncio.run(Test.get_services())
for service in Servic:
    print(service)
"""
#asyncio.run(bluetooth_communication.bluetooth())