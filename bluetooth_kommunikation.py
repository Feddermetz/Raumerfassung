import logging
import kivy
import asyncio
import bleak
import platform
import sys
from kivy.app import App
from kivy.uix.label import Label
from bleak import BleakScanner, BleakClient
from mapping import Map
from bleak.exc import BleakError


class Bluetooth_connection:
    def __init__(self):
        self.MAKEBLOCKDevice = None
        self.send_request = False
        self.direction = None
        self.read_characteristic = f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb"
        self.write_characteristic = f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb"
        self.connection_bool = False

    async def bluetooth_verbinden(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == 'MAKEBLOCK':
                self.MAKEBLOCKDevice = d
                print("MAKEBLOCK wurde gefunden!")
                async with BleakClient(self.MAKEBLOCKDevice) as client:
                    print("Direkt vor connect")
                    client.connect(timeout=10.0)
                    if not client.is_connected:
                        self.connection_bool = False
                        print("Fehler, die Bluetooth-Verbindung kann nicht hergestellt werden!")
                    else:
                        self.connection_bool = True
                        print("Bluetooth-Verbindung erfolgreich hergestellt!")

                    await client.start_notify(self.read_characteristic, notification_handler)
                    #try:
                    while True:
                        if self.send_request:
                            await client.write_gatt_char(self.write_characteristic, self.direction)
                            self.send_request = False
                        await asyncio.sleep(5.0)
                        update_all()
                        print(Map.data_now)
                    #except:
                        #print("Die Verbindung ist aus unbekannten Gründen abgebrochen!")
                    await client.stop_notify(self.read_characteristic)


#Simple notification handler which prints the data received
def notification_handler(sender, data):
     received_data = data
     for element in received_data:
         Map.data_as_bytes.append(element)
     print(received_data)

def eventloop(function):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(function)

def update_all():
    Map.save_data_now(self=Map)
    Map.calculate_robot_position(self=Map)
    Map.coordinates_wall = Map.update_walls(self=Map)
    Map.map_to_draw = True