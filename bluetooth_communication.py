'''
@author: Robin Justinger
'''
import logging
import struct
import kivy
import asyncio
import bleak
import platform
import sys
from kivy.app import App
from kivy.uix.label import Label
from bleak import BleakScanner, BleakClient
from mapping import Map
from mapping import Roommap
from bleak.exc import BleakError


class BluetoothConnection:
    def __init__(self):
        self.MAKEBLOCKDevice = None
        self.send_request = False
        self.direction = None
        self.read_characteristic = f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb"
        self.write_characteristic = f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb"
        self.connection_status = False
        self.is_connection_wanted = False

    async def manage_bluetooth_connection(self):
        if not self.is_connection_wanted:
            return
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == 'MAKEBLOCK':
                self.MAKEBLOCKDevice = d
                print("MAKEBLOCK wurde gefunden!")
                async with BleakClient(self.MAKEBLOCKDevice) as client:
                    if self.is_connection_wanted:
                        client.connect(timeout=10.0)
                        if client.is_connected:
                            self.connection_status = True
                            print("Bluetooth-Verbindung erfolgreich hergestellt!")
                            await client.start_notify(self.read_characteristic, notification_handler)
                            while True:
                                if not client.is_connected:
                                    self.connection_status = False
                                    break
                                if self.send_request:
                                    await client.write_gatt_char(self.write_characteristic, self.direction)
                                    self.send_request = False
                                if not self.is_connection_wanted:
                                    await client.stop_notify(self.read_characteristic)
                                    await client.disconnect()
                                    self.connection_status = False
                                    break
                                await asyncio.sleep(2.0)
                                update_all()
                        else:
                            self.connection_status = False
                            print("Bluetooth-Verbindung kann nicht hergestellt werden!")
                    else:
                        await client.stop_notify(self.read_characteristic)
                        await client.disconnect()


# Simple notification handler which prints the data received
def notification_handler(sender, data):
    received_data = data
    for element in received_data:
        Roommap.data_as_bytes.append(element)


def eventloop(function):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(function)


def update_all():
    # 402 is the length of data_as_bytes, when all values are received
    if Roommap.data_as_bytes.endswith(b'\n\n'):
        Roommap.save_data_now()
        #Roommap.calculate_robot_position()
        #Roommap.coordinates = Roommap.update_walls()
        #Roommap.map_to_draw = True
    #print(len(Roommap.data_as_bytes))