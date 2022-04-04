import logging
import kivy
import asyncio
import bleak
import platform
import sys
kivy.require('1.11.1')
from kivy.app import App
from kivy.uix.label import Label
from bleak import BleakScanner, BleakClient
from bleak.exc import BleakError

MAKEBLOCKDevice = 0

class Bluetooth_connection:
    def __init__(self):
        self.MAKEBLOCKDevice = None
        self.send_request = False
        self.direction = None
        self.read_characteristic = f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb"
        self.write_characteristic = f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb"

    async def bluetooth_verbinden(self):
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == 'MAKEBLOCK':
                MAKEBLOCKDevice = d
                print("MAKEBLOCK wurde gefunden!")
                async with BleakClient(MAKEBLOCKDevice) as client:
                    print("Direkt vor connect")
                    client.connect(timeout=10.0)
                    if not client.is_connected:
                        print("Fehler, die Bluetooth-Verbindung kann nicht hergestellt werden!")
                    else:
                        print("Bluetooth-Verbindung erfolgreich hergestellt!")

                    await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
                    #try:
                    while True:
                        if self.send_request:
                            await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", self.direction)
                            self.send_request = False
                        await asyncio.sleep(1.0)

                    #except:
                        #print("Die Verbindung ist aus unbekannten Gründen abgebrochen!")
                    await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")


def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    output_numbers = list(data)
    print(output_numbers)

def eventloop(function):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(function)


async def bluetooth_verbinden():
    devices = await BleakScanner.discover()
    #MAKEBLOCKDevice = 0
    for d in devices:
        if d.name == 'MAKEBLOCK':
            MAKEBLOCKDevice = d
            print("MAKEBLOCK wurde gefunden!")
            async with BleakClient(MAKEBLOCKDevice) as client:
                print("Direkt vor connect")
                client.connect(timeout=10.0)
                if not client.is_connected:
                    print("Fehler, die Bluetooth-Verbindung kann nicht hergestellt werden!")
                else:
                    print("Bluetooth-Verbindung erfolgreich hergestellt!")

                await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
                try:
                    while True:
                        if send_request:
                            await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", b'Hello')
                        await asyncio.sleep(5.0)

                except:
                    print("Die Verbindung ist aus unbekannten Gründen abgebrochen!")
                await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")
                #await client.start_notify(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", notification_handler)
                #await asyncio.sleep(5.0)
                #await client.stop_notify(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb")

async def richtung_fahren(Richtung):
    async with BleakClient(MAKEBLOCKDevice) as client:
        await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")
        await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", bytes(5))
'''
    if MAKEBLOCKDevice == 0:
        print("Fehler, kein MAKEBLOCK gefunden!")
    else:
        print("MAKEBLOCK wurde gefunden!")
        async with BleakClient(MAKEBLOCKDevice) as client:
            await client.connect()
            if not client.is_connected:
                print("Fehler, die Bluetooth-Verbindung kann nicht hergestellt werden!")
            else:
                print("Bluetooth-Verbindung erfolgreich hergestellt!")

            await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
            # await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", bytes(5))
            await asyncio.sleep(10.0)
            await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")

'''
'''
def anweisung_senden(direction):
    await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", bytes(5))

'''
#scan for available bluetooth-devices and connect to the device called "MAKEBLOCK"
async def bluetooth():
    devices = await BleakScanner.discover()
    MAKEBLOCKDevice = 0
    for d in devices:
        if d.name =='MAKEBLOCK':
            MAKEBLOCKDevice = d
            print("Gefunden!")
    if MAKEBLOCKDevice == 0:
        #TODO: Fehlermeldung ausgeben, dass kein MAKEBLOCK-Gerät erkannt wurde
        print("Fehler: Kein MAKEBLOCK gefunden!")
    else:
        print("MAKEBLOCK-Device gefunden!")
    #return MAKEBLOCKDevice

#MAKEBLOCKDevice = asyncio.run(bluetooth())

        async with BleakClient(MAKEBLOCKDevice) as client:
            client.connect()
            if not client.is_connected:
                #TODO: Fehlermeldung ausgeben, dass keine Verbindung hergestellt wurde
                print("Fehler: Es wurde keine Verbindung hergestellt!")
            #TODO: Meldung rausgeben, dass die Verbindung zum MAKEBLOCK steht
            #await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
            #await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", bytes(5))
            #await asyncio.sleep(10.0)
            #await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")
            #await client.start_notify(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", notification_handler)
            await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", b'~HELLO')
            #await asyncio.sleep(0.5, loop=loop)  # Sleeping just to make sure the response is not missed...
            #await client.stop_notify(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb")