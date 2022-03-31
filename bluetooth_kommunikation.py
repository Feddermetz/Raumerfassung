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
    MAKEBLOCKDevice = 0
    for d in devices:
        if d.name == 'MAKEBLOCK':
            MAKEBLOCKDevice = d

    if MAKEBLOCKDevice == 0:
        print("Fehler, kein MAKEBLOCK gefunden!")
    else:
        async with BleakClient(MAKEBLOCKDevice) as client:
            await client.connect()
            if not client.is_connected:
                print("Fehler, die Bluetooth-Verbindung kann nicht hergestellt werden!")

            await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
            # await client.write_gatt_char(f"0000{0xFFF1:x}-0000-1000-8000-00805f9b34fb", bytes(5))
            await asyncio.sleep(10.0)
            await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")


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