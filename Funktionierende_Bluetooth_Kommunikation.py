
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

#class MyApp(App):
#    def build(self):
#        return Label(text='Hello World!')

#MyApp().run()

def notification_handler(sender, data):
    """Simple notification handler which prints the data received."""
    output_numbers = list(data)
    print(output_numbers)

async def bluetooth():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name =='MAKEBLOCK':
            MAKEBLOCKDevice = d
            print(MAKEBLOCKDevice.metadata)

    async with BleakClient(MAKEBLOCKDevice) as client:
        #svcs = await client.get_services()
        #print("Services:")
        #for service in svcs:
        #    print(service)
        #    print(service.uuid)
        await client.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler)
        await asyncio.sleep(10.0)
        await client.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb")

    return client

Test = asyncio.run(bluetooth())
print(Test)
asyncio.run(Test.connect())
#MakeblockDevice = bleak.backends.client.BaseBleakClient(Test)
if Test.is_connected:
    print("Client verbunden")
else:
    print("Client nicht verbunden!")

Servic = asyncio.run(Test.get_services())
for service in Servic:
    print(service)

#CHARACTERISTIC_UUID =
#asyncio.run(Test.start_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb", notification_handler))
#ArduinoDaten = asyncio.run(Test.read_gatt_char(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb"))
#asyncio.sleep(10.0)
#print(ArduinoDaten)
#asyncio.run(Test.stop_notify(f"0000{0xFFF4:x}-0000-1000-8000-00805f9b34fb"))

