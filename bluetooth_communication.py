"""
@author: Robin Justinger
"""
import asyncio
from bleak import BleakScanner, BleakClient
from raw_data import Roommap


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
        """
        Manages the bluetooth connection to the bluetooth module of the Makeblock robot.
        """
        if not self.is_connection_wanted:
            return
        devices = await BleakScanner.discover()
        for d in devices:
            if d.name == 'MAKEBLOCK':
                self.MAKEBLOCKDevice = d
                async with BleakClient(self.MAKEBLOCKDevice) as client:
                    if self.is_connection_wanted:
                        client.connect(timeout=10.0)
                        if client.is_connected:
                            self.connection_status = True
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
                    else:
                        await client.stop_notify(self.read_characteristic)
                        await client.disconnect()


def notification_handler(sender, data):
    """
    Simple notification handler which saves the data received
    """
    received_data = data
    for element in received_data:
        Roommap.data_as_bytes.append(element)


def eventloop(function):
    """
    Needed for handling the asynchronous bluetooth connection
    """
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(function)


def update_all():
    """
    saves the new data when all data of the most recent scan has been received
    """
    if Roommap.data_as_bytes.endswith(b'\n\n'):
        Roommap.save_data_now()
