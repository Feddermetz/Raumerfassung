'''
@author: Robin Justinger
'''
from kivy.config import Config
# has to be at the beginning of the main file to work for fixed window size
Config.set('graphics', 'width', 1280)
Config.set('graphics', 'height', 720)
Config.set('graphics', 'resizable', False)
import asyncio
from bluetooth_communication import BluetoothConnection
from raw_data import Roommap
from kivy.app import App
from kivy.lang import Builder
from kivy.garden.matplotlib.backend_kivyagg import FigureCanvasKivyAgg
from kivy.uix.widget import Widget
from kivy.clock import Clock
from threading import Thread
from EKF_SLAM import run_ekf_slam
from no_correction_SLAM import run_no_correction_slam
from matplotlib import pyplot as plt

Makeblock_connection = BluetoothConnection()


def start_coroutine(routine):
    """
    Starts a new thread to deal with the bluetooth connection
    """
    loop = asyncio.new_event_loop()
    Thread(target=loop.run_forever, daemon=True).start()
    loop.call_soon_threadsafe(asyncio.create_task, routine)


class Mapping(Widget):

    def __init__(self, **kwargs):
        self.is_csv_imported = False
        self.data_all_old = []
        self.slam_mode = 'None'
        self.slam_mode_old = 'None'
        super().__init__(**kwargs)
        map = self.ids.map
        map.add_widget(FigureCanvasKivyAgg(plt.gcf()))

    def turn_left_45(self):
        Makeblock_connection.direction = b'1'
        Makeblock_connection.send_request = True

    def drive_forward(self):
        Makeblock_connection.direction = b'2'
        Makeblock_connection.send_request = True

    def turn_right_45(self):
        Makeblock_connection.direction = b'3'
        Makeblock_connection.send_request = True

    def turn_left_90(self):
        Makeblock_connection.direction = b'4'
        Makeblock_connection.send_request = True

    def turn_right_90(self):
        Makeblock_connection.direction = b'6'
        Makeblock_connection.send_request = True

    def turn_left_135(self):
        Makeblock_connection.direction = b'7'
        Makeblock_connection.send_request = True

    def drive_backward(self):
        Makeblock_connection.direction = b'8'
        Makeblock_connection.send_request = True

    def turn_right_135(self):
        Makeblock_connection.direction = b'9'
        Makeblock_connection.send_request = True

    def manage_bluetooth_connection(self, is_connection_wanted):
        """
        Sets the wanted connection status and starts a coroutine in a different thread

        param is_connection_wanted: bool to set if connection is wanted or not
        """
        Makeblock_connection.is_connection_wanted = is_connection_wanted
        start_coroutine(Makeblock_connection.manage_bluetooth_connection())

    def show_connection_status(self, dt):
        """
        Checks status of the bluetooth connection and refreshes the picture to show
        connection status

        param dt: needed for recurring execution of the function
        """
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
        Sets the SLAM mode to use for the calculations. Disables the buttons for setting the slam mode.

        :param mode: name of the SLAM mode to be used
        """
        if mode == 'ekf':
            self.slam_mode = 'ekf'
            self.ids.EKF_mode.disabled = True
            self.ids.graph_mode.disabled = False
            self.ids.no_correction_mode.disabled = False
        elif mode == 'graph':
            self.slam_mode = 'graph'
            self.ids.EKF_mode.disabled = False
            self.ids.graph_mode.disabled = True
            self.ids.no_correction_mode.disabled = False
        elif mode == 'noCorrection':
            self.slam_mode = 'noCorrection'
            self.ids.EKF_mode.disabled = False
            self.ids.graph_mode.disabled = False
            self.ids.no_correction_mode.disabled = True
        else:
            self.slam_mode = 'None'
            self.ids.EKF_mode.disabled = False
            self.ids.graph_mode.disabled = False
            self.ids.no_correction_mode.disabled = False

    def draw_data(self, dt):
        """
        Checks if any data has changed since the last execution. If so, runs calculations
        in the currently selected SLAM-mode and creates the plot to show the results.

        param dt: needed for recurring execution of the function
        """
        if Roommap.data_all == self.data_all_old and self.slam_mode == self.slam_mode_old:
            return
        if self.slam_mode == 'ekf':
            plot = run_ekf_slam()
            self.ids.map.add_widget(FigureCanvasKivyAgg(plot.gcf()))
        elif self.slam_mode == 'graph':
            # TODO: call graph slam method
            pass
        elif self.slam_mode == 'noCorrection':
            plot = run_no_correction_slam()
            self.ids.map.add_widget(FigureCanvasKivyAgg(plot.gcf()))
        else:
            pass
        self.data_all_old = Roommap.data_all
        self.slam_mode_old = self.slam_mode

    def create_csv_file(self):
        """
        Creates a .csv file named "ScanDaten.csv" in the project directory,
        where all sensor data send by the robot so far is saved.
        """
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
        if self.is_csv_imported:
            return
        f = open("ScanDaten.csv")
        for line in f:
            row_as_int = []
            row = line.split(sep=";")
            for data in row:
                row_as_int.append(int(data))
            Roommap.data_all.append(row_as_int)
        f.close()
        self.is_csv_imported = True


class MappingApp(App):
    def build(self):
        Builder.load_file("Mapping.kv")
        return Mapping()

    def on_start(self):
        """
        Calls the functions below every two seconds
        """
        Clock.schedule_interval(self.root.show_connection_status, 2.0)
        Clock.schedule_interval(self.root.draw_data, 2.0)


if __name__ == '__main__':
    MappingApp().run()
