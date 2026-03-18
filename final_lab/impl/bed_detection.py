from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from enum import Enum

class BedColor(Enum):
    RED = "red"
    GREEN = "green"

"""
    Class to handle bed detection. Process is run with the "run" method
    that is called only when the robot has arrived at a room, i.e. at the
    "doorstep". Is meant to detect one of two bed colors: red or green.
"""
class BedDetector:
    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor
        self.pilot = pilot


    def run(self):
        pass