from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController

class BedSweeper:
    def __init__(self, sensor: ColourSensorController, pilot: DrivingController):
        self.sensor = sensor
        self.pilot = pilot

    def sweep_bed(self):
        pass