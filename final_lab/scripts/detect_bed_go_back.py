from controllers.driving_controller import DrivingController
from controllers.color_sensor_controller import ColourSensorController
from impl.bed_sweeping import BedSweeper
from impl.room_traverser import RoomTraverser
from utils.brick import wait_ready_sensors, reset_brick, Motor
from utils.brick import EV3ColorSensor
import time

left_motor = Motor("C")
right_motor = Motor("B")
motor = Motor("A")
def move_back():
    left_motor.set_dps(-100)
    right_motor.set_dps(-100)
    time.sleep(1.5)
    left_motor.set_dps(0)
    right_motor.set_dps(0)

driving_controller = DrivingController(left_motor, right_motor, 100)
color_sensor_controller = ColourSensorController(1, motor)

wait_ready_sensors(True)

driving_controller.move_forward()
color_detected = ""
while color_detected != "red":
    color_detected = color_sensor_controller.read_colour()
    print(f"Detected color: {color_detected}")

driving_controller.stop_motors()
time.sleep(2)
move_back()