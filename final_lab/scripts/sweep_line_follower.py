import sys
sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time

# Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")
color_sensor_motor = Motor("A")

driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(2, color_sensor_motor)

line_follower = SweepLineFollower(color_sensor_controller, driving_controller)

wait_ready_sensors(True)

try:
    line_follower.move_forward(200)

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
