import sys
sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time
import data.measurements as consts
from data.ports_brick import Ports

# Sensor declaration

left_motor = Motor(Ports.LEFT_MOTOR)
right_motor = Motor(Ports.RIGHT_MOTOR)
color_sensor_motor = Motor(Ports.COLOR_MOTOR)

driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(Ports.COLOR_SENSOR, color_sensor_motor)

line_follower = SweepLineFollower(color_sensor_controller, driving_controller)

wait_ready_sensors(True)

try:
    line_follower.pilot.move_distance(-50)
    line_follower.pilot.right_motor.wait_is_stopped()
    line_follower.turn_logic("right", "back")
    line_follower.turn_logic("right", "front")
    line_follower.move_forward(200, "o")
    line_follower.move_backward(430)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
