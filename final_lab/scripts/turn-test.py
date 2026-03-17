import sys
#sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time

# Sensor declaration

left_motor = Motor("C")
right_motor = Motor("B")

driving_controller = DrivingController(left_motor, right_motor, 150)

try:
    print("trying turn in place")
    driving_controller.turn_in_place(90)
    time.sleep(6)
    print("trying pivot left")
    driving_controller.pivot_90("left")
    time.sleep(6)
    print("trying pivot right")
    driving_controller.pivot_90("right")
    time.sleep(6)

    
except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
