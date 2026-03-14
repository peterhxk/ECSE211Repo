import sys
#sys.path.append("../")

import traceback
from utils.brick import wait_ready_sensors, reset_brick, Motor
from controllers.color_sensor_controller import ColourSensorController
from controllers.driving_controller import DrivingController
from impl.sweep_line_follower import SweepLineFollower
import time

# Sensor declaration

left_motor = Motor("B")
right_motor = Motor("C")
color_sensor_motor = Motor("A")

driving_controller = DrivingController(left_motor, right_motor, 150)
color_sensor_controller = ColourSensorController(2, color_sensor_motor)

line_follower = SweepLineFollower(color_sensor_controller, driving_controller)

wait_ready_sensors(True)

try:
    base_power = 150
    while True:
        right_motor.set_dps(base_power)
        left_motor.set_dps(base_power)
        time.sleep(10)
    #line_follower.run()

except KeyboardInterrupt:
    print("Interrupted via keyboard...")

except Exception as err:
    print(err)
    traceback.print_exc()
finally:
    reset_brick() # Turn off everything on the brick's hardware, and reset it
    exit()
