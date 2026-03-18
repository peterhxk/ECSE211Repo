import sys
#sys.path.append("../")

from utils.brick import wait_ready_sensors, Motor, EV3UltrasonicSensor
from controllers.driving_controller import DrivingController
import time

left_motor = Motor("B")
right_motor = Motor("C")
ultrasonic_sensor = EV3UltrasonicSensor(1)

driving_controller = DrivingController(left_motor, right_motor, 20)
print("waiting for sensors to be ready...")
wait_ready_sensors(True)
print("sensors ready!")

base_power = 150


try:
    while True:
        left_motor.set_dps(base_power)
        right_motor.set_dps(-base_power)
        time.sleep(1)
        left_motor.set_dps(-base_power)
        right_motor.set_dps(base_power)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
