import sys
#sys.path.append("../")

from utils.brick import wait_ready_sensors, reset_brick, Motor, EV3UltrasonicSensor
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
phase_time_s = 0.2


try:
    while True:
        left_motor.set_dps(base_power)
        right_motor.set_dps(-base_power)
        print(f"US distance: {ultrasonic_sensor.get_cm()} cm")
        time.sleep(phase_time_s)
        left_motor.set_dps(-base_power)
        right_motor.set_dps(base_power)
        print(f"US distance: {ultrasonic_sensor.get_cm()} cm")
        time.sleep(phase_time_s)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
finally:
    driving_controller.stop_motors()
    reset_brick()
