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

base_power = 60
phase_time_s = 0.2

prev_distance = 0
smoothed_closing_rate = 0
dt = phase_time_s
alpha = 0.5

try:
    left_motor.set_dps(base_power)
    right_motor.set_dps(base_power)
    while True:
        distance_mm = ultrasonic_sensor.get_distance_mm()
        raw_closing_rate = (distance_mm - prev_distance) / dt
        smoothed_closing_rate = alpha * raw_closing_rate + (1 - alpha) * smoothed_closing_rate
        print(f"Smoothed closing rate: {smoothed_closing_rate:.2f}")
        prev_distance = distance_mm
        time.sleep(phase_time_s)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
finally:
    driving_controller.stop_motors()
    reset_brick()
