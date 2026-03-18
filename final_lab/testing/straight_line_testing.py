from utils.brick import wait_ready_sensors, reset_brick, Motor, EV3UltrasonicSensor
from controllers.driving_controller import DrivingController
import time
import numpy as np

left_motor = Motor("B")
right_motor = Motor("C")
ultrasonic_sensor = EV3UltrasonicSensor(1)

driving_controller = DrivingController(left_motor, right_motor, 20)
print("waiting for sensors to be ready...")
wait_ready_sensors(True)
print("sensors ready!")

base_power = 60
phase_time_s = 0.5

prev_distance = ultrasonic_sensor.get_cm() * 10
smoothed_closing_rate = 0
dt = phase_time_s
alpha = 0.1

def get_stable_us_reading(sensor, samples=10):
    readings = []
    for _ in range(samples):
        reading = sensor.get_cm() * 10
        readings.append(reading)
        time.sleep(0.05)

    readings = np.array(readings)
    median = np.median(readings)
    mad = np.median(np.abs(readings - median))

    if mad == 0:
        return np.mean(readings)
    
    mask = np.abs(readings - median) <= 1.5 * mad
    return np.mean(readings[mask])


try:
    while True:
        driving_controller.stop_motors()
        distance_mm = get_stable_us_reading(ultrasonic_sensor)
        raw_closing_rate = (distance_mm - prev_distance) / dt
        smoothed_closing_rate = alpha * raw_closing_rate + (1 - alpha) * smoothed_closing_rate
        print(f"Current distance: {distance_mm:.2f} mm, Smoothed closing rate: {smoothed_closing_rate:.2f} mm/s")
        prev_distance = distance_mm

        if distance_mm < 100: #10 cm
            break
        
        left_motor.set_dps(base_power)
        right_motor.set_dps(base_power)

        time.sleep(phase_time_s)
except KeyboardInterrupt:
    print("Interrupted via keyboard...")
finally:
    driving_controller.stop_motors()
    reset_brick()
