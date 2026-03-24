# Class to test EV3UltrasonicSensor readings in a controller manner.
# Plots the filtered readings against an expected distance computed from wheel encoder data.
# This is to check whether we can get reliable data from the sensor when a filter is applied.
import time
import math
import statistics
from collections import deque

import matplotlib.pyplot as plt

from utils.brick import (
    wait_ready_sensors,
    reset_brick,
    EV3UltrasonicSensor,
    Motor
)

US_PORT = 1

LEFT_MOTOR_PORT = "C"
RIGHT_MOTOR_PORT = "B"

RECORD_SECONDS = 5.0
COUNTDOWN_SECONDS = 3

MEDIAN_WINDOW_SIZE = 5
EMA_ALPHA = 0.4
SAMPLE_DELAY = 0.02 # 20ms between samples

# modify according to measurements.
WHEEL_DIAMETER_CM = 4.26
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM

INITIAL_DISTANCE_CM = 60.0
FORWARD_TOWARD_WALL = True


# Piecewise motion profile:
# 0.0s - 1.0s: stop
# 1.0s - 3.0s: move forward at constant speed
# 3.0s - 4.0s: stop
# 4.0s - 5.0s: move backward slowly

def velocity_profile_deg_per_s(t: float) -> float:
    if 0.0 <= t < 1.0:
        return 0.0
    elif 1.0 <= t < 3.0:
        return 180.0
    elif 3.0 <= t < 4.0:
        return 0.0
    elif 4.0 <= t < 5.0:
        return -90.0
    return 0.0


def countdown(seconds: int) -> None:
    print("Get ready...")
    for i in range(seconds, 0, -1):
        print(f"Recording starts in {i}...")
        time.sleep(1)
    print("Recording started.\n")

def deg_to_cm(deg: float) -> float:
    return (deg / 360.0) * WHEEL_CIRCUMFERENCE_CM

def compute_expected_distance(initial_distance_cm: float, avg_wheel_deg: float) -> float:
    travel_cm = deg_to_cm(avg_wheel_deg)

    if FORWARD_TOWARD_WALL:
        return initial_distance_cm - travel_cm
    else:
        return initial_distance_cm + travel_cm


def make_motors():
    left = Motor(LEFT_MOTOR_PORT)
    right = Motor(RIGHT_MOTOR_PORT)
    return left, right

def reset_motor_encoder(motor):
    motor.reset_encoder()

def get_motor_encoder_deg(motor):
    return motor.get_encoder()

def set_motor_speed_deg_per_s(motor, speed_deg_s):
    motor.set_dps(speed_deg_s)

def stop_motor(motor):
    motor.stop()

def stop_both(left_motor, right_motor):
    stop_motor(left_motor)
    stop_motor(right_motor)

def record_with_control(sensor, left_motor, right_motor, duration_s: float):
    start = time.time()

    times = []
    raw_values = []
    median_values = []
    ema_values = []
    control_values = []

    median_window = deque(maxlen=MEDIAN_WINDOW_SIZE)
    ema_estimate = None

    reset_motor_encoder(left_motor)
    reset_motor_encoder(right_motor)

    while True:
        now = time.time()
        elapsed = now - start
        if elapsed >= duration_s:
            break

        # 1) Command motion profile
        commanded_speed = velocity_profile_deg_per_s(elapsed)
        set_motor_speed_deg_per_s(left_motor, commanded_speed)
        set_motor_speed_deg_per_s(right_motor, commanded_speed)

        # 2) Read ultrasonic
        raw = sensor.get_cm()
        if raw is None:
            time.sleep(SAMPLE_DELAY)
            continue

        median_window.append(raw)
        denoised = statistics.median(median_window)

        if ema_estimate is None:
            ema_estimate = denoised
        else:
            ema_estimate = EMA_ALPHA * denoised + (1 - EMA_ALPHA) * ema_estimate

        # 3) Read wheel encoders and build expected/control line
        left_deg = get_motor_encoder_deg(left_motor)
        right_deg = get_motor_encoder_deg(right_motor)
        avg_deg = (left_deg + right_deg) / 2.0

        expected_distance = compute_expected_distance(INITIAL_DISTANCE_CM, avg_deg)

        # 4) Store
        times.append(elapsed)
        raw_values.append(raw)
        median_values.append(denoised)
        ema_values.append(ema_estimate)
        control_values.append(expected_distance)

        time.sleep(SAMPLE_DELAY)

    stop_both(left_motor, right_motor)

    return times, raw_values, median_values, ema_values, control_values


def print_stats(raw_values, median_values, ema_values, control_values):
    if not raw_values:
        print("No data recorded.")
        return

    print("Recording complete.\n")
    print(f"Samples collected: {len(raw_values)}")
    if len(raw_values) > 1:
        print(f"Raw stdev:   {statistics.stdev(raw_values):.2f} cm")
    else:
        print("Raw stdev:   N/A")

    raw_mean_err = statistics.mean(abs(r - c) for r, c in zip(raw_values, control_values))
    med_mean_err = statistics.mean(abs(m - c) for m, c in zip(median_values, control_values))
    ema_mean_err = statistics.mean(abs(e - c) for e, c in zip(ema_values, control_values))

    print(f"Mean abs error vs control (raw):    {raw_mean_err:.2f} cm")
    print(f"Mean abs error vs control (median): {med_mean_err:.2f} cm")
    print(f"Mean abs error vs control (EMA):    {ema_mean_err:.2f} cm")


def plot_data(times, raw_values, median_values, ema_values, control_values):
    plt.figure(figsize=(11, 6))
    plt.plot(times, control_values, label="Control / expected distance")
    plt.plot(times, raw_values, label="Raw ultrasonic")
    plt.plot(times, median_values, label="Median filtered")
    plt.plot(times, ema_values, label="EMA smoothed")

    plt.xlabel("Time from recording start (s)")
    plt.ylabel("Distance (cm)")
    plt.title("Ultrasonic Sensor vs Controlled Motion")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    sensor = EV3UltrasonicSensor(US_PORT)
    left_motor, right_motor = make_motors()

    wait_ready_sensors(True)

    try:
        print("Ultrasonic controlled-motion test")
        print(f"Duration: {RECORD_SECONDS} s")
        print(f"Initial distance: {INITIAL_DISTANCE_CM} cm")
        print(f"Wheel diameter: {WHEEL_DIAMETER_CM} cm")
        print()
        print("Recommended setup:")
        print("- Face the robot toward a flat wall")
        print("- Measure the initial wall distance carefully")
        print("- Make sure both wheels can roll straight")
        print()

        countdown(COUNTDOWN_SECONDS)

        data = record_with_control(sensor, left_motor, right_motor, RECORD_SECONDS)
        times, raw_values, median_values, ema_values, control_values = data

        print_stats(raw_values, median_values, ema_values, control_values)
        # plot_data(times, raw_values, median_values, ema_values, control_values)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Exiting.")

    finally:
        try:
            stop_both(left_motor, right_motor)
        except Exception:
            pass
        reset_brick()


if __name__ == "__main__":
    main()