# Class to test EV3UltrasonicSensor readings in a controlled manner.
# Saves filtered readings and expected distance computed from wheel encoder data to a CSV.
# This is to check whether we can get reliable data from the sensor when a filter is applied.
import time
import math
import statistics
import csv
from collections import deque

from utils.brick import (
    wait_ready_sensors,
    reset_brick,
    EV3UltrasonicSensor,
    Motor
)

US_PORT = 1

LEFT_MOTOR_PORT = "C"
RIGHT_MOTOR_PORT = "B"

COUNTDOWN_SECONDS = 3

MEDIAN_WINDOW_SIZE = 5
EMA_ALPHA = 0.4
SAMPLE_DELAY = 0.02  # 20ms between samples

# modify according to measurements.
WHEEL_DIAMETER_CM = 4.26
WHEEL_CIRCUMFERENCE_CM = math.pi * WHEEL_DIAMETER_CM

INITIAL_DISTANCE_CM = 60.0
FORWARD_TOWARD_WALL = True

CSV_FILENAME = "ultrasonic_motion_data.csv"

# ============================================================
# Motion cycle config
# One full cycle:
#   forward for FORWARD_TIME_S
#   wait for WAIT_AFTER_FORWARD_S
#   backward for BACKWARD_TIME_S
#   wait for WAIT_AFTER_BACKWARD_S
#
# This repeats until you stop the program with Ctrl+C.
# ============================================================
FORWARD_TIME_S = 5.0
WAIT_AFTER_FORWARD_S = 2.0
BACKWARD_TIME_S = 5.0
WAIT_AFTER_BACKWARD_S = 2.0

FORWARD_SPEED_DPS = 180.0
BACKWARD_SPEED_DPS = -180.0

CYCLE_DURATION_S = (
    FORWARD_TIME_S
    + WAIT_AFTER_FORWARD_S
    + BACKWARD_TIME_S
    + WAIT_AFTER_BACKWARD_S
)


def velocity_profile_deg_per_s(t: float) -> float:
    cycle_t = t % CYCLE_DURATION_S

    if 0.0 <= cycle_t < FORWARD_TIME_S:
        return FORWARD_SPEED_DPS

    cycle_t -= FORWARD_TIME_S
    if 0.0 <= cycle_t < WAIT_AFTER_FORWARD_S:
        return 0.0

    cycle_t -= WAIT_AFTER_FORWARD_S
    if 0.0 <= cycle_t < BACKWARD_TIME_S:
        return BACKWARD_SPEED_DPS

    cycle_t -= BACKWARD_TIME_S
    if 0.0 <= cycle_t < WAIT_AFTER_BACKWARD_S:
        return 0.0

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
    motor.set_dps(0)


def stop_both(left_motor, right_motor):
    stop_motor(left_motor)
    stop_motor(right_motor)


def init_csv(filename: str):
    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "time_s",
            "cycle_time_s",
            "commanded_speed_dps",
            "raw_cm",
            "median_cm",
            "ema_cm",
            "left_encoder_deg",
            "right_encoder_deg",
            "avg_encoder_deg",
            "expected_distance_cm"
        ])


def append_csv_row(
    filename: str,
    elapsed: float,
    cycle_t: float,
    commanded_speed: float,
    raw: float,
    median_val: float,
    ema_val: float,
    left_deg: float,
    right_deg: float,
    avg_deg: float,
    expected_distance: float
):
    with open(filename, mode="a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            elapsed,
            cycle_t,
            commanded_speed,
            raw,
            median_val,
            ema_val,
            left_deg,
            right_deg,
            avg_deg,
            expected_distance
        ])


def record_with_control(sensor, left_motor, right_motor, csv_filename: str):
    start = time.time()

    raw_values = []
    median_values = []
    ema_values = []
    control_values = []

    median_window = deque(maxlen=MEDIAN_WINDOW_SIZE)
    ema_estimate = None

    reset_motor_encoder(left_motor)
    reset_motor_encoder(right_motor)

    init_csv(csv_filename)

    while True:
        now = time.time()
        elapsed = now - start
        cycle_t = elapsed % CYCLE_DURATION_S

        commanded_speed = velocity_profile_deg_per_s(elapsed)
        set_motor_speed_deg_per_s(left_motor, commanded_speed)
        set_motor_speed_deg_per_s(right_motor, commanded_speed)

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

        left_deg = get_motor_encoder_deg(left_motor)
        right_deg = get_motor_encoder_deg(right_motor)
        avg_deg = (left_deg + right_deg) / 2.0

        expected_distance = compute_expected_distance(INITIAL_DISTANCE_CM, avg_deg)

        raw_values.append(raw)
        median_values.append(denoised)
        ema_values.append(ema_estimate)
        control_values.append(expected_distance)

        append_csv_row(
            csv_filename,
            elapsed,
            cycle_t,
            commanded_speed,
            raw,
            denoised,
            ema_estimate,
            left_deg,
            right_deg,
            avg_deg,
            expected_distance
        )

        time.sleep(SAMPLE_DELAY)


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


def main():
    sensor = EV3UltrasonicSensor(US_PORT)
    left_motor, right_motor = make_motors()

    wait_ready_sensors(True)

    raw_values = []
    median_values = []
    ema_values = []
    control_values = []

    try:
        print("Ultrasonic controlled-motion test")
        print(f"CSV output: {CSV_FILENAME}")
        print(f"Initial distance: {INITIAL_DISTANCE_CM} cm")
        print(f"Wheel diameter: {WHEEL_DIAMETER_CM} cm")
        print()
        print("Motion cycle:")
        print(f"- Forward  for {FORWARD_TIME_S}s at {FORWARD_SPEED_DPS} dps")
        print(f"- Wait     for {WAIT_AFTER_FORWARD_S}s")
        print(f"- Backward for {BACKWARD_TIME_S}s at {BACKWARD_SPEED_DPS} dps")
        print(f"- Wait     for {WAIT_AFTER_BACKWARD_S}s")
        print("- Repeats until Ctrl+C")
        print()

        countdown(COUNTDOWN_SECONDS)

        raw_values, median_values, ema_values, control_values = record_with_control(
            sensor,
            left_motor,
            right_motor,
            CSV_FILENAME
        )

    except KeyboardInterrupt:
        print("\nCtrl+C pressed. Stopping and finalizing.")

    finally:
        try:
            stop_both(left_motor, right_motor)
        except Exception:
            pass

        if raw_values:
            print_stats(raw_values, median_values, ema_values, control_values)
        else:
            print(f"Data saved to {CSV_FILENAME}")

        reset_brick()


if __name__ == "__main__":
    main()