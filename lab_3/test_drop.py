#!/usr/bin/env python3
import time
import statistics
from collections import deque
from utils.brick import Motor

MOTOR_PORT = "A"
POWER = 50
SLEEP_TIME = 0.02
EJECT_TIME = 2.0
DRY_RUN_TIME = 5.0

K = 3.0
CONSECUTIVE_SAMPLES = 5

USE_MOVING_AVERAGE = True
MOVING_AVG_WINDOW = 5

COOLDOWN = 0.8


def compute_stats(samples):
    if len(samples) < 2:
        raise RuntimeError("Not enough samples to compute mu and sigma.")
    return statistics.mean(samples), statistics.stdev(samples)


def sample_motor_speeds(motor, power, duration, sleep_time):
    samples = []
    motor.set_power(power)
    start = time.time()
    try:
        while time.time() - start < duration:
            samples.append(motor.get_speed())
            time.sleep(sleep_time)
    finally:
        motor.set_power(0)
    return samples


def moving_average_filter(window, new_value):
    window.append(new_value)
    return sum(window) / len(window)


def main():
    motor = Motor(MOTOR_PORT)

    try:
        print("Step 1/3: Ejecting outward...")
        motor.set_power(-POWER)
        time.sleep(EJECT_TIME)
        motor.set_power(0)
        time.sleep(0.5)

        print("Step 2/3: Dry run outward...")
        outward_samples = sample_motor_speeds(motor, -POWER, DRY_RUN_TIME, SLEEP_TIME)
        mu_out, sigma_out = compute_stats(outward_samples)
        print(f"Outward: mu={mu_out:.3f}, sigma={sigma_out:.3f}")

        time.sleep(0.5)

        print("Step 2/3: Dry run inward...")
        inward_samples = sample_motor_speeds(motor, POWER, DRY_RUN_TIME, SLEEP_TIME)
        mu_in, sigma_in = compute_stats(inward_samples)
        print(f"Inward: mu={mu_in:.3f}, sigma={sigma_in:.3f}")

        if sigma_in == 0:
            sigma_in = 1e-6

        threshold = mu_in - K * sigma_in
        print(f"Threshold = {threshold:.3f}")

        print("Step 3/3: Running inward and monitoring...")
        motor.set_power(POWER)

        consecutive = 0
        ma_window = deque(maxlen=max(1, MOVING_AVG_WINDOW))

        first_trigger_time = None

        while True:
            raw_speed = motor.get_speed()
            if USE_MOVING_AVERAGE and MOVING_AVG_WINDOW > 1:
                speed_for_detection = moving_average_filter(ma_window, raw_speed)
            else:
                speed_for_detection = raw_speed

            condition = speed_for_detection < threshold

            if condition:
                consecutive += 1
            else:
                consecutive = 0

            confirmed_event = consecutive >= CONSECUTIVE_SAMPLES

            if first_trigger_time is None and confirmed_event:
                first_trigger_time = time.time()
                print(
                    f"FIRST EVENT | raw={raw_speed:.3f} | "
                    f"filtered={speed_for_detection:.3f}"
                )

            elif first_trigger_time is not None:
                if time.time() - first_trigger_time >= COOLDOWN and confirmed_event:
                    print("Condition still present after cooldown, stopping motor.")
                    break

            time.sleep(SLEEP_TIME)

    except KeyboardInterrupt:
        print("\nStopped (KeyboardInterrupt).")
    finally:
        try:
            motor.set_power(0)
        except Exception:
            pass


if __name__ == "__main__":
    main()