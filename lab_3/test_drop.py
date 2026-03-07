#!/usr/bin/env python3
import time
import statistics
from collections import deque
from utils.brick import Motor

# ------------------ Settings ------------------
MOTOR_PORT = "B"

POWER = 45
SLEEP_TIME = 0.02

EJECT_TIME = 2.0          # seconds: initial outward purge
DRY_RUN_TIME = 5.0        # seconds per direction for baseline collection

K = 3.0                   # threshold multiplier
CONSECUTIVE_SAMPLES = 5   # event fires after this many consecutive matches

USE_MOVING_AVERAGE = True
MOVING_AVG_WINDOW = 5     # set to 1 to disable smoothing
# ----------------------------------------------


def compute_stats(samples):
    if len(samples) < 2:
        raise RuntimeError("Not enough samples to compute mu and sigma.")
    mu = statistics.mean(samples)
    sigma = statistics.stdev(samples)
    return mu, sigma


def sample_motor_speeds(motor, power, duration, sleep_time, use_abs=False):
    """
    Run motor at constant power for `duration` seconds and collect speed samples.
    If use_abs=True, stores abs(speed); otherwise stores raw signed speed.
    """
    samples = []
    motor.set_power(power)
    start = time.time()

    try:
        while time.time() - start < duration:
            speed = motor.get_speed()
            samples.append(abs(speed) if use_abs else speed)
            time.sleep(sleep_time)
    finally:
        motor.set_power(0)

    return samples


def print_stats(label, samples):
    mu, sigma = compute_stats(samples)
    print(f"{label}:")
    print(f"  n     = {len(samples)}")
    print(f"  mu    = {mu:.3f}")
    print(f"  sigma = {sigma:.3f}")
    return mu, sigma


def moving_average_filter(window, new_value):
    window.append(new_value)
    return sum(window) / len(window)


def main():
    motor = Motor(MOTOR_PORT)

    print("=== Conveyor calibration + event detection ===")
    print(f"Motor port: {MOTOR_PORT}")
    print(f"Power: {POWER}")
    print(f"Sleep time: {SLEEP_TIME}s")
    print(f"k: {K}")
    print(f"Consecutive samples required: {CONSECUTIVE_SAMPLES}")
    print()

    try:
        # 1) Eject any blocks inside: outward = negative
        print("Step 1/3: Ejecting any blocks outward...")
        motor.set_power(-POWER)
        time.sleep(EJECT_TIME)
        motor.set_power(0)
        time.sleep(0.5)

        # 2) Dry run both ways at power=45
        print("Step 2/3: Dry run outward for baseline...")
        outward_samples = sample_motor_speeds(
            motor=motor,
            power=-POWER,
            duration=DRY_RUN_TIME,
            sleep_time=SLEEP_TIME,
            use_abs=False,
        )
        mu_out, sigma_out = print_stats("Outward baseline", outward_samples)
        print()

        time.sleep(0.5)

        print("Step 2/3: Dry run inward for baseline...")
        inward_samples = sample_motor_speeds(
            motor=motor,
            power=POWER,
            duration=DRY_RUN_TIME,
            sleep_time=SLEEP_TIME,
            use_abs=False,
        )
        mu_in, sigma_in = print_stats("Inward baseline", inward_samples)
        print()

        if sigma_in == 0:
            print("Warning: sigma_in is 0. Setting a tiny fallback value.")
            sigma_in = 1e-6

        threshold = mu_in - K * sigma_in
        print("Detection settings (used during inward run):")
        print(f"  inward threshold = mu_in - k*sigma_in = {threshold:.3f}")
        print()

        # 3) Run inward and notify on stdout when event condition is detected
        print("Step 3/3: Running inward and monitoring for load events...")
        print("Press Ctrl+C to stop.")
        print()

        motor.set_power(POWER)

        consecutive = 0
        in_event_state = False
        ma_window = deque(maxlen=max(1, MOVING_AVG_WINDOW))

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
                in_event_state = False

            if consecutive >= CONSECUTIVE_SAMPLES and not in_event_state:
                print(
                    f"EVENT detected | "
                    f"raw_speed={raw_speed:.3f} | "
                    f"filtered_speed={speed_for_detection:.3f} | "
                    f"threshold={threshold:.3f} | "
                    f"consecutive={consecutive}"
                )
                in_event_state = True

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