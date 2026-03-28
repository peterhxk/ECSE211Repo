"""
Standalone PID wall-follow tuning script.

Runs the robot along a wall using a PID controller whose gains are set via CLI
arguments.  Every loop iteration is logged to a CSV file under data/pid_logs/
so runs can be compared with scripts/plot_pid_log.py.

Usage examples (from final_lab/):
    python3 -m scripts.tune_wall_pid --kp 14 --ki 0.5 --kd 8 --distance 40
    python3 -m scripts.tune_wall_pid --kp 10 --kd 0 --direction backward --distance 80
"""

import argparse
import csv
import math
import os
import time
from datetime import datetime

from utils.brick import Motor, EV3UltrasonicSensor, wait_ready_sensors, reset_brick
from data.ports_brick import Ports

WHEEL_DIAMETER_MM = 42.6
OUTLIER_THRESHOLD_CM = 8


def parse_args():
    p = argparse.ArgumentParser(description="PID wall-follow tuning")
    p.add_argument("--kp", type=float, default=12.0)
    p.add_argument("--ki", type=float, default=0.0)
    p.add_argument("--kd", type=float, default=0.0)
    p.add_argument("--target", type=float, default=4.8,
                    help="Target wall distance in cm")
    p.add_argument("--distance", type=float, default=40.0,
                    help="Straight distance to travel in cm")
    p.add_argument("--power", type=int, default=260,
                    help="Base motor DPS")
    p.add_argument("--direction", choices=["forward", "backward"],
                    default="forward")
    p.add_argument("--alpha", type=float, default=0.7,
                    help="Low-pass smoothing alpha for steering")
    p.add_argument("--steer-scale", type=float, default=1.3,
                    help="Multiplier on adjustment applied to DPS")
    return p.parse_args()


def build_csv_path(args):
    log_dir = os.path.join(os.path.dirname(__file__), "..", "data", "pid_logs")
    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = (f"pid_{ts}_{args.direction}"
            f"_kp{args.kp}_ki{args.ki}_kd{args.kd}.csv")
    return os.path.join(log_dir, name)


CSV_HEADER = [
    "t", "dt", "raw_us", "filtered_us", "target", "error",
    "integral", "derivative", "raw_adj", "smooth_adj",
    "left_dps", "right_dps", "left_enc", "right_enc",
]


def run(args):
    left_motor = Motor(Ports.LEFT_MOTOR.value)
    right_motor = Motor(Ports.RIGHT_MOTOR.value)
    us = EV3UltrasonicSensor(Ports.ULTRASONIC_SENSOR.value)
    wait_ready_sensors(True)

    csv_path = build_csv_path(args)
    csv_file = open(csv_path, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(CSV_HEADER)

    sign = 1 if args.direction == "forward" else -1
    target_deg = abs(args.distance * 10 / (WHEEL_DIAMETER_MM * math.pi) * 360)

    left_motor.reset_encoder()
    right_motor.reset_encoder()
    left_motor.float_motor()
    right_motor.float_motor()

    integral = 0.0
    prev_error = 0.0
    prev_time = time.time()
    smoothed_adj = None
    last_us_cm = None
    errors = []

    start_dps = args.power * sign
    left_motor.set_dps(start_dps)
    right_motor.set_dps(start_dps)

    t0 = time.time()

    print(f"--- PID tuning run ---")
    print(f"  Kp={args.kp}  Ki={args.ki}  Kd={args.kd}")
    print(f"  target={args.target}cm  distance={args.distance}cm  power={args.power}")
    print(f"  direction={args.direction}  alpha={args.alpha}  steer_scale={args.steer_scale}")
    print(f"  logging to {csv_path}")
    print()

    try:
        while True:
            travelled_deg = abs(
                (left_motor.get_encoder() + right_motor.get_encoder()) / 2.0
            )
            if travelled_deg >= target_deg:
                break

            now = time.time()
            dt = max(now - prev_time, 0.001)
            prev_time = now

            raw_us = us.get_cm()
            if last_us_cm is None or abs(raw_us - last_us_cm) <= OUTLIER_THRESHOLD_CM:
                last_us_cm = raw_us
            filtered_us = last_us_cm

            error = args.target - filtered_us
            integral = max(-50.0, min(50.0, integral + error * dt))
            derivative = (error - prev_error) / dt
            prev_error = error

            raw_adj = args.kp * error + args.ki * integral + args.kd * derivative

            base_abs = args.power
            max_steer = max(12, min(int(base_abs * 0.42), base_abs - 15))
            raw_adj = max(-max_steer, min(max_steer, raw_adj))

            if smoothed_adj is None:
                smoothed_adj = raw_adj
            else:
                smoothed_adj = args.alpha * raw_adj + (1.0 - args.alpha) * smoothed_adj

            adj = int(round(smoothed_adj))
            s = args.steer_scale

            if args.direction == "forward":
                left_dps = args.power + adj * s
                right_dps = args.power - adj * s
            else:
                left_dps = -args.power - adj * s
                right_dps = -args.power + adj * s

            left_motor.set_dps(left_dps)
            right_motor.set_dps(right_dps)

            errors.append(error)

            writer.writerow([
                f"{now - t0:.4f}", f"{dt:.4f}",
                f"{raw_us:.2f}", f"{filtered_us:.2f}", f"{args.target:.2f}",
                f"{error:.3f}", f"{integral:.3f}", f"{derivative:.3f}",
                f"{raw_adj:.3f}", f"{smoothed_adj:.3f}",
                f"{left_dps:.1f}", f"{right_dps:.1f}",
                f"{left_motor.get_encoder():.0f}",
                f"{right_motor.get_encoder():.0f}",
            ])

            print(f"adj={adj:+4d}  L={left_dps:+7.1f}  R={right_dps:+7.1f}  "
                  f"us={filtered_us:5.1f}  err={error:+6.2f}")

            time.sleep(0.04)

    except KeyboardInterrupt:
        print("\nInterrupted — stopping motors.")
    finally:
        left_motor.set_dps(0)
        right_motor.set_dps(0)
        csv_file.close()

    elapsed = time.time() - t0
    if errors:
        abs_errors = [abs(e) for e in errors]
        mean_ae = sum(abs_errors) / len(abs_errors)
        max_ae = max(abs_errors)
        variance = sum((e - sum(errors) / len(errors)) ** 2 for e in errors) / len(errors)
        std_dev = math.sqrt(variance)
        print(f"\n--- Summary ---")
        print(f"  samples      : {len(errors)}")
        print(f"  elapsed      : {elapsed:.2f}s")
        print(f"  mean |error| : {mean_ae:.3f} cm")
        print(f"  max  |error| : {max_ae:.3f} cm")
        print(f"  std dev      : {std_dev:.3f} cm")
        print(f"  CSV saved to : {csv_path}")

    reset_brick()


if __name__ == "__main__":
    run(parse_args())
