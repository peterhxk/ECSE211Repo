#!/usr/bin/env python3
import time

# Your gyro class lives here:
# utils/brick.py  -> contains EV3GyroSensor
from utils.brick import EV3GyroSensor


# -------------------- Settings to tune --------------------
SAMPLE_HZ = 50.0
DT = 1.0 / SAMPLE_HZ

ANGLE_THRESHOLD_DEG = 8.0    # abs angle beyond this => "pushed"
CENTER_BAND_DEG = 3.0        # abs angle within this => "centered"
MIN_PUSH_TIME_S = 0.06       # must remain beyond threshold this long
MIN_CENTER_TIME_S = 0.12     # must remain inside center band this long

DPS_DEADBAND = 2.0           # ignore tiny dps noise
# ----------------------------------------------------------


def direction_from_angle(angle_deg: float) -> str:
    # Positive vs negative defines direction.
    # If you want "LEFT/RIGHT" instead, rename these labels.
    if angle_deg > 0:
        return "CW"
    if angle_deg < 0:
        return "CCW"
    return "NONE"


def main():
    # Create gyro on Port 1 in BOTH mode -> [abs, dps]
    gyro = EV3GyroSensor(port=1, mode=EV3GyroSensor.Mode.BOTH)

    # IMPORTANT: zero the gyro while the mechanism is at neutral (middle)
    print("Hold the spring mechanism at the neutral middle position...")
    print("Calibrating / zeroing in 1 second.")
    time.sleep(1.0)

    gyro.reset_measure()
    gyro.wait_ready()

    print("\nRunning. Push away from center then let it spring back.")
    print("Ctrl+C to exit.\n")

    state = "CENTERED"     # CENTERED -> PUSHED -> CENTERED
    push_start = None
    center_start = None
    last_dir = "UNKNOWN"

    next_t = time.monotonic()

    try:
        while True:
            now = time.monotonic()

            # pace loop
            if now < next_t:
                time.sleep(next_t - now)
                continue
            next_t += DT

            # Read BOTH: [abs, dps]
            abs_angle, dps = gyro.get_both_measure()

            if abs(dps) < DPS_DEADBAND:
                dps = 0.0

            inst_dir = direction_from_angle(abs_angle)

            if state == "CENTERED":
                # candidate push
                if abs(abs_angle) >= ANGLE_THRESHOLD_DEG:
                    if push_start is None:
                        push_start = now
                        last_dir = inst_dir
                    if (now - push_start) >= MIN_PUSH_TIME_S:
                        state = "PUSHED"
                        center_start = None
                        print(f"[{now:.3f}] PUSH   dir={last_dir:4s} abs={abs_angle:7.2f}° dps={dps:7.2f}")
                else:
                    push_start = None

            else:  # state == "PUSHED"
                # keep direction updated while away from center
                if inst_dir != "NONE":
                    last_dir = inst_dir

                # candidate return
                if abs(abs_angle) <= CENTER_BAND_DEG:
                    if center_start is None:
                        center_start = now
                    if (now - center_start) >= MIN_CENTER_TIME_S:
                        state = "CENTERED"
                        push_start = None
                        print(f"[{now:.3f}] RETURN dir={last_dir:4s} abs={abs_angle:7.2f}° dps={dps:7.2f}")
                else:
                    center_start = None

    except KeyboardInterrupt:
        print("\nExiting.")


if __name__ == "__main__":
    main()