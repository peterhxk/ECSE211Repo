#!/usr/bin/env python3
import time
from utils.brick import EV3GyroSensor

# ---- Settings ----
THRESHOLD = 20      # degrees per second
SLEEP_TIME = 0.02   # 50 Hz polling
# ------------------

# initialize gyro on port 1 in DPS mode
gyro = EV3GyroSensor(port=1, mode=EV3GyroSensor.Mode.DPS)

print("Monitoring gyro DPS... (Ctrl+C to stop)")

gyro.wait_ready()

gyro.reset_measure()

print("Reset degrees to zero. Starting monitoring...")

try:
    while True:
        dps = gyro.get_dps_measure()
        if abs(dps) > THRESHOLD:
            print(f"DPS: {dps}")

        time.sleep(SLEEP_TIME)

except KeyboardInterrupt:
    print("Stopped")