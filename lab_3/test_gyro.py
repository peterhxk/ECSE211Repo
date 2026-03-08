#!/usr/bin/env python3
import time
from utils.brick import EV3GyroSensor, TouchSensor, Motor
import csv

# ---- Settings ----
LOADING_THRESHOLD = 60  # gentle push threshold
SLEEP_TIME = 0.02
MOTOR_SPEED = 45
# ------------------

# Sensors
gyro = EV3GyroSensor(port=1, mode=EV3GyroSensor.Mode.DPS)
button = TouchSensor(port=2)

# Motor
motor = Motor("B")

print("Initializing sensors...")
gyro.wait_ready()
button.wait_ready()

gyro.reset_measure()
print("Reset degrees to zero.")
print("Motor running until button is pressed...")

try:
    # ---- Run motor until button pressed ----
    motor.set_power(MOTOR_SPEED)
    speeds = []
    while True:
        speeds.append(motor.get_speed())
        if button.is_pressed():
            print("Button pressed, stopping motor.")
            break

    with open("motor_speeds.csv","w") as f:
        writer = csv.writer(f)
        writer.writerow(["speed"])
        for speed in speeds:
            writer.writerow([speed])

except KeyboardInterrupt:
    print("Stopped (KeyboardInterrupt).")
finally:
    # safest stop for most EV3 motor APIs
    try:
        motor.set_power(0)
    except Exception:
        motor.set_dps(0)