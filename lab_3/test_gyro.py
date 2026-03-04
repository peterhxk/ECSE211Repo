#!/usr/bin/env python3
import time
from utils.brick import EV3GyroSensor, TouchSensor, Motor

# ---- Settings ----
LOADING_THRESHOLD = 25  # gentle push threshold
SLEEP_TIME = 0.02
MOTOR_SPEED = 20
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

    while not button.get_value():
        time.sleep(SLEEP_TIME)

    print("Button pressed! Starting loading of first block")
    first_block_positioned = False

    while not first_block_positioned:
            motor.set_power(MOTOR_SPEED)
            dps = gyro.get_dps_measure()
            if dps is None:
                print("Error reading gyro (None).")
            elif dps > LOADING_THRESHOLD:
                print(
                    "First block successfully got in position! Stopping motor.\n"
                    "Press the button to load the second one."
                )
                motor.set_power(0)
                first_block_positioned = True
                break

            time.sleep(SLEEP_TIME)

    # ---- Wait for button press to load second block ----
    while not button.get_value():
        time.sleep(SLEEP_TIME)

    print("Button pressed! Loading the second block.")
    motor.set_power(MOTOR_SPEED)

    # ---- Keep running until Ctrl+C (or replace with your own stop condition) ----
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped (KeyboardInterrupt).")
finally:
    # safest stop for most EV3 motor APIs
    try:
        motor.set_power(0)
    except Exception:
        motor.set_dps(0)