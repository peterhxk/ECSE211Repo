#!/usr/bin/env python3
import time
from utils.brick import EV3GyroSensor, TouchSensor, Motor

# ---- Settings ----
THRESHOLD = 170
SLEEP_TIME = 0.02
MOTOR_SPEED = 200
#
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

# ---- Run motor until button pressed ----
motor.set_dps(MOTOR_SPEED)

while True:
    pressed = button.get_value()

    if pressed:
        print("Button pressed! Starting back-and-forth motion.")
        break

    #time.sleep(0.02)

# ---- Back and forth motion ----
direction = -1

try:
    while True:

        # Reverse direction each cycle
        motor.set_dps(direction * MOTOR_SPEED)
        direction *= -1

        # Run for 1 second in that direction
        start = time.time()

        while time.time() - start < 5:

            dps = gyro.get_dps_measure()

            if dps is None:
                print("Error reading")

            elif abs(dps) > THRESHOLD:
                print(f"DPS: {dps}")

            time.sleep(SLEEP_TIME)

except KeyboardInterrupt:
    motor.set_dps(0)
    print("Stopped")