import sys
import select
from time import sleep

sys.path.append("../")

from utils.brick import Motor, TouchSensor, wait_ready_sensors

# ---- Setup ----
motor = Motor("A")
BUTTON = TouchSensor(1)

wait_ready_sensors(True)

POWER = 40
direction = 1      # 1 = forward, -1 = backward
running = True

print("Type 'switch' to change direction. Press button on port 1 to stop.")

try:
    motor.set_power(POWER * direction)

    while True:

        # ---- Check keyboard without blocking ----
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == "switch":
                direction *= -1
                if running:
                    motor.set_power(POWER * direction)
                print("Direction:", "forward" if direction == 1 else "backward")
            if cmd == "toggle":
                if running:
                    motor.set_power(0)
                    print("Motor stopped.")
                else:
                    motor.set_power(POWER * direction)
                    print("Motor running.")
                running = not running

        # ---- Check button ----
        if BUTTON.is_pressed():
            print("Button pressed! Stopping motor.")
            #motor.set_power(0)
            #running = False

        sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting safely...")

finally:
    motor.set_power(0)