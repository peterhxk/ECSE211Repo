import sys
from time import sleep

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# ---- Setup ----
motor = Motor("A")
GRABBER_SENSOR = TouchSensor(1)  # kept, not used here

wait_ready_sensors(True)

POWER = 20
direction = 1          # 1 = forward, -1 = backward
running = False        # motor initially stopped

print("Commands: switch (toggle direction), start, stop, quit")

try:
    while True:
        cmd = input("> ").strip().lower()

        if GRABBER_SENSOR.is_pressed():
            print("Grabber sensor is pressed!")

        if cmd == "switch":
            direction *= -1
            if running:
                motor.set_power(POWER * direction)
            print("Direction is now:", "forward" if direction == 1 else "backward")

        elif cmd == "start":
            running = True
            motor.set_power(POWER * direction)
            print("Motor running", "forward" if direction == 1 else "backward")

        elif cmd == "stop":
            running = False
            motor.set_power(0)
            print("Motor stopped")

        elif cmd in ("quit", "exit"):
            break

        else:
            print("Unknown command. Use: switch, start, stop, quit")

except KeyboardInterrupt:
    print("\nGently exiting...")

except BaseException as error:
    print("There was an error:", error)

finally:
    motor.set_power(0)