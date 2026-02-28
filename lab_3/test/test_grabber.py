import sys
import select
from time import sleep, time

sys.path.append("../")
from utils.brick import Motor, TouchSensor, wait_ready_sensors

# ---- Setup ----
motor = Motor("A")
BUTTON = TouchSensor(1)

wait_ready_sensors(True)

POWER = 40
direction = 1  # 1 = forward, -1 = backward

# ---- Bloc logic ----
bloc_count = 0
delays = [3.0, 0.1]  # after 1st press, keep spinning 3s; after 2nd press, 0.5s

running = True
motor.set_power(POWER * direction)

print("Type 'switch' to change direction. Press button on port 1 to store blocs.")

def stop_then_restart(delay_s: float, next_bloc: int):
    """Keep motor running for delay_s seconds, stop, announce, then restart."""
    sleep(delay_s)
    motor.set_power(0)
    print(f"Bloc {next_bloc} has been stored.")
    sleep(0.2)  # small pause
    motor.set_power(POWER * direction)

try:
    while True:
        # ---- Keyboard (non-blocking) ----
        if select.select([sys.stdin], [], [], 0)[0]:
            cmd = sys.stdin.readline().strip().lower()

            if cmd == "switch":
                direction *= -1
                if running:
                    motor.set_power(POWER * direction)
                print("Direction:", "forward" if direction == 1 else "backward")

            elif cmd == "toggle":
                if running:
                    motor.set_power(0)
                    print("Motor stopped.")
                else:
                    motor.set_power(POWER * direction)
                    print("Motor running.")
                running = not running

        # ---- Button press handling (edge-detect + debounce) ----
        if running and bloc_count < 2 and BUTTON.is_pressed():
            bloc_count += 1
            print(f"Button pressed! Capturing bloc {bloc_count}...")

            # Wait until released (simple debounce)
            while BUTTON.is_pressed():
                sleep(0.001)

            stop_then_restart(delays[bloc_count - 1], bloc_count)

            if bloc_count >= 2:
                print("Two blocs stored. Waiting for more commands...")
                motor.set_power(POWER * -1)
                sleep(0.2)
                motor.set_power(0)

        sleep(0.05)

except KeyboardInterrupt:
    print("\nExiting safely...")

finally:
    motor.set_power(0)