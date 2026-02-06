import sys
from time import sleep, time
import csv

sys.path.append("../")
from utils.brick import Motor, reset_brick

motor = Motor("B")

POWER = 10              # adjust
STALL_WINDOW = 0.35     # seconds to observe
STALL_DELTA = 3         # encoder ticks considered "no movement"
PAUSE_AT_END = 0.2      # pause before reversing

angles = []

def motor_command(direction: int):
    """
    direction: +1 or -1
    Replace this with your library's way of running the motor continuously.
    Examples in other libs: motor.set_power(direction*POWER)
    """
    motor.set_power(direction * POWER)   # <-- CHANGE if your API differs

def motor_stop():
    """
    Replace with your library's stop/brake call.
    """
    motor.set_power(0)                   # <-- CHANGE if your API differs

def is_stalling(window_s=STALL_WINDOW, delta_ticks=STALL_DELTA):
    p0 = motor.get_position()
    t0 = time()
    while time() - t0 < window_s:
        sleep(0.02)
    p1 = motor.get_position()
    return abs(p1 - p0) <= delta_ticks

direction = +1

while True:
    try:
        motor_command(direction)

        # Wait until stall is detected
        while True:
            if is_stalling():
                motor_stop()
                sleep(PAUSE_AT_END)
                direction *= -1
                angles.append(motor.get_position())
                break

            sleep(0.01)

    except KeyboardInterrupt:
        print("Interrupting")
        with open("angles.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["angle"])
            for a in angles:
                writer.writerow([a])
        print("Saved angles:", angles)
        reset_brick()
        break


    # motor.get_position() returns the encoder value of the motor, which is set to 0 when initializing the motor.
    # use Motor.reset_encoder() to reset the position
    #offset_encoder also helps manually offsetting the position
