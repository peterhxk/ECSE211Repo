import sys
from time import sleep

sys.path.append("../../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("C")
KILL_SWITCH = TouchSensor(3)


wait_ready_sensors(True)
drumming_active = False
toggle_pressed = False  # boolean to check if the toggle sensor is pressed

try:
    for i in range(0, 100):
        motor.set_power(i)
        sleep(0.1)
    while True:
        pass
except KeyboardInterrupt:
    print("Gently exiting test")
    motor.set_power(0)

except BaseException as error:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
    print("There was an error: ", error)
finally:
    motor.set_power(0)
