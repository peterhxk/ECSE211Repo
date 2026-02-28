import sys
from time import sleep

sys.path.append("../../../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("A")
GRABBER_SENSOR = TouchSensor(3)


wait_ready_sensors(True)
motor_state = False

try:
    while True:
        if input("Enter 'switch') to toggle the motor: ") == "switch":
            motor_state = not motor_state
            if motor_state:
                motor.set_power(20)
            else:
                motor.set_power(-20)
except KeyboardInterrupt:
    print("Gently exiting test")
    motor.set_power(0)

except BaseException as error:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
    print("There was an error: ", error)
finally:
    motor.set_power(0)
