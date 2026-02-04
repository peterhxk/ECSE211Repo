import sys

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("B")
KILL_SWITCH = TouchSensor(3)
DRUM_TOGGLE = TouchSensor(1)

wait_ready_sensors(True)

drumming_active = False
toggle_pressed = False   # boolean to check if the toggle sensor is pressed

try:
    while not KILL_SWITCH.is_pressed():
        if not toggle_pressed and DRUM_TOGGLE.is_pressed():
            # toggle the drumming mechanism.
            drumming_active = not drumming_active
            toggle_pressed = True
        elif toggle_pressed and not DRUM_TOGGLE.is_pressed():
            # reset the boolean flag.
            toggle_pressed = False
        
        if (drumming_active):
            motor.set_power(30)
        else:
            motor.set_power(0)
    print("Kill switch activated!")
except BaseException as error:  # capture all exceptions including KeyboardInterrupt (Ctrl-C)
    print("There was an error: ", error)
finally:
    motor.set_power(0)
