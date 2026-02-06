import sys
from time import sleep

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("B")
button = TouchSensor("2")

while True :
    if button.is_pressed():
        motor.reset_encoder()
    print("Motor position: ", motor.get_position())
    sleep(1)
    # motor.get_position() returns the encoder value of the motor, which is set to 0 when initializing the motor.
    # use Motor.reset_encoder() to reset the position
