import sys

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("B")
while True :
    print("Motor position: ", motor.get_position())
    # motor.get_encoder() returns the encoder value of the motor, which is set to 0 when initializing the motor.
