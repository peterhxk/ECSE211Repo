import sys

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors

# Initialise inputs and outputs
motor = Motor("B")
while True :
    print("Motor position: ", motor.get_encoder())