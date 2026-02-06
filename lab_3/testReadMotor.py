import sys
from time import sleep

sys.path.append("../")

from utils.brick import Motor, TouchSensor, reset_brick, wait_ready_sensors
import csv

# Initialise inputs and outputs
motor = Motor("B")
button = TouchSensor("2")

data,i = [],0

while True :
    try:
        if button.is_pressed():
            data.append((i,motor.get_position()))
            motor.reset_encoder()
            i+=1
            sleep(0.8)
    except Exception as e :
        print("Got error",e)
        reset_brick()
    finally:
        with open("results.csv","a+",newline='') as f:
            writer = csv.writer(f)
            writer.writerows(data)


    # motor.get_position() returns the encoder value of the motor, which is set to 0 when initializing the motor.
    # use Motor.reset_encoder() to reset the position
