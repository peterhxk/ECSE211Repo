import sys
from time import sleep
import time
import csv
"""
This code performs full rotations rotations of the encoder and makes sure the maximum / minimum angle values are consistent over the rotations. It then computes the standard deviation on the minimum and maximum
"""

sys.path.append("../../")
from utils.brick import  Motor, reset_brick,wait_ready_sensors,TouchSensor
from utils import sound
from test_encoder import reset_encoder,is_stalling,motor_command,motor_stop
from utils import brick

motor = Motor("B")
button = TouchSensor("2")

direction = +1
print("Resetting the encoder")
reset_encoder()
# ASSUMING THE MOTOR IS NOW AT 0 DEG. POSITION
motor.reset_encoder()
print("set angle to 0 degrees")

running_angle = []
try :
    while True:
        # do a full rotation then record running angles
        direction *= -1 # reverse the direction
        motor.set_power(direction)
        t0 = time.time()
        timeout = 2
        while not is_stalling():
            if time.time() - t0 > timeout:
                raise RuntimeError("The BrickPi timed out to detect stalling. Please check connections")
            sleep(0.02)
        motor_stop()
        running_angle.append(motor.get_position())
        print("Appended angle")
        sleep(1)
except KeyboardInterrupt:
    print("Gently excitting test")

with open("angles.csv","w") as f:
    writer = csv.writer(f)
    writer.writerow(["angle"])
    for angle in running_angle:
        writer.writerow([angle])

print("wrote angles")
brick.reset_brickpi()



