# imports
from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures
print("done importing...")

left_motor = Motor("A")
right_motor = Motor("D")

MOTOR_POWER = 10

def move_straight(x):
    """x: distance in cm"""
    print(f"Move straight by {x}")


def turn_angle(x):
    """x: angle in degrees?"""
    pass

def turn_right():
    pass

def turn_left():
    pass

if __name__ == "__main__":
    move_straight(5)
