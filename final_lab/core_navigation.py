# TO-DO: 1. change from using sleep() to using a timer 
# (we want robot to still collect sensor data while it moves)
# 2. Currently move straight by x cm and turn by x deg doesnt work (we need to do math >:( )

# imports
from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures
print("done importing...")



def __init__(self):
    self.left_motor = Motor("B")
    self.right_motor = Motor("C")

    self.MOTOR_POWER = 10 # DEFAULT VALUE

def move_straight(x, power):
    """x: distance in cm
    power: 0-100 value of maximum motor power"""
    print(f"Move straight by {x} with {power}% power")
    left_motor.set_power(power)
    self.right_motor.set_power(power)
    sleep(x) # CHANGE THIS
    self.left_motor.set_power(0)
    self.right_motor.set_power(0)

def move_straight(x):
    """x: distance in cm"""
    move_straight(x, self.MOTOR_POWER)

def turn_angle(x, power):
    """x: angle in degrees?
    power: value -100 to 100"""
    left_motor.set_power(power)
    right_motor.set_power(-power)
    sleep(x) # CHANGE THIS
    left_motor.set_power(0)
    right_motor.set_power(0)

def turn_angle(x):
    turn_angle(x, MOTOR_POWER)

def turn_right(power):
    turn_angle(90, power)

def turn_right():
    turn_angle(90)

def turn_left(power):
    turn_angle(-90,power)

def turn_left():
    turn_angle(-90)

if __name__ == "__main__":
    while True:
        dir = input("T (turn) or S (straight)?")
        x = float(input("x: "))
        power = (input("power: "))
        if dir == "T":
            turn_angle(x)
        elif dir == "S":
            move_straight(x, power)
        else:
            print("no work")
