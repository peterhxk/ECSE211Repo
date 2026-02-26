# TO-DO: change from using sleep() to using a timer (we want robot to still collect sensor data while it moves)
# imports
from time import sleep, time
from utils.brick import wait_ready_sensors, reset_brick, Motor, TouchSensor, EV3UltrasonicSensor
from concurrent import futures
print("done importing...")

left_motor = Motor("B")
right_motor = Motor("C")

MOTOR_POWER = 10

def move_straight(x):
    """x: distance in cm"""
    print(f"Move straight by {x}")
    left_motor.set_power(MOTOR_POWER)
    right_motor.set_power(MOTOR_POWER)
    sleep(x)
    left_motor.set_power(0)
    right_motor.set_power(0)

def move_straight(x, power):
    """x: distance in cm
    power: 0-100 value of maximum motor power"""
    print(f"Move straight by {x} with {power}% power")
    left_motor.set_power(power)
    right_motor.set_power(power)
    sleep(x)
    left_motor.set_power(0)
    right_motor.set_power(0)

def turn_angle(x):
    """x: angle in degrees?"""
    left_motor.set_power(MOTOR_POWER)
    right_motor.set_power(-MOTOR_POWER)
    sleep(x)
    left_motor.set_power(0)
    right_motor.set_power(0)

def turn_right():
    pass

def turn_left():
    pass

if __name__ == "__main__":
    while True:
        dir = input("T (turn) or S (straight)?")
        x = float(input("x?: "))
        power = (input("power: "))
        if dir == "T":
            turn_angle(x)
        elif dir == "S":
            move_straight(x, power)
        else:
            print("no work")
